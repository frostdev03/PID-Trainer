#include <Wire.h> // Untuk komunikasi I2C (AS5600 dan OLED/TFT)
#include <Adafruit_GFX.h> // Library grafis dasar untuk TFT
#include <Adafruit_ST7735.h> // Driver spesifik untuk TFT ST7735

// --- TFT Setup ---
#define TFT_CS    5
#define TFT_RST   4
#define TFT_DC    15
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// --- AS5600 (I2C Address) ---
#define AS5600_ADDR 0x36

// --- Rotary Encoder ---
#define CLK 32
#define DT  25
#define SW  13

// --- BTS7960 Motor Driver ---
#define RPWM 2  // Pin PWM untuk arah maju (Right PWM)
#define LPWM 0  // Pin PWM untuk arah mundur (Left PWM)
#define REN 27  // Enable untuk arah maju
#define LEN 14  // Enable untuk arah mundur

// --- Variabel Global AS5600 (dari kode pertama, disesuaikan) ---
int magnetStatus = 0;             // Status register magnet (MD, ML, MH)
int lowbyte_as5600;               // Raw angle 7:0
word highbyte_as5600;             // Raw angle 7:0 and 11:8
int rawAngle_as5600;              // Final raw angle (0-4095)
float degAngle_as5600;            // Raw angle in degrees (0-360)

int quadrantNumber, previousquadrantNumber; // ID kuadran untuk pelacakan putaran
float numberOfTurns = 0;          // Jumlah putaran penuh
float correctedAngle = 0;         // Sudut terkoreksi (setelah taring)
float startAngle_as5600 = 0;      // Sudut awal saat kalibrasi
float totalAngle = 0;             // Total posisi sudut absolut (termasuk putaran)
float previousTotalAngle = 0;     // Untuk pembaruan tampilan
unsigned long magneticEncoderTime = 0; // Tidak digunakan saat ini, tapi bisa untuk timing pembacaan

// --- Variabel Global Kontrol (Target, Aktual, Rotary Encoder, PID) ---
float targetAngle = 0;        // Target posisi sudut (dari rotary encoder)
float currentAngle = 0;       // Posisi motor aktual (dari AS5600)

int lastCLK = HIGH;           // Status pin CLK rotary encoder sebelumnya
bool lastSW = HIGH;           // Status pin SW rotary encoder sebelumnya
unsigned long lastButtonPress = 0; // Untuk debounce tombol rotary
int rotaryStep = 5;           // Step sudut tiap klik rotary

// --- Variabel PID ---
float Kp = 0.6;   // Proportional gain (sesuaikan)
float Ki = 0.002; // Integral gain (sesuaikan)
float Kd = 0.25;  // Derivative gain (sesuaikan)

float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;
float outputPID = 0;

unsigned long lastPIDTime = 0; // Untuk timing PID
int pidInterval = 10;          // Interval perhitungan PID (ms)

// --- Fungsi Prototype ---
void checkMagnetPresence();
void ReadRawAngle();
void correctAngle();
void checkQuadrant();
float readAS5600AngleCombined(); // Fungsi baru untuk menggabungkan logika AS5600
void calculatePID();
void driveMotor(int speed);
void displayValues();

void setup() {
  Serial.begin(115200);
  // Inisialisasi Wire dengan pin I2C yang benar untuk ESP32 (SDA, SCL)
  // Pin SDA: 21, Pin SCL: 22 dari skematikmu
  Wire.begin(16, 17);
  Wire.setClock(100000); // Kecepatan I2C standar (bisa 100KHz atau 400KHz)

  // --- Inisialisasi Rotary Encoder ---
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  pinMode(SW, INPUT_PULLUP);
  lastCLK = digitalRead(CLK);

  // --- Inisialisasi Motor Driver ---
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);
  digitalWrite(REN, HIGH); // Aktifkan driver
  digitalWrite(LEN, HIGH); // Aktifkan driver
  analogWrite(RPWM, 0);    // Pastikan motor tidak bergerak saat start
  analogWrite(LPWM, 0);

  // --- Inisialisasi TFT ---
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1); // Sesuaikan rotasi jika diperlukan
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1); // Ukuran teks diperbesar
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 20);
  tft.println("PID Trainer");
  tft.setCursor(10, 40);
  tft.println("Initializing...");
  delay(2000);

  // --- Kalibrasi AS5600 ---
  checkMagnetPresence(); // Memblokir sampai magnet ditemukan
  readAS5600AngleCombined(); // Membaca sudut awal
  startAngle_as5600 = degAngle_as5600; // Setel sudut awal sebagai referensi taring
  totalAngle = 0; // Reset totalAngle setelah kalibrasi

  // Set target awal ke posisi motor saat ini (setelah kalibrasi)
  targetAngle = readAS5600AngleCombined();
  currentAngle = targetAngle; // Inisialisasi currentAngle

  displayValues(); // Tampilkan nilai awal
  lastPIDTime = millis(); // Mulai timer PID
}

void loop() {
  // --- Baca Rotary Encoder untuk Mengubah Target Angle ---
  int currentCLK = digitalRead(CLK);
  if (currentCLK != lastCLK && currentCLK == LOW) {
    // Logika arah putaran yang lebih andal
    int direction = (digitalRead(DT) != currentCLK) ? 1 : -1;
    targetAngle += direction * rotaryStep;
    // Optional: batasi targetAngle jika tidak ingin putaran tak terbatas
    // if (targetAngle >= 360) targetAngle -= 360;
    // if (targetAngle < 0) targetAngle += 360;
    if (targetAngle >= 360) {
    targetAngle -= 360; 
} else if (targetAngle < 0) {
    targetAngle += 360;
}
  }
  lastCLK = currentCLK;

  // --- Baca Tombol Rotary Encoder (untuk reset target) ---
  bool currentSW = digitalRead(SW);
  if (lastSW == HIGH && currentSW == LOW && (millis() - lastButtonPress) > 250) {
    lastButtonPress = millis();
    // Tekan tombol untuk menyamakan target dengan posisi motor saat ini
    targetAngle = currentAngle;
    // Reset PID integral saat target diatur ulang
    integral = 0; 
  }
  lastSW = currentSW;

  // --- Baca Posisi Motor Saat Ini dari AS5600 ---
  currentAngle = readAS5600AngleCombined();

  // --- Hitung dan Terapkan Kontrol PID ---
  if (millis() - lastPIDTime >= pidInterval) {
    calculatePID();
    // Batasi outputPID agar tidak melebihi rentang analogWrite (0-255)
    int motorSpeed = constrain(outputPID, -255, 255);
    driveMotor(motorSpeed);
    lastPIDTime = millis();
  }

  // --- Perbarui Tampilan TFT ---
  // Perbarui tampilan hanya jika ada perubahan signifikan untuk mencegah flicker berlebih
  // Serta perbarui setiap beberapa detik untuk memastikan PID ditampilkan
  if (abs(totalAngle - previousTotalAngle) > 0.1 || abs(targetAngle - previousTotalAngle) > 0.1 || (millis() % 2000 < pidInterval) ) { // Update every 2 seconds roughly
    displayValues();
    previousTotalAngle = totalAngle; // Perbarui previousTotalAngle setelah display
  }

// Di akhir loop() atau setelah calculatePID()
Serial.print(targetAngle);
Serial.print(" ");
Serial.print(currentAngle);
Serial.print(" ");
Serial.println(outputPID); // Atau error untuk melihat respon
  // Delay minimal untuk stabilitas, sesuaikan jika perlu
  delay(50);
}

// --- Fungsi-fungsi AS5600 dari Kode Pertama ---

// Memastikan magnet AS5600 terdeteksi dengan benar
void checkMagnetPresence() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(10, 20);
  tft.setTextColor(ST77XX_RED);
  tft.println("Checking Magnet...");

  while ((magnetStatus & 32) != 32) { // Selama magnet belum terdeteksi (MD = 1)
    magnetStatus = 0; // Reset pembacaan

    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x0B); // Register Status: MD ML MH
    Wire.endTransmission();
    Wire.requestFrom(AS5600_ADDR, 1);

    while (Wire.available() == 0);
    magnetStatus = Wire.read();

    tft.setCursor(10, 50);
    tft.fillRect(10, 50, 100, 20, ST77XX_BLACK); // Hapus teks lama
    tft.setTextColor(ST77XX_WHITE);
    if ((magnetStatus & 8) == 8) { // MH bit
      tft.println("Magnet too strong!");
    } else if ((magnetStatus & 16) == 16) { // ML bit
      tft.println("Magnet too weak!");
    } else {
      tft.println("Adjusting magnet...");
    }
    delay(200);
  }
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(10, 20);
  tft.setTextColor(ST77XX_GREEN);
  tft.println("Magnet Found!");
  delay(1000);
}

// Membaca sudut mentah dari sensor AS5600 (0-4095)
void ReadRawAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0D); // Raw angle (7:0)
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR, 1);
  while (Wire.available() == 0);
  lowbyte_as5600 = Wire.read();

  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C); // Raw angle (11:8)
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR, 1);
  while (Wire.available() == 0);
  highbyte_as5600 = Wire.read();

  highbyte_as5600 = highbyte_as5600 << 8;
  rawAngle_as5600 = highbyte_as5600 | lowbyte_as5600;
  degAngle_as5600 = rawAngle_as5600 * 0.087890625; // Konversi ke derajat
}

// Mengoreksi sudut berdasarkan sudut awal (taring)
void correctAngle() {
  correctedAngle = degAngle_as5600 - startAngle_as5600;
  if (correctedAngle < 0) {
    correctedAngle += 360;
  }
}

// Melacak putaran penuh motor
void checkQuadrant() {
  if (correctedAngle >= 0 && correctedAngle <= 90) {
    quadrantNumber = 1;
  } else if (correctedAngle > 90 && correctedAngle <= 180) {
    quadrantNumber = 2;
  } else if (correctedAngle > 180 && correctedAngle <= 270) {
    quadrantNumber = 3;
  } else if (correctedAngle > 270 && correctedAngle < 360) {
    quadrantNumber = 4;
  }

  if (quadrantNumber != previousquadrantNumber) {
    if (quadrantNumber == 1 && previousquadrantNumber == 4) {
      numberOfTurns++; // CW rotation (4 -> 1)
    } else if (quadrantNumber == 4 && previousquadrantNumber == 1) {
      numberOfTurns--; // CCW rotation (1 -> 4)
    }
    previousquadrantNumber = quadrantNumber;
  }
  totalAngle = (numberOfTurns * 360) + correctedAngle;
}

// Fungsi pembacaan AS5600 gabungan untuk loop()
float readAS5600AngleCombined() {
  ReadRawAngle();
  correctAngle();
  checkQuadrant();
  return correctedAngle; // Mengembalikan sudut absolut
}

// --- Fungsi Kontrol Motor ---

// Menggerakkan motor berdasarkan nilai speed (-255 hingga 255)
void driveMotor(int speed) {
  // Deadband untuk mencegah motor bergetar saat mendekati target
  if (abs(speed) < 10) { // Sesuaikan nilai deadband ini
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
    return;
  }

  if (speed > 0) { // Maju (CW)
    analogWrite(RPWM, abs(speed));
    analogWrite(LPWM, 0);
  } else { // Mundur (CCW)
    analogWrite(RPWM, 0);
    analogWrite(LPWM, abs(speed));
  }
}

// --- Fungsi PID ---
void calculatePID() {
  error = targetAngle - currentAngle; // Hitung error

  integral += error; // Akumulasi integral
  // Batasi integral untuk mencegah integral wind-up
  if (integral > 200) integral = 200; // Sesuaikan batas
  if (integral < -200) integral = -200; // Sesuaikan batas

  derivative = error - previousError; // Hitung derivatif

  outputPID = (Kp * error) + (Ki * integral) + (Kd * derivative); // Hitung output PID

  previousError = error; // Simpan error saat ini untuk iterasi berikutnya
}

void displayValues() {
  tft.fillScreen(ST77XX_BLACK); // Hapus layar

  // Atur ukuran teks default untuk semua menjadi size 1
  tft.setTextSize(1); 
  
  // Tampilkan Target
  tft.setCursor(10, 10);
  tft.setTextColor(ST77XX_YELLOW);
  tft.print("Target: ");
  tft.print(targetAngle, 1);
  tft.print(" deg");

  // Tampilkan Motor
  tft.setCursor(10, 30); // Posisikan di bawah target dengan jarak yang sesuai
  tft.setTextColor(ST77XX_CYAN);
  tft.print("Motor : ");
  tft.print(currentAngle, 1);
  tft.print(" deg");

  // Tampilkan PID Parameter
  tft.setCursor(10, 60); // Posisikan di bawah informasi motor
  tft.setTextColor(ST77XX_MAGENTA);
  tft.print("Kp: ");
  tft.print(Kp, 3); // 3 desimal untuk presisi

  tft.setCursor(10, 75); // Baris berikutnya
  tft.setTextColor(ST77XX_GREEN);
  tft.print("Ki: ");
  tft.print(Ki, 4); // 4 desimal untuk presisi

  tft.setCursor(10, 90); // Baris berikutnya
  tft.setTextColor(ST77XX_ORANGE);
  tft.print("Kd: ");
  tft.print(Kd, 3); // 3 desimal untuk presisi

  // Tampilkan output PID
  tft.setCursor(10, 110);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("PID Out: ");
  tft.print(outputPID, 0);
}