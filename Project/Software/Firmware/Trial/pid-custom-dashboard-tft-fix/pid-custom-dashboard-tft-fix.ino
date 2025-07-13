#include <Wire.h> // Untuk komunikasi I2C (AS5600 dan OLED/TFT)
#include <Adafruit_GFX.h> // Library grafis dasar untuk TFT
#include <Adafruit_ST7735.h> // Driver spesifik untuk TFT ST7735

// --- Library Baru untuk Wi-Fi dan WebServer (Async) ---
#include <WiFi.h> // Untuk fungsionalitas Wi-Fi (AP mode)
#include <ESPAsyncWebServer.h> // Untuk membuat server HTTP Asynchronous (PENTING!)
#include <DNSServer.h> // Untuk Captive Portal (mengarahkan DNS)
#include <SPIFFS.h> // Untuk sistem file SPIFFS (menyimpan halaman web)

// --- TFT Setup ---
#define TFT_CS    5
#define TFT_RST   4
#define TFT_DC    15
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// --- AS5600 (I2C Address) ---
#define AS5600_ADDR 0x36

// --- Definisi pin I2C BARU untuk AS5600 ---
#define AS5600_SDA_PIN 16 // Contoh: Pin SDA baru untuk AS5600
#define AS5600_SCL_PIN 17 // Contoh: Pin SCL baru untuk AS5600

// --- Rotary Encoder ---
#define CLK 32
#define DT  25
#define SW  13 // Rotary Encoder Button

// --- BTS7960 Motor Driver ---
#define RPWM 2  // Pin PWM untuk arah maju (Right PWM)
#define LPWM 0  // Pin PWM untuk arah mundur (Left PWM)
#define REN 27  // Enable untuk arah maju
#define LEN 14  // Enable untuk arah mundur

// --- Variabel Global AS5600 ---
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

// --- Variabel Global Kontrol (Target, Aktual, Rotary Encoder, PID) ---
float targetAngle = 0;        // Target posisi sudut (dari rotary encoder)
float currentAngle = 0;       // Posisi motor aktual (dari AS5600)

int lastCLK = HIGH;           // Status pin CLK rotary encoder sebelumnya
bool lastSW = HIGH;           // Status pin SW rotary encoder sebelumnya
unsigned long lastButtonPress = 0; // Untuk debounce klik tombol rotary
unsigned long buttonPressStartTime = 0; // Waktu tombol ditekan (bukan hanya hold)
const unsigned long HOLD_DURATION = 3000; // Durasi hold untuk kalibrasi (3 detik)
bool isCalibrating = false; // Flag untuk status kalibrasi
bool buttonWasHeld = false; // Flag untuk melacak apakah tombol sudah dianggap ditahan

float rotaryStep = 0.01;      // Step default untuk derajat target

// --- Variabel PID ---
float Kp = 0.6;   // Proportional gain (sesuaikan)
float Ki = 0.0001; // Integral gain (sesuaikan)
float Kd = 0.25;  // Derivative gain (sesuaikan)

float error = 0;
float previousError = 0;
float integral = 0;
float outputPID = 0;

unsigned long lastPIDTime = 0; // Untuk timing PID
int pidInterval = 10;          // Interval perhitungan PID (ms)

// --- Definisi Mode Rotary Encoder ---
enum RotaryMode {
  MODE_ANGLE,
  MODE_KP,
  MODE_KI,
  MODE_KD
};
RotaryMode currentMode = MODE_ANGLE; // Mode awal adalah mengatur derajat

// --- Variabel Wi-Fi AP dan WebServer ---
const char *ssid = "ESP32_PID_AP"; // Nama Wi-Fi AP
const char *password = "password123"; // Password Wi-Fi AP (min 8 karakter)
IPAddress apIP(192, 168, 4, 1); // Alamat IP untuk AP
IPAddress netMsk(255, 255, 255, 0); // Subnet mask

DNSServer dnsServer;
AsyncWebServer server(80); // WebServer di port 80 - Menggunakan AsyncWebServer!

// --- Fungsi Prototype ---
void checkMagnetPresence();
void ReadRawAngle();
void correctAngle();
void checkQuadrant();
float readAS5600AngleCombined();
void calculatePID();
void driveMotor(int speed);
void displayValues();
void handleRotaryEncoder();
void handleRotaryButton(); // Diubah untuk handle hold
void performCalibration(); // Fungsi baru untuk kalibrasi

// --- Fungsi baru untuk WebServer ---
void handleRoot(AsyncWebServerRequest *request);
void handleNotFound(AsyncWebServerRequest *request);
void setupCaptivePortal(); // Setup Captive Portal

// Fungsi bantuan untuk mendapatkan lebar teks dengan ukuran font tertentu
int getTextWidth(String text, int textSize) {
  return text.length() * 6 * textSize; 
}

void setup() {
  Serial.begin(115200);
  Wire.begin(AS5600_SDA_PIN, AS5600_SCL_PIN); 
  Wire.setClock(100000);

  pinMode(CLK, INPUT_PULLUP);
  pinMode(DT, INPUT_PULLUP);
  pinMode(SW, INPUT_PULLUP);
  lastSW = digitalRead(SW); // Inisialisasi lastSW
  
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);

  // --- Inisialisasi TFT (Pesan Loading Single Line) ---
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(0); // ROTASI HORIZONTAL: 0
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, tft.height() / 2 - 7); // Posisi tengah vertikal
  tft.println("PID Trainer Initializing...");

  // --- Inisialisasi SPIFFS ---
  if (!SPIFFS.begin(true)) { 
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(10, tft.height() / 2 - 7);
    tft.setTextColor(ST77XX_RED);
    tft.println("SPIFFS Mount FAILED!");
    Serial.println("SPIFFS Mount Failed");
    while (true); // Stop if SPIFFS fails
  }
  Serial.println("SPIFFS Mounted Successfully");
  
  // --- Setup Captive Portal (AP Mode) ---
  setupCaptivePortal(); 

  // --- Kalibrasi AS5600 ---
  checkMagnetPresence(); // Fungsi ini akan menampilkan pesan error magnet jika ada

  readAS5600AngleCombined();
  startAngle_as5600 = degAngle_as5600;
  totalAngle = 0;

  targetAngle = readAS5600AngleCombined();
  currentAngle = targetAngle;

  lastPIDTime = millis();
}

void loop() {
  // Hanya jalankan logika utama jika tidak sedang kalibrasi
  if (!isCalibrating) {
    dnsServer.processNextRequest(); 
    
    handleRotaryEncoder();
    handleRotaryButton(); // Akan memicu kalibrasi jika ditahan

    currentAngle = readAS5600AngleCombined();

    if (millis() - lastPIDTime >= pidInterval) {
      calculatePID();
      int motorSpeed = constrain(outputPID, -255, 255);
      driveMotor(motorSpeed);
      lastPIDTime = millis();
    }

    if (abs(currentAngle - previousTotalAngle) > 0.1 || (millis() % 200 < pidInterval) || (millis() % 2000 < pidInterval) ) {
      displayValues();
      previousTotalAngle = currentAngle;
    }

    // Debugging Serial Plotter
    Serial.print(targetAngle);
    Serial.print(" ");
    Serial.print(currentAngle);
    Serial.print(" ");
    Serial.println(outputPID); 
  } else {
    // Jika sedang kalibrasi, terus update progress bar
    performCalibration(); 
  }
}

// --- Fungsi untuk menangani putaran Rotary Encoder ---
void handleRotaryEncoder() {
  int currentCLK = digitalRead(CLK);
  if (currentCLK != lastCLK && currentCLK == LOW) {
    int direction = (digitalRead(DT) != currentCLK) ? 1 : -1;

    switch (currentMode) {
      case MODE_ANGLE:
        rotaryStep = 1.0;
        targetAngle += direction * rotaryStep;
        if (targetAngle >= 360) targetAngle -= 360; 
        else if (targetAngle < 0) targetAngle += 360;
        break;
      case MODE_KP:
        rotaryStep = 0.1;
        Kp += direction * rotaryStep;
        if (Kp < 0) Kp = 0;
        break;
      case MODE_KI:
        rotaryStep = 0.00001;
        Ki += direction * rotaryStep;
        if (Ki < 0) Ki = 0;
        break;
      case MODE_KD:
        rotaryStep = 0.01;
        Kd += direction * rotaryStep;
        if (Kd < 0) Kd = 0;
        break;
    }
    displayValues();
  }
  lastCLK = currentCLK;
}

// --- Fungsi untuk menangani klik tombol Rotary Encoder (Ditambah Hold) ---
void handleRotaryButton() {
  bool currentSW = digitalRead(SW);

  // Deteksi penekanan tombol
  if (currentSW == LOW && lastSW == HIGH) { 
    buttonPressStartTime = millis(); // Catat waktu mulai ditekan
    buttonWasHeld = false; // Reset flag hold
  } 
  // Deteksi penahanan tombol
  else if (currentSW == LOW && !buttonWasHeld && (millis() - buttonPressStartTime >= HOLD_DURATION)) {
    // Tombol ditahan selama HOLD_DURATION (3 detik)
    isCalibrating = true;
    buttonWasHeld = true; // Set flag agar tidak terpicu berulang kali saat ditahan
    Serial.println("Starting Calibration!");
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(tft.width()/2 - getTextWidth("Calibrating...", 1)/2, tft.height()/2 - 20);
    tft.setTextColor(ST77XX_YELLOW);
    tft.println("Calibrating...");
    // Inisialisasi progress bar
    tft.drawRect(tft.width()/2 - 50, tft.height()/2, 100, 10, ST77XX_WHITE);
    // Matikan motor saat kalibrasi dimulai
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
  }
  // Deteksi pelepasan tombol
  else if (currentSW == HIGH && lastSW == LOW) { 
    unsigned long holdDuration = millis() - buttonPressStartTime;

    if (holdDuration < HOLD_DURATION) { // Jika dilepas sebelum durasi hold
      // Ini adalah klik singkat, jalankan logika ganti mode
      switch (currentMode) {
        case MODE_ANGLE:
          currentMode = MODE_KP;
          break;
        case MODE_KP:
          currentMode = MODE_KI;
          break;
        case MODE_KI:
          currentMode = MODE_KD;
          break;
        case MODE_KD:
          currentMode = MODE_ANGLE;
          integral = 0; 
          break;
      }
      displayValues();
    }
    buttonPressStartTime = 0; // Reset timer tekan
  }
  lastSW = currentSW;
}

// --- Fungsi untuk proses kalibrasi ---
void performCalibration() {
  static unsigned long calibrationStartTime = 0;
  static int progressBarWidth = 0;
  static bool calibrationSetupDone = false; // Flag untuk memastikan setup hanya sekali

  if (!calibrationSetupDone) {
    calibrationStartTime = millis();
    progressBarWidth = 0;
    calibrationSetupDone = true;
    // TFT sudah menampilkan "Calibrating..." dan bar di handleRotaryButton()
  }

  // Hitung progres
  float progress = (float)(millis() - calibrationStartTime) / HOLD_DURATION;
  if (progress > 1.0) progress = 1.0;

  int newProgressBarWidth = (int)(progress * 96); // 96 = lebar bar di dalam border (100 - 2 * border)
  if (newProgressBarWidth > progressBarWidth) {
    tft.fillRect(tft.width()/2 - 48, tft.height()/2 + 2, newProgressBarWidth, 6, ST77XX_GREEN); // Isi bar
    progressBarWidth = newProgressBarWidth;
  }

  if (progress >= 1.0 && calibrationSetupDone) { // Pastikan selesai dan setup sudah dilakukan
    // Kalibrasi selesai
    isCalibrating = false;
    calibrationStartTime = 0;
    progressBarWidth = 0;
    calibrationSetupDone = false; // Reset untuk kalibrasi berikutnya
    
    // Atur semua parameter ke 0
    targetAngle = 0;
    Kp = 0;
    Ki = 0;
    Kd = 0;
    error = 0;
    previousError = 0;
    integral = 0;
    outputPID = 0;
    numberOfTurns = 0; // Reset juga jumlah putaran jika ini dianggap "kalibrasi total"

    // Kembali ke tampilan normal
    displayValues(); 
    Serial.println("Calibration Complete! All parameters reset to 0.");
  }
}

// --- Fungsi-fungsi AS5600, Motor, PID (tidak berubah) ---
void checkMagnetPresence() {
  tft.fillScreen(ST77XX_BLACK); // Hapus layar saat mulai cek magnet
  tft.setCursor(10, tft.height() / 2 - 7); // Posisi tengah vertikal
  tft.setTextColor(ST77XX_RED);
  tft.println("Checking Magnet...");

  while ((magnetStatus & 32) != 32) {
    magnetStatus = 0;

    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x0B);
    Wire.endTransmission();
    Wire.requestFrom(AS5600_ADDR, 1);

    while (Wire.available() == 0);
    magnetStatus = Wire.read();

    tft.fillRect(0, tft.height() / 2 + 10, tft.width(), 20, ST77XX_BLACK); // Bersihkan pesan sebelumnya
    tft.setCursor(10, tft.height() / 2 + 10); // Posisi pesan status magnet
    tft.setTextColor(ST77XX_WHITE);
    if ((magnetStatus & 8) == 8) {
      tft.println("Magnet too strong!");
    } else if ((magnetStatus & 16) == 16) {
      tft.println("Magnet too weak!");
    } else {
      tft.println("Adjusting magnet...");
    }
    delay(200);
  }
  // Tidak perlu delay atau pesan "Magnet Found!" di sini, langsung kembali ke main loop.
}

void ReadRawAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0D);
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR, 1);
  while (Wire.available() == 0);
  lowbyte_as5600 = Wire.read();

  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C);
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR, 1); 
  while (Wire.available() == 0);
  highbyte_as5600 = Wire.read();

  highbyte_as5600 = highbyte_as5600 << 8;
  rawAngle_as5600 = highbyte_as5600 | lowbyte_as5600; 
  degAngle_as5600 = rawAngle_as5600 * 0.087890625;
}

void correctAngle() {
  correctedAngle = degAngle_as5600 - startAngle_as5600;
  if (correctedAngle < 0) {
    correctedAngle += 360;
  }
}

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
      numberOfTurns++;
    } else if (quadrantNumber == 4 && previousquadrantNumber == 1) {
      numberOfTurns--;
    }
    previousquadrantNumber = quadrantNumber;
  }
}

float readAS5600AngleCombined() {
  ReadRawAngle();
  correctAngle();
  checkQuadrant();
  return correctedAngle;
}

void driveMotor(int speed) {
  if (abs(speed) < 10) {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
    return;
  }
  if (speed > 0) {
    analogWrite(RPWM, abs(speed));
    analogWrite(LPWM, 0);
  } else {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, abs(speed));
  }
}

void calculatePID() {
  error = targetAngle - currentAngle;
  integral += error;
  if (integral > 200) integral = 200;
  if (integral < -200) integral = -200;

  float derivative = error - previousError;
  outputPID = (Kp * error) + (Ki * integral) + (Kd * derivative);
  previousError = error;
}

// --- Fungsi Tampilan TFT ---
void displayValues() {
  tft.fillScreen(ST77XX_BLACK); // Bersihkan layar
  tft.setTextColor(ST77XX_WHITE); // Semua teks warna putih

  int paddingX = 5; // Padding kiri
  int currentY = 5; // Posisi Y awal
  int lineSpacingSmall = 12; // Spasi untuk teks ukuran 1 (tinggi font 7 + 5)
  int lineSpacingMedium = 18; // Spasi untuk teks ukuran 1.5 (tinggi font ~10.5 + 7.5)


  // --- Target & Motor (Ukuran teks yang disesuaikan) ---
  tft.setTextSize(2); 

  // Teks "Target:"
  tft.setCursor(paddingX, currentY);
  tft.setTextColor((currentMode == MODE_ANGLE) ? ST77XX_YELLOW : ST77XX_WHITE); // Highlight label
  tft.print("Target:");
  
  // Nilai Target
  tft.setCursor(paddingX + getTextWidth("Target:", 2), currentY); // Setelah "Target:"
  tft.print(targetAngle, 0); // Presisi 0 untuk tampilan besar
  tft.print("\xF8"); // Simbol derajat
  currentY += lineSpacingMedium; // Pindah ke baris Motor

  // Teks "Motor:"
  tft.setTextSize(2); // Label Motor ukuran 1
  tft.setTextColor(ST77XX_WHITE); // Motor tidak highlight, selalu putih
  tft.setCursor(paddingX, currentY);
  tft.print("Motor:");

  // Nilai Motor
  tft.setCursor(paddingX + getTextWidth("Motor:", 2), currentY); // Setelah "Motor:"
  tft.print(currentAngle, 0); // Presisi 0 untuk tampilan besar
  tft.print("\xF8"); // Simbol derajat
  currentY += lineSpacingMedium + 5; // Spasi setelah Motor, sebelum Error

  // --- Error: & PID Output: (Ukuran teks sedang) ---
  tft.setTextSize(1); // Ukuran sedang (default)
  tft.setTextColor(ST77XX_WHITE);

  tft.setCursor(paddingX, currentY);
  tft.print("Error:");
  tft.print(error, 1);
  currentY += lineSpacingSmall; 

  tft.setCursor(paddingX, currentY);
  tft.print("PID Out: ");
  tft.print(outputPID, 0);
  currentY += lineSpacingSmall + 5; // Spasi setelah PID Out, sebelum Kp

  // --- Kp: Ki: Kd: (Ukuran teks sedang) ---
  tft.setCursor(paddingX, currentY);
  tft.setTextColor((currentMode == MODE_KP) ? ST77XX_YELLOW : ST77XX_WHITE); // Highlight warna
  tft.print("Kp: ");
  tft.print(Kp, 3);
  currentY += lineSpacingSmall;

  tft.setCursor(paddingX, currentY);
  tft.setTextColor((currentMode == MODE_KI) ? ST77XX_YELLOW : ST77XX_WHITE);
  tft.print("Ki: ");
  tft.print(Ki, 5);
  currentY += lineSpacingSmall;

  tft.setCursor(paddingX, currentY);
  tft.setTextColor((currentMode == MODE_KD) ? ST77XX_YELLOW : ST77XX_WHITE);
  tft.print("Kd: ");
  tft.print(Kd, 3);
  currentY += lineSpacingSmall + 5; // Spasi setelah Kd, sebelum SSID

  // --- Informasi Jaringan (Ukuran teks sedang, selalu tampil) ---
  tft.setCursor(paddingX, currentY);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("SSID: ");
  tft.println(ssid);
  currentY += lineSpacingSmall;

  tft.setCursor(paddingX, currentY);
  tft.print("Pass: ");
  tft.println(password);
  currentY += lineSpacingSmall;

  tft.setCursor(paddingX, currentY);
  tft.print("IP: ");
  tft.println(WiFi.softAPIP());
}

// --- Implementasi Fungsi WebServer ---

void setupCaptivePortal() {
  Serial.print("Setting AP (Access Point)... ");
  WiFi.softAP(ssid, password);

  WiFi.softAPConfig(apIP, apIP, netMsk);

  Serial.print("AP SSID: ");
  Serial.println(ssid);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  dnsServer.start(53, "*", apIP);

  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html").setCacheControl("max-age=600"); 
  server.serveStatic("/halo.txt", SPIFFS, "/halo.txt").setCacheControl("max-age=600");

  server.on("/", HTTP_GET, handleRoot); 

  server.onNotFound(handleNotFound);
  
  server.begin(); 
  Serial.println("HTTP server started (Async)");
}

void handleRoot(AsyncWebServerRequest *request) {
  String html = "<!DOCTYPE html><html lang='en'><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>PID Tuner</title>";
  html += "<style>body{font-family:sans-serif; text-align:center; margin-top:50px;} button{padding:10px 20px; font-size:16px;}</style>";
  html += "</head><body><h1>Welcome to ESP32 PID Tuner!</h1><p>Connect to this Wi-Fi to access the tuning interface.</p>";
  html += "<p><a href='/index.html'><button>Go to Web Interface</button></a></p>"; 
  html += "</body></html>";
  request->send(200, "text/html", html);
}

void handleNotFound(AsyncWebServerRequest *request) {
  if (!request->host().equalsIgnoreCase(apIP.toString())) {
    request->redirect("http://" + apIP.IPAddress::toString());
  } else {
    request->send(404, "text/plain", "Not Found");
  }
}