#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// === TFT Setup ===
#define TFT_CS    5
#define TFT_RST   4
#define TFT_DC    15
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// === AS5600 (I2C Address) ===
#define AS5600_ADDR 0x36

// === Rotary Encoder ===
#define CLK 32
#define DT  25
#define SW  13

// === BTS7960 ===
#define RPWM 2
#define LPWM 0
#define REN 27
#define LEN 14

// === Variabel Global ===
float targetAngle = 0;
float currentAngle = 0;

int lastCLK = HIGH;
bool lastSW = HIGH;
unsigned long lastButtonPress = 0; // Untuk debounce non-blocking
int rotaryStep = 5; // Step sudut tiap klik rotary


void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  // Rotary
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  pinMode(SW, INPUT_PULLUP);
  lastCLK = digitalRead(CLK);

  // Motor (pin tetap disiapkan untuk nanti)
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);
  // Pastikan motor tidak bergerak saat start
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);

  // TFT
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2); // Ukuran teks diperbesar agar lebih jelas

  // Baca posisi awal motor sebagai target awal
  targetAngle = readAS5600Angle();
  currentAngle = targetAngle;
  displayValues();
}

void loop() {
  // === BACA ROTARY ENCODER UNTUK MENGUBAH TARGET ANGLE ===
  int currentCLK = digitalRead(CLK);
  if (currentCLK != lastCLK && currentCLK == LOW) {
    // DIUBAH: Logika arah putaran yang lebih andal
    int direction = (digitalRead(DT) != currentCLK) ? 1 : -1;
    targetAngle += direction * rotaryStep;
    // DIHAPUS: Batasan 0-360 pada targetAngle, biarkan berputar bebas
  }
  lastCLK = currentCLK;

  // === BACA POSISI MOTOR SAAT INI ===
  currentAngle = readAS5600Angle();

  // === BACA TOMBOL (UNTUK RESET TARGET) ===
  bool currentSW = digitalRead(SW);
  // DIUBAH: Logika tombol dengan debounce non-blocking
  if (lastSW == HIGH && currentSW == LOW && (millis() - lastButtonPress) > 250) {
    lastButtonPress = millis();
    // FUNGSI BARU: Tekan tombol untuk menyamakan target dengan posisi motor saat ini
    targetAngle = currentAngle;
  }
  lastSW = currentSW;
  
  // === PERBARUI TAMPILAN ===
  displayValues();

  // Tidak ada kontrol motor di sini, hanya pembacaan dan tampilan
  // Fungsi driveMotor() bisa ditambahkan kembali saat logika kontrol siap

  delay(10); // Memberi jeda agar loop tidak terlalu cepat
}

// Fungsi ini disimpan untuk penggunaan di masa depan, tapi tidak dipanggil dari loop()
void driveMotor(int speed) {
  if (abs(speed) < 10) { // Deadband
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
    return;
  }

  if (speed > 0) {
    analogWrite(RPWM, speed);
    analogWrite(LPWM, 0);
  } else {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, -speed);
  }
}

// Fungsi ini sudah benar, tidak perlu diubah
float readAS5600Angle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0E); // Register sudut MSB
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR, 2);

  if (Wire.available() < 2) {
    return currentAngle; // Jika gagal baca, kembalikan nilai terakhir
  }

  uint16_t high = Wire.read();
  uint16_t low = Wire.read();
  uint16_t raw = ((high & 0x0F) << 8) | low;

  return (raw * 360.0) / 4096.0;
}

// DISEDERHANAKAN: Hanya menampilkan nilai yang relevan
void displayValues() {
  tft.fillScreen(ST77XX_BLACK);
  
  tft.setCursor(10, 20);
  tft.setTextColor(ST77XX_YELLOW);
  tft.print("Target: ");
  tft.print(targetAngle, 1);
  tft.print(" deg");

  tft.setCursor(10, 60);
  tft.setTextColor(ST77XX_CYAN);
  tft.print("Motor : ");
  tft.print(currentAngle, 1);
  tft.print(" deg");
}