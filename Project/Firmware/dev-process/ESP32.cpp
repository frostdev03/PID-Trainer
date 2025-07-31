// ESP32 PID Trainer - Main Program
// Kompatibel dengan: Rotary Encoder, Motor Driver L298N, TFT LCD SPI 1.3", MicroSD
// Dibuat oleh Tim Riset - Standar Coding Terstruktur

#include <SPI.h>
#include <TFT_eSPI.h>
#include <Encoder.h>
#include <SD.h>
#include <FS.h>

#define ENCODER_CLK 32
#define ENCODER_DT 33
#define ENCODER_SW 25

#define MOTOR_IN1 26
#define MOTOR_IN2 27
#define MOTOR_ENA 14

#define SD_CS     5

// TFT (menggunakan TFT_eSPI config khusus, pastikan User_Setup.h sesuai)
TFT_eSPI tft = TFT_eSPI();

// Rotary Encoder
Encoder myEnc(ENCODER_CLK, ENCODER_DT);
long lastEncoderValue = 0;
int paramIndex = 0; // 0 = Kp, 1 = Ki, 2 = Kd
float Kp = 1.0, Ki = 0.0, Kd = 0.0;

// Tombol Encoder
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// SD
File logFile;

void setup() {
  Serial.begin(115200);

  // --- Inisialisasi Pin --- //
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(ENCODER_SW, INPUT_PULLUP);

  // TFT
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);

  // SD
  if (!SD.begin(SD_CS)) {
    Serial.println("Gagal inisialisasi SD card");
  } else {
    Serial.println("SD card terdeteksi");
  }

  tampilkanNilaiPID();
}

void loop() {
  // --- Encoder Rotasi --- //
  long newValue = myEnc.read() / 4;
  if (newValue != lastEncoderValue) {
    float delta = (newValue - lastEncoderValue) * 0.1;
    switch (paramIndex) {
      case 0: Kp += delta; break;
      case 1: Ki += delta; break;
      case 2: Kd += delta; break;
    }
    lastEncoderValue = newValue;
    tampilkanNilaiPID();
  }

  // --- Tombol Encoder --- //
  bool reading = digitalRead(ENCODER_SW);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading == LOW && lastButtonState == HIGH) {
      paramIndex = (paramIndex + 1) % 3;
      tampilkanNilaiPID();
    }
  }
  lastButtonState = reading;

  // --- Motor Contoh Aktivasi --- //
  analogWrite(MOTOR_ENA, 180); // PWM kecepatan
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);

  //--- Logging SD --- //
  if (SD.begin(SD_CS)) {
    logFile = SD.open("pid_log.txt", FILE_APPEND);
    if (logFile) {
      logFile.printf("Kp=%.2f Ki=%.2f Kd=%.2f\n", Kp, Ki, Kd);
      logFile.close();
    }
  }
  delay(500);
}

void tampilkanNilaiPID() {
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
  tft.printf("Kp: %.2f\n", Kp);
  tft.printf("Ki: %.2f\n", Ki);
  tft.printf("Kd: %.2f\n", Kd);
  tft.setCursor(0, 100);
  tft.printf("Edit: %s", paramIndex == 0 ? "Kp" : paramIndex == 1 ? "Ki" : "Kd");
}
