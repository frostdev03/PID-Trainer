#include <Wire.h>             // Untuk komunikasi I2C (AS5600 dan OLED/TFT)
#include <Adafruit_GFX.h>     // Library grafis dasar untuk TFT
#include <Adafruit_ST7735.h>  // Driver spesifik untuk TFT ST7735

// --- Library Baru untuk Wi-Fi dan WebServer (Async) ---
#include <WiFi.h>               // Untuk fungsionalitas Wi-Fi (AP mode)
#include <ESPAsyncWebServer.h>  // Untuk membuat server HTTP Asynchronous
#include <DNSServer.h>          // Untuk Captive Portal (mengarahkan DNS)
#include <SPIFFS.h>             // Untuk sistem file SPIFFS (menyimpan halaman web)
#include <WebSocketsServer.h>   // Library untuk WebSocket Server
#include <ArduinoJson.h>        // Library untuk parsing JSON

#include "KalmanFilter.h"
KalmanFilter kalman;

// --- TFT Setup ---
#define TFT_CS 5
#define TFT_RST 4
#define TFT_DC 15
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// --- AS5600 (I2C Address) ---
#define AS5600_ADDR 0x36

// --- Definisi pin I2C BARU untuk AS5600 ---
#define AS5600_SDA_PIN 16
#define AS5600_SCL_PIN 17

// --- Rotary Encoder ---
#define CLK 32
#define DT 25
#define SW 13  // Rotary Encoder Button

// --- BTS7960 Motor Driver ---
#define RPWM 2
#define LPWM 0
#define REN 27
#define LEN 14

// kalman
float lastAngle = 0.0;

// --- Variabel Global AS5600 (DISEDERHANAKAN untuk ZPOS) ---
int magnetStatus = 0;
int lowbyte_as5600;
word highbyte_as5600;
int rawAngle_as5600;
float degAngle_as5600;

// --- Variabel Global Kontrol (Target, Aktual, Rotary Encoder, PID) ---
float targetAngle = 0;
float currentAngle = 0;
bool needFullRefresh = true;  // Flag untuk menandakan perlu full refresh layar

bool isEditMode = false;  // flag mode web

int lastCLK = HIGH;
int lastDT = HIGH;
bool lastSW = HIGH;
unsigned long lastEncoderTime = 0;
unsigned long lastButtonTime = 0;
unsigned long buttonPressStartTime = 0;
const unsigned long ENCODER_DEBOUNCE_MS = 2;  // Debounce untuk rotary
const unsigned long BUTTON_DEBOUNCE_MS = 50;  // Debounce untuk tombol
const unsigned long HOLD_DURATION = 1500;     // Hold duration untuk reset
bool isCalibrating = false;
bool buttonWasHeld = false;

float rotaryStep = 0.01;  // Step default untuk derajat target

// --- Variabel PID ---
float Kp = 0.7000;  // Proportional gain (sesuaikan)
float Ki = 0.00010;  // Integral gain (sesuaikan)
float Kd = 0.001000;  // Derivative gain (sesuaikan)
float dt = 0;
float error = 0;
float previousError = 0;
float integral = 0;
float outputPID = 0;
float derivative = 0;

const float CW_LIMIT = 35.0;
const float CCW_LIMIT = -35.0;

bool pidEnabled = true;
unsigned long startTime = 0;

unsigned long lastPIDTime = 0;
int pidInterval = 10;  // Interval perhitungan PID (ms)

// --- Variabel tampilan sebelumnya untuk deteksi perubahan ---
float lastTargetAngle = -999;
float lastCurrentAngle = -999;
float lastError = -999;
float lastPIDOut = -999;
float lastKp = -999;
float lastKi = -999;
float lastKd = -999;
int lastDisplayTarget = -999;
int lastDisplayAngle = -999;
float lastDisplayError = -999.0;
float lastDisplayPID = -999.0;

// --- Definisi Mode Rotary Encoder ---
enum RotaryMode {
  MODE_KP,
  MODE_KI,
  MODE_KD
};
RotaryMode currentMode = MODE_KP;  // Mode awal adalah mengatur Kp

RotaryMode lastMode = MODE_KP;
bool shouldUpdateTFT = true;

// --- Variabel Wi-Fi AP dan WebServer ---
const char *ssid = "PID Trainer";    // Nama Wi-Fi AP
const char *password = "motordc1";   // Password Wi-Fi AP (min 8 karakter)
IPAddress apIP(192, 168, 4, 1);      // Alamat IP untuk AP
IPAddress netMsk(255, 255, 255, 0);  // Subnet mask

DNSServer dnsServer;
AsyncWebServer server(80);                          // WebServer di port 80 - Menggunakan AsyncWebServer!
WebSocketsServer webSocket = WebSocketsServer(81);  // WebSocket Server di port 81

// --- Fungsi Prototype ---
void checkMagnetPresence();
void ReadRawAngle();
float readAS5600Angle();  // FUNGSI BARU yang lebih sederhana
void calculatePID();
void driveMotor(int speed);
void displayValues();
void handleRotaryEncoder();
void handleRotaryButton();
void performParameterReset();  // GANTI NAMA dari performCalibration
float shortestAngleDiff(float target, float current);
float normalizeAngle(float angle);

// --- Fungsi WebServer ---
void handleRoot(AsyncWebServerRequest *request);
void handleNotFound(AsyncWebServerRequest *request);
void setupCaptivePortal();

// --- Fungsi WebSocket ---
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
void sendPIDDataToWeb();  // Fungsi untuk mengirim data PID ke web

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
  lastSW = digitalRead(SW);  // Inisialisasi lastSW

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
  tft.setRotation(1);  // ROTASI HORIZONTAL: 0
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, tft.height() / 2 - 7);
  tft.println("PID Trainer Initializing...");

  // --- Inisialisasi SPIFFS ---
  if (!SPIFFS.begin(true)) {
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(10, tft.height() / 2 - 7);
    tft.setTextColor(ST77XX_RED);
    tft.println("SPIFFS Mount FAILED!");
    Serial.println("SPIFFS Mount Failed");
    while (true)
      ;  // Stop if SPIFFS fails
  }
  Serial.println("SPIFFS Mounted Successfully");

  // --- Setup Captive Portal (AP Mode) dan WebSockets ---
  setupCaptivePortal();
  webSocket.begin();                  // Mulai WebSocket server
  webSocket.onEvent(webSocketEvent);  // Daftarkan event handler untuk WebSocket

  // --- Cek Magnet AS5600 ---
  checkMagnetPresence();  // Fungsi ini akan menampilkan pesan error magnet jika ada

  // --- INISIALISASI SEDERHANA untuk AS5600 dengan ZPOS ---
  // Tidak perlu kalibrasi startAngle, karena AS5600 sudah ter-burn ZPOS
  currentAngle = readAS5600Angle();  // Baca sudut langsung dari AS5600
  // targetAngle = 0;                   // Set target = 0° (karena ZPOS sudah di-burn)

  lastPIDTime = millis();
}

void loop() {
  // --- Tangani WebServer, DNS, dan WebSocket ---
  dnsServer.processNextRequest();
  webSocket.loop();  // Penting: Panggil ini di loop() untuk memproses event WebSocket

  // debugZPOSShift();

  // Hanya jalankan logika utama jika tidak sedang reset parameter
  if (!isCalibrating) {
    handleRotaryEncoder();
    handleRotaryButton();

    // // Baca sudut dari AS5600 dengan Kalman Filter
    // float measuredAngle = readAS5600Angle();  // Sudut baru dari sensor
    // dt = pidInterval / 1000.0;
    // float rate = (measuredAngle - lastAngle) / dt;

    // currentAngle = kalman.getAngle(measuredAngle, rate, dt);
    // lastAngle = currentAngle;

    // --- continuity-aware rate calc & safer lastAngle handling ---
    // float measuredAngle = readAS5600Angle();
    float measuredAngle = readAS5600AngleSigned();

    dt = pidInterval / 1000.0;

    // gunakan lastRawAngle untuk menyimpan pembacaan raw sebelumnya
    // static float lastRawAngle = 0.0;
    // static bool haveLastRaw = false;

    // float deltaRaw;
    // if (!haveLastRaw) {
    //   deltaRaw = 0.0;
    //   haveLastRaw = true;
    // } else {
    //   deltaRaw = measuredAngle - lastRawAngle;
    //   // wrap-around correction (AS5600 0..360)
    //   if (deltaRaw > 180.0) deltaRaw -= 360.0;
    //   else if (deltaRaw < -180.0) deltaRaw += 360.0;
    // }

    // // hitung rate berdasarkan delta raw (deg/s)
    // float rate = deltaRaw / dt;

    // // cap rate supaya noise besar tidak meledakkan filter
    // const float MAX_RATE = 2000.0;  // deg/s, sesuaikan jika perlu
    // if (rate > MAX_RATE) rate = MAX_RATE;
    // if (rate < -MAX_RATE) rate = -MAX_RATE;

    // feed Kalman with measured raw angle and the computed rate
    // currentAngle = kalman.getAngle(measuredAngle, rate, dt);
    // currentAngle = readAS5600Angle();
    currentAngle = normalizeAngle(measuredAngle);

    // update raw-last, dan jangan set lastAngle = currentAngle (terfilter)
    // agar perhitungan deltaRaw tetap reliable
    // lastRawAngle = measuredAngle;
    // lastAngle = currentAngle;  // optional: jika masih dipakai elsewhere
    // ---------------------------------------------------------------


    if (millis() - lastPIDTime >= pidInterval) {
      calculatePID();
      int motorSpeed = constrain(outputPID, -100, 100);

      driveMotor(motorSpeed);
      lastPIDTime = millis();
      sendPIDDataToWeb();  // Kirim data PID ke web setelah perhitungan
    }

    // if (millis() - lastPIDTime >= pidInterval) {
    //   float errorNow = targetAngle - currentAngle;

    //   // Jika ada gangguan besar (misalnya > toleransi), aktifkan PID
    //   if (fabs(errorNow) > 1.0) {
    //     pidEnabled = true;
    //   }

    //   if (pidEnabled) {
    //     calculatePID();
    //     int motorSpeed = constrain(outputPID, -255, 255);
    //     driveMotor(motorSpeed);

    //     // Matikan kalau sudah mendekati target
    //     if (fabs(errorNow) <= 1.0) {
    //       pidEnabled = false;
    //       driveMotor(0);
    //     }
    //   }


    //   lastPIDTime = millis();
    //   sendPIDDataToWeb();
    // }


    if (abs(currentAngle - lastCurrentAngle) > 0.1 || (millis() % 200 < pidInterval) || (millis() % 2000 < pidInterval)) {
      checkTFTUpdate();
      displayValues();
      lastCurrentAngle = currentAngle;
    }

    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();

      if (input == "mode:edit") {
        isEditMode = true;
      } else if (input == "mode:view") {
        isEditMode = false;
      }
      // tambahkan handler untuk perintah lain jika perlu
    }

    if (!isEditMode) {
      Serial.println(targetAngle);
      Serial.println(currentAngle);
    }
  } else {
    performParameterReset();  // GANTI NAMA fungsi
  }
}

// --- Fungsi untuk menangani putaran Rotary Encoder ---
void handleRotaryEncoder() {
  unsigned long now = millis();

  if (now - lastEncoderTime < ENCODER_DEBOUNCE_MS) {
    return;
  }

  int currentCLK = digitalRead(CLK);
  int currentDT = digitalRead(DT);

  if (currentCLK != lastCLK) {
    lastEncoderTime = now;

    if (currentCLK == LOW) {  // Falling edge pada CLK
      int direction;
      if (currentDT != currentCLK) {
        direction = 1;  // Clockwise
      } else {
        direction = -1;  // Counter-clockwise
      }

      switch (currentMode) {
        case MODE_KP:
          Kp += direction * 0.1;  // Step lebih kecil untuk kontrol yang lebih halus
          // if (Kp < 0) Kp = 0;
          break;
        case MODE_KI:
          Ki += direction * 0.0001;
          // if (Ki < 0) Ki = 0;
          break;
        case MODE_KD:
          Kd += direction * 0.001;
          // if (Kd < 0) Kd = 0;
          break;
      }
      shouldUpdateTFT = true;

      sendPIDDataToWeb();
    }
    lastCLK = currentCLK;
    lastDT = currentDT;
  }
}

// --- Fungsi untuk menangani klik tombol Rotary Encoder (Ditambah Hold) ---
void handleRotaryButton() {
  unsigned long now = millis();
  bool currentSW = digitalRead(SW);

  if (currentSW != lastSW && (now - lastButtonTime > BUTTON_DEBOUNCE_MS)) {
    lastButtonTime = now;

    if (currentSW == LOW && lastSW == HIGH) {
      // Tombol baru ditekan
      buttonPressStartTime = now;
      buttonWasHeld = false;
      Serial.println("Button pressed");
    } else if (currentSW == HIGH && lastSW == LOW) {
      // Tombol dilepas
      unsigned long holdDuration = now - buttonPressStartTime;

      if (holdDuration >= HOLD_DURATION) {
        // Long press sudah dihandle saat hold
        Serial.println("Button released after hold");
      } else if (holdDuration > BUTTON_DEBOUNCE_MS) {
        // Short press - ganti mode
        switch (currentMode) {
          case MODE_KP:
            currentMode = MODE_KI;
            Serial.println("Mode: Ki");
            break;
          case MODE_KI:
            currentMode = MODE_KD;
            Serial.println("Mode: Kd");
            break;
          case MODE_KD:
            currentMode = MODE_KP;
            integral = 0;  // Reset integral saat kembali ke Kp
            Serial.println("Mode: Kp");
            break;
        }
        shouldUpdateTFT = true;
        sendPIDDataToWeb();
      }
      buttonPressStartTime = 0;
    }
  }

  // Deteksi penahanan tombol untuk RESET PARAMETER
  if (currentSW == LOW && !buttonWasHeld && buttonPressStartTime > 0) {
    if (now - buttonPressStartTime >= HOLD_DURATION) {

      isCalibrating = true;  // Gunakan nama variabel yang sama untuk konsistensi
      buttonWasHeld = true;
      Serial.println("Starting Parameter Reset!");
      tft.fillScreen(ST77XX_BLACK);
      tft.setCursor(tft.width() / 2 - getTextWidth("Resetting...", 1) / 2, tft.height() / 2 - 20);
      tft.setTextColor(ST77XX_YELLOW);
      tft.println("Resetting...");
      tft.drawRect(tft.width() / 2 - 50, tft.height() / 2, 100, 10, ST77XX_WHITE);
      analogWrite(RPWM, 0);
      analogWrite(LPWM, 0);
      sendPIDDataToWeb();  // Kirim status reset ke web (opsional)
    }
  }

  // else if (currentSW == HIGH && lastSW == LOW) {
  //   unsigned long holdDuration = millis() - buttonPressStartTime;

  //   if (holdDuration < HOLD_DURATION) {
  //     // Ini adalah klik singkat, jalankan logika ganti mode
  //     switch (currentMode) {
  //       case MODE_KP:
  //         currentMode = MODE_KI;
  //         break;
  //       case MODE_KI:
  //         currentMode = MODE_KD;
  //         break;
  //       case MODE_KD:
  //         currentMode = MODE_KP;
  //         integral = 0;
  //         break;
  //     }
  //     shouldUpdateTFT = true;
  //     sendPIDDataToWeb();
  //   }
  //   buttonPressStartTime = 0;
  // }
  lastSW = currentSW;
}

// --- Fungsi untuk proses reset parameter (GANTI NAMA dari performCalibration) ---
void performParameterReset() {
  static unsigned long resetStartTime = 0;
  static int progressBarWidth = 0;
  static bool resetSetupDone = false;

  if (!resetSetupDone) {
    resetStartTime = millis();
    progressBarWidth = 0;
    resetSetupDone = true;
  }

  float progress = (float)(millis() - resetStartTime) / HOLD_DURATION;
  if (progress > 1.0) progress = 1.0;

  int newProgressBarWidth = (int)(progress * 96);  // 96 = lebar bar di dalam border (100 - 2 * border)
  if (newProgressBarWidth > progressBarWidth) {
    tft.fillRect(tft.width() / 2 - 48, tft.height() / 2 + 2, newProgressBarWidth, 6, ST77XX_GREEN);
    progressBarWidth = newProgressBarWidth;
  }

  if (progress >= 1.0 && resetSetupDone) {
    isCalibrating = false;
    resetStartTime = 0;
    progressBarWidth = 0;
    resetSetupDone = false;

    Kp = 0;
    Ki = 0;
    Kd = 0;
    error = 0;
    previousError = 0;
    integral = 0;
    outputPID = 0;

    triggerFullRefresh();

    shouldUpdateTFT = true;
    sendPIDDataToWeb();
    Serial.println("Parameter Reset Complete! All PID parameters reset to 0.");
  }
}

// --- Fungsi-fungsi AS5600 (DISEDERHANAKAN) ---
void checkMagnetPresence() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(10, tft.height() / 2 - 7);
  tft.setTextColor(ST77XX_RED);
  tft.println("Checking Magnet...");

  while ((magnetStatus & 32) != 32) {
    magnetStatus = 0;

    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x0B);
    Wire.endTransmission();
    Wire.requestFrom(AS5600_ADDR, 1);

    while (Wire.available() == 0)
      ;
    magnetStatus = Wire.read();

    tft.fillRect(0, tft.height() / 2 + 10, tft.width(), 20, ST77XX_BLACK);
    tft.setCursor(10, tft.height() / 2 + 10);
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

  triggerFullRefresh();
}

void ReadProcessedAngle() {
  // Baca PROCESSED angle (0x0E, 0x0F) yang sudah dikurangi ZPOS
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0F);  // ANGLE_LOW register (PROCESSED, bukan RAW)
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR, 1);
  while (Wire.available() == 0)
    ;
  lowbyte_as5600 = Wire.read();

  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0E);  // ANGLE_HIGH register (PROCESSED, bukan RAW)
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR, 1);
  while (Wire.available() == 0)
    ;
  highbyte_as5600 = Wire.read();

  highbyte_as5600 = highbyte_as5600 << 8;
  rawAngle_as5600 = highbyte_as5600 | lowbyte_as5600;
  degAngle_as5600 = rawAngle_as5600 * 0.087890625;
}

float readAS5600Angle() {
  ReadProcessedAngle();  // Gunakan PROCESSED, bukan raw
  // Sekarang akan return ~0° di posisi yang sudah di-burn
  return degAngle_as5600;
}

// Untuk signed angle (-180 to +180)
float readAS5600AngleSigned() {
  ReadProcessedAngle();                 // Gunakan PROCESSED
  float angle = degAngle_as5600;        // 0..360 dari sensor (sudah ter-ZPOS)
  if (angle > 180.0f) angle -= 360.0f;  // jadi -180..180
  return angle;
}

float readAS5600RawAngle() {
  // Baca RAW angle (0x0C, 0x0D) tanpa ZPOS correction
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0D);  // RAW_ANGLE_LOW
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR, 1);
  while (Wire.available() == 0)
    ;
  int lowbyte = Wire.read();

  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C);  // RAW_ANGLE_HIGH
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR, 1);
  while (Wire.available() == 0)
    ;
  int highbyte = Wire.read();

  int rawValue = (highbyte << 8) | lowbyte;
  return rawValue * 0.087890625;
}

// void printAngleDebug() {
//   float rawAngle = readAS5600RawAngle();
//   float processedAngle = readAS5600Angle();

//   Serial.printf("Debug - RAW: %.2f° | PROCESSED: %.2f° | ZPOS_OFFSET: %.2f°\n",
//                 rawAngle, processedAngle, rawAngle - processedAngle);
// }

float normalizeAngle(float angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle <= -180.0) angle += 360.0;
    return angle;
}

float toUnsigned360(float signedAngle) {
  if (signedAngle < 0) signedAngle += 360.0f;
  return signedAngle;
}


void driveMotor(int speed) {
  // Proteksi sudut batas (pakai signed angle)
  bool outOfCWLimit = (currentAngle >= CW_LIMIT) && (speed > 0);
  bool outOfCCWLimit = (currentAngle <= CCW_LIMIT) && (speed < 0);

  if (outOfCWLimit || outOfCCWLimit) {
    speed = 0;
  }

  if (abs(speed) < 5) {
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

float shortestAngleDiff(float target, float current) {
  float diff = target - current;
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;
  return diff;
}

void calculatePID() {

  if (currentAngle > 180) currentAngle -= 360;
  if (currentAngle < -180) currentAngle += 360;

  error = targetAngle - currentAngle;

// Serial.printf("Target: %.2f | Current: %.2f | Error: %.2f | Output(before limit): %.2f\n",
//               targetAngle, currentAngle, error, Kp * error + Ki * integral + Kd * derivative);

  integral += error * dt;
  integral = constrain(integral, -100, 50); // Sesuaikan batas ini

  derivative = (error - previousError) / dt;

  outputPID = Kp * error + Ki * integral + Kd * derivative;

  bool outOfCWLimit = (currentAngle >= CW_LIMIT) && (outputPID > 0);
  bool outOfCCWLimit = (currentAngle <= CCW_LIMIT) && (outputPID < 0);

  if (outOfCWLimit || outOfCCWLimit) {
    outputPID = 0;
  }

  previousError = error;
}

void displayValues() {
  if (!shouldUpdateTFT || isCalibrating) return;
  shouldUpdateTFT = false;

  if (needFullRefresh) {
    tft.fillScreen(ST77XX_BLACK);

    lastDisplayAngle = -999;
    lastDisplayError = -999.0;
    lastDisplayPID = -999.0;
    lastKp = -999;
    lastKi = -999;
    lastKd = -999;

    needFullRefresh = false;  // Reset flag setelah refresh
  }

  int paddingX = 5;
  int currentY = 5;
  int lineSpacingSmall = 12;
  int clearWidth = 120;  // Lebar area yang dibersihkan - disesuaikan dengan panjang teks maksimal

  // --- TARGET ---
  tft.setTextSize(1);
  tft.setCursor(paddingX, currentY);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("TARGET: ");
  tft.print(targetAngle, 0);
  tft.print(" DEG");
  currentY += lineSpacingSmall;

  // --- MOTOR ---
  if (currentAngle != lastDisplayAngle) {
    tft.fillRect(paddingX, currentY, clearWidth, 10, ST77XX_BLACK);

    tft.setTextSize(1);
    tft.setCursor(paddingX, currentY);
    tft.setTextColor(ST77XX_WHITE);
    tft.print("MOTOR: ");
    tft.print(currentAngle, 0);
    // tft.print(toUnsigned360(currentAngle), 0);
    tft.print(" DEG");
    lastDisplayAngle = currentAngle;
  }
  currentY += lineSpacingSmall;

  // --- ERROR ---
  if (abs(error - lastDisplayError) > 0.1) {
    tft.fillRect(paddingX, currentY, clearWidth, 10, ST77XX_BLACK);

    tft.setTextSize(1);
    tft.setCursor(paddingX, currentY);
    tft.setTextColor(ST77XX_WHITE);
    tft.print("Error: ");
    tft.print(error, 1);

    lastDisplayError = error;
  }
  currentY += lineSpacingSmall;

  // --- PID OUT ---
  if (abs(outputPID - lastDisplayPID) > 1.0) {
    tft.fillRect(paddingX, currentY, clearWidth, 10, ST77XX_BLACK);

    tft.setTextSize(1);
    tft.setCursor(paddingX, currentY);
    tft.setTextColor(ST77XX_WHITE);
    tft.print("PID Out: ");
    tft.print(outputPID, 0);

    lastDisplayPID = outputPID;
  }
  currentY += lineSpacingSmall;

  // --- Kp ---
  if (abs(Kp - lastKp) > 0.001) {  // Gunakan threshold kecil untuk float
    tft.fillRect(paddingX, currentY, clearWidth, 10, ST77XX_BLACK);
    tft.setTextSize(1);
    tft.setCursor(paddingX, currentY);
    tft.setTextColor((currentMode == MODE_KP) ? ST77XX_YELLOW : ST77XX_WHITE);
    tft.print("Kp: ");
    tft.print(Kp, 6);

    lastKp = Kp;  // Update nilai terakhir
  }
  currentY += lineSpacingSmall;

  // --- Ki ---
  if (abs(Ki - lastKi) > 0.00001) {  // Threshold sangat kecil untuk Ki
    tft.fillRect(paddingX, currentY, clearWidth, 10, ST77XX_BLACK);

    tft.setTextSize(1);
    tft.setCursor(paddingX, currentY);
    tft.setTextColor((currentMode == MODE_KI) ? ST77XX_YELLOW : ST77XX_WHITE);
    tft.print("Ki: ");
    tft.print(Ki, 6);

    lastKi = Ki;  // Update nilai terakhir
  }
  currentY += lineSpacingSmall;

  // --- Kd ---
  if (abs(Kd - lastKd) > 0.001) {  // Gunakan threshold kecil untuk float
    tft.fillRect(paddingX, currentY, clearWidth, 10, ST77XX_BLACK);

    tft.setTextSize(1);
    tft.setCursor(paddingX, currentY);
    tft.setTextColor((currentMode == MODE_KD) ? ST77XX_YELLOW : ST77XX_WHITE);
    tft.print("Kd: ");
    tft.print(Kd, 6);

    lastKd = Kd;  // Update nilai terakhir
  }
  currentY += lineSpacingSmall + 5;

  tft.setTextSize(1);
  tft.setCursor(paddingX, currentY);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("IP: ");
  tft.print(WiFi.softAPIP());
}

// Juga perlu perbaikan di fungsi checkTFTUpdate untuk menambahkan update variabel display
void checkTFTUpdate() {
  bool modeChanged = (currentMode != lastMode);

  if (currentAngle != lastCurrentAngle || error != lastError || outputPID != lastPIDOut || currentMode != lastMode) {

    shouldUpdateTFT = true;

    if (modeChanged) {
      needFullRefresh = true;
    }

    lastCurrentAngle = currentAngle;
    lastError = error;
    lastPIDOut = outputPID;
    lastMode = currentMode;
  }
}

// Fungsi untuk inisialisasi tampilan pertama kali (panggil di setup atau setelah kalibrasi)
void initializeDisplay() {
  tft.fillScreen(ST77XX_BLACK);

  lastDisplayAngle = -999;
  lastDisplayError = -999.0;
  lastDisplayPID = -999.0;
  lastKp = -999;
  lastKi = -999;
  lastKd = -999;

  shouldUpdateTFT = true;
  displayValues();
}

void triggerFullRefresh() {
  needFullRefresh = true;
  shouldUpdateTFT = true;
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

// --- Implementasi Fungsi WebSocket ---
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
        sendPIDDataToWeb();
      }
      break;
    case WStype_TEXT:
      Serial.printf("[%u] get Text: %s\n", num, payload);
      {
        StaticJsonDocument<250> doc;
        DeserializationError error = deserializeJson(doc, payload);

        if (error) {
          Serial.print(F("deserializeJson() failed: "));
          Serial.println(error.f_str());
          return;
        }

        String commandType = doc["type"];
        if (commandType == "mode") {
          String modeValue = doc["value"];
          if (modeValue == "edit") {
            isEditMode = true;
            Serial.println("Mode: EDIT aktif, tidak kirim data ke UI.");
          } else if (modeValue == "view") {
            isEditMode = false;
            Serial.println("Mode: VIEW aktif, kirim data ke UI.");
          }
        }

        if (commandType == "pid_single") {
          String param = doc["parameter"];
          float val = doc["value"];
          if (param == "kp") {
            Kp = val;
          } else if (param == "ki") {
            Ki = val;
          } else if (param == "kd") {
            Kd = val;
          }
          Serial.printf("Received PID update: %s = %f\n", param.c_str(), val);
          shouldUpdateTFT = true;
          sendPIDDataToWeb();
        } else if (commandType == "calibrate") {
          // UBAH: Sekarang untuk reset parameter, bukan kalibrasi posisi
          isCalibrating = true;
          buttonPressStartTime = millis();
          buttonWasHeld = true;
          Serial.println("Parameter reset triggered from web!");
          tft.fillScreen(ST77XX_BLACK);
          tft.setCursor(tft.width() / 2 - getTextWidth("Resetting...", 1) / 2, tft.height() / 2 - 20);
          tft.setTextColor(ST77XX_YELLOW);
          tft.println("Resetting...");
          tft.drawRect(tft.width() / 2 - 50, tft.height() / 2, 100, 10, ST77XX_WHITE);
          analogWrite(RPWM, 0);
          analogWrite(LPWM, 0);
        }
      }
      break;
    case WStype_BIN:
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
      break;
  }
}

// Fungsi untuk mengirim data PID ke web
void sendPIDDataToWeb() {
  if (webSocket.connectedClients() > 0) {
    StaticJsonDocument<250> doc;

    doc["motor"] = currentAngle;
    if (!isEditMode) {
      doc["kp"] = Kp;
      doc["ki"] = Ki;
      doc["kd"] = Kd;
    }
    doc["error"] = error;
    doc["pid_out"] = outputPID;
    doc["time_s"] = millis() / 1000.0;  // Tambahkan waktu dalam detik

    String jsonString;
    serializeJson(doc, jsonString);
    webSocket.broadcastTXT(jsonString);
  }
}