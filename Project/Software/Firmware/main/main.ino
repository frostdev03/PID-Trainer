#include <Wire.h> // Untuk komunikasi I2C (AS5600 dan OLED/TFT)
#include <Adafruit_GFX.h> // Library grafis dasar untuk TFT
#include <Adafruit_ST7735.h> // Driver spesifik untuk TFT ST7735

// --- Library Baru untuk Wi-Fi dan WebServer (Async) ---
#include <WiFi.h> // Untuk fungsionalitas Wi-Fi (AP mode)
#include <ESPAsyncWebServer.h> // Untuk membuat server HTTP Asynchronous
#include <DNSServer.h> // Untuk Captive Portal (mengarahkan DNS)
#include <SPIFFS.h> // Untuk sistem file SPIFFS (menyimpan halaman web)
#include <WebSocketsServer.h> // Library untuk WebSocket Server
#include <ArduinoJson.h> // Library untuk parsing JSON

// --- TFT Setup ---
#define TFT_CS    5
#define TFT_RST   4
#define TFT_DC    15
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// --- AS5600 (I2C Address) ---
#define AS5600_ADDR 0x36

// --- Definisi pin I2C BARU untuk AS5600 ---
#define AS5600_SDA_PIN 16
#define AS5600_SCL_PIN 17

// --- Rotary Encoder ---
#define CLK 32
#define DT  25
#define SW  13 // Rotary Encoder Button

// --- BTS7960 Motor Driver ---
#define RPWM 2
#define LPWM 0
#define REN 27
#define LEN 14

// --- Variabel Global AS5600 ---
int magnetStatus = 0;
int lowbyte_as5600;
word highbyte_as5600;
int rawAngle_as5600;
float degAngle_as5600;

int quadrantNumber, previousquadrantNumber;
float numberOfTurns = 0;
float correctedAngle = 0;
float startAngle_as5600 = 0;
float totalAngle = 0;
float previousTotalAngle = 0;

// --- Variabel Global Kontrol (Target, Aktual, Rotary Encoder, PID) ---
float targetAngle = 0;
float currentAngle = 0;

bool isEditMode = false;  // flag mode web

int lastCLK = HIGH;
bool lastSW = HIGH;
unsigned long lastButtonPress = 0;
unsigned long buttonPressStartTime = 0; // Waktu tombol ditekan (bukan hanya hold)
const unsigned long HOLD_DURATION = 3000; // Durasi hold untuk kalibrasi (3 detik)
bool isCalibrating = false; // Flag untuk status kalibrasi
bool buttonWasHeld = false; // Flag untuk melacak apakah tombol sudah dianggap ditahan

float rotaryStep = 0.01; // Step default untuk derajat target

// --- Variabel PID ---
float Kp = 0.6;   // Proportional gain (sesuaikan)
float Ki = 0.0001; // Integral gain (sesuaikan)
float Kd = 0.25;  // Derivative gain (sesuaikan)

float error = 0;
float previousError = 0;
float integral = 0;
float outputPID = 0;

unsigned long lastPIDTime = 0;
int pidInterval = 10; // Interval perhitungan PID (ms)

// --- Variabel tampilan sebelumnya untuk deteksi perubahan ---
float lastTargetAngle = -999;
float lastCurrentAngle = -999;
float lastError = -999;
float lastPIDOut = -999;
float lastKp = -999;
float lastKi = -999;
float lastKd = -999;

// --- Definisi Mode Rotary Encoder ---
enum RotaryMode {
  MODE_ANGLE,
  MODE_KP,
  MODE_KI,
  MODE_KD
};
RotaryMode currentMode = MODE_ANGLE; // Mode awal adalah mengatur derajat

RotaryMode lastMode = MODE_ANGLE;
bool shouldUpdateTFT = true;

// --- Variabel Wi-Fi AP dan WebServer ---
const char *ssid = "PID Traianer"; // Nama Wi-Fi AP
const char *password = "password123"; // Password Wi-Fi AP (min 8 karakter)
IPAddress apIP(192, 168, 4, 1); // Alamat IP untuk AP
IPAddress netMsk(255, 255, 255, 0); // Subnet mask

DNSServer dnsServer;
AsyncWebServer server(80); // WebServer di port 80 - Menggunakan AsyncWebServer!
WebSocketsServer webSocket = WebSocketsServer(81); // WebSocket Server di port 81

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
void handleRotaryButton();
void performCalibration();

// --- Fungsi WebServer ---
void handleRoot(AsyncWebServerRequest *request);
void handleNotFound(AsyncWebServerRequest *request);
void setupCaptivePortal();

// --- Fungsi WebSocket ---
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
void sendPIDDataToWeb(); // Fungsi untuk mengirim data PID ke web

// Fungsi bantuan untuk mendapatkan lebar teks dengan ukuran font tertentu
int getTextWidth(String text, int textSize) {
  return text.length() * 6 * textSize; // 6 piksel per karakter (5px char + 1px spacing)
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
  tft.setRotation(1); // ROTASI HORIZONTAL: 0
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
    while (true); // Stop if SPIFFS fails
  }
  Serial.println("SPIFFS Mounted Successfully");
  
  // --- Setup Captive Portal (AP Mode) dan WebSockets ---
  setupCaptivePortal(); 
  webSocket.begin(); // Mulai WebSocket server
  webSocket.onEvent(webSocketEvent); // Daftarkan event handler untuk WebSocket

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
  // --- Tangani WebServer, DNS, dan WebSocket ---
  dnsServer.processNextRequest(); 
  webSocket.loop(); // Penting: Panggil ini di loop() untuk memproses event WebSocket
  
  // Hanya jalankan logika utama jika tidak sedang kalibrasi
  if (!isCalibrating) {
    handleRotaryEncoder();
    handleRotaryButton();

    currentAngle = readAS5600AngleCombined();

    if (millis() - lastPIDTime >= pidInterval) {
      calculatePID();
      int motorSpeed = constrain(outputPID, -255, 255);
      driveMotor(motorSpeed);
      lastPIDTime = millis();
      sendPIDDataToWeb(); // Kirim data PID ke web setelah perhitungan
    }

    if (abs(currentAngle - previousTotalAngle) > 0.1 || (millis() % 200 < pidInterval) || (millis() % 2000 < pidInterval) ) {
      checkTFTUpdate();
      displayValues();
      previousTotalAngle = currentAngle;
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
      // Debugging Serial Plotter
      // Serial.print("Target: ");
      Serial.println(targetAngle);
      // Serial.print(" Current: ");
      Serial.println(currentAngle);
      // Serial.print(" PID_Out: ");
      // Serial.print(outputPID);
      // Serial.print(" Kp: ");
      // Serial.print(Kp);
      // Serial.print(" Ki: ");
      // Serial.print(Ki);
      // Serial.print(" Kd: ");
      // Serial.println(Kd);
    }
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
    shouldUpdateTFT = true;
    // displayValues();
    sendPIDDataToWeb(); // Kirim perubahan dari rotary ke web
  }
  lastCLK = currentCLK;
}

// --- Fungsi untuk menangani klik tombol Rotary Encoder (Ditambah Hold) ---
void handleRotaryButton() {
  bool currentSW = digitalRead(SW);

  // Deteksi penekanan tombol
  if (currentSW == LOW && lastSW == HIGH) { 
    buttonPressStartTime = millis();
    buttonWasHeld = false;
  } 
  // Deteksi penahanan tombol
  else if (currentSW == LOW && !buttonWasHeld && (millis() - buttonPressStartTime >= HOLD_DURATION)) {
    isCalibrating = true;
    buttonWasHeld = true;
    Serial.println("Starting Calibration!");
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(tft.width()/2 - getTextWidth("Calibrating...", 1)/2, tft.height()/2 - 20);
    tft.setTextColor(ST77XX_YELLOW);
    tft.println("Calibrating...");
    tft.drawRect(tft.width()/2 - 50, tft.height()/2, 100, 10, ST77XX_WHITE);
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
    sendPIDDataToWeb(); // Kirim status kalibrasi ke web (opsional)
  }
  // Deteksi pelepasan tombol
  else if (currentSW == HIGH && lastSW == LOW) { 
    unsigned long holdDuration = millis() - buttonPressStartTime;

    if (holdDuration < HOLD_DURATION) {
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
      shouldUpdateTFT = true;
      // displayValues();
      sendPIDDataToWeb(); // Kirim perubahan mode (dan nilai) ke web
    }
    buttonPressStartTime = 0;
  }
  lastSW = currentSW;
}

// --- Fungsi untuk proses kalibrasi ---
void performCalibration() {
  static unsigned long calibrationStartTime = 0;
  static int progressBarWidth = 0;
  static bool calibrationSetupDone = false;

  if (!calibrationSetupDone) {
    calibrationStartTime = millis();
    progressBarWidth = 0;
    calibrationSetupDone = true;
  }

  float progress = (float)(millis() - calibrationStartTime) / HOLD_DURATION;
  if (progress > 1.0) progress = 1.0;

  int newProgressBarWidth = (int)(progress * 96); // 96 = lebar bar di dalam border (100 - 2 * border)
  if (newProgressBarWidth > progressBarWidth) {
    tft.fillRect(tft.width()/2 - 48, tft.height()/2 + 2, newProgressBarWidth, 6, ST77XX_GREEN);
    progressBarWidth = newProgressBarWidth;
  }

  if (progress >= 1.0 && calibrationSetupDone) {
    isCalibrating = false;
    calibrationStartTime = 0;
    progressBarWidth = 0;
    calibrationSetupDone = false;
    
    targetAngle = 0;
    Kp = 0;
    Ki = 0;
    Kd = 0;
    error = 0;
    previousError = 0;
    integral = 0;
    outputPID = 0;
    numberOfTurns = 0;

shouldUpdateTFT = true;
    // displayValues(); 
    sendPIDDataToWeb();
    Serial.println("Calibration Complete! All parameters reset to 0.");
  }
}

// --- Fungsi-fungsi AS5600, Motor, PID (tidak berubah) ---
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

    while (Wire.available() == 0);
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

void checkTFTUpdate() {
  if (targetAngle != lastTargetAngle ||
      currentAngle != lastCurrentAngle ||
      error != lastError ||
      outputPID != lastPIDOut ||
      Kp != lastKp ||
      Ki != lastKi ||
      Kd != lastKd ||
      currentMode != lastMode) {

    shouldUpdateTFT = true;

    lastTargetAngle = targetAngle;
    lastCurrentAngle = currentAngle;
    lastError = error;
    lastPIDOut = outputPID;
    lastKp = Kp;
    lastKi = Ki;
    lastKd = Kd;
    lastMode = currentMode;
  }
}

// --- Fungsi Tampilan TFT ---
void displayValues() {
  if (!shouldUpdateTFT) return;
  shouldUpdateTFT = false;

  tft.fillScreen(ST77XX_BLACK); // Bersihkan layar
  tft.setTextColor(ST77XX_WHITE); // Semua teks warna putih

  int paddingX = 5; // Padding kiri
  int currentY = 5; // Posisi Y awal
  int lineSpacingSmall = 12; // Spasi untuk teks ukuran 1 (tinggi font 7 + 5)
  int lineSpacingLarge = 24; // Spasi untuk teks ukuran 2 (tinggi font 14 + 10)


  // --- Target & Motor (Ukuran teks yang disesuaikan) ---
  // Teks "Target:" dan nilainya
  tft.setTextSize(1); // Ukuran teks besar untuk "Target:" dan nilainya
  tft.setCursor(paddingX, currentY);
  tft.setTextColor((currentMode == MODE_ANGLE) ? ST77XX_YELLOW : ST77XX_WHITE); // Highlight label dan nilai
  tft.print("TARGET:");
  tft.print(targetAngle, 0); // Presisi 0 untuk tampilan besar
  currentY += lineSpacingSmall;

  // Teks "Motor:" dan nilainya
  tft.setTextSize(1); // Ukuran teks besar untuk "Motor:" dan nilainya
  tft.setCursor(paddingX, currentY);
  tft.setTextColor(ST77XX_WHITE); // Motor tidak highlight, selalu putih
  tft.print("MOTOR:");
  tft.print(currentAngle, 0); // Presisi 0 untuk tampilan besar
  currentY += lineSpacingSmall;

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
  // tft.setCursor(paddingX, currentY);
  // tft.setTextColor(ST77XX_WHITE);
  // tft.print("SSID: ");
  // tft.println(ssid);
  // currentY += lineSpacingSmall;

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

// --- Implementasi Fungsi WebSocket ---
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED: {
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
          // displayValues();
          sendPIDDataToWeb();
        } else if (commandType == "target_set") {
          targetAngle = doc["value"];
          if (targetAngle >= 360) targetAngle -= 360; 
          else if (targetAngle < 0) targetAngle += 360;
          Serial.printf("Received Target update: %f\n", targetAngle);
          // displayValues();
          shouldUpdateTFT = true;
          sendPIDDataToWeb();
        } else if (commandType == "calibrate") {
          isCalibrating = true;
          buttonPressStartTime = millis(); // Simulasikan penekanan tombol
          buttonWasHeld = true; // Agar tidak memicu klik singkat
          Serial.println("Calibration triggered from web!");
          tft.fillScreen(ST77XX_BLACK);
          tft.setCursor(tft.width()/2 - getTextWidth("Calibrating...", 1)/2, tft.height()/2 - 20);
          tft.setTextColor(ST77XX_YELLOW);
          tft.println("Calibrating...");
          tft.drawRect(tft.width()/2 - 50, tft.height()/2, 100, 10, ST77XX_WHITE);
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

    doc["target"] = targetAngle;
    doc["motor"] = currentAngle;
if (!isEditMode) {
      doc["target_sync"] = targetAngle;  // Disinkronkan ke form input hanya kalau bukan edit
        doc["kp"] = Kp;
        doc["ki"] = Ki;
        doc["kd"] = Kd;
    }
    doc["error"] = error;
    doc["pid_out"] = outputPID;
    doc["time_s"] = millis() / 1000.0; // Tambahkan waktu dalam detik

    String jsonString;
    serializeJson(doc, jsonString);
    webSocket.broadcastTXT(jsonString);
  }
}