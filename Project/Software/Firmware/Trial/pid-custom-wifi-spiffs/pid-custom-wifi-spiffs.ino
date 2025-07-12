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
#define SW  13

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
unsigned long lastButtonPress = 0; // Untuk debounce tombol rotary
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
void handleRotaryButton();

// --- Fungsi baru untuk WebServer ---
void handleRoot(AsyncWebServerRequest *request);
void handleNotFound(AsyncWebServerRequest *request);
void setupCaptivePortal(); // Setup Captive Portal

void setup() {
  Serial.begin(115200);
  // Inisialisasi Wire dengan pin I2C yang benar untuk ESP32 (SDA, SCL)
  Wire.begin(AS5600_SDA_PIN, AS5600_SCL_PIN); 
  Wire.setClock(100000); // Kecepatan I2C standar (bisa 100KHz atau 400KHz)

  // --- Inisialisasi Rotary Encoder ---
  pinMode(CLK, INPUT_PULLUP);
  pinMode(DT, INPUT_PULLUP);
  pinMode(SW, INPUT_PULLUP);
  lastCLK = digitalRead(CLK);

  // --- Inisialisasi Motor Driver ---
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);

  // --- Inisialisasi TFT ---
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 10);
  tft.println("PID Trainer");
  tft.setCursor(10, 30);
  tft.println("Initializing...");

  // --- Inisialisasi SPIFFS ---
  tft.setCursor(10, 50);
  tft.print("SPIFFS...");
  if (!SPIFFS.begin(true)) { // true will format SPIFFS if not mounted
    tft.println("FAIL!");
    Serial.println("SPIFFS Mount Failed");
    while (true); // Stop if SPIFFS fails
  }
  tft.println("OK");
  Serial.println("SPIFFS Mounted Successfully");
  
  // --- Setup Captive Portal (AP Mode) ---
  setupCaptivePortal();

  // --- Kalibrasi AS5600 ---
  checkMagnetPresence();
  readAS5600AngleCombined();
  startAngle_as5600 = degAngle_as5600;
  totalAngle = 0; // Reset totalAngle after calibration for clarity

  // Set target awal ke posisi motor saat ini
  targetAngle = readAS5600AngleCombined();
  currentAngle = targetAngle;

  displayValues();
  lastPIDTime = millis();
}

void loop() {
  // --- Tangani WebServer dan DNS ---
  dnsServer.processNextRequest(); // Penting untuk Captive Portal
  // server.handleClient(); // TIDAK DIPERLUKAN LAGI dengan AsyncWebServer

  // --- Tangani Rotary Encoder ---
  handleRotaryEncoder();
  handleRotaryButton();

  // --- Baca Posisi Motor Saat Ini dari AS5600 ---
  currentAngle = readAS5600AngleCombined();

  // --- Hitung dan Terapkan Kontrol PID ---
  if (millis() - lastPIDTime >= pidInterval) {
    calculatePID();
    int motorSpeed = constrain(outputPID, -255, 255);
    driveMotor(motorSpeed);
    lastPIDTime = millis();
  }

  // --- Perbarui Tampilan TFT ---
  // Kondisi untuk update TFT tidak perlu lagi cek server.hasClient() dengan AsyncWebServer
  if (abs(currentAngle - previousTotalAngle) > 0.1 || (millis() % 500 < pidInterval) ) {
    displayValues();
    previousTotalAngle = currentAngle;
  }

  // Debugging Serial Plotter
  Serial.print(targetAngle);
  Serial.print(" ");
  Serial.print(currentAngle);
  Serial.print(" ");
  Serial.println(outputPID); 
  
  // No delay here, AsyncWebServer needs loop to run fast
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

// --- Fungsi untuk menangani klik tombol Rotary Encoder ---
void handleRotaryButton() {
  bool currentSW = digitalRead(SW);
  if (lastSW == HIGH && currentSW == LOW && (millis() - lastButtonPress) > 250) { // Debounce
    lastButtonPress = millis();

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
  lastSW = currentSW;
}

// --- Fungsi-fungsi AS5600, Motor, PID (tidak berubah) ---
void checkMagnetPresence() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(10, 20);
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

    tft.setCursor(10, 50);
    tft.fillRect(10, 50, 100, 20, ST77XX_BLACK);
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
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(10, 20);
  tft.setTextColor(ST77XX_GREEN);
  tft.println("Magnet Found!");
  delay(1000);
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

void displayValues() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  
  // Tampilkan Mode Aktif
  tft.setCursor(5, 5);
  tft.setTextColor(ST77XX_WHITE);
  switch (currentMode) {
    case MODE_ANGLE: tft.print("Mode: Angle"); break;
    case MODE_KP:    tft.print("Mode: Kp");    break;
    case MODE_KI:    tft.print("Mode: Ki");    break;
    case MODE_KD:    tft.print("Mode: Kd");    break;
  }

  // Tampilkan Target
  tft.setCursor(10, 25);
  tft.setTextColor((currentMode == MODE_ANGLE) ? ST77XX_YELLOW : ST77XX_WHITE);
  tft.print("Target: ");
  tft.print(targetAngle, 1);
  tft.print(" deg");

  // Tampilkan Motor
  tft.setCursor(10, 40);
  tft.setTextColor(ST77XX_CYAN);
  tft.print("Motor : ");
  tft.print(currentAngle, 1);
  tft.print(" deg");

  // Tampilkan PID Parameter
  tft.setCursor(10, 70);
  tft.setTextColor((currentMode == MODE_KP) ? ST77XX_YELLOW : ST77XX_MAGENTA);
  tft.print("Kp: ");
  tft.print(Kp, 3);

  tft.setCursor(10, 85);
  tft.setTextColor((currentMode == MODE_KI) ? ST77XX_YELLOW : ST77XX_GREEN);
  tft.print("Ki: ");
  tft.print(Ki, 5);

  tft.setCursor(10, 100);
  tft.setTextColor((currentMode == MODE_KD) ? ST77XX_YELLOW : ST77XX_ORANGE);
  tft.print("Kd: ");
  tft.print(Kd, 3);

  // Tampilkan output PID
  tft.setCursor(10, 115);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("PID Out: ");
  tft.print(outputPID, 0);
}

// --- Implementasi Fungsi WebServer ---

void setupCaptivePortal() {
  Serial.print("Setting AP (Access Point)... ");
  WiFi.softAP(ssid, password);

  WiFi.softAPConfig(apIP, apIP, netMsk);

  tft.setCursor(10, 70);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("AP SSID: ");
  tft.println(ssid);
  tft.setCursor(10, 85);
  tft.print("IP: ");
  tft.println(WiFi.softAPIP());
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  // DNS Server untuk Captive Portal
  dnsServer.start(53, "*", apIP);

  // --- Handler untuk AsyncWebServer ---
  // Melayani file dari SPIFFS
  // Default handler untuk semua file di root atau subdirektori SPIFFS
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("halo.txt"); // Set halo.txt sebagai file default

  // Handler untuk root
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<!DOCTYPE html><html lang='en'><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>PID Tuner</title>";
    html += "<style>body{font-family:sans-serif; text-align:center; margin-top:50px;} button{padding:10px 20px; font-size:16px;}</style>";
    html += "</head><body><h1>Welcome to ESP32 PID Tuner!</h1><p>Connect to this Wi-Fi to access the tuning interface.</p>";
    html += "<p><a href='/halo.txt'><button>Go to Web Interface</button></a></p>";
    html += "</body></html>";
    request->send(200, "text/html", html);
  });

  // Handler untuk halaman tidak ditemukan (redirect ke captive portal)
  server.onNotFound([](AsyncWebServerRequest *request){
    request->redirect("http://" + apIP.toString());
  });
  
  server.begin(); // Mulai server HTTP Asynchronous
  Serial.println("HTTP server started (Async)");
  tft.setCursor(10, 100);
  tft.println("Web server started.");
  delay(1000);
  tft.fillScreen(ST77XX_BLACK);
}