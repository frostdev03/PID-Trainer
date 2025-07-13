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

// Fungsi bantuan untuk mendapatkan lebar teks (menggantikan textWidth() jika tidak ada)
// Ini adalah perkiraan berdasarkan ukuran teks 1 dan font default Adafruit GFX
int getTextWidth(String text, int textSize) {
  return text.length() * 6 * textSize; // 6 piksel per karakter (5px char + 1px spacing)
}

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
  totalAngle = 0;

  // Set target awal ke posisi motor saat ini
  targetAngle = readAS5600AngleCombined();
  currentAngle = targetAngle;

  lastPIDTime = millis();
}

void loop() {
  // --- Tangani WebServer dan DNS ---
  dnsServer.processNextRequest(); 
  
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
  if (abs(currentAngle - previousTotalAngle) > 0.1 || (millis() % 500 < pidInterval) || (currentMode == MODE_ANGLE && millis() % 2000 < pidInterval) ) {
    displayValues();
    previousTotalAngle = currentAngle;
  }

  // Debugging Serial Plotter
  Serial.print(targetAngle);
  Serial.print(" ");
  Serial.print(currentAngle);
  Serial.print(" ");
  Serial.println(outputPID); 
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
  Wire.requestFrom(AS5600_ADDR, 1); // Perbaikan: Pastikan AS5600_ADDR untuk requestFrom
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
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE); // Atur semua warna teks menjadi putih

  // Offset Y untuk memperkecil spasi antar baris
  int lineSpacing = 15; 
  int currentY = 5; // Posisi Y awal

  // Tampilkan Target & Motor (sejajar, align left)
  tft.setCursor(0, currentY);
  tft.print("Target:");
  tft.print(targetAngle, 1);
  tft.print("\xF8"); // Simbol derajat
  
  // Perbaiki align motor: Tulis "Motor:" dari posisi kiri, lalu nilai
  tft.setCursor(tft.width() / 2 + 5, currentY); // Mulai motor di tengah layar + sedikit offset
  tft.print("Motor:");
  tft.print(currentAngle, 1);
  tft.print("\xF8");
  currentY += lineSpacing;

  // Baris kedua: Error dan Output PID (sejajar, align left)
  tft.setCursor(0, currentY);
  tft.print("Error:");
  tft.print(error, 1);
  
  tft.setCursor(tft.width() / 2 + 5, currentY); // Mulai output PID di tengah layar + sedikit offset
  tft.print("Out:");
  tft.print(outputPID, 0);
  currentY += lineSpacing;

  // Spasi kosong sebelum PID Parameters
  currentY += lineSpacing / 2; // Sedikit spasi tambahan

  // Tampilkan PID Parameter (menurun ke bawah, align left, highlight jika mode aktif)
  tft.setCursor(0, currentY);
  tft.setTextColor((currentMode == MODE_KP) ? ST77XX_YELLOW : ST77XX_WHITE); // Warna highlight
  tft.print("Kp: ");
  tft.print(Kp, 3);
  currentY += lineSpacing;

  tft.setCursor(0, currentY);
  tft.setTextColor((currentMode == MODE_KI) ? ST77XX_YELLOW : ST77XX_WHITE);
  tft.print("Ki: ");
  tft.print(Ki, 5);
  currentY += lineSpacing;

  tft.setCursor(0, currentY);
  tft.setTextColor((currentMode == MODE_KD) ? ST77XX_YELLOW : ST77XX_WHITE);
  tft.print("Kd: ");
  tft.print(Kd, 3);
  currentY += lineSpacing;

  // Tampilkan informasi WiFi jika mode adalah MODE_ANGLE
  if (currentMode == MODE_ANGLE) {
    currentY += lineSpacing / 2; // Spasi tambahan sebelum info WiFi
    tft.setCursor(0, currentY);
    tft.setTextColor(ST77XX_WHITE);
    tft.print("SSID: ");
    tft.println(ssid);

    tft.setCursor(0, currentY + lineSpacing);
    tft.print("Pass: ");
    tft.println(password);

    tft.setCursor(0, currentY + lineSpacing * 2);
    tft.print("IP: ");
    tft.println(WiFi.softAPIP());
  } else {
    // Jika bukan MODE_ANGLE, kosongkan area di mana info WiFi biasanya berada
    // Ini penting agar info WiFi tidak "tertulis" di atas info PID saat berganti mode
    // Hapus area yang lebih besar untuk memastikan tidak ada sisa teks
    tft.fillRect(0, 115, tft.width(), tft.height() - 115, ST77XX_BLACK); 
  }
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
    request->send(404, "text/plain", "Not Found"); // Kirim 404 jika bukan redirect captive portal
  }
}