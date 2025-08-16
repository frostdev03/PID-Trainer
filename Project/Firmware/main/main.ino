#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include <SPIFFS.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

#include "KalmanFilter.h"
KalmanFilter kalman;

#define TFT_CS 5
#define TFT_RST 4
#define TFT_DC 15
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

#define AS5600_ADDR 0x36

#define AS5600_SDA_PIN 16
#define AS5600_SCL_PIN 17

#define CLK 32
#define DT 25
#define SW 13

// #define RPWM 2
// #define LPWM 0
// #define REN 27
// #define LEN 14

#define LEN 21
#define REN 22
#define LPWM 27
#define RPWM 26

int magnetStatus = 0;

float targetAngle = 0;
float currentAngle = 0;
int motorSpeed = 0;

bool needFullRefresh = true;
bool isEditMode = false;

int lastCLK = HIGH;
int lastDT = HIGH;
bool lastSW = HIGH;
unsigned long lastEncoderTime = 0;
unsigned long lastButtonTime = 0;
unsigned long buttonPressStartTime = 0;
const unsigned long ENCODER_DEBOUNCE_MS = 2;
const unsigned long BUTTON_DEBOUNCE_MS = 50;
const unsigned long HOLD_DURATION = 1500;
bool isCalibrating = false;
bool buttonWasHeld = false;

float rotaryStep = 0.01;

float Kp = 0.3000;
float Ki = 0.00010;
float Kd = 0.001000;
float dt = 0;
float error = 0;
float previousError = 0;
float integral = 0;
float outputPID = 0;
float derivative = 0;

float offsetFromZpos = 108;
// float offsetFromZpos = 69.69;

bool pidEnabled = true;
unsigned long startTime = 0;
unsigned long lastPIDTime = 0;
int pidInterval = 10;

const float CW_LIMIT = 35.0;
const float CCW_LIMIT = -35.0;

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

enum RotaryMode {
  MODE_KP,
  MODE_KI,
  MODE_KD
};
RotaryMode currentMode = MODE_KP;

RotaryMode lastMode = MODE_KP;
bool shouldUpdateTFT = true;

const char *ssid = "PID Trainer";
const char *password = "motordc1";
IPAddress apIP(192, 168, 4, 1);
IPAddress netMsk(255, 255, 255, 0);

DNSServer dnsServer;
AsyncWebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

void checkMagnetPresence();
float readAngle();
void calculatePID();
void driveMotor(int speed);
void displayValues();
void handleRotaryEncoder();
void handleRotaryButton();
void performParameterReset();
float readAngle();
float getCalibratedAngle();
float normalizeAngle(float angle);
void handleRoot(AsyncWebServerRequest *request);
void handleNotFound(AsyncWebServerRequest *request);
void setupCaptivePortal();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
void sendPIDDataToWeb();
void checkTFTUpdate();

int getTextWidth(String text, int textSize) {
  return text.length() * 6 * textSize;
}

void testMotor() {
  Serial.println("Testing motor - Forward");
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);
  analogWrite(RPWM, 50);
  analogWrite(LPWM, 0);
  delay(2000);

  Serial.println("Testing motor - Backward");
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 50);
  delay(2000);

  Serial.println("Testing motor - Stop");
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  delay(1000);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(AS5600_SDA_PIN, AS5600_SCL_PIN);
  Wire.setClock(100000);  // TODO: CHECK EFFECT

  pinMode(CLK, INPUT_PULLUP);
  pinMode(DT, INPUT_PULLUP);
  pinMode(SW, INPUT_PULLUP);
  lastSW = digitalRead(SW);

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);
  // analogWrite(RPWM, 0);
  // analogWrite(LPWM, 0);

  // testMotor();

  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, tft.height() / 2 - 7);
  tft.println("PID Trainer Initializing...");

  if (!SPIFFS.begin(true)) {
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(10, tft.height() / 2 - 7);
    tft.setTextColor(ST77XX_RED);
    tft.println("SPIFFS Mount FAILED!");
    Serial.println("SPIFFS Mount Failed");
    while (true)
      ;
  }
  Serial.println("SPIFFS Mounted Successfully");

  setupCaptivePortal();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  checkMagnetPresence();
  currentAngle = readAngle();
  kalman.setAngle(readAngle());  // set initial angle

  // offsetFromZpos = currentAngle;
  lastPIDTime = millis();
}

void loop() {

  dnsServer.processNextRequest();
  webSocket.loop();

  if (!isCalibrating) {
    handleRotaryEncoder();
    handleRotaryButton();

    // float measuredAngle = readAngle();
    dt = pidInterval / 1000.0;

    // currentAngle = measuredAngle;
    unsigned long now = millis();
    if (now - lastPIDTime >= pidInterval) {
      float dt = (now - lastPIDTime) / 1000.0;

      // currentAngle = normalizeAngle(readAngle());
      currentAngle = getCalibratedAngle();
      // currentAngle = normalizeAngle(getFilteredAngle() - offsetFromZpos);

      calculatePID();

      // motorSpeed = constrain((int)outputPID, -255, 255);
      motorSpeed = outputPID;
      Serial.print("About to call driveMotor with: ");
      Serial.println(motorSpeed);

      driveMotor(motorSpeed);

      lastPIDTime = now;
      sendPIDDataToWeb();
    }

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
    }

    if (!isEditMode) {
      Serial.println(targetAngle);
      // Serial.println(readAngle());
      Serial.println(currentAngle);
      // Serial.println(motorSpeed);
      // Serial.println(normalizeAngle(readAngle()));
    }
  } else {
    performParameterReset();
  }
}

float readAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0E);  // MSB PROCESSED ANGLE register
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() < 2) return 0;  // fallback bila gagal

  int high = Wire.read();
  int low = Wire.read();
  int rawAngle = (high << 8) | low;

  return rawAngle * 0.087890625;
  // return rawAngle;
}

float normalizeAngle(float angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle <= -180.0) angle += 360.0;
  return angle;
}



void calculatePID() {

  // if (currentAngle > 180) currentAngle -= 360;
  // if (currentAngle < -180) currentAngle += 360;

  error = targetAngle - currentAngle;
  error = normalizeAngle(error);

  if (abs(error) < 0.5) {
    error = 0;
  }

  integral += error * dt;
  // integral = constrain(integral, -100, 100);

  derivative = (error - previousError) / dt;

  outputPID = (Kp * error) + (Ki * integral) + (Kd * derivative);

  Serial.print("Error: ");
  Serial.print(error);
  Serial.print(", PID Output: ");
  Serial.print(outputPID);
  Serial.print(", Motor Speed: ");
  Serial.println(motorSpeed);

  previousError = error;
}

// void driveMotor(int speed) {
//   speed = constrain(speed, -255, 255); // jaga tetap di range
//   digitalWrite(REN, HIGH);
//   digitalWrite(LEN, HIGH);

//   if (speed > 0) {
//     analogWrite(RPWM, speed);
//     analogWrite(LPWM, 0);
//   } else if (speed < 0) {
//     analogWrite(RPWM, 0);
//     analogWrite(LPWM, -speed);
//   } else {
//     analogWrite(RPWM, 0);
//     analogWrite(LPWM, 0);
//   }
// }

// void driveMotor(int speed) {
//   speed = constrain(speed, -255, 255);

//   if (speed > 0) {
//     digitalWrite(REN, HIGH);
//     digitalWrite(LEN, HIGH);
//     analogWrite(RPWM, speed);  // CCW = REN, LEN, RPWM == HIGH
//   } else {
//     digitalWrite(REN, HIGH);
//     digitalWrite(LEN, HIGH);
//     analogWrite(LPWM, -speed);  // CW = REN, LEN, LPWM == HIGH
//   }
// }
void driveMotor(int speed) {
  speed = constrain(speed, -255, 255);

  // float limitFactor = 1.0;

  // if (currentAngle > CW_LIMIT - 5.0 && speed > 0) {
  //   // Approaching CW limit, slow down
  //   limitFactor = (CW_LIMIT - currentAngle) / 5.0;
  //   limitFactor = constrain(limitFactor, 0.0, 1.0);
  // } else if (currentAngle < CCW_LIMIT + 5.0 && speed < 0) {
  //   // Approaching CCW limit, slow down
  //   limitFactor = (currentAngle - CCW_LIMIT) / 5.0;
  //   limitFactor = constrain(limitFactor, 0.0, 1.0);
  // }

  // HARD LIMITS - complete stop
  bool outOfCWLimit = (currentAngle >= CW_LIMIT) && (speed > 0);
  bool outOfCCWLimit = (currentAngle <= CCW_LIMIT) && (speed < 0);

  if (outOfCWLimit || outOfCCWLimit) {
    speed = 0;
    // } else {
    //   speed = (int)(speed * limitFactor);  // apply soft limit
  }

  delay(10);
  // Debug print
  Serial.print("driveMotor called with speed: ");
  Serial.println(speed);

  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);

  // Debug enable pins
  Serial.print("REN: ");
  Serial.print(digitalRead(REN));
  Serial.print(", LEN: ");
  Serial.println(digitalRead(LEN));

  if (speed > 0) {
    analogWrite(RPWM, speed);
    analogWrite(LPWM, 0);
    Serial.print("Setting RPWM=");
    Serial.print(speed);
    Serial.println(", LPWM=0");
  } else if (speed < 0) {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, -speed);
    Serial.print("Setting RPWM=0, LPWM=");
    Serial.println(-speed);
  } else {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
    Serial.println("Setting RPWM=0, LPWM=0");
  }
}

float getCalibratedAngle() {
  return normalizeAngle(readAngle() - offsetFromZpos);
}

// Ganti fungsi getCalibratedAngle():
// float getCalibratedAngle() {
//   static unsigned long lastKalmanTime = 0;
//   static float lastFilteredAngle = 0;

//   unsigned long now = millis();
//   float dt = (now - lastKalmanTime) / 1000.0;

//   if (lastKalmanTime == 0) {
//     dt = 0.01; // first run
//     lastKalmanTime = now;
//   }

//   float rawAngle = readAngle();

//   // Calculate rate (change in angle)
//   float angleRate = 0;
//   if (dt > 0) {
//     float normalizedRaw = normalizeAngle(rawAngle - offsetFromZpos);
//     float normalizedLast = normalizeAngle(lastFilteredAngle);
//     angleRate = normalizeAngle(normalizedRaw - normalizedLast) / dt;
//   }

//   // Apply Kalman filter
//   float filteredAngle = kalman.getAngle(
//     normalizeAngle(rawAngle - offsetFromZpos),
//     angleRate,
//     dt
//   );

//   lastFilteredAngle = filteredAngle;
//   lastKalmanTime = now;

//   return filteredAngle;
// }

void handleRotaryEncoder() {
  unsigned long now = millis();

  if (now - lastEncoderTime < ENCODER_DEBOUNCE_MS) {
    return;
  }

  int currentCLK = digitalRead(CLK);
  int currentDT = digitalRead(DT);

  if (currentCLK != lastCLK) {
    lastEncoderTime = now;

    if (currentCLK == LOW) {
      int direction;
      if (currentDT != currentCLK) {
        direction = 1;
      } else {
        direction = -1;
      }

      switch (currentMode) {
        case MODE_KP:
          Kp += direction * 0.1;
          if (Kp < 0) Kp = 0;
          if (Kp > 1.0) Kp = 1.0;
          break;
        case MODE_KI:
          Ki += direction * 0.0001;
          if (Ki < 0) Ki = 0;
          break;
        case MODE_KD:
          Kd += direction * 0.001;
          if (Kd < 0) Kd = 0;
          break;
      }
      shouldUpdateTFT = true;

      sendPIDDataToWeb();
    }
    lastCLK = currentCLK;
    lastDT = currentDT;
  }
}

void handleRotaryButton() {
  unsigned long now = millis();
  bool currentSW = digitalRead(SW);

  if (currentSW != lastSW && (now - lastButtonTime > BUTTON_DEBOUNCE_MS)) {
    lastButtonTime = now;

    if (currentSW == LOW && lastSW == HIGH) {

      buttonPressStartTime = now;
      buttonWasHeld = false;
      Serial.println("Button pressed");
    } else if (currentSW == HIGH && lastSW == LOW) {

      unsigned long holdDuration = now - buttonPressStartTime;

      if (holdDuration >= HOLD_DURATION) {

        Serial.println("Button released after hold");
        offsetFromZpos = readAngle();
        integral = 0;
      } else if (holdDuration > BUTTON_DEBOUNCE_MS) {

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
            integral = 0;
            Serial.println("Mode: Kp");
            break;
        }
        shouldUpdateTFT = true;
        sendPIDDataToWeb();
      }
      buttonPressStartTime = 0;
    }
  }

  if (currentSW == LOW && !buttonWasHeld && buttonPressStartTime > 0) {
    if (now - buttonPressStartTime >= HOLD_DURATION) {

      isCalibrating = true;
      buttonWasHeld = true;
      Serial.println("Starting Parameter Reset!");
      // This is the correct way to set the offset based on the current physical position
      // offsetFromZpos = readAngle();

      // // Reset the integral to prevent sudden movement after calibration
      // integral = 0;
      tft.fillScreen(ST77XX_BLACK);
      tft.setCursor(tft.width() / 2 - getTextWidth("Resetting...", 1) / 2, tft.height() / 2 - 20);
      tft.setTextColor(ST77XX_YELLOW);
      tft.println("Resetting...");
      tft.drawRect(tft.width() / 2 - 50, tft.height() / 2, 100, 10, ST77XX_WHITE);
      analogWrite(RPWM, 0);
      analogWrite(LPWM, 0);
      sendPIDDataToWeb();
    }
  }

  lastSW = currentSW;
}

// void handleRotaryButton() {
//   unsigned long now = millis();
//   bool currentSW = digitalRead(SW);

//   // Check for button state change with debounce
//   if (currentSW != lastSW && (now - lastButtonTime > BUTTON_DEBOUNCE_MS)) {

//     lastButtonTime = now;

//     if (currentSW == LOW) {
//       // Button pressed (LOW state)
//       buttonPressStartTime = now;
//       Serial.println("Button pressed");
//     } else {
//       // Button released (HIGH state)
//       unsigned long holdDuration = now - buttonPressStartTime;
//       Serial.println("Button released");

//       // Check if it was a short press
//       if (holdDuration > BUTTON_DEBOUNCE_MS && holdDuration < HOLD_DURATION) {
//         // Short press logic: Cycle through modes (Kp, Ki, Kd)
//         switch (currentMode) {
//           case MODE_KP:
//             currentMode = MODE_KI;
//             Serial.println("Mode: Ki");
//             break;
//           case MODE_KI:
//             currentMode = MODE_KD;
//             Serial.println("Mode: Kd");
//             break;
//           case MODE_KD:
//             currentMode = MODE_KP;
//             integral = 0;
//             Serial.println("Mode: Kp");
//             break;
//         }
//         shouldUpdateTFT = true;
//         sendPIDDataToWeb();
//       }
//       // Check if it was a long press
//       else if (holdDuration >= HOLD_DURATION) {
//         // Long press logic: Recalibrate the zero position
//         Serial.println("Long press detected. Recalibrating zero position.");

//         // This is the correct way to set the offset based on the current physical position
//         offsetFromZpos = readAngle();

//         // Reset the integral to prevent sudden movement after calibration
//         integral = 0;

//         // Add any additional actions for a long press, e.g., display a message
//         tft.fillScreen(ST77XX_BLACK);
//         tft.setCursor(tft.width() / 2 - getTextWidth("Calibrated!", 1) / 2, tft.height() / 2 - 20);
//         tft.setTextColor(ST77XX_GREEN);
//         tft.println("Calibrated!");
//         delay(1000); // Display message for a second
//         needFullRefresh = true; // Tell the display loop to redraw everything
//       }

//       buttonPressStartTime = 0; // Reset the timer
//     }
//   }

//   // Update the last button state
//   lastSW = currentSW;
// }

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

  int newProgressBarWidth = (int)(progress * 96);
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

    needFullRefresh = false;
  }

  int paddingX = 5;
  int currentY = 5;
  int lineSpacingSmall = 12;
  int clearWidth = 120;

  tft.setTextSize(1);
  tft.setCursor(paddingX, currentY);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("TARGET: ");
  tft.print(targetAngle, 0);
  tft.print(" DEG");
  currentY += lineSpacingSmall;

  if (currentAngle != lastDisplayAngle) {
    tft.fillRect(paddingX, currentY, clearWidth, 10, ST77XX_BLACK);

    tft.setTextSize(1);
    tft.setCursor(paddingX, currentY);
    tft.setTextColor(ST77XX_WHITE);
    tft.print("MOTOR: ");
    tft.print(currentAngle, 0);

    tft.print(" DEG");
    lastDisplayAngle = currentAngle;
  }
  currentY += lineSpacingSmall;

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

  if (abs(Kp - lastKp) > 0.001) {
    tft.fillRect(paddingX, currentY, clearWidth, 10, ST77XX_BLACK);
    tft.setTextSize(1);
    tft.setCursor(paddingX, currentY);
    tft.setTextColor((currentMode == MODE_KP) ? ST77XX_YELLOW : ST77XX_WHITE);
    tft.print("Kp: ");
    tft.print(Kp, 6);

    lastKp = Kp;
  }
  currentY += lineSpacingSmall;

  if (abs(Ki - lastKi) > 0.00001) {
    tft.fillRect(paddingX, currentY, clearWidth, 10, ST77XX_BLACK);

    tft.setTextSize(1);
    tft.setCursor(paddingX, currentY);
    tft.setTextColor((currentMode == MODE_KI) ? ST77XX_YELLOW : ST77XX_WHITE);
    tft.print("Ki: ");
    tft.print(Ki, 6);

    lastKi = Ki;
  }
  currentY += lineSpacingSmall;

  if (abs(Kd - lastKd) > 0.001) {
    tft.fillRect(paddingX, currentY, clearWidth, 10, ST77XX_BLACK);

    tft.setTextSize(1);
    tft.setCursor(paddingX, currentY);
    tft.setTextColor((currentMode == MODE_KD) ? ST77XX_YELLOW : ST77XX_WHITE);
    tft.print("Kd: ");
    tft.print(Kd, 6);

    lastKd = Kd;
  }
  currentY += lineSpacingSmall + 5;

  tft.setTextSize(1);
  tft.setCursor(paddingX, currentY);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("IP: ");
  tft.print(WiFi.softAPIP());
}

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
    doc["time_s"] = millis() / 1000.0;

    String jsonString;
    serializeJson(doc, jsonString);
    webSocket.broadcastTXT(jsonString);
  }
}