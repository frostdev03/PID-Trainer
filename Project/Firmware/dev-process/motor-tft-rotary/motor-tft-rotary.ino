#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// TFT Pins
#define TFT_CS     5
#define TFT_RST    4
#define TFT_DC     15
#define TFT_SCLK   18
#define TFT_MOSI   23
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// Rotary Pins
#define CLK 32
#define DT  25
#define SW  13

// BTS7960 Pins
#define RPWM 2
#define LPWM 0
#define REN 27
#define LEN 14

// PID values
float Kp = 0.0, Ki = 0.0, Kd = 0.0;
int motorSpeed = 0;

int lastStateCLK;
bool lastButtonState = HIGH;
int rotaryValue = 0;

enum PIDParam { KP, KI, KD };
PIDParam currentParam = KP;

void setup() {
  Serial.begin(115200);

  // Rotary
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  pinMode(SW, INPUT_PULLUP);
  lastStateCLK = digitalRead(CLK);

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);

  tft.initR(INITR_BLACKTAB);  
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);

  displayValues();
}

void loop() {
  int currentStateCLK = digitalRead(CLK);
  if (currentStateCLK != lastStateCLK && currentStateCLK == LOW) {
    int direction = digitalRead(DT) == HIGH ? 1 : -1;
    rotaryValue += direction;

    bool updated = false;
    switch (currentParam) {
      case KP: if (Kp + 0.1 * direction >= 0) { Kp += 0.1 * direction; updated = true; } break;
      case KI: if (Ki + 0.1 * direction >= 0) { Ki += 0.1 * direction; updated = true; } break;
      case KD: if (Kd + 0.1 * direction >= 0) { Kd += 0.1 * direction; updated = true; } break;
    }

    motorSpeed += 20 * direction;
    motorSpeed = constrain(motorSpeed, -255, 255);
    driveMotor(motorSpeed);
    updated = true;

    if (updated) {
      displayValues();
    }

    Serial.print("[ACTIVE: ");
    Serial.print(currentParam == KP ? "Kp" : currentParam == KI ? "Ki" : "Kd");
    Serial.print("] => Kp: "); Serial.print(Kp, 2);
    Serial.print(" | Ki: "); Serial.print(Ki, 2);
    Serial.print(" | Kd: "); Serial.print(Kd, 2);
    Serial.print(" | Motor: "); Serial.println(motorSpeed);
  }
  lastStateCLK = currentStateCLK;

  bool currentButtonState = digitalRead(SW);
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    currentParam = (PIDParam)((currentParam + 1) % 3);
    motorSpeed = 0;
    driveMotor(0);
    displayValues();

    Serial.print("==> Parameter aktif: ");
    Serial.println(currentParam == KP ? "Kp" : currentParam == KI ? "Ki" : "Kd");

    delay(200);
  }
  lastButtonState = currentButtonState;

  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'p') {
      motorSpeed += 20;
    } else if (cmd == 'o') {
      motorSpeed -= 20;
    }
    motorSpeed = constrain(motorSpeed, -255, 255);
    driveMotor(motorSpeed);
    displayValues();
  }

  delay(1);
}

void driveMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    analogWrite(RPWM, speed);
    analogWrite(LPWM, 0);
  } else if (speed < 0) {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, -speed);
  } else {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
  }
}

void displayValues() {
  tft.fillScreen(ST77XX_BLACK);

  // Header PID
  tft.setTextSize(2);
  for (int i = 0; i < 3; i++) {
    tft.setCursor(10, 10 + i * 25);  
    if (i == currentParam)
      tft.setTextColor(ST77XX_YELLOW);
    else
      tft.setTextColor(ST77XX_GREEN);

    const char* label = (i == 0) ? "Kp" : (i == 1) ? "Ki" : "Kd";
    float val = (i == 0) ? Kp : (i == 1) ? Ki : Kd;

    tft.print(label);
    tft.print(": ");
    tft.print(val, 2);
  }

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(10, 90);  
  tft.print("Motor Speed: ");
  tft.print(motorSpeed);

  tft.setCursor(10, 105);
  tft.setTextColor(ST77XX_MAGENTA);
  tft.print("Rotary Raw: ");
  tft.print(rotaryValue);
}
