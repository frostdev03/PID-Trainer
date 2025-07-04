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

// Rotary Encoder Pins
#define CLK 25
#define DT  32

// L298N Motor Pins
#define IN1 14
#define IN2 27
#define ENA 25

int lastCLK = HIGH;
int targetAngle = 0;
int currentAngle = 0;

unsigned long lastMotorUpdate = 0;
const int motorSpeedFixed = 100; // PWM

void setup() {
  Serial.begin(115200);

  pinMode(CLK, INPUT_PULLUP);
  pinMode(DT, INPUT_PULLUP);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1); // landscape
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  displayValues();
}

void loop() {
  // Deteksi perubahan rotary encoder
  int currentCLK = digitalRead(CLK);
  if (currentCLK != lastCLK && currentCLK == LOW) {
    int dir = digitalRead(DT) == HIGH ? -1 : 1;
    targetAngle += dir;

    Serial.print("Target Angle: "); Serial.println(targetAngle);
    displayValues();
  }
  lastCLK = currentCLK;

  // Simulasi motor bergerak mendekati target
  if (millis() - lastMotorUpdate > 10) {
    lastMotorUpdate = millis();

    if (currentAngle < targetAngle) {
      currentAngle++;
      driveMotor(1);  // Motor ke kanan
    } else if (currentAngle > targetAngle) {
      currentAngle--;
      driveMotor(-1); // Motor ke kiri
    } else {
      driveMotor(0);  // Berhenti
    }

    displayValues();
  }
}

void driveMotor(int dir) {
  if (dir > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, motorSpeedFixed);
  } else if (dir < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, motorSpeedFixed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void displayValues() {
  tft.fillScreen(ST77XX_BLACK);
  
  tft.setCursor(10, 10);
  tft.setTextColor(ST77XX_YELLOW);
  tft.print("Target: ");
  tft.println(targetAngle);

  tft.setCursor(10, 30);
  tft.setTextColor(ST77XX_CYAN);
  tft.print("Actual: ");
  tft.println(currentAngle);

  tft.setCursor(10, 50);
  tft.setTextColor(ST77XX_MAGENTA);
  tft.print("Delta: ");
  tft.println(targetAngle - currentAngle);
}
