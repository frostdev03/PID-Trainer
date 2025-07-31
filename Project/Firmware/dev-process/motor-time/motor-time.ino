#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// === Pin TFT ===
#define TFT_CS    5
#define TFT_RST   4
#define TFT_DC    15
#define TFT_SCLK   18
#define TFT_MOSI   23
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// === Pin Motor & Tombol ===
#define IN1 14
#define IN2 27
#define ENA 12

#define CLK 32
#define DT  25
#define SW  13

// ==========================================================
// === PENGATURAN & ASUMSI (SESUAIKAN DI SINI!) ===
// ==========================================================
const int ENCODER_PPR = 24;           // Asumsi resolusi encoder Anda
const long MOTOR_TIME_FOR_360 = 1500; // ASUMSI: Waktu motor berputar 360° (dalam ms)
const int motorSpeed = 0;           // Kecepatan motor saat bergerak (0-255)

// === Variabel dihitung otomatis, jangan diubah ===
const float DEGREES_PER_TICK = 360.0 / ENCODER_PPR;
const long TIME_PER_TICK = MOTOR_TIME_FOR_360 / ENCODER_PPR;

// === Variabel State ===
long targetTicks = 0;   
long motorTicks = 0;    
int lastStateCLK;
bool lastButtonState = HIGH;

void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(CLK, INPUT); pinMode(DT, INPUT); pinMode(SW, INPUT_PULLUP);
  
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);

  lastStateCLK = digitalRead(CLK);
  displayValues();
}

void loop() {
  // 1. Baca Rotary Encoder untuk mengubah target
  int currentStateCLK = digitalRead(CLK);
  if (currentStateCLK != lastStateCLK && currentStateCLK == HIGH) {
    targetTicks += (digitalRead(DT) == LOW) ? 1 : -1;
    displayValues();
  }
  lastStateCLK = currentStateCLK;

  // 2. Baca tombol untuk reset
  bool currentButtonState = digitalRead(SW);
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    targetTicks = 0;
    motorTicks = 0;
    driveMotor(0);
    displayValues();
    delay(200);
  }
  lastButtonState = currentButtonState;

  // 3. Logika utama: Gerakkan motor untuk mengejar target
  if (motorTicks < targetTicks) {
    driveMotor(motorSpeed);
    delay(TIME_PER_TICK);
    driveMotor(0); // Berhenti sejenak untuk presisi
    motorTicks++;
    displayValues();
  } else if (motorTicks > targetTicks) {
    driveMotor(-motorSpeed);
    delay(TIME_PER_TICK);
    driveMotor(0);
    motorTicks--;
    displayValues();
  }
}

void driveMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    analogWrite(ENA, speed);
  } else if (speed < 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    analogWrite(ENA, -speed);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void displayValues() {
  float targetDisplayAngle = targetTicks * DEGREES_PER_TICK;
  float motorDisplayAngle = motorTicks * DEGREES_PER_TICK;

  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(10, 20);
  tft.setTextSize(2);
  
  tft.setTextColor(ST77XX_CYAN);
  tft.print("Target: ");
  tft.print(targetDisplayAngle, 1);
  tft.print(" deg");

  tft.setCursor(10, 50);
  tft.setTextColor(ST77XX_YELLOW);
  tft.print("Motor : ");
  tft.print(motorDisplayAngle, 1);
  tft.print(" deg");
}
