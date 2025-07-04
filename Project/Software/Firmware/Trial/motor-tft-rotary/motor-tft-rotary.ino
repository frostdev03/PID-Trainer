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
//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST, TFT_SCLK, TFT_MOSI);


// Rotary Pins
#define CLK 32
#define DT  25
#define SW  13

// Motor Driver BTS7960 Pins
//#define RPWM 26
//#define LPWM 33
//
//#define enr 14
//#define enl 27 

#define IN1 14
#define IN2 27
#define ENA 25

// PID values
float Kp = 0.0, Ki = 0.0, Kd = 0.0;

// Motor speed
int motorSpeed = 0;

// Rotary state
int lastStateCLK;
bool lastButtonState = HIGH;
int rotaryValue = 0;


enum PIDParam { KP, KI, KD };
PIDParam currentParam = KP;

void setup() {
  Serial.begin(115200);
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  pinMode(SW, INPUT_PULLUP);
//
//  pinMode(RPWM, OUTPUT);
//  pinMode(LPWM, OUTPUT);

pinMode(IN1, OUTPUT);
pinMode(IN2, OUTPUT);
pinMode(ENA, OUTPUT);

  
  lastStateCLK = digitalRead(CLK);

  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);

  displayValues();
}

void loop() {
  // Rotary Rotation
  int currentStateCLK = digitalRead(CLK);
//  
//  if (currentStateCLK != lastStateCLK && currentStateCLK == HIGH) {
//    int direction = digitalRead(DT) == LOW ? 1 : -1;
//    rotaryValue += direction;
//
//    // Ubah nilai parameter PID
//    switch (currentParam) {
//      case KP: Kp += 0.1 * direction; if (Kp < 0) Kp = 0; break;
//      case KI: Ki += 0.1 * direction; if (Ki < 0) Ki = 0; break;
//      case KD: Kd += 0.1 * direction; if (Kd < 0) Kd = 0; break;
//    }
//
//    // Ubah kecepatan motor berdasarkan arah putaran
//    motorSpeed += 20 * direction;
//    motorSpeed = constrain(motorSpeed, -255, 255);
//    driveMotor(motorSpeed);
//
//    displayValues();
//
//    Serial.print("[ACTIVE: ");
//    Serial.print(currentParam == KP ? "Kp" : currentParam == KI ? "Ki" : "Kd");
//    Serial.print("] => Kp: "); Serial.print(Kp, 2);
//    Serial.print(" | Ki: "); Serial.print(Ki, 2);
//    Serial.print(" | Kd: "); Serial.print(Kd, 2);
//    Serial.print(" | Motor: "); Serial.println(motorSpeed);
//    Serial.print(" | Rotary: "); Serial.println(rotaryValue);
//
//  }


  if (currentStateCLK != lastStateCLK && currentStateCLK == LOW) {
    int direction = digitalRead(DT) == HIGH ? 1 : -1;
    rotaryValue += direction;

    bool updated = false;

    // Ubah nilai parameter PID
    switch (currentParam) {
      case KP: if (Kp + 0.1 * direction >= 0) { Kp += 0.1 * direction; updated = true; } break;
      case KI: if (Ki + 0.1 * direction >= 0) { Ki += 0.1 * direction; updated = true; } break;
      case KD: if (Kd + 0.1 * direction >= 0) { Kd += 0.1 * direction; updated = true; } break;
    }

    // Ubah kecepatan motor berdasarkan arah putaran
    motorSpeed += 20 * direction;
    motorSpeed = constrain(motorSpeed, -255, 255);
    driveMotor(motorSpeed);
    updated = true; // tetap true karena motorSpeed berubah

    if (updated) {
      displayValues();
    }

    Serial.print("[ACTIVE: ");
    Serial.print(currentParam == KP ? "Kp" : currentParam == KI ? "Ki" : "Kd");
    Serial.print("] => Kp: "); Serial.print(Kp, 2);
    Serial.print(" | Ki: "); Serial.print(Ki, 2);
    Serial.print(" | Kd: "); Serial.print(Kd, 2);
    Serial.print(" | Motor: "); Serial.print(motorSpeed);
    Serial.print(" | Rotary: "); Serial.println(rotaryValue);
  }

  lastStateCLK = currentStateCLK;
    delay(1);

  // Button Press to switch parameter
  bool currentButtonState = digitalRead(SW);
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    currentParam = (PIDParam)((currentParam + 1) % 3);

    // Reset motor saat pindah parameter
    motorSpeed = 0;
    driveMotor(motorSpeed);

    displayValues();
    Serial.print("==> Parameter aktif: ");
    Serial.println(currentParam == KP ? "Kp" : currentParam == KI ? "Ki" : "Kd");
    delay(200); // debounce
  }
  lastButtonState = currentButtonState;

    // Kontrol via Serial
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'p') {
      motorSpeed += 20;
      motorSpeed = constrain(motorSpeed, -255, 255);
      driveMotor(motorSpeed);
      displayValues();
      Serial.println("Motor +");
    } else if (cmd == 'o') {
      motorSpeed -= 20;
      motorSpeed = constrain(motorSpeed, -255, 255);
      driveMotor(motorSpeed);
      displayValues();
      Serial.println("Motor -");
    }
  }


  delay(1);
}

// Fungsi untuk mengatur arah dan kecepatan motor DC
//void driveMotor(int speed) {
//  speed = constrain(speed, -255, 255);
//  if (speed > 0) {
//    analogWrite(RPWM, speed);
//    analogWrite(LPWM, 0);
//  } else if (speed < 0) {
//    analogWrite(RPWM, 0);
//    analogWrite(LPWM, -speed);
//  } else {
//    analogWrite(RPWM, 0);
//    analogWrite(LPWM, 0);
//  }
//}

void driveMotor(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speed);
  } else if (speed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -speed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

//
//void displayValues() {
//  tft.fillScreen(ST77XX_BLACK);
//  for (int i = 0; i < 3; i++) {
//    tft.setCursor(10, 20 + i * 30);
//    if (i == currentParam)
//      tft.setTextColor(ST77XX_YELLOW);
//    else
//      tft.setTextColor(ST77XX_GREEN);
//
//    const char* label = (i == 0) ? "Kp" : (i == 1) ? "Ki" : "Kd";
//    float val = (i == 0) ? Kp : (i == 1) ? Ki : Kd;
//
//    tft.print(label);
//    tft.print(": ");
//    tft.print(val, 2);
//  }
//
//  // Tampilkan kecepatan motor
//  tft.setCursor(10, 120); // posisi baris bawah
//  tft.setTextColor(ST77XX_CYAN);
//  tft.setTextSize(1);
//  tft.print("Motor Speed: ");
//  tft.print(motorSpeed);
//  tft.setCursor(10, 140);
//  tft.setTextColor(ST77XX_MAGENTA);
//  tft.setTextSize(1);
//  tft.print("Rotary Raw: ");
//  tft.print(rotaryValue);
//
//}

void displayValues() {
  tft.fillScreen(ST77XX_BLACK);

  // Header PID
  tft.setTextSize(2);
  for (int i = 0; i < 3; i++) {
    tft.setCursor(10, 10 + i * 25);  // Ukuran 2, tiap line kira-kira 25px
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

  // Info tambahan: motor & rotary
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(10, 90);  // Start setelah 3 baris besar
  tft.print("Motor Speed: ");
  tft.print(motorSpeed);

  tft.setCursor(10, 105);
  tft.setTextColor(ST77XX_MAGENTA);
  tft.print("Rotary Raw: ");
  tft.print(rotaryValue);
}



//void displayValues() {
//  tft.fillScreen(ST77XX_BLACK);
//  for (int i = 0; i < 3; i++) {
//    tft.setCursor(10, 20 + i * 30);
//    if (i == currentParam)
//      tft.setTextColor(ST77XX_YELLOW);
//    else
//      tft.setTextColor(ST77XX_GREEN);
//
//    const char* label = (i == 0) ? "Kp" : (i == 1) ? "Ki" : "Kd";
//    float val = (i == 0) ? Kp : (i == 1) ? Ki : Kd;
//
//    tft.print(label);
//    tft.print(": ");
//    tft.print(val, 2);
//  }
//}
