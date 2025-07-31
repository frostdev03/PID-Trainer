#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// TFT Pins (ST7735)
#define TFT_CS     5
#define TFT_DC     4
#define TFT_RST    2
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// Rotary Encoder Pins
#define CLK 26
#define DT  25
#define SW  33

// AS5600
#define AS5600_ADDRESS 0x36
#define SDA_PIN 21
#define SCL_PIN 22

// PID values
float Kp = 0.0, Ki = 0.0, Kd = 0.0;

// Rotary state
int lastStateCLK;
bool lastButtonState = HIGH;

enum PIDParam { KP, KI, KD };
PIDParam currentParam = KP;

// Angle data
float angleDeg = 0.0;

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  pinMode(SW, INPUT_PULLUP);
  lastStateCLK = digitalRead(CLK);

  Wire.begin(SDA_PIN, SCL_PIN);

  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);  // Lebih kecil biar muat semua
  tft.setTextColor(ST77XX_GREEN);

  displayValues();
}

void loop() {
  // Rotary Encoder Logic
  int currentStateCLK = digitalRead(CLK);
  if (currentStateCLK != lastStateCLK && currentStateCLK == HIGH) {
    int direction = digitalRead(DT) == LOW ? 1 : -1;

    switch (currentParam) {
      case KP: Kp += 0.1 * direction; if (Kp < 0) Kp = 0; break;
      case KI: Ki += 0.1 * direction; if (Ki < 0) Ki = 0; break;
      case KD: Kd += 0.1 * direction; if (Kd < 0) Kd = 0; break;
    }

    Serial.print("[ACTIVE: ");
    Serial.print(currentParam == KP ? "Kp" : currentParam == KI ? "Ki" : "Kd");
    Serial.print("] => Kp: "); Serial.print(Kp, 2);
    Serial.print(" | Ki: "); Serial.print(Ki, 2);
    Serial.print(" | Kd: "); Serial.println(Kd, 2);

    displayValues();
  }
  lastStateCLK = currentStateCLK;

  // Tombol Encoder
  bool currentButtonState = digitalRead(SW);
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    currentParam = (PIDParam)((currentParam + 1) % 3);
    Serial.print("==> Parameter aktif: ");
    Serial.println(currentParam == KP ? "Kp" : currentParam == KI ? "Ki" : "Kd");
    displayValues();
    delay(200); // debounce
  }
  lastButtonState = currentButtonState;

  // Baca AS5600 angle
  angleDeg = readAS5600Angle();
  displayAngle(angleDeg);

  delay(100);
}

float readAS5600Angle() {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(0x0C);  // MSB of raw_angle
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDRESS, 2);
  if (Wire.available() == 2) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    uint16_t raw = (msb << 8) | lsb;
    return raw * 0.08789;  // Konversi ke derajat
  }
  return -1.0;  // Jika gagal baca
}

void displayValues() {
  tft.fillScreen(ST77XX_BLACK);

  for (int i = 0; i < 3; i++) {
    tft.setCursor(5, 10 + i * 12);
    if (i == currentParam)
      tft.setTextColor(ST77XX_YELLOW);
    else
      tft.setTextColor(ST77XX_GREEN);

    const char* label = (i == 0) ? "Kp" : (i == 1) ? "Ki" : "Kd";
    float val = (i == 0) ? Kp : (i == 1) ? Ki : Kd;

    tft.print(label); tft.print(": ");
    tft.print(val, 2);
  }
}

void displayAngle(float deg) {
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);  // Warna berbeda, background dihapus
  tft.setCursor(5, 50);
  tft.print("Angle: ");
  tft.print(deg, 2);
  tft.print((char)247); // derajat
}
