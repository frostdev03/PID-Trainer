#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// TFT Pins
#define TFT_CS     5
#define TFT_DC     4
#define TFT_RST    2
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// Rotary Pins
#define CLK 26
#define DT  25
#define SW  33

// PID values
float Kp = 0.0, Ki = 0.0, Kd = 0.0;

// Rotary state
int lastStateCLK;
bool lastButtonState = HIGH;

enum PIDParam { KP, KI, KD };
PIDParam currentParam = KP;

void setup() {
  Serial.begin(115200);
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  pinMode(SW, INPUT_PULLUP);
  
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
  if (currentStateCLK != lastStateCLK && currentStateCLK == HIGH) {
    int direction = digitalRead(DT) == LOW ? 1 : -1;

    switch (currentParam) {
      case KP: Kp += 0.1 * direction; if (Kp < 0) Kp = 0; break;
      case KI: Ki += 0.1 * direction; if (Ki < 0) Ki = 0; break;
      case KD: Kd += 0.1 * direction; if (Kd < 0) Kd = 0; break;
    }

    displayValues();

    Serial.print("[ACTIVE: ");
    Serial.print(currentParam == KP ? "Kp" : currentParam == KI ? "Ki" : "Kd");
    Serial.print("] => Kp: "); Serial.print(Kp, 2);
    Serial.print(" | Ki: "); Serial.print(Ki, 2);
    Serial.print(" | Kd: "); Serial.println(Kd, 2);
  }
  lastStateCLK = currentStateCLK;

  // Button Press to switch parameter
  bool currentButtonState = digitalRead(SW);
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    currentParam = (PIDParam)((currentParam + 1) % 3);
    displayValues();
    Serial.print("==> Parameter aktif: ");
    Serial.println(currentParam == KP ? "Kp" : currentParam == KI ? "Ki" : "Kd");
    delay(200); // debounce
  }
  lastButtonState = currentButtonState;

  delay(1);
}

void displayValues() {
  tft.fillScreen(ST77XX_BLACK);
  for (int i = 0; i < 3; i++) {
    tft.setCursor(10, 20 + i * 30);
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
}
