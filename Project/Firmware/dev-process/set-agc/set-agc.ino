#include <Wire.h>

#define SDA_PIN 21  // ganti sesuai koneksi kamu
#define SCL_PIN 22
#define AS5600_ADDRESS 0x36
#define AGC_REGISTER 0x1A

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);  // setup I2C untuk ESP32
}

void loop() {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(AGC_REGISTER);
  Wire.endTransmission();

  Wire.requestFrom(AS5600_ADDRESS, 1);
  if (Wire.available()) {
    uint8_t agc = Wire.read();
    Serial.print("AGC: ");
    Serial.println(agc);
  }

  delay(200);
}
