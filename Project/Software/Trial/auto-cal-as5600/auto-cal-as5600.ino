#include <Wire.h>

#define AS5600_ADDRESS 0x36  // Alamat I2C default AS5600

uint16_t zero_offset = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);  // SDA = 21, SCL = 22
  Serial.println("Cek sensor AS5600...");

  delay(500);  // Tunggu posisi diam
  zero_offset = readAS5600RawAngle();  // Simpan posisi awal sebagai nol
  Serial.print("Offset awal disimpan: ");
  Serial.println(zero_offset);
}

void loop() {
  uint16_t rawAngle = readAS5600RawAngle();

  // Koreksi sudut terhadap offset
  int16_t corrected = rawAngle - zero_offset;
  if (corrected < 0) corrected += 4096;

  float degAngle = corrected * 0.08789; // 360 / 4096

  Serial.print("Raw: ");
  Serial.print(rawAngle);
  Serial.print(" | Terkoreksi: ");
  Serial.print(corrected);
  Serial.print(" | Derajat: ");
  Serial.println(degAngle, 2);

  delay(500);
}

// Fungsi baca sudut mentah AS5600
uint16_t readAS5600RawAngle() {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(0x0C);  // Register MSB raw_angle
  Wire.endTransmission(false);

  Wire.requestFrom(AS5600_ADDRESS, 2);
  if (Wire.available() == 2) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    return ((msb << 8) | lsb);
  } else {
    Serial.println("Gagal baca sensor!");
    return 0;
  }
}