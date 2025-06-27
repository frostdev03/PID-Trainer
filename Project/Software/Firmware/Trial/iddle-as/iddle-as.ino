#include <Wire.h>
#include <AS5600.h>

AS5600 as5600;

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Untuk ESP32, pin default adalah GPIO 21 (SDA) dan 22 (SCL)

  as5600.begin();
  if (!as5600.isConnected()) {
    Serial.println("Error: Sensor AS5600 tidak terdeteksi! Periksa koneksi.");
    while (1);
  }
  Serial.println("Sensor AS5600 terdeteksi. Putar poros motor untuk melihat perubahan nilai.");
}

void loop() {
  // Baca sudut mentah (0-4095)
  uint16_t rawPosition = as5600.rawAngle();

  Serial.print("Posisi Sudut Mentah: ");
  Serial.println(rawPosition);
  Serial.print(" || Derajat: ");
  Serial.print(rawPosition)

  delay(100); 
}