#include <Wire.h>
#include "AS5600.h"

AS5600 as5600;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // ESP32 default I2C pins: SDA=21, SCL=22
  as5600.begin();
  as5600.setDirection(AS5600_CLOCK_WISE); 

  Serial.println("\n===== SKRIP KALIBRASI SEMENTARA AS5600 =====");

  if (as5600.isConnected()) {
    Serial.println("Sensor AS5600 terdeteksi.");
    Serial.println("1. Putar poros motor ke posisi NOL yang Anda inginkan.");
    Serial.println("2. Setelah poros diam di posisi tersebut, ketik 'c' lalu tekan Enter untuk mengkalibrasi.");
  } else {
    Serial.println("Sensor AS5600 tidak ditemukan! Periksa kembali koneksi.");
  }
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'c') {
      // Baca posisi mentah saat ini
      int currentPosition = as5600.rawAngle();

      // Atur posisi saat ini sebagai titik awal (zero) sementara
      as5600.setZPosition(currentPosition);

      Serial.println("\n==============================================");
      Serial.print("Kalibrasi sementara SELESAI. Posisi nol diatur pada nilai mentah: ");
      Serial.println(currentPosition);
      Serial.println("Pengaturan ini akan HILANG jika daya sensor dicabut.");
      Serial.println("\nSEKARANG, unggah kembali skrip pembacaan sudut untuk verifikasi.");
      Serial.println("==============================================");
    }
  }
}