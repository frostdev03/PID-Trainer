#include <Wire.h>

#define AS5600_ADDR 0x36
#define RAW_ANGLE_HIGH 0x0C
#define RAW_ANGLE_LOW  0x0D

#define FILTER_SIZE 10

float angleBuffer[FILTER_SIZE];
int bufferIndex = 0;

float zeroOffset = 0.0;
float previousAngle = 0.0;
float totalRotation = 0.0;

// === BACA SUDUT DARI AS5600 (0 - 4095) ===
int readRawAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(RAW_ANGLE_HIGH);
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() >= 2) {
    int highByte = Wire.read();
    int lowByte = Wire.read();
    return (highByte << 8) | lowByte;
  }
  return 0;
}

// === KONVERSI RAW KE DERAJAT (0° - 360°) ===
float readAngleDeg() {
  int raw = readRawAngle();
  return (raw * 360.0) / 4096.0;
}

// === FILTER SEDERHANA SMA ===
float filterAngle(float newAngle) {
  angleBuffer[bufferIndex] = newAngle;
  bufferIndex = (bufferIndex + 1) % FILTER_SIZE;

  float sum = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    sum += angleBuffer[i];
  }
  return sum / FILTER_SIZE;
}

// === SETUP ===
void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(500);
  Serial.println("AS5600 angle tracking initialized");
}

// === LOOP UTAMA ===
void loop() {
  float rawAngle = readAngleDeg();
  float filteredAngle = filterAngle(rawAngle);

  // === KALIBRASI 'cal' UNTUK JADIKAN POSISI SEKARANG SEBAGAI 0°
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "cal") {
      zeroOffset = filteredAngle;
      totalRotation = 0;
      Serial.println("Zero position calibrated.");
    }
  }

  // === HITUNG ANGLE RELATIF TERHADAP ZERO ===
  // float relativeAngle = filteredAngle - zeroOffset;
  // if (relativeAngle > 180)  relativeAngle -= 360;
  // if (relativeAngle < -180) relativeAngle += 360;

  // // === DETEKSI PUTARAN TERUSAN (wrap-around) ===
  // float delta = relativeAngle - previousAngle;

    // Gunakan angle asli untuk tracking rotasi total
  float currentAngle = filteredAngle;
  float delta = currentAngle - previousAngle;

  // Perbaiki saat wrap-around (cross 0° ↔ 360°)
  if (delta > 180)  delta -= 360;
  if (delta < -180) delta += 360;

  totalRotation += delta;
  previousAngle = currentAngle;

  // Untuk user: angle relatif ke posisi nol (setelah kalibrasi)
  float relativeAngle = currentAngle - zeroOffset;
  if (relativeAngle > 180)  relativeAngle -= 360;
  if (relativeAngle < -180) relativeAngle += 360;

  // if (delta > 180)  delta -= 360;
  // if (delta < -180) delta += 360;
  // totalRotation += delta;
  // previousAngle = relativeAngle;

  // === OUTPUT SERIAL ===
  Serial.print("Filtered Angle: ");
  Serial.print(relativeAngle, 2);
  Serial.print("°, Total: ");
  Serial.print(totalRotation, 2);
  Serial.println("°");

  delay(20); // 50 Hz sampling
}
