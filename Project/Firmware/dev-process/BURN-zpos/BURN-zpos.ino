// Program debug untuk memeriksa dan memperbaiki ZPOS burn
// Upload program ini untuk diagnosa masalah

#include <Wire.h>

#define AS5600_ADDR 0x36
#define AS5600_SDA_PIN 16
#define AS5600_SCL_PIN 17

// Register addresses
#define AS5600_RAW_ANGLE_HIGH 0x0C
#define AS5600_RAW_ANGLE_LOW 0x0D
#define AS5600_ANGLE_HIGH 0x0E  // Processed angle (with ZPOS applied)
#define AS5600_ANGLE_LOW 0x0F
#define AS5600_ZPOS_HIGH 0x01
#define AS5600_ZPOS_LOW 0x02
#define AS5600_MPOS_HIGH 0x03
#define AS5600_MPOS_LOW 0x04
#define AS5600_MANG_HIGH 0x05
#define AS5600_MANG_LOW 0x06
#define AS5600_STATUS 0x0B
#define AS5600_BURN 0xFF

void setup() {
  Serial.begin(115200);
  Wire.begin(AS5600_SDA_PIN, AS5600_SCL_PIN);
  Wire.setClock(100000);
  
  Serial.println("=== AS5600 BURN DIAGNOSTIC ===");
  delay(1000);
  
  // Diagnostic lengkap
  diagnosticAS5600();
  
  Serial.println("\nPilihan:");
  Serial.println("1. Ketik 'DIAGNOSTIC' untuk diagnosa ulang");
  Serial.println("2. Ketik 'BURNFIX' untuk burn ulang dengan metode berbeda");
  Serial.println("3. Ketik 'RESET' untuk reset ZPOS ke 0");
  Serial.println("4. Ketik 'STATUS' untuk cek status sensor");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toUpperCase();
    
    if (input == "DIAGNOSTIC") {
      diagnosticAS5600();
    } else if (input == "BURNFIX") {
      burnZPOSFixed();
    } else if (input == "RESET") {
      resetZPOS();
    } else if (input == "STATUS") {
      checkStatus();
    } else {
      Serial.println("Command tidak dikenal. Ketik DIAGNOSTIC, BURNFIX, RESET, atau STATUS");
    }
  }
  
  // Real-time reading
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 1000) {
    Serial.printf("RAW: %.2f° | PROCESSED: %.2f°\n", 
                  readRawAngle(), readProcessedAngle());
    lastUpdate = millis();
  }
}

void diagnosticAS5600() {
  Serial.println("\n=== DIAGNOSTIC AS5600 ===");
  
  // 1. Check I2C communication
  Wire.beginTransmission(AS5600_ADDR);
  byte error = Wire.endTransmission();
  if (error != 0) {
    Serial.printf("❌ I2C Error: %d\n", error);
    return;
  } else {
    Serial.println("✅ I2C Communication OK");
  }
  
  // 2. Read status register
  int status = readRegister8(AS5600_STATUS);
  Serial.printf("Status Register: 0x%02X\n", status);
  if (status & 0x20) Serial.println("✅ Magnet detected");
  else Serial.println("❌ No magnet detected");
  if (status & 0x10) Serial.println("⚠️ Magnet too weak");
  if (status & 0x08) Serial.println("⚠️ Magnet too strong");
  
  // 3. Read current ZPOS
  int zposh = readRegister8(AS5600_ZPOS_HIGH);
  int zposl = readRegister8(AS5600_ZPOS_LOW);
  int zposValue = (zposh << 8) | zposl;
  float zposAngle = zposValue * 0.087890625;
  
  Serial.printf("Current ZPOS: 0x%04X = %.2f°\n", zposValue, zposAngle);
  
  // 4. Read angles
  float rawAngle = readRawAngle();
  float processedAngle = readProcessedAngle();
  
  Serial.printf("Raw Angle: %.2f°\n", rawAngle);
  Serial.printf("Processed Angle: %.2f°\n", processedAngle);
  Serial.printf("Difference: %.2f°\n", processedAngle - rawAngle);
  
  // 5. Calculate what ZPOS should be
  float targetZPOS = rawAngle;
  int targetZPOSRaw = (int)(targetZPOS / 0.087890625);
  Serial.printf("Required ZPOS untuk 0°: 0x%04X = %.2f°\n", targetZPOSRaw, targetZPOS);
  
  // 6. Check if burn was successful
  if (abs(processedAngle) < 2.0) {
    Serial.println("✅ ZPOS burn berhasil");
  } else {
    Serial.println("❌ ZPOS burn gagal atau tidak diterapkan");
    
    // Kemungkinan penyebab
    if (zposValue == 0) {
      Serial.println("💡 Kemungkinan: ZPOS tidak ter-burn ke OTP memory");
    } else if (abs(zposAngle - rawAngle) > 5) {
      Serial.println("💡 Kemungkinan: ZPOS ter-burn tapi nilai salah");
    } else {
      Serial.println("💡 Kemungkinan: AS5600 tidak menggunakan ZPOS dari OTP");
    }
  }
}

void burnZPOSFixed() {
  Serial.println("\n=== BURN ZPOS (FIXED METHOD) ===");
  
  // Method yang lebih robust
  float currentRaw = readRawAngle();
  if (currentRaw < 0) {
    Serial.println("❌ Cannot read sensor");
    return;
  }
  
  Serial.printf("Current raw position: %.2f°\n", currentRaw);
  Serial.println("This will become the new 0° position");
  Serial.println("Type 'YES' to continue:");
  
  while (!Serial.available()) delay(10);
  String confirm = Serial.readStringUntil('\n');
  confirm.trim();
  confirm.toUpperCase();
  
  if (confirm != "YES") {
    Serial.println("Burn cancelled");
    return;
  }
  
  // Calculate ZPOS value
  int zposRaw = (int)(currentRaw / 0.087890625);
  byte zposhigh = (zposRaw >> 8) & 0xFF;
  byte zposlow = zposRaw & 0xFF;
  
  Serial.printf("Setting ZPOS to: 0x%04X (%.2f°)\n", zposRaw, currentRaw);
  
  // Method 1: Write ZPOS registers first
  Serial.println("Step 1: Writing ZPOS registers...");
  
  if (!writeRegister8(AS5600_ZPOS_HIGH, zposhigh)) {
    Serial.println("❌ Failed to write ZPOS_HIGH");
    return;
  }
  delay(10);
  
  if (!writeRegister8(AS5600_ZPOS_LOW, zposlow)) {
    Serial.println("❌ Failed to write ZPOS_LOW");
    return;
  }
  delay(10);
  
  // Verify write
  int readZH = readRegister8(AS5600_ZPOS_HIGH);
  int readZL = readRegister8(AS5600_ZPOS_LOW);
  
  if (readZH != zposhigh || readZL != zposlow) {
    Serial.printf("❌ ZPOS write verification failed!\n");
    Serial.printf("Expected: H=0x%02X L=0x%02X\n", zposhigh, zposlow);
    Serial.printf("Read: H=0x%02X L=0x%02X\n", readZH, readZL);
    return;
  }
  
  Serial.println("✅ ZPOS registers written successfully");
  
  // Check if processed angle changed
  delay(100);
  float processedBefore = readProcessedAngle();
  Serial.printf("Processed angle after ZPOS write: %.2f°\n", processedBefore);
  
  // Method 2: Burn to OTP
  Serial.println("Step 2: Burning to OTP memory...");
  
  // Different burn sequences to try
  byte burnCommands[] = {0x80, 0x40, 0x08, 0x01};
  
  for (int i = 0; i < 4; i++) {
    Serial.printf("Trying burn command: 0x%02X\n", burnCommands[i]);
    
    if (writeRegister8(AS5600_BURN, burnCommands[i])) {
      Serial.printf("Burn command 0x%02X sent\n", burnCommands[i]);
      delay(1000); // Wait for burn
      
      // Test if burn worked
      float testAngle = readProcessedAngle();
      if (abs(testAngle) < 2.0) {
        Serial.printf("✅ Burn successful with command 0x%02X!\n", burnCommands[i]);
        Serial.printf("New processed angle: %.2f°\n", testAngle);
        return;
      }
    }
    
    delay(500);
  }
  
  Serial.println("❌ All burn methods failed");
  
  // Final test
  float finalProcessed = readProcessedAngle();
  Serial.printf("Final processed angle: %.2f°\n", finalProcessed);
  
  if (abs(finalProcessed) < 2.0) {
    Serial.println("✅ ZPOS is working (registers only, not burned to OTP)");
  } else {
    Serial.println("❌ ZPOS not working");
  }
}

void resetZPOS() {
  Serial.println("Resetting ZPOS to 0...");
  writeRegister8(AS5600_ZPOS_HIGH, 0);
  writeRegister8(AS5600_ZPOS_LOW, 0);
  delay(100);
  Serial.printf("ZPOS reset. Processed angle: %.2f°\n", readProcessedAngle());
}

void checkStatus() {
  Serial.println("\n=== STATUS CHECK ===");
  diagnosticAS5600();
}

// Helper functions
float readRawAngle() {
  int high = readRegister8(AS5600_RAW_ANGLE_HIGH);
  int low = readRegister8(AS5600_RAW_ANGLE_LOW);
  if (high < 0 || low < 0) return -1;
  
  int rawValue = (high << 8) | low;
  return rawValue * 0.087890625;
}

float readProcessedAngle() {
  int high = readRegister8(AS5600_ANGLE_HIGH);
  int low = readRegister8(AS5600_ANGLE_LOW);
  if (high < 0 || low < 0) return -1;
  
  int rawValue = (high << 8) | low;
  return rawValue * 0.087890625;
}

int readRegister8(byte reg) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  byte error = Wire.endTransmission();
  if (error != 0) return -1;
  
  Wire.requestFrom(AS5600_ADDR, 1);
  if (Wire.available()) {
    return Wire.read();
  }
  return -1;
}

bool writeRegister8(byte reg, byte value) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  Wire.write(value);
  byte error = Wire.endTransmission();
  return (error == 0);
}