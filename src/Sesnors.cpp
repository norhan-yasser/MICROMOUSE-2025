#include <Wire.h>
#include <Arduino.h>
#include "Sensors.h"

//init for VLX
Adafruit_VL6180X sensors[COUNT_SENSORS];
//init for MPU
MPU6050 mpu(Wire);
float filteredAngleZ = 0.0;
float alpha = MPU_ALPHA;
unsigned long previousTime = 0;

void sensorsInit() {
  // Configure shutdown pins as output
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);

  // Reset sensors array
  for (uint8_t i=0; i<COUNT_SENSORS; i++) {
    sensors[i] = Adafruit_VL6180X();
  }

  // First reset all sensors
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  delay(10);

  // Power on all (but still inactive until begin)
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  // === Sensor 1 (Right) ===
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  delay(10);
  if (!sensors[RIGHT_VLX].begin()) {
    Serial.println("Right Sensor Failed!");
  }
  sensors[RIGHT_VLX].setAddress(LOX1_ADDRESS);
  delay(20);

  // === Sensor 2 (Middle) ===
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);
  if (!sensors[MIDDLE_VLX].begin()) {
    Serial.println("Middle Sensor Failed!");
  }
  sensors[MIDDLE_VLX].setAddress(LOX2_ADDRESS);
  delay(20);

  // === Sensor 3 (Left) ===
  digitalWrite(SHT_LOX3, HIGH);
  delay(20);
  if (!sensors[LEFT_VLX].begin()) {
    Serial.println("Left Sensor Failed!");
  }
  sensors[LEFT_VLX].setAddress(LOX3_ADDRESS);
  delay(20);

  delay(1000); // Allow sensors to stabilize

//for MPU
  Wire.begin();
  mpu.begin();
  mpu.calcOffsets();

  // Initialize complementary filter
  filteredAngleZ = mpu.getAngleZ();
  previousTime = millis();
}

uint8_t VLX_readDistance(uint8_t sensorID) {
  if (sensorID >= COUNT_SENSORS) return MAX_DISTANCE_CM;

  // NOTE: library returns mm; divide by 10 → cm (kept as in original)
  uint8_t dist = sensors[sensorID].readRange() / 10; // mm → cm
  if (sensors[sensorID].readRangeStatus() != 0) {
    return MAX_DISTANCE_CM;  // invalid reading
  }
  return dist;
}


//functions for MPU
void updateMPU() {
  mpu.update();

  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0f;
  previousTime = currentTime;

  float rawAngleZ = mpu.getAngleZ();

  // Complementary filter
  filteredAngleZ = alpha * (filteredAngleZ + mpu.getGyroZ() * deltaTime ) + (1.0f - alpha) * rawAngleZ;
}

//test functions
void testMPU(unsigned long duration_ms) {
  Serial.println("[TEST] MPU start");
  unsigned long start = millis();
  while (millis() - start < duration_ms) {
    updateMPU();
    Serial.print("gyroZ="); Serial.print(mpu.getGyroZ(), 2);
    Serial.print(" rawZ=");  Serial.print(mpu.getAngleZ(), 2);
    Serial.print(" filtZ="); Serial.println(filteredAngleZ, 2);
    delay(20);
  }
  Serial.println("[TEST] MPU done");
}

void testVLX(unsigned long duration_ms) {
  Serial.println("[TEST] VLX start");
  unsigned long start = millis();
  while (millis() - start < duration_ms) {
    uint16_t r = VLX_readDistance(RIGHT_VLX);
    uint16_t m = VLX_readDistance(MIDDLE_VLX);
    uint16_t l = VLX_readDistance(LEFT_VLX);
    Serial.print("VLX (R,M,L) = ");
    Serial.print(r); Serial.print(", ");
    Serial.print(m); Serial.print(", ");
    Serial.println(l);
    delay(100);
  }
  Serial.println("[TEST] VLX done");
}

