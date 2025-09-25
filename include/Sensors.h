#pragma once
#include <Wire.h>
#include <Adafruit_VL6180X.h>
#include <MPU6050_light.h>
#include "config.h"

// Expose variables (for MPU)
extern MPU6050 mpu;
extern float filteredAngleZ;
extern float alpha;
extern unsigned long previousTime;

// Declare array of sensor objects
extern Adafruit_VL6180X sensors[COUNT_SENSORS];

//init for sensors 
void sensorsInit();

// API functions (VLX)
uint8_t VLX_readDistance(uint8_t sensorID);

// Functions for MPU
void initMPU();
void updateMPU();

// ===== Tests =====
void testMPU(unsigned long duration_ms = TEST_DURATION_MS);
void testVLX(unsigned long duration_ms = TEST_DURATION_MS);