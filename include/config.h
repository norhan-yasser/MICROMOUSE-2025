#pragma once
#include <Arduino.h>

// ================== PIN CONFIG ==================
// Motor Driver Pins
#ifndef AIN1
#define AIN1 27
#endif

#ifndef AIN2
#define AIN2 5
#endif

#ifndef BIN1
#define BIN1 18
#endif

#ifndef BIN2
#define BIN2 26
#endif

#ifndef SLEEP_PIN
#define SLEEP_PIN 19
#endif

// Encoder Pins
#ifndef ENC_L_A
#define ENC_L_A 23
#endif
#ifndef ENC_L_B
#define ENC_L_B 17
#endif
#ifndef ENC_R_A
#define ENC_R_A 4
#endif
#ifndef ENC_R_B
#define ENC_R_B 13
#endif

// ================== PWM CONFIG ==================
#ifndef CH_AIN1
#define CH_AIN1 0
#endif
#ifndef CH_AIN2
#define CH_AIN2 1
#endif
#ifndef CH_BIN1
#define CH_BIN1 3
#endif
#ifndef CH_BIN2
#define CH_BIN2 4
#endif

#ifndef PWM_FREQ
#define PWM_FREQ 5000
#endif
#ifndef PWM_RES
#define PWM_RES 8
#endif

// Speed defaults
#ifndef MOTOR_SPEED_DEFAULT
#define MOTOR_SPEED_DEFAULT 110
#endif
#ifndef MOTOR_SPEED_MIN
#define MOTOR_SPEED_MIN 0
#endif
#ifndef MOTOR_SPEED_MAX
#define MOTOR_SPEED_MAX 255
#endif

// ================== ENCODER CONFIG ==================
#ifndef CPR
#define CPR 11 // Counts per revolution (edges you count)
#endif
#ifndef SAMPLE_MS
#define SAMPLE_MS 100 // Telemetry sample window (ms)
#endif
#ifndef MIN_PULSE_US
#define MIN_PULSE_US 100 // Debounce/noise reject
#endif

// Distance unit calibration
// K_TICK: encoder ticks required to move exactly one maze CELL (e.g., 18 cm).
#ifndef K_TICK
#define K_TICK 356.5f // TODO: replace with measured value
#endif
#ifndef CELL_CM
#define CELL_CM 18.0f // common Micromouse standard cell
#endif

// ================== TURN / DRIVE TUNING ==================
#ifndef SLOWDOWN_MARGIN_TICKS
#define SLOWDOWN_MARGIN_TICKS 100
#endif
#ifndef PWM_MIN_RUN
#define PWM_MIN_RUN 50
#endif
#ifndef TURN_TIMEOUT_MS
#define TURN_TIMEOUT_MS 4000
#endif

// ================== SENSOR CONFIG (VL6180X + MPU6050) ==================
// I2C addresses after reassignment
#ifndef LOX1_ADDRESS
#define LOX1_ADDRESS 0x30
#endif
#ifndef LOX2_ADDRESS
#define LOX2_ADDRESS 0x31
#endif
#ifndef LOX3_ADDRESS
#define LOX3_ADDRESS 0x32
#endif

// Shutdown pins for ESP32
#ifndef SHT_LOX1
#define SHT_LOX1 0 // Right sensor
#endif
#ifndef SHT_LOX2
#define SHT_LOX2 2 // Middle sensor
#endif
#ifndef SHT_LOX3
#define SHT_LOX3 15 // Left sensor
#endif

#ifndef COUNT_SENSORS
#define COUNT_SENSORS 3
#endif

#ifndef RIGHT_VLX
#define RIGHT_VLX 0
#endif
#ifndef MIDDLE_VLX
#define MIDDLE_VLX 1
#endif
#ifndef LEFT_VLX
#define LEFT_VLX 2
#endif

#ifndef MAX_DISTANCE_CM
#define MAX_DISTANCE_CM 25 // cm (safety fallback)
#endif

// Complementary filter alpha (MPU)
#ifndef MPU_ALPHA
#define MPU_ALPHA 0.80f
#endif

// =================== Algorithm Config ===================

#ifndef ALGO_BASE_PWM
#define ALGO_BASE_PWM 110
#endif

// safe distance
static const uint8_t TH_FRONT = 4; // cm
static const uint8_t TH_SIDE = 4;  // cm

// ================== DEFAULT GAINS ==================
#ifndef HEADING_KP
#define HEADING_KP 0.6f
#endif
#ifndef HEADING_KI
#define HEADING_KI 0.01f
#endif
#ifndef HEADING_KD
#define HEADING_KD 0.0f
#endif

#define WALL_KP 12.0f
#define WALL_KI 0.0f
#define WALL_KD 1.5f

// ================== TEST DEFAULTS ==================
#ifndef TEST_DURATION_MS
#define TEST_DURATION_MS 5000
#endif
#ifndef TEST_BASE_PWM
#define TEST_BASE_PWM 120
#endif
