#pragma once
#include <Arduino.h>
#include "config.h"
#include "pid.h"

struct SpeedSample
{
  long countsL;
  long countsR;
  float rpsL;
  float rpsR;
  float rpmL;
  float rpmR;
};

// ===== Motor API =====
void movementInit();

// Simple motions
void moveForward();
void moveBackward();
void turnRight();
void turnLeft();
void moveStop();

// Speed helpers
void motorSetSpeed(int left, int right);
void motorSetAll(int s);
void motorBoost(int amount = 20);
int motorGetLeftSpeed();
int motorGetRightSpeed();

// ===== Encoder helpers =====
SpeedSample encoderSampleAndCompute(uint32_t dt_ms);
long encoderGetCountL();
long encoderGetCountR();
void encoderResetCounts();

// ===== High-level movement =====
void driveCells(int nCells, int basePwm = MOTOR_SPEED_DEFAULT, bool brake = true);
void driveCm(float cm, int basePwm = MOTOR_SPEED_DEFAULT, bool brake = true);
void straightHoldTick(int basePwm);
void turnLeft90_MPU(int basePwm = MOTOR_SPEED_DEFAULT, float tolDeg = 2.0f);
void turnRight90_MPU(int basePwm = MOTOR_SPEED_DEFAULT, float tolDeg = 2.0f);

// ===== Tests =====
// test all direction
void testMotor();
// Runs motors forward while sampling encoders and printing telemetry for ms duration.
void testEncoder(unsigned long duration_ms = TEST_DURATION_MS, int basePwm = TEST_BASE_PWM);

// === Wall-follow drive of one or more cells ===
// ==== Wall-follow drive (VLX) ====
enum MoveResult
{
  MOVE_OK,
  MOVE_BLOCKED,
  MOVE_STALLED
};

struct WallFollowCfg
{
  // VLX validity in centimeters (because VLX_readDistance returns cm)
  uint8_t minValid = 0;  // أقل مسافة موثوقة
  uint8_t maxValid = 7;  // أكبر مسافة موثوقة
  uint8_t frontStop = 9; // لو الأمامي <= دي → اعتبره عائق وأوقف

  // Control
  int basePwm = MOTOR_SPEED_DEFAULT; // سرعة الأساس
  int maxAddPwm = 80;                // أقصى فرق تصحيح (يمين-شمال)
  float targetDelta = 0.0f;          // لو في حيطتين: L - R ≈ targetDelta
  float sideTarget = 2.0f;           // لو حيطة واحدة: المسافة الهدف للحيطة

  // Timing
  uint16_t timeoutMsPerCell = 1500;
  bool frontSquareAtEnd = true; // تربيع أمامي خفيف عند نهاية الخلية
};

// قدِّم n خلايا مع سنترة VLX ويرجع الحالة
MoveResult driveCell_WF(int nCells = 1, const WallFollowCfg &cfg = {});
