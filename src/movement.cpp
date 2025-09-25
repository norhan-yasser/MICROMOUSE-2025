#include "movement.h"
#include "pid.h"
#include "sensors.h"

// Values for motors
static int g_speedL = 105;
static int g_speedR = 110;

// Values for Encoder
static volatile long g_cntL = 0;
static volatile long g_cntR = 0;

static volatile uint32_t g_lastUsL = 0, g_lastUsR = 0;

// interupts for encoder
static void IRAM_ATTR isr_leftA()
{
  uint32_t now = micros();
  if (now - g_lastUsL < MIN_PULSE_US)
    return;
  g_lastUsL = now;
  bool b = digitalRead(ENC_L_B);
  g_cntL += (b ? -1 : +1);
}

static void IRAM_ATTR isr_rightA()
{
  uint32_t now = micros();
  if (now - g_lastUsR < MIN_PULSE_US)
    return;
  g_lastUsR = now;
  bool b = digitalRead(ENC_R_B);
  g_cntR += (b ? -1 : +1);
}

static PID headingPI;
static long prevL = 0, prevR = 0;
static uint32_t lastHoldMs = 0;
static uint32_t lastMs = 0;

void movementInit()
{
  // Motors
  pinMode(SLEEP_PIN, OUTPUT);
  digitalWrite(SLEEP_PIN, LOW);

  ledcSetup(CH_AIN1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_AIN2, PWM_FREQ, PWM_RES);
  ledcSetup(CH_BIN1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_BIN2, PWM_FREQ, PWM_RES);

  ledcAttachPin(AIN1, CH_AIN1);
  ledcAttachPin(AIN2, CH_AIN2);
  ledcAttachPin(BIN1, CH_BIN1);
  ledcAttachPin(BIN2, CH_BIN2);

  delay(40);

  digitalWrite(SLEEP_PIN, HIGH);

  moveStop();

  // Encoder
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isr_leftA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isr_rightA, RISING);

  // PID init (keep same gains/defaults)
  headingPI.kp = HEADING_KP;
  headingPI.ki = HEADING_KI;
  headingPI.kd = HEADING_KD;
  headingPI.outMin = -255.0f;
  headingPI.outMax = 255.0f;
  pidReset(headingPI);
}

static inline long cmToTicks(float cm)
{
  return (long)((cm * (float)K_TICK / CELL_CM) + 0.5f);
}

// functions for motors
void moveForward()
{
  ledcWrite(CH_AIN2, 0);
  ledcWrite(CH_AIN1, g_speedL);
  ledcWrite(CH_BIN1, 0);
  ledcWrite(CH_BIN2, g_speedR);
}

void moveBackward()
{
  ledcWrite(CH_AIN2, g_speedL);
  ledcWrite(CH_AIN1, 0);
  ledcWrite(CH_BIN1, g_speedR);
  ledcWrite(CH_BIN2, 0);
}

void moveStop()
{
  ledcWrite(CH_AIN1, 0);
  ledcWrite(CH_AIN2, 0);
  ledcWrite(CH_BIN1, 0);
  ledcWrite(CH_BIN2, 0);
}

// فرملة (اختياري)
void moveBrake(int strength = MOTOR_SPEED_MAX)
{
  ledcWrite(CH_AIN1, strength);
  ledcWrite(CH_AIN2, strength);
  ledcWrite(CH_BIN1, strength);
  ledcWrite(CH_BIN2, strength);
}

void turnRight()
{
  ledcWrite(CH_AIN1, g_speedL);
  ledcWrite(CH_AIN2, 0);

  ledcWrite(CH_BIN1, g_speedR);
  ledcWrite(CH_BIN2, 0);
}

void turnLeft()
{
  ledcWrite(CH_AIN1, 0);
  ledcWrite(CH_AIN2, g_speedL);

  ledcWrite(CH_BIN1, 0);
  ledcWrite(CH_BIN2, g_speedR);
}

void motorSetSpeed(int left, int right)
{
  g_speedL = constrain(left, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
  g_speedR = constrain(right, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
}

void motorSetAll(int s) { motorSetSpeed(s - 10, s); }
void motorBoost(int amount) { motorSetSpeed(g_speedL + amount, g_speedR + amount); }

int motorGetLeftSpeed() { return g_speedL; }
int motorGetRightSpeed() { return g_speedR; }

// Counting in Encoder
SpeedSample encoderSampleAndCompute(uint32_t dt_ms)
{
  static long lastL = 0, lastR = 0;

  noInterrupts();
  long cL = g_cntL;
  long cR = g_cntR;
  interrupts();

  long dL = cL - lastL;
  long dR = cR - lastR;
  lastL = cL;
  lastR = cR;

  float dt_s = dt_ms / 1000.0f;
  float rpsL = (dL / (float)CPR) / dt_s;
  float rpsR = (dR / (float)CPR) / dt_s;

  SpeedSample s;
  s.countsL = cL;
  s.countsR = cR;
  s.rpsL = rpsL;
  s.rpsR = rpsR;
  s.rpmL = rpsL * 60.0f;
  s.rpmR = rpsR * 60.0f;
  return s;
}

long encoderGetCountL() { return g_cntL; }
long encoderGetCountR() { return g_cntR; }
void encoderResetCounts()
{
  noInterrupts();
  g_cntL = g_cntR = 0;
  interrupts();
}

// drive with cm and cells
void driveTicks(long targetTicks, int basePwm, bool brake)
{
  encoderResetCounts();
  bool forward = (targetTicks >= 0);
  long target = labs(targetTicks);

  motorSetAll(basePwm);
  if (forward)
  {
    moveForward();
  }
  else
  {
    moveBackward();
  }

  pidReset(headingPI);
  lastMs = millis();

  while (true)
  {
    long cL = labs(encoderGetCountL());
    long cR = labs(encoderGetCountR());
    long avg = (cL + cR) / 2;

    if (avg >= target)
      break;

    long remaining = target - avg;

    int pwm = basePwm;
    if (remaining < SLOWDOWN_MARGIN_TICKS)
    {
      int scaled = (int)((long)basePwm * remaining / SLOWDOWN_MARGIN_TICKS);
      if (scaled < PWM_MIN_RUN)
        scaled = PWM_MIN_RUN;
      pwm = scaled;
    }

    long err = (long)cL - (long)cR;

    uint32_t now = millis();
    float dt = (now - lastMs) / 1000.0f;
    lastMs = now;

    float corr = pidUpdate(headingPI, (float)err, dt);

    int pwmL = pwm - (int)corr;
    int pwmR = pwm + (int)corr;

    motorSetSpeed(pwmL, pwmR);
  }

  if (brake)
  {
    moveStop();
  }
}

// NOTE: Use the provided basePwm argument (fixes hard-coded 150)
void driveCells(int nCells, int basePwm, bool brake)
{
  long ticks = (long)K_TICK * (long)nCells;
  driveTicks(ticks, basePwm, brake);
}

// NOTE: Use the provided basePwm argument (fixes hard-coded 150)
void driveCm(float cm, int basePwm, bool brake)
{
  long ticks = cmToTicks(cm);
  driveTicks(ticks, basePwm, brake);
}

// PID for encoders
void straightHoldTick(int basePwm)
{
  long cL = encoderGetCountL();
  long cR = encoderGetCountR();
  long dL = cL - prevL;
  long dR = cR - prevR;
  prevL = cL;
  prevR = cR;

  uint32_t now = millis();
  float dt = (lastHoldMs == 0) ? 0.02f : (now - lastHoldMs) / 1000.0f;
  lastHoldMs = now;

  long err = dL - dR; // +ve => left faster → correct right
  float corr = pidUpdate(headingPI, (float)err, dt);

  int pwmL = basePwm - (int)corr;
  int pwmR = basePwm + (int)corr;

  motorSetSpeed(pwmL, pwmR);
  moveForward();
}

static inline float norm180(float a)
{
  while (a <= -180.f)
    a += 360.f;
  while (a > 180.f)
    a -= 360.f;
  return a;
}

// يتحدد مرة واحدة: هل الزيادة الموجبة في الزاوية = يمين؟
static bool g_posIsRight_known = false;
static bool g_posIsRight = true; // قيمة افتراضية

void detectAngleSignOnce()
{
  if (g_posIsRight_known)
    return;
  // نبضة صغيرة يمين ونشوف الزاوية زادت ولا قلت
  updateMPU();
  float a0 = norm180(filteredAngleZ);
  motorSetAll(150);
  turnRight();
  delay(80);
  moveStop();
  delay(40);
  updateMPU();
  float a1 = norm180(filteredAngleZ);
  g_posIsRight = (a1 > a0); // لو زادت → الموجب = يمين
  g_posIsRight_known = true;
  delay(60);
}

void turnByAngleSnap(float targetDelta, int basePwm = 170, float tolDeg = 5.0f)
{
  const unsigned long TIMEOUT_MS = 5000UL;
  const float decelAngle = 60.0f;
  const int minPWM = 100;
  const int brakePWM = 120;
  const int brakeMs = 70;

  detectAngleSignOnce(); // <<< مهم

  updateMPU();
  const float start = norm180(filteredAngleZ);
  const float target = norm180(start + targetDelta);
  const float need = norm180(target - start);

  // اتجاه الحركة المطلوب كـ "يمين؟"
  // لو الموجب = يمين → right = (need > 0)
  // لو الموجب = شمال → right = (need < 0)
  const bool right = g_posIsRight ? (need > 0) : (need < 0);

  int kick = max(basePwm, minPWM);
  motorSetAll(kick);
  if (right)
    turnRight();
  else
    turnLeft();
  unsigned long t0 = millis();

  while (true)
  {
    if (millis() - t0 > TIMEOUT_MS)
      break;

    updateMPU();
    float curr = norm180(filteredAngleZ);
    float err = norm180(target - curr);
    float aerr = fabs(err);
    if (aerr <= tolDeg)
      break;

    float scale = 1.0f;
    if (aerr < decelAngle)
    {
      scale = aerr / decelAngle;
      if (scale < 0.35f)
        scale = 0.35f;
    }
    int pwm = (int)(basePwm * scale);
    if (pwm < minPWM)
      pwm = minPWM;

    motorSetAll(pwm);
    if (right)
      turnRight();
    else
      turnLeft();
    delay(6);
  }

  // فرملة عكسية قصيرة ثم وقف — بدون أي تصحيح بعد الوقف
  motorSetAll(brakePWM);
  if (right)
    turnLeft();
  else
    turnRight();
  delay(brakeMs);

  moveBrake();
  delay(150);
}

void turnLeft90_MPU(int basePwm, float tolDeg) { turnByAngleSnap(+90.0f, basePwm, tolDeg); }
void turnRight90_MPU(int basePwm, float tolDeg) { turnByAngleSnap(-90.0f, basePwm, tolDeg); }

// test for Motor
void testMotor()
{
  moveForward();
  delay(2000);
  moveBackward();
  delay(2000);
  turnRight();
  delay(2000);
  turnLeft();
  delay(2000);
  moveStop();
  delay(2000);
}

// test for Encoder
void testEncoder(unsigned long duration_ms, int basePwm)
{
  Serial.println("[TEST] Encoder start");
  encoderResetCounts();
  motorSetAll(basePwm);
  moveForward();

  unsigned long start = millis();
  unsigned long lastSample = millis();
  while (millis() - start < duration_ms)
  {
    if (millis() - lastSample >= SAMPLE_MS)
    {
      lastSample += SAMPLE_MS;
      SpeedSample s = encoderSampleAndCompute(SAMPLE_MS);
      Serial.print("L=");
      Serial.print(s.countsL);
      Serial.print(" R=");
      Serial.print(s.countsR);
      Serial.print(" RPM_L=");
      Serial.print(s.rpmL, 1);
      Serial.print(" RPM_R=");
      Serial.print(s.rpmR, 1);
      Serial.print(" pwmL=");
      Serial.print(motorGetLeftSpeed());
      Serial.print(" pwmR=");
      Serial.println(motorGetRightSpeed());
    }
    delay(1);
  }
  moveStop();
  Serial.println("[TEST] Encoder done");
}

// ===== Wall-follow helpers (cm-based) =====
static inline bool vlxValidCm(uint8_t d, const WallFollowCfg &cfg)
{
  return (d >= cfg.minValid && d <= cfg.maxValid);
}

// خطوة تتبّع حائط واحدة: تقرأ VLX وتكتب PWM
static bool wallFollowTickOnce(const WallFollowCfg &cfg, PID &pid, uint32_t &lastMs)
{
  uint8_t r = VLX_readDistance(RIGHT_VLX);  // cm
  uint8_t m = VLX_readDistance(MIDDLE_VLX); // cm
  uint8_t l = VLX_readDistance(LEFT_VLX);   // cm

  static float lastCorr = 0.0f;
  // لو في عائق قدّام قريب → وقف واعتبر BLOCKED
  if (m <= cfg.frontStop)
  {
    moveStop();
    return false;
  }

  const bool haveL = vlxValidCm(l, cfg);
  const bool haveR = vlxValidCm(r, cfg);

  // fallback: لو مفيش جوانب موثوقة، امشِ مستقيم بإنكودر
  if (!haveL && !haveR)
  {
    // ما فيش جوانب موثوقة → امشي مع احتفاظ جزئي بآخر تصحيح
    lastCorr *= 0.9f; // decay خفيف
    int pwmL = cfg.basePwm - (int)lastCorr;
    int pwmR = cfg.basePwm + (int)lastCorr;
    motorSetSpeed(pwmL, pwmR);
    moveForward();
    return true;
  }

  // خطأ السنترة (سم)
  float error = 0.0f;
  if (haveL && haveR)
  {
    error = (float)l - (float)r - cfg.targetDelta; // وسط الممر
  }
  else if (haveL)
  {
    error = (float)l - cfg.sideTarget; // اتبع يسار
  }
  else
  {                                    // haveR
    error = cfg.sideTarget - (float)r; // اتبع يمين
  }

  // dt للـ PID
  uint32_t now = millis();
  float dt = (lastMs == 0) ? 0.02f : (now - lastMs) / 1000.0f;
  if (dt < 0.001f)
    dt = 0.001f;
  lastMs = now;

  float corr = pidUpdate(pid, error, dt); // + يمين / - شمال

  int pwmL = cfg.basePwm - (int)corr;
  int pwmR = cfg.basePwm + (int)corr;

  // حد أقصى لفرق التصحيح
  int diff = pwmR - pwmL;
  int maxDiff = cfg.maxAddPwm * 2;
  if (diff > maxDiff)
  {
    int over = diff - maxDiff;
    pwmR -= over / 2;
    pwmL += over / 2;
  }
  if (diff < -maxDiff)
  {
    int over = (-maxDiff) - diff;
    pwmR += over / 2;
    pwmL -= over / 2;
  }

  pwmL = constrain(pwmL, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
  pwmR = constrain(pwmR, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);

  motorSetSpeed(pwmL, pwmR);
  moveForward();
  return true;
}

// قدِّم n خلايا مع سنترة VLX
MoveResult driveCell_WF(int nCells, const WallFollowCfg &cfg)
{
  // PID للسنترة (نستخدم نفس gains بتاعة heading كبداية)
  PID pid{};
  pid.kp = WALL_KP;
  pid.ki = WALL_KI;
  pid.kd = WALL_KD;
  pid.outMin = -255.0f;
  pid.outMax = 255.0f;
  pidReset(pid);

  // هدف التيكات
  const long targetTicks = (long)(K_TICK * nCells); // K_TICK موجود في config.h
  encoderResetCounts();

  uint32_t t0 = millis();
  uint32_t lastMs = 0;

  while (true)
  {
    // // Timeout
    // if (millis() - t0 > (uint32_t)cfg.timeoutMsPerCell * (uint32_t)nCells)
    // {
    //   moveStop();
    //   return MOVE_STALLED;
    // }

    // خطوة تتبّع واحدة
    if (!wallFollowTickOnce(cfg, pid, lastMs))
    {
      // شاف قدّام قريب → BLOCKED
      return MOVE_BLOCKED;
    }

    // تحقق المسافة (متوسط التيكات)
    long cL = labs(encoderGetCountL());
    long cR = labs(encoderGetCountR());
    long avg = (cL + cR) / 2;

    // تهدئة قرب نهاية الخلية (لو driveTicks عندك بيعملها، تمام)
    // هنا بنكتفي بالتحقق النهائي:
    if (avg >= targetTicks)
    {
      moveBrake(120);

      // تربيع أمامي اختياري
      if (cfg.frontSquareAtEnd)
      {
        uint8_t m = VLX_readDistance(MIDDLE_VLX); // cm
        if (vlxValidCm(m, cfg))
        {
          const uint8_t FRONT_TARGET = 4; // cm
          const uint8_t TOL = 1;
          const int PWM = 110;
          uint32_t t1 = millis();
          while (millis() - t1 < 350)
          {
            m = VLX_readDistance(MIDDLE_VLX);
            if (!vlxValidCm(m, cfg))
              break;
            int err = (int)m - (int)FRONT_TARGET;
            if (abs(err) <= TOL)
              break;
            if (err > 0)
            {
              motorSetAll(PWM);
              moveForward();
            }
            else
            {
              motorSetAll(PWM);
              moveBackward();
            }
            delay(10);
          }
          moveBrake(120);
        }
      }

      moveStop();
      return MOVE_OK;
    }

    delay(5);
  }
}
