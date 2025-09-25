#pragma once
#include <Arduino.h>

struct PID {
  float kp, ki, kd;
  float i = 0.0f;
  float prev = 0.0f;
  float outMin = -255.0f;
  float outMax = 255.0f;
};

inline void pidReset(PID &c) {
  c.i = 0.0f;
  c.prev = 0.0f;
}

inline float pidUpdate(PID &c, float error, float dt_s) {
  c.i += error * c.ki * dt_s;
  if (c.i > c.outMax) c.i = c.outMax;
  if (c.i < c.outMin) c.i = c.outMin;

  float d = (error - c.prev) / (dt_s > 0 ? dt_s : 1e-3f);
  c.prev = error;

  float out = (c.kp * error) + c.i + (c.kd * d);
  if (out > c.outMax) out = c.outMax;
  if (out < c.outMin) out = c.outMin;
  return out;
}
