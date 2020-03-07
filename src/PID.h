#include <Arduino.h>
#ifndef PID_LIBRARY
#define PID_LIBRARY

struct PIDConstants {
  float kP, kI, kD;
};

struct PIDState {
  float prevError, integral, derivative;
};

void wait(unsigned long cycleLengthMs, unsigned long *lastTime);

float iterate(float error, struct PIDConstants *consts, struct PIDState *state);

#endif // PID_LIBRARY
