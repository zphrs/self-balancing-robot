#include <Arduino.h>
#ifndef PID_LIBRARY
#define PID_LIBRARY

struct PIDConstants {
  float kP, kI, kD;
};

struct PIDState {
  float prevError, integral, derivative;
};

int wait(int cycleLengthMs, int timer);

float iterate(float error, struct PIDConstants *consts, struct PIDState *state);

#endif // PID_LIBRARY
