#include <Arduino.h>
#include "PID.h"

void wait(unsigned long cycleLengthMs, unsigned long *lastTime){
  unsigned long timeToWaitUntil = *lastTime + cycleLengthMs;
  while(millis() < timeToWaitUntil){
    delay(10);
  }
  *lastTime = millis();
}

float iterate(float error, struct PIDConstants *consts, struct PIDState *state){
  state->integral = state->integral + error;
  state->derivative = error - state->prevError;
  state->prevError = error;


  float voltage = consts->kP * error + consts->kI * state->integral + consts->kD * state->derivative;

  if((state->integral > 0) != (error > 0)){
      state->integral = 0;
  }

  return voltage;
}
