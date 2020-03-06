#include <Arduino.h>
#include "PID.h"

int wait(int cycleLengthMs, int timer){
  int startTime = micros();
  while(micros() < cycleLengthMs){
    delayMicroseconds(10);
  }
  int timeWaited = micros() - startTime;
  return timeWaited;
}

float iterate(float error, struct PIDConstants *consts, struct PIDState *state){
  state->integral = state->integral + error;
  state->derivative = error - state->prevError;
  state->prevError = error;


  float voltage = consts->kP * error + consts->kI * state->integral + consts->kD * state->derivative;

  /*if(integral > 0 != error > 0){
      integral = 0;
  }*/

  return voltage;
}
