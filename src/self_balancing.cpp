#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "PID.c"

Adafruit_MPU6050 mpu;


#define enA 6
#define in1 7
#define in2 8
#define enB 11
#define in3 12
#define in4 13

int motorSpeedA = 255; 
int motorSpeedB = 255;

struct PIDConstants drivetrainConsts = {
  .kP = 1,
  .kI = 0.1,
  .kD = 0.01
};


struct PIDState drivetrainState;

int timeStepMs = 500;
float pos = 0;
float goal = 1000;
float voltage = 0;

void both(int speed){
    if(speed < 0){
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
    } else {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
    }
    analogWrite(enB, abs(speed));
    analogWrite(enA, abs(speed));
}

void setup(void){
   Serial.begin(9600);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
 
  Serial.println("Adafruit MPU6050 test!");
 
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  // Init stuff for gyro
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

}
void loop(void){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  pos = a.gyro.y;
  
  float error = goal-pos;
  voltage = iterate(error, &drivetrainConsts, &drivetrainState);

  

  Serial.print("voltage: ");
  Serial.print(voltage);
  Serial.print("\tposition: ");
  Serial.print(pos);
  Serial.print("\terror: ");
  Serial.print(error);
  Serial.print("\tintegral: ");
  Serial.print(drivetrainState.integral);
  Serial.print("\tderivative: ");
  Serial.print(drivetrainState.derivative);
  Serial.println();
  
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, motorSpeedA);
  analogWrite(enB, motorSpeedB);
}
