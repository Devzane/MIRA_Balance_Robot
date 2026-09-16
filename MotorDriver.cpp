#include <Arduino.h>
#include "MotorDriver.h"
#include "Config.h"

// Notice we changed 'pwmChannel' to 'enPin' here
static void setOneMotor(int in1, int in2, int enPin, float speed) {
  bool forward = speed >= 0;
  int pwmVal = (int)fabs(speed);
  if (pwmVal > 255) pwmVal = 255;

  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW  : HIGH);
  
  // ESP32 V3: Write directly to the physical pin
  ledcWrite(enPin, pwmVal); 
}

void motorInit() {
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  // ESP32 V3: 1-line setup combining attachPin and setup
  ledcAttach(MOTOR_A_EN, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcAttach(MOTOR_B_EN, PWM_FREQ_HZ, PWM_RESOLUTION);

  motorStop();
}

void motorSetSpeed(float speed) {
  // If your robot drives itself the WRONG way, flip the sign here
  // We now pass the EN pins instead of the old channels
  setOneMotor(MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_EN, speed);
  setOneMotor(MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_EN, speed);
}

void motorStop() {
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
  
  // ESP32 V3: Write 0 directly to the pins
  ledcWrite(MOTOR_A_EN, 0);
  ledcWrite(MOTOR_B_EN, 0);
}