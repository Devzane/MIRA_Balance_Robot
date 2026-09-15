#include <Arduino.h>
#include "MotorDriver.h"
#include "Config.h"

static void setOneMotor(int in1, int in2, int pwmChannel, float speed) {
  bool forward = speed >= 0;
  int pwmVal = (int)fabs(speed);
  if (pwmVal > 255) pwmVal = 255;

  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW  : HIGH);
  ledcWrite(pwmChannel, pwmVal);
}

void motorInit() {
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  ledcSetup(PWM_CHANNEL_A, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_A_EN, PWM_CHANNEL_A);

  ledcSetup(PWM_CHANNEL_B, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_B_EN, PWM_CHANNEL_B);

  motorStop();
}

void motorSetSpeed(float speed) {
  // If your robot drives itself the WRONG way (leans forward, motors
  // push it further forward instead of catching it), the fastest fix
  // is to flip the sign here -- try `-speed` on one or both calls below
  // -- rather than rewiring anything.
  setOneMotor(MOTOR_A_IN1, MOTOR_A_IN2, PWM_CHANNEL_A, speed);
  setOneMotor(MOTOR_B_IN1, MOTOR_B_IN2, PWM_CHANNEL_B, speed);
}

void motorStop() {
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);
}
