#include <Wire.h>
#include "Config.h"
#include "IMU.h"
#include "KalmanFilter.h"
#include "FuzzyController.h"
#include "MotorDriver.h"

KalmanFilter kalman;
FuzzyController fuzzy;

unsigned long lastLoopTime = 0;
float previousError = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!imuInit()) {
    Serial.println("MPU6050 not responding. Check wiring/address before anything else.");
  }

  motorInit();

  // Hold the robot still and vertical while this runs (~1 second) --
  // it measures and stores the gyro's at-rest offset and the
  // accelerometer's at-rest angle offset. Without this, every angle
  // reading has a built-in error that no amount of Kalman/fuzzy tuning
  // can fix, because the filter would just be tracking a biased signal.
  imuCalibrate();

  // Seed the Kalman filter with a real (now-calibrated) reading instead
  // of 0, so it doesn't start by "correcting" a fake angle for the
  // first few cycles.
  IMUData firstReading = imuRead();
  kalman.setAngle(firstReading.accelAngle);

  lastLoopTime = micros();
}

void loop() {
  unsigned long now = micros();
  unsigned long elapsed = now - lastLoopTime;

  // Fixed-rate loop: only run the control step once LOOP_PERIOD_US has
  // passed. This keeps dt predictable, which the Kalman filter assumes.
  if (elapsed < LOOP_PERIOD_US) return;
  float dt = elapsed / 1000000.0f;
  lastLoopTime = now;

  IMUData sensor = imuRead();
  float filteredAngle = kalman.update(sensor.accelAngle, sensor.gyroRate, dt);

  float error = SETPOINT_ANGLE - filteredAngle;

  // If the robot is past the point of no return, stop fighting it --
  // cutting power here also protects the motors/driver from stalling
  // at full torque against the floor.
  if (fabs(error) > FALL_ANGLE_LIMIT) {
    motorStop();
    previousError = 0;
    return;
  }

  // Error rate: prefer the gyro (fast, low-latency) over differentiating
  // the filtered angle (laggier, amplifies noise).
  float errorRate = -sensor.gyroRate;

  float motorOutput = fuzzy.compute(error, errorRate);
  motorSetSpeed(motorOutput);

  previousError = error;

  // Debug print -- comment this out once things are stable, since
  // Serial writes eat into your loop timing budget.
  Serial.print("angle: "); Serial.print(filteredAngle);
  Serial.print("  error: "); Serial.print(error);
  Serial.print("  output: "); Serial.println(motorOutput);
}
