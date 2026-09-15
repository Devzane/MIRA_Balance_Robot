#ifndef IMU_H
#define IMU_H

// Talks to the MPU6050 directly over I2C using raw register reads.
// Deliberately avoids external libraries (Adafruit_MPU6050 etc.) so this
// compiles and runs the same way on real hardware and in Wokwi, with
// nothing extra to install.

struct IMUData {
  float accelAngle;  // tilt angle from accelerometer only, degrees
  float gyroRate;    // angular rate around the pitch axis, deg/s
};

// Wakes the MPU6050 up (it powers on in sleep mode). Call once in setup().
// Returns true if the device responded, false if the I2C read failed
// (wrong wiring, wrong address, or the sensor isn't powered).
bool imuInit();

// Reads accel + gyro and returns the pitch angle estimate from the
// accelerometer alone, plus the raw gyro rate. This does NOT filter
// anything -- that's the Kalman filter's job, in KalmanFilter.h.
IMUData imuRead();

#endif
