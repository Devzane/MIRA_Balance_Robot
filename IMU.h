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
// Returned values already have the calibration offsets (below) applied.
IMUData imuRead();

// Averages a batch of samples to find the gyro's at-rest offset and the
// accelerometer's at-rest angle offset, and stores them for imuRead() to
// subtract automatically from then on. Call this ONCE in setup(), with
// the robot held still and as close to your intended balance position
// as possible (usually vertical). Blocks for about numSamples * 5ms.
void imuCalibrate(int numSamples = 200);

#endif
