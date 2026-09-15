#include <Wire.h>
#include <math.h>
#include "IMU.h"
#include "Config.h"

// MPU6050 register addresses
#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_XOUT_H  0x43
#define REG_WHO_AM_I     0x75

static int16_t readWord(uint8_t highReg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(highReg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU6050_ADDR, 2, true);
  int16_t value = (Wire.read() << 8) | Wire.read();
  return value;
}

bool imuInit() {
  // Wake the sensor up: write 0x00 to the power management register.
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00);
  uint8_t err = Wire.endTransmission(true);

  // Sanity check: WHO_AM_I should read back the device's I2C address (0x68).
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU6050_ADDR, 1, true);
  uint8_t who = Wire.read();

  return (err == 0) && (who == MPU6050_ADDR);
}

IMUData imuRead() {
  IMUData data;

  // Burst-read all 6 accel bytes starting at ACCEL_XOUT_H.
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU6050_ADDR, 6, true);
  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();

  // Gyro Y axis = rotation rate around the pitch axis for a robot that
  // leans forward/back with the IMU mounted flat, X-axis pointing
  // "forward." If your IMU is mounted differently, this is the first
  // thing to change (try gyro X or Z instead).
  int16_t rawGy = readWord(REG_GYRO_XOUT_H + 2);

  float ax = rawAx / ACCEL_SENSITIVITY;
  float ay = rawAy / ACCEL_SENSITIVITY;
  float az = rawAz / ACCEL_SENSITIVITY;

  // Pitch angle from accelerometer geometry alone. Noisy on its own
  // (vibration shows up directly), which is exactly why we feed this
  // into the Kalman filter rather than using it directly.
  float angle = atan2(ax, sqrt(ay * ay + az * az)) * 180.0f / PI;

  data.accelAngle = ACCEL_ANGLE_SIGN * angle;
  data.gyroRate   = GYRO_RATE_SIGN * (rawGy / GYRO_SENSITIVITY);

  return data;
}
