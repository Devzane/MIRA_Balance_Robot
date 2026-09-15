#ifndef CONFIG_H
#define CONFIG_H

// ======================= PIN MAP (matches diagram.json) =======================
#define I2C_SDA        21
#define I2C_SCL        22

#define MOTOR_A_IN1    27
#define MOTOR_A_IN2    26
#define MOTOR_A_EN     14   // PWM (speed) for motor A

#define MOTOR_B_IN1    25
#define MOTOR_B_IN2    33
#define MOTOR_B_EN     32   // PWM (speed) for motor B

// ESP32 LEDC PWM channels for the two enable pins
#define PWM_CHANNEL_A  0
#define PWM_CHANNEL_B  1
#define PWM_FREQ_HZ    2000
#define PWM_RESOLUTION 8     // 8-bit -> speed range 0-255

// ======================= LOOP TIMING =======================
// Control loop period in microseconds. 5000us = 200Hz.
// This MUST be fast enough that the robot can't fall between reads.
// If the loop can't keep up (Serial prints, delays, etc.) balancing gets shaky.
#define LOOP_PERIOD_US 5000

// ======================= MPU6050 =======================
#define MPU6050_ADDR   0x68

// Default sensor sensitivities (only valid if you do NOT change the
// MPU6050's full-scale range registers in IMU.cpp). If you ever change
// the range for less noise / more headroom, update these two numbers.
#define ACCEL_SENSITIVITY 16384.0f  // LSB per g        (±2g range)
#define GYRO_SENSITIVITY  131.0f    // LSB per deg/s    (±250 dps range)

// If your IMU is mounted upside-down or rotated, the sign of the angle
// or gyro rate can come out backwards (robot leans the "wrong" way and
// the motors fight it instead of correcting it). Flip these to -1 if so.
#define ACCEL_ANGLE_SIGN  1
#define GYRO_RATE_SIGN    1

// ======================= KALMAN FILTER =======================
// Q_ANGLE  : how much you trust the gyro's angle estimate over time.
//            Higher = filter reacts faster to gyro, but noisier.
// Q_BIAS   : how much the gyro's bias is allowed to drift.
//            Higher = adapts to bias drift faster, but less stable.
// R_MEASURE: how much you trust the accelerometer.
//            Higher = smoother but slower response; lower = twitchier,
//            more vibration-sensitive.
// These three numbers are 90% of "Kalman filter tuning."
#define KALMAN_Q_ANGLE   0.001f
#define KALMAN_Q_BIAS    0.003f
#define KALMAN_R_MEASURE 0.03f

// ======================= FUZZY CONTROLLER =======================
// The "universe of discourse" for each input. Pick these based on the
// worst-case tilt/rate you expect before the robot is considered fallen.
#define FUZZY_ERROR_RANGE      25.0f   // degrees, +/-
#define FUZZY_ERROR_RATE_RANGE 150.0f  // degrees/sec, +/-
#define FUZZY_OUTPUT_MAX       255.0f  // matches PWM resolution

// Robot's target (balance) angle in degrees, as reported by getAngle().
// If your robot leans consistently to one side at rest, adjust this
// FIRST before touching anything else.
#define SETPOINT_ANGLE 0.0f

// Angle beyond which we consider the robot "fallen" and cut the motors,
// instead of fighting a lost cause and burning out the driver/motors.
#define FALL_ANGLE_LIMIT 35.0f

#endif
