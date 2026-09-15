#include "KalmanFilter.h"
#include "Config.h"

KalmanFilter::KalmanFilter() {
  Q_angle   = KALMAN_Q_ANGLE;
  Q_bias    = KALMAN_Q_BIAS;
  R_measure = KALMAN_R_MEASURE;

  angle = 0.0f;
  bias  = 0.0f;

  P[0][0] = 0.0f; P[0][1] = 0.0f;
  P[1][0] = 0.0f; P[1][1] = 0.0f;
}

void KalmanFilter::setAngle(float initAngle) {
  angle = initAngle;
}

float KalmanFilter::update(float newAngle, float newRate, float dt) {
  // ---- Predict step ----
  // Integrate the (bias-corrected) gyro rate to project the angle forward.
  rate = newRate - bias;
  angle += dt * rate;

  // Project the error covariance forward.
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // ---- Update step ----
  // Compare the predicted angle against the accelerometer's measurement.
  float S = P[0][0] + R_measure;        // estimate error
  float K0 = P[0][0] / S;               // Kalman gain (angle)
  float K1 = P[1][0] / S;               // Kalman gain (bias)

  float y = newAngle - angle;           // measurement residual

  angle += K0 * y;
  bias  += K1 * y;

  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K0 * P00_temp;
  P[0][1] -= K0 * P01_temp;
  P[1][0] -= K1 * P00_temp;
  P[1][1] -= K1 * P01_temp;

  return angle;
}
