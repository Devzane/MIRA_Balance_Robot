#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

// Standard 2-state Kalman filter for angle estimation: it fuses a noisy,
// drift-free angle measurement (accelerometer) with a smooth-but-drifting
// rate measurement (gyro) to get an angle estimate that is both fast AND
// stable long-term.
//
// State vector: [angle, gyro_bias]. The gyro bias is estimated too,
// because MPU6050 gyros drift over time -- if you only integrated the
// gyro directly, your angle estimate would slowly wander off even when
// the robot is standing still.

class KalmanFilter {
public:
  KalmanFilter();

  // Call this ONCE with a good starting angle (e.g. the first
  // accelerometer reading) before the control loop starts.
  void setAngle(float angle);

  // Call every loop iteration.
  //   newAngle : angle measurement from the accelerometer (degrees)
  //   newRate  : angular rate from the gyro (degrees/sec)
  //   dt       : time since the last call (seconds)
  // Returns the filtered angle estimate (degrees).
  float update(float newAngle, float newRate, float dt);

  float getBias() const { return bias; }

private:
  float Q_angle;
  float Q_bias;
  float R_measure;

  float angle;   // filtered angle estimate
  float bias;    // estimated gyro bias
  float rate;    // unbiased rate, exposed for debugging if needed

  float P[2][2]; // error covariance matrix
};

#endif
