// カルマンフィルタ。
// 2016/3/3
//
// 計算式の参考資料。
// http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/

#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() {
  angle = 0.0;
  bias = 0.0;
  P[0][0] = 0.0;
  P[0][1] = 0.0;
  P[1][0] = 0.0;
  P[1][1] = 0.0;
};

void KalmanFilter::setAngle(float newAngle) {
  angle = newAngle;
};

float KalmanFilter::calcAngle(float newAngle, float newRate, float dt) {

  // variances
  float Q_angle = 0.001;
  float Q_bias = 0.003;
  float R_measure = 0.03;

  // step 1
  float rate = newRate - bias;
  angle += dt * rate;

  // step 2
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // step 3
  float y = newAngle - angle;

  // step 4
  float S = P[0][0] + R_measure;

  // step 5
  float K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  // step 6
  angle += K[0] * y;
  bias += K[1] * y;

  // step 7
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return angle;
};