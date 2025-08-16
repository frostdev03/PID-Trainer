#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter {
public:
  KalmanFilter();
  float getAngle(float newAngle, float newRate, float dt);
    void setAngle(float angle); // tambah ini

private:
  float Q_angle;
  float Q_bias;
  float R_measure;

  float angle;
  float bias;
  float rate;

  float P[2][2];
};

#endif
