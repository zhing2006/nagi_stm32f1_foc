#include "kalman_filter_1d.h"

void kalman_filter_1d_init(kalman_filter_1d_t *filter, float Q, float R) {
  filter->x = 0.0f;
  filter->P = 1.0f;
  filter->Q = Q;
  filter->R = R;
}

void kalman_filter_1d_reset(kalman_filter_1d_t *filter, float x, float P) {
  filter->x = x;
  filter->P = P;
}

float kalman_filter_1d_update(kalman_filter_1d_t *filter, float input) {
  // Prediction.
  float x_predict = filter->x;
  float P_predict = filter->P + filter->Q;

  // Gain.
  float K = P_predict / (P_predict + filter->R);;

  // Update estimate.
  filter->x = x_predict + K * (input - x_predict);

  // Update error covariance.
  filter->P = (1 - K) * P_predict;

  return filter->x;
}