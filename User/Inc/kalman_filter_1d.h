#ifndef __KALMAN_FILTER_1D_H__
#define __KALMAN_FILTER_1D_H__

#include <stddef.h>
#include <stdint.h>

/// @brief Filter structure.
typedef struct kalman_filter_1d_t {
  /// @brief State estimate.
  float x;
  /// @brief Error covariance.
  float P;
  /// @brief Process noise covariance.
  float Q;
  /// @brief Measurement noise covariance.
  float R;
} kalman_filter_1d_t;

/// @brief Initialize a kalman filter.
/// @param filter The filter to initialize.
/// @param Q The process noise covariance.
/// @param R The measurement noise covariance.
void kalman_filter_1d_init(kalman_filter_1d_t *filter, float Q, float R);

/// @brief Reset a kalman filter.
/// @param filter The filter to reset.
/// @param x The initial state estimate.
/// @param P The initial error covariance.
void kalman_filter_1d_reset(kalman_filter_1d_t *filter, float x, float P);

/// @brief Update a kalman filter.
/// @param filter The filter to update.
/// @param input The input value.
/// @return The output value.
float kalman_filter_1d_update(kalman_filter_1d_t *filter, float input);

#endif // __KALMAN_FILTER_1D_H__