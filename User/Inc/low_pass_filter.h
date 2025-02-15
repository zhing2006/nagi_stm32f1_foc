#ifndef __LOW_PASS_FILTER_H__
#define __LOW_PASS_FILTER_H__

#include <stddef.h>
#include <stdint.h>

/// @brief Filter structure.
typedef struct low_pass_filter_t {
  /// @brief The alpha value.
  float alpha;
  /// @brief The previous output.
  float x;
} low_pass_filter_t;

/// @brief Initialize a low-pass filter.
/// @param filter The filter to initialize.
/// @param cutoff_freq The cutoff frequency.
/// @param sample_time The sample time(second).
void low_pass_filter_init(low_pass_filter_t *filter, float cutoff_freq, float sample_time);

/// @brief Initialize a low-pass filter.
/// @param filter The filter to initialize.
/// @param alpha The alpha value.
void low_pass_filter_init_with_alpha(low_pass_filter_t *filter, float alpha);

/// @brief Reset a low-pass filter.
/// @param filter The filter to reset.
/// @param input The input value.
void low_pass_filter_reset(low_pass_filter_t *filter, float input);

/// @brief Update a low-pass filter.
/// @param filter The filter to update.
/// @param input The input value.
/// @return The output value.
float low_pass_filter_update(low_pass_filter_t *filter, float input);

#endif // __LOW_PASS_FILTER_H__