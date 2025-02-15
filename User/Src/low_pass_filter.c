#include "low_pass_filter.h"

#include <math.h>

void low_pass_filter_init(low_pass_filter_t *filter, float cutoff_freq, float sample_time) {
  const float rc = 1.0f / (2.0f * M_PI * cutoff_freq);
  filter->alpha = sample_time / (rc + sample_time);
  filter->x = 0.0f;
}

void low_pass_filter_init_with_alpha(low_pass_filter_t *filter, float alpha) {
  filter->alpha = alpha;
  filter->x = 0.0f;
}

void low_pass_filter_reset(low_pass_filter_t *filter, float input) {
  filter->x = input;
}

float low_pass_filter_update(low_pass_filter_t *filter, float input) {
  filter->x = filter->alpha * input + (1.0f - filter->alpha) * filter->x;
  return filter->x;
}