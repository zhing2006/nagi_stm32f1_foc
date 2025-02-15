#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include <stddef.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI (3.14159265358979)
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (0.017453292519943295)
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (57.29577951308232)
#endif

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifndef CLAMP
#define CLAMP(x, min, max) (MIN(MAX(x, min), max))
#endif

extern uint8_t g_mt6701_rx_data[3];

typedef struct nagi_foc_motor_t nagi_foc_motor_t;
extern nagi_foc_motor_t g_motor;
typedef struct kalman_filter_1d_t kalman_filter_1d_t;
extern kalman_filter_1d_t g_speed_filter;
typedef struct low_pass_filter_t low_pass_filter_t;
extern low_pass_filter_t g_current_d_filter;
extern low_pass_filter_t g_current_q_filter;

/// @brief FOC motor control type.
typedef enum control_type_t {
  CONTROL_TYPE_NONE, ///< No control.
  CONTROL_TYPE_POSITION, ///< Position control.
  CONTROL_TYPE_SPEED, ///< Speed control.
  CONTROL_TYPE_CURRENT, ///< Current control.
  CONTROL_TYPE_SPEED_CURRENT, ///< Speed and current control.
  CONTROL_TYPE_POSITION_SPEED_CURRENT, ///< Position, speed, and current control.
} control_type_t;

/// @brief FOC motor control context.
typedef struct control_context_t {
  /// @brief Control type.
  control_type_t type;
  /// @brief Target position in radians.
  float position;
  /// @brief Target speed.
  float speed;
  /// @brief Target d axis torque.
  float current_norm_d;
  /// @brief Target q axis torque.
  float current_norm_q;
  /// @brief Max speed for cascade control.
  float max_speed;
  /// @brief Max current for cascade control.
  float max_current;
} control_context_t;

extern control_context_t g_control_context;

#endif // __GLOBAL_H__