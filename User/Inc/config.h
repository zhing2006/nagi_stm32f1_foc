#ifndef __CONFIG_H__
#define __CONFIG_H__

#define MOTOR_POLE_PAIRS (7)
#define MOTOR_VOLTAGE_LIMIT (12)
#define MOTOR_POSITION_CYCLE (6 * 3.14159265358979)
#define MOTOR_PWM_FREQ (28000)
#define MOTOR_LD (0.0026)
#define MOTOR_LQ (0.0026)

#define SUPPLY_VOLTAGE (12.6)
#define CURRENT_R_SHUNT (0.02)
#define CURRENT_OP_GAIN (50)
#define ADC_REF_VOLTAGE (3.3)
#define ADC_BITS (12)

#define SPEED_CALC_FREQ (930)

#endif // __CONFIG_H__