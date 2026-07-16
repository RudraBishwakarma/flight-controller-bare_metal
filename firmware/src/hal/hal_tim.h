#ifndef HAL_TIM_H
#define HAL_TIM_H

#include <stdint.h>

// Timer channels for motor output
#define MOTOR_TIMER_CH1    0
#define MOTOR_TIMER_CH2    1
#define MOTOR_TIMER_CH3    2
#define MOTOR_TIMER_CH4    3

// Initialize timer for motor PWM/DShot output
// Uses TIM1 with 4 channels for 4 motors
void motor_timer_init(void);

// Set PWM value for one motor channel
// channel: 0-3 (CH1-CH4)
// value: 1000-2000 (PWM pulse width in microseconds)
void motor_set_pwm(uint8_t channel, uint16_t value);

// Set all 4 motors at once
void motor_set_all(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

// Stop all motors (set to minimum)
void motor_stop_all(void);

// Arm motors (set to idle speed)
void motor_arm(void);

// Get current motor value (for telemetry)
uint16_t motor_get_value(uint8_t channel);

#endif // HAL_TIM_H