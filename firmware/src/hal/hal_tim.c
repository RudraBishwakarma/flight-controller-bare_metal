#include "hal_tim.h"
#include "config/target_qemu.h"

static uint16_t motor_values[4] = {1000, 1000, 1000, 1000};

void motor_timer_init(void) {
#if TARGET_QEMU
    DEBUG_PRINT(3, "Motor Timer: Simulated init");
#else
    DEBUG_PRINT(3, "Motor Timer: TIM1 init for 4 motors");
    // Real timer initialization for PWM/DShot
#endif
    motor_stop_all();
}

void motor_set_pwm(uint8_t channel, uint16_t value) {
    if (channel < 4) {
        if (value < PWM_MIN_VALUE) value = PWM_MIN_VALUE;
        if (value > PWM_MAX_VALUE) value = PWM_MAX_VALUE;
        motor_values[channel] = value;
    }
}

void motor_set_all(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
    motor_set_pwm(0, m1);
    motor_set_pwm(1, m2);
    motor_set_pwm(2, m3);
    motor_set_pwm(3, m4);
}

void motor_stop_all(void) {
    for (int i = 0; i < 4; i++) motor_values[i] = PWM_MIN_VALUE;
}

void motor_arm(void) {
    for (int i = 0; i < 4; i++) motor_values[i] = PWM_IDLE_VALUE;
}

uint16_t motor_get_value(uint8_t channel) {
    return (channel < 4) ? motor_values[channel] : 0;
}