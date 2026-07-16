#ifndef MIXER_H
#define MIXER_H

#include <stdint.h>

// Number of motors
#define MOTOR_COUNT 4

// PWM limits (standard for ESCs)
#define PWM_MIN  1000   // Motor stopped
#define PWM_MAX  2000   // Motor at full throttle
#define PWM_MID  1500   // Motor at half throttle (hover point)

// Mix a set of control inputs into motor outputs
// throttle:  0.0 to 1.0 (0% to 100% throttle)
// roll:      Roll correction from PID (-1.0 to +1.0)
// pitch:     Pitch correction from PID (-1.0 to +1.0)
// yaw:       Yaw correction from PID (-1.0 to +1.0)
// outputs:   Array of 4 uint16_t (1000-2000 PWM values)
void mixer_quadx(float throttle, float roll, float pitch, float yaw, 
                 uint16_t outputs[MOTOR_COUNT]);

// Stop all motors (disarm)
void mixer_stop(uint16_t outputs[MOTOR_COUNT]);

// Set all motors to idle (armed, ready to fly)
void mixer_idle(uint16_t outputs[MOTOR_COUNT]);

#endif // MIXER_H