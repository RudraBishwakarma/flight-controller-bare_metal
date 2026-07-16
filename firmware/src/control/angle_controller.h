#ifndef ANGLE_CONTROLLER_H
#define ANGLE_CONTROLLER_H

#include "pid.h"

// Angle Controller — Outer loop of cascaded PID
// Controls absolute orientation angle (degrees or radians)
typedef struct {
    PIDController roll_pid;      // PID for roll angle
    PIDController pitch_pid;     // PID for pitch angle
    
    float max_angle;             // Maximum allowed angle (e.g., 45° = 0.785 rad)
    float max_rate_output;       // Maximum rate command sent to rate controller
} AngleController;

// Initialize the angle controller
// dt: Time step (1.0/1000.0 for 1kHz)
// max_angle_deg: Maximum tilt angle in degrees (e.g., 45.0)
void angle_controller_init(AngleController *ac, float dt, float max_angle_deg);

// Update the angle controller
// roll_setpoint:  Desired roll angle (radians)
// pitch_setpoint: Desired pitch angle (radians)
// roll_measured:  Current roll from Mahony filter (radians)
// pitch_measured: Current pitch from Mahony filter (radians)
// roll_rate_set:  Output — desired roll rate for rate controller (rad/s)
// pitch_rate_set: Output — desired pitch rate for rate controller (rad/s)
void angle_controller_update(AngleController *ac,
                             float roll_setpoint, float pitch_setpoint,
                             float roll_measured, float pitch_measured,
                             float *roll_rate_set, float *pitch_rate_set);

// Reset all PIDs
void angle_controller_reset(AngleController *ac);

// Set gains
void angle_controller_set_gains(AngleController *ac,
                                float roll_kp, float roll_ki, float roll_kd,
                                float pitch_kp, float pitch_ki, float pitch_kd);

#endif // ANGLE_CONTROLLER_H