#ifndef RATE_CONTROLLER_H
#define RATE_CONTROLLER_H

#include "pid.h"

// Rate Controller — Inner loop of cascaded PID
// Controls angular velocity (deg/s) around each axis
typedef struct {
    PIDController roll_pid;     // PID for roll rate
    PIDController pitch_pid;    // PID for pitch rate
    PIDController yaw_pid;      // PID for yaw rate
    
    // Output limits per axis
    float roll_limit;           // Max roll correction
    float pitch_limit;          // Max pitch correction
    float yaw_limit;            // Max yaw correction
} RateController;

// Initialize the rate controller
// dt: Time step (1.0/8000.0 for 8kHz)
void rate_controller_init(RateController *rc, float dt);

// Update the rate controller
// roll_setpoint:  Desired roll rate (rad/s)
// pitch_setpoint: Desired pitch rate (rad/s)
// yaw_setpoint:   Desired yaw rate (rad/s)
// roll_measured:  Current gyro roll rate (rad/s)
// pitch_measured: Current gyro pitch rate (rad/s)
// yaw_measured:   Current gyro yaw rate (rad/s)
// roll_corr:      Output roll correction
// pitch_corr:     Output pitch correction
// yaw_corr:       Output yaw correction
void rate_controller_update(RateController *rc,
                            float roll_setpoint, float pitch_setpoint, float yaw_setpoint,
                            float roll_measured, float pitch_measured, float yaw_measured,
                            float *roll_corr, float *pitch_corr, float *yaw_corr);

// Reset all PIDs in the rate controller
void rate_controller_reset(RateController *rc);

// Set gains for all axes at once (for tuning)
void rate_controller_set_gains(RateController *rc,
                               float roll_kp, float roll_ki, float roll_kd,
                               float pitch_kp, float pitch_ki, float pitch_kd,
                               float yaw_kp, float yaw_ki, float yaw_kd);

#endif // RATE_CONTROLLER_H