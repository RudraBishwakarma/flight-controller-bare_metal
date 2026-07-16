#include "angle_controller.h"
#include <math.h>

void angle_controller_init(AngleController *ac, float dt, float max_angle_deg) {
    
    // Store max angle in radians
    ac->max_angle = max_angle_deg * M_PI / 180.0f;
    
    // Maximum rate the angle controller can request (deg/s → rad/s)
    ac->max_rate_output = 200.0f * M_PI / 180.0f;  // 200°/s max
    
    // ==========================================
    // Roll angle PID
    // ==========================================
    // Usually only P-gain, no I or D
    // The rate controller handles I and D
    pid_init(&ac->roll_pid,
             6.0f,    // kp — converts angle error to rate
             0.0f,    // ki — usually 0 for angle loop
             0.0f,    // kd — usually 0 for angle loop
             50.0f,   // integral_limit
             200.0f,  // output_limit (max rate in °/s)
             dt);
    
    // ==========================================
    // Pitch angle PID (same as roll)
    // ==========================================
    pid_init(&ac->pitch_pid,
             6.0f,    // kp
             0.0f,    // ki
             0.0f,    // kd
             50.0f,   // integral_limit
             200.0f,  // output_limit
             dt);
}

void angle_controller_update(AngleController *ac,
                             float roll_setpoint, float pitch_setpoint,
                             float roll_measured, float pitch_measured,
                             float *roll_rate_set, float *pitch_rate_set) {
    
    // ==========================================
    // Clamp setpoints to max angle
    // ==========================================
    // Prevent commanding impossible angles
    if (roll_setpoint > ac->max_angle)   roll_setpoint = ac->max_angle;
    if (roll_setpoint < -ac->max_angle)  roll_setpoint = -ac->max_angle;
    if (pitch_setpoint > ac->max_angle)  pitch_setpoint = ac->max_angle;
    if (pitch_setpoint < -ac->max_angle) pitch_setpoint = -ac->max_angle;
    
    // ==========================================
    // Roll angle → Roll rate
    // ==========================================
    // Error = desired_angle - actual_angle
    // Output = how fast to rotate to fix the error
    
    *roll_rate_set = pid_update(&ac->roll_pid, roll_setpoint, roll_measured);
    
    // Clamp rate output
    if (*roll_rate_set > ac->max_rate_output)  *roll_rate_set = ac->max_rate_output;
    if (*roll_rate_set < -ac->max_rate_output) *roll_rate_set = -ac->max_rate_output;
    
    // ==========================================
    // Pitch angle → Pitch rate
    // ==========================================
    *pitch_rate_set = pid_update(&ac->pitch_pid, pitch_setpoint, pitch_measured);
    
    if (*pitch_rate_set > ac->max_rate_output)  *pitch_rate_set = ac->max_rate_output;
    if (*pitch_rate_set < -ac->max_rate_output) *pitch_rate_set = -ac->max_rate_output;
}

void angle_controller_reset(AngleController *ac) {
    pid_reset(&ac->roll_pid);
    pid_reset(&ac->pitch_pid);
}

void angle_controller_set_gains(AngleController *ac,
                                float roll_kp, float roll_ki, float roll_kd,
                                float pitch_kp, float pitch_ki, float pitch_kd) {
    pid_set_gains(&ac->roll_pid,  roll_kp,  roll_ki,  roll_kd);
    pid_set_gains(&ac->pitch_pid, pitch_kp, pitch_ki, pitch_kd);
}