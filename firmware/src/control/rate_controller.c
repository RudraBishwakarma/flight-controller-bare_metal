#include "rate_controller.h"

void rate_controller_init(RateController *rc, float dt) {
    
    // ==========================================
    // Roll rate PID
    // ==========================================
    pid_init(&rc->roll_pid, 
             4.0f,    // kp — aggressive response for roll
             0.3f,    // ki — small integral to fix steady error
             18.0f,   // kd — strong damping to prevent oscillation
             100.0f,  // integral_limit
             400.0f,  // output_limit
             dt);
    
    // ==========================================
    // Pitch rate PID (typically same as roll)
    // ==========================================
    pid_init(&rc->pitch_pid,
             4.0f,    // kp
             0.3f,    // ki
             18.0f,   // kd
             100.0f,  // integral_limit
             400.0f,  // output_limit
             dt);
    
    // ==========================================
    // Yaw rate PID (usually different — less authority)
    // ==========================================
    pid_init(&rc->yaw_pid,
             3.0f,    // kp — lower gain (yaw has less torque)
             0.2f,    // ki
             0.0f,    // kd — no D-term for yaw (too noisy)
             80.0f,   // integral_limit
             300.0f,  // output_limit — less than roll/pitch
             dt);
    
    // Output limits
    rc->roll_limit = 400.0f;
    rc->pitch_limit = 400.0f;
    rc->yaw_limit = 300.0f;
}

void rate_controller_update(RateController *rc,
                            float roll_setpoint, float pitch_setpoint, float yaw_setpoint,
                            float roll_measured, float pitch_measured, float yaw_measured,
                            float *roll_corr, float *pitch_corr, float *yaw_corr) {
    
    // ==========================================
    // Roll rate control
    // ==========================================
    // Setpoint: How fast we WANT to roll
    // Measurement: How fast the gyro says we ARE rolling
    // Output: Correction to make them match
    
    *roll_corr = pid_update(&rc->roll_pid, roll_setpoint, roll_measured);
    
    // Clamp to axis limit
    if (*roll_corr > rc->roll_limit)  *roll_corr = rc->roll_limit;
    if (*roll_corr < -rc->roll_limit) *roll_corr = -rc->roll_limit;
    
    // ==========================================
    // Pitch rate control
    // ==========================================
    *pitch_corr = pid_update(&rc->pitch_pid, pitch_setpoint, pitch_measured);
    
    if (*pitch_corr > rc->pitch_limit)  *pitch_corr = rc->pitch_limit;
    if (*pitch_corr < -rc->pitch_limit) *pitch_corr = -rc->pitch_limit;
    
    // ==========================================
    // Yaw rate control
    // ==========================================
    *yaw_corr = pid_update(&rc->yaw_pid, yaw_setpoint, yaw_measured);
    
    if (*yaw_corr > rc->yaw_limit)  *yaw_corr = rc->yaw_limit;
    if (*yaw_corr < -rc->yaw_limit) *yaw_corr = -rc->yaw_limit;
}

void rate_controller_reset(RateController *rc) {
    pid_reset(&rc->roll_pid);
    pid_reset(&rc->pitch_pid);
    pid_reset(&rc->yaw_pid);
}

void rate_controller_set_gains(RateController *rc,
                               float roll_kp, float roll_ki, float roll_kd,
                               float pitch_kp, float pitch_ki, float pitch_kd,
                               float yaw_kp, float yaw_ki, float yaw_kd) {
    pid_set_gains(&rc->roll_pid,  roll_kp,  roll_ki,  roll_kd);
    pid_set_gains(&rc->pitch_pid, pitch_kp, pitch_ki, pitch_kd);
    pid_set_gains(&rc->yaw_pid,   yaw_kp,   yaw_ki,   yaw_kd);
}