#include "pid.h"

void pid_init(PIDController *pid, 
              float kp, float ki, float kd,
              float integral_limit, 
              float output_limit, 
              float dt) {
    
    // Store the gains
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    
    // Store the limits
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    
    // Store the time step
    pid->dt = dt;
    
    // Reset the state
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
}

float pid_update(PIDController *pid, float setpoint, float measurement) {
    
    // ==========================================
    // STEP 1: Calculate error
    // ==========================================
    // Positive error = need to increase output
    // Negative error = need to decrease output
    float error = setpoint - measurement;
    
    
    // ==========================================
    // STEP 2: PROPORTIONAL TERM
    // ==========================================
    // Direct reaction to current error
    // Bigger error → bigger correction
    float p_term = pid->kp * error;
    
    
    // ==========================================
    // STEP 3: INTEGRAL TERM
    // ==========================================
    // Accumulate error over time
    pid->integral += error * pid->dt;
    
    // Anti-windup: clamp integral to prevent excessive buildup
    // Without this, integral can grow huge during long maneuvers
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    
    // Calculate integral contribution
    float i_term = pid->ki * pid->integral;
    
    
    // ==========================================
    // STEP 4: DERIVATIVE TERM (on measurement)
    // ==========================================
    // Use "derivative on measurement" instead of "derivative on error"
    // This prevents "derivative kick" when setpoint changes suddenly
    //
    // Derivative on error: d(error)/dt — setpoint changes cause spikes
    // Derivative on measurement: d(measurement)/dt — smoother
    
    float derivative = (measurement - pid->prev_measurement) / pid->dt;
    
    // Note: derivative of measurement = -derivative of error (when setpoint constant)
    // We negate it to get the correct sign
    float d_term = -pid->kd * derivative;
    
    
    // ==========================================
    // STEP 5: Combine all terms
    // ==========================================
    float output = p_term + i_term + d_term;
    
    
    // ==========================================
    // STEP 6: Clamp output to limits
    // ==========================================
    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }
    
    
    // ==========================================
    // STEP 7: Save state for next iteration
    // ==========================================
    pid->prev_error = error;
    pid->prev_measurement = measurement;
    
    
    return output;
}

void pid_reset(PIDController *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
}

void pid_set_gains(PIDController *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

float pid_get_integral(const PIDController *pid) {
    return pid->integral;
}