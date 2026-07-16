#ifndef PID_H
#define PID_H

// PID Controller structure
// Holds all the gains, limits, and state for one axis
typedef struct {
    // User-set gains
    float kp;              // Proportional gain
    float ki;              // Integral gain
    float kd;              // Derivative gain
    
    // Limits
    float integral_limit;  // Maximum absolute value of integral term
    float output_limit;    // Maximum absolute value of output
    
    // State (changes every update)
    float integral;        // Accumulated integral term
    float prev_error;      // Previous error (for derivative)
    float prev_measurement; // Previous measurement (for D-on-measurement)
    
    // Timing
    float dt;              // Time step in seconds
} PIDController;

// Initialize a PID controller with given gains and limits
// Call this ONCE before using pid_update
void pid_init(PIDController *pid, 
              float kp, float ki, float kd,
              float integral_limit, 
              float output_limit, 
              float dt);

// Update the PID controller — call this every control loop iteration
// setpoint:     What you WANT (desired angle/rate)
// measurement:  What you HAVE (current angle/rate)
// Returns:      Correction value (clamped to output_limit)
float pid_update(PIDController *pid, float setpoint, float measurement);

// Reset the PID controller state (integral and previous values)
// Call when changing flight modes or after disarming
void pid_reset(PIDController *pid);

// Set new gains at runtime (for tuning)
void pid_set_gains(PIDController *pid, float kp, float ki, float kd);

// Get current integral value (for debugging)
float pid_get_integral(const PIDController *pid);

#endif // PID_H