#include "mixer.h"

void mixer_quadx(float throttle, float roll, float pitch, float yaw, 
                 uint16_t outputs[MOTOR_COUNT]) {
    
    // ==========================================
    // STEP 1: Constrain throttle to valid range
    // ==========================================
    if (throttle > 1.0f) throttle = 1.0f;
    if (throttle < 0.0f) throttle = 0.0f;
    
    // ==========================================
    // STEP 2: Calculate raw motor values
    // ==========================================
    // Each motor = throttle + roll_contribution + pitch_contribution + yaw_contribution
    //
    // Motor 1 (Front-Right): +Roll, -Pitch, +Yaw
    // Motor 2 (Rear-Left):   -Roll, +Pitch, +Yaw
    // Motor 3 (Front-Left):  +Roll, +Pitch, -Yaw
    // Motor 4 (Rear-Right):  -Roll, -Pitch, -Yaw
    
    float m1 = throttle + roll - pitch + yaw;
    float m2 = throttle - roll + pitch + yaw;
    float m3 = throttle + roll + pitch - yaw;
    float m4 = throttle - roll - pitch - yaw;
    
    // ==========================================
    // STEP 3: Find the maximum motor value
    // ==========================================
    // If any motor exceeds 1.0, we need to scale ALL motors down
    // to maintain the correct ratios while staying within limits
    
    float max_motor = m1;
    if (m2 > max_motor) max_motor = m2;
    if (m3 > max_motor) max_motor = m3;
    if (m4 > max_motor) max_motor = m4;
    
    // ==========================================
    // STEP 4: Scale if any motor exceeds 1.0
    // ==========================================
    // This preserves the RELATIVE differences between motors
    // while keeping all values within [0.0, 1.0]
    
    if (max_motor > 1.0f) {
        float scale = 1.0f / max_motor;
        m1 *= scale;
        m2 *= scale;
        m3 *= scale;
        m4 *= scale;
        
        // Throttle also needs to be scaled for consistency
        // This prevents the mixer from artificially boosting throttle
    }
    
    // ==========================================
    // STEP 5: Also check for negative values
    // ==========================================
    // If any motor goes negative, we could boost others,
    // but for safety we just clamp negative values to zero
    
    if (m1 < 0.0f) m1 = 0.0f;
    if (m2 < 0.0f) m2 = 0.0f;
    if (m3 < 0.0f) m3 = 0.0f;
    if (m4 < 0.0f) m4 = 0.0f;
    
    // ==========================================
    // STEP 6: Convert to PWM range (1000-2000)
    // ==========================================
    // m values are 0.0 to 1.0
    // PWM output = 1000 + m * 1000
    
    outputs[0] = (uint16_t)(PWM_MIN + m1 * 1000.0f);
    outputs[1] = (uint16_t)(PWM_MIN + m2 * 1000.0f);
    outputs[2] = (uint16_t)(PWM_MIN + m3 * 1000.0f);
    outputs[3] = (uint16_t)(PWM_MIN + m4 * 1000.0f);
    
    // ==========================================
    // STEP 7: Final safety clamp
    // ==========================================
    // Ensure no value exceeds PWM_MAX or goes below PWM_MIN
    // (shouldn't happen after scaling, but safety first)
    
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (outputs[i] > PWM_MAX) outputs[i] = PWM_MAX;
        if (outputs[i] < PWM_MIN) outputs[i] = PWM_MIN;
    }
}

void mixer_stop(uint16_t outputs[MOTOR_COUNT]) {
    // All motors to minimum (stopped)
    for (int i = 0; i < MOTOR_COUNT; i++) {
        outputs[i] = PWM_MIN;
    }
}

void mixer_idle(uint16_t outputs[MOTOR_COUNT]) {
    // All motors to idle speed (barely spinning)
    // Typically 1050-1075, depends on ESC calibration
    for (int i = 0; i < MOTOR_COUNT; i++) {
        outputs[i] = 1050;
    }
}