#include <stdio.h>
#include <math.h>
#include "../firmware/src/control/pid.h"
#include "../firmware/src/control/angle_controller.h"

#define TEST(name) printf("  %s: ", name)
#define PASS() printf("PASSED\n")
#define FAIL() printf("FAILED\n")

int float_eq(float a, float b, float tolerance) {
    float diff = a - b;
    if (diff < 0) diff = -diff;
    return diff < tolerance;
}

int main() {
    printf("=== ANGLE CONTROLLER TESTS ===\n\n");
    
    // Test 1: Initialization
    printf("Initialization:\n");
    {
        AngleController ac;
        angle_controller_init(&ac, 1.0f/1000.0f, 45.0f);
        
        TEST("Max angle set to 45°");
        if (float_eq(ac.max_angle, 45.0f * M_PI / 180.0f, 0.01f)) PASS(); else FAIL();
        
        TEST("Roll PID has kp=6.0");
        if (float_eq(ac.roll_pid.kp, 6.0f, 0.001f)) PASS(); else FAIL();
        
        TEST("Pitch PID has kp=6.0");
        if (float_eq(ac.pitch_pid.kp, 6.0f, 0.001f)) PASS(); else FAIL();
    }
    
    // Test 2: Level — no angle error
    printf("\nLevel (No Error):\n");
    {
        AngleController ac;
        angle_controller_init(&ac, 1.0f/1000.0f, 45.0f);
        
        float roll_rate, pitch_rate;
        angle_controller_update(&ac,
            0.0f, 0.0f,    // desired: level
            0.0f, 0.0f,    // actual: level
            &roll_rate, &pitch_rate);
        
        TEST("No rate command when level");
        if (float_eq(roll_rate, 0.0f, 0.01f) && float_eq(pitch_rate, 0.0f, 0.01f))
            PASS(); else FAIL();
    }
    
    // Test 3: Angle error produces rate command
    printf("\nAngle Error → Rate Command:\n");
    {
        AngleController ac;
        angle_controller_init(&ac, 1.0f/1000.0f, 45.0f);
        
        float roll_rate, pitch_rate;
        // Desired 30° roll, but drone is level (0°)
        float desired = 30.0f * M_PI / 180.0f;
        
        angle_controller_update(&ac,
            desired, 0.0f,   // want 30° roll
            0.0f, 0.0f,      // currently level
            &roll_rate, &pitch_rate);
        
        TEST("Positive rate command to reach target");
        if (roll_rate > 0.0f) PASS(); else FAIL();
        
        // Expected: 6.0 * 30° = 180°/s (but clamped to max_rate_output)
        // Actually: 6.0 * 0.523 rad = 3.14 rad/s ≈ 180°/s
        // That exceeds max_rate_output of 200°/s (3.49 rad/s)? No, 180 < 200
        TEST("Roll rate is reasonable (not zero, not insane)");
        if (roll_rate > 0.1f && roll_rate < 10.0f) PASS(); else FAIL();
    }
    
    // Test 4: Angle setpoint clamped
    printf("\nAngle Clamping:\n");
    {
        AngleController ac;
        angle_controller_init(&ac, 1.0f/1000.0f, 45.0f);
        
        float roll_rate, pitch_rate;
        // Request 90° (beyond 45° limit)
        float too_much = 90.0f * M_PI / 180.0f;
        
        angle_controller_update(&ac,
            too_much, 0.0f,   // want 90° — should be clamped to 45°
            0.0f, 0.0f,       // level
            &roll_rate, &pitch_rate);
        
        // Should still produce a rate command (for 45°, not 90°)
        TEST("Still produces output when setpoint clamped");
        if (roll_rate > 0.0f) PASS(); else FAIL();
    }
    
    // Test 5: Rate output clamped
    printf("\nRate Output Clamping:\n");
    {
        AngleController ac;
        angle_controller_init(&ac, 1.0f/1000.0f, 45.0f);
        
        float roll_rate, pitch_rate;
        // Max angle error with kp=6 → tries to output very high rate
        float max_angle = 45.0f * M_PI / 180.0f;
        
        angle_controller_update(&ac,
            max_angle, 0.0f,   // max angle error
            -max_angle, 0.0f,  // drone tilted opposite way (double error!)
            &roll_rate, &pitch_rate);
        
        float max_rate = 200.0f * M_PI / 180.0f;  // 200°/s in rad/s
        TEST("Roll rate clamped to max (200°/s)");
        if (roll_rate <= max_rate + 0.01f && roll_rate >= -max_rate - 0.01f)
            PASS(); else FAIL();
    }
    
    // Test 6: Reset
    printf("\nReset:\n");
    {
        AngleController ac;
        angle_controller_init(&ac, 1.0f/1000.0f, 45.0f);
        
        // Build up state
        for (int i = 0; i < 50; i++) {
            float rr, pr;
            angle_controller_update(&ac, 0.5f, 0.5f, 0.0f, 0.0f, &rr, &pr);
        }
        
        angle_controller_reset(&ac);
        
        TEST("Roll integral zero after reset");
        if (float_eq(ac.roll_pid.integral, 0.0f, 0.001f)) PASS(); else FAIL();
    }
    
    printf("\n=== ALL TESTS COMPLETE ===\n");
    return 0;
}