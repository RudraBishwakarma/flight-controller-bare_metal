#include <stdio.h>
#include <math.h>
#include "../firmware/src/control/pid.h"
#include "../firmware/src/control/rate_controller.h"

#define TEST(name) printf("  %s: ", name)
#define PASS() printf("PASSED\n")
#define FAIL() printf("FAILED\n")

int float_eq(float a, float b, float tolerance) {
    float diff = a - b;
    if (diff < 0) diff = -diff;
    return diff < tolerance;
}

int main() {
    printf("=== RATE CONTROLLER TESTS ===\n\n");
    
    // Test 1: Initialization
    printf("Initialization:\n");
    {
        RateController rc;
        rate_controller_init(&rc, 1.0f/8000.0f);
        
        TEST("Roll PID has correct kp (4.0)");
        if (float_eq(rc.roll_pid.kp, 4.0f, 0.001f)) PASS(); else FAIL();
        
        TEST("Pitch PID has correct kp (4.0)");
        if (float_eq(rc.pitch_pid.kp, 4.0f, 0.001f)) PASS(); else FAIL();
        
        TEST("Yaw PID has lower kp (3.0)");
        if (float_eq(rc.yaw_pid.kp, 3.0f, 0.001f)) PASS(); else FAIL();
    }
    
    // Test 2: Zero error produces near-zero correction
    printf("\nZero Error (Hover):\n");
    {
        RateController rc;
        rate_controller_init(&rc, 1.0f/8000.0f);
        
        float r, p, y;
        rate_controller_update(&rc,
            0.0f, 0.0f, 0.0f,   // setpoints: no rotation desired
            0.0f, 0.0f, 0.0f,   // measurements: not rotating
            &r, &p, &y);
        
        TEST("Roll correction near zero when hovering");
        if (float_eq(r, 0.0f, 0.01f)) PASS(); else FAIL();
        
        TEST("Pitch correction near zero when hovering");
        if (float_eq(p, 0.0f, 0.01f)) PASS(); else FAIL();
        
        TEST("Yaw correction near zero when hovering");
        if (float_eq(y, 0.0f, 0.01f)) PASS(); else FAIL();
    }
    
    // Test 3: Positive error produces positive correction
    printf("\nPositive Error:\n");
    {
        RateController rc;
        rate_controller_init(&rc, 1.0f/8000.0f);
        
        float r, p, y;
        // Want to roll at 100°/s, but currently at 0°/s
        float roll_set = 100.0f * M_PI / 180.0f;  // 1.745 rad/s
        
        rate_controller_update(&rc,
            roll_set, 0.0f, 0.0f,   // setpoints
            0.0f, 0.0f, 0.0f,       // measurements
            &r, &p, &y);
        
        TEST("Roll correction positive when rate too low");
        if (r > 0.0f) PASS(); else FAIL();
        
        TEST("Pitch correction near zero (no pitch error)");
        if (float_eq(p, 0.0f, 0.1f)) PASS(); else FAIL();
    }
    
    // Test 4: Output respects limits
    printf("\nOutput Limits:\n");
    {
        RateController rc;
        rate_controller_init(&rc, 1.0f/8000.0f);
        
        float r, p, y;
        // Huge error to try to push output beyond limits
        rate_controller_update(&rc,
            1000.0f, 1000.0f, 1000.0f,   // huge setpoints
            0.0f, 0.0f, 0.0f,             // zero measurement
            &r, &p, &y);
        
        TEST("Roll correction limited to 400");
        if (r <= 400.0f && r >= -400.0f) PASS(); else FAIL();
        
        TEST("Pitch correction limited to 400");
        if (p <= 400.0f && p >= -400.0f) PASS(); else FAIL();
        
        TEST("Yaw correction limited to 300");
        if (y <= 300.0f && y >= -300.0f) PASS(); else FAIL();
    }
    
    // Test 5: Reset clears all PIDs
    printf("\nReset:\n");
    {
        RateController rc;
        rate_controller_init(&rc, 1.0f/8000.0f);
        
        // Build up integral
        for (int i = 0; i < 100; i++) {
            float r, p, y;
            rate_controller_update(&rc,
                5.0f, 5.0f, 5.0f,   // constant error
                0.0f, 0.0f, 0.0f,
                &r, &p, &y);
        }
        
        rate_controller_reset(&rc);
        
        TEST("Roll integral zero after reset");
        if (float_eq(rc.roll_pid.integral, 0.0f, 0.001f)) PASS(); else FAIL();
        
        TEST("Pitch integral zero after reset");
        if (float_eq(rc.pitch_pid.integral, 0.0f, 0.001f)) PASS(); else FAIL();
        
        TEST("Yaw integral zero after reset");
        if (float_eq(rc.yaw_pid.integral, 0.0f, 0.001f)) PASS(); else FAIL();
    }
    
    printf("\n=== ALL TESTS COMPLETE ===\n");
    return 0;
}