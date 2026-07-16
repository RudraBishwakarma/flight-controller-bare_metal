#include <stdio.h>
#include <stdint.h>
#include "../firmware/src/control/mixer.h"

#define TEST(name) printf("  %s: ", name)
#define PASS() printf("PASSED\n")
#define FAIL() printf("FAILED\n")

int main() {
    printf("=== MOTOR MIXER TESTS ===\n\n");
    
    // ==========================================
    // TEST 1: Hover (all corrections zero)
    // ==========================================
    printf("Hover Condition:\n");
    {
        uint16_t motors[MOTOR_COUNT];
        mixer_quadx(0.5f, 0.0f, 0.0f, 0.0f, motors);
        
        TEST("All motors at 1500 (mid-point)");
        if (motors[0] == 1500 && motors[1] == 1500 && 
            motors[2] == 1500 && motors[3] == 1500)
            PASS(); 
        else 
            FAIL();
    }
    
    // ==========================================
    // TEST 2: Roll Right
    // ==========================================
    printf("\nRoll Right:\n");
    {
        uint16_t motors[MOTOR_COUNT];
        mixer_quadx(0.5f, 0.2f, 0.0f, 0.0f, motors);
        
        // Left motors (M2, M3) should INCREASE
        // Right motors (M1, M4) should DECREASE
        TEST("Left motors increase, right motors decrease");
        if (motors[2] > 1500 && motors[3] < 1500 &&  // M3 up, M4 down
            motors[1] > 1500 && motors[0] < 1500)    // M2 up, M1 down
            PASS(); 
        else 
            FAIL();
    }
    
    // ==========================================
    // TEST 3: Pitch Forward
    // ==========================================
    printf("\nPitch Forward:\n");
    {
        uint16_t motors[MOTOR_COUNT];
        mixer_quadx(0.5f, 0.0f, 0.2f, 0.0f, motors);
        
        // Rear motors (M2, M4) should INCREASE
        // Front motors (M1, M3) should DECREASE
        TEST("Rear motors increase, front motors decrease");
        if (motors[1] > 1500 && motors[3] > 1500 &&  // Rear up
            motors[0] < 1500 && motors[2] < 1500)    // Front down
            PASS(); 
        else 
            FAIL();
    }
    
    // ==========================================
    // TEST 4: Yaw Right
    // ==========================================
    printf("\nYaw Right:\n");
    {
        uint16_t motors[MOTOR_COUNT];
        mixer_quadx(0.5f, 0.0f, 0.0f, 0.2f, motors);
        
        // CW motors (M1, M4) should INCREASE
        // CCW motors (M2, M3) should DECREASE
        TEST("CW motors increase, CCW motors decrease");
        if (motors[0] > 1500 && motors[3] > 1500 &&  // CW up
            motors[1] < 1500 && motors[2] < 1500)    // CCW down
            PASS(); 
        else 
            FAIL();
    }
    
    // ==========================================
    // TEST 5: Zero Throttle
    // ==========================================
    printf("\nZero Throttle:\n");
    {
        uint16_t motors[MOTOR_COUNT];
        mixer_quadx(0.0f, 0.0f, 0.0f, 0.0f, motors);
        
        TEST("All motors at PWM_MIN (1000)");
        if (motors[0] == 1000 && motors[1] == 1000 && 
            motors[2] == 1000 && motors[3] == 1000)
            PASS(); 
        else 
            FAIL();
    }
    
    // ==========================================
    // TEST 6: Full Throttle
    // ==========================================
    printf("\nFull Throttle:\n");
    {
        uint16_t motors[MOTOR_COUNT];
        mixer_quadx(1.0f, 0.0f, 0.0f, 0.0f, motors);
        
        TEST("All motors at PWM_MAX (2000)");
        if (motors[0] == 2000 && motors[1] == 2000 && 
            motors[2] == 2000 && motors[3] == 2000)
            PASS(); 
        else 
            FAIL();
    }
    
    // ==========================================
    // TEST 7: Saturation (motor values clamped)
    // ==========================================
    printf("\nSaturation Handling:\n");
    {
        uint16_t motors[MOTOR_COUNT];
        // Full throttle + full roll right
        // This would push M2 and M3 above 1.0
        mixer_quadx(1.0f, 0.5f, 0.0f, 0.0f, motors);
        
        TEST("No motor exceeds PWM_MAX (2000)");
        int all_ok = 1;
        for (int i = 0; i < MOTOR_COUNT; i++) {
            if (motors[i] > 2000) all_ok = 0;
        }
        if (all_ok) PASS(); else FAIL();
        
        TEST("No motor below PWM_MIN (1000)");
        all_ok = 1;
        for (int i = 0; i < MOTOR_COUNT; i++) {
            if (motors[i] < 1000) all_ok = 0;
        }
        if (all_ok) PASS(); else FAIL();
    }
    
    // ==========================================
    // TEST 8: Stop Function
    // ==========================================
    printf("\nStop Function:\n");
    {
        uint16_t motors[MOTOR_COUNT];
        mixer_stop(motors);
        
        TEST("All motors at PWM_MIN after stop");
        if (motors[0] == 1000 && motors[1] == 1000 && 
            motors[2] == 1000 && motors[3] == 1000)
            PASS(); 
        else 
            FAIL();
    }
    
    // ==========================================
    // TEST 9: Idle Function
    // ==========================================
    printf("\nIdle Function:\n");
    {
        uint16_t motors[MOTOR_COUNT];
        mixer_idle(motors);
        
        TEST("All motors at idle speed (1050)");
        if (motors[0] == 1050 && motors[1] == 1050 && 
            motors[2] == 1050 && motors[3] == 1050)
            PASS(); 
        else 
            FAIL();
    }
    
    // ==========================================
    // TEST 10: Throttle Clamp
    // ==========================================
    printf("\nThrottle Clamping:\n");
    {
        uint16_t motors[MOTOR_COUNT];
        // Negative throttle should be clamped to 0
        mixer_quadx(-0.5f, 0.0f, 0.0f, 0.0f, motors);
        
        TEST("Negative throttle clamped to minimum");
        if (motors[0] == 1000 && motors[1] == 1000 && 
            motors[2] == 1000 && motors[3] == 1000)
            PASS(); 
        else 
            FAIL();
        
        // Over-range throttle should be clamped to 1.0
        mixer_quadx(1.5f, 0.0f, 0.0f, 0.0f, motors);
        
        TEST("Over-range throttle clamped to maximum");
        if (motors[0] == 2000 && motors[1] == 2000 && 
            motors[2] == 2000 && motors[3] == 2000)
            PASS(); 
        else 
            FAIL();
    }
    
    printf("\n=== ALL TESTS COMPLETE ===\n");
    return 0;
}