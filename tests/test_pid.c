#include <stdio.h>
#include <math.h>
#include "../firmware/src/control/pid.h"

#define TEST(name) printf("  %s: ", name)
#define PASS() printf("PASSED\n")
#define FAIL() printf("FAILED\n")

int float_eq(float a, float b, float tolerance) {
    float diff = a - b;
    if (diff < 0) diff = -diff;
    return diff < tolerance;
}

int main() {
    printf("=== PID CONTROLLER TESTS ===\n\n");
    
    // ==========================================
    // TEST 1: Initialization
    // ==========================================
    printf("Initialization:\n");
    {
        PIDController pid;
        pid_init(&pid, 2.0f, 0.5f, 0.1f, 100.0f, 400.0f, 0.01f);
        
        TEST("kp stored correctly");
        if (float_eq(pid.kp, 2.0f, 0.001f)) PASS(); else FAIL();
        
        TEST("ki stored correctly");
        if (float_eq(pid.ki, 0.5f, 0.001f)) PASS(); else FAIL();
        
        TEST("kd stored correctly");
        if (float_eq(pid.kd, 0.1f, 0.001f)) PASS(); else FAIL();
        
        TEST("integral starts at zero");
        if (float_eq(pid.integral, 0.0f, 0.001f)) PASS(); else FAIL();
        
        TEST("prev_error starts at zero");
        if (float_eq(pid.prev_error, 0.0f, 0.001f)) PASS(); else FAIL();
    }
    
    // ==========================================
    // TEST 2: Proportional Only (ki=0, kd=0)
    // ==========================================
    printf("\nProportional Term:\n");
    {
        PIDController pid;
        pid_init(&pid, 2.0f, 0.0f, 0.0f, 100.0f, 400.0f, 0.01f);
        
        // Error = 10, kp = 2 → output should be 20
        float output = pid_update(&pid, 10.0f, 0.0f);
        TEST("P-only: 10° error, kp=2 → output=20");
        if (float_eq(output, 20.0f, 0.01f)) PASS(); else FAIL();
        
        // Error = 5, kp = 2 → output should be 10
        output = pid_update(&pid, 5.0f, 0.0f);
        TEST("P-only: 5° error, kp=2 → output=10");
        if (float_eq(output, 10.0f, 0.01f)) PASS(); else FAIL();
        
        // Error = -5, kp = 2 → output should be -10
        output = pid_update(&pid, 0.0f, 5.0f);
        TEST("P-only: -5° error, kp=2 → output=-10");
        if (float_eq(output, -10.0f, 0.01f)) PASS(); else FAIL();
    }
    
    // ==========================================
    // TEST 3: Integral Buildup
    // ==========================================
    printf("\nIntegral Term:\n");
    {
        PIDController pid;
        pid_init(&pid, 0.0f, 1.0f, 0.0f, 100.0f, 400.0f, 0.01f);
        
        // Apply constant error for 10 iterations
        // Each iteration: error=2, dt=0.01
        // integral = 2*0.01*10 = 0.2
        // i_term = 1.0 * 0.2 = 0.2
        float output = 0;
        for (int i = 0; i < 10; i++) {
            output = pid_update(&pid, 2.0f, 0.0f);
        }
        
        TEST("I-only: 2° error for 0.1s → integral=0.2, output=0.2");
        if (float_eq(output, 0.2f, 0.01f)) PASS(); else FAIL();
    }
    
    // ==========================================
    // TEST 4: Integral Anti-Windup
    // ==========================================
    printf("\nIntegral Anti-Windup:\n");
    {
        PIDController pid;
        pid_init(&pid, 0.0f, 1.0f, 0.0f, 10.0f, 400.0f, 0.01f);
        
        // Apply huge error for many iterations
        // integral should be clamped at limit (10.0)
        for (int i = 0; i < 1000; i++) {
            pid_update(&pid, 100.0f, 0.0f);
        }
        
        TEST("Integral clamped at limit (10.0)");
        if (float_eq(pid.integral, 10.0f, 0.01f)) PASS(); else FAIL();
    }
    
    // ==========================================
    // TEST 5: Output Clamping
    // ==========================================
    printf("\nOutput Clamping:\n");
    {
        PIDController pid;
        pid_init(&pid, 100.0f, 0.0f, 0.0f, 100.0f, 50.0f, 0.01f);
        
        // Error = 10, kp = 100 → raw output = 1000
        // Should be clamped to 50
        float output = pid_update(&pid, 10.0f, 0.0f);
        TEST("Output clamped at 50.0 when raw output=1000");
        if (float_eq(output, 50.0f, 0.01f)) PASS(); else FAIL();
        
        // Negative clamping
        output = pid_update(&pid, 0.0f, 10.0f);
        TEST("Output clamped at -50.0 when raw output=-1000");
        if (float_eq(output, -50.0f, 0.01f)) PASS(); else FAIL();
    }
    
    // ==========================================
    // TEST 6: Derivative Term
    // ==========================================
    printf("\nDerivative Term:\n");
    {
        PIDController pid;
        pid_init(&pid, 0.0f, 0.0f, 1.0f, 100.0f, 400.0f, 0.01f);
        
        // First call: measurement = 0
        pid_update(&pid, 0.0f, 0.0f);
        
        // Second call: measurement = 10 (rapid change)
        // derivative = (10-0)/0.01 = 1000
        // d_term = -1.0 * 1000 = -1000 (opposes the change)
        float output = pid_update(&pid, 0.0f, 10.0f);
        TEST("D-only: measurement jumps 0→10, output opposes change");
        if (float_eq(output, -1000.0f, 10.0f)) PASS(); else FAIL();
    }
    
    // ==========================================
    // TEST 7: Reset
    // ==========================================
    printf("\nReset:\n");
    {
        PIDController pid;
        pid_init(&pid, 1.0f, 1.0f, 1.0f, 100.0f, 400.0f, 0.01f);
        
        // Build up some state
        for (int i = 0; i < 50; i++) {
            pid_update(&pid, 10.0f, 0.0f);
        }
        
        // Reset
        pid_reset(&pid);
        
        TEST("Integral is zero after reset");
        if (float_eq(pid.integral, 0.0f, 0.001f)) PASS(); else FAIL();
        
        TEST("Prev_error is zero after reset");
        if (float_eq(pid.prev_error, 0.0f, 0.001f)) PASS(); else FAIL();
    }
    
    // ==========================================
    // TEST 8: Set Gains at Runtime
    // ==========================================
    printf("\nSet Gains:\n");
    {
        PIDController pid;
        pid_init(&pid, 1.0f, 1.0f, 1.0f, 100.0f, 400.0f, 0.01f);
        
        pid_set_gains(&pid, 5.0f, 0.5f, 0.05f);
        
        TEST("kp updated to 5.0");
        if (float_eq(pid.kp, 5.0f, 0.001f)) PASS(); else FAIL();
        
        TEST("ki updated to 0.5");
        if (float_eq(pid.ki, 0.5f, 0.001f)) PASS(); else FAIL();
        
        TEST("kd updated to 0.05");
        if (float_eq(pid.kd, 0.05f, 0.001f)) PASS(); else FAIL();
    }
    
    // ==========================================
    // TEST 9: Full PID Response
    // ==========================================
    printf("\nFull PID Response:\n");
    {
        PIDController pid;
        // Moderate gains for realistic response
        pid_init(&pid, 4.0f, 0.3f, 18.0f, 100.0f, 400.0f, 0.01f);
        
        // Simulate step response: setpoint=30, measurement starts at 0
        // Run 100 iterations (1 second at dt=0.01)
        float output = 0;
        float measurement = 0.0f;
        
        for (int i = 0; i < 100; i++) {
            output = pid_update(&pid, 30.0f, measurement);
            // Simulate measurement approaching setpoint
            measurement += output * 0.001f;
        }
        
        // After 1 second, measurement should be close to setpoint
        TEST("Measurement approaches setpoint after 1s");
        if (float_eq(measurement, 30.0f, 5.0f)) PASS(); else FAIL();
    }
    
    printf("\n=== ALL TESTS COMPLETE ===\n");
    return 0;
}