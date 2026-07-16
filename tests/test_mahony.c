#include <stdio.h>
#include <math.h>
#include "../firmware/src/estimation/mahony_filter.h"

#define TEST(name) printf("  %s: ", name)
#define PASS() printf("PASSED\n")
#define FAIL() printf("FAILED\n")

int float_eq(float a, float b, float tolerance) {
    float diff = a - b;
    if (diff < 0) diff = -diff;
    return diff < tolerance;
}

int main() {
    printf("=== MAHONY FILTER TESTS ===\n\n");
    
    // Test 1: Initialization
    printf("Initialization:\n");
    {
        MahonyFilter filter;
        mahony_init(&filter, 8000.0f);
        
        TEST("Starts at level (q = 1,0,0,0)");
        if (float_eq(filter.q[0], 1.0f, 0.001f) &&
            float_eq(filter.q[1], 0.0f, 0.001f) &&
            float_eq(filter.q[2], 0.0f, 0.001f) &&
            float_eq(filter.q[3], 0.0f, 0.001f))
            PASS(); else FAIL();
        
        float roll, pitch, yaw;
        mahony_get_euler(&filter, &roll, &pitch, &yaw);
        
        TEST("Euler angles are zero at init");
        if (float_eq(roll, 0.0f, 0.01f) &&
            float_eq(pitch, 0.0f, 0.01f) &&
            float_eq(yaw, 0.0f, 0.01f))
            PASS(); else FAIL();
    }
    
    // Test 2: Level - no movement
    printf("\nLevel (no movement):\n");
    {
        MahonyFilter filter;
        mahony_init(&filter, 1000.0f);
        
        // Stay level for 100 iterations
        // Gyro = 0, Accel = (0, 0, 1g)
        for (int i = 0; i < 100; i++) {
            mahony_update(&filter, 0, 0, 0, 0, 0, 9.81f, 0.001f);
        }
        
        float roll, pitch, yaw;
        mahony_get_euler(&filter, &roll, &pitch, &yaw);
        
        TEST("Roll stays near 0 after 0.1s level");
        if (float_eq(roll, 0.0f, 0.05f)) PASS(); else FAIL();
        
        TEST("Pitch stays near 0 after 0.1s level");
        if (float_eq(pitch, 0.0f, 0.05f)) PASS(); else FAIL();
    }
    
    // Test 3: Pure rotation around X (roll)
    printf("\nPure Roll Detection:\n");
    {
        MahonyFilter filter;
        mahony_init(&filter, 1000.0f);
        
        // Simulate 30°/s roll for 1 second = 30° total
        float roll_rate = 30.0f * M_PI / 180.0f;  // 0.5236 rad/s
        
        // When rolled 30°, accelerometer should show gravity tilted
        // ax ≈ 0, ay ≈ g*sin(30°), az ≈ g*cos(30°)
        
        for (int i = 0; i < 1000; i++) {  // 1 second
            float current_angle = roll_rate * (i * 0.001f);
            float ay = 9.81f * sinf(current_angle);
            float az = 9.81f * cosf(current_angle);
            
            mahony_update(&filter, roll_rate, 0, 0, 0, ay, az, 0.001f);
        }
        
        float roll, pitch, yaw;
        mahony_get_euler(&filter, &roll, &pitch, &yaw);
        
        TEST("Roll is approximately 30° after rotating 30°");
        if (float_eq(roll, 30.0f * M_PI / 180.0f, 0.1f)) PASS(); else FAIL();
    }
    
    // Test 4: Reset
    printf("\nReset:\n");
    {
        MahonyFilter filter;
        mahony_init(&filter, 1000.0f);
        
        // Tilt the filter
        for (int i = 0; i < 500; i++) {
            mahony_update(&filter, 1.0f, 0, 0, 0, 5.0f, 8.0f, 0.001f);
        }
        
        // Reset
        mahony_reset(&filter);
        
        float roll, pitch, yaw;
        mahony_get_euler(&filter, &roll, &pitch, &yaw);
        
        TEST("Roll is zero after reset");
        if (float_eq(roll, 0.0f, 0.01f)) PASS(); else FAIL();
        
        TEST("Pitch is zero after reset");
        if (float_eq(pitch, 0.0f, 0.01f)) PASS(); else FAIL();
    }
    
    printf("\n=== ALL TESTS COMPLETE ===\n");
    return 0;
}