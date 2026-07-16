#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "../firmware/src/failsafe/rc_failsafe.h"

#define TEST(name) printf("  %s: ", name)
#define PASS() printf("PASSED\n")
#define FAIL() printf("FAILED\n")

int main() {
    printf("=== RC FAILSAFE TESTS ===\n\n");
    
    // Test 1: Initialization
    printf("Initialization:\n");
    {
        RCFailsafe fs;
        failsafe_init(&fs, 500);
        
        TEST("Starts in OK state");
        if (failsafe_get_state(&fs) == FAILSAFE_OK) PASS(); else FAIL();
        
        TEST("Should not disarm at init");
        if (!failsafe_should_disarm(&fs)) PASS(); else FAIL();
    }
    
    // Test 2: Normal operation (frames arriving)
    printf("\nNormal Operation:\n");
    {
        RCFailsafe fs;
        failsafe_init(&fs, 500);
        
        // Simulate frames arriving for 10 seconds
        for (uint32_t t = 0; t < 10000; t += 10) {
            failsafe_update(&fs, t, true);
        }
        
        TEST("Stays in OK state with frames arriving");
        if (failsafe_get_state(&fs) == FAILSAFE_OK) PASS(); else FAIL();
    }
    
    // Test 3: Signal loss triggers failsafe
    printf("\nSignal Loss:\n");
    {
        RCFailsafe fs;
        failsafe_init(&fs, 500);
        
        // First, establish signal
        failsafe_update(&fs, 0, true);
        
        // Then lose signal for 600ms
        failsafe_update(&fs, 100, false);
        TEST("Not triggered at 100ms");
        if (failsafe_get_state(&fs) == FAILSAFE_OK) PASS(); else FAIL();
        
        failsafe_update(&fs, 300, false);
        TEST("Not triggered at 300ms");
        if (failsafe_get_state(&fs) == FAILSAFE_OK) PASS(); else FAIL();
        
        failsafe_update(&fs, 500, false);
        TEST("Warning at 500ms");
        if (failsafe_get_state(&fs) == FAILSAFE_WARNING) PASS(); else FAIL();
        
        failsafe_update(&fs, 650, false);
        TEST("Active at 650ms (warning + 100ms)");
        if (failsafe_get_state(&fs) == FAILSAFE_ACTIVE) PASS(); else FAIL();
        
        TEST("Motors disarmed when ACTIVE");
        if (failsafe_should_disarm(&fs)) PASS(); else FAIL();
    }
    
    // Test 4: Quick signal loss doesn't trigger
    printf("\nBrief Signal Loss:\n");
    {
        RCFailsafe fs;
        failsafe_init(&fs, 500);
        
        failsafe_update(&fs, 0, true);    // Signal good
        failsafe_update(&fs, 200, false); // Brief loss
        failsafe_update(&fs, 210, true);  // Signal back quickly
        
        TEST("Brief loss (<500ms) doesn't trigger failsafe");
        if (failsafe_get_state(&fs) == FAILSAFE_OK) PASS(); else FAIL();
    }
    
    // Test 5: Recovery from failsafe
    printf("\nRecovery:\n");
    {
        RCFailsafe fs;
        failsafe_init(&fs, 500);
        
        // Trigger failsafe
        failsafe_update(&fs, 0, true);
        for (uint32_t t = 100; t < 700; t += 100) {
            failsafe_update(&fs, t, false);
        }
        
        TEST("Failsafe is ACTIVE after signal loss");
        if (failsafe_get_state(&fs) == FAILSAFE_ACTIVE) PASS(); else FAIL();
        
        // Signal returns
        failsafe_update(&fs, 700, true);
        TEST("Recovering when signal first returns");
        if (failsafe_get_state(&fs) == FAILSAFE_RECOVERING) PASS(); else FAIL();
        
        // Stable signal for recovery period
        for (uint32_t t = 710; t < 1300; t += 10) {
            failsafe_update(&fs, t, true);
        }
        
        TEST("Returns to OK after stable signal");
        if (failsafe_get_state(&fs) == FAILSAFE_OK) PASS(); else FAIL();
        
        TEST("Motors no longer disarmed after recovery");
        if (!failsafe_should_disarm(&fs)) PASS(); else FAIL();
    }
    
    // Test 6: Reset
    printf("\nReset:\n");
    {
        RCFailsafe fs;
        failsafe_init(&fs, 500);
        
        // Trigger failsafe
        failsafe_update(&fs, 0, true);
        for (uint32_t t = 100; t < 700; t += 100) {
            failsafe_update(&fs, t, false);
        }
        
        // Reset
        failsafe_reset(&fs, 1000);
        
        TEST("State is OK after reset");
        if (failsafe_get_state(&fs) == FAILSAFE_OK) PASS(); else FAIL();
        
        TEST("Motors not disarmed after reset");
        if (!failsafe_should_disarm(&fs)) PASS(); else FAIL();
    }
    
    printf("\n=== ALL TESTS COMPLETE ===\n");
    return 0;
}