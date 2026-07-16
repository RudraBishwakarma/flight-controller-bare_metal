#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "../firmware/src/system/scheduler.h"

#define TEST(name) printf("  %s: ", name)
#define PASS() printf("PASSED\n")
#define FAIL() printf("FAILED\n")

int main() {
    printf("=== SCHEDULER TESTS ===\n\n");
    
    // Test 1: Initialization
    printf("Initialization:\n");
    {
        Scheduler sched;
        scheduler_init(&sched, 8000);
        
        TEST("Target interval is 125us for 8kHz");
        if (sched.target_interval_us == 125) PASS(); else FAIL();
        
        TEST("dt is 0.000125 for 8kHz");
        if (scheduler_get_dt(&sched) == 0.000125f) PASS(); else FAIL();
        
        TEST("Loop count starts at 0");
        if (scheduler_get_count(&sched) == 0) PASS(); else FAIL();
    }
    
    // Test 2: First run always executes
    printf("\nFirst Run:\n");
    {
        Scheduler sched;
        scheduler_init(&sched, 8000);
        
        TEST("First call returns true (run immediately)");
        if (scheduler_should_run(&sched, 1000)) PASS(); else FAIL();
        
        TEST("Loop count is 1 after first run");
        if (scheduler_get_count(&sched) == 1) PASS(); else FAIL();
    }
    
    // Test 3: Not ready before interval
    printf("\nTiming:\n");
    {
        Scheduler sched;
        scheduler_init(&sched, 8000);
        
        scheduler_should_run(&sched, 1000);  // First run at t=1000us
        
        TEST("Not ready at 1100us (only 100us elapsed)");
        if (!scheduler_should_run(&sched, 1100)) PASS(); else FAIL();
        
        TEST("Not ready at 1120us (120us elapsed)");
        if (!scheduler_should_run(&sched, 1120)) PASS(); else FAIL();
        
        TEST("Ready at 1125us (125us elapsed)");
        if (scheduler_should_run(&sched, 1125)) PASS(); else FAIL();
    }
    
    // Test 4: Loop count increments
    printf("\nLoop Count:\n");
    {
        Scheduler sched;
        scheduler_init(&sched, 8000);
        
        scheduler_should_run(&sched, 0);      // Count=1
        scheduler_should_run(&sched, 125);    // Count=2
        scheduler_should_run(&sched, 250);    // Count=3
        scheduler_should_run(&sched, 375);    // Count=4
        
        TEST("Loop count is 4 after 4 iterations");
        if (scheduler_get_count(&sched) == 4) PASS(); else FAIL();
    }
    
    // Test 5: Slow task scheduling
    printf("\nSlow Task:\n");
    {
        TEST("Count 0 % 8 = 0 -> should run");
        if (scheduler_slow_task_due(0, 8)) PASS(); else FAIL();
        
        TEST("Count 1 % 8 != 0 -> should NOT run");
        if (!scheduler_slow_task_due(1, 8)) PASS(); else FAIL();
        
        TEST("Count 8 % 8 = 0 -> should run");
        if (scheduler_slow_task_due(8, 8)) PASS(); else FAIL();
        
        TEST("Count 16 % 8 = 0 -> should run");
        if (scheduler_slow_task_due(16, 8)) PASS(); else FAIL();
    }
    
    // Test 6: Different frequencies
    printf("\nDifferent Frequencies:\n");
    {
        Scheduler sched_1k;
        scheduler_init(&sched_1k, 1000);
        
        TEST("1kHz interval is 1000us");
        if (sched_1k.target_interval_us == 1000) PASS(); else FAIL();
        
        TEST("1kHz dt is 0.001");
        if (scheduler_get_dt(&sched_1k) == 0.001f) PASS(); else FAIL();
        
        Scheduler sched_100;
        scheduler_init(&sched_100, 100);
        
        TEST("100Hz interval is 10000us");
        if (sched_100.target_interval_us == 10000) PASS(); else FAIL();
    }
    
    // Test 7: Reset
    printf("\nReset:\n");
    {
        Scheduler sched;
        scheduler_init(&sched, 8000);
        
        for (int i = 0; i < 100; i++) {
            scheduler_should_run(&sched, i * 125);
        }
        
        TEST("Loop count is 100 after 100 iterations");
        if (scheduler_get_count(&sched) == 100) PASS(); else FAIL();
        
        scheduler_reset(&sched);
        
        TEST("Loop count is 0 after reset");
        if (scheduler_get_count(&sched) == 0) PASS(); else FAIL();
    }
    
    printf("\n=== ALL TESTS COMPLETE ===\n");
    return 0;
}