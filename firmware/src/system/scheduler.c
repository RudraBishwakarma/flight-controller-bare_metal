#include "scheduler.h"

void scheduler_init(Scheduler *sched, uint32_t target_freq_hz) {
    // Calculate interval in microseconds
    // Example: 8kHz → 1,000,000 / 8000 = 125 microseconds
    sched->target_interval_us = 1000000 / target_freq_hz;
    
    sched->last_run_us = 0;
    sched->loop_count = 0;
    sched->dt = 1.0f / (float)target_freq_hz;  // e.g., 1/8000 = 0.000125
    sched->first_run = true;
}

bool scheduler_should_run(Scheduler *sched, uint32_t current_time_us) {
    
    // ==========================================
    // First iteration: Always run immediately
    // ==========================================
    if (sched->first_run) {
        sched->first_run = false;
        sched->last_run_us = current_time_us;
        sched->loop_count = 1;
        return true;
    }
    
    // ==========================================
    // Calculate time since last iteration
    // ==========================================
    uint32_t elapsed_us;
    
    // Handle microsecond timer overflow (happens every ~71 minutes for 32-bit)
    if (current_time_us >= sched->last_run_us) {
        elapsed_us = current_time_us - sched->last_run_us;
    } else {
        // Timer wrapped around
        elapsed_us = (UINT32_MAX - sched->last_run_us) + current_time_us + 1;
    }
    
    // ==========================================
    // Check if enough time has passed
    // ==========================================
    if (elapsed_us >= sched->target_interval_us) {
        // Update timing
        sched->last_run_us = current_time_us;
        sched->loop_count++;
        return true;
    }
    
    // ==========================================
    // Not yet time to run
    // ==========================================
    return false;
}

float scheduler_get_dt(const Scheduler *sched) {
    return sched->dt;
}

uint32_t scheduler_get_count(const Scheduler *sched) {
    return sched->loop_count;
}

bool scheduler_slow_task_due(uint32_t current_count, uint32_t divider) {
    // Run every Nth iteration
    // Example: divider=8 means run when count % 8 == 0
    return (current_count % divider) == 0;
}

void scheduler_reset(Scheduler *sched) {
    sched->last_run_us = 0;
    sched->loop_count = 0;
    sched->first_run = true;
}