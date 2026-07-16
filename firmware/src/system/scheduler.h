#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>

// Scheduler for fixed-rate control loop
typedef struct {
    uint32_t target_interval_us;   // Target time between iterations (e.g., 125 for 8kHz)
    uint32_t last_run_us;          // Timestamp of last iteration
    uint32_t loop_count;           // Total number of iterations executed
    float dt;                      // Actual time step in seconds
    bool first_run;                // True before first iteration
} Scheduler;

// Initialize the scheduler
// target_freq_hz: Desired loop frequency (e.g., 8000 for 8kHz)
void scheduler_init(Scheduler *sched, uint32_t target_freq_hz);

// Check if it's time to run the next iteration
// Call this in a tight loop from main
// current_time_us: Current system time in microseconds
// Returns: true if the control loop should execute now
bool scheduler_should_run(Scheduler *sched, uint32_t current_time_us);

// Get the time step (dt) for this iteration
float scheduler_get_dt(const Scheduler *sched);

// Get the current loop count
uint32_t scheduler_get_count(const Scheduler *sched);

// Check if a slower task should run
// Useful for tasks that run at lower frequencies (angle controller, telemetry, etc.)
// current_count: Current loop count
// divider: Run every Nth iteration (e.g., 8 for 1kHz from 8kHz base)
bool scheduler_slow_task_due(uint32_t current_count, uint32_t divider);

// Reset the scheduler
void scheduler_reset(Scheduler *sched);

#endif // SCHEDULER_H