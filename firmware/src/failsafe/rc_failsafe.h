#ifndef RC_FAILSAFE_H
#define RC_FAILSAFE_H

#include <stdint.h>
#include <stdbool.h>

// Failsafe states
typedef enum {
    FAILSAFE_OK = 0,         // Signal is good, normal operation
    FAILSAFE_WARNING,        // Signal degrading, prepare for failsafe
    FAILSAFE_ACTIVE,         // Signal lost, failsafe is active
    FAILSAFE_RECOVERING      // Signal returning, checking stability
} FailsafeState;

// RC Failsafe structure
typedef struct {
    FailsafeState state;           // Current failsafe state
    uint32_t timeout_ms;           // How long before triggering (e.g., 500ms)
    uint32_t last_frame_ms;        // Timestamp of last valid frame
    uint32_t signal_lost_at_ms;    // When signal was first lost
    uint32_t recovery_time_ms;     // How long signal must be good to recover (e.g., 500ms)
    uint32_t signal_recovered_at_ms; // When signal first returned
    bool motors_disarmed;          // True when failsafe has disarmed motors
} RCFailsafe;

// Initialize the failsafe
// timeout_ms: How long without signal before triggering (typical: 500ms)
void failsafe_init(RCFailsafe *fs, uint32_t timeout_ms);

// Call this every control loop iteration with current time
// current_time_ms: System uptime in milliseconds
// frame_received: True if a valid SBUS frame was received this iteration
void failsafe_update(RCFailsafe *fs, uint32_t current_time_ms, bool frame_received);

// Check if motors should be disarmed
bool failsafe_should_disarm(const RCFailsafe *fs);

// Get the current failsafe state (for telemetry/status)
FailsafeState failsafe_get_state(const RCFailsafe *fs);

// Get a human-readable state name
const char* failsafe_state_name(FailsafeState state);

// Reset failsafe (call when arming)
void failsafe_reset(RCFailsafe *fs, uint32_t current_time_ms);

#endif // RC_FAILSAFE_H