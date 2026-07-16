#include "rc_failsafe.h"

void failsafe_init(RCFailsafe *fs, uint32_t timeout_ms) {
    fs->state = FAILSAFE_OK;
    fs->timeout_ms = timeout_ms;
    fs->last_frame_ms = 0;
    fs->signal_lost_at_ms = 0;
    fs->recovery_time_ms = 500;  // Need 500ms of good signal to recover
    fs->signal_recovered_at_ms = 0;
    fs->motors_disarmed = false;
}

void failsafe_update(RCFailsafe *fs, uint32_t current_time_ms, bool frame_received) {
    
    // ==========================================
    // Update last frame timestamp
    // ==========================================
    if (frame_received) {
        fs->last_frame_ms = current_time_ms;
    }
    
    // ==========================================
    // Calculate time since last frame
    // ==========================================
    uint32_t time_since_last_frame = current_time_ms - fs->last_frame_ms;
    
    // Handle the case where last_frame_ms is 0 (never received a frame)
    if (fs->last_frame_ms == 0) {
        time_since_last_frame = UINT32_MAX;  // Effectively infinite
    }
    
    // ==========================================
    // State machine
    // ==========================================
    
    switch (fs->state) {
        
        case FAILSAFE_OK:
            // Check if signal is lost
            if (time_since_last_frame >= fs->timeout_ms) {
                fs->state = FAILSAFE_WARNING;
                fs->signal_lost_at_ms = current_time_ms;
            }
            break;
            
        case FAILSAFE_WARNING:
            // Warning phase: signal just lost, wait briefly
            // If signal returns quickly, go back to OK
            if (frame_received) {
                fs->state = FAILSAFE_OK;
            }
            // If warning persists for 100ms, escalate to ACTIVE
            else if ((current_time_ms - fs->signal_lost_at_ms) > 100) {
                fs->state = FAILSAFE_ACTIVE;
                fs->motors_disarmed = true;
            }
            break;
            
        case FAILSAFE_ACTIVE:
            // Signal is lost, motors are disarmed
            // Check if signal is returning
            if (frame_received) {
                fs->signal_recovered_at_ms = current_time_ms;
                fs->state = FAILSAFE_RECOVERING;
            }
            break;
            
        case FAILSAFE_RECOVERING:
            // Signal is back, but need to verify it's stable
            if (!frame_received) {
                // Signal dropped again — back to ACTIVE
                fs->state = FAILSAFE_ACTIVE;
            }
            // Need stable signal for recovery_time_ms
            else if ((current_time_ms - fs->signal_recovered_at_ms) >= fs->recovery_time_ms) {
                // Signal stable — fully recover
                fs->state = FAILSAFE_OK;
                fs->motors_disarmed = false;
            }
            break;
    }
}

bool failsafe_should_disarm(const RCFailsafe *fs) {
    return fs->motors_disarmed;
}

FailsafeState failsafe_get_state(const RCFailsafe *fs) {
    return fs->state;
}

const char* failsafe_state_name(FailsafeState state) {
    switch (state) {
        case FAILSAFE_OK:         return "OK";
        case FAILSAFE_WARNING:    return "WARNING";
        case FAILSAFE_ACTIVE:     return "ACTIVE";
        case FAILSAFE_RECOVERING: return "RECOVERING";
        default:                  return "UNKNOWN";
    }
}

void failsafe_reset(RCFailsafe *fs, uint32_t current_time_ms) {
    fs->state = FAILSAFE_OK;
    fs->last_frame_ms = current_time_ms;
    fs->signal_lost_at_ms = 0;
    fs->signal_recovered_at_ms = 0;
    fs->motors_disarmed = false;
}