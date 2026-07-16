#include "sbus_decoder.h"

// Internal state for the byte-by-byte parser
static uint8_t frame_buffer[SBUS_FRAME_SIZE];
static int frame_index = 0;
static bool in_frame = false;

void sbus_init(SBUSData *sbus) {
    // Clear all channels
    for (int i = 0; i < SBUS_CHANNEL_COUNT; i++) {
        sbus->channels[i] = SBUS_MID_VALUE;
    }
    
    sbus->frame_lost = false;
    sbus->failsafe_activated = false;
    sbus->last_valid_frame_ms = 0;
    sbus->new_frame_available = false;
    
    // Reset internal parser state
    frame_index = 0;
    in_frame = false;
}

bool sbus_parse_byte(SBUSData *sbus, uint8_t byte) {
    
    // ==========================================
    // STEP 1: Look for the start byte (0x0F)
    // ==========================================
    if (!in_frame) {
        if (byte == 0x0F) {
            // Found start byte - begin new frame
            in_frame = true;
            frame_index = 0;
            frame_buffer[frame_index++] = byte;
        }
        return false;  // Still building frame
    }
    
    // ==========================================
    // STEP 2: Collect bytes into frame buffer
    // ==========================================
    frame_buffer[frame_index++] = byte;
    
    // ==========================================
    // STEP 3: Check if frame is complete (25 bytes)
    // ==========================================
    if (frame_index >= SBUS_FRAME_SIZE) {
        in_frame = false;
        frame_index = 0;
        
        // ==========================================
        // STEP 4: Validate the frame
        // ==========================================
        
        // Check start byte
        if (frame_buffer[0] != 0x0F) {
            return false;
        }
        
        // Check end byte
        if (frame_buffer[24] != 0x00) {
            return false;
        }
        
        // ==========================================
        // STEP 5: Parse the 16 channels
        // ==========================================
        // Each channel is 11 bits
        // 16 channels × 11 bits = 176 bits = 22 bytes
        
        // Channel data starts at byte 1
        // We extract 11 bits at a time
        
        sbus->channels[0]  = (uint16_t)((frame_buffer[1]      | (frame_buffer[2]  << 8)) & 0x07FF);
        sbus->channels[1]  = (uint16_t)((frame_buffer[2]  >> 3 | (frame_buffer[3]  << 5)) & 0x07FF);
        sbus->channels[2]  = (uint16_t)((frame_buffer[3]  >> 6 | (frame_buffer[4]  << 2) | (frame_buffer[5] << 10)) & 0x07FF);
        sbus->channels[3]  = (uint16_t)((frame_buffer[5]  >> 1 | (frame_buffer[6]  << 7)) & 0x07FF);
        sbus->channels[4]  = (uint16_t)((frame_buffer[6]  >> 4 | (frame_buffer[7]  << 4)) & 0x07FF);
        sbus->channels[5]  = (uint16_t)((frame_buffer[7]  >> 7 | (frame_buffer[8]  << 1) | (frame_buffer[9] << 9)) & 0x07FF);
        sbus->channels[6]  = (uint16_t)((frame_buffer[9]  >> 2 | (frame_buffer[10] << 6)) & 0x07FF);
        sbus->channels[7]  = (uint16_t)((frame_buffer[10] >> 5 | (frame_buffer[11] << 3)) & 0x07FF);
        sbus->channels[8]  = (uint16_t)((frame_buffer[12]     | (frame_buffer[13] << 8)) & 0x07FF);
        sbus->channels[9]  = (uint16_t)((frame_buffer[13] >> 3 | (frame_buffer[14] << 5)) & 0x07FF);
        sbus->channels[10] = (uint16_t)((frame_buffer[14] >> 6 | (frame_buffer[15] << 2) | (frame_buffer[16] << 10)) & 0x07FF);
        sbus->channels[11] = (uint16_t)((frame_buffer[16] >> 1 | (frame_buffer[17] << 7)) & 0x07FF);
        sbus->channels[12] = (uint16_t)((frame_buffer[17] >> 4 | (frame_buffer[18] << 4)) & 0x07FF);
        sbus->channels[13] = (uint16_t)((frame_buffer[18] >> 7 | (frame_buffer[19] << 1) | (frame_buffer[20] << 9)) & 0x07FF);
        sbus->channels[14] = (uint16_t)((frame_buffer[20] >> 2 | (frame_buffer[21] << 6)) & 0x07FF);
        sbus->channels[15] = (uint16_t)((frame_buffer[21] >> 5 | (frame_buffer[22] << 3)) & 0x07FF);
        
        // ==========================================
        // STEP 6: Parse flags byte (byte 23)
        // ==========================================
        uint8_t flags = frame_buffer[23];
        sbus->frame_lost = (flags & 0x04) ? true : false;
        sbus->failsafe_activated = (flags & 0x08) ? true : false;
        
        // ==========================================
        // STEP 7: Mark frame as valid
        // ==========================================
        sbus->new_frame_available = true;
        
        return true;  // Complete valid frame received
    }
    
    return false;  // Still collecting bytes
}

float sbus_get_channel(const SBUSData *sbus, int channel_index) {
    if (channel_index < 0 || channel_index >= SBUS_CHANNEL_COUNT) {
        return 0.0f;
    }
    
    uint16_t raw = sbus->channels[channel_index];
    
    // Map from [172, 1811] to [-1.0, +1.0]
    // Center (992) maps to 0.0
    
    if (raw <= SBUS_MID_VALUE) {
        // Below center: map [172, 992] to [-1.0, 0.0]
        return -(float)(SBUS_MID_VALUE - raw) / (float)(SBUS_MID_VALUE - SBUS_MIN_VALUE);
    } else {
        // Above center: map [992, 1811] to [0.0, +1.0]
        return (float)(raw - SBUS_MID_VALUE) / (float)(SBUS_MAX_VALUE - SBUS_MID_VALUE);
    }
}

float sbus_get_throttle(const SBUSData *sbus, int channel_index) {
    if (channel_index < 0 || channel_index >= SBUS_CHANNEL_COUNT) {
        return 0.0f;
    }
    
    uint16_t raw = sbus->channels[channel_index];
    
    // Map from [172, 1811] to [0.0, 1.0]
    // 172 → 0.0, 1811 → 1.0
    
    if (raw <= SBUS_MIN_VALUE) {
        return 0.0f;
    }
    
    return (float)(raw - SBUS_MIN_VALUE) / (float)(SBUS_MAX_VALUE - SBUS_MIN_VALUE);
}

bool sbus_is_alive(const SBUSData *sbus, uint32_t current_time_ms, uint32_t timeout_ms) {
    if (sbus->last_valid_frame_ms == 0) {
        return false;  // Never received a frame
    }
    
    return (current_time_ms - sbus->last_valid_frame_ms) < timeout_ms;
}