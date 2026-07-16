#ifndef SBUS_DECODER_H
#define SBUS_DECODER_H

#include <stdint.h>
#include <stdbool.h>

// Number of channels in an SBUS frame
#define SBUS_CHANNEL_COUNT  16
#define SBUS_FRAME_SIZE     25

// SBUS channel value ranges
#define SBUS_MIN_VALUE      172
#define SBUS_MAX_VALUE      1811
#define SBUS_MID_VALUE      992

// SBUS data structure
typedef struct {
    uint16_t channels[SBUS_CHANNEL_COUNT];  // Raw channel values (172-1811)
    bool frame_lost;                         // True if frame lost flag is set
    bool failsafe_activated;                 // True if receiver is in failsafe
    uint32_t last_valid_frame_ms;            // Timestamp of last good frame
    bool new_frame_available;                // True when a new frame is ready
} SBUSData;

// Initialize the SBUS decoder
void sbus_init(SBUSData *sbus);

// Feed one byte at a time from UART into the decoder
// Returns true when a complete valid frame has been received
bool sbus_parse_byte(SBUSData *sbus, uint8_t byte);

// Get a channel value normalized to [-1.0, +1.0]
// channel_index: 0-15
// Returns: -1.0 (min) to +1.0 (max), 0.0 is center
float sbus_get_channel(const SBUSData *sbus, int channel_index);

// Get a channel value normalized to [0.0, 1.0]
// Useful for throttle (channel 2 typically)
float sbus_get_throttle(const SBUSData *sbus, int channel_index);

// Check if the SBUS connection is alive
bool sbus_is_alive(const SBUSData *sbus, uint32_t current_time_ms, uint32_t timeout_ms);

#endif // SBUS_DECODER_H