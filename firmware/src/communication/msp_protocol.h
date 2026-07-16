#ifndef MSP_PROTOCOL_H
#define MSP_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

// MSP packet structure
#define MSP_MAX_PAYLOAD   64
#define MSP_PREAMBLE_SIZE 2
#define MSP_HEADER_BYTES  5   // Preamble(2) + Dir(1) + Size(1) + Cmd(1) - RENAMED!
#define MSP_CHECKSUM_SIZE 1

// MSP parser states
typedef enum {
    MSP_STATE_IDLE,           // Waiting for '$'
    MSP_STATE_HEADER_M,       // Waiting for 'M'
    MSP_STATE_HEADER_DIR,     // Waiting for direction '<' or '>'
    MSP_STATE_HEADER_SIZE,    // Reading payload size
    MSP_STATE_HEADER_CMD,     // Reading command
    MSP_STATE_PAYLOAD,        // Reading payload
    MSP_STATE_CHECKSUM        // Reading checksum
} MSPState;

// MSP packet structure
typedef struct {
    uint8_t size;                   // Payload size
    uint8_t cmd;                    // Command
    uint8_t payload[MSP_MAX_PAYLOAD]; // Payload data
    uint8_t checksum;               // Received checksum
    uint8_t calculated_checksum;    // Calculated checksum
    bool valid;                     // Packet is valid
} MSPPacket;

// MSP command codes (subset of Betaflight MSP commands)
#define MSP_API_VERSION          1
#define MSP_FC_VARIANT           2
#define MSP_FC_VERSION           3
#define MSP_BOARD_INFO           4
#define MSP_BUILD_INFO           5

#define MSP_RAW_IMU            102
#define MSP_ATTITUDE           108
#define MSP_ALTITUDE           109
#define MSP_BATTERY_STATE      130
#define MSP_RC                 105

#define MSP_SET_RAW_RC         200
#define MSP_SET_PID            202
#define MSP_SET_MODE           214
#define MSP_SET_MOTOR          215
#define MSP_ARMING_CONFIG      61

#define MSP_PID                112
#define MSP_PID_NAMES          117
#define MSP_MODE_RANGES         34
#define MSP_MOTOR              104
#define MSP_STATUS             101

// Initialize the MSP parser
void msp_init(void);

// Parse one byte from the UART stream
// Returns true when a complete valid packet is received
bool msp_parse_byte(uint8_t byte);

// Get the last received packet
const MSPPacket* msp_get_packet(void);

// Build a response packet
// cmd: Command code
// payload: Data to send
// size: Number of payload bytes
// output: Buffer for the complete packet (must be at least size+6 bytes)
// Returns: Total packet size
int msp_build_response(uint8_t cmd, const uint8_t *payload, uint8_t size, uint8_t *output);

// Build a response with 16-bit value
int msp_build_response_u16(uint8_t cmd, uint16_t value, uint8_t *output);

// Build a response with 32-bit value
int msp_build_response_u32(uint8_t cmd, uint32_t value, uint8_t *output);

// Process a received command and build response
// Call this after receiving a valid packet
int msp_process_command(const MSPPacket *request, uint8_t *response);

#endif // MSP_PROTOCOL_H