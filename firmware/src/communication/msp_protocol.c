#include "msp_protocol.h"
#include <string.h>

// Internal parser state
static MSPState state = MSP_STATE_IDLE;
static MSPPacket rx_packet;
static uint8_t payload_index = 0;
static uint8_t calculated_checksum = 0;

void msp_init(void) {
    state = MSP_STATE_IDLE;
    memset(&rx_packet, 0, sizeof(MSPPacket));
    payload_index = 0;
    calculated_checksum = 0;
}

bool msp_parse_byte(uint8_t byte) {
    
    switch (state) {
        
        // ==========================================
        // STATE 1: Waiting for '$' preamble
        // ==========================================
        case MSP_STATE_IDLE:
            if (byte == '$') {
                state = MSP_STATE_HEADER_M;
                rx_packet.valid = false;
            }
            break;
            
        // ==========================================
        // STATE 2: Waiting for 'M'
        // ==========================================
        case MSP_STATE_HEADER_M:
            if (byte == 'M') {
                state = MSP_STATE_HEADER_DIR;
            } else if (byte == '$') {
                // Another '$' — restart
                state = MSP_STATE_HEADER_M;
            } else {
                // Invalid — go back to idle
                state = MSP_STATE_IDLE;
            }
            break;
            
        // ==========================================
        // STATE 3: Reading direction
        // ==========================================
        case MSP_STATE_HEADER_DIR:
            if (byte == '<' || byte == '>') {
                state = MSP_STATE_HEADER_SIZE;
            } else {
                state = MSP_STATE_IDLE;
            }
            break;
            
        // ==========================================
        // STATE 4: Reading payload size
        // ==========================================
        case MSP_STATE_HEADER_SIZE:
            rx_packet.size = byte;
            if (rx_packet.size > MSP_MAX_PAYLOAD) {
                state = MSP_STATE_IDLE;  // Too large — reject
            } else {
                calculated_checksum = byte;  // Start checksum
                state = MSP_STATE_HEADER_CMD;
            }
            break;
            
        // ==========================================
        // STATE 5: Reading command
        // ==========================================
        case MSP_STATE_HEADER_CMD:
            rx_packet.cmd = byte;
            calculated_checksum ^= byte;
            payload_index = 0;
            
            if (rx_packet.size == 0) {
                // No payload — go straight to checksum
                state = MSP_STATE_CHECKSUM;
            } else {
                state = MSP_STATE_PAYLOAD;
            }
            break;
            
        // ==========================================
        // STATE 6: Reading payload
        // ==========================================
        case MSP_STATE_PAYLOAD:
            rx_packet.payload[payload_index] = byte;
            calculated_checksum ^= byte;
            payload_index++;
            
            if (payload_index >= rx_packet.size) {
                state = MSP_STATE_CHECKSUM;
            }
            break;
            
        // ==========================================
        // STATE 7: Reading checksum
        // ==========================================
        case MSP_STATE_CHECKSUM:
            rx_packet.checksum = byte;
            rx_packet.calculated_checksum = calculated_checksum;
            
            // Validate
            if (rx_packet.checksum == calculated_checksum) {
                rx_packet.valid = true;
            }
            
            state = MSP_STATE_IDLE;
            return true;  // Complete packet received
    }
    
    return false;
}

const MSPPacket* msp_get_packet(void) {
    return &rx_packet;
}

int msp_build_response(uint8_t cmd, const uint8_t *payload, uint8_t size, uint8_t *output) {
    int index = 0;
    uint8_t checksum = 0;
    
    // Preamble
    output[index++] = '$';
    output[index++] = 'M';
    
    // Direction (from flight controller)
    output[index++] = '>';
    
    // Size
    output[index++] = size;
    checksum ^= size;
    
    // Command
    output[index++] = cmd;
    checksum ^= cmd;
    
    // Payload
    for (int i = 0; i < size; i++) {
        output[index++] = payload[i];
        checksum ^= payload[i];
    }
    
    // Checksum
    output[index++] = checksum;
    
    return index;  // Total packet size
}

int msp_build_response_u16(uint8_t cmd, uint16_t value, uint8_t *output) {
    uint8_t payload[2];
    payload[0] = value & 0xFF;         // Low byte
    payload[1] = (value >> 8) & 0xFF;  // High byte
    return msp_build_response(cmd, payload, 2, output);
}

int msp_build_response_u32(uint8_t cmd, uint32_t value, uint8_t *output) {
    uint8_t payload[4];
    payload[0] = value & 0xFF;
    payload[1] = (value >> 8) & 0xFF;
    payload[2] = (value >> 16) & 0xFF;
    payload[3] = (value >> 24) & 0xFF;
    return msp_build_response(cmd, payload, 4, output);
}

int msp_process_command(const MSPPacket *request, uint8_t *response) {
    
    if (!request || !request->valid) {
        return 0;
    }
    
    // Handle specific commands
    switch (request->cmd) {
        case MSP_API_VERSION:
            return msp_build_response_u16(request->cmd, 0x0100, response);  // v1.0
        
        case MSP_STATUS:
            // Placeholder status (11 bytes of zeros for now)
            {
                uint8_t status[11] = {0};
                return msp_build_response(request->cmd, status, 11, response);
            }
        
        default:
            // Unknown command — return empty response with same command code
            return msp_build_response(request->cmd, NULL, 0, response);
    }
}