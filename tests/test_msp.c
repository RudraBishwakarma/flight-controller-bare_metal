#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "../firmware/src/communication/msp_protocol.h"

#define TEST(name) printf("  %s: ", name)
#define PASS() printf("PASSED\n")
#define FAIL() printf("FAILED\n")

int main() {
    printf("=== MSP PROTOCOL TESTS ===\n\n");
    
    // Test 1: Parse valid packet
    printf("Valid Packet:\n");
    {
        msp_init();
        
        // Build a valid MSP packet: $ M < 0 1 checksum
        // Size=0, Cmd=1 (API_VERSION)
        // Checksum = size ^ cmd = 0 ^ 1 = 1
        uint8_t packet[] = {'$', 'M', '<', 0, 1, 1};
        
        bool complete = false;
        for (int i = 0; i < 6; i++) {
            complete = msp_parse_byte(packet[i]);
        }
        
        TEST("Complete packet detected");
        if (complete) PASS(); else FAIL();
        
        const MSPPacket *pkt = msp_get_packet();
        TEST("Packet is valid");
        if (pkt->valid) PASS(); else FAIL();
        
        TEST("Command is API_VERSION (1)");
        if (pkt->cmd == 1) PASS(); else FAIL();
        
        TEST("Size is 0");
        if (pkt->size == 0) PASS(); else FAIL();
    }
    
    // Test 2: Invalid checksum
    printf("\nInvalid Checksum:\n");
    {
        msp_init();
        
        // Correct checksum would be 1, we send 99
        uint8_t packet[] = {'$', 'M', '<', 0, 1, 99};
        
        bool complete = false;
        for (int i = 0; i < 6; i++) {
            complete = msp_parse_byte(packet[i]);
        }
        
        const MSPPacket *pkt = msp_get_packet();
        TEST("Packet rejected with wrong checksum");
        if (!pkt->valid) PASS(); else FAIL();
    }
    
    // Test 3: Garbage bytes before valid packet
    printf("\nGarbage Resilience:\n");
    {
        msp_init();
        
        // Garbage bytes then valid packet
        // Size=2, Cmd=105 (MSP_RC), Payload=[0x12, 0x34]
        // Checksum = 2 ^ 105 ^ 0x12 ^ 0x34
        // 0x02 = 00000010
        // 0x69 = 01101001  (105 decimal)
        // XOR  = 01101011 = 0x6B
        // 0x6B ^ 0x12 = 01101011 ^ 00010010 = 01111001 = 0x79
        // 0x79 ^ 0x34 = 01111001 ^ 00110100 = 01001101 = 0x4D
        uint8_t full[] = {0xFF, 0x00, 0xAA, '$', 'M', '<', 2, 105, 0x12, 0x34, 0x4D};
        
        bool complete = false;
        for (int i = 0; i < 11; i++) {
            complete = msp_parse_byte(full[i]);
        }
        
        TEST("Garbage bytes ignored, valid packet found");
        if (complete) PASS(); else FAIL();
        
        const MSPPacket *pkt = msp_get_packet();
        TEST("Packet is valid after garbage");
        if (pkt->valid) PASS(); else FAIL();
        
        TEST("Payload received correctly");
        if (pkt->payload[0] == 0x12 && pkt->payload[1] == 0x34) PASS(); else FAIL();
    }
    
    // Test 4: Build response
    printf("\nBuild Response:\n");
    {
        uint8_t response[64];
        int size = msp_build_response_u16(MSP_API_VERSION, 0x0100, response);
        
        TEST("Response starts with $M>");
        if (response[0] == '$' && response[1] == 'M' && response[2] == '>')
            PASS(); else FAIL();
        
        TEST("Response size field is 2");
        if (response[3] == 2) PASS(); else FAIL();
        
        TEST("Response command matches request");
        if (response[4] == MSP_API_VERSION) PASS(); else FAIL();
        
        TEST("Response payload contains correct value (0x0100)");
        if (response[5] == 0x00 && response[6] == 0x01) PASS(); else FAIL();
    }
    
    // Test 5: Process MSP_API_VERSION command
    printf("\nProcess Command:\n");
    {
        // Simulate receiving an API_VERSION request
        msp_init();
        uint8_t request_bytes[] = {'$', 'M', '<', 0, MSP_API_VERSION, 0 ^ MSP_API_VERSION};
        
        for (int i = 0; i < 6; i++) {
            msp_parse_byte(request_bytes[i]);
        }
        
        const MSPPacket *req = msp_get_packet();
        uint8_t response[64];
        int size = msp_process_command(req, response);
        
        TEST("Process command returns non-zero size");
        if (size > 0) PASS(); else FAIL();
        
        TEST("Response has correct command");
        if (response[4] == MSP_API_VERSION) PASS(); else FAIL();
    }
    
    // Test 6: Invalid start byte rejected
    printf("\nInvalid Start Byte:\n");
    {
        msp_init();
        
        // Missing '$' at start
        uint8_t packet[] = {'X', 'M', '<', 0, 1, 1};
        
        bool complete = false;
        for (int i = 0; i < 6; i++) {
            complete = msp_parse_byte(packet[i]);
        }
        
        TEST("Packet without $ is rejected");
        if (!complete) PASS(); else FAIL();
    }
    
    // Test 7: Empty packet handled
    printf("\nEmpty Response:\n");
    {
        uint8_t response[64];
        int size = msp_build_response(99, NULL, 0, response);
        
        // $ M > 0 99 checksum
        // checksum = 0 ^ 99 = 99
        TEST("Empty response has correct size (6 bytes)");
        if (size == 6) PASS(); else FAIL();
        
        TEST("Empty response checksum correct");
        if (response[5] == 99) PASS(); else FAIL();
    }
    
    // Test 8: 32-bit response
    printf("\n32-bit Response:\n");
    {
        uint8_t response[64];
        int size = msp_build_response_u32(200, 0x12345678, response);
        
        TEST("32-bit response has correct size (8 bytes total)");
        if (size == 8) PASS(); else FAIL();
        
        TEST("32-bit value encoded correctly (little-endian)");
        if (response[5] == 0x78 && response[6] == 0x56 && 
            response[7] == 0x34 && response[8] == 0x12) PASS(); else FAIL();
    }
    
    // Test 9: Maximum size packet rejected
    printf("\nOversized Packet:\n");
    {
        msp_init();
        
        // Size byte = 255 (exceeds MSP_MAX_PAYLOAD = 64)
        uint8_t packet[] = {'$', 'M', '<', 255, 1, 1};
        
        bool complete = false;
        for (int i = 0; i < 6; i++) {
            complete = msp_parse_byte(packet[i]);
        }
        
        TEST("Oversized packet rejected");
        if (!complete) PASS(); else FAIL();
    }
    
    printf("\n=== ALL TESTS COMPLETE ===\n");
    return 0;
}