#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "../firmware/src/communication/sbus_decoder.h"

#define TEST(name) printf("  %s: ", name)
#define PASS() printf("PASSED\n")
#define FAIL() printf("FAILED\n")

int main() {
    printf("=== SBUS DECODER TESTS ===\n\n");
    
    // ==========================================
    // TEST 1: Initialization
    // ==========================================
    printf("Initialization:\n");
    {
        SBUSData sbus;
        sbus_init(&sbus);
        
        TEST("Channels initialized to mid value (992)");
        int all_mid = 1;
        for (int i = 0; i < SBUS_CHANNEL_COUNT; i++) {
            if (sbus.channels[i] != 992) all_mid = 0;
        }
        if (all_mid) PASS(); else FAIL();
        
        TEST("Frame lost flag is false");
        if (!sbus.frame_lost) PASS(); else FAIL();
        
        TEST("Failsafe flag is false");
        if (!sbus.failsafe_activated) PASS(); else FAIL();
    }
    
    // ==========================================
    // TEST 2: Parse Valid Frame (All Channels at Mid)
    // ==========================================
    printf("\nValid Frame (Mid Values):\n");
    {
        SBUSData sbus;
        sbus_init(&sbus);
        
        // Create a valid SBUS frame with all channels at mid (992)
        // 992 in 11-bit binary: 01111100000
        uint8_t frame[25] = {
            0x0F,  // Start byte
            0xE0, 0x03,  // Channel 1: 992 (lower 8 bits: 11100000, upper 3 bits: 000)
            0x1F, 0x00,  // Channel 2: 992
            0xC0, 0x07,  // Channel 3: 992
            0x3E, 0x00,  // Channel 4: 992
            0x80, 0x0F,  // Channel 5: 992
            0x7D, 0x00,  // Channel 6: 992
            0xE0, 0x03,  // Channel 7: 992
            0x1F, 0x00,  // Channel 8: 992
            0xC0, 0x07,  // Channel 9: 992
            0x3E, 0x00,  // Channel 10: 992
            0x80, 0x0F,  // Channel 11: 992
            0x00,        // Flags: no frame lost, no failsafe
            0x00         // End byte
        };
        
        bool complete = false;
        for (int i = 0; i < 25; i++) {
            complete = sbus_parse_byte(&sbus, frame[i]);
        }
        
        TEST("Complete frame detected");
        if (complete) PASS(); else FAIL();
        
        TEST("Channel 0 is mid (992)");
        if (sbus.channels[0] == 992) PASS(); else FAIL();
    }
    
    // ==========================================
    // TEST 3: Reject Invalid Start Byte
    // ==========================================
    printf("\nInvalid Frame Detection:\n");
    {
        SBUSData sbus;
        sbus_init(&sbus);
        
        uint8_t bad_frame[25];
        bad_frame[0] = 0x00;  // Wrong start byte!
        
        bool complete = false;
        for (int i = 0; i < 25; i++) {
            complete = sbus_parse_byte(&sbus, bad_frame[i]);
        }
        
        TEST("Frame with wrong start byte is rejected");
        if (!complete) PASS(); else FAIL();
    }
    
    // ==========================================
    // TEST 4: Normalize Channel Values
    // ==========================================
    printf("\nChannel Normalization:\n");
    {
        SBUSData sbus;
        sbus_init(&sbus);
        
        // Test minimum value (172)
        sbus.channels[0] = 172;
        float val = sbus_get_channel(&sbus, 0);
        TEST("Channel min (172) maps to -1.0");
        if (val <= -0.99f && val >= -1.01f) PASS(); else FAIL();
        
        // Test maximum value (1811)
        sbus.channels[0] = 1811;
        val = sbus_get_channel(&sbus, 0);
        TEST("Channel max (1811) maps to 1.0");
        if (val >= 0.99f && val <= 1.01f) PASS(); else FAIL();
        
        // Test mid value (992)
        sbus.channels[0] = 992;
        val = sbus_get_channel(&sbus, 0);
        TEST("Channel mid (992) maps to 0.0");
        if (val >= -0.01f && val <= 0.01f) PASS(); else FAIL();
    }
    
    // ==========================================
    // TEST 5: Throttle Normalization
    // ==========================================
    printf("\nThrottle Normalization:\n");
    {
        SBUSData sbus;
        sbus_init(&sbus);
        
        sbus.channels[2] = 172;  // Min throttle
        float val = sbus_get_throttle(&sbus, 2);
        TEST("Min throttle (172) maps to 0.0");
        if (val >= -0.01f && val <= 0.01f) PASS(); else FAIL();
        
        sbus.channels[2] = 1811;  // Max throttle
        val = sbus_get_throttle(&sbus, 2);
        TEST("Max throttle (1811) maps to 1.0");
        if (val >= 0.99f && val <= 1.01f) PASS(); else FAIL();
    }
    
    // ==========================================
    // TEST 6: Frame Lost Flag
    // ==========================================
    printf("\nFrame Lost Flag:\n");
    {
        SBUSData sbus;
        sbus_init(&sbus);
        
        uint8_t frame[25] = {0};
        frame[0] = 0x0F;
        frame[23] = 0x04;  // Set frame lost bit
        frame[24] = 0x00;
        
        for (int i = 0; i < 25; i++) {
            sbus_parse_byte(&sbus, frame[i]);
        }
        
        TEST("Frame lost flag detected");
        if (sbus.frame_lost) PASS(); else FAIL();
    }
    
    // ==========================================
    // TEST 7: Failsafe Flag
    // ==========================================
    printf("\nFailsafe Flag:\n");
    {
        SBUSData sbus;
        sbus_init(&sbus);
        
        uint8_t frame[25] = {0};
        frame[0] = 0x0F;
        frame[23] = 0x08;  // Set failsafe bit
        frame[24] = 0x00;
        
        for (int i = 0; i < 25; i++) {
            sbus_parse_byte(&sbus, frame[i]);
        }
        
        TEST("Failsafe flag detected");
        if (sbus.failsafe_activated) PASS(); else FAIL();
    }
    
    printf("\n=== ALL TESTS COMPLETE ===\n");
    return 0;
}