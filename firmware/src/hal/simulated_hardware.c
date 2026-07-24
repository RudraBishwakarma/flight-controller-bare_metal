/**
 * @file simulated_hardware.c
 * @brief Simulated hardware functions for QEMU/PC testing
 * 
 * Provides fake implementations of hardware functions
 * so the firmware can run on a PC without real hardware.
 */

#include "config/target_qemu.h"
#include <string.h>
#include <stdio.h>

// Simulated time counters
static uint32_t sim_millis = 0;
static uint32_t sim_micros = 0;

// Simulated IMU data
static float sim_gyro[3] = {0.0f, 0.0f, 0.0f};
static float sim_accel[3] = {0.0f, 0.0f, 9.81f};  // Level, gravity only

// Simulated SBUS channels
static float sim_sbus_channels[16] = {
    0.0f, 0.0f, 0.0f, 0.0f,  // Roll, Pitch, Throttle, Yaw
    -1.0f,                     // Arm switch (disarmed)
    0.0f, 0.0f, 0.0f, 0.0f,  // Extra channels
    0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f
};

// Simulated battery
static float sim_battery_voltage = 16.8f;
static float sim_battery_current = 0.5f;


// ==========================================
// INITIALIZATION
// ==========================================

void simulated_hardware_init(void) {
    printf("[SIM] Simulated hardware initialized\n");
    printf("[SIM] IMU: Level, gravity only\n");
    printf("[SIM] SBUS: All channels centered, disarmed\n");
    printf("[SIM] Battery: %.1fV (4S)\n", sim_battery_voltage);
}


// ==========================================
// TIME FUNCTIONS
// ==========================================

uint32_t simulated_millis(void) {
    return sim_millis;
}

uint32_t simulated_micros(void) {
    return sim_micros;
}

// Call this periodically to advance simulated time
void simulated_time_tick(uint32_t delta_ms) {
    sim_millis += delta_ms;
    sim_micros += delta_ms * 1000;
}


// ==========================================
// IMU FUNCTIONS
// ==========================================

void simulated_imu_set(float gyro_x, float gyro_y, float gyro_z,
                       float accel_x, float accel_y, float accel_z) {
    sim_gyro[0] = gyro_x;
    sim_gyro[1] = gyro_y;
    sim_gyro[2] = gyro_z;
    sim_accel[0] = accel_x;
    sim_accel[1] = accel_y;
    sim_accel[2] = accel_z;
}

void simulated_imu_read(float *gx, float *gy, float *gz,
                        float *ax, float *ay, float *az) {
    *gx = sim_gyro[0];
    *gy = sim_gyro[1];
    *gz = sim_gyro[2];
    *ax = sim_accel[0];
    *ay = sim_accel[1];
    *az = sim_accel[2];
}


// ==========================================
// SBUS FUNCTIONS
// ==========================================

void simulated_sbus_set_channels(const float channels[16]) {
    for (int i = 0; i < 16; i++) {
        sim_sbus_channels[i] = channels[i];
    }
}

bool simulated_sbus_read(float channels[16]) {
    for (int i = 0; i < 16; i++) {
        channels[i] = sim_sbus_channels[i];
    }
    return true;  // Always returns data in simulation
}


// ==========================================
// BATTERY FUNCTIONS
// ==========================================

void simulated_battery_set(float voltage, float current) {
    sim_battery_voltage = voltage;
    sim_battery_current = current;
}

void simulated_battery_read(float *voltage, float *current) {
    *voltage = sim_battery_voltage;
    *current = sim_battery_current;
}