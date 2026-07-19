#ifndef TARGET_QEMU_H
#define TARGET_QEMU_H

// ==========================================
// BUILD TARGET IDENTIFICATION
// ==========================================

#define TARGET_QEMU         1
#define TARGET_STM32F405    0

// ==========================================
// STANDARD HEADERS
// ==========================================

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// ==========================================
// SYSTEM CLOCK CONFIGURATION
// ==========================================

// In QEMU, we don't have a real clock tree
// These values are used by the scheduler and timing functions
#define SYSTEM_CLOCK_HZ     168000000   // Simulated 168MHz
#define AHB_CLOCK_HZ        168000000
#define APB1_CLOCK_HZ        42000000
#define APB2_CLOCK_HZ        84000000

// SysTick configuration
#define SYSTICK_FREQ_HZ     1000        // 1kHz tick for FreeRTOS compatibility

// ==========================================
// MEMORY CONFIGURATION
// ==========================================

#define FLASH_SIZE          (1024 * 1024)  // 1MB
#define RAM_SIZE            (192 * 1024)    // 192KB
#define HEAP_SIZE           (40 * 1024)     // 40KB heap

// ==========================================
// PERIPHERAL ENABLE/DISABLE
// ==========================================

// In QEMU simulation, we don't use real hardware peripherals
// These flags tell the HAL to use simulated versions

#define HAL_USE_SIMULATED_SPI    1
#define HAL_USE_SIMULATED_I2C    1
#define HAL_USE_SIMULATED_UART   1
#define HAL_USE_SIMULATED_TIM    1
#define HAL_USE_SIMULATED_ADC    1
#define HAL_USE_SIMULATED_GPIO   1
#define HAL_USE_SIMULATED_DMA    1

// ==========================================
// SENSOR SIMULATION
// ==========================================

// Enable simulated sensor data injection
#define SIMULATED_IMU_ENABLE    1
#define SIMULATED_BARO_ENABLE   1
#define SIMULATED_MAG_ENABLE    1
#define SIMULATED_GPS_ENABLE    1

// ==========================================
// COMMUNICATION CONFIGURATION
// ==========================================

// SBUS simulation
#define SBUS_SIMULATED_INPUT    1
#define SBUS_DEFAULT_ROLL       0.0f    // Centered
#define SBUS_DEFAULT_PITCH      0.0f    // Centered
#define SBUS_DEFAULT_THROTTLE   0.0f    // Zero (disarmed)
#define SBUS_DEFAULT_YAW        0.0f    // Centered

// MSP communication
#define MSP_USE_STDIO           1       // Print MSP to console instead of UART

// Telemetry output
#define TELEMETRY_PRINT_TO_CONSOLE  1   // Print telemetry to console

// ==========================================
// DEBUG CONFIGURATION
// ==========================================

// Enable verbose debug output
#define DEBUG_ENABLED           1
#define DEBUG_LEVEL             2       // 0=off, 1=errors, 2=warnings, 3=info

// Logging
#define LOG_CONTROL_LOOP_RATE   0       // Don't log every iteration (too much output)
#define LOG_ATTITUDE            1       // Log attitude every telemetry cycle
#define LOG_MOTOR_OUTPUTS       1       // Log motor values
#define LOG_FAILSAFE_EVENTS     1       // Log failsafe state changes

// ==========================================
// CONTROL LOOP CONFIGURATION
// ==========================================

// Main control loop frequency
#define CONTROL_LOOP_FREQ_HZ    8000    // 8kHz

// Sub-task frequencies (derived from main loop)
#define ANGLE_CONTROLLER_DIVIDER    8   // 8kHz/8 = 1kHz
#define TELEMETRY_DIVIDER          80   // 8kHz/80 = 100Hz
#define BATTERY_CHECK_DIVIDER     800   // 8kHz/800 = 10Hz
#define GPS_UPDATE_DIVIDER        800   // 8kHz/800 = 10Hz
#define LOGGING_DIVIDER           800   // 8kHz/800 = 10Hz

// ==========================================
// FAILSAFE CONFIGURATION
// ==========================================

#define FAILSAFE_RC_TIMEOUT_MS      500   // 500ms without signal → failsafe
#define FAILSAFE_WARNING_DURATION_MS 100  // 100ms warning before disarm
#define FAILSAFE_RECOVERY_TIME_MS   500   // 500ms stable signal to recover

// ==========================================
// PID DEFAULT GAINS
// ==========================================

// Rate controller (inner loop)
#define RATE_ROLL_KP     4.0f
#define RATE_ROLL_KI     0.3f
#define RATE_ROLL_KD    18.0f

#define RATE_PITCH_KP    4.0f
#define RATE_PITCH_KI    0.3f
#define RATE_PITCH_KD   18.0f

#define RATE_YAW_KP      3.0f
#define RATE_YAW_KI      0.2f
#define RATE_YAW_KD      0.0f

// Angle controller (outer loop)
#define ANGLE_ROLL_KP    6.0f
#define ANGLE_PITCH_KP   6.0f

// Limits
#define RATE_OUTPUT_LIMIT     400.0f
#define YAW_OUTPUT_LIMIT      300.0f
#define ANGLE_OUTPUT_LIMIT    200.0f
#define MAX_TILT_ANGLE_DEG     45.0f

// ==========================================
// MAHONY FILTER GAINS
// ==========================================

#define MAHONY_KP   0.5f
#define MAHONY_KI   0.1f

// ==========================================
// MIXER CONFIGURATION
// ==========================================

#define PWM_MIN_VALUE   1000
#define PWM_MAX_VALUE   2000
#define PWM_IDLE_VALUE  1050

// ==========================================
// BATTERY CONFIGURATION
// ==========================================

#define BATTERY_CELLS           4       // 4S LiPo
#define BATTERY_WARNING_VOLTAGE 14.0f   // 3.5V per cell
#define BATTERY_CRITICAL_VOLTAGE 13.6f  // 3.4V per cell
#define BATTERY_EMERGENCY_VOLTAGE 12.8f // 3.2V per cell

// ==========================================
// HELPER MACROS
// ==========================================

// Debug print (goes to console in QEMU)
#if DEBUG_ENABLED
    #define DEBUG_PRINT(level, fmt, ...) \
        do { \
            if (level <= DEBUG_LEVEL) { \
                printf("[%s] " fmt "\n", \
                    level == 1 ? "ERROR" : \
                    level == 2 ? "WARN"  : \
                    level == 3 ? "INFO"  : "DEBUG", \
                    ##__VA_ARGS__); \
            } \
        } while(0)
#else
    #define DEBUG_PRINT(level, fmt, ...)
#endif

// ==========================================
// SIMULATED HARDWARE FUNCTIONS
// ==========================================

// These are implemented in a simulated_hardware.c file
// that provides fake sensor data for QEMU testing
// Note: millis() and micros() are declared in clock_config.h
//       and implemented in clock_config.c using simulated functions below

#ifdef __cplusplus
extern "C" {
#endif

// Initialize simulated hardware
void simulated_hardware_init(void);

// Get simulated microsecond timestamp
uint32_t simulated_micros(void);

// Get simulated millisecond timestamp
uint32_t simulated_millis(void);

// Inject simulated IMU data
void simulated_imu_set(float gyro_x, float gyro_y, float gyro_z,
                       float accel_x, float accel_y, float accel_z);

// Get simulated IMU data (called by HAL)
void simulated_imu_read(float *gx, float *gy, float *gz,
                        float *ax, float *ay, float *az);

// Inject simulated SBUS channels
void simulated_sbus_set_channels(const float channels[16]);

// Get simulated SBUS data (called by HAL)
bool simulated_sbus_read(float channels[16]);

// Set simulated battery voltage
void simulated_battery_set(float voltage, float current);

// Get simulated battery data (called by HAL)
void simulated_battery_read(float *voltage, float *current);

#ifdef __cplusplus
}
#endif

#endif // TARGET_QEMU_H