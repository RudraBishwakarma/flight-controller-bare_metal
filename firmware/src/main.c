/**
 * @file main.c
 * @brief Bare-Metal Drone Flight Controller — Main Entry Point
 * 
 * This is the complete integration of all flight controller modules:
 * - Time-triggered scheduler (8kHz control loop)
 * - Mahony sensor fusion filter
 * - Cascaded PID control (Angle → Rate)
 * - SBUS radio protocol decoder
 * - MSP configuration protocol
 * - RC failsafe state machine
 * - Motor mixer (Quad-X configuration)
 * 
 * Target: STM32F405 (or QEMU simulation)
 * Architecture: Bare-metal cyclic executive (no RTOS)
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>

// ==========================================
// TARGET CONFIGURATION
// ==========================================
#include "config/target_qemu.h"

// ==========================================
// SYSTEM
// ==========================================
#include "system/clock_config.h"
#include "system/scheduler.h"

// ==========================================
// HARDWARE ABSTRACTION LAYER
// ==========================================
#include "hal/hal_spi.h"
#include "hal/hal_uart.h"
#include "hal/hal_tim.h"
#include "hal/hal_adc.h"

// ==========================================
// SENSOR DRIVERS
// ==========================================
#if !TARGET_QEMU
#include "drivers/imu/mpu6000.h"
#endif

// ==========================================
// STATE ESTIMATION
// ==========================================
#include "estimation/mahony_filter.h"

// ==========================================
// CONTROL
// ==========================================
#include "control/pid.h"
#include "control/rate_controller.h"
#include "control/angle_controller.h"
#include "control/mixer.h"

// ==========================================
// COMMUNICATION
// ==========================================
#include "communication/sbus_decoder.h"
#include "communication/msp_protocol.h"

// ==========================================
// SAFETY
// ==========================================
#include "failsafe/rc_failsafe.h"


// ==========================================
// FORWARD DECLARATIONS
// ==========================================
void output_motors(uint16_t motors[4]);
void read_imu_sensors(void);
void read_rc_input(void);
void handle_arming(void);
void handle_failsafe(uint32_t now_ms);
void control_loop_iteration(uint32_t now_ms);


// ==========================================
// GLOBAL OBJECTS — FLIGHT CONTROLLER STATE
// ==========================================

// Timing
static Scheduler sched;

// State estimation
static MahonyFilter mahony;

// Controllers
static RateController rate_ctrl;
static AngleController angle_ctrl;

// Communication
static SBUSData sbus;
static uint8_t msp_response[128];

// Safety
static RCFailsafe failsafe;

// Flight state
static bool armed = false;
static float throttle = 0.0f;

// Sensor data
static float current_gyro[3] = {0.0f, 0.0f, 0.0f};
static float current_accel[3] = {0.0f, 0.0f, 9.81f};  // Start with gravity
static float current_attitude[3] = {0.0f, 0.0f, 0.0f}; // roll, pitch, yaw

// RC channels (normalized)
static float rc_channels[16] = {0.0f};

// Motor outputs
static uint16_t motor_outputs[4] = {PWM_MIN_VALUE, PWM_MIN_VALUE, PWM_MIN_VALUE, PWM_MIN_VALUE};

// Statistics
static uint32_t total_loop_iterations = 0;
static uint32_t failsafe_events = 0;


// ==========================================
// SYSTEM INITIALIZATION
// ==========================================

void system_init(void) {
    
    system_clock_init();
    scheduler_init(&sched, CONTROL_LOOP_FREQ_HZ);
    
#if TARGET_QEMU
    DEBUG_PRINT(3, "Target: QEMU Simulation");
    DEBUG_PRINT(3, "Initializing simulated hardware...");
    simulated_hardware_init();
#else
    DEBUG_PRINT(3, "Target: STM32F405 Hardware");
    
    SPIConfig spi_cfg = {
        .bus = SPI_BUS_1,
        .baudrate = 21000000,
        .cpol = 0,
        .cpha = 0,
        .msb_first = true
    };
    spi_init(&spi_cfg);
    DEBUG_PRINT(3, "SPI1 initialized at 21MHz");
    
    UARTConfig sbus_uart = {
        .port = UART_PORT_2,
        .baudrate = 100000,
        .data_bits = 8,
        .stop_bits = 2,
        .parity = true,
        .inverted = true
    };
    uart_init(&sbus_uart);
    DEBUG_PRINT(3, "SBUS UART initialized");
    
    UARTConfig msp_uart = {
        .port = UART_PORT_3,
        .baudrate = 115200,
        .data_bits = 8,
        .stop_bits = 1,
        .parity = false,
        .inverted = false
    };
    uart_init(&msp_uart);
    DEBUG_PRINT(3, "MSP UART initialized");
    
    motor_timer_init();
    DEBUG_PRINT(3, "Motor timer initialized");
    
    adc_init();
    DEBUG_PRINT(3, "ADC initialized");
    
    DEBUG_PRINT(3, "Initializing MPU6000 IMU...");
    if (!mpu6000_init()) {
        DEBUG_PRINT(1, "FATAL: IMU initialization failed!");
        while (1) {
            delay_ms(100);
        }
    }
    DEBUG_PRINT(3, "IMU initialized successfully");
    
    DEBUG_PRINT(3, "Calibrating gyroscope (keep drone stationary)...");
    mpu6000_calibrate_gyro(1000);
    DEBUG_PRINT(3, "Gyroscope calibration complete");
#endif
    
    mahony_init(&mahony, CONTROL_LOOP_FREQ_HZ);
    mahony_set_gains(&mahony, MAHONY_KP, MAHONY_KI);
    DEBUG_PRINT(3, "Mahony filter initialized");
    
    rate_controller_init(&rate_ctrl, 1.0f / CONTROL_LOOP_FREQ_HZ);
    rate_controller_set_gains(&rate_ctrl,
        RATE_ROLL_KP, RATE_ROLL_KI, RATE_ROLL_KD,
        RATE_PITCH_KP, RATE_PITCH_KI, RATE_PITCH_KD,
        RATE_YAW_KP, RATE_YAW_KI, RATE_YAW_KD);
    DEBUG_PRINT(3, "Rate controller initialized");
    
    angle_controller_init(&angle_ctrl, 
        1.0f / (CONTROL_LOOP_FREQ_HZ / ANGLE_CONTROLLER_DIVIDER), 
        MAX_TILT_ANGLE_DEG);
    DEBUG_PRINT(3, "Angle controller initialized");
    
    sbus_init(&sbus);
    msp_init();
    DEBUG_PRINT(3, "SBUS and MSP initialized");
    
    failsafe_init(&failsafe, FAILSAFE_RC_TIMEOUT_MS);
    DEBUG_PRINT(3, "Failsafe initialized (timeout: %dms)", FAILSAFE_RC_TIMEOUT_MS);
    
    armed = false;
    throttle = 0.0f;
    mixer_stop(motor_outputs);
    output_motors(motor_outputs);
    
    DEBUG_PRINT(3, "System initialization complete");
    DEBUG_PRINT(3, "Ready for arming");
}


// ==========================================
// SENSOR READING
// ==========================================

void read_imu_sensors(void) {
#if TARGET_QEMU
    simulated_imu_read(
        &current_gyro[0], &current_gyro[1], &current_gyro[2],
        &current_accel[0], &current_accel[1], &current_accel[2]);
#else
    MPU6000_Data imu_data;
    mpu6000_read(&imu_data);
    current_gyro[0] = imu_data.gyro_x;
    current_gyro[1] = imu_data.gyro_y;
    current_gyro[2] = imu_data.gyro_z;
    current_accel[0] = imu_data.accel_x;
    current_accel[1] = imu_data.accel_y;
    current_accel[2] = imu_data.accel_z;
#endif
}


// ==========================================
// RC INPUT READING
// ==========================================

void read_rc_input(void) {
#if TARGET_QEMU
    simulated_sbus_read(rc_channels);
#else
    for (int i = 0; i < SBUS_CHANNEL_COUNT; i++) {
        rc_channels[i] = sbus_get_channel(&sbus, i);
    }
#endif
}


// ==========================================
// MOTOR OUTPUT
// ==========================================

void output_motors(uint16_t motors[4]) {
#if TARGET_QEMU
    motor_outputs[0] = motors[0];
    motor_outputs[1] = motors[1];
    motor_outputs[2] = motors[2];
    motor_outputs[3] = motors[3];
#else
    motor_set_all(motors[0], motors[1], motors[2], motors[3]);
#endif
}


// ==========================================
// ARMING LOGIC
// ==========================================

void handle_arming(void) {
    float arm_channel = rc_channels[4];
    float throttle_raw = rc_channels[2];
    
    if (arm_channel > 0.5f && throttle_raw < 0.05f) {
        if (!armed) {
            armed = true;
            rate_controller_reset(&rate_ctrl);
            angle_controller_reset(&angle_ctrl);
            DEBUG_PRINT(3, "*** ARMED ***");
        }
    }
    
    if (arm_channel < -0.5f) {
        if (armed) {
            armed = false;
            throttle = 0.0f;
            DEBUG_PRINT(3, "*** DISARMED ***");
        }
    }
    
    throttle = armed ? throttle_raw : 0.0f;
}


// ==========================================
// FAILSAFE HANDLING
// ==========================================

void handle_failsafe(uint32_t now_ms) {
#if TARGET_QEMU
    failsafe_update(&failsafe, now_ms, true);
#else
    failsafe_update(&failsafe, now_ms, sbus.new_frame_available);
#endif
    
    if (failsafe_should_disarm(&failsafe)) {
        if (armed) {
            armed = false;
            throttle = 0.0f;
            failsafe_events++;
            DEBUG_PRINT(2, "FAILSAFE ACTIVE — Motors disarmed");
        }
    }
}


// ==========================================
// CONTROL LOOP (8kHz)
// ==========================================

void control_loop_iteration(uint32_t now_ms) {
    
    float dt = scheduler_get_dt(&sched);
    uint32_t loop_count = scheduler_get_count(&sched);
    
    read_imu_sensors();
    
    mahony_update(&mahony,
        current_gyro[0], current_gyro[1], current_gyro[2],
        current_accel[0], current_accel[1], current_accel[2],
        dt);
    
    mahony_get_euler(&mahony,
        &current_attitude[0],
        &current_attitude[1],
        &current_attitude[2]);
    
    read_rc_input();
    handle_arming();
    handle_failsafe(now_ms);
    
    float roll_stick  = rc_channels[0];
    float pitch_stick = rc_channels[1];
    float yaw_stick   = rc_channels[3];
    
    static float roll_rate_set = 0.0f;
    static float pitch_rate_set = 0.0f;
    
    if (scheduler_slow_task_due(loop_count, ANGLE_CONTROLLER_DIVIDER)) {
        float max_angle = MAX_TILT_ANGLE_DEG * M_PI / 180.0f;
        float roll_target  = roll_stick * max_angle;
        float pitch_target = pitch_stick * max_angle;
        
        angle_controller_update(&angle_ctrl,
            roll_target, pitch_target,
            current_attitude[0], current_attitude[1],
            &roll_rate_set, &pitch_rate_set);
    }
    
    float max_yaw_rate = 200.0f * M_PI / 180.0f;
    float yaw_rate_set = yaw_stick * max_yaw_rate;
    
    float roll_corr, pitch_corr, yaw_corr;
    rate_controller_update(&rate_ctrl,
        roll_rate_set, pitch_rate_set, yaw_rate_set,
        current_gyro[0], current_gyro[1], current_gyro[2],
        &roll_corr, &pitch_corr, &yaw_corr);
    
    uint16_t motors[4];
    
    if (armed && !failsafe_should_disarm(&failsafe)) {
        mixer_quadx(throttle, roll_corr, pitch_corr, yaw_corr, motors);
    } else {
        mixer_stop(motors);
    }
    
    output_motors(motors);
    
    if (scheduler_slow_task_due(loop_count, TELEMETRY_DIVIDER)) {
#if TELEMETRY_PRINT_TO_CONSOLE
        float roll_deg  = current_attitude[0] * 180.0f / M_PI;
        float pitch_deg = current_attitude[1] * 180.0f / M_PI;
        float yaw_deg   = current_attitude[2] * 180.0f / M_PI;
        
        DEBUG_PRINT(3, "Att: R=%5.1f P=%5.1f Y=%5.1f | M: %d %d %d %d | %s",
            roll_deg, pitch_deg, yaw_deg,
            motors[0], motors[1], motors[2], motors[3],
            armed ? "ARMED" : "DISARMED");
#endif
    }
    
    if (scheduler_slow_task_due(loop_count, BATTERY_CHECK_DIVIDER)) {
        float battery_voltage = adc_read_battery_voltage();
        uint8_t battery_percent = adc_get_battery_percent();
        if (battery_voltage < BATTERY_WARNING_VOLTAGE && battery_voltage > 0) {
            DEBUG_PRINT(2, "Battery low: %.1fV (%d%%)", battery_voltage, battery_percent);
        }
    }
    
    total_loop_iterations++;
}


// ==========================================
// MAIN FUNCTION
// ==========================================

int main(void) {
    
    DEBUG_PRINT(3, "");
    DEBUG_PRINT(3, "========================================");
    DEBUG_PRINT(3, "  BARE-METAL FLIGHT CONTROLLER v1.0.0");
    DEBUG_PRINT(3, "========================================");
    DEBUG_PRINT(3, "");
    
    system_init();
    
    DEBUG_PRINT(3, "Entering main control loop at %d Hz", CONTROL_LOOP_FREQ_HZ);
    DEBUG_PRINT(3, "");
    
    while (1) {
        uint32_t now_us = micros();
        uint32_t now_ms = millis();
        
        if (!scheduler_should_run(&sched, now_us)) {
            continue;
        }
        
        control_loop_iteration(now_ms);
    }
    
    return 0;
}


// ==========================================
// RETARGET PRINTF
// ==========================================

#if TARGET_QEMU
int _write(int file, char *ptr, int len) {
    (void)file;
    for (int i = 0; i < len; i++) {
        putchar(ptr[i]);
    }
    return len;
}
#endif