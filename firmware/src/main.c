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

/**
 * @brief Initialize all system components
 */
void system_init(void) {
    
    // ─────────────────────────────────────
    // 1. Hardware initialization
    // ─────────────────────────────────────
    
    system_clock_init();
    scheduler_init(&sched, CONTROL_LOOP_FREQ_HZ);
    
#if TARGET_QEMU
    DEBUG_PRINT(3, "Target: QEMU Simulation");
    DEBUG_PRINT(3, "Initializing simulated hardware...");
    simulated_hardware_init();
#else
    DEBUG_PRINT(3, "Target: STM32F405 Hardware");
    
    // Initialize SPI for IMU
    SPIConfig spi_cfg = {
        .bus = SPI_BUS_1,
        .baudrate = 21000000,
        .cpol = 0,
        .cpha = 0,
        .msb_first = true
    };
    spi_init(&spi_cfg);
    DEBUG_PRINT(3, "SPI1 initialized at 21MHz");
    
    // Initialize UART for SBUS (UART2, 100kbps, inverted)
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
    
    // Initialize UART for MSP (UART3, 115200bps)
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
    
    // Initialize motor timer
    motor_timer_init();
    DEBUG_PRINT(3, "Motor timer initialized");
    
    // Initialize ADC for battery monitoring
    adc_init();
    DEBUG_PRINT(3, "ADC initialized");
    
    // Initialize IMU
    DEBUG_PRINT(3, "Initializing MPU6000 IMU...");
    if (!mpu6000_init()) {
        DEBUG_PRINT(1, "FATAL: IMU initialization failed!");
        while (1) {
            // Blink error code: IMU failure
            delay_ms(100);
        }
    }
    DEBUG_PRINT(3, "IMU initialized successfully");
    
    // Calibrate gyroscope
    DEBUG_PRINT(3, "Calibrating gyroscope (keep drone stationary)...");
    mpu6000_calibrate_gyro(1000);
    DEBUG_PRINT(3, "Gyroscope calibration complete");
#endif
    
    // ─────────────────────────────────────
    // 2. State estimation initialization
    // ─────────────────────────────────────
    
    mahony_init(&mahony, CONTROL_LOOP_FREQ_HZ);
    mahony_set_gains(&mahony, MAHONY_KP, MAHONY_KI);
    DEBUG_PRINT(3, "Mahony filter initialized");
    
    // ─────────────────────────────────────
    // 3. Controller initialization
    // ─────────────────────────────────────
    
    // Rate controller (inner loop, 8kHz)
    rate_controller_init(&rate_ctrl, 1.0f / CONTROL_LOOP_FREQ_HZ);
    rate_controller_set_gains(&rate_ctrl,
        RATE_ROLL_KP, RATE_ROLL_KI, RATE_ROLL_KD,
        RATE_PITCH_KP, RATE_PITCH_KI, RATE_PITCH_KD,
        RATE_YAW_KP, RATE_YAW_KI, RATE_YAW_KD);
    DEBUG_PRINT(3, "Rate controller initialized");
    
    // Angle controller (outer loop, ~1kHz)
    angle_controller_init(&angle_ctrl, 
        1.0f / (CONTROL_LOOP_FREQ_HZ / ANGLE_CONTROLLER_DIVIDER), 
        MAX_TILT_ANGLE_DEG);
    DEBUG_PRINT(3, "Angle controller initialized");
    
    // ─────────────────────────────────────
    // 4. Communication initialization
    // ─────────────────────────────────────
    
    sbus_init(&sbus);
    msp_init();
    DEBUG_PRINT(3, "SBUS and MSP initialized");
    
    // ─────────────────────────────────────
    // 5. Safety initialization
    // ─────────────────────────────────────
    
    failsafe_init(&failsafe, FAILSAFE_RC_TIMEOUT_MS);
    DEBUG_PRINT(3, "Failsafe initialized (timeout: %dms)", FAILSAFE_RC_TIMEOUT_MS);
    
    // ─────────────────────────────────────
    // 6. Initial state
    // ─────────────────────────────────────
    
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

/**
 * @brief Read IMU sensors (target-specific)
 */
void read_imu_sensors(void) {
#if TARGET_QEMU
    // Get simulated sensor data
    simulated_imu_read(
        &current_gyro[0], &current_gyro[1], &current_gyro[2],
        &current_accel[0], &current_accel[1], &current_accel[2]);
#else
    // Read from real MPU6000
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

/**
 * @brief Read RC channels (target-specific)
 */
void read_rc_input(void) {
#if TARGET_QEMU
    // Get simulated SBUS channels
    simulated_sbus_read(rc_channels);
#else
    // Read from real SBUS decoder
    for (int i = 0; i < SBUS_CHANNEL_COUNT; i++) {
        rc_channels[i] = sbus_get_channel(&sbus, i);
    }
#endif
}


// ==========================================
// MOTOR OUTPUT
// ==========================================

/**
 * @brief Output motor commands (target-specific)
 */
void output_motors(uint16_t motors[4]) {
#if TARGET_QEMU
    // Store for telemetry display
    motor_outputs[0] = motors[0];
    motor_outputs[1] = motors[1];
    motor_outputs[2] = motors[2];
    motor_outputs[3] = motors[3];
#else
    // Output to real motor timer
    motor_set_all(motors[0], motors[1], motors[2], motors[3]);
#endif
}


// ==========================================
// ARMING LOGIC
// ==========================================

/**
 * @brief Handle arm/disarm requests from pilot
 */
void handle_arming(void) {
    // Channel 5 is typically the arm switch on most radios
    float arm_channel = rc_channels[4];
    float throttle_raw = rc_channels[2];
    
    // ARM: Switch high + throttle low
    if (arm_channel > 0.5f && throttle_raw < 0.05f) {
        if (!armed) {
            armed = true;
            
            // Reset all controllers for clean start
            rate_controller_reset(&rate_ctrl);
            angle_controller_reset(&angle_ctrl);
            
            DEBUG_PRINT(3, "*** ARMED ***");
        }
    }
    
    // DISARM: Switch low
    if (arm_channel < -0.5f) {
        if (armed) {
            armed = false;
            throttle = 0.0f;
            
            DEBUG_PRINT(3, "*** DISARMED ***");
        }
    }
    
    // Set throttle only when armed
    if (armed) {
        throttle = throttle_raw;
    } else {
        throttle = 0.0f;
    }
}


// ==========================================
// FAILSAFE HANDLING
// ==========================================

/**
 * @brief Check and handle failsafe conditions
 */
void handle_failsafe(uint32_t now_ms) {
#if TARGET_QEMU
    // In simulation, assume frames are always available
    failsafe_update(&failsafe, now_ms, true);
#else
    // Real hardware: check if new SBUS frame was received
    failsafe_update(&failsafe, now_ms, sbus.new_frame_available);
#endif
    
    // If failsafe triggers, disarm
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

/**
 * @brief One iteration of the flight control loop
 * @param now_ms Current system time in milliseconds
 * 
 * This runs at 8kHz (every 125 microseconds).
 * It reads sensors, estimates attitude, runs cascaded PID,
 * and outputs motor commands.
 */
void control_loop_iteration(uint32_t now_ms) {
    
    // Get timing information
    float dt = scheduler_get_dt(&sched);
    uint32_t loop_count = scheduler_get_count(&sched);
    
    // ─────────────────────────────────────
    // STEP 1: Read IMU sensors
    // ─────────────────────────────────────
    read_imu_sensors();
    
    // ─────────────────────────────────────
    // STEP 2: Update Mahony filter
    // ─────────────────────────────────────
    mahony_update(&mahony,
        current_gyro[0], current_gyro[1], current_gyro[2],
        current_accel[0], current_accel[1], current_accel[2],
        dt);
    
    // Get current attitude estimate
    mahony_get_euler(&mahony,
        &current_attitude[0],
        &current_attitude[1],
        &current_attitude[2]);
    
    // ─────────────────────────────────────
    // STEP 3: Read pilot commands
    // ─────────────────────────────────────
    read_rc_input();
    
    // ─────────────────────────────────────
    // STEP 4: Handle arming
    // ─────────────────────────────────────
    handle_arming();
    
    // ─────────────────────────────────────
    // STEP 5: Check failsafe
    // ─────────────────────────────────────
    handle_failsafe(now_ms);
    
    // ─────────────────────────────────────
    // STEP 6: Extract stick commands
    // ─────────────────────────────────────
    float roll_stick  = rc_channels[0];   // -1.0 to +1.0
    float pitch_stick = rc_channels[1];   // -1.0 to +1.0
    float yaw_stick   = rc_channels[3];   // -1.0 to +1.0
    
    // ─────────────────────────────────────
    // STEP 7: Angle Controller (1kHz)
    // ─────────────────────────────────────
    static float roll_rate_set = 0.0f;
    static float pitch_rate_set = 0.0f;
    
    if (scheduler_slow_task_due(loop_count, ANGLE_CONTROLLER_DIVIDER)) {
        // Convert stick positions to target angles
        float max_angle = MAX_TILT_ANGLE_DEG * M_PI / 180.0f;
        float roll_target  = roll_stick * max_angle;
        float pitch_target = pitch_stick * max_angle;
        
        // Run angle controller
        angle_controller_update(&angle_ctrl,
            roll_target, pitch_target,
            current_attitude[0], current_attitude[1],
            &roll_rate_set, &pitch_rate_set);
    }
    
    // ─────────────────────────────────────
    // STEP 8: Rate Controller (8kHz)
    // ─────────────────────────────────────
    float max_yaw_rate = 200.0f * M_PI / 180.0f;  // 200°/s
    float yaw_rate_set = yaw_stick * max_yaw_rate;
    
    float roll_corr, pitch_corr, yaw_corr;
    rate_controller_update(&rate_ctrl,
        roll_rate_set, pitch_rate_set, yaw_rate_set,
        current_gyro[0], current_gyro[1], current_gyro[2],
        &roll_corr, &pitch_corr, &yaw_corr);
    
    // ─────────────────────────────────────
    // STEP 9: Mixer → Motor outputs
    // ─────────────────────────────────────
    uint16_t motors[4];
    
    if (armed && !failsafe_should_disarm(&failsafe)) {
        mixer_quadx(throttle, roll_corr, pitch_corr, yaw_corr, motors);
    } else {
        mixer_stop(motors);
    }
    
    output_motors(motors);
    
    // ─────────────────────────────────────
    // STEP 10: Telemetry (100Hz)
    // ─────────────────────────────────────
    if (scheduler_slow_task_due(loop_count, TELEMETRY_DIVIDER)) {
#if TELEMETRY_PRINT_TO_CONSOLE
        // Convert radians to degrees for display
        float roll_deg  = current_attitude[0] * 180.0f / M_PI;
        float pitch_deg = current_attitude[1] * 180.0f / M_PI;
        float yaw_deg   = current_attitude[2] * 180.0f / M_PI;
        
        DEBUG_PRINT(3, "Att: R=%5.1f° P=%5.1f° Y=%5.1f° | "
                       "Motors: %d %d %d %d | %s",
            roll_deg, pitch_deg, yaw_deg,
            motors[0], motors[1], motors[2], motors[3],
            armed ? "ARMED" : "DISARMED");
#endif
        
        // Send MSP telemetry if connected
        // msp_send_attitude(current_attitude[0], current_attitude[1], current_attitude[2]);
    }
    
    // ─────────────────────────────────────
    // STEP 11: Slow tasks (10Hz)
    // ─────────────────────────────────────
    if (scheduler_slow_task_due(loop_count, BATTERY_CHECK_DIVIDER)) {
        // Battery monitoring
        float battery_voltage = adc_read_battery_voltage();
        uint8_t battery_percent = adc_get_battery_percent();
        
        // Low battery warning
        if (battery_voltage < BATTERY_WARNING_VOLTAGE && battery_voltage > 0) {
            DEBUG_PRINT(2, "Battery low: %.1fV (%d%%)", battery_voltage, battery_percent);
        }
    }
    
    // ─────────────────────────────────────
    // STEP 12: Logging (10Hz)
    // ─────────────────────────────────────
    if (scheduler_slow_task_due(loop_count, LOGGING_DIVIDER)) {
        // Blackbox logging would go here
    }
    
    // Update statistics
    total_loop_iterations++;
}


// ==========================================
// MAIN FUNCTION
// ==========================================

/**
 * @brief Main entry point
 * 
 * Initializes all systems and enters the main control loop.
 * The control loop runs at exactly 8kHz using the scheduler.
 */
int main(void) {
    
    // ─────────────────────────────────────
    // Print startup banner
    // ─────────────────────────────────────
    DEBUG_PRINT(3, "");
    DEBUG_PRINT(3, "╔══════════════════════════════════════════╗");
    DEBUG_PRINT(3, "║     BARE-METAL FLIGHT CONTROLLER         ║");
    DEBUG_PRINT(3, "║     v1.0.0 — QEMU Simulation             ║");
    DEBUG_PRINT(3, "╚══════════════════════════════════════════╝");
    DEBUG_PRINT(3, "");
    
    // ─────────────────────────────────────
    // Initialize all systems
    // ─────────────────────────────────────
    system_init();
    
    DEBUG_PRINT(3, "Entering main control loop at %d Hz", CONTROL_LOOP_FREQ_HZ);
    DEBUG_PRINT(3, "Loop period: %.1f microseconds", 1000000.0f / CONTROL_LOOP_FREQ_HZ);
    DEBUG_PRINT(3, "");
    
    // ─────────────────────────────────────
    // Main control loop
    // ─────────────────────────────────────
    while (1) {
        
        // Get current time
        uint32_t now_us = micros();
        uint32_t now_ms = millis();
        
        // Check if it's time to run the control loop
        if (!scheduler_should_run(&sched, now_us)) {
            continue;  // Not yet — keep waiting
        }
        
        // Run one control loop iteration
        control_loop_iteration(now_ms);
    }
    
    // Never reached
    return 0;
}


// ==========================================
// INTERRUPT HANDLERS (for real hardware)
// ==========================================

#if !TARGET_QEMU

/**
 * @brief SysTick interrupt handler
 * Called every 1ms for system timing
 */
void SysTick_Handler(void) {
    // System tick is handled in clock_config.c
}

/**
 * @brief UART2 interrupt handler (SBUS)
 * Called for each byte received from the radio receiver
 */
void USART2_IRQHandler(void) {
    if (USART2->SR & USART_SR_RXNE) {
        uint8_t byte = USART2->DR;
        sbus_parse_byte(&sbus, byte);
    }
}

/**
 * @brief UART3 interrupt handler (MSP)
 * Called for each byte received from the configurator
 */
void USART3_IRQHandler(void) {
    if (USART3->SR & USART_SR_RXNE) {
        uint8_t byte = USART3->DR;
        
        if (msp_parse_byte(byte)) {
            // Complete MSP packet received
            const MSPPacket *request = msp_get_packet();
            int response_size = msp_process_command(request, msp_response);
            
            // Send response
            if (response_size > 0) {
                for (int i = 0; i < response_size; i++) {
                    while (!(USART3->SR & USART_SR_TXE));
                    USART3->DR = msp_response[i];
                }
            }
        }
    }
}

/**
 * @brief EXTI interrupt for IMU data ready
 * Signals that new gyro/accel data is available
 */
void EXTI4_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR4) {
        EXTI->PR = EXTI_PR_PR4;  // Clear interrupt flag
        // Data ready flag is checked in the main loop
    }
}

/**
 * @brief Hard fault handler
 * Called on CPU faults (invalid memory access, etc.)
 */
void HardFault_Handler(void) {
    // Disarm motors immediately
    motor_stop_all();
    
    // Blink LED rapidly to indicate fault
    while (1) {
        // Toggle LED
        delay_ms(50);
    }
}

#endif // !TARGET_QEMU


// ==========================================
// RETARGET PRINTF (for QEMU/debug output)
// ==========================================

#if TARGET_QEMU
/**
 * @brief Retarget printf to console for QEMU simulation
 */
int _write(int file, char *ptr, int len) {
    for (int i = 0; i < len; i++) {
        putchar(ptr[i]);
    }
    return len;
}
#endif