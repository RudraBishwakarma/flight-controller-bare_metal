#ifndef TARGET_STM32F405_H
#define TARGET_STM32F405_H

// ==========================================
// BUILD TARGET IDENTIFICATION
// ==========================================

#define TARGET_QEMU         0
#define TARGET_STM32F405    1

// ==========================================
// SYSTEM CLOCK CONFIGURATION
// ==========================================

#define SYSTEM_CLOCK_HZ     168000000
#define AHB_CLOCK_HZ        168000000
#define APB1_CLOCK_HZ        42000000
#define APB2_CLOCK_HZ        84000000

#define SYSTICK_FREQ_HZ     1000

// ==========================================
// MEMORY CONFIGURATION
// ==========================================

#define FLASH_BASE          0x08000000
#define FLASH_SIZE          (1024 * 1024)
#define RAM_BASE            0x20000000
#define RAM_SIZE            (192 * 1024)

// ==========================================
// PERIPHERAL ENABLE/DISABLE
// ==========================================

#define HAL_USE_SIMULATED_SPI    0
#define HAL_USE_SIMULATED_I2C    0
#define HAL_USE_SIMULATED_UART   0
#define HAL_USE_SIMULATED_TIM    0
#define HAL_USE_SIMULATED_ADC    0
#define HAL_USE_SIMULATED_GPIO   0
#define HAL_USE_SIMULATED_DMA    0

// ==========================================
// PIN MAPPINGS (STM32F405RGT6)
// ==========================================

// SPI1 — IMU (MPU6000)
#define IMU_SPI             SPI1
#define IMU_SPI_SCK_PIN     GPIO_PIN_5
#define IMU_SPI_SCK_PORT    GPIOA
#define IMU_SPI_MISO_PIN    GPIO_PIN_6
#define IMU_SPI_MISO_PORT   GPIOA
#define IMU_SPI_MOSI_PIN    GPIO_PIN_7
#define IMU_SPI_MOSI_PORT   GPIOA
#define IMU_SPI_CS_PIN      GPIO_PIN_4
#define IMU_SPI_CS_PORT     GPIOA

// UART2 — SBUS Receiver
#define SBUS_UART           USART2
#define SBUS_UART_RX_PIN    GPIO_PIN_3
#define SBUS_UART_RX_PORT   GPIOA

// UART3 — Telemetry/MSP
#define MSP_UART            USART3
#define MSP_UART_TX_PIN     GPIO_PIN_10
#define MSP_UART_TX_PORT    GPIOB
#define MSP_UART_RX_PIN     GPIO_PIN_11
#define MSP_UART_RX_PORT    GPIOB

// TIM1 — Motor outputs (CH1-4)
#define MOTOR_TIM           TIM1
#define MOTOR_CH1_PIN       GPIO_PIN_8
#define MOTOR_CH1_PORT      GPIOA
#define MOTOR_CH2_PIN       GPIO_PIN_9
#define MOTOR_CH2_PORT      GPIOA
#define MOTOR_CH3_PIN       GPIO_PIN_10
#define MOTOR_CH3_PORT      GPIOA
#define MOTOR_CH4_PIN       GPIO_PIN_11
#define MOTOR_CH4_PORT      GPIOA

// ADC1 — Battery monitoring
#define VBAT_ADC            ADC1
#define VBAT_ADC_CHANNEL    ADC_CHANNEL_1
#define VBAT_ADC_PIN        GPIO_PIN_1
#define VBAT_ADC_PORT       GPIOC

// LED
#define STATUS_LED_PIN      GPIO_PIN_13
#define STATUS_LED_PORT     GPIOC

// ==========================================
// SAME CONTROL LOOP CONFIGURATION
// ==========================================

#define CONTROL_LOOP_FREQ_HZ    8000
#define ANGLE_CONTROLLER_DIVIDER    8
#define TELEMETRY_DIVIDER          80
#define BATTERY_CHECK_DIVIDER     800

// ==========================================
// SAME PID & FILTER GAINS
// ==========================================

#define RATE_ROLL_KP     4.0f
#define RATE_ROLL_KI     0.3f
#define RATE_ROLL_KD    18.0f
#define RATE_PITCH_KP    4.0f
#define RATE_PITCH_KI    0.3f
#define RATE_PITCH_KD   18.0f
#define RATE_YAW_KP      3.0f
#define RATE_YAW_KI      0.2f
#define RATE_YAW_KD      0.0f

#define ANGLE_ROLL_KP    6.0f
#define ANGLE_PITCH_KP   6.0f

#define MAHONY_KP   0.5f
#define MAHONY_KI   0.1f

#define MAX_TILT_ANGLE_DEG  45.0f

// ==========================================
// FAILSAFE
// ==========================================

#define FAILSAFE_RC_TIMEOUT_MS      500
#define FAILSAFE_WARNING_DURATION_MS 100
#define FAILSAFE_RECOVERY_TIME_MS   500

// ==========================================
// BATTERY
// ==========================================

#define BATTERY_CELLS           4
#define BATTERY_WARNING_VOLTAGE 14.0f
#define BATTERY_CRITICAL_VOLTAGE 13.6f
#define BATTERY_EMERGENCY_VOLTAGE 12.8f

// ==========================================
// MIXER
// ==========================================

#define PWM_MIN_VALUE   1000
#define PWM_MAX_VALUE   2000
#define PWM_IDLE_VALUE  1050

#endif // TARGET_STM32F405_H