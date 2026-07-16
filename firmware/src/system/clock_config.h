#ifndef CLOCK_CONFIG_H
#define CLOCK_CONFIG_H

#include <stdint.h>

// System clock frequencies after initialization
#define SYSTEM_CLOCK_HZ    168000000   // 168 MHz
#define AHB_CLOCK_HZ       168000000   // AHB bus
#define APB1_CLOCK_HZ       42000000   // APB1 (timers, UART2-5, I2C)
#define APB2_CLOCK_HZ       84000000   // APB2 (SPI1, TIM1, UART1)

// Initialize the STM32F405 clock tree
// HSE (8MHz) → PLL (×336, /2) → SYSCLK (168MHz)
void system_clock_init(void);

// Get current system clock in Hz
uint32_t get_system_clock(void);

// Simple delay in milliseconds (blocking)
void delay_ms(uint32_t ms);

// Simple delay in microseconds (blocking)
void delay_us(uint32_t us);

// Get system uptime in milliseconds
uint32_t millis(void);

// Get system uptime in microseconds
uint32_t micros(void);

#endif // CLOCK_CONFIG_H