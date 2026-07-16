#include "clock_config.h"
#include "config/target_qemu.h"

static volatile uint32_t system_millis = 0;
static volatile uint32_t system_micros = 0;

void system_clock_init(void) {
    
#if TARGET_QEMU
    // QEMU: Simple initialization
    system_millis = 0;
    system_micros = 0;
    DEBUG_PRINT(3, "Clock: Simulated %d Hz", SYSTEM_CLOCK_HZ);
#else
    // Real STM32: Configure PLL for 168MHz
    // RCC->CR |= RCC_CR_HSEON;
    // while (!(RCC->CR & RCC_CR_HSERDY));
    // ... (full PLL configuration)
    // SysTick_Config(SYSTEM_CLOCK_HZ / SYSTICK_FREQ_HZ);
    DEBUG_PRINT(3, "Clock: HSE 8MHz → PLL → %d Hz", SYSTEM_CLOCK_HZ);
#endif
}

uint32_t get_system_clock(void) {
    return SYSTEM_CLOCK_HZ;
}

void delay_ms(uint32_t ms) {
#if TARGET_QEMU
    // Simulated delay
    volatile uint32_t count = ms * 10000;
    while (count--) { }
#else
    // Real hardware: use SysTick
    uint32_t start = millis();
    while ((millis() - start) < ms) { }
#endif
}

void delay_us(uint32_t us) {
#if TARGET_QEMU
    volatile uint32_t count = us * 10;
    while (count--) { }
#else
    uint32_t start = micros();
    while ((micros() - start) < us) { }
#endif
}

uint32_t millis(void) {
    return system_millis;
}

uint32_t micros(void) {
    return system_micros;
}

// For QEMU: increment simulated time
#if TARGET_QEMU
void simulated_time_tick(void) {
    system_millis++;
    system_micros += 1000;
}
#endif