#include "hal_adc.h"
#include "config/target_qemu.h"

static float simulated_voltage = 16.8f;
static float simulated_current = 0.5f;

void adc_init(void) {
#if TARGET_QEMU
    DEBUG_PRINT(3, "ADC: Simulated init");
#else
    DEBUG_PRINT(3, "ADC: ADC1 init for battery monitoring");
#endif
}

uint16_t adc_read_raw(uint8_t channel) {
    (void)channel;
    return 0;
}

float adc_read_battery_voltage(void) {
#if TARGET_QEMU
    return simulated_voltage;
#else
    // Real ADC reading with voltage divider calculation
    return 16.8f;
#endif
}

float adc_read_battery_current(void) {
#if TARGET_QEMU
    return simulated_current;
#else
    return 0.5f;
#endif
}

uint8_t adc_get_battery_percent(void) {
    float voltage = adc_read_battery_voltage();
    float cell_voltage = voltage / BATTERY_CELLS;
    
    if (cell_voltage >= 4.2f) return 100;
    if (cell_voltage <= 3.2f) return 0;
    return (uint8_t)((cell_voltage - 3.2f) / (4.2f - 3.2f) * 100.0f);
}