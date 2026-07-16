#ifndef HAL_ADC_H
#define HAL_ADC_H

#include <stdint.h>

// ADC channels for battery monitoring
#define ADC_CHANNEL_VBAT    0
#define ADC_CHANNEL_CURRENT 1

// Battery voltage divider ratio
// Example: 10k/1k divider → ratio = (10+1)/1 = 11
#define VBAT_DIVIDER_RATIO  11.0f

// Initialize ADC for battery monitoring
void adc_init(void);

// Read raw ADC value (0-4095 for 12-bit)
uint16_t adc_read_raw(uint8_t channel);

// Read battery voltage (in volts)
float adc_read_battery_voltage(void);

// Read battery current (in amps)
float adc_read_battery_current(void);

// Get estimated battery percentage (0-100)
uint8_t adc_get_battery_percent(void);

#endif // HAL_ADC_H