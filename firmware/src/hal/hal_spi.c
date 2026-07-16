#include "hal_spi.h"
#include "config/target_qemu.h"

void spi_init(SPIConfig *config) {
    
#if TARGET_QEMU
    DEBUG_PRINT(3, "SPI%d: Simulated init", config->bus + 1);
#else
    // Real STM32 SPI initialization
    // Enable clock, configure GPIO, set baud rate, enable peripheral
    DEBUG_PRINT(3, "SPI%d: Real init at %d Hz", config->bus + 1, config->baudrate);
#endif
}

uint8_t spi_transfer_byte(SPIBus bus, uint8_t data) {
#if TARGET_QEMU
    (void)bus;
    return data;  // Echo back in simulation
#else
    // Real SPI transfer
    // Wait for TXE, write DR, wait for RXNE, read DR
    (void)bus;
    return data;
#endif
}

void spi_transfer(SPIBus bus, uint8_t *tx_data, uint8_t *rx_data, uint16_t length) {
    for (uint16_t i = 0; i < length; i++) {
        rx_data[i] = spi_transfer_byte(bus, tx_data ? tx_data[i] : 0x00);
    }
}

void spi_read_register(SPIBus bus, uint8_t reg, uint8_t *data, uint16_t length) {
    spi_transfer_byte(bus, reg | 0x80);
    for (uint16_t i = 0; i < length; i++) {
        data[i] = spi_transfer_byte(bus, 0x00);
    }
}

void spi_write_register(SPIBus bus, uint8_t reg, uint8_t data) {
    spi_transfer_byte(bus, reg & 0x7F);
    spi_transfer_byte(bus, data);
}

void spi_chip_select(SPIBus bus, bool select) {
    (void)bus;
    (void)select;
}