#ifndef HAL_SPI_H
#define HAL_SPI_H

#include <stdint.h>
#include <stdbool.h>

// SPI bus selection
typedef enum {
    SPI_BUS_1 = 0,
    SPI_BUS_2,
    SPI_BUS_3,
    SPI_BUS_4
} SPIBus;

// SPI configuration structure
typedef struct {
    SPIBus bus;
    uint32_t baudrate;        // e.g., 21000000 (21MHz)
    bool cpol;                // Clock polarity (0 or 1)
    bool cpha;                // Clock phase (0 or 1)
    bool msb_first;           // Data order
} SPIConfig;

// Initialize an SPI bus
void spi_init(SPIConfig *config);

// Transfer one byte (send and receive simultaneously)
uint8_t spi_transfer_byte(SPIBus bus, uint8_t data);

// Transfer multiple bytes
void spi_transfer(SPIBus bus, uint8_t *tx_data, uint8_t *rx_data, uint16_t length);

// Read from a register (most IMUs use this pattern)
// Sends register address with read bit, then reads data
void spi_read_register(SPIBus bus, uint8_t reg, uint8_t *data, uint16_t length);

// Write to a register
void spi_write_register(SPIBus bus, uint8_t reg, uint8_t data);

// Select/deselect chip (CS pin control)
void spi_chip_select(SPIBus bus, bool select);

#endif // HAL_SPI_H