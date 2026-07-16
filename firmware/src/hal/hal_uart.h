#ifndef HAL_UART_H
#define HAL_UART_H

#include <stdint.h>
#include <stdbool.h>

// UART port selection
typedef enum {
    UART_PORT_1 = 0,
    UART_PORT_2,
    UART_PORT_3,
    UART_PORT_4,
    UART_PORT_5,
    UART_PORT_6
} UARTPort;

// UART configuration
typedef struct {
    UARTPort port;
    uint32_t baudrate;        // e.g., 115200
    uint8_t data_bits;        // 8
    uint8_t stop_bits;        // 1
    bool parity;              // false = none
    bool inverted;            // true for SBUS (inverted signal)
} UARTConfig;

// Callback type for received data
typedef void (*UARTRxCallback)(uint8_t byte);

// Initialize a UART port
void uart_init(UARTConfig *config);

// Send a single byte
void uart_send_byte(UARTPort port, uint8_t byte);

// Send multiple bytes
void uart_send(UARTPort port, const uint8_t *data, uint16_t length);

// Check if data is available to read
bool uart_data_available(UARTPort port);

// Read a single byte
uint8_t uart_read_byte(UARTPort port);

// Register a callback for received data
void uart_set_rx_callback(UARTPort port, UARTRxCallback callback);

// For SBUS: configure inverted serial
void uart_set_inverted(UARTPort port, bool inverted);

#endif // HAL_UART_H