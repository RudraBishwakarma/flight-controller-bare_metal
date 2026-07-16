#include "hal_uart.h"
#include "config/target_qemu.h"

static UARTRxCallback rx_callbacks[6] = {NULL};

void uart_init(UARTConfig *config) {
    
#if TARGET_QEMU
    DEBUG_PRINT(3, "UART%d: Simulated init at %d baud", config->port + 1, config->baudrate);
#else
    DEBUG_PRINT(3, "UART%d: Real init at %d baud", config->port + 1, config->baudrate);
    // Real STM32 UART initialization
#endif
}

void uart_send_byte(UARTPort port, uint8_t byte) {
#if TARGET_QEMU
    #if MSP_USE_STDIO
        putchar(byte);
    #endif
#else
    // Real UART: wait TXE, write DR
#endif
    (void)port;
}

void uart_send(UARTPort port, const uint8_t *data, uint16_t length) {
    for (uint16_t i = 0; i < length; i++) {
        uart_send_byte(port, data[i]);
    }
}

bool uart_data_available(UARTPort port) {
    (void)port;
    return false;
}

uint8_t uart_read_byte(UARTPort port) {
    (void)port;
    return 0;
}

void uart_set_rx_callback(UARTPort port, UARTRxCallback callback) {
    if (port < 6) rx_callbacks[port] = callback;
}

void uart_set_inverted(UARTPort port, bool inverted) {
    (void)port;
    (void)inverted;
}

void uart_simulate_rx(UARTPort port, uint8_t byte) {
    if (port < 6 && rx_callbacks[port]) {
        rx_callbacks[port](byte);
    }
}