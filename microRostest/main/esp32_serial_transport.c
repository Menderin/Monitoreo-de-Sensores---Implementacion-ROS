#include "esp32_serial_transport.h"

#include <uxr/client/transport.h>

#include <driver/uart.h>

#include <freertos/FreeRTOS.h>

#define UART_BUFFER_SIZE (512)

bool esp32_serial_open(struct uxrCustomTransport * transport)
{
    size_t * uart_port = (size_t *)transport->args;
    const uart_port_t uart_num = (uart_port_t)(*uart_port);

    // Configure UART (pins left as-is; especially important for UART0 default pins).
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    (void)uart_param_config(uart_num, &uart_config);
    (void)uart_set_pin(uart_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // If the console already installed the UART driver (common on UART0), don't touch it.
    if (!uart_is_driver_installed(uart_num)) {
        if (uart_driver_install(uart_num, UART_BUFFER_SIZE * 2, UART_BUFFER_SIZE * 2, 0, NULL, 0) != ESP_OK) {
            return false;
        }
    }

    return true;
}

bool esp32_serial_close(struct uxrCustomTransport * transport)
{
    // Avoid deleting UART driver (could be owned by console). In this app we never close anyway.
    (void)transport;
    return true;
}

size_t esp32_serial_write(struct uxrCustomTransport * transport, const uint8_t * buf, size_t len, uint8_t * err)
{
    (void)err;
    size_t * uart_port = (size_t *)transport->args;
    const uart_port_t uart_num = (uart_port_t)(*uart_port);

    const int tx_bytes = uart_write_bytes(uart_num, (const char *)buf, len);
    return (tx_bytes > 0) ? (size_t)tx_bytes : 0;
}

size_t esp32_serial_read(struct uxrCustomTransport * transport, uint8_t * buf, size_t len, int timeout, uint8_t * err)
{
    (void)err;
    size_t * uart_port = (size_t *)transport->args;
    const uart_port_t uart_num = (uart_port_t)(*uart_port);

    const TickType_t timeout_ticks = (timeout <= 0) ? 0 : (TickType_t)(timeout / portTICK_PERIOD_MS);
    const int rx_bytes = uart_read_bytes(uart_num, buf, len, timeout_ticks);
    return (rx_bytes > 0) ? (size_t)rx_bytes : 0;
}
