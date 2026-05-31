/**
 * SAINT.OS Firmware - Pathfinder BMS transport (RP2040)
 *
 * Pico SDK UART adapter for shared/src/pathfinder_bms_driver.c. JBD
 * protocol is bidirectional, so this implements read in addition to
 * the SyRen-style write — non-blocking, drains whatever's in the
 * hardware FIFO on each call.
 */

#include "pathfinder_bms_transport.h"

#include "uart_pin_pairs.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef SIMULATION
#include "hardware/gpio.h"
#include "hardware/uart.h"
#endif

#define BMS_DEFAULT_TX_PIN 0
#define BMS_DEFAULT_RX_PIN 1

#ifndef SIMULATION
static uart_inst_t* g_uart = NULL;
#endif
static uint8_t  g_tx_pin   = BMS_DEFAULT_TX_PIN;
static uint8_t  g_rx_pin   = BMS_DEFAULT_RX_PIN;
static uint8_t  g_instance = 0xFF;
static uint16_t g_baud     = 0;
static bool     g_open     = false;

static bool uart_open(uint8_t tx_pin, uint8_t rx_pin, uint16_t baud)
{
    if (g_open && tx_pin == g_tx_pin && rx_pin == g_rx_pin && baud == g_baud) {
        return true;
    }

#ifndef SIMULATION
    if (tx_pin == 0 && rx_pin == 0) {
        tx_pin = BMS_DEFAULT_TX_PIN;
        rx_pin = BMS_DEFAULT_RX_PIN;
    }
    uint8_t inst;
    if (!uart_pin_pair_lookup(tx_pin, rx_pin, &inst)) {
        tx_pin = BMS_DEFAULT_TX_PIN;
        rx_pin = BMS_DEFAULT_RX_PIN;
        inst   = 0;
    }
    g_uart     = (inst == 0) ? uart0 : uart1;
    g_instance = inst;

    uart_init(g_uart, baud);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
#else
    g_instance = 0;
#endif

    g_tx_pin = tx_pin;
    g_rx_pin = rx_pin;
    g_baud   = baud;
    g_open   = true;
    return true;
}

static bool uart_is_open(void) { return g_open; }

static bool uart_write(const uint8_t* data, size_t len)
{
#ifndef SIMULATION
    if (!g_open || !g_uart) return false;
    uart_write_blocking(g_uart, data, len);
    return true;
#else
    (void)data; (void)len;
    return false;
#endif
}

static size_t uart_read(uint8_t* data, size_t max_len)
{
#ifndef SIMULATION
    if (!g_open || !g_uart) return 0;
    size_t n = 0;
    while (n < max_len && uart_is_readable(g_uart)) {
        data[n++] = (uint8_t)uart_getc(g_uart);
    }
    return n;
#else
    (void)data; (void)max_len;
    return 0;
#endif
}

static uint8_t uart_instance(void) { return g_instance; }

static const pathfinder_bms_transport_ops_t uart_ops = {
    .name              = "uart",
    .open              = uart_open,
    .is_open           = uart_is_open,
    .write             = uart_write,
    .read              = uart_read,
    .resolved_instance = uart_instance,
};

const pathfinder_bms_transport_ops_t* pathfinder_bms_get_transport(void)
{
    return &uart_ops;
}
