/**
 * SAINT.OS Firmware - SyRen transport (RP2040)
 *
 * Pico SDK UART adapter for the shared SyRen driver core
 * (shared/src/syren_driver.c). Opens uart0/uart1 based on the
 * configured TX pin pair (resolved via uart_pin_pair_lookup) and
 * writes packet bytes via uart_write_blocking.
 *
 * SyRen is write-only — no read op needed.
 */

#include "syren_transport.h"

#include "uart_pin_pairs.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef SIMULATION
#include "hardware/gpio.h"
#include "hardware/uart.h"
#endif

#define SYREN_DEFAULT_TX_PIN 4
#define SYREN_DEFAULT_RX_PIN 5

#ifndef SIMULATION
static uart_inst_t* g_uart = NULL;
#endif
static uint8_t g_tx_pin   = SYREN_DEFAULT_TX_PIN;
static uint8_t g_rx_pin   = SYREN_DEFAULT_RX_PIN;
static uint8_t g_instance = 0xFF;
static uint16_t g_baud    = 0;
static bool    g_open     = false;

static bool uart_open(uint8_t tx_pin, uint8_t rx_pin, uint16_t baud)
{
    /* Idempotent: same pins + baud already open → no-op. Re-opening
     * would briefly drive the pad low and could lose a packet mid-
     * flight. */
    if (g_open && tx_pin == g_tx_pin && rx_pin == g_rx_pin && baud == g_baud) {
        return true;
    }

#ifndef SIMULATION
    uint8_t inst;
    if (tx_pin == 0 && rx_pin == 0) {
        tx_pin = SYREN_DEFAULT_TX_PIN;
        rx_pin = SYREN_DEFAULT_RX_PIN;
    }
    if (!uart_pin_pair_lookup(tx_pin, rx_pin, &inst)) {
        tx_pin = SYREN_DEFAULT_TX_PIN;
        rx_pin = SYREN_DEFAULT_RX_PIN;
        inst   = 1;
    }
    g_uart     = (inst == 0) ? uart0 : uart1;
    g_instance = inst;

    uart_init(g_uart, baud);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
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

static uint8_t uart_instance(void) { return g_instance; }

static const syren_transport_ops_t uart_ops = {
    .name              = "uart",
    .open              = uart_open,
    .is_open           = uart_is_open,
    .write             = uart_write,
    .resolved_instance = uart_instance,
};

const syren_transport_ops_t* syren_get_transport(void)
{
    return &uart_ops;
}
