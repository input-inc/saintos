/**
 * SAINT.OS Firmware - Kangaroo transport adapter for RP2040
 *
 * Standard 8N1 HW UART via Pico SDK, read-capable (the Kangaroo replies
 * to Get/Status requests). Mirrors tic_transport.c — plain-vanilla TTL
 * serial, RX pulled up against float during the controller's boot
 * window, pin tear-down on rebind.
 */

#include "kangaroo_transport.h"
#include "uart_pin_pairs.h"

#include "hardware/uart.h"
#include "hardware/gpio.h"

static uart_inst_t* s_uart = NULL;
static uint8_t      s_tx_pin = 0xFF;
static uint8_t      s_rx_pin = 0xFF;
static uint8_t      s_instance = 0xFF;
static uint32_t     s_baud = 0;

static bool kangaroo_rp2040_open(uint8_t tx_pin, uint8_t rx_pin, uint32_t baud)
{
    uint8_t inst;
    if (!uart_pin_pair_lookup(tx_pin, rx_pin, &inst)) return false;

    if (s_uart && s_tx_pin == tx_pin && s_rx_pin == rx_pin
        && s_instance == inst && s_baud == baud) {
        return true;
    }

    if (s_tx_pin != 0xFF) {
        gpio_set_function(s_tx_pin, GPIO_FUNC_SIO);
        gpio_set_dir(s_tx_pin, GPIO_IN);
    }
    if (s_rx_pin != 0xFF) {
        gpio_set_function(s_rx_pin, GPIO_FUNC_SIO);
        gpio_set_dir(s_rx_pin, GPIO_IN);
    }
    if (s_uart && s_instance != inst) {
        uart_deinit(s_uart);
    }

    s_uart = (inst == 0) ? uart0 : uart1;
    uart_init(s_uart, baud);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    gpio_pull_up(rx_pin);

    s_tx_pin = tx_pin;
    s_rx_pin = rx_pin;
    s_instance = inst;
    s_baud = baud;
    return true;
}

static bool kangaroo_rp2040_is_open(void)
{
    return s_uart != NULL;
}

static bool kangaroo_rp2040_write(const uint8_t* data, size_t len)
{
    if (!s_uart) return false;
    uart_write_blocking(s_uart, data, len);
    return true;
}

static size_t kangaroo_rp2040_read(uint8_t* data, size_t max_len)
{
    if (!s_uart || max_len == 0) return 0;
    size_t n = 0;
    while (n < max_len && uart_is_readable(s_uart)) {
        data[n++] = (uint8_t)uart_getc(s_uart);
    }
    return n;
}

static uint8_t kangaroo_rp2040_resolved_instance(void)
{
    return s_instance;
}

static const kangaroo_transport_ops_t s_ops = {
    .name              = "rp2040-uart",
    .open              = kangaroo_rp2040_open,
    .is_open           = kangaroo_rp2040_is_open,
    .write             = kangaroo_rp2040_write,
    .read              = kangaroo_rp2040_read,
    .resolved_instance = kangaroo_rp2040_resolved_instance,
};

const kangaroo_transport_ops_t* kangaroo_get_transport(void)
{
    return &s_ops;
}
