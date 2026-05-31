/**
 * SAINT.OS Firmware - Pathfinder BMS transport (Teensy 4.1)
 *
 * HardwareSerial adapter for shared/src/pathfinder_bms_driver.c.
 * Resolves the configured TX/RX pin pair to Serial1..Serial8 via
 * teensy_serial_from_instance + uart_pin_pair_lookup, opens at the
 * requested baud, and pumps bytes through write/read.
 */

#include <Arduino.h>

extern "C" {
#include "pathfinder_bms_transport.h"
#include "uart_pin_pairs.h"
}
#include "uart_serial_lookup.h"

#define BMS_DEFAULT_TX_PIN  17
#define BMS_DEFAULT_RX_PIN  16
#define BMS_DEFAULT_INST    4   /* Serial4 on Teensy 4.1 */

static HardwareSerial* g_serial   = &Serial4;
static uint8_t         g_tx_pin   = BMS_DEFAULT_TX_PIN;
static uint8_t         g_rx_pin   = BMS_DEFAULT_RX_PIN;
static uint8_t         g_instance = 0xFF;
static uint16_t        g_baud     = 0;
static bool            g_open     = false;

static bool uart_open(uint8_t tx_pin, uint8_t rx_pin, uint16_t baud)
{
    if (g_open && tx_pin == g_tx_pin && rx_pin == g_rx_pin && baud == g_baud) {
        return true;
    }

    if (tx_pin == 0 && rx_pin == 0) {
        tx_pin = BMS_DEFAULT_TX_PIN;
        rx_pin = BMS_DEFAULT_RX_PIN;
    }
    uint8_t inst;
    if (!uart_pin_pair_lookup(tx_pin, rx_pin, &inst)) {
        tx_pin = BMS_DEFAULT_TX_PIN;
        rx_pin = BMS_DEFAULT_RX_PIN;
        inst   = BMS_DEFAULT_INST;
    }
    HardwareSerial* s = teensy_serial_from_instance(inst);
    if (s) g_serial = s;

    g_serial->begin(baud);

    g_tx_pin   = tx_pin;
    g_rx_pin   = rx_pin;
    g_baud     = baud;
    g_instance = inst;
    g_open     = true;
    return true;
}

static bool uart_is_open(void) { return g_open; }

static bool uart_write(const uint8_t* data, size_t len)
{
    if (!g_open || !g_serial) return false;
    size_t written = g_serial->write(data, len);
    return written == len;
}

static size_t uart_read(uint8_t* data, size_t max_len)
{
    if (!g_open || !g_serial) return 0;
    size_t n = 0;
    while (n < max_len && g_serial->available()) {
        data[n++] = (uint8_t)g_serial->read();
    }
    return n;
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

extern "C" const pathfinder_bms_transport_ops_t* pathfinder_bms_get_transport(void)
{
    return &uart_ops;
}
