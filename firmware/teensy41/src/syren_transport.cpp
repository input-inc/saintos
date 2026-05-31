/**
 * SAINT.OS Firmware - SyRen transport (Teensy 4.1)
 *
 * HardwareSerial adapter for the shared SyRen driver core
 * (shared/src/syren_driver.c). Resolves the configured TX/RX pin pair
 * to one of Serial1-Serial8 via teensy_serial_from_instance + the
 * uart_pin_pair_lookup table, then opens it at the requested baud and
 * writes packet bytes verbatim.
 */

#include <Arduino.h>

extern "C" {
#include "syren_transport.h"
#include "uart_pin_pairs.h"
}
#include "uart_serial_lookup.h"

#define SYREN_DEFAULT_TX_PIN  8
#define SYREN_DEFAULT_RX_PIN  7
#define SYREN_DEFAULT_INST    2  /* Serial2 on Teensy 4.1 */

static HardwareSerial* g_serial   = &Serial2;
static uint8_t         g_tx_pin   = SYREN_DEFAULT_TX_PIN;
static uint8_t         g_rx_pin   = SYREN_DEFAULT_RX_PIN;
static uint8_t         g_instance = 0xFF;
static uint16_t        g_baud     = 0;
static bool            g_open     = false;

static bool uart_open(uint8_t tx_pin, uint8_t rx_pin, uint16_t baud)
{
    if (g_open && tx_pin == g_tx_pin && rx_pin == g_rx_pin && baud == g_baud) {
        return true;
    }

    if (tx_pin == 0 && rx_pin == 0) {
        tx_pin = SYREN_DEFAULT_TX_PIN;
        rx_pin = SYREN_DEFAULT_RX_PIN;
    }
    uint8_t inst;
    if (!uart_pin_pair_lookup(tx_pin, rx_pin, &inst)) {
        tx_pin = SYREN_DEFAULT_TX_PIN;
        rx_pin = SYREN_DEFAULT_RX_PIN;
        inst   = SYREN_DEFAULT_INST;
    }
    HardwareSerial* s = teensy_serial_from_instance(inst);
    if (s) g_serial = s;

    g_serial->begin(baud);
    delay(50);

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

static uint8_t uart_instance(void) { return g_instance; }

static const syren_transport_ops_t uart_ops = {
    .name              = "uart",
    .open              = uart_open,
    .is_open           = uart_is_open,
    .write             = uart_write,
    .resolved_instance = uart_instance,
};

extern "C" const syren_transport_ops_t* syren_get_transport(void)
{
    return &uart_ops;
}
