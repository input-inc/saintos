/**
 * SAINT.OS Firmware - Tic transport adapter for Teensy 4.1
 *
 * Standard 8N1 HW UART via HardwareSerial. The pin pair resolves to
 * one of Serial1..7 through teensy_serial_from_instance +
 * uart_pin_pair_lookup.
 */

#include "tic_transport.h"
#include "uart_pin_pairs.h"
#include "uart_serial_lookup.h"

#include <Arduino.h>

static HardwareSerial* s_port = nullptr;
static uint8_t  s_tx_pin = 0xFF;
static uint8_t  s_rx_pin = 0xFF;
static uint8_t  s_instance = 0xFF;
static uint32_t s_baud = 0;

extern "C" {

static bool tic_teensy_open(uint8_t tx_pin, uint8_t rx_pin, uint32_t baud)
{
    uint8_t inst;
    if (!uart_pin_pair_lookup(tx_pin, rx_pin, &inst)) return false;
    HardwareSerial* port = teensy_serial_from_instance(inst);
    if (!port) return false;

    if (s_port == port && s_tx_pin == tx_pin && s_rx_pin == rx_pin
        && s_instance == inst && s_baud == baud) {
        return true;
    }

    if (s_port && s_port != port) {
        s_port->end();
    }
    port->begin(baud);

    s_port = port;
    s_tx_pin = tx_pin;
    s_rx_pin = rx_pin;
    s_instance = inst;
    s_baud = baud;
    return true;
}

static bool tic_teensy_is_open(void)
{
    return s_port != nullptr;
}

static bool tic_teensy_write(const uint8_t* data, size_t len)
{
    if (!s_port) return false;
    size_t written = s_port->write(data, len);
    return written == len;
}

static size_t tic_teensy_read(uint8_t* data, size_t max_len)
{
    if (!s_port || max_len == 0) return 0;
    size_t n = 0;
    while (n < max_len) {
        int b = s_port->read();
        if (b < 0) break;
        data[n++] = (uint8_t)b;
    }
    return n;
}

static uint8_t tic_teensy_resolved_instance(void)
{
    return s_instance;
}

static const tic_transport_ops_t s_ops = {
    /* .name              = */ "teensy-hwserial",
    /* .open              = */ tic_teensy_open,
    /* .is_open           = */ tic_teensy_is_open,
    /* .write             = */ tic_teensy_write,
    /* .read              = */ tic_teensy_read,
    /* .resolved_instance = */ tic_teensy_resolved_instance,
};

const tic_transport_ops_t* tic_get_transport(void)
{
    return &s_ops;
}

}  /* extern "C" */
