/**
 * SAINT.OS Firmware - Kangaroo transport adapter for Teensy 4.1
 *
 * Standard 8N1 HW UART via HardwareSerial, read-capable (the Kangaroo
 * replies to Get/Status requests). The pin pair resolves to one of
 * Serial1..7 through teensy_serial_from_instance + uart_pin_pair_lookup.
 * Mirrors tic_transport.cpp.
 */

#include "kangaroo_transport.h"
#include "uart_pin_pairs.h"
#include "uart_serial_lookup.h"

#include <Arduino.h>
#include "platform.h"

static HardwareSerial* s_port = nullptr;
static uint8_t  s_tx_pin = 0xFF;
static uint8_t  s_rx_pin = 0xFF;
static uint8_t  s_instance = 0xFF;
static uint32_t s_baud = 0;

extern "C" {

static bool kangaroo_teensy_open(uint8_t tx_pin, uint8_t rx_pin, uint32_t baud)
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

static bool kangaroo_teensy_is_open(void)
{
    return s_port != nullptr;
}

static bool kangaroo_teensy_write(const uint8_t* data, size_t len)
{
    if (!s_port) return false;
    size_t written = s_port->write(data, len);
    return written == len;
}

static size_t kangaroo_teensy_read(uint8_t* data, size_t max_len)
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

static uint8_t kangaroo_teensy_resolved_instance(void)
{
    return s_instance;
}

static const kangaroo_transport_ops_t s_ops = {
    /* .name              = */ "teensy-hwserial",
    /* .open              = */ kangaroo_teensy_open,
    /* .is_open           = */ kangaroo_teensy_is_open,
    /* .write             = */ kangaroo_teensy_write,
    /* .read              = */ kangaroo_teensy_read,
    /* .resolved_instance = */ kangaroo_teensy_resolved_instance,
};

const kangaroo_transport_ops_t* kangaroo_get_transport(void)
{
    return &s_ops;
}

}  /* extern "C" */
