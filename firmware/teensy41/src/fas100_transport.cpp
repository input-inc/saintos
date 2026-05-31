/**
 * SAINT.OS Firmware - FAS100 transport adapter for Teensy 4.1
 *
 * Half-duplex inverted UART (TX → 1 kΩ → RX, sensor signal pin tied to
 * RX). The shared driver core switches baud via set_baud() during
 * S.Port/FBUS auto-detection.
 *
 * Teensy's HardwareSerial supports inverted signaling natively via the
 * SERIAL_8N1_RXINV_TXINV format flag — much simpler than RP2040's pad-
 * override dance. Each Serial<N> object is tied to a specific TX/RX pin
 * pair on the chip; we resolve the pair through the shared
 * uart_pin_pairs table (UART instance N → Serial<N>) and call its
 * begin() with the inverted format.
 */

#include "fas100_transport.h"
#include "uart_pin_pairs.h"

#include <Arduino.h>

static HardwareSerial* s_port = nullptr;
static uint8_t  s_tx_pin = 0xFF;
static uint8_t  s_rx_pin = 0xFF;
static uint8_t  s_instance = 0xFF;
static uint32_t s_baud = 0;
static bool     s_invert_active = false;

/* Map a UART instance number (1..7) to the matching Serial<N> object.
 * Pulled out so open() and set_baud() don't have to repeat the switch. */
static HardwareSerial* port_for_instance(uint8_t instance)
{
    switch (instance) {
        case 1: return &Serial1;
        case 2: return &Serial2;
        case 3: return &Serial3;
        case 4: return &Serial4;
        case 5: return &Serial5;
        case 6: return &Serial6;
        case 7: return &Serial7;
        default: return nullptr;
    }
}

extern "C" {

static bool fas100_teensy_open(uint8_t tx_pin, uint8_t rx_pin, uint32_t baud, bool invert)
{
    uint8_t inst;
    if (!uart_pin_pair_lookup(tx_pin, rx_pin, &inst)) return false;
    HardwareSerial* port = port_for_instance(inst);
    if (!port) return false;

    /* Same pair, baud, and inversion — no-op so we don't drop a byte in flight. */
    if (s_port == port && s_tx_pin == tx_pin && s_rx_pin == rx_pin
        && s_instance == inst && s_baud == baud
        && s_invert_active == invert) {
        return true;
    }

    /* Tear down the previous port if we're moving to a different one
     * (or to a different mode on the same port). HardwareSerial.end()
     * releases the pins and stops the peripheral. */
    if (s_port && s_port != port) {
        s_port->end();
    }

    uint32_t format = invert ? SERIAL_8N1_RXINV_TXINV : SERIAL_8N1;
    port->begin(baud, format);

    s_port = port;
    s_tx_pin = tx_pin;
    s_rx_pin = rx_pin;
    s_instance = inst;
    s_baud = baud;
    s_invert_active = invert;
    return true;
}

static void fas100_teensy_set_baud(uint32_t baud)
{
    if (!s_port) return;
    uint32_t format = s_invert_active ? SERIAL_8N1_RXINV_TXINV : SERIAL_8N1;
    /* HardwareSerial doesn't expose a baud-only setter; re-call begin
     * with the new baud. Drain any leftover bytes from the old rate so
     * a half-decoded frame doesn't poison the parser at the new rate. */
    s_port->begin(baud, format);
    s_baud = baud;
    while (s_port->available()) (void)s_port->read();
}

static bool fas100_teensy_is_open(void)
{
    return s_port != nullptr;
}

static bool fas100_teensy_write(const uint8_t* data, size_t len)
{
    if (!s_port) return false;
    size_t written = s_port->write(data, len);
    return written == len;
}

static void fas100_teensy_flush(void)
{
    if (!s_port) return;
    s_port->flush();
}

static size_t fas100_teensy_read(uint8_t* data, size_t max_len)
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

static uint8_t fas100_teensy_resolved_instance(void)
{
    return s_instance;
}

static const fas100_transport_ops_t s_ops = {
    /* .name              = */ "teensy-hwserial",
    /* .open              = */ fas100_teensy_open,
    /* .set_baud          = */ fas100_teensy_set_baud,
    /* .is_open           = */ fas100_teensy_is_open,
    /* .write             = */ fas100_teensy_write,
    /* .flush             = */ fas100_teensy_flush,
    /* .read              = */ fas100_teensy_read,
    /* .resolved_instance = */ fas100_teensy_resolved_instance,
};

const fas100_transport_ops_t* fas100_get_transport(void)
{
    return &s_ops;
}

}  /* extern "C" */
