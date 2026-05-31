/**
 * SAINT.OS Firmware - RoboClaw transport adapter for Teensy 4.1
 *
 * Standard 8N1 packet serial over a HardwareSerial port. The pin pair
 * resolves to one of Serial1-Serial8 via teensy_serial_from_instance +
 * uart_pin_pair_lookup. pio_swap is ignored — the chip's UART matrix
 * supports any documented pin pair natively, so there's no need for
 * a PIO fallback.
 *
 * E-stop GPIO output uses pinMode/digitalWrite. Teensy's GPIO range
 * is the same 0..29 numbering convention as RP2040 for the on-board
 * pins; the shared driver clamps `pin` to that range before calling
 * us.
 */

#include "roboclaw_transport.h"
#include "uart_pin_pairs.h"
#include "uart_serial_lookup.h"

#include <Arduino.h>

static HardwareSerial* s_port = nullptr;
static uint8_t  s_tx_pin = 0xFF;
static uint8_t  s_rx_pin = 0xFF;
static uint8_t  s_instance = 0xFF;
static uint32_t s_baud = 0;

extern "C" {

static bool roboclaw_teensy_open(uint8_t tx_pin, uint8_t rx_pin,
                                  uint32_t baud, bool pio_swap)
{
    (void)pio_swap;  /* Teensy uses the chip UART matrix; no PIO path. */
    uint8_t inst;
    if (!uart_pin_pair_lookup(tx_pin, rx_pin, &inst)) return false;
    HardwareSerial* port = teensy_serial_from_instance(inst);
    if (!port) return false;

    /* Idempotent fast-path. */
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

static bool roboclaw_teensy_verify_binding(void)
{
    /* No equivalent of RP2040's GPIO function clobber on Teensy — the
     * Serial<N> object owns its pins exclusively. Bound iff the pointer
     * is set. */
    return s_port != nullptr;
}

static bool roboclaw_teensy_is_open(void)
{
    return s_port != nullptr;
}

static bool roboclaw_teensy_write(const uint8_t* data, size_t len)
{
    if (!s_port) return false;
    size_t written = s_port->write(data, len);
    return written == len;
}

static size_t roboclaw_teensy_read(uint8_t* data, size_t max_len)
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

static bool roboclaw_teensy_pio_swap_active(void)
{
    return false;  /* never on Teensy */
}

static uint8_t roboclaw_teensy_resolved_instance(void)
{
    return s_instance;
}

static const roboclaw_transport_ops_t s_ops = {
    /* .name              = */ "teensy-hwserial",
    /* .open              = */ roboclaw_teensy_open,
    /* .verify_binding    = */ roboclaw_teensy_verify_binding,
    /* .is_open           = */ roboclaw_teensy_is_open,
    /* .write             = */ roboclaw_teensy_write,
    /* .read              = */ roboclaw_teensy_read,
    /* .pio_swap_active   = */ roboclaw_teensy_pio_swap_active,
    /* .resolved_instance = */ roboclaw_teensy_resolved_instance,
};

const roboclaw_transport_ops_t* roboclaw_get_transport(void)
{
    return &s_ops;
}

/* ── E-stop GPIO helpers ────────────────────────────────────────── */

void roboclaw_estop_apply_pin(uint8_t pin)
{
    if (pin == 0 || pin > 29) return;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);   /* LOW = deasserted */
}

void roboclaw_estop_assert_pin(uint8_t pin)
{
    if (pin == 0 || pin > 29) return;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);  /* HIGH = asserted = motor disabled */
}

}  /* extern "C" */
