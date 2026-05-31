/**
 * SAINT.OS Firmware - TMC2208 transport adapter for Teensy 4.1
 *
 * UART config goes through a HardwareSerial port. The TMC2208 needs
 * the line to be high in idle (both ends drive HIGH); half-duplex
 * echo-back is drained by xfer().
 *
 * Step generation: one IntervalTimer per axis. Each IntervalTimer
 * fires its own ISR at the configured step rate, toggling the STEP
 * pin and calling tmc2208_step_done(axis). When tmc2208_step_done
 * returns false (target reached), the ISR stops the timer.
 */

#include "tmc2208_transport.h"
#include "tmc2208_driver.h"  /* for tmc2208_step_done() */
#include "uart_pin_pairs.h"
#include "uart_serial_lookup.h"

#include <Arduino.h>

/* ── UART state ────────────────────────────────────────────────── */

static HardwareSerial* s_port = nullptr;
static uint8_t  s_tx_pin = 0xFF;
static uint8_t  s_rx_pin = 0xFF;
static uint8_t  s_instance = 0xFF;
static uint32_t s_baud = 0;

/* ── Step generation state ─────────────────────────────────────── */

struct axis_slot_t {
    IntervalTimer  timer;
    uint8_t        step_pin = 0xFF;
    uint8_t        dir_pin  = 0xFF;
    volatile bool  step_high = false;
    bool           active = false;
};

static axis_slot_t s_axis[4];

/* IntervalTimer ISRs can't take arguments, so we need per-axis
 * trampolines. Each one toggles its STEP pin (half-period semantics
 * — true HIGH duration is the same as the LOW duration) and calls
 * tmc2208_step_done on the falling edge. */
extern "C" {

static inline void step_isr_common(uint8_t axis)
{
    axis_slot_t* slot = &s_axis[axis];
    if (slot->step_high) {
        digitalWriteFast(slot->step_pin, LOW);
        slot->step_high = false;
        if (!tmc2208_step_done(axis)) {
            slot->timer.end();
            slot->active = false;
        }
    } else {
        digitalWriteFast(slot->step_pin, HIGH);
        slot->step_high = true;
    }
}

static void step_isr_0(void) { step_isr_common(0); }
static void step_isr_1(void) { step_isr_common(1); }
static void step_isr_2(void) { step_isr_common(2); }
static void step_isr_3(void) { step_isr_common(3); }

static void (* const k_isrs[4])(void) = {
    step_isr_0, step_isr_1, step_isr_2, step_isr_3,
};

/* ── UART ops ──────────────────────────────────────────────────── */

static bool tmc_teensy_open(uint8_t tx_pin, uint8_t rx_pin, uint32_t baud)
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

static bool tmc_teensy_is_open(void)
{
    return s_port != nullptr;
}

static uint8_t tmc_teensy_resolved_instance(void)
{
    return s_instance;
}

/* Drain echo bytes, then optionally read the reply. The TMC2208 in
 * half-duplex echoes our TX into RX, so we must consume those echo
 * bytes before they get mistaken for the reply. */
static size_t tmc_teensy_xfer(const uint8_t* tx, size_t tx_len,
                               uint8_t* rx_buf, size_t rx_len)
{
    if (!s_port) return 0;
    /* Flush any stale bytes in RX before TX, so the echo drain
     * doesn't accidentally consume them. */
    while (s_port->available()) (void)s_port->read();

    s_port->write(tx, tx_len);
    s_port->flush();

    /* Drain echo. Each byte ≈ 100 µs at 115200; use a generous
     * timeout per byte. */
    uint32_t per_byte_us = (12 * 1000000UL) / s_baud + 200;
    for (size_t i = 0; i < tx_len; i++) {
        uint32_t start = micros();
        while (!s_port->available()) {
            if ((uint32_t)(micros() - start) > per_byte_us) {
                /* Echo byte didn't arrive — likely wiring issue. */
                return 0;
            }
        }
        (void)s_port->read();
    }

    if (rx_len == 0 || rx_buf == nullptr) return 0;

    size_t got = 0;
    uint32_t start_ms = millis();
    while (got < rx_len) {
        if (s_port->available()) {
            rx_buf[got++] = (uint8_t)s_port->read();
            start_ms = millis();
            continue;
        }
        if ((uint32_t)(millis() - start_ms) > TMC2208_READ_TIMEOUT_MS) {
            return got;
        }
    }
    return got;
}

/* ── Step generation ops ───────────────────────────────────────── */

static bool tmc_teensy_axis_attach(uint8_t axis, uint8_t step_pin, uint8_t dir_pin)
{
    if (axis >= 4) return false;
    axis_slot_t* slot = &s_axis[axis];

    if (slot->step_pin == step_pin && slot->dir_pin == dir_pin && slot->active == false) {
        /* Already attached, currently idle — just confirm pin modes. */
        pinMode(step_pin, OUTPUT);
        pinMode(dir_pin, OUTPUT);
        digitalWriteFast(step_pin, LOW);
        return true;
    }
    if (slot->step_pin != 0xFF && slot->step_pin != step_pin) {
        /* Different pin — release old, then claim new. */
        slot->timer.end();
        pinMode(slot->step_pin, INPUT);
        if (slot->dir_pin != 0xFF) pinMode(slot->dir_pin, INPUT);
    }

    pinMode(step_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
    digitalWriteFast(step_pin, LOW);
    digitalWriteFast(dir_pin, LOW);

    slot->step_pin = step_pin;
    slot->dir_pin  = dir_pin;
    slot->step_high = false;
    slot->active = false;
    return true;
}

static void tmc_teensy_axis_detach(uint8_t axis)
{
    if (axis >= 4) return;
    axis_slot_t* slot = &s_axis[axis];
    slot->timer.end();
    if (slot->step_pin != 0xFF) pinMode(slot->step_pin, INPUT);
    if (slot->dir_pin  != 0xFF) pinMode(slot->dir_pin,  INPUT);
    slot->step_pin = 0xFF;
    slot->dir_pin  = 0xFF;
    slot->active = false;
}

static void tmc_teensy_axis_set_rate(uint8_t axis, uint32_t pps, bool forward)
{
    if (axis >= 4) return;
    axis_slot_t* slot = &s_axis[axis];
    if (slot->step_pin == 0xFF) return;

    digitalWriteFast(slot->dir_pin, forward ? HIGH : LOW);

    if (pps == 0) {
        slot->timer.end();
        slot->active = false;
        digitalWriteFast(slot->step_pin, LOW);
        slot->step_high = false;
        return;
    }

    /* IntervalTimer period in microseconds = half-period of the step
     * waveform (toggle ISR fires twice per step). 50% duty. */
    uint32_t period_us = 500000U / pps;
    if (period_us < 2) period_us = 2;  /* sanity floor */

    if (slot->active) {
        slot->timer.update(period_us);
    } else {
        slot->timer.begin(k_isrs[axis], period_us);
        slot->active = true;
    }
}

/* ── Ops table ─────────────────────────────────────────────────── */

static const tmc2208_transport_ops_t s_ops = {
    /* .name              = */ "teensy-hwserial-itimer",
    /* .open              = */ tmc_teensy_open,
    /* .is_open           = */ tmc_teensy_is_open,
    /* .resolved_instance = */ tmc_teensy_resolved_instance,
    /* .xfer              = */ tmc_teensy_xfer,
    /* .axis_attach       = */ tmc_teensy_axis_attach,
    /* .axis_detach       = */ tmc_teensy_axis_detach,
    /* .axis_set_rate     = */ tmc_teensy_axis_set_rate,
};

const tmc2208_transport_ops_t* tmc2208_get_transport(void)
{
    return &s_ops;
}

}  /* extern "C" */
