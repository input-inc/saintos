/**
 * SAINT.OS Firmware - TMC2208 transport ops (shared)
 *
 * Two responsibilities lumped into one ops table because they share
 * pin assignments and lifecycle:
 *
 *   1. UART register access for chip configuration:
 *      - open(tx, rx, baud) — bind one UART pair shared by all axes
 *      - write_frame / read_frame — half-duplex with echo drain
 *
 *   2. Step pulse generation (platform-specific):
 *      - axis_attach(axis, step_pin, dir_pin) — claim two GPIOs
 *      - axis_set_rate(axis, pulses_per_sec, direction) — start/stop
 *      - axis_detach(axis) — release GPIOs
 *
 *      RP2040 uses a hardware timer + Bresenham-style multiplexed
 *      pulse output across axes (one ISR for all). Teensy uses one
 *      IntervalTimer per axis.
 *
 * The shared driver calls into the transport for both; per-platform
 * implementations decide how to satisfy them.
 */

#ifndef SAINT_TMC2208_TRANSPORT_H
#define SAINT_TMC2208_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct tmc2208_transport_ops {
    const char* name;

    /* ── UART config interface ─────────────────────────────────── */

    bool (*open)(uint8_t tx_pin, uint8_t rx_pin, uint32_t baud);
    bool (*is_open)(void);
    uint8_t (*resolved_instance)(void);

    /* Send `tx_len` bytes, drain `tx_len` echo bytes from RX, then
     * (if rx_len > 0) wait for and return up to `rx_len` reply bytes.
     * Returns the actual number of bytes read into `rx_buf`. Implements
     * per-byte timeout (TMC2208_READ_TIMEOUT_MS). */
    size_t (*xfer)(const uint8_t* tx, size_t tx_len,
                   uint8_t* rx_buf, size_t rx_len);

    /* ── Step generation interface ─────────────────────────────── */

    /* Attach the (step_pin, dir_pin) GPIO pair to `axis`. Returns
     * false if step generation infrastructure is unavailable
     * (e.g. PIO state machines exhausted on RP2040). Idempotent on
     * the same (axis, step_pin, dir_pin) combination. */
    bool (*axis_attach)(uint8_t axis, uint8_t step_pin, uint8_t dir_pin);

    /* Release a previously-attached axis. Stops pulse output and
     * releases the GPIOs back to inputs. */
    void (*axis_detach)(uint8_t axis);

    /* Set the pulse rate and direction. pulses_per_sec=0 stops the
     * axis (timer entry disabled). `forward` is true for the
     * direction the operator chose as positive. */
    void (*axis_set_rate)(uint8_t axis, uint32_t pulses_per_sec, bool forward);

} tmc2208_transport_ops_t;

const tmc2208_transport_ops_t* tmc2208_get_transport(void);

#define TMC2208_READ_TIMEOUT_MS  10

#ifdef __cplusplus
}
#endif

#endif /* SAINT_TMC2208_TRANSPORT_H */
