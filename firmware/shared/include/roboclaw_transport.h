/**
 * SAINT.OS Firmware - RoboClaw transport ops (shared)
 *
 * Standard 8N1 packet serial. Platforms differ in how they express
 * "open this UART at this baud" and how they pump bytes:
 *
 *   - RP2040: hardware UART (uart_init + gpio_set_function) or PIO
 *             UART when the PCB has SWAPPED TX/RX routing — the
 *             pio_swap flag picks between them. The PIO path lets
 *             any GPIO pair drive a UART.
 *   - Teensy: HardwareSerial.begin(baud). pio_swap is ignored (the
 *             chip's UART matrix handles any of the documented pairs
 *             natively).
 *
 * The shared driver core (shared/src/roboclaw_driver.c) routes every
 * wire-level read/write through this ops table. RP2040-specific
 * concerns (PIO vs HW, GPIO function clobber detection) live entirely
 * inside firmware/rp2040/src/roboclaw_transport.c.
 */

#ifndef SAINT_ROBOCLAW_TRANSPORT_H
#define SAINT_ROBOCLAW_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct roboclaw_transport_ops {
    const char* name;

    /* Open the UART bound to the requested pin pair at `baud`. If
     * `pio_swap` is true and the platform supports it, use a PIO-
     * based UART so non-default GPIOs can drive TX/RX (used when
     * the PCB routes TX→TX and RX→RX rather than crossed). Platforms
     * that don't support PIO ignore the flag and always use HW serial.
     *
     * Idempotent: opening with the same params as the current
     * binding is a no-op. Different params tear down the prior
     * binding first (so the old pins stop driving the bus). */
    bool (*open)(uint8_t tx_pin, uint8_t rx_pin, uint32_t baud, bool pio_swap);

    /* Returns true if the transport is currently bound AND its pads
     * still match what the open() bound them to. RP2040 verifies the
     * actual gpio_get_function value (defense against external pad
     * clobbering during boot). Teensy returns true if the Serial<N>
     * pointer is set. The shared driver's idempotent fast-path gates
     * on this — if it returns false, a re-bind is forced. */
    bool (*verify_binding)(void);

    bool (*is_open)(void);

    bool (*write)(const uint8_t* data, size_t len);
    size_t (*read)(uint8_t* data, size_t max_len);

    /* After a successful open, returns true if the PIO path is
     * actually active. Differs from the open() argument iff PIO
     * was requested but the transport fell back to HW (e.g. PIO
     * state-machine allocation failed on RP2040). Teensy always
     * returns false. */
    bool (*pio_swap_active)(void);

    uint8_t (*resolved_instance)(void);
} roboclaw_transport_ops_t;

const roboclaw_transport_ops_t* roboclaw_get_transport(void);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_ROBOCLAW_TRANSPORT_H */
