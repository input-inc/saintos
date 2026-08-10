/**
 * SAINT.OS Firmware - Pimoroni Servo 2040 driver core (shared)
 *
 * Host-side driver for the Pimoroni Servo 2040 servo-controller board.
 * Owns: per-channel servo extents, LED color/brightness state, current
 * telemetry cache, and the peripheral_driver_t glue. Speaks the ASCII
 * line protocol in pimoroni_servo2040_protocol.h to the board's fixed
 * firmware over a UART transport (pimoroni_servo2040_transport.h).
 *
 * Registered on every MCU controller (Teensy 4.1 + RP2040) exactly like
 * the Maestro driver. The board itself is never a SAINT.OS node — it's a
 * peripheral, hosted here.
 *
 * Channel map (virtual-GPIO slab, base PIMORONI_SERVO2040_VIRTUAL_GPIO_BASE):
 *   channels 0..17  — servos, set_value takes normalized −1..+1
 *   channels 18..23 — onboard RGB LEDs, set_value takes packed uint24 RGB
 * Telemetry ("connected", "current_a", "error_flags") is emitted by
 * channel_id string, not slab index.
 */

#ifndef SAINT_PIMORONI_SERVO2040_DRIVER_H
#define SAINT_PIMORONI_SERVO2040_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "pimoroni_servo2040_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Per-channel servo configuration ─────────────────────────────────
 * The four-tuple extent model shared with the native servo path
 * (pin_types.h servo struct + the dashboard's ServoExtentsControl):
 * −1 → start_us, 0 → center_us, +1 → end_us, home_us on connect/reset. */
typedef struct {
    uint16_t start_us;   /* pulse at normalized input −1 */
    uint16_t end_us;     /* pulse at normalized input +1 */
    uint16_t center_us;  /* pulse at normalized input  0 */
    uint16_t home_us;    /* startup / safe-reset pulse; 0 = leave relaxed */
} pimoroni_servo2040_channel_config_t;

/* ── Lifecycle (called from the peripheral framework) ─────────────── */
void    pimoroni_servo2040_init(void);
void    pimoroni_servo2040_update(void);
bool    pimoroni_servo2040_is_connected(void);

/* ── Telemetry accessors (populated from the board's 'T' line) ────── */
float   pimoroni_servo2040_get_current_amps(void);
uint8_t pimoroni_servo2040_get_flags(void);

/* ── Registration with the peripheral manager ─────────────────────── */
struct peripheral_driver;
const struct peripheral_driver* pimoroni_servo2040_get_peripheral_driver(void);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_PIMORONI_SERVO2040_DRIVER_H */
