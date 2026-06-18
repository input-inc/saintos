/**
 * SAINT.OS Firmware - Pololu Maestro-24 Servo Controller Driver (shared)
 *
 * The Pololu Maestro speaks the same compact protocol over USB CDC or
 * UART TTL. This header is the public API every platform's main.c
 * registers against. Transport selection (USB host vs UART) happens
 * at init time based on the saved flash config — see
 * shared/include/maestro_transport.h for the per-platform plug.
 *
 * Provides 24 servo channels mapped as virtual GPIO 200-223.
 */

#ifndef SAINT_MAESTRO_DRIVER_H
#define SAINT_MAESTRO_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ───────────────────────────────────────────────────── */

#define MAESTRO_VIRTUAL_GPIO_BASE   200
#define MAESTRO_MAX_CHANNELS        24

/* Default pulse widths (microseconds) */
#define MAESTRO_DEFAULT_MIN_PULSE   992
#define MAESTRO_DEFAULT_MAX_PULSE   2000
#define MAESTRO_DEFAULT_NEUTRAL     1500

/* Default UART baud rate when transport_mode == UART. The Maestro
 * auto-detects baud in compact-protocol mode, so any common rate works;
 * 9600 is the factory default and what the Pololu Control Center
 * defaults to when the device is reset. */
#define MAESTRO_DEFAULT_UART_BAUD   9600

/* ── Per-Channel Configuration ───────────────────────────────────── */

typedef struct {
    uint16_t min_pulse_us;      /* Minimum pulse width (us)             */
    uint16_t max_pulse_us;      /* Maximum pulse width (us)             */
    uint16_t neutral_us;        /* Neutral/center pulse width (us)      */
    uint16_t speed;             /* Speed limit (0 = unlimited)          */
    uint16_t acceleration;      /* Acceleration limit (0 = none)        */
    uint16_t home_us;           /* Home position pulse width (0=neutral)*/
} maestro_channel_config_t;

/* ── High-level lifecycle (called from peripheral framework) ─────── */

void maestro_init(void);
void maestro_update(void);
bool maestro_is_connected(void);
uint8_t maestro_get_channel_count(void);

/* ── Compact-protocol commands ───────────────────────────────────── */

bool     maestro_set_target(uint8_t channel, uint16_t quarter_us);
bool     maestro_set_speed(uint8_t channel, uint16_t speed);
bool     maestro_set_acceleration(uint8_t channel, uint16_t accel);
uint16_t maestro_get_position(uint8_t channel);
uint16_t maestro_get_errors(void);
void     maestro_go_home(void);

/* ── Channel config ──────────────────────────────────────────────── */

void maestro_set_channel_config(uint8_t channel,
                                 const maestro_channel_config_t* config);
const maestro_channel_config_t* maestro_get_channel_config(uint8_t channel);

/* Read a channel's config from the Maestro EEPROM via vendor
 * GET_PARAMETER (0x81). Populates `out` with min/max/neutral/speed/
 * accel/home sourced from the chip rather than from RAM.
 *
 * Only available when the active transport implements ctrl_xfer
 * (usb_vendor on the Teensy when task #17 lands; pyusb on the Pi
 * side). Returns false on UART / USB CDC transports — those carriers
 * cannot issue EP0 control transfers at all, and the Maestro's
 * Compact Protocol has no equivalent. Returns false on any vendor
 * read error (timeout, STALL, channel out of range).
 *
 * Used by the channel-edit UI: when the operator opens the channel
 * modal, the server-side defaults come from this readback if the
 * transport supports it, falling back to the peripheral-level
 * Advanced settings otherwise. See docs/MAESTRO_BRINGUP.md.
 *
 * Per-channel param IDs from Pololu's Usc_protocol.cs uscParameter
 * enum: SERVO0_HOME = 30, with 9-byte stride per channel
 * (HOME 2B, MIN 1B, MAX 1B, NEUTRAL 2B, RANGE 1B, SPEED 1B, ACCEL 1B).
 * MIN/MAX are stored × 1/64 of qus; we convert to µs in `out`. */
bool maestro_read_channel_config_from_device(uint8_t channel,
                                              maestro_channel_config_t* out);

/* Convert a 0-180° angle to a quarter-µs target for set_target,
 * applying the channel's calibrated min/max pulse range. */
uint16_t maestro_angle_to_target(float angle,
                                  const maestro_channel_config_t* config);

/* ── Registration with the peripheral manager ────────────────────── */

struct peripheral_driver;
const struct peripheral_driver* maestro_get_peripheral_driver(void);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_MAESTRO_DRIVER_H */
