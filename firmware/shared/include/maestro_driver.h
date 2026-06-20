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

/* Hard absolute safety window for maestro_set_target_preview — wider
 * than a typical servo's calibrated range (so dial-in can reach the
 * real limits) but tight enough to protect against a runaway value. */
#define MAESTRO_PREVIEW_MIN_US      400
#define MAESTRO_PREVIEW_MAX_US      2600

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
    /* Idle disengage: after this many ms of the channel's target not
     * changing, the driver writes SET_TARGET=0 to release PWM so the
     * servo stops the idle-correction whine. The next control value
     * restores PWM. 0 = always engaged (no auto-release). Server caps
     * at 600_000 (10 min). uint32 because uint16 maxes at ~65 s — fine
     * for the eye/mouth case but tight if an operator wants "release
     * after 2 minutes of idle". See peripheral_model._MAESTRO_CHANNEL_KEYS. */
    uint32_t idle_disengage_ms;
} maestro_channel_config_t;

/* ── High-level lifecycle (called from peripheral framework) ─────── */

void maestro_init(void);
void maestro_update(void);
bool maestro_is_connected(void);
uint8_t maestro_get_channel_count(void);

/* ── Compact-protocol commands ───────────────────────────────────── */

bool     maestro_set_target(uint8_t channel, uint16_t quarter_us);

/* Drive a channel to an absolute pulse width (microseconds) for the
 * dashboard's live extent-dial preview. Unlike maestro_set_target this
 * does NOT apply the per-channel software min/max clamp — the operator
 * is deliberately exploring pulses outside the (not-yet-saved)
 * configured range to find the servo's mechanical limits. It clamps
 * only to a hard absolute safety window (MAESTRO_PREVIEW_MIN/MAX_US) so
 * a fat-fingered value can't peg a servo far past any plausible travel.
 * Not persisted and not the runtime control path — purely a "go here so
 * I can see it" jog. */
bool     maestro_set_target_preview(uint8_t channel, uint16_t us);
bool     maestro_set_speed(uint8_t channel, uint16_t speed);
bool     maestro_set_acceleration(uint8_t channel, uint16_t accel);
uint16_t maestro_get_position(uint8_t channel);
uint16_t maestro_get_errors(void);
/* Compact Protocol "Get Moving State" (0xA6). Returns 0 if all servos
 * are at their target, 1 if any are still moving. 0xFF means the
 * device didn't answer (transport failure). Mini Maestro 1.01+. */
uint8_t  maestro_get_moving_state(void);
void     maestro_go_home(void);

/* Stop any currently running user script on the Maestro. Pololu
 * Maestros sometimes auto-start a default script on power-on (the
 * 2-blink amber Status LED is the Pololu-documented indicator); a
 * running script overrides channel targets, so our SET_TARGET
 * commands have no visible effect until the script is stopped.
 * Always called on driver connect — see maestro_apply_home_positions
 * in shared/src/maestro_driver.c. Implemented as the single-byte
 * Compact Protocol opcode 0xA4 over the active transport — works
 * over USB CDC, UART TTL, and (when task #17 lands) USB vendor. */
bool     maestro_stop_script(void);

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

/* Provision a channel's Maestro EEPROM to match its SaintOS config via
 * vendor SET_PARAMETER (0x82): channel Mode = Servo, MIN/MAX, NEUTRAL,
 * and HomeMode = Goto at home_us. This is the one-time chip setup that
 * used to require Maestro Control Center — Pololu's MCC writes every
 * setting through this same vendor request with no separate EEPROM
 * commit step (verified against pololu-usb-sdk Usc.cs
 * setRawParameterNoChecks), so the parameters ARE the persistent
 * config. The byte count is encoded in the high byte of wIndex exactly
 * as MCC does.
 *
 * Only functional on a transport that implements ctrl_xfer (usb_vendor
 * on Teensy; pyusb on the Pi). Returns the number of EEPROM parameters
 * actually written (0 if the chip already matches — the writes are
 * diff-checked to avoid EEPROM wear), or < 0 if the transport can't do
 * control transfers or isn't connected.
 *
 * maestro_provision_all_channels() provisions every channel whose
 * home_us > 0 (an explicit "enable at startup" intent; channels left
 * at home_us == 0 are not touched), then issues REINITIALIZE so the
 * init parameters (channel mode, HOME) take effect without a power
 * cycle. See docs/MAESTRO_BRINGUP.md. */
int maestro_provision_channel(uint8_t channel);
int maestro_provision_all_channels(void);

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
