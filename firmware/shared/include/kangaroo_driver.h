/**
 * SAINT.OS Firmware - Dimension Engineering Kangaroo X2 driver (shared)
 *
 * Drives up to 8 Kangaroo motor channels over packet OR simplified
 * serial (selected per channel by the operator). Closed-loop: each
 * channel takes position / speed setpoints and reports back current
 * position, current speed, a "moving" (busy) flag and the last error.
 *
 * Platform-agnostic: protocol byte assembly lives in
 * shared/include/kangaroo_protocol.h, channel state + the
 * peripheral_driver_t glue in shared/src/kangaroo_driver.c, and each
 * platform's read-capable UART adapter implements
 * kangaroo_transport_ops (shared/include/kangaroo_transport.h).
 * Mirrors the Tic driver's architecture.
 */

#ifndef SAINT_KANGAROO_DRIVER_H
#define SAINT_KANGAROO_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "kangaroo_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Per-Channel Configuration ──────────────────────────────────── */

typedef struct {
    uint8_t  address;       /* 128-135                                  */
    uint8_t  channel_name;  /* '1'/'2' (independent) or 'D'/'T' (mixed) */
    uint8_t  protocol;      /* KANGAROO_PROTO_PACKET | _SIMPLE          */
    uint8_t  home_on_start; /* 1 = send Home after Start                */
    int32_t  max_position;  /* operator scaling for target_position     */
    int32_t  max_speed;     /* operator scaling for target_speed (units/s) */
} kangaroo_channel_config_t;

/* ── Driver API ─────────────────────────────────────────────────── */

void     kangaroo_init(void);
void     kangaroo_update(void);
bool     kangaroo_is_connected(void);

/* Scaled motion entry points (unit = channel index 0..7). value is the
 * already-scaled machine value: position in units, speed in units/s. */
bool     kangaroo_set_position(uint8_t unit, int32_t position);
bool     kangaroo_set_speed(uint8_t unit, int32_t speed);
bool     kangaroo_powerdown(uint8_t unit);
void     kangaroo_powerdown_all(void);

/* ── Teach tune (Mode 1) ────────────────────────────────────────── */
/*
 * Drives a Kangaroo teach tune over packet serial, so an operator can
 * set an axis's travel range from the dashboard instead of standing at
 * the board with the Autotune button and a transfer switch.
 *
 * Sequence: enter() → jog() repeatedly to the retract end, the extend
 * end, then back to centre → go(). The Kangaroo then runs its own tune
 * cycle (minutes, on a slow linear actuator) and the axis MUST be power
 * cycled before the new tune takes effect.
 *
 * The firmware — not the caller — owns the three safety properties,
 * because none of them can be guaranteed across a network:
 *
 *   - keep-alive: tuning has an automatic serial timeout and aborts if
 *     packets stop, so kangaroo_update() holds a Get loop for the whole
 *     tune.
 *   - dead-man: jog power decays to zero if kangaroo_tune_jog() is not
 *     refreshed within KANGAROO_JOG_DEADMAN_MS. A dropped link stops the
 *     actuator instead of pinning it against a hard stop.
 *   - power cap: jog is open loop — no feedback, no travel limits, no
 *     protection at all — so magnitude is clamped to the unit's
 *     configured fraction of full scale.
 *
 * Packet protocol only. Simplified serial has no equivalent and the
 * calls are rejected with a log rather than silently doing nothing.
 */

typedef enum {
    KANGAROO_TUNE_IDLE = 0,  /* not tuning — normal motion path active   */
    KANGAROO_TUNE_ENTERING,  /* Enter Mode sent, clearing safety interlock */
    KANGAROO_TUNE_JOG,       /* operator positioning the axis open loop  */
    KANGAROO_TUNE_GOING,     /* Kangaroo running its own tune cycle      */
    KANGAROO_TUNE_DONE,      /* completed — power cycle required         */
    KANGAROO_TUNE_FAILED,    /* aborted, errored, or timed out           */
} kangaroo_tune_state_t;

/* Enter Mode 1 (Teach) and clear the post-entry safety interlock. */
bool kangaroo_tune_enter(uint8_t unit);

/* Open-loop jog. `fraction` is -1..1 of the unit's configured power cap;
 * 0 stops. Must be called at least every KANGAROO_JOG_DEADMAN_MS or the
 * dead-man zeroes it. */
bool kangaroo_tune_jog(uint8_t unit, float fraction);

/* Begin the tune cycle. The axis starts moving on its own. */
bool kangaroo_tune_go(uint8_t unit);

/* Abort a tune at any stage. Also the software e-stop while tuning. */
bool kangaroo_tune_abort(uint8_t unit);

/* Read the taught travel limits (Get 8/9). These are read-only on the
 * wire — there is no command to set them; they come from where the axis
 * was jogged during the teach. Valid only after a post-tune power cycle. */
bool kangaroo_tune_read_extents(uint8_t unit, int32_t* out_min, int32_t* out_max);

kangaroo_tune_state_t kangaroo_tune_get_state(uint8_t unit);

/* ── Registration with the peripheral manager ───────────────────── */

struct peripheral_driver;
const struct peripheral_driver* kangaroo_get_peripheral_driver(void);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_KANGAROO_DRIVER_H */
