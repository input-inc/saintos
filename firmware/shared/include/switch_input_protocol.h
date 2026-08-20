/**
 * SAINT.OS Firmware - Generic switch / sensor input constants
 *
 * A limit switch, end-stop, e-stop button, or any other on/off sensor,
 * as a peripheral in its own right rather than a private field on
 * whichever driver happens to care. See docs/SENSOR_INPUTS.md.
 */

#ifndef SWITCH_INPUT_PROTOCOL_H
#define SWITCH_INPUT_PROTOCOL_H

#include <stdint.h>

/* ── Sense mode ─────────────────────────────────────────────────── */
/*
 * Analog is not a convenience. A 2-wire sensor with a series voltage
 * drop cannot be read on a digital pin at all: the IDC PSR-2's
 * anti-parallel diode pair drops ~1.9 V, which lands between the 3.3 V
 * logic thresholds (valid low < 1.16 V, valid high > 2.15 V on RP2040),
 * so the pin reads high in both states. No pull-up value fixes it — the
 * level is set by the diode, not the resistor.
 */
#define SWITCH_SENSE_DIGITAL   0
#define SWITCH_SENSE_ANALOG    1

/* ── Trip action (local interlock, Tier 1) ──────────────────────── */
/*
 * Resolved on the node with no server in the path. A switch that stops
 * an actuator cannot depend on a round trip: node → server → routing
 * evaluator → node is tens of milliseconds at best and never completes
 * when the link is down, which is exactly when a runaway happens.
 *
 * Everything non-protective belongs in the routing graph instead.
 */
#define SWITCH_TRIP_NONE          0   /* report only; routing does the rest */
#define SWITCH_TRIP_STOP_TARGETS  1   /* estop the listed peripherals       */
#define SWITCH_TRIP_ESTOP_NODE    2   /* estop every peripheral on the node */

/* ── Virtual GPIO map ───────────────────────────────────────────── */

#define SWITCH_INPUT_VIRTUAL_GPIO_BASE  444  /* first free after Kangaroo (364..443) */
#define SWITCH_INPUT_MAX_UNITS          8
#define SWITCH_INPUT_CHANNELS_PER_UNIT  4
#define SWITCH_INPUT_MAX_CHANNELS \
    (SWITCH_INPUT_MAX_UNITS * SWITCH_INPUT_CHANNELS_PER_UNIT)

/* Sub-channel indices within each unit. Keep in lock-step with the
 * catalog entry in peripheral_model.py AND with the stride in
 * state_manager._FIRMWARE_CHANNEL_MAP — those are three declarations of
 * one contract, and a mismatch silently mis-addresses units. */
#define SWITCH_SUB_STATE       0  /* read, 1 = asserted (debounced)       */
#define SWITCH_SUB_LATCHED     1  /* read, 1 = latched until cleared      */
#define SWITCH_SUB_VOLTAGE     2  /* read, volts (analog sense only)      */
#define SWITCH_SUB_TRIP_COUNT  3  /* read, asserts since boot             */

/* ── Blocked direction, per target ──────────────────────────────── */
/*
 * An end-of-travel switch should not simply freeze the axis — it should
 * stop motion INTO itself while still allowing the axis to retreat.
 * Otherwise tripping a limit strands the mechanism on the switch with no
 * way off it but a manual clear.
 *
 * Direction is per (switch, target) because it describes where the
 * switch sits relative to that axis's travel, which the switch itself
 * cannot know. Sign convention matches the control channels: positive =
 * extend / forward / increasing position.
 *
 * BOTH is the conservative fallback and is what an unannotated target
 * decodes to — over-blocking is recoverable, whereas a WRONG direction
 * drives further into the switch.
 */
#define SWITCH_BLOCK_BOTH       0
#define SWITCH_BLOCK_POSITIVE   1   /* block extend / forward  */
#define SWITCH_BLOCK_NEGATIVE   2   /* block retract / reverse */

/* Max targets one switch can stop. Small on purpose: this is the local
 * interlock list, not a routing graph. */
#define SWITCH_INPUT_MAX_TARGETS      4
#define SWITCH_INPUT_TARGET_ID_LEN    32

#endif /* SWITCH_INPUT_PROTOCOL_H */
