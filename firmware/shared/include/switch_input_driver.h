/**
 * SAINT.OS Firmware - Generic switch / sensor input driver (shared)
 *
 * One physical on/off sensor as a first-class peripheral, so it can
 * affect more than one thing. See docs/SENSOR_INPUTS.md for the design.
 *
 * Two tiers of behaviour, deliberately separate:
 *
 *   Tier 1 — the local interlock implemented here. On assert, stop the
 *            named peripherals on THIS node, immediately, with no
 *            network in the path.
 *   Tier 2 — the state/latched channels flow to the server as ordinary
 *            telemetry, where the routing graph can drive anything at
 *            all, including sinks on other nodes.
 *
 * Debounce and latch live here rather than server-side because a magnet
 * sweeping past a reed is a PULSE, not a state: by the time the server
 * polls, it is over. That is the same reason these switches can't be
 * used on a Kangaroo's L1/L2 inputs.
 */

#ifndef SAINT_SWITCH_INPUT_DRIVER_H
#define SAINT_SWITCH_INPUT_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "switch_input_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

void switch_input_init(void);
void switch_input_update(void);

/* Clear a latched assert. No-op when the input is still physically
 * asserted — otherwise "clear" would appear to work and then instantly
 * re-latch, which reads as a broken button. */
bool switch_input_clear_latch(uint8_t unit);

/* True if any configured input is currently latched. */
bool switch_input_any_latched(void);

/* ── Per-platform pin access ────────────────────────────────────── */
/*
 * Implemented per platform, mirroring the RoboClaw's estop-pin hooks.
 * Deliberately NOT routed through pin_control_read_digital/_read_adc:
 * those require the pin to be registered in pin_config under
 * PIN_MODE_DIGITAL_IN / PIN_MODE_ADC, but a switch_input pin is claimed
 * under PIN_MODE_SWITCH_INPUT and would be rejected.
 *
 * Keeping the read behind these two functions is also what makes the
 * opto-isolated variant a one-function change rather than a rewrite.
 */
bool switch_input_read_digital_pin(uint8_t pin, bool pull_up);
bool switch_input_read_analog_mv(uint8_t pin, uint16_t* out_mv);

/* ── Registration ───────────────────────────────────────────────── */

struct peripheral_driver;
const struct peripheral_driver* switch_input_get_peripheral_driver(void);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_SWITCH_INPUT_DRIVER_H */
