/**
 * SAINT.OS Firmware - TMC2208 Stepper Driver (shared)
 *
 * Up to 4 Trinamic TMC2208 stepper amplifiers on one node. Each axis
 * needs:
 *   - STEP and DIR GPIO pins (firmware-generated pulses)
 *   - A shared UART pair (config + status readback)
 *   - A unique slave address 0..3 (set on the PCB via MS1/MS2 strapping)
 *
 * Operator-facing channels per axis:
 *   0  target_position    write, [-1,1] -> ±max_position steps
 *   1  target_velocity    write, [-1,1] -> ±max_speed_pps pulses/s
 *   2  current_position   read,  signed step count
 *   3  error_flags        read,  TMC2208 DRV_STATUS bits as float
 *
 * Motion is constant-velocity (no accel ramp). When target_position
 * is commanded, the firmware computes the required steps + direction
 * and emits pulses at target_velocity until current_position reaches
 * target_position.
 *
 * Step generation backbone is platform-specific (RP2040 PIO,
 * Teensy IntervalTimer) — see the transport ops.
 */

#ifndef SAINT_TMC2208_DRIVER_H
#define SAINT_TMC2208_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "tmc2208_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

void tmc2208_init(void);
void tmc2208_update(void);
bool tmc2208_is_connected(void);

bool tmc2208_set_target_position(uint8_t axis, int32_t position);
bool tmc2208_set_target_velocity(uint8_t axis, int32_t pulses_per_sec);
bool tmc2208_halt(uint8_t axis);
void tmc2208_halt_all(void);

struct peripheral_driver;
const struct peripheral_driver* tmc2208_get_peripheral_driver(void);

/* Called from the step-generation ISR / timer once per emitted STEP
 * pulse to bump the axis's step counter toward target_position.
 * Returns true if more steps are pending for this axis; false when
 * current_position == target_position and the timer should stop
 * scheduling this axis. */
bool tmc2208_step_done(uint8_t axis);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_TMC2208_DRIVER_H */
