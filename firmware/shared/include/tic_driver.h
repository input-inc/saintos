/**
 * SAINT.OS Firmware - Pololu Tic Stepper Controller Driver (shared)
 *
 * Drives up to 8 Pololu Tic stepper controllers (T834 / T825 / T249 /
 * 36v4) over TTL serial using the Pololu binary protocol with device-
 * ID addressing (1..127). Operator-controlled target_position and
 * target_velocity sub-channels; read-only telemetry for
 * current_position, current_velocity, vin_voltage, and error_status.
 *
 * Platform-agnostic: protocol layer, connection state machine,
 * keepalive (Reset Command Timeout while energized), save/load, and
 * peripheral_driver_t glue live in shared/src/tic_driver.c. Each
 * platform's UART adapter implements tic_transport_ops — see
 * shared/include/tic_transport.h.
 *
 * Step mode and current limit must be preconfigured on each Tic via
 * the Pololu Tic Control Center (USB) before deployment. The firmware
 * only emits motion commands + reads telemetry.
 */

#ifndef SAINT_TIC_DRIVER_H
#define SAINT_TIC_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "tic_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

void tic_init(void);
void tic_update(void);
bool tic_is_connected(void);

bool tic_set_target_position(uint8_t unit, int32_t position);
bool tic_set_target_velocity(uint8_t unit, int32_t pulses_per_sec);
bool tic_halt(uint8_t unit);
void tic_halt_all(void);

struct peripheral_driver;
const struct peripheral_driver* tic_get_peripheral_driver(void);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_TIC_DRIVER_H */
