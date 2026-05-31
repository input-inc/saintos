/**
 * SAINT.OS Firmware - Pathfinder BMS Driver (shared)
 *
 * Read-only BMS sensor using the JBD UART protocol (9600 baud 8N1).
 * Reports pack voltage, current, SOC, remaining capacity, temperatures,
 * cycle count, protection status, and individual cell voltages.
 * Values are exposed as 24 virtual GPIO channels (276-299).
 *
 * Platform-agnostic: protocol frame parsing + state machine + the
 * peripheral_driver_t glue live in shared/src/pathfinder_bms_driver.c.
 * Each platform's UART adapter implements pathfinder_bms_transport_ops
 * — see shared/include/pathfinder_bms_transport.h.
 */

#ifndef SAINT_PATHFINDER_BMS_DRIVER_H
#define SAINT_PATHFINDER_BMS_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "jbd_bms_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Driver API ─────────────────────────────────────────────────── */

void  pathfinder_bms_init(void);
void  pathfinder_bms_update(void);
bool  pathfinder_bms_is_connected(void);

float pathfinder_bms_get_pack_voltage(void);
float pathfinder_bms_get_current(void);
float pathfinder_bms_get_soc(void);
float pathfinder_bms_get_remain_cap(void);
float pathfinder_bms_get_temp1(void);
float pathfinder_bms_get_temp2(void);
uint16_t pathfinder_bms_get_cycles(void);
uint16_t pathfinder_bms_get_protection_status(void);
float pathfinder_bms_get_cell_voltage(uint8_t cell);

/* ── Registration with the peripheral manager ───────────────────── */

struct peripheral_driver;
const struct peripheral_driver* pathfinder_bms_get_peripheral_driver(void);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_PATHFINDER_BMS_DRIVER_H */
