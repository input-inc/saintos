/**
 * SAINT.OS Node Firmware - Pathfinder BMS Driver (Teensy 4.1)
 *
 * Read-only BMS sensor using the JBD UART protocol (9600 baud, 8N1).
 * Reports pack voltage, current, SOC, remaining capacity, temperatures,
 * cycle count, protection status, and individual cell voltages.
 *
 * Values are exposed as 24 virtual GPIO channels (276-299).
 */

#ifndef PATHFINDER_BMS_DRIVER_H
#define PATHFINDER_BMS_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "jbd_bms_protocol.h"

// =============================================================================
// Driver API
// =============================================================================

void pathfinder_bms_init(void);
void pathfinder_bms_update(void);
bool pathfinder_bms_is_connected(void);

float pathfinder_bms_get_pack_voltage(void);
float pathfinder_bms_get_current(void);
float pathfinder_bms_get_soc(void);
float pathfinder_bms_get_remain_cap(void);
float pathfinder_bms_get_temp1(void);
float pathfinder_bms_get_temp2(void);
uint16_t pathfinder_bms_get_cycles(void);
uint16_t pathfinder_bms_get_protection_status(void);
float pathfinder_bms_get_cell_voltage(uint8_t cell);

struct peripheral_driver;
const struct peripheral_driver* pathfinder_bms_get_peripheral_driver(void);

#ifdef __cplusplus
}
#endif

#endif // PATHFINDER_BMS_DRIVER_H
