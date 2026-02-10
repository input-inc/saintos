/**
 * SAINT.OS Node Firmware - FrSky FAS100 ADV Sensor Driver (RP2040)
 *
 * Read-only current/voltage/temperature sensor using the FrSky S.Port
 * protocol (inverted half-duplex UART at 57600 baud).
 *
 * The MCU polls the sensor and parses response frames containing:
 *   - Current (0-100A)
 *   - Voltage (up to 14S LiPo)
 *   - Temperature 1 & 2
 *
 * Values are exposed as 4 virtual GPIO channels (232-235).
 */

#ifndef FAS100_DRIVER_H
#define FAS100_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "sport_protocol.h"

// =============================================================================
// Driver API
// =============================================================================

void fas100_init(void);
void fas100_update(void);
bool fas100_is_connected(void);

float fas100_get_current(void);
float fas100_get_voltage(void);
float fas100_get_temp1(void);
float fas100_get_temp2(void);

struct peripheral_driver;
const struct peripheral_driver* fas100_get_peripheral_driver(void);

#endif // FAS100_DRIVER_H
