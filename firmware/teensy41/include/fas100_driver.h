/**
 * SAINT.OS Node Firmware - FrSky FAS100 ADV Sensor Driver (Teensy 4.1)
 *
 * Read-only current/voltage/temperature sensor using the FrSky S.Port
 * protocol (inverted half-duplex UART at 57600 baud).
 *
 * Values are exposed as 4 virtual GPIO channels (232-235).
 */

#ifndef FAS100_DRIVER_H
#define FAS100_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

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

#ifdef __cplusplus
}
#endif

#endif // FAS100_DRIVER_H
