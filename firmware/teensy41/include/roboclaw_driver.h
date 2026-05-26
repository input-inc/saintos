/**
 * SAINT.OS Node Firmware - RoboClaw Solo 60A Motor Controller Driver (Teensy 4.1)
 *
 * Drives up to 8 BasicMicro RoboClaw Solo 60A motor controllers
 * over CRC16 packet serial (addresses 0x80-0x87). Uses HardwareSerial.
 * Provides bidirectional communication with encoder, voltage, current,
 * and temperature telemetry.
 */

#ifndef ROBOCLAW_DRIVER_H
#define ROBOCLAW_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "roboclaw_protocol.h"

// =============================================================================
// Driver API
// =============================================================================

void roboclaw_init(void);
void roboclaw_update(void);
bool roboclaw_is_connected(void);

bool roboclaw_set_duty(uint8_t unit, int16_t duty);
bool roboclaw_stop(uint8_t unit);
void roboclaw_stop_all(void);

struct peripheral_driver;
const struct peripheral_driver* roboclaw_get_peripheral_driver(void);

#ifdef __cplusplus
}
#endif

#endif // ROBOCLAW_DRIVER_H
