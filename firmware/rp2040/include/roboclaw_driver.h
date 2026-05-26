/**
 * SAINT.OS Node Firmware - RoboClaw Solo 60A Motor Controller Driver (RP2040)
 *
 * Drives up to 8 BasicMicro RoboClaw Solo 60A motor controllers
 * over CRC16 packet serial (addresses 0x80-0x87). Uses Pico SDK UART.
 * Provides bidirectional communication with encoder, voltage, current,
 * and temperature telemetry.
 */

#ifndef ROBOCLAW_DRIVER_H
#define ROBOCLAW_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
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

// =============================================================================
// Wire-level debug passthrough
// =============================================================================
//
// Lets the host drive the RoboClaw UART directly to investigate
// protocol issues that the normal driver path obscures (timeouts,
// CRC behavior, idle-line bytes, baud rate). Invoked from the
// control-message handler when JSON arrives with
//   {"action":"roboclaw_debug","op":"raw"|"sniff"|"reconfigure", …}
//
// Every result is reported via saint_log_publish with a "roboclaw_dbg:"
// prefix so the host-side CLI can filter for them in the log stream.
void roboclaw_debug_handle_json(const char* json);

#endif // ROBOCLAW_DRIVER_H
