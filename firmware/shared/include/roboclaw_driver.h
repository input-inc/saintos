/**
 * SAINT.OS Firmware - RoboClaw Solo 60A Motor Controller Driver (shared)
 *
 * Drives up to 8 BasicMicro RoboClaw Solo 60A motor controllers over
 * CRC16 packet serial (addresses 0x80-0x87) — bidirectional protocol
 * with encoder, voltage, current, and temperature telemetry.
 *
 * Platform-agnostic: protocol, connection state machine, duty
 * keepalive, save/load, JSON parse, ESTOP, and peripheral_driver_t
 * glue live in shared/src/roboclaw_driver.c. Each platform's UART
 * adapter implements roboclaw_transport_ops — see
 * shared/include/roboclaw_transport.h. RP2040 transport additionally
 * handles the PIO-UART path (used when the PCB has swapped TX/RX
 * routing); Teensy transport always uses HardwareSerial.
 */

#ifndef SAINT_ROBOCLAW_DRIVER_H
#define SAINT_ROBOCLAW_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "roboclaw_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

void roboclaw_init(void);
void roboclaw_update(void);
bool roboclaw_is_connected(void);

bool roboclaw_set_duty(uint8_t unit, int16_t duty);
bool roboclaw_stop(uint8_t unit);
void roboclaw_stop_all(void);

struct peripheral_driver;
const struct peripheral_driver* roboclaw_get_peripheral_driver(void);

/* Wire-level debug passthrough — lets the host drive the RoboClaw UART
 * directly to investigate protocol issues. JSON ops: raw / sniff /
 * reconfigure. Results publish via saint_log_publish with the
 * "roboclaw_dbg:" prefix so the CLI can filter. */
void roboclaw_debug_handle_json(const char* json);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_ROBOCLAW_DRIVER_H */
