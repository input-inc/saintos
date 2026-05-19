/**
 * SAINT.OS Firmware - FrSky FBUS Protocol Definitions
 *
 * Platform-agnostic protocol constants for the FrSky FBUS telemetry protocol.
 * FBUS is the newer single-wire variant that the FAS100 ADV also supports;
 * the sensor auto-selects between S.Port and FBUS at boot based on what the
 * master is doing on the bus.
 *
 * Differences from S.Port:
 *   - 460800 baud instead of 57600
 *   - No byte stuffing (frames have an explicit length byte)
 *   - Poll frame is [0x08, sensor_id, 0x10] — 3 bytes instead of 2
 *   - Response frame is [0x08, sensor_id, 0x10, data_id(2), value(4), crc] — 10 bytes
 *
 * What's the same:
 *   - Inverted half-duplex UART signaling
 *   - sensor_id is the same CRC-encoded physical ID (0x22 for FAS100)
 *   - data_id values (current/voltage/temp) and units
 *   - CRC algorithm (sport_crc_calculate covers the bytes from len through value)
 */

#ifndef FBUS_PROTOCOL_H
#define FBUS_PROTOCOL_H

#include <stdint.h>
#include "sport_protocol.h"  /* reuse sensor_id, data IDs, CRC */

#ifdef __cplusplus
extern "C" {
#endif

#define FBUS_BAUD_RATE              460800

/* Wire-level frame bytes — the first three bytes of every FBUS frame
 * (poll or response) identify the target sensor. */
#define FBUS_LEN_BYTE               0x08    /* payload byte count between len and crc */
#define FBUS_FRAME_ID_DATA          0x10    /* sensor data frame type */

/* Sensor IDs are the same CRC-encoded physical IDs used by S.Port,
 * so the FAS100 still answers to 0x22 in FBUS mode. */
#define FBUS_FAS100_SENSOR_ID       SPORT_FAS100_PHYSICAL_ID

#define FBUS_POLL_FRAME_SIZE        3       /* len + sensor_id + frame_id */
#define FBUS_RESPONSE_FRAME_SIZE    10      /* poll bytes + data_id(2) + value(4) + crc */

#ifdef __cplusplus
}
#endif

#endif /* FBUS_PROTOCOL_H */
