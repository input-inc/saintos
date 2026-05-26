/**
 * SAINT.OS Firmware - RoboClaw Solo 60A Packet Serial Protocol
 *
 * Platform-agnostic protocol definitions for the BasicMicro
 * RoboClaw Solo 60A motor controller. Uses CRC16 packet serial:
 *   [Address] [Command] [Data...] [CRC16_hi] [CRC16_lo]
 *
 * Up to 8 units share one serial line via addresses 0x80-0x87.
 */

#ifndef ROBOCLAW_PROTOCOL_H
#define ROBOCLAW_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Protocol Constants
// =============================================================================

// Motor commands (Solo = M1 only)
#define ROBOCLAW_CMD_M1FORWARD      0   // Forward 0-127
#define ROBOCLAW_CMD_M1BACKWARD     1   // Reverse 0-127
#define ROBOCLAW_CMD_M1DUTY         32  // Signed 16-bit duty (-32767..+32767)

// Read commands
#define ROBOCLAW_CMD_GETM1ENC       16  // 32-bit encoder count + status
#define ROBOCLAW_CMD_GETM1SPEED     18  // 32-bit encoder speed (counts/sec)
#define ROBOCLAW_CMD_GETVERSION     21  // Firmware version string (null-terminated)
#define ROBOCLAW_CMD_GETMBATT       24  // 16-bit voltage (value/10 = volts)
#define ROBOCLAW_CMD_GETCURRENTS    49  // 16-bit M1 current + 16-bit M2 current
#define ROBOCLAW_CMD_GETTEMP        82  // 16-bit temp (value/10 = C)

// Address range
#define ROBOCLAW_ADDRESS_MIN        0x80
#define ROBOCLAW_ADDRESS_MAX        0x87
#define ROBOCLAW_DEFAULT_ADDRESS    0x80

// Serial
#define ROBOCLAW_DEFAULT_BAUD       38400
#define ROBOCLAW_ACK_BYTE           0xFF

// Timeouts
#define ROBOCLAW_BYTE_TIMEOUT_MS    10   // Max gap between bytes
#define ROBOCLAW_RESPONSE_TIMEOUT_MS 50  // Max wait for response start

// Virtual GPIO mapping: 5 channels per unit x 8 units = 40 channels
#define ROBOCLAW_VIRTUAL_GPIO_BASE  236
#define ROBOCLAW_MAX_UNITS          8
#define ROBOCLAW_CHANNELS_PER_UNIT  5
#define ROBOCLAW_MAX_CHANNELS       (ROBOCLAW_MAX_UNITS * ROBOCLAW_CHANNELS_PER_UNIT)

// Sub-channel indices within each unit
#define ROBOCLAW_SUB_MOTOR          0   // Read/Write (duty)
#define ROBOCLAW_SUB_ENCODER        1   // Read-only (position)
#define ROBOCLAW_SUB_VOLTAGE        2   // Read-only (battery V)
#define ROBOCLAW_SUB_CURRENT        3   // Read-only (motor A)
#define ROBOCLAW_SUB_TEMP           4   // Read-only (C)

// Duty range
#define ROBOCLAW_DUTY_MAX           32767
#define ROBOCLAW_DUTY_MIN           (-32767)

// Version response max length
#define ROBOCLAW_VERSION_MAX_LEN    48

// =============================================================================
// CRC16 Helpers (XMODEM polynomial 0x1021)
// =============================================================================

/**
 * Update CRC16 with one byte.
 * Uses XMODEM polynomial 0x1021, big-endian.
 */
static inline uint16_t roboclaw_crc16_update(uint16_t crc, uint8_t byte)
{
    crc ^= ((uint16_t)byte) << 8;
    for (uint8_t i = 0; i < 8; i++) {
        if (crc & 0x8000) {
            crc = (crc << 1) ^ 0x1021;
        } else {
            crc <<= 1;
        }
    }
    return crc;
}

/**
 * Calculate CRC16 over a buffer.
 */
static inline uint16_t roboclaw_crc16_calculate(const uint8_t* data, size_t len)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc = roboclaw_crc16_update(crc, data[i]);
    }
    return crc;
}

#ifdef __cplusplus
}
#endif

#endif // ROBOCLAW_PROTOCOL_H
