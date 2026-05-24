/**
 * SAINT.OS Firmware - SyRen 50 Packetized Serial Protocol
 *
 * Platform-agnostic protocol definitions for the Dimension Engineering
 * SyRen 50 regenerative motor driver. Uses 4-byte packetized serial:
 *   [Address] [Command] [Value] [Checksum]
 *
 * Up to 8 units share one serial line via addresses 128-135.
 */

#ifndef SYREN_PROTOCOL_H
#define SYREN_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Protocol Constants
// =============================================================================

// Commands
#define SYREN_CMD_MOTOR_FWD         0   // Forward (value 0-127)
#define SYREN_CMD_MOTOR_REV         1   // Reverse (value 0-127)
#define SYREN_CMD_MIN_VOLTAGE       2   // Set minimum voltage
#define SYREN_CMD_MAX_VOLTAGE       3   // Set maximum voltage
#define SYREN_CMD_SERIAL_TIMEOUT    14  // Set serial timeout
#define SYREN_CMD_BAUD_RATE         15  // Set baud rate
#define SYREN_CMD_DEADBAND          16  // Set deadband
#define SYREN_CMD_RAMPING           17  // Set ramping (acceleration profile)

// Address range
#define SYREN_ADDRESS_MIN           128
#define SYREN_ADDRESS_MAX           135
#define SYREN_DEFAULT_ADDRESS       128

// Serial
#define SYREN_DEFAULT_BAUD          9600
#define SYREN_AUTOBAUD_BYTE         0xAA

// Virtual GPIO mapping
#define SYREN_VIRTUAL_GPIO_BASE     224
#define SYREN_MAX_CHANNELS          8

// =============================================================================
// Packet Builder
// =============================================================================

/**
 * Build a SyRen packetized serial packet.
 *
 * @param address  Device address (128-135)
 * @param command  Command byte (0=fwd, 1=rev, etc.)
 * @param value    Value byte (0-127)
 * @param packet   Output buffer (must be at least 4 bytes)
 */
static inline void syren_build_packet(uint8_t address, uint8_t command,
                                       uint8_t value, uint8_t packet[4])
{
    packet[0] = address;
    packet[1] = command;
    packet[2] = value;
    packet[3] = (address + command + value) & 0x7F;
}

#ifdef __cplusplus
}
#endif

#endif // SYREN_PROTOCOL_H
