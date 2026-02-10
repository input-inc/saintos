/**
 * SAINT.OS Firmware - FrSky S.Port Protocol Definitions
 *
 * Platform-agnostic protocol constants for the FrSky S.Port (Smart Port)
 * telemetry protocol. Used by the FAS100 ADV current/voltage sensor.
 *
 * S.Port is an inverted half-duplex UART at 57600 baud.
 * The MCU acts as poll master, sending poll frames and parsing responses.
 */

#ifndef SPORT_PROTOCOL_H
#define SPORT_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// S.Port Protocol Constants
// =============================================================================

// UART settings
#define SPORT_BAUD_RATE             57600

// Frame bytes
#define SPORT_POLL_HEADER           0x7E    // Poll frame start byte
#define SPORT_DATA_HEADER           0x10    // Data frame type byte (sensor response)

// Byte stuffing
#define SPORT_STUFF_MARKER          0x7D
#define SPORT_STUFF_MASK            0x20    // XOR mask for stuffed bytes

// FAS100 ADV physical ID (sensor address for polling)
#define SPORT_FAS100_PHYSICAL_ID    0x22

// =============================================================================
// FAS100 Data IDs (16-bit, little-endian in frames)
// =============================================================================

#define SPORT_DATA_ID_CURRENT       0x0200  // Current (value / 10.0 = amps)
#define SPORT_DATA_ID_VOLTAGE       0x0210  // Voltage (value / 100.0 = volts)
#define SPORT_DATA_ID_TEMP1         0x0400  // Temperature 1 (value = degrees C)
#define SPORT_DATA_ID_TEMP2         0x0410  // Temperature 2 (value = degrees C)

// =============================================================================
// Virtual GPIO Mapping
// =============================================================================

#define FAS100_VIRTUAL_GPIO_BASE    232
#define FAS100_CHANNEL_COUNT        4

// Channel indices
#define FAS100_CH_CURRENT           0
#define FAS100_CH_VOLTAGE           1
#define FAS100_CH_TEMP1             2
#define FAS100_CH_TEMP2             3

// =============================================================================
// Protocol Defaults
// =============================================================================

#define FAS100_DEFAULT_POLL_INTERVAL_MS  50
#define FAS100_POLL_FRAME_SIZE           2   // 0x7E + physical_id
#define FAS100_RESPONSE_FRAME_SIZE       8   // header + data_id(2) + value(4) + crc

// =============================================================================
// CRC Calculation
// =============================================================================

/**
 * Calculate S.Port CRC byte.
 * Sum all payload bytes with carry folding, then complement.
 *
 * @param data   Pointer to payload bytes (excluding poll header and CRC)
 * @param len    Number of bytes
 * @return       CRC byte to send/verify
 */
static inline uint8_t sport_crc_calculate(const uint8_t* data, uint8_t len)
{
    uint16_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc += data[i];
        crc = (crc & 0xFF) + (crc >> 8);
    }
    return 0xFF - (uint8_t)(crc & 0xFF);
}

#ifdef __cplusplus
}
#endif

#endif // SPORT_PROTOCOL_H
