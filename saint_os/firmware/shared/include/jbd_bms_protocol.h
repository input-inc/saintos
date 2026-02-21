/**
 * SAINT.OS Firmware - JBD BMS UART Protocol Definitions
 *
 * Platform-agnostic protocol constants for the JBD (Overkill Solar Pathfinder)
 * BMS UART protocol. 9600 baud, 8N1.
 *
 * Frame format:
 *   Request:  [0xDD] [Action] [Register] [Length] [Data...] [CRC_hi] [CRC_lo] [0x77]
 *   Response: [0xDD] [Action] [Register] [Length] [Data...] [CRC_hi] [CRC_lo] [0x77]
 *
 * Read request uses Action=0xA5, Length=0x00 (7 bytes total, no data).
 * Success response uses Action=0x00; error response uses Action=0x01.
 *
 * Registers:
 *   0x03 - Basic info (voltage, current, SOC, capacity, temps, protection, cycles)
 *   0x04 - Cell voltages (2 bytes per cell, in mV)
 */

#ifndef JBD_BMS_PROTOCOL_H
#define JBD_BMS_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Protocol Constants
// =============================================================================

// UART settings
#define JBD_BAUD_RATE               9600

// Frame delimiters
#define JBD_FRAME_START             0xDD
#define JBD_FRAME_END               0x77

// Action bytes
#define JBD_ACTION_READ             0xA5    // Read request
#define JBD_ACTION_WRITE            0x5A    // Write request (not used)
#define JBD_ACTION_OK               0x00    // Response: success
#define JBD_ACTION_ERR              0x01    // Response: error

// Register addresses
#define JBD_REG_BASIC_INFO          0x03    // Pack voltage, current, SOC, etc.
#define JBD_REG_CELL_VOLTAGES       0x04    // Individual cell voltages

// Read request frame size (start + action + register + length=0 + crc_hi + crc_lo + end)
#define JBD_READ_REQUEST_SIZE       7

// Response limits
#define JBD_RESPONSE_MAX_DATA       64      // Max data payload in a response
#define JBD_RESPONSE_HEADER_SIZE    4       // start + action + register + length
#define JBD_RESPONSE_TRAILER_SIZE   3       // crc_hi + crc_lo + end
#define JBD_RESPONSE_MAX_SIZE       (JBD_RESPONSE_HEADER_SIZE + JBD_RESPONSE_MAX_DATA + JBD_RESPONSE_TRAILER_SIZE)

// Basic info register (0x03) data offsets (within data payload)
#define JBD_BASIC_PACK_VOLTAGE_OFF  0   // bytes 0-1: pack voltage (raw * 0.01V)
#define JBD_BASIC_CURRENT_OFF       2   // bytes 2-3: current (signed, raw * 0.01A)
#define JBD_BASIC_REMAIN_CAP_OFF    4   // bytes 4-5: remaining capacity (raw * 0.01Ah)
#define JBD_BASIC_FULL_CAP_OFF      6   // bytes 6-7: full capacity (raw * 0.01Ah)
#define JBD_BASIC_CYCLES_OFF        8   // bytes 8-9: cycle count
#define JBD_BASIC_PROD_DATE_OFF     10  // bytes 10-11: production date
#define JBD_BASIC_BALANCE_OFF       12  // bytes 12-13: balance status (low cells)
#define JBD_BASIC_BALANCE_HI_OFF    14  // bytes 14-15: balance status (high cells)
#define JBD_BASIC_PROTECTION_OFF    16  // bytes 16-17: protection status bits
#define JBD_BASIC_SOFTWARE_VER_OFF  18  // byte 18: software version
#define JBD_BASIC_SOC_OFF           19  // byte 19: SOC (0-100%)
#define JBD_BASIC_FET_STATUS_OFF    20  // byte 20: FET status
#define JBD_BASIC_CELL_COUNT_OFF    21  // byte 21: number of cells
#define JBD_BASIC_NTC_COUNT_OFF     22  // byte 22: number of NTC temp sensors
#define JBD_BASIC_NTC_START_OFF     23  // bytes 23+: NTC values (2 bytes each)

// Minimum data length for basic info (23 bytes + at least 1 NTC)
#define JBD_BASIC_MIN_DATA_LEN      25

// Max cell count
#define JBD_MAX_CELLS               16

// =============================================================================
// Virtual GPIO Mapping
// =============================================================================

#define JBD_BMS_VIRTUAL_GPIO_BASE   276
#define JBD_BMS_CHANNEL_COUNT       24

// Channel indices
#define JBD_BMS_CH_PACK_VOLTAGE     0   // GPIO 276
#define JBD_BMS_CH_CURRENT          1   // GPIO 277
#define JBD_BMS_CH_SOC              2   // GPIO 278
#define JBD_BMS_CH_REMAIN_CAP       3   // GPIO 279
#define JBD_BMS_CH_TEMP1            4   // GPIO 280
#define JBD_BMS_CH_TEMP2            5   // GPIO 281
#define JBD_BMS_CH_CYCLES           6   // GPIO 282
#define JBD_BMS_CH_PROTECTION       7   // GPIO 283
#define JBD_BMS_CH_CELL_BASE        8   // GPIO 284-299 (Cell01-Cell16)

// =============================================================================
// Protocol Defaults
// =============================================================================

#define JBD_BMS_DEFAULT_POLL_INTERVAL_MS  1000

// =============================================================================
// Checksum Calculation
// =============================================================================

/**
 * Calculate JBD checksum over register + length + data bytes.
 *
 * The checksum is computed by subtracting each byte from a running sum:
 *   uint16_t sum = 0; for each byte: sum -= byte;
 * Result is a 16-bit value appended as [hi][lo] before the end byte.
 *
 * @param data   Pointer to bytes (register + length + data payload)
 * @param len    Number of bytes
 * @return       16-bit checksum
 */
static inline uint16_t jbd_checksum_calculate(const uint8_t* data, size_t len)
{
    uint16_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum -= data[i];
    }
    return sum;
}

/**
 * Build a read request frame for the given register.
 *
 * @param reg    Register address (JBD_REG_BASIC_INFO or JBD_REG_CELL_VOLTAGES)
 * @param buf    Output buffer (must be at least JBD_READ_REQUEST_SIZE bytes)
 */
static inline void jbd_build_read_request(uint8_t reg, uint8_t* buf)
{
    buf[0] = JBD_FRAME_START;
    buf[1] = JBD_ACTION_READ;
    buf[2] = reg;
    buf[3] = 0x00;  // length = 0 for read requests

    // Checksum over register + length bytes
    uint8_t crc_data[2] = { reg, 0x00 };
    uint16_t crc = jbd_checksum_calculate(crc_data, 2);
    buf[4] = (uint8_t)(crc >> 8);
    buf[5] = (uint8_t)(crc & 0xFF);
    buf[6] = JBD_FRAME_END;
}

#ifdef __cplusplus
}
#endif

#endif // JBD_BMS_PROTOCOL_H
