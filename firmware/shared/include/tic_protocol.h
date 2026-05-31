/**
 * SAINT.OS Firmware - Pololu Tic Stepper Controller Serial Protocol
 *
 * Platform-agnostic protocol definitions for the Pololu Tic family
 * (T834 / T825 / T249 / 36v4) stepper motor controllers over TTL
 * serial. Up to 8 units share one serial line via the Pololu binary
 * protocol with per-unit device IDs (1..127).
 *
 * Pololu protocol frame layout:
 *   [0xAA] [device_id] [command & 0x7F] [data...]
 *
 * The command byte's MSB is CLEARED in Pololu mode (it's set in
 * compact-mode-only). Data bytes for 32-bit-write commands have
 * their MSB cleared and the 4 high bits are packed into a leading
 * "MSbs" byte. Block-read responses come back as raw little-endian
 * bytes — no CRC by default (we don't enable it).
 *
 * The Tic has a configurable command-timeout watchdog (default 1 s)
 * that kills motion if no command arrives. We send Reset Command
 * Timeout (quick 0x8C) as a keepalive while units are energized.
 */

#ifndef TIC_PROTOCOL_H
#define TIC_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Wire framing ──────────────────────────────────────────────── */

#define TIC_POLOLU_START_BYTE       0xAA
#define TIC_DEFAULT_DEVICE_ID       0x0E  /* factory default */
#define TIC_DEVICE_ID_MIN           1
#define TIC_DEVICE_ID_MAX           127

#define TIC_DEFAULT_BAUD            9600
#define TIC_BYTE_TIMEOUT_MS         10
#define TIC_RESPONSE_TIMEOUT_MS     50

/* ── Quick commands (single-byte, opcode MSB-cleared on wire) ───── */

#define TIC_CMD_SET_TARGET_POSITION      0xE0  /* 32-bit write */
#define TIC_CMD_SET_TARGET_VELOCITY      0xE3  /* 32-bit write */
#define TIC_CMD_HALT_AND_SET_POSITION    0xEC  /* 32-bit write */
#define TIC_CMD_HALT_AND_HOLD            0x89  /* quick */
#define TIC_CMD_ENERGIZE                 0x85  /* quick */
#define TIC_CMD_DEENERGIZE               0x86  /* quick */
#define TIC_CMD_EXIT_SAFE_START          0x83  /* quick */
#define TIC_CMD_RESET_COMMAND_TIMEOUT    0x8C  /* quick — keepalive */
#define TIC_CMD_GET_VARIABLE             0xA1  /* block read */

/* ── Variable offsets (Block Read targets) ──────────────────────── */

#define TIC_VAR_OPERATION_STATE     0x00  /* 1 byte */
#define TIC_VAR_ERROR_STATUS        0x02  /* 2 bytes uint16 — active errors */
#define TIC_VAR_PLANNING_MODE       0x09  /* 1 byte */
#define TIC_VAR_TARGET_POSITION     0x0A  /* 4 bytes int32 */
#define TIC_VAR_TARGET_VELOCITY     0x0E  /* 4 bytes int32 (pulses/10000s) */
#define TIC_VAR_CURRENT_POSITION    0x22  /* 4 bytes int32 */
#define TIC_VAR_CURRENT_VELOCITY    0x26  /* 4 bytes int32 (pulses/10000s) */
#define TIC_VAR_VIN_VOLTAGE         0x33  /* 2 bytes uint16 (mV) */

/* error_status bit indexes (TIC_VAR_ERROR_STATUS is a bit-set). */
#define TIC_ERR_INTENTIONALLY_DEENERGIZED  0x0001
#define TIC_ERR_MOTOR_DRIVER_ERROR         0x0002
#define TIC_ERR_LOW_VIN                    0x0004
#define TIC_ERR_KILL_SWITCH                0x0008
#define TIC_ERR_REQUIRED_INPUT_INVALID     0x0010
#define TIC_ERR_SERIAL_ERROR               0x0020
#define TIC_ERR_COMMAND_TIMEOUT            0x0040
#define TIC_ERR_SAFE_START_VIOLATION       0x0080
#define TIC_ERR_ERR_LINE_HIGH              0x0100

/* The wire unit for target_velocity / current_velocity. Multiply
 * pulses-per-second by this when sending; divide by it when reading. */
#define TIC_VELOCITY_WIRE_SCALE     10000

/* ── Virtual GPIO map: 8 units × 6 channels = 48 channels ───────── */

#define TIC_VIRTUAL_GPIO_BASE       300
#define TIC_MAX_UNITS               8
#define TIC_CHANNELS_PER_UNIT       6
#define TIC_MAX_CHANNELS            (TIC_MAX_UNITS * TIC_CHANNELS_PER_UNIT)

/* Sub-channel indices within each unit */
#define TIC_SUB_TARGET_POSITION     0   /* writable, scaled [-1,1] -> ±max_position */
#define TIC_SUB_TARGET_VELOCITY     1   /* writable, scaled [-1,1] -> ±max_speed_pps */
#define TIC_SUB_CURRENT_POSITION    2   /* read-only, steps */
#define TIC_SUB_CURRENT_VELOCITY    3   /* read-only, steps/s */
#define TIC_SUB_VIN_VOLTAGE         4   /* read-only, volts */
#define TIC_SUB_ERROR_STATUS        5   /* read-only, uint16 cast to float */

/* ── Encoding helpers ───────────────────────────────────────────── */

/* All Pololu-protocol encoders return bytes-written, never fail
 * (caller-supplied buffer must be large enough — max output is 8
 * bytes for a 32-bit write). */

static inline size_t tic_encode_quick(uint8_t device_id, uint8_t cmd,
                                       uint8_t* out)
{
    out[0] = TIC_POLOLU_START_BYTE;
    out[1] = device_id;
    out[2] = (uint8_t)(cmd & 0x7F);
    return 3;
}

static inline size_t tic_encode_32bit(uint8_t device_id, uint8_t cmd,
                                       int32_t value, uint8_t* out)
{
    uint32_t v = (uint32_t)value;
    uint8_t msbs = 0;
    if (v & 0x80U)         msbs |= 0x01;
    if (v & 0x8000U)       msbs |= 0x02;
    if (v & 0x800000U)     msbs |= 0x04;
    if (v & 0x80000000U)   msbs |= 0x08;
    out[0] = TIC_POLOLU_START_BYTE;
    out[1] = device_id;
    out[2] = (uint8_t)(cmd & 0x7F);
    out[3] = msbs;
    out[4] = (uint8_t)(v        & 0x7F);
    out[5] = (uint8_t)((v >> 8)  & 0x7F);
    out[6] = (uint8_t)((v >> 16) & 0x7F);
    out[7] = (uint8_t)((v >> 24) & 0x7F);
    return 8;
}

static inline size_t tic_encode_block_read(uint8_t device_id,
                                            uint8_t offset, uint8_t length,
                                            uint8_t* out)
{
    out[0] = TIC_POLOLU_START_BYTE;
    out[1] = device_id;
    out[2] = (uint8_t)(TIC_CMD_GET_VARIABLE & 0x7F);
    out[3] = (uint8_t)(offset & 0x7F);
    out[4] = (uint8_t)(length & 0x7F);
    return 5;
}

/* Decode helpers for block-read responses (little-endian). */
static inline uint16_t tic_decode_u16(const uint8_t* p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline int32_t tic_decode_i32(const uint8_t* p)
{
    return (int32_t)((uint32_t)p[0]
                   | ((uint32_t)p[1] << 8)
                   | ((uint32_t)p[2] << 16)
                   | ((uint32_t)p[3] << 24));
}

#ifdef __cplusplus
}
#endif

#endif /* TIC_PROTOCOL_H */
