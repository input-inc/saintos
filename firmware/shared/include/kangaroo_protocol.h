/**
 * SAINT.OS Firmware - Dimension Engineering Kangaroo X2 protocol
 *
 * The Kangaroo X2 is a self-tuning closed-loop motion controller that
 * piggybacks on a Sabertooth / SyRen power stage. Unlike the SyRen
 * (open-loop, write-only), the Kangaroo speaks a bidirectional protocol
 * with position/speed feedback and supports TWO serial framings:
 *
 *   1. Packet Serial (default here) — binary, CRC-14 framed, bit-packed
 *      numbers. Robust against line noise; what DE's own library uses.
 *        [Address | Command | Length | Data... | CRC_lo | CRC_hi]
 *
 *   2. Simplified Serial — newline-terminated ASCII commands, replies
 *      terminated with "\r\n". Human-readable, easy to debug, no CRC.
 *        e.g.  "1,start\n"  "1,p1000 s200\n"  "1,getp\n" -> "1,P1000\r\n"
 *
 * This header is a faithful port of DE's Kangaroo Arduino / C# library:
 * crc14(), bitpackNumber(), and writeKangarooCommand() are translated
 * verbatim (see the Packet Serial Reference Manual). The driver core
 * (shared/src/kangaroo_driver.c) selects packet vs simple per channel
 * via the operator-set protocol param and dispatches UART I/O through
 * the per-platform transport ops (shared/include/kangaroo_transport.h).
 *
 * Channels: one Kangaroo board = one address (default 128, high bit set
 * on the wire) and two motor channels named by a single character —
 * '1'/'2' (independent mode) or 'D'/'T' (mixed/differential mode). The
 * channel name lives INSIDE the payload, NOT in the address byte. There
 * is no Sabertooth-style 0xAA autobaud byte: open at the configured
 * baud (default 9600 8N1) and start framing immediately.
 */

#ifndef KANGAROO_PROTOCOL_H
#define KANGAROO_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Serial ─────────────────────────────────────────────────────── */

#define KANGAROO_DEFAULT_BAUD       9600u
#define KANGAROO_ADDRESS_MIN        128
#define KANGAROO_ADDRESS_MAX        135
#define KANGAROO_DEFAULT_ADDRESS    128

/* protocol param values (also stored in flash) */
#define KANGAROO_PROTO_PACKET       0
#define KANGAROO_PROTO_SIMPLE       1

/* ── Packet command opcodes (DE Kangaroo.h enum) ────────────────── */

#define KANGAROO_CMD_START          32  /* 0x20 */
#define KANGAROO_CMD_UNITS          33  /* 0x21 */
#define KANGAROO_CMD_HOME           34  /* 0x22 */
#define KANGAROO_CMD_STATUS         35  /* 0x23 — "Get" */
#define KANGAROO_CMD_MOVE           36  /* 0x24 */
#define KANGAROO_CMD_SYSTEM         37  /* 0x25 */
#define KANGAROO_RC_STATUS          67  /* 0x43 — Get/Status reply opcode */

/* Move-parameter type bytes (add KANGAROO_INCREMENTAL for relative). */
#define KANGAROO_MOVE_POSITION      1
#define KANGAROO_MOVE_SPEED         2   /* speed limit when combined with position */
#define KANGAROO_MOVE_SPEED_RAMP    3

/* Get-parameter codes. */
#define KANGAROO_GET_POSITION       1
#define KANGAROO_GET_SPEED          2
#define KANGAROO_GET_ABS_MIN        8
#define KANGAROO_GET_ABS_MAX        9

#define KANGAROO_INCREMENTAL        64  /* OR into move/get parameter byte */

/* System sub-commands (first data byte after channel+flags). */
#define KANGAROO_SYS_POWERDOWN      0   /* power down this channel  */
#define KANGAROO_SYS_POWERDOWN_ALL  1   /* power down all channels  */

/* Get/Status reply flag bits (KangarooStatusFlags). */
#define KANGAROO_STATUS_ERROR       0x01  /* value is an error code        */
#define KANGAROO_STATUS_BUSY        0x02  /* motion still pending          */
#define KANGAROO_STATUS_ECHO_CODE   0x10  /* reply carries an echo code    */
#define KANGAROO_STATUS_RAW_UNITS   0x20  /* value is in raw machine units */
#define KANGAROO_STATUS_SEQUENCE    0x40  /* reply carries a sequence code */

/* CRC-14: init 0x3fff, reflected, poly constant 0x22f0 (0x21E8 Koopman),
 * final XOR 0x3fff. Processes 7 bits per byte. Verbatim from DE. */
#define KANGAROO_CRC_INIT           0x3fff
#define KANGAROO_CRC_POLY           0x22f0

/* Largest magnitude bitpackNumber can encode (2^29 - 1). */
#define KANGAROO_BITPACK_MAX        536870911L

/* ── Virtual GPIO map: 8 boards/channels × 6 sub-channels = 48 ──── */

#define KANGAROO_VIRTUAL_GPIO_BASE  364   /* first free base after TMC2208 (348..363) */
#define KANGAROO_MAX_UNITS          8
#define KANGAROO_CHANNELS_PER_UNIT  6
#define KANGAROO_MAX_CHANNELS       (KANGAROO_MAX_UNITS * KANGAROO_CHANNELS_PER_UNIT)

/* Sub-channel indices within each unit (one Kangaroo motor channel). */
#define KANGAROO_SUB_TARGET_POSITION   0  /* write, [-1,1] -> ±max_position */
#define KANGAROO_SUB_TARGET_SPEED      1  /* write, [-1,1] -> ±max_speed    */
#define KANGAROO_SUB_CURRENT_POSITION  2  /* read,  machine units           */
#define KANGAROO_SUB_CURRENT_SPEED     3  /* read,  units/s                 */
#define KANGAROO_SUB_MOVING            4  /* read,  1 = motion pending (busy)*/
#define KANGAROO_SUB_ERROR_STATUS      5  /* read,  last Kangaroo error code */

/* ── CRC-14 (verbatim port of DE crc14) ─────────────────────────── */

static inline uint16_t kangaroo_crc14(const uint8_t* data, size_t length)
{
    uint16_t crc = KANGAROO_CRC_INIT;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)(data[i] & 0x7f);
        for (int bit = 0; bit < 7; bit++) {
            if (crc & 1) { crc = (uint16_t)((crc >> 1) ^ KANGAROO_CRC_POLY); }
            else         { crc = (uint16_t)(crc >> 1); }
        }
    }
    return (uint16_t)(crc ^ KANGAROO_CRC_INIT);
}

/* ── Bit-packed numbers (verbatim port of DE bitpackNumber) ─────── */
/*
 * Positive n -> n*2; negative n -> abs(n)*2 + 1 (sign in bit 0). 6 bits
 * packed per output byte from low to high; bit 0x40 set means "more
 * bytes follow". 1..5 bytes out. We widen to int64 internally so the
 * doubling can't overflow / hit INT32_MIN UB for in-range inputs.
 */
static inline size_t kangaroo_bitpack(uint8_t* buffer, int32_t number)
{
    int64_t n = (int64_t)number;
    if (n < 0) { n = -n; n <<= 1; n |= 1; }
    else       {         n <<= 1;        }

    size_t i = 0;
    while (i < 5) {
        buffer[i++] = (uint8_t)((n & 0x3f) | (n >= 0x40 ? 0x40 : 0x00));
        n >>= 6;
        if (n == 0) break;
    }
    return i;
}

/* Decode a bit-packed number starting at buf[*idx], advancing *idx past
 * the bytes consumed. Mirror of DE readBitPackedNumber. */
static inline int32_t kangaroo_bitunpack(const uint8_t* buf, size_t len,
                                         size_t* idx)
{
    uint32_t enc = 0;
    int shift = 0;
    while (*idx < len && shift < 30) {
        uint8_t b = buf[*idx];
        (*idx)++;
        enc |= (uint32_t)(b & 0x3f) << shift;
        shift += 6;
        if (!(b & 0x40)) break;
    }
    if (enc & 1u) return -(int32_t)(enc >> 1);
    return (int32_t)(enc >> 1);
}

/* ── Packet framing (verbatim port of DE writeKangarooCommand) ──── */
/*
 * buffer must hold 5 + length bytes. The address is transmitted with
 * its high bit set (128..135); crc14 masks it off with & 0x7f. Returns
 * total bytes written (always 5 + length).
 */
static inline size_t kangaroo_write_command(uint8_t address, uint8_t command,
                                            const uint8_t* data, uint8_t length,
                                            uint8_t* buffer)
{
    buffer[0] = address;
    buffer[1] = command;
    buffer[2] = length;
    for (uint8_t i = 0; i < length; i++) buffer[3 + i] = data[i];
    uint16_t crc = kangaroo_crc14(buffer, (size_t)(3 + length));
    buffer[3 + length] = (uint8_t)(crc & 0x7f);
    buffer[4 + length] = (uint8_t)((crc >> 7) & 0x7f);
    return (size_t)(5 + length);
}

/* ── Higher-level packet builders ───────────────────────────────── */
/* `channel` is the single-character channel name ('1','2','D','T'). */

/* Start: must be sent to a channel after every power-up before any
 * motion command, or commands are ignored (error 1 / "not started"). */
static inline size_t kangaroo_build_start(uint8_t address, uint8_t channel,
                                          uint8_t* buf)
{
    uint8_t data[2] = { channel, 0 /* flags */ };
    return kangaroo_write_command(address, KANGAROO_CMD_START, data, 2, buf);
}

static inline size_t kangaroo_build_home(uint8_t address, uint8_t channel,
                                         uint8_t* buf)
{
    uint8_t data[2] = { channel, 0 /* flags */ };
    return kangaroo_write_command(address, KANGAROO_CMD_HOME, data, 2, buf);
}

/* Move to absolute position, optionally capping at speed_limit (units/s).
 * speed_limit < 0 omits the limit. Speed in a combined move is a LIMIT
 * and must be non-negative — the caller passes its magnitude. */
static inline size_t kangaroo_build_move_position(uint8_t address,
                                                  uint8_t channel,
                                                  int32_t position,
                                                  int32_t speed_limit,
                                                  uint8_t* buf)
{
    uint8_t data[16];
    size_t n = 0;
    data[n++] = channel;
    data[n++] = 0;                       /* move flags */
    data[n++] = KANGAROO_MOVE_POSITION;
    n += kangaroo_bitpack(&data[n], position);
    if (speed_limit >= 0) {
        data[n++] = KANGAROO_MOVE_SPEED; /* becomes a limit alongside position */
        n += kangaroo_bitpack(&data[n], speed_limit);
    }
    return kangaroo_write_command(address, KANGAROO_CMD_MOVE, data, (uint8_t)n, buf);
}

/* Run at signed speed (units/s). */
static inline size_t kangaroo_build_move_speed(uint8_t address, uint8_t channel,
                                               int32_t speed, uint8_t* buf)
{
    uint8_t data[10];
    size_t n = 0;
    data[n++] = channel;
    data[n++] = 0;                    /* move flags */
    data[n++] = KANGAROO_MOVE_SPEED;
    n += kangaroo_bitpack(&data[n], speed);
    return kangaroo_write_command(address, KANGAROO_CMD_MOVE, data, (uint8_t)n, buf);
}

/* Get/Status request for `param` (KANGAROO_GET_POSITION / _SPEED / ...). */
static inline size_t kangaroo_build_get(uint8_t address, uint8_t channel,
                                        uint8_t param, uint8_t* buf)
{
    uint8_t data[3] = { channel, 0 /* flags */, param };
    return kangaroo_write_command(address, KANGAROO_CMD_STATUS, data, 3, buf);
}

/* Power down (freewheel) this channel. */
static inline size_t kangaroo_build_powerdown(uint8_t address, uint8_t channel,
                                              uint8_t* buf)
{
    uint8_t data[3] = { channel, 0 /* flags */, KANGAROO_SYS_POWERDOWN };
    return kangaroo_write_command(address, KANGAROO_CMD_SYSTEM, data, 3, buf);
}

#ifdef __cplusplus
}
#endif

#endif /* KANGAROO_PROTOCOL_H */
