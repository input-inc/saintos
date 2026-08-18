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

#include <stdbool.h>
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

/* System sub-commands (first data byte after channel+flags), followed by
 * zero or more bit-packed parameters. Reference Manual pp.13-15. */
#define KANGAROO_SYS_POWERDOWN      0   /* power down this channel  */
#define KANGAROO_SYS_POWERDOWN_ALL  1   /* power down all channels  */
/* Tuning. These drive a tune without the Autotune button — the same
 * commands DEScribe itself uses. Order matters: ENTER_MODE, then
 * SET_DISABLED_CHANNELS (all channels come up disabled for safety and
 * will not move until you clear the mask), then jog with
 * CONTROL_OPEN_LOOP, then GO. See docs/KANGAROO_BRINGUP.md. */
#define KANGAROO_SYS_TUNE_ENTER_MODE        3   /* param: tune mode      */
#define KANGAROO_SYS_TUNE_GO                4   /* no params             */
#define KANGAROO_SYS_TUNE_ABORT             5   /* no params             */
#define KANGAROO_SYS_TUNE_CONTROL_OPEN_LOOP 6   /* param: signed power   */
#define KANGAROO_SYS_TUNE_SET_DISABLED_CH   8   /* param: bitmask        */
#define KANGAROO_SYS_SET_BAUD_RATE          32  /* param: 0..3           */
#define KANGAROO_SYS_SET_SERIAL_TIMEOUT     33  /* param: 1/16 s units   */

/* Tune modes for KANGAROO_SYS_TUNE_ENTER_MODE. Mode 1 is the one this
 * firmware drives: with absolute (potentiometer) feedback there is
 * nothing to seek, so the axis does not home on startup. Modes 2 and 3
 * both home automatically and are deliberately unused here. */
#define KANGAROO_TUNE_MODE_TEACH          1
#define KANGAROO_TUNE_MODE_LIMIT_SWITCH   2
#define KANGAROO_TUNE_MODE_MECH_STOPS     3

/* Get/Status reply error codes (Reference Manual p.12). NOTE these are a
 * DIFFERENT numbering from the LED blink codes in the main Kangaroo
 * manual — do not decode one with the other's table. */
#define KANGAROO_ERR_NONE           0
#define KANGAROO_ERR_NOT_STARTED    1   /* send Start                     */
#define KANGAROO_ERR_NOT_HOMED      2   /* send Home                      */
#define KANGAROO_ERR_CONTROL        3   /* send Start to clear            */
#define KANGAROO_ERR_WRONG_MODE     4   /* DIPs disagree with the tune    */
#define KANGAROO_ERR_BAD_PARAMETER  5
#define KANGAROO_ERR_SERIAL_TIMEOUT 6   /* or TX disconnected; Start clears */

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

/* Control Open Loop has its own, NARROWER range than the bit-packer:
 * -(2^28 - 1) to 2^28 - 1 (Reference Manual p.14). Clamp jog power with
 * this, never with KANGAROO_BITPACK_MAX — using the latter would let a
 * caller command double the intended power on the one operation that
 * runs with no feedback and no limits. */
#define KANGAROO_OPEN_LOOP_MAX      268435455L

/* ── Virtual GPIO map: 8 boards/channels × 6 sub-channels = 48 ──── */

#define KANGAROO_VIRTUAL_GPIO_BASE  364   /* first free base after TMC2208 (348..363) */
#define KANGAROO_MAX_UNITS          8
#define KANGAROO_CHANNELS_PER_UNIT  10
#define KANGAROO_MAX_CHANNELS       (KANGAROO_MAX_UNITS * KANGAROO_CHANNELS_PER_UNIT)

/* Sub-channel indices within each unit (one Kangaroo motor channel). */
#define KANGAROO_SUB_TARGET_POSITION   0  /* write, [-1,1] -> ±max_position */
#define KANGAROO_SUB_TARGET_SPEED      1  /* write, [-1,1] -> ±max_speed    */
#define KANGAROO_SUB_CURRENT_POSITION  2  /* read,  machine units           */
#define KANGAROO_SUB_CURRENT_SPEED     3  /* read,  units/s                 */
#define KANGAROO_SUB_MOVING            4  /* read,  1 = motion pending (busy)*/
#define KANGAROO_SUB_ERROR_STATUS      5  /* read,  last Kangaroo error code */
/* Teach-tune jog. Deliberately a CHANNEL rather than a peripheral_command:
 * press-and-hold jog is a stream, and /control is BEST_EFFORT depth 1
 * (newest-wins) while /command is RELIABLE depth 8. On a stalled link a
 * reliable queue would deliver a burst of stale non-zero jogs ahead of
 * the operator's release-to-zero — and because each arrival refreshes
 * the firmware dead-man, that burst would defeat the dead-man rather
 * than trip it. Newest-wins has no such failure mode. */
#define KANGAROO_SUB_JOG               6  /* write, [-1,1] of the power cap */
#define KANGAROO_SUB_TUNE_STATE        7  /* read,  kangaroo_tune_state_t   */
/* Taught travel limits, cached from the last tune_read_extents. These
 * read the driver's cache, NOT the wire — the Get 8/9 round-trip only
 * happens on an explicit tune_read_extents command, so polling these
 * costs nothing. Read-only by nature: the protocol has no command to
 * set them, they come from where the axis was jogged during the teach. */
#define KANGAROO_SUB_TAUGHT_MIN        8  /* read,  machine units */
#define KANGAROO_SUB_TAUGHT_MAX        9  /* read,  machine units */

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

/* ── System commands ────────────────────────────────────────────── */
/*
 * Data layout is channel, flags, sub-command, then zero or more
 * bit-packed parameters.
 *
 * flags stays 0 — i.e. no sequence code — and must stay 0 around tuning.
 * Reference Manual p.12: "Tuning commands may have unusual effects on
 * sequence code... These effects are not necessarily limited to the
 * channel being commanded."
 */
static inline size_t kangaroo_build_system(uint8_t address, uint8_t channel,
                                           uint8_t sub, bool has_param,
                                           int32_t param, uint8_t* buf)
{
    uint8_t data[8];
    size_t n = 0;
    data[n++] = channel;
    data[n++] = 0;              /* flags — see note above */
    data[n++] = sub;
    if (has_param) n += kangaroo_bitpack(&data[n], param);
    return kangaroo_write_command(address, KANGAROO_CMD_SYSTEM, data,
                                  (uint8_t)n, buf);
}

/* Enter a tune mode. Equivalent to pressing the Autotune button until
 * you reach `mode`. Every channel comes up DISABLED afterwards — you
 * must send kangaroo_build_tune_set_disabled_channels(…, 0) before the
 * axis will move. */
static inline size_t kangaroo_build_tune_enter_mode(uint8_t address,
                                                    uint8_t channel,
                                                    uint8_t mode, uint8_t* buf)
{
    return kangaroo_build_system(address, channel,
                                 KANGAROO_SYS_TUNE_ENTER_MODE, true,
                                 (int32_t)mode, buf);
}

/* Clear the post-Enter-Mode safety interlock. mask 0 enables all channels. */
static inline size_t kangaroo_build_tune_set_disabled_channels(
    uint8_t address, uint8_t channel, int32_t mask, uint8_t* buf)
{
    return kangaroo_build_system(address, channel,
                                 KANGAROO_SYS_TUNE_SET_DISABLED_CH, true,
                                 mask, buf);
}

/* Open-loop jog, used to position the axis for a Teach tune.
 *
 * This is genuinely open loop: no feedback, no travel limits, no
 * protection. It will drive into the mechanical hard stops if nothing
 * stops it, so callers own a dead-man and a power cap. `power` is
 * clamped here to the command's documented range, which is NARROWER
 * than the bit-packer's — see KANGAROO_OPEN_LOOP_MAX. */
static inline size_t kangaroo_build_tune_open_loop(uint8_t address,
                                                   uint8_t channel,
                                                   int32_t power, uint8_t* buf)
{
    if (power >  KANGAROO_OPEN_LOOP_MAX) power =  (int32_t)KANGAROO_OPEN_LOOP_MAX;
    if (power < -KANGAROO_OPEN_LOOP_MAX) power = -(int32_t)KANGAROO_OPEN_LOOP_MAX;
    return kangaroo_build_system(address, channel,
                                 KANGAROO_SYS_TUNE_CONTROL_OPEN_LOOP, true,
                                 power, buf);
}

/* Begin the tune cycle. The axis starts moving on its own shortly after
 * this. Tuning has an automatic serial timeout — the caller must keep
 * sending packets (a Get loop does the job) or the Kangaroo aborts. */
static inline size_t kangaroo_build_tune_go(uint8_t address, uint8_t channel,
                                            uint8_t* buf)
{
    return kangaroo_build_system(address, channel, KANGAROO_SYS_TUNE_GO,
                                 false, 0, buf);
}

/* Abort an in-progress tune. This is the software e-stop for the tune. */
static inline size_t kangaroo_build_tune_abort(uint8_t address, uint8_t channel,
                                               uint8_t* buf)
{
    return kangaroo_build_system(address, channel, KANGAROO_SYS_TUNE_ABORT,
                                 false, 0, buf);
}

#ifdef __cplusplus
}
#endif

#endif /* KANGAROO_PROTOCOL_H */
