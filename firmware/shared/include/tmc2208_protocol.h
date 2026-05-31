/**
 * SAINT.OS Firmware - TMC2208 UART Protocol
 *
 * Trinamic TMC2208 stepper driver, single-wire UART register access.
 * Slave address (0..3) is hardwired at the chip via MS1/MS2 pin
 * strapping; up to 4 chips can share one UART pair (TX → 1 kΩ → RX,
 * controllers' PDN_UART pins tied together).
 *
 * The chip itself only does step amplification — the MCU still has to
 * generate STEP and DIR pulses. This protocol layer is just for
 * register configuration (microsteps, run/hold current, stealthChop)
 * and status readback (DRV_STATUS, GSTAT).
 *
 * Frame layout:
 *   Write (master → chip, 8 bytes):
 *     [SYNC=0x05] [slave_addr] [reg|0x80] [d3] [d2] [d1] [d0] [CRC]
 *   Read request (master → chip, 4 bytes):
 *     [SYNC=0x05] [slave_addr] [reg     ] [CRC]
 *   Read reply (chip → master, 8 bytes):
 *     [SYNC=0x05] [0xFF     ] [reg     ] [d3] [d2] [d1] [d0] [CRC]
 *
 * Data bytes are MSB first on the wire. The chip's reply uses
 * slave_address = 0xFF (master).
 *
 * Half-duplex on a single wire: every byte we transmit echoes back
 * into our own RX FIFO. The transport layer is responsible for
 * draining the echo before parsing replies (same pattern as FAS100).
 */

#ifndef TMC2208_PROTOCOL_H
#define TMC2208_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TMC2208_SYNC                   0x05
#define TMC2208_MASTER_REPLY_ADDR      0xFF
#define TMC2208_WRITE_FLAG             0x80
#define TMC2208_DEFAULT_BAUD           115200

#define TMC2208_WRITE_FRAME_BYTES      8
#define TMC2208_READ_REQUEST_BYTES     4
#define TMC2208_READ_REPLY_BYTES       8

#define TMC2208_SLAVE_ADDR_MIN         0
#define TMC2208_SLAVE_ADDR_MAX         3

/* ── Register addresses (7-bit; OR with TMC2208_WRITE_FLAG for write) ── */

#define TMC2208_REG_GCONF              0x00
#define TMC2208_REG_GSTAT              0x01
#define TMC2208_REG_IFCNT              0x02
#define TMC2208_REG_SLAVECONF          0x03
#define TMC2208_REG_IOIN               0x06
#define TMC2208_REG_IHOLD_IRUN         0x10
#define TMC2208_REG_TPOWERDOWN         0x11
#define TMC2208_REG_TSTEP              0x12
#define TMC2208_REG_TPWMTHRS           0x13
#define TMC2208_REG_VACTUAL            0x22
#define TMC2208_REG_MSCNT              0x6A
#define TMC2208_REG_CHOPCONF           0x6C
#define TMC2208_REG_DRV_STATUS         0x6F
#define TMC2208_REG_PWMCONF            0x70

/* ── GCONF bits ─────────────────────────────────────────────────── */

#define TMC2208_GCONF_I_SCALE_ANALOG   (1u << 0)
#define TMC2208_GCONF_INTERNAL_RSENSE  (1u << 1)
#define TMC2208_GCONF_EN_SPREADCYCLE   (1u << 2)
#define TMC2208_GCONF_SHAFT            (1u << 3)
#define TMC2208_GCONF_INDEX_OTPW       (1u << 4)
#define TMC2208_GCONF_INDEX_STEP       (1u << 5)
#define TMC2208_GCONF_PDN_DISABLE      (1u << 6)
#define TMC2208_GCONF_MSTEP_REG_SELECT (1u << 7)
#define TMC2208_GCONF_MULTISTEP_FILT   (1u << 8)

/* ── DRV_STATUS bits — what we surface to the operator ─────────── */

#define TMC2208_DRV_STATUS_OTPW        (1u << 0)   /* overtemp warning 120°C */
#define TMC2208_DRV_STATUS_OT          (1u << 1)   /* overtemp shutdown 150°C */
#define TMC2208_DRV_STATUS_S2GA        (1u << 2)   /* short to GND A */
#define TMC2208_DRV_STATUS_S2GB        (1u << 3)   /* short to GND B */
#define TMC2208_DRV_STATUS_S2VSA       (1u << 4)
#define TMC2208_DRV_STATUS_S2VSB       (1u << 5)
#define TMC2208_DRV_STATUS_OLA         (1u << 6)   /* open load A */
#define TMC2208_DRV_STATUS_OLB         (1u << 7)
#define TMC2208_DRV_STATUS_STEALTH     (1u << 30)  /* stealthChop active */
#define TMC2208_DRV_STATUS_STST        (1u << 31)  /* standstill */

/* ── Microstep encoding (CHOPCONF MRES[27:24]) ──────────────────── */

#define TMC2208_CHOPCONF_MRES_SHIFT    24
#define TMC2208_CHOPCONF_MRES_MASK     (0xFu << TMC2208_CHOPCONF_MRES_SHIFT)
#define TMC2208_CHOPCONF_INTPOL        (1u << 28)
/* Default chopper settings — needed for any motor motion. TOFF=3,
 * HSTRT=4, HEND=1, TBL=2 (matches TMCStepper defaults). */
#define TMC2208_CHOPCONF_DEFAULT       0x10000053u

/* Convert microstep count (1, 2, 4, ..., 256) to MRES field value. */
static inline uint8_t tmc2208_mres_for_microsteps(uint16_t microsteps)
{
    switch (microsteps) {
    case 256: return 0;
    case 128: return 1;
    case  64: return 2;
    case  32: return 3;
    case  16: return 4;
    case   8: return 5;
    case   4: return 6;
    case   2: return 7;
    case   1: return 8;
    default:  return 8;  /* fall back to full step on invalid input */
    }
}

/* ── IHOLD_IRUN layout ─────────────────────────────────────────── */
/* bits [4:0]   IHOLD     standby current 0..31 */
/* bits [12:8]  IRUN      active current  0..31 */
/* bits [19:16] IHOLDDELAY power-down delay 0..15 (×2^18 t_clk) */

static inline uint32_t tmc2208_pack_ihold_irun(uint8_t ihold, uint8_t irun,
                                                uint8_t iholddelay)
{
    return ((uint32_t)(ihold & 0x1F))
         | (((uint32_t)(irun  & 0x1F)) <<  8)
         | (((uint32_t)(iholddelay & 0x0F)) << 16);
}

/* Compute IRUN/IHOLD register value (0..31) from RMS milliamps.
 * Uses VSENSE=0 (V_FS = 0.325V) for the broader current range.
 * Formula from TMC2208 datasheet §9:
 *   I_RMS = (CS+1)/32 × V_FS / (R_SENSE + 0.02Ω) × 1/sqrt(2)
 * Solving:
 *   CS = I_RMS × 32 × sqrt(2) × (R_SENSE + 0.02) / V_FS  − 1
 *
 * Clamps at 31 (max). Returns 0 if current_ma is 0 or rsense_mohm
 * is implausibly small. The caller is expected to log if clamping
 * happens so the operator knows their requested current was capped. */
static inline uint8_t tmc2208_current_to_cs(uint16_t current_ma,
                                             uint16_t rsense_mohm)
{
    if (current_ma == 0 || rsense_mohm < 10) return 0;
    /* (R_SENSE + 0.02Ω) in milliohms = rsense_mohm + 20.
     * Numerator: I_mA × 32 × 1414 × (rsense + 20) [scaled by 1000].
     * Denominator: V_FS_mV × 1000 = 325 × 1000.
     * Use 64-bit to avoid overflow at high I_mA / rsense combos. */
    uint64_t num = (uint64_t)current_ma * 32u * 1414u * ((uint32_t)rsense_mohm + 20u);
    uint64_t den = (uint64_t)325u * 1000u * 1000u;
    if (den == 0) return 0;
    uint64_t cs_plus_1 = num / den;
    if (cs_plus_1 == 0) return 0;
    if (cs_plus_1 > 32) cs_plus_1 = 32;
    return (uint8_t)(cs_plus_1 - 1);
}

/* ── CRC8 (poly 0x07, reflected input, init 0) ──────────────────── */
/* TMC's bit-by-bit CRC: processes the LSB of each input byte first,
 * shifts the CRC register LEFT. Used over (datagram - 1) bytes
 * (everything except the trailing CRC byte). */
static inline uint8_t tmc2208_crc8(const uint8_t* data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (byte & 0x01)) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc = (uint8_t)(crc << 1);
            }
            byte >>= 1;
        }
    }
    return crc;
}

/* ── Frame builders ─────────────────────────────────────────────── */

static inline void tmc2208_build_write(uint8_t slave_addr, uint8_t reg,
                                         uint32_t value, uint8_t out[8])
{
    out[0] = TMC2208_SYNC;
    out[1] = (uint8_t)(slave_addr & 0x03);
    out[2] = (uint8_t)(reg | TMC2208_WRITE_FLAG);
    out[3] = (uint8_t)((value >> 24) & 0xFF);
    out[4] = (uint8_t)((value >> 16) & 0xFF);
    out[5] = (uint8_t)((value >>  8) & 0xFF);
    out[6] = (uint8_t)(value         & 0xFF);
    out[7] = tmc2208_crc8(out, 7);
}

static inline void tmc2208_build_read_request(uint8_t slave_addr, uint8_t reg,
                                                uint8_t out[4])
{
    out[0] = TMC2208_SYNC;
    out[1] = (uint8_t)(slave_addr & 0x03);
    out[2] = (uint8_t)(reg & 0x7F);
    out[3] = tmc2208_crc8(out, 3);
}

/* Parse an 8-byte read reply. Returns true if sync, master-reply
 * address, register match, and CRC validates. *value receives the
 * 32-bit register value in host byte order. */
static inline bool tmc2208_parse_read_reply(const uint8_t in[8], uint8_t expected_reg,
                                              uint32_t* value)
{
    if (in[0] != TMC2208_SYNC) return false;
    if (in[1] != TMC2208_MASTER_REPLY_ADDR) return false;
    if ((in[2] & 0x7F) != (expected_reg & 0x7F)) return false;
    if (tmc2208_crc8(in, 7) != in[7]) return false;
    *value =  ((uint32_t)in[3] << 24)
            | ((uint32_t)in[4] << 16)
            | ((uint32_t)in[5] <<  8)
            |  (uint32_t)in[6];
    return true;
}

/* ── Virtual GPIO map: 4 axes × 4 channels = 16 channels ─────────
 *
 * Base 348 follows Tic (300..347). pin_config_t.gpio is uint8_t, so
 * 348 truncates to (uint8_t)92 — same trick as Tic uses for the
 * "first channel of first unit" detector in drv_parse_json. */
#define TMC2208_VIRTUAL_GPIO_BASE      348
#define TMC2208_MAX_AXES               4
#define TMC2208_CHANNELS_PER_AXIS      4
#define TMC2208_MAX_CHANNELS           (TMC2208_MAX_AXES * TMC2208_CHANNELS_PER_AXIS)

#define TMC2208_SUB_TARGET_POSITION    0   /* writable, [-1,1] -> ±max_position */
#define TMC2208_SUB_TARGET_VELOCITY    1   /* writable, [-1,1] -> ±max_speed_pps */
#define TMC2208_SUB_CURRENT_POSITION   2   /* read-only, steps */
#define TMC2208_SUB_ERROR_FLAGS        3   /* read-only, DRV_STATUS bits as float */

#ifdef __cplusplus
}
#endif

#endif /* TMC2208_PROTOCOL_H */
