/**
 * SAINT.OS Firmware - Pimoroni Servo 2040 I2C protocol (shared)
 *
 * The Pimoroni Servo 2040 (PIM613/PIM584) is an RP2040-based servo
 * controller: 18 servo outputs, an aggregate current-sense ADC, and 6
 * onboard WS2812 RGB LEDs. It ships bare, so SAINT.OS flashes it with a
 * small fixed image (built on Pimoroni's C++ SDK — see
 * firmware/pimoroni_servo2040/) that acts as an I2C TARGET speaking the
 * register map defined here. That image is provisioned once and is NOT
 * part of SAINT.OS's OTA/firmware pipeline; from SAINT.OS's perspective
 * the board is an opaque peripheral driven by one of our controller
 * nodes (Teensy 4.1 / RP2040 / Raspberry Pi) as the I2C MASTER.
 *
 * This header is the single source of truth for the wire contract. Both
 * the board firmware and every host driver (shared/src/
 * pimoroni_servo2040_driver.c and firmware/raspberrypi/.../
 * pimoroni_servo2040.py) reference it so the two ends can never drift.
 *
 * ── Link ────────────────────────────────────────────────────────────
 *   I2C on the board's Qwiic / STEMMA-QT connector: GP20 = SDA,
 *   GP21 = SCL (i2c0). Board is the target at
 *   PIMORONI_SERVO2040_I2C_ADDR; the SAINT.OS controller is the master.
 *   All 18 servo pins stay free for servos.
 *
 * ── Register access ─────────────────────────────────────────────────
 *   Standard I2C register semantics. WRITE: master sends [reg][data...].
 *   READ: master sends [reg] (no stop), repeated-START, reads N bytes.
 *   Multi-byte values are little-endian.
 *
 * ── Feature parity with the Maestro ─────────────────────────────────
 *   Servo EXTENTS live host-side (the driver maps normalized −1..+1 to a
 *   pulse and clamps), exactly as the Maestro driver holds per-channel
 *   min/max. HOME positions mirror the Maestro's EEPROM HomeMode=Goto:
 *   the host writes SERVO_HOME[ch] then REG_COMMIT, and the board
 *   persists home pulses to its own flash and drives them on power-on —
 *   so the rig comes up at known positions before the host even
 *   connects. The driver also re-homes on connect (belt-and-suspenders).
 */

#ifndef SAINT_PIMORONI_SERVO2040_PROTOCOL_H
#define SAINT_PIMORONI_SERVO2040_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

/* ── Firmware channel-slab layout (host driver side) ─────────────────
 * The host driver claims a contiguous virtual-GPIO slab. Servos occupy
 * the first 18 channels; the 6 LED-color channels follow. Status/current
 * telemetry is emitted by peripheral_id/channel_id string (not a slab
 * index) so it does NOT consume channel slots.
 *
 * Base 168 is deliberately BELOW the Maestro's 200 and above every real
 * GPIO on our controllers (Teensy 4.1 tops out at ~54). base +
 * channel_count (168 + 24 = 192) stays under 256 so the uint8_t
 * pin_config_t.gpio never wraps — unlike the >255 bases (Tic 300,
 * Kangaroo 364) that alias into the low range. */
#define PIMORONI_SERVO2040_VIRTUAL_GPIO_BASE  168
#define PIMORONI_SERVO2040_NUM_SERVOS         18
#define PIMORONI_SERVO2040_NUM_LEDS           6
/* Servos are channels 0..17; LEDs are channels 18..23. */
#define PIMORONI_SERVO2040_LED_CHANNEL_BASE   PIMORONI_SERVO2040_NUM_SERVOS
#define PIMORONI_SERVO2040_CHANNELS_PER_INSTANCE \
    (PIMORONI_SERVO2040_NUM_SERVOS + PIMORONI_SERVO2040_NUM_LEDS)   /* 24 */

/* ── Link parameters ─────────────────────────────────────────────── */
#define PIMORONI_SERVO2040_I2C_ADDR       0x30   /* board's I2C target address */
#define PIMORONI_SERVO2040_I2C_BAUD       400000u /* 400 kHz fast-mode */
#define PIMORONI_SERVO2040_WHOAMI_MAGIC   0x53   /* 'S' — identifies our image */
#define PIMORONI_SERVO2040_FW_VERSION     1

/* Board disables all servos if no I2C heartbeat within this window. The
 * host driver writes REG_HEARTBEAT well inside it. */
#define PIMORONI_SERVO2040_HEARTBEAT_TIMEOUT_MS   1000u
#define PIMORONI_SERVO2040_PING_MS                250u   /* host keepalive cadence */
#define PIMORONI_SERVO2040_POLL_MS                200u   /* host telemetry poll cadence */
#define PIMORONI_SERVO2040_TELEM_MS               100u   /* board current-sample cadence */

/* ── Servo pulse envelope (microseconds) — SDK calibration ───────── */
#define PIMORONI_SERVO2040_MIN_PULSE_US    500u
#define PIMORONI_SERVO2040_MID_PULSE_US    1500u
#define PIMORONI_SERVO2040_MAX_PULSE_US    2500u
#define PIMORONI_SERVO2040_HARD_MIN_US     400u   /* SDK LOWER_HARD_LIMIT */
#define PIMORONI_SERVO2040_HARD_MAX_US     2600u  /* SDK UPPER_HARD_LIMIT */

/* ── Register map ────────────────────────────────────────────────────
 * All multi-byte values little-endian. (r) = master reads, (w) = writes. */
#define PIMORONI_SERVO2040_REG_WHOAMI      0x00   /* (r) 1B  = WHOAMI_MAGIC        */
#define PIMORONI_SERVO2040_REG_FW_VERSION  0x01   /* (r) 1B  = FW_VERSION          */
#define PIMORONI_SERVO2040_REG_STATUS      0x02   /* (r) 1B  flags bitmask         */
#define PIMORONI_SERVO2040_REG_CURRENT_MA  0x04   /* (r) 2B  aggregate current mA  */

/* Runtime servo targets: 18 × uint16 pulse µs (0 = disable/relax). */
#define PIMORONI_SERVO2040_REG_SERVO_TARGET_BASE  0x10   /* (w) reg = base + ch*2 */
/* Persisted power-on home pulses: 18 × uint16 µs (0 = not homed at boot). */
#define PIMORONI_SERVO2040_REG_SERVO_HOME_BASE    0x40   /* (w) reg = base + ch*2 */
/* Onboard LEDs: 6 × {R,G,B}. */
#define PIMORONI_SERVO2040_REG_LED_BASE           0x70   /* (w) reg = base + idx*3 */
#define PIMORONI_SERVO2040_REG_BRIGHTNESS         0x90   /* (w) 1B  0..255 */

#define PIMORONI_SERVO2040_REG_COMMIT      0xF0   /* (w) 1B: write 1 → persist HOME to flash */
#define PIMORONI_SERVO2040_REG_HEARTBEAT   0xF1   /* (w) 1B: feed failsafe watchdog          */
#define PIMORONI_SERVO2040_REG_ESTOP       0xF2   /* (w) 1B: write 1 → disable all servos    */

/* Largest register payload we transfer in one transaction (a home/target
 * write is reg + 2 data bytes; an LED write is reg + 3). */
#define PIMORONI_SERVO2040_MAX_XFER        4u

/* ── STATUS flag bits (REG_STATUS) ───────────────────────────────── */
#define PIMORONI_SERVO2040_FLAG_OVERCURRENT  0x01  /* current over soft limit          */
#define PIMORONI_SERVO2040_FLAG_FAILSAFE     0x02  /* servos disabled: no host heartbeat */
#define PIMORONI_SERVO2040_FLAG_HOMED        0x04  /* power-on home applied from flash   */

#ifdef __cplusplus
}
#endif

#endif /* SAINT_PIMORONI_SERVO2040_PROTOCOL_H */
