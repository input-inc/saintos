/**
 * SAINT.OS — Pimoroni Servo 2040 board firmware (I2C target)
 *
 * A small FIXED image that turns a Pimoroni Servo 2040 into an I2C
 * peripheral for SAINT.OS. It is provisioned onto the board ONCE (manual
 * drag-and-drop of the .uf2) and is NOT part of SAINT.OS's OTA/CI
 * firmware pipeline — from SAINT.OS's perspective the board arrives
 * "pre-flashed and ready to communicate."
 *
 * Role: I2C TARGET on the Qwiic/STEMMA-QT connector (i2c0, GP20 SDA /
 * GP21 SCL). The SAINT.OS controller node (Teensy 4.1 / RP2040 / Pi) is
 * the master. The register map is the shared contract in
 * ../shared/include/pimoroni_servo2040_protocol.h.
 *
 * Responsibilities:
 *   - 18 servo outputs via Pimoroni ServoCluster (pio0).
 *   - 6 onboard WS2812 RGB LEDs via the plasma::WS2812 driver (pio1).
 *   - Aggregate current sense via Analog + AnalogMux (SHARED_ADC).
 *   - Persist per-servo HOME pulses to flash and drive them on power-on
 *     (mirrors the Pololu Maestro's EEPROM HomeMode=Goto) so the rig
 *     comes up at known positions before the host connects.
 *   - Failsafe: relax all servos if the host heartbeat stops.
 *
 * Design note: the I2C IRQ is a dumb register-file mem-slave (fast,
 * timing-safe); ALL application logic runs in the main loop by diffing
 * the register file. This decouples bus timing from servo/LED/flash work.
 */

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"

#include "servo2040.hpp"
#include "analog.hpp"
#include "analogmux.hpp"
#include "button.hpp"

#include <cstring>

extern "C" {
#include "pimoroni_servo2040_protocol.h"
}

using namespace servo;
using namespace pimoroni;
using namespace plasma;

// ── Peripherals ──────────────────────────────────────────────────────
static ServoCluster servos(pio0, 0, servo2040::SERVO_1, servo2040::NUM_SERVOS);
static WS2812 leds(servo2040::NUM_LEDS, pio1, 0, servo2040::LED_DATA);
static Analog cur_adc(servo2040::SHARED_ADC, servo2040::CURRENT_GAIN,
                      servo2040::SHUNT_RESISTOR, servo2040::CURRENT_OFFSET);
static AnalogMux mux(servo2040::ADC_ADDR_0, servo2040::ADC_ADDR_1,
                     servo2040::ADC_ADDR_2, PIN_UNUSED, servo2040::SHARED_ADC);

// Soft over-current limit (amps) → OVERCURRENT status flag. The Servo
// 2040's 3 mΩ shunt / 69× amp tops out ~ a few amps; this is advisory
// telemetry, not a hardware cutoff.
static constexpr float OVERCURRENT_LIMIT_A = 8.0f;

// ── I2C register file (the wire contract) ────────────────────────────
static volatile uint8_t g_regs[256];
static volatile uint8_t g_reg_ptr = 0;
static volatile bool    g_addr_seen = false;   // first byte of a write = ptr

// ── Flash-persisted home config ──────────────────────────────────────
// Stored in the last 4 KB sector of flash: a magic + 18×uint16 home
// pulses. Loaded at boot to seed power-on homing.
#define HOME_FLASH_OFFSET   (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define HOME_FLASH_MAGIC    0x53484F4Du   // "SHOM"

struct HomeBlob {
    uint32_t magic;
    uint16_t home_us[PIMORONI_SERVO2040_NUM_SERVOS];
};

static uint16_t reg_u16(uint8_t reg) {
    return (uint16_t)g_regs[reg] | ((uint16_t)g_regs[reg + 1] << 8);
}
static void set_reg_u16(uint8_t reg, uint16_t v) {
    g_regs[reg] = (uint8_t)(v & 0xFF);
    g_regs[reg + 1] = (uint8_t)((v >> 8) & 0xFF);
}

// ── I2C target IRQ: raw register-file mem-slave ──────────────────────
static void i2c_target_irq() {
    i2c_hw_t* hw = i2c_get_hw(i2c0);
    uint32_t stat = hw->intr_stat;

    // Master issued a read request: return the byte at the pointer and
    // auto-increment.
    if (stat & I2C_IC_INTR_STAT_R_RD_REQ_BITS) {
        hw->data_cmd = g_regs[g_reg_ptr];
        g_reg_ptr++;
        hw->clr_rd_req;   // ack
    }

    // A STOP/RESTART resets the "next byte is a register pointer" latch.
    if (stat & I2C_IC_INTR_STAT_R_RESTART_DET_BITS) { hw->clr_restart_det; }
    if (stat & I2C_IC_INTR_STAT_R_STOP_DET_BITS)    { hw->clr_stop_det; g_addr_seen = false; }

    // Master wrote a byte.
    while (hw->status & I2C_IC_STATUS_RFNE_BITS) {
        uint8_t b = (uint8_t)hw->data_cmd;
        if (!g_addr_seen) {
            g_reg_ptr = b;          // first byte = register pointer
            g_addr_seen = true;
        } else {
            g_regs[g_reg_ptr] = b;  // subsequent bytes = data (auto-inc)
            g_reg_ptr++;
        }
    }
}

static void i2c_target_init() {
    i2c_init(i2c0, PIMORONI_SERVO2040_I2C_BAUD);
    i2c_set_slave_mode(i2c0, true, PIMORONI_SERVO2040_I2C_ADDR);
    gpio_set_function(servo2040::I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(servo2040::I2C_SCL, GPIO_FUNC_I2C);

    i2c_hw_t* hw = i2c_get_hw(i2c0);
    // Enable RX-full, read-request, stop/restart detect interrupts.
    hw->intr_mask = I2C_IC_INTR_MASK_M_RX_FULL_BITS
                  | I2C_IC_INTR_MASK_M_RD_REQ_BITS
                  | I2C_IC_INTR_MASK_M_STOP_DET_BITS
                  | I2C_IC_INTR_MASK_M_RESTART_DET_BITS;
    irq_set_exclusive_handler(I2C0_IRQ, i2c_target_irq);
    irq_set_enabled(I2C0_IRQ, true);
}

// ── Flash-persisted home ─────────────────────────────────────────────
static void home_load(uint16_t out[PIMORONI_SERVO2040_NUM_SERVOS]) {
    const HomeBlob* blob = (const HomeBlob*)(XIP_BASE + HOME_FLASH_OFFSET);
    if (blob->magic == HOME_FLASH_MAGIC) {
        memcpy(out, blob->home_us, sizeof(blob->home_us));
    } else {
        memset(out, 0, sizeof(uint16_t) * PIMORONI_SERVO2040_NUM_SERVOS);
    }
}

static void home_save(const uint16_t home_us[PIMORONI_SERVO2040_NUM_SERVOS]) {
    HomeBlob blob;
    blob.magic = HOME_FLASH_MAGIC;
    memcpy(blob.home_us, home_us, sizeof(blob.home_us));

    // Diff-check to avoid needless flash wear.
    const HomeBlob* cur = (const HomeBlob*)(XIP_BASE + HOME_FLASH_OFFSET);
    if (cur->magic == HOME_FLASH_MAGIC &&
        memcmp(cur->home_us, home_us, sizeof(blob.home_us)) == 0) {
        return;
    }

    uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));
    memcpy(page, &blob, sizeof(blob));

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(HOME_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(HOME_FLASH_OFFSET, page, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
}

// ── Servo helpers ────────────────────────────────────────────────────
static void servo_apply(uint8_t ch, uint16_t pulse_us) {
    if (pulse_us == 0) {
        servos.disable(ch);
    } else {
        if (pulse_us < PIMORONI_SERVO2040_HARD_MIN_US) pulse_us = PIMORONI_SERVO2040_HARD_MIN_US;
        if (pulse_us > PIMORONI_SERVO2040_HARD_MAX_US) pulse_us = PIMORONI_SERVO2040_HARD_MAX_US;
        servos.pulse(ch, (float)pulse_us);   // enables the channel
    }
}

int main() {
    servos.init();
    leds.start();

    // Seed the register file with identity + defaults.
    memset((void*)g_regs, 0, sizeof(g_regs));
    g_regs[PIMORONI_SERVO2040_REG_WHOAMI]     = PIMORONI_SERVO2040_WHOAMI_MAGIC;
    g_regs[PIMORONI_SERVO2040_REG_FW_VERSION] = PIMORONI_SERVO2040_FW_VERSION;
    g_regs[PIMORONI_SERVO2040_REG_BRIGHTNESS] = 255;

    // Load persisted home + apply on power-on (before the host connects).
    uint16_t home_us[PIMORONI_SERVO2040_NUM_SERVOS];
    home_load(home_us);
    uint16_t last_target[PIMORONI_SERVO2040_NUM_SERVOS];
    for (uint8_t ch = 0; ch < PIMORONI_SERVO2040_NUM_SERVOS; ch++) {
        set_reg_u16(PIMORONI_SERVO2040_REG_SERVO_HOME_BASE + ch * 2, home_us[ch]);
        set_reg_u16(PIMORONI_SERVO2040_REG_SERVO_TARGET_BASE + ch * 2, home_us[ch]);
        servo_apply(ch, home_us[ch]);
        last_target[ch] = home_us[ch];
    }
    g_regs[PIMORONI_SERVO2040_REG_STATUS] |= PIMORONI_SERVO2040_FLAG_HOMED;

    i2c_target_init();

    uint8_t  last_bright = 255;
    uint8_t  last_led[PIMORONI_SERVO2040_NUM_LEDS][3] = {{0}};
    bool     leds_dirty = true;
    bool     host_seen = false;
    uint32_t last_heartbeat_ms = 0;
    uint32_t last_telem_ms = 0;

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // Heartbeat (feed failsafe).
        if (g_regs[PIMORONI_SERVO2040_REG_HEARTBEAT]) {
            g_regs[PIMORONI_SERVO2040_REG_HEARTBEAT] = 0;
            last_heartbeat_ms = now;
            host_seen = true;
            g_regs[PIMORONI_SERVO2040_REG_STATUS] &= ~PIMORONI_SERVO2040_FLAG_FAILSAFE;
        }

        // E-stop: relax all servos immediately.
        if (g_regs[PIMORONI_SERVO2040_REG_ESTOP]) {
            g_regs[PIMORONI_SERVO2040_REG_ESTOP] = 0;
            for (uint8_t ch = 0; ch < PIMORONI_SERVO2040_NUM_SERVOS; ch++) {
                servos.disable(ch);
                last_target[ch] = 0;
                set_reg_u16(PIMORONI_SERVO2040_REG_SERVO_TARGET_BASE + ch * 2, 0);
            }
        }

        // Commit: persist the current HOME registers to flash.
        if (g_regs[PIMORONI_SERVO2040_REG_COMMIT]) {
            g_regs[PIMORONI_SERVO2040_REG_COMMIT] = 0;
            uint16_t to_save[PIMORONI_SERVO2040_NUM_SERVOS];
            for (uint8_t ch = 0; ch < PIMORONI_SERVO2040_NUM_SERVOS; ch++)
                to_save[ch] = reg_u16(PIMORONI_SERVO2040_REG_SERVO_HOME_BASE + ch * 2);
            home_save(to_save);
        }

        // Apply servo targets that changed.
        bool failsafe = host_seen &&
            (now - last_heartbeat_ms > PIMORONI_SERVO2040_HEARTBEAT_TIMEOUT_MS);
        if (failsafe) {
            g_regs[PIMORONI_SERVO2040_REG_STATUS] |= PIMORONI_SERVO2040_FLAG_FAILSAFE;
        }
        for (uint8_t ch = 0; ch < PIMORONI_SERVO2040_NUM_SERVOS; ch++) {
            uint16_t t = failsafe ? 0
                       : reg_u16(PIMORONI_SERVO2040_REG_SERVO_TARGET_BASE + ch * 2);
            if (t != last_target[ch]) {
                servo_apply(ch, t);
                last_target[ch] = t;
            }
        }

        // Apply LED colors / brightness that changed.
        for (uint8_t i = 0; i < PIMORONI_SERVO2040_NUM_LEDS; i++) {
            uint8_t r = g_regs[PIMORONI_SERVO2040_REG_LED_BASE + i * 3 + 0];
            uint8_t g = g_regs[PIMORONI_SERVO2040_REG_LED_BASE + i * 3 + 1];
            uint8_t b = g_regs[PIMORONI_SERVO2040_REG_LED_BASE + i * 3 + 2];
            if (r != last_led[i][0] || g != last_led[i][1] || b != last_led[i][2]) {
                leds.set_rgb(i, r, g, b);
                last_led[i][0] = r; last_led[i][1] = g; last_led[i][2] = b;
                leds_dirty = true;
            }
        }
        uint8_t bright = g_regs[PIMORONI_SERVO2040_REG_BRIGHTNESS];
        if (bright != last_bright) {
            leds.set_brightness(bright);
            last_bright = bright;
            leds_dirty = true;
        }
        if (leds_dirty) { leds.update(); leds_dirty = false; }

        // Telemetry: sample aggregate current and publish it + flags.
        if (now - last_telem_ms >= PIMORONI_SERVO2040_TELEM_MS) {
            last_telem_ms = now;
            mux.select(servo2040::CURRENT_SENSE_ADDR);
            float amps = cur_adc.read_current();
            if (amps < 0.0f) amps = 0.0f;
            uint32_t ma = (uint32_t)(amps * 1000.0f + 0.5f);
            if (ma > 0xFFFF) ma = 0xFFFF;
            set_reg_u16(PIMORONI_SERVO2040_REG_CURRENT_MA, (uint16_t)ma);
            if (amps > OVERCURRENT_LIMIT_A)
                g_regs[PIMORONI_SERVO2040_REG_STATUS] |= PIMORONI_SERVO2040_FLAG_OVERCURRENT;
            else
                g_regs[PIMORONI_SERVO2040_REG_STATUS] &= ~PIMORONI_SERVO2040_FLAG_OVERCURRENT;
        }

        sleep_ms(2);
    }
}
