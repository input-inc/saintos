/**
 * Host-runnable regression test for the boot-reload path in
 * firmware/rp2040/src/pin_config.c.
 *
 * The bug this pins down (2026-09-26, Maestro "poses land in the wrong
 * place until I hit Sync"):
 *
 *   pin_config_load() ran, in order,
 *     1. pin_config_set() per stored pin  — peripheral modes get NO
 *        params restored from flash (the loader only unpacks PWM and
 *        SERVO params), so the pin table holds zeros on RP2040 and
 *        set_defaults' output on Teensy;
 *     2. drv->load_config()               — the driver restores its
 *        REAL per-channel config from the flash blob;
 *     3. pin_config_apply_hardware()      — whose peripheral `default:`
 *        branch called drv->apply_config() with the step-1 pin table,
 *        overwriting everything step 2 just restored.
 *
 *   On the Head Node's Maestro that meant every channel came back from
 *   a reboot on the 992-2000 us / 1500 us default envelope instead of
 *   its calibrated one, so a pose resolved to the wrong pulse widths.
 *   A config Sync re-pushed the real values and "fixed" it until the
 *   next boot.
 *
 * The fix is pin_config_apply_hardware_from_flash(), which runs the
 * same physical-pin setup but skips the peripheral-driver delegation.
 * case_boot_reload_keeps_flash_config covers the whole load path;
 * case_from_flash_skips_apply_config covers the two entry points
 * directly, including the Teensy's "defaults in the pin table" shape
 * (Teensy's pin_config.cpp can't be compiled host-side — it is Arduino
 * bound — but it took the identical one-line change at the identical
 * call site).
 *
 * Built with -DSIMULATION=1 against tests/stubs/, which supplies the
 * handful of pico-sdk entry points pin_config.c reaches for.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

/* ── saint_log stub — the unit under test logs freely; we don't
 *    assert on it here, so swallow the lines. ─────────────────────── */
#define SAINT_LOG_H
static void saint_log_publish(const char* level, const char* fmt, ...)
{
    (void)level; (void)fmt;
}

#include "pin_config.h"
#include "flash_storage.h"
#include "saint_node.h"
#include "peripheral_driver.h"
#include "pin_control_types.h"
#include "neopixel_strip.h"

/* ── Fake peripheral driver ──────────────────────────────────────────
 * Stands in for the Maestro: it owns a per-channel min_pulse_us that
 * load_config restores from flash and apply_config overwrites from the
 * pin table. That is the exact pair of writes whose ordering broke. */

#define FAKE_BASE      200   /* MAESTRO_VIRTUAL_GPIO_BASE */
#define FAKE_CHANNELS  4
#define FAKE_MODE      PIN_MODE_MAESTRO_SERVO

/* Mirrors MAESTRO_DEFAULT_MIN_PULSE — what set_defaults writes, and
 * the value the Head Node's channels wrongly came up holding. */
#define FAKE_DEFAULT_MIN   992
/* A calibrated extent, as saved by a real Sync. Head Node ch0
 * ("Right Iris") really was 1010. */
#define FAKE_FLASH_MIN     1010

static uint16_t fake_state[FAKE_CHANNELS];      /* the driver's live config */
static int      fake_apply_config_calls;
static int      fake_load_config_calls;

static void fake_set_defaults(uint8_t channel, pin_config_t* config)
{
    (void)channel;
    if (!config) return;
    config->params.maestro.min_pulse_us = FAKE_DEFAULT_MIN;
}

static bool fake_apply_config(uint8_t channel, const pin_config_t* config)
{
    if (channel >= FAKE_CHANNELS || !config) return false;
    fake_apply_config_calls++;
    fake_state[channel] = config->params.maestro.min_pulse_us;
    return true;
}

static bool fake_load_config(const void* storage)
{
    const flash_storage_data_t* s = (const flash_storage_data_t*)storage;
    if (!s) return false;
    fake_load_config_calls++;
    for (uint8_t ch = 0; ch < FAKE_CHANNELS; ch++) {
        fake_state[ch] = s->maestro_config.channels[ch].min_pulse_us;
    }
    return true;
}

static const peripheral_driver_t fake_driver = {
    .name                 = "fake",
    .mode_string          = "maestro_servo",
    .pin_mode             = FAKE_MODE,
    .virtual_gpio_base    = FAKE_BASE,
    .channel_count        = FAKE_CHANNELS,
    .channels_per_instance = FAKE_CHANNELS,
    .set_defaults         = fake_set_defaults,
    .apply_config         = fake_apply_config,
    .load_config          = fake_load_config,
};

/* ── Peripheral manager stubs — one registered driver, ours. ─────── */

uint8_t peripheral_get_count(void) { return 1; }

const peripheral_driver_t* peripheral_get(uint8_t index)
{
    return index == 0 ? &fake_driver : NULL;
}

const peripheral_driver_t* peripheral_find_by_mode(pin_mode_t mode)
{
    return mode == FAKE_MODE ? &fake_driver : NULL;
}

const peripheral_driver_t* peripheral_find_by_mode_string(const char* mode_str)
{
    if (mode_str && strcmp(mode_str, fake_driver.mode_string) == 0) {
        return &fake_driver;
    }
    return NULL;
}

uint8_t peripheral_gpio_to_channel(const peripheral_driver_t* drv, uint16_t gpio)
{
    if (!drv) return 0;
    return (uint8_t)(gpio - drv->virtual_gpio_base);
}

bool peripheral_is_virtual_gpio(uint16_t gpio)
{
    return gpio >= FAKE_BASE;
}

/* ── Remaining externals pin_config.c links against ──────────────── */

saint_node_config_t g_node;

static flash_storage_data_t fake_flash;
static bool                 fake_flash_valid;

bool flash_storage_load(flash_storage_data_t* data)
{
    if (!fake_flash_valid || !data) return false;
    memcpy(data, &fake_flash, sizeof(*data));
    return true;
}

bool flash_storage_save(const flash_storage_data_t* data)
{
    if (!data) return false;
    memcpy(&fake_flash, data, sizeof(fake_flash));
    fake_flash_valid = true;
    return true;
}

void flash_storage_from_node(flash_storage_data_t* data,
                             const saint_node_config_t* node)
{
    (void)data; (void)node;
}

bool pin_control_drive_servo_pulse(uint8_t gpio, uint16_t pulse_us)
{
    (void)gpio; (void)pulse_us;
    return true;
}

void neopixel_strip_reset(void) { }

bool neopixel_strip_add(const char* id, uint8_t pin, uint16_t count)
{
    (void)id; (void)pin; (void)count;
    return true;
}

/* ── Unit under test ─────────────────────────────────────────────── */

#include "../src/pin_config.c"

/* ── Test plumbing (same shape as the sibling driver tests) ──────── */

static int fail_count = 0;

#define EXPECT(cond, what)                                          \
    do {                                                            \
        if (cond) {                                                 \
            printf("  ok   %s\n", (what));                          \
        } else {                                                    \
            printf("  FAIL %s  (%s:%d)\n", (what), __FILE__, __LINE__); \
            fail_count++;                                           \
        }                                                           \
    } while (0)

/* Seed flash with FAKE_CHANNELS peripheral pins and a calibrated
 * per-channel config — i.e. what a node holds after the operator has
 * synced real extents and the node has saved them. */
static void seed_flash(void)
{
    memset(&fake_flash, 0, sizeof(fake_flash));
    fake_flash.magic   = FLASH_STORAGE_MAGIC;
    fake_flash.version = FLASH_STORAGE_VERSION;

    fake_flash.pin_config.version   = PIN_CONFIG_VERSION;
    fake_flash.pin_config.pin_count = FAKE_CHANNELS;
    for (uint8_t i = 0; i < FAKE_CHANNELS; i++) {
        fake_flash.pin_config.pins[i].gpio = (uint8_t)(FAKE_BASE + i);
        fake_flash.pin_config.pins[i].mode = (uint8_t)FAKE_MODE;
        strncpy(fake_flash.pin_config.pins[i].logical_name, "maestro-1",
                FLASH_PIN_CONFIG_MAX_NAME_LEN - 1);
    }

    fake_flash.maestro_config.channel_count  = FAKE_CHANNELS;
    fake_flash.maestro_config.transport_mode = FLASH_MAESTRO_TRANSPORT_UART;
    for (uint8_t ch = 0; ch < FAKE_CHANNELS; ch++) {
        fake_flash.maestro_config.channels[ch].min_pulse_us = FAKE_FLASH_MIN;
    }
    fake_flash_valid = true;
}

static void reset_all(void)
{
    pin_config_reset();
    memset(fake_state, 0, sizeof(fake_state));
    fake_apply_config_calls = 0;
    fake_load_config_calls  = 0;
}

/* The whole boot path: flash → pin table → driver restore → apply. */
static void case_boot_reload_keeps_flash_config(void)
{
    printf("case_boot_reload_keeps_flash_config\n");
    reset_all();
    seed_flash();

    EXPECT(pin_config_load(), "pin_config_load returned true");
    EXPECT(fake_load_config_calls == 1,
           "driver load_config ran once (restored from flash)");
    EXPECT(fake_apply_config_calls == 0,
           "driver apply_config NOT called on the boot-reload path");
    EXPECT(fake_state[0] == FAKE_FLASH_MIN,
           "channel 0 still holds the calibrated flash value");
    EXPECT(fake_state[FAKE_CHANNELS - 1] == FAKE_FLASH_MIN,
           "last channel still holds the calibrated flash value");
}

/* The two entry points, head to head. The `from_flash` one must skip
 * the peripheral delegation even when the pin table looks perfectly
 * plausible (Teensy leaves set_defaults' output there); the ordinary
 * one must still delegate, or a dashboard Sync would stop applying. */
static void case_from_flash_skips_apply_config(void)
{
    printf("case_from_flash_skips_apply_config\n");
    reset_all();

    /* Build the pin table the way boot leaves it on Teensy: entries
     * present, params holding set_defaults' "defaults". */
    for (uint8_t i = 0; i < FAKE_CHANNELS; i++) {
        uint8_t gpio = (uint8_t)(FAKE_BASE + i);
        EXPECT(pin_config_set(gpio, FAKE_MODE, "maestro-1"),
               "pin_config_set accepted a peripheral pin");
        /* Exactly what Teensy's pin_config_set does for peripheral
         * modes, and the whole reason the clobber had teeth: the pin
         * table ends up holding plausible-looking defaults that
         * apply_config is happy to push. */
        pin_config_t* pcfg = find_or_create_config(gpio);
        fake_set_defaults(i, pcfg);
    }

    /* Driver state as drv_load would have left it. */
    for (uint8_t ch = 0; ch < FAKE_CHANNELS; ch++) {
        fake_state[ch] = FAKE_FLASH_MIN;
    }

    pin_config_apply_hardware_from_flash();
    EXPECT(fake_apply_config_calls == 0,
           "from_flash variant skipped the peripheral delegation");
    EXPECT(fake_state[0] == FAKE_FLASH_MIN,
           "from_flash variant left the restored config alone");

    /* The sync path must be unchanged — this is the regression guard
     * on the fix itself. */
    pin_config_apply_hardware();
    EXPECT(fake_apply_config_calls == FAKE_CHANNELS,
           "ordinary variant still delegates to apply_config");
    EXPECT(fake_state[0] == FAKE_DEFAULT_MIN,
           "ordinary variant pushed the pin table's params through");
}

int main(void)
{
    printf("=== pin_config boot-reload tests ===\n");
    case_boot_reload_keeps_flash_config();
    case_from_flash_skips_apply_config();

    if (fail_count == 0) {
        printf("\nAll pin_config boot-reload tests passed.\n");
        return 0;
    }
    printf("\n%d check(s) FAILED.\n", fail_count);
    return 1;
}
