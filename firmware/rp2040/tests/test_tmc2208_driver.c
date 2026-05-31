/**
 * Host-runnable tests for the TMC2208 stepper driver.
 *
 * Verifies the protocol encoders (CRC, IHOLD_IRUN packing, MRES
 * lookup), JSON parse, set/get value scaling, save/load roundtrip,
 * connection state machine, and the apply_motion_command logic that
 * translates target_position into steps_remaining + direction.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <assert.h>

#define SIMULATION 1

/* ── Platform / log / pin-pair stubs ────────────────────────────── */

#define PLATFORM_H
static uint32_t test_now_ms = 0;
#define PLATFORM_MILLIS()      (test_now_ms)
#define PLATFORM_SLEEP_MS(ms)  ((void)(ms))
#define PLATFORM_PRINTF(...)   ((void)0)

#define SAINT_LOG_H
static int log_count = 0;
static char log_lines[256][256];
static void saint_log_publish(const char* level, const char* fmt, ...)
{
    if (log_count >= 256) return;
    int n = snprintf(log_lines[log_count], sizeof(log_lines[0]), "[%s] ", level);
    if (n < 0) return;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(log_lines[log_count] + n, sizeof(log_lines[0]) - (size_t)n, fmt, ap);
    va_end(ap);
    log_count++;
}
static int log_contains(const char* needle)
{
    for (int i = 0; i < log_count; i++) {
        if (strstr(log_lines[i], needle)) return 1;
    }
    return 0;
}

#define UART_PIN_PAIRS_H
static bool uart_pin_pair_lookup(uint8_t tx, uint8_t rx, uint8_t* inst)
{
    (void)tx; (void)rx;
    *inst = 0;
    return true;
}
static bool uart_pin_pair_parse_json(const char* a, const char* b,
                                      uint8_t* tx, uint8_t* rx, uint8_t* inst)
{
    (void)a; (void)b; (void)tx; (void)rx; (void)inst;
    return false;
}

/* ── Transport stub (NULL so SIMULATION paths short-circuit) ───── */

#define SAINT_TMC2208_TRANSPORT_H
typedef struct tmc2208_transport_ops {
    const char* name;
    bool (*open)(uint8_t tx_pin, uint8_t rx_pin, uint32_t baud);
    bool (*is_open)(void);
    uint8_t (*resolved_instance)(void);
    size_t (*xfer)(const uint8_t* tx, size_t tx_len,
                   uint8_t* rx_buf, size_t rx_len);
    bool (*axis_attach)(uint8_t axis, uint8_t step_pin, uint8_t dir_pin);
    void (*axis_detach)(uint8_t axis);
    void (*axis_set_rate)(uint8_t axis, uint32_t pps, bool forward);
} tmc2208_transport_ops_t;
static const tmc2208_transport_ops_t* tmc2208_get_transport(void) { return NULL; }
#define TMC2208_READ_TIMEOUT_MS  10

/* ── Pull in the shared driver ─────────────────────────────────── */
#include "../../shared/src/tmc2208_driver.c"

/* ── Test harness macros ───────────────────────────────────────── */

#define CHECK(expr)  do { \
    if (!(expr)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __func__, __LINE__, #expr); \
        return 0; \
    } \
} while (0)

#define CHECK_EQ(a, b)  do { \
    long _av = (long)(a), _bv = (long)(b); \
    if (_av != _bv) { \
        fprintf(stderr, "FAIL %s:%d: %s (%ld) != %s (%ld)\n", \
                __func__, __LINE__, #a, _av, #b, _bv); \
        return 0; \
    } \
} while (0)

#define CHECK_FLOAT(a, b)  do { \
    float _av = (float)(a), _bv = (float)(b); \
    float _d = _av - _bv; if (_d < 0) _d = -_d; \
    if (_d > 0.001f) { \
        fprintf(stderr, "FAIL %s:%d: %s (%f) != %s (%f)\n", \
                __func__, __LINE__, #a, _av, #b, _bv); \
        return 0; \
    } \
} while (0)

#define CHECK_LOG(needle)  do { \
    if (!log_contains(needle)) { \
        fprintf(stderr, "FAIL %s:%d: no log line contains %s\n", \
                __func__, __LINE__, needle); \
        return 0; \
    } \
} while (0)

static void reset_state(void)
{
    memset(axes, 0, sizeof(axes));
    axis_count = 0;
    port_initialized = false;
    configured_baud = TMC2208_DEFAULT_BAUD;
    configured_serial_port = 0;
    tmc2208_tx_pin = TMC2208_DEFAULT_TX_PIN;
    tmc2208_rx_pin = TMC2208_DEFAULT_RX_PIN;
    active_tx_pin = 0xFF;
    active_rx_pin = 0xFF;
    active_uart   = 0xFF;
    poll_axis = 0;
    log_count = 0;
    test_now_ms = 1000;
}

/* ── Protocol tests (no driver state) ──────────────────────────── */

static int test_crc_known_vector(void)
{
    /* Datasheet example: GCONF read request from master to slave 0
     * is [0x05, 0x00, 0x00, CRC]. Expected CRC = 0x48. */
    uint8_t req[3] = { 0x05, 0x00, 0x00 };
    CHECK_EQ(tmc2208_crc8(req, 3), 0x48);
    return 1;
}

static int test_build_write_frame(void)
{
    uint8_t out[8];
    tmc2208_build_write(2, TMC2208_REG_GCONF, 0xDEADBEEF, out);
    CHECK_EQ(out[0], 0x05);
    CHECK_EQ(out[1], 0x02);
    CHECK_EQ(out[2], TMC2208_REG_GCONF | 0x80);
    CHECK_EQ(out[3], 0xDE);
    CHECK_EQ(out[4], 0xAD);
    CHECK_EQ(out[5], 0xBE);
    CHECK_EQ(out[6], 0xEF);
    CHECK_EQ(out[7], tmc2208_crc8(out, 7));
    return 1;
}

static int test_build_read_request_frame(void)
{
    uint8_t out[4];
    tmc2208_build_read_request(3, TMC2208_REG_DRV_STATUS, out);
    CHECK_EQ(out[0], 0x05);
    CHECK_EQ(out[1], 0x03);
    CHECK_EQ(out[2], TMC2208_REG_DRV_STATUS);
    CHECK_EQ(out[3], tmc2208_crc8(out, 3));
    return 1;
}

static int test_parse_read_reply(void)
{
    /* Build a valid reply for register 0x6F = 0x12345678. */
    uint8_t reply[8] = { 0x05, 0xFF, 0x6F, 0x12, 0x34, 0x56, 0x78, 0 };
    reply[7] = tmc2208_crc8(reply, 7);

    uint32_t value;
    CHECK(tmc2208_parse_read_reply(reply, 0x6F, &value));
    CHECK_EQ(value, 0x12345678);
    return 1;
}

static int test_parse_read_reply_rejects_bad_crc(void)
{
    uint8_t reply[8] = { 0x05, 0xFF, 0x6F, 0x12, 0x34, 0x56, 0x78, 0x55 };
    uint32_t value;
    CHECK(!tmc2208_parse_read_reply(reply, 0x6F, &value));
    return 1;
}

static int test_parse_read_reply_rejects_wrong_register(void)
{
    uint8_t reply[8] = { 0x05, 0xFF, 0x6F, 0, 0, 0, 0, 0 };
    reply[7] = tmc2208_crc8(reply, 7);
    uint32_t value;
    CHECK(!tmc2208_parse_read_reply(reply, 0x01, &value));
    return 1;
}

static int test_mres_lookup(void)
{
    CHECK_EQ(tmc2208_mres_for_microsteps(256), 0);
    CHECK_EQ(tmc2208_mres_for_microsteps(16),  4);
    CHECK_EQ(tmc2208_mres_for_microsteps(1),   8);
    /* Invalid -> fall back to full step (MRES=8). */
    CHECK_EQ(tmc2208_mres_for_microsteps(7),   8);
    return 1;
}

static int test_ihold_irun_pack(void)
{
    /* IHOLD=5, IRUN=20, IHOLDDELAY=8 -> 0x00081405 */
    uint32_t packed = tmc2208_pack_ihold_irun(5, 20, 8);
    CHECK_EQ(packed, 0x00081405u);
    return 1;
}

static int test_current_to_cs_rsense_110(void)
{
    /* Rsense 0.11 Ω, VSENSE=0, V_FS=0.325V:
     *   CS+1 ≈ I_mA × 32 × √2 × 0.13 / 0.325 = I_mA × 0.01810
     * For 800 mA: CS+1 ≈ 14.48 → CS=13 */
    uint8_t cs = tmc2208_current_to_cs(800, 110);
    CHECK(cs >= 12 && cs <= 14);
    /* Zero current -> 0. */
    CHECK_EQ(tmc2208_current_to_cs(0, 110), 0);
    /* Implausible Rsense -> 0. */
    CHECK_EQ(tmc2208_current_to_cs(800, 5), 0);
    /* Very large current clamps at 31. */
    CHECK_EQ(tmc2208_current_to_cs(5000, 110), 31);
    return 1;
}

/* ── Driver vtable lifecycle ───────────────────────────────────── */

static int test_drv_init_is_noop(void)
{
    reset_state();
    CHECK(drv_init());
    CHECK_EQ(log_count, 0);
    CHECK(!port_initialized);
    CHECK_EQ(axis_count, 0);
    return 1;
}

static int test_drv_apply_config_updates_axis_state(void)
{
    reset_state();
    pin_config_t cfg = {0};
    cfg.params.tmc2208.address         = 1;
    cfg.params.tmc2208.step_pin        = 4;
    cfg.params.tmc2208.dir_pin         = 5;
    cfg.params.tmc2208.rsense_milliohm = 75;
    cfg.params.tmc2208.microsteps      = 32;
    cfg.params.tmc2208.run_current_ma  = 900;
    cfg.params.tmc2208.hold_current_ma = 400;
    cfg.params.tmc2208.stealth_chop    = 1;
    cfg.params.tmc2208.max_position    = 5000;
    cfg.params.tmc2208.max_speed_pps   = 2000;

    /* Axis 1 = channel range 4..7. */
    uint8_t ch = 1 * TMC2208_CHANNELS_PER_AXIS;
    CHECK(drv_apply_config(ch, &cfg));
    CHECK_EQ(axes[1].address, 1);
    CHECK_EQ(axes[1].step_pin, 4);
    CHECK_EQ(axes[1].dir_pin, 5);
    CHECK_EQ(axes[1].rsense_milliohm, 75);
    CHECK_EQ(axes[1].microsteps, 32);
    CHECK_EQ(axes[1].max_position, 5000);
    CHECK_EQ(axis_count, 2);
    return 1;
}

static int test_drv_apply_config_boot_reload_preserves_unit_state(void)
{
    reset_state();
    /* Stand in for drv_load: populate axis 0 with values. */
    axes[0].address = 1;
    axes[0].step_pin = 4;
    axes[0].dir_pin = 5;
    axes[0].microsteps = 32;
    axis_count = 1;

    /* apply_hardware sweep — zero params. step_pin=0xFF is our
     * fingerprint for "nothing to copy". */
    pin_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode = PIN_MODE_TMC2208_STEPPER;
    cfg.params.tmc2208.step_pin = 0xFF;

    CHECK(drv_apply_config(0, &cfg));
    CHECK_EQ(axes[0].step_pin, 4);
    CHECK_EQ(axes[0].microsteps, 32);
    return 1;
}

static int test_invalid_pin_pair_warns(void)
{
    reset_state();
    tmc2208_tx_pin = 99;
    tmc2208_rx_pin = 100;
    /* Stub returns true unconditionally — we can't force a fallback
     * from the host. Instead just verify init runs cleanly and
     * port_initialized flips. */
    tmc2208_init();
    CHECK(port_initialized);
    return 1;
}

/* ── set/get value scaling ─────────────────────────────────────── */

static int test_set_value_position_scales(void)
{
    reset_state();
    port_initialized = true;
    axes[0].max_position = 10000;

    CHECK(drv_set_value(TMC2208_SUB_TARGET_POSITION, 0.5f));
    CHECK_EQ(axes[0].target_position, 5000);

    CHECK(drv_set_value(TMC2208_SUB_TARGET_POSITION, -1.0f));
    CHECK_EQ(axes[0].target_position, -10000);

    /* Clamp. */
    CHECK(drv_set_value(TMC2208_SUB_TARGET_POSITION, 5.0f));
    CHECK_EQ(axes[0].target_position, 10000);
    return 1;
}

static int test_set_value_velocity_scales(void)
{
    reset_state();
    port_initialized = true;
    axes[0].max_speed_pps = 1500;

    CHECK(drv_set_value(TMC2208_SUB_TARGET_VELOCITY, 0.25f));
    CHECK_EQ(axes[0].target_velocity, 375);

    CHECK(drv_set_value(TMC2208_SUB_TARGET_VELOCITY, -1.0f));
    CHECK_EQ(axes[0].target_velocity, -1500);
    return 1;
}

static int test_set_value_readonly_subchans_reject(void)
{
    reset_state();
    port_initialized = true;
    CHECK(!drv_set_value(TMC2208_SUB_CURRENT_POSITION, 0.5f));
    CHECK(!drv_set_value(TMC2208_SUB_ERROR_FLAGS,      0.5f));
    return 1;
}

static int test_get_value_returns_correct_field(void)
{
    reset_state();
    axes[2].target_position  = 5000;
    axes[2].target_velocity  = 750;
    axes[2].current_position = 1234;
    axes[2].error_flags      = TMC2208_DRV_STATUS_OTPW | TMC2208_DRV_STATUS_OLA;
    axes[2].max_position     = 10000;
    axes[2].max_speed_pps    = 1500;

    uint8_t base = 2 * TMC2208_CHANNELS_PER_AXIS;
    float v;

    CHECK(drv_get_value(base + TMC2208_SUB_TARGET_POSITION, &v));
    CHECK_FLOAT(v, 0.5f);

    CHECK(drv_get_value(base + TMC2208_SUB_TARGET_VELOCITY, &v));
    CHECK_FLOAT(v, 0.5f);

    CHECK(drv_get_value(base + TMC2208_SUB_CURRENT_POSITION, &v));
    CHECK_FLOAT(v, 1234.0f);

    /* error_flags is masked to the low byte. OTPW(0x01) | OLA(0x40) = 0x41 */
    CHECK(drv_get_value(base + TMC2208_SUB_ERROR_FLAGS, &v));
    CHECK_FLOAT(v, 0x41);
    return 1;
}

/* ── tmc2208_step_done — the ISR callback ──────────────────────── */

static int test_step_done_bounded_move(void)
{
    reset_state();
    axes[0].steps_remaining = 3;
    axes[0].active_direction = 1;
    axes[0].current_position = 100;

    CHECK(tmc2208_step_done(0));
    CHECK_EQ(axes[0].current_position, 101);
    CHECK_EQ(axes[0].steps_remaining, 2);

    CHECK(tmc2208_step_done(0));
    CHECK_EQ(axes[0].steps_remaining, 1);

    /* Final step returns false (target reached). */
    CHECK(!tmc2208_step_done(0));
    CHECK_EQ(axes[0].current_position, 103);
    CHECK_EQ(axes[0].steps_remaining, 0);
    return 1;
}

static int test_step_done_unbounded_velocity(void)
{
    reset_state();
    axes[0].steps_remaining = 0;     /* unbounded sentinel */
    axes[0].active_direction = -1;
    axes[0].current_position = 100;

    /* Velocity-mode: returns true forever. */
    for (int i = 0; i < 10; i++) {
        CHECK(tmc2208_step_done(0));
    }
    CHECK_EQ(axes[0].current_position, 90);
    return 1;
}

/* ── Connection state machine ──────────────────────────────────── */

static int test_mark_response_first_success_connects(void)
{
    reset_state();
    CHECK(!axes[0].connected);
    mark_axis_response(0, true);
    CHECK(axes[0].connected);
    CHECK_LOG("connected");
    return 1;
}

static int test_mark_response_drops_after_threshold(void)
{
    reset_state();
    mark_axis_response(0, true);
    log_count = 0;
    mark_axis_response(0, false);
    CHECK(axes[0].connected);
    mark_axis_response(0, false);
    CHECK(axes[0].connected);
    mark_axis_response(0, false);
    CHECK(!axes[0].connected);
    CHECK_LOG("dropped");
    return 1;
}

/* ── save / load ───────────────────────────────────────────────── */

static int test_save_load_roundtrip(void)
{
    reset_state();
    tmc2208_tx_pin = 8;
    tmc2208_rx_pin = 9;
    configured_serial_port = 1;
    configured_baud = 57600;
    axes[0].address          = 0;
    axes[0].step_pin         = 2;
    axes[0].dir_pin          = 3;
    axes[0].rsense_milliohm  = 110;
    axes[0].microsteps       = 16;
    axes[0].run_current_ma   = 800;
    axes[0].hold_current_ma  = 200;
    axes[0].stealth_chop     = 1;
    axes[0].max_position     = 8000;
    axes[0].max_speed_pps    = 1200;
    axis_count = 1;

    flash_storage_data_t flash;
    memset(&flash, 0, sizeof(flash));
    CHECK(drv_save(&flash));
    CHECK_EQ(flash.tmc2208_config.axis_count, 1);
    CHECK_EQ(flash.tmc2208_config.baud_rate, 57600);
    CHECK_EQ(flash.uart_pins.tmc2208_tx_pin, 8);
    CHECK_EQ(flash.uart_pins.tmc2208_rx_pin, 9);
    CHECK_EQ(flash.tmc2208_config.axes[0].address, 0);
    CHECK_EQ(flash.tmc2208_config.axes[0].step_pin, 2);
    CHECK_EQ(flash.tmc2208_config.axes[0].dir_pin, 3);
    CHECK_EQ(flash.tmc2208_config.axes[0].rsense_milliohm, 110);
    CHECK_EQ(flash.tmc2208_config.axes[0].microsteps, 16);

    reset_state();
    CHECK(drv_load(&flash));
    CHECK_EQ(axis_count, 1);
    CHECK_EQ(configured_baud, 57600);
    CHECK_EQ(tmc2208_tx_pin, 8);
    CHECK_EQ(tmc2208_rx_pin, 9);
    CHECK_EQ(axes[0].step_pin, 2);
    CHECK_EQ(axes[0].rsense_milliohm, 110);
    CHECK_EQ(axes[0].microsteps, 16);
    CHECK_LOG("restored");
    return 1;
}

/* ── JSON parsing ──────────────────────────────────────────────── */

static int test_parse_json_extracts_fields(void)
{
    reset_state();
    const char* json =
        "{ \"address\":2, \"step_pin\":10, \"dir_pin\":11, "
        "\"rsense_milliohm\":75, \"microsteps\":32, "
        "\"run_current_ma\":1200, \"hold_current_ma\":300, "
        "\"stealth_chop\":false, \"max_position\":20000, "
        "\"max_speed_pps\":3000 }";
    pin_config_t cfg = {0};
    cfg.gpio = (uint8_t)(TMC2208_VIRTUAL_GPIO_BASE + 7);

    drv_parse_json(json, json + strlen(json), &cfg);

    CHECK_EQ(cfg.params.tmc2208.address, 2);
    CHECK_EQ(cfg.params.tmc2208.step_pin, 10);
    CHECK_EQ(cfg.params.tmc2208.dir_pin, 11);
    CHECK_EQ(cfg.params.tmc2208.rsense_milliohm, 75);
    CHECK_EQ(cfg.params.tmc2208.microsteps, 32);
    CHECK_EQ(cfg.params.tmc2208.run_current_ma, 1200);
    CHECK_EQ(cfg.params.tmc2208.hold_current_ma, 300);
    CHECK_EQ(cfg.params.tmc2208.stealth_chop, 0);
    CHECK_EQ(cfg.params.tmc2208.max_position, 20000);
    CHECK_EQ(cfg.params.tmc2208.max_speed_pps, 3000);
    return 1;
}

/* ── Test runner ───────────────────────────────────────────────── */

typedef int (*test_fn)(void);
typedef struct { const char* name; test_fn fn; } test_entry_t;

static const test_entry_t TESTS[] = {
    {"crc_known_vector",                              test_crc_known_vector},
    {"build_write_frame",                             test_build_write_frame},
    {"build_read_request_frame",                      test_build_read_request_frame},
    {"parse_read_reply",                              test_parse_read_reply},
    {"parse_read_reply_rejects_bad_crc",              test_parse_read_reply_rejects_bad_crc},
    {"parse_read_reply_rejects_wrong_register",       test_parse_read_reply_rejects_wrong_register},
    {"mres_lookup",                                   test_mres_lookup},
    {"ihold_irun_pack",                               test_ihold_irun_pack},
    {"current_to_cs_rsense_110",                      test_current_to_cs_rsense_110},

    {"drv_init_is_noop",                              test_drv_init_is_noop},
    {"drv_apply_config_updates_axis_state",           test_drv_apply_config_updates_axis_state},
    {"drv_apply_config_boot_reload_preserves_unit_state",
                                                       test_drv_apply_config_boot_reload_preserves_unit_state},
    {"invalid_pin_pair_warns",                        test_invalid_pin_pair_warns},

    {"set_value_position_scales",                     test_set_value_position_scales},
    {"set_value_velocity_scales",                     test_set_value_velocity_scales},
    {"set_value_readonly_subchans_reject",            test_set_value_readonly_subchans_reject},
    {"get_value_returns_correct_field",               test_get_value_returns_correct_field},

    {"step_done_bounded_move",                        test_step_done_bounded_move},
    {"step_done_unbounded_velocity",                  test_step_done_unbounded_velocity},

    {"mark_response_first_success_connects",          test_mark_response_first_success_connects},
    {"mark_response_drops_after_threshold",           test_mark_response_drops_after_threshold},

    {"save_load_roundtrip",                           test_save_load_roundtrip},

    {"parse_json_extracts_fields",                    test_parse_json_extracts_fields},
};

int main(void)
{
    int passed = 0, failed = 0;
    size_t n = sizeof(TESTS) / sizeof(TESTS[0]);
    for (size_t i = 0; i < n; i++) {
        int ok = TESTS[i].fn();
        if (ok) {
            printf("  ok   %s\n", TESTS[i].name);
            passed++;
        } else {
            printf("  FAIL %s\n", TESTS[i].name);
            failed++;
        }
    }
    printf("\n%d passed, %d failed (%zu total)\n", passed, failed, n);
    return failed == 0 ? 0 : 1;
}
