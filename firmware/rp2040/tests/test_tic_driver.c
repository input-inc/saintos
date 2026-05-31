/**
 * Host-runnable tests for the Pololu Tic stepper driver.
 *
 * Build/run: `./run_tests.sh` (in this directory). Uses cc on the host.
 *
 * Same shape as test_roboclaw_driver.c — stub the platform/log headers
 * via define-guards, then `#include "../../shared/src/tic_driver.c"`
 * so static state and helpers are reachable. SIMULATION drops the
 * wire-level send paths so set_target_position / set_target_velocity
 * return true without trying to talk to a UART.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <assert.h>

#define SIMULATION 1

/* ── Platform stub ─────────────────────────────────────────────── */
#define PLATFORM_H
static uint32_t test_now_ms = 0;
#define PLATFORM_MILLIS()      (test_now_ms)
#define PLATFORM_SLEEP_MS(ms)  ((void)(ms))
#define PLATFORM_PRINTF(...)   ((void)0)

/* ── saint_log stub ────────────────────────────────────────────── */
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

/* ── uart_pin_pairs stub ───────────────────────────────────────── */
#define UART_PIN_PAIRS_H
static uint8_t test_pair_reject_tx = 0xFF;
static uint8_t test_pair_reject_rx = 0xFF;
static const char* test_pair_json_payload = NULL;
static uint8_t test_pair_json_tx = 0;
static uint8_t test_pair_json_rx = 0;
static uint8_t test_pair_json_inst = 0;

static bool uart_pin_pair_lookup(uint8_t tx, uint8_t rx, uint8_t* inst)
{
    if (tx == test_pair_reject_tx && rx == test_pair_reject_rx) return false;
    *inst = 0;
    return true;
}
static bool uart_pin_pair_parse_json(const char* a, const char* b,
                                      uint8_t* tx, uint8_t* rx, uint8_t* inst)
{
    (void)a; (void)b;
    if (!test_pair_json_payload) return false;
    *tx = test_pair_json_tx;
    *rx = test_pair_json_rx;
    *inst = test_pair_json_inst;
    return true;
}

/* ── Transport stub ────────────────────────────────────────────── */
#define SAINT_TIC_TRANSPORT_H
typedef struct tic_transport_ops {
    const char* name;
    bool (*open)(uint8_t tx_pin, uint8_t rx_pin, uint32_t baud);
    bool (*is_open)(void);
    bool (*write)(const uint8_t* data, size_t len);
    size_t (*read)(uint8_t* data, size_t max_len);
    uint8_t (*resolved_instance)(void);
} tic_transport_ops_t;
static const tic_transport_ops_t* tic_get_transport(void) { return NULL; }

/* ── Pull in the shared driver ─────────────────────────────────── */
#include "../../shared/src/tic_driver.c"

/* ── Test helpers ──────────────────────────────────────────────── */
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
        for (int _i = 0; _i < log_count; _i++) \
            fprintf(stderr, "    log[%d]: %s\n", _i, log_lines[_i]); \
        return 0; \
    } \
} while (0)

static void reset_state(void)
{
    memset(units, 0, sizeof(units));
    unit_count = 0;
    port_initialized = false;
    configured_baud = TIC_DEFAULT_BAUD;
    configured_serial_port = TIC_DEFAULT_SERIAL_PORT;
    tic_tx_pin = TIC_DEFAULT_TX_PIN;
    tic_rx_pin = TIC_DEFAULT_RX_PIN;
    active_tx_pin = 0xFF;
    active_rx_pin = 0xFF;
    active_uart   = 0xFF;
    poll_unit = 0;
    poll_register = 0;
    log_count = 0;
    test_now_ms = 1000;
    test_pair_reject_tx = 0xFF;
    test_pair_reject_rx = 0xFF;
    test_pair_json_payload = NULL;
}

/* ── Protocol encoding tests ───────────────────────────────────── */

static int test_encode_quick(void)
{
    uint8_t buf[8] = {0};
    size_t n = tic_encode_quick(14, TIC_CMD_ENERGIZE, buf);
    CHECK_EQ(n, 3);
    CHECK_EQ(buf[0], 0xAA);
    CHECK_EQ(buf[1], 14);
    /* MSB cleared in Pololu mode. */
    CHECK_EQ(buf[2], TIC_CMD_ENERGIZE & 0x7F);
    return 1;
}

static int test_encode_32bit_packs_msbs(void)
{
    /* 0x499602D2: bytes 0xD2, 0x02, 0x96, 0x49.
     * MSB pattern = 1, 0, 1, 0 -> 0x05. */
    uint8_t buf[8] = {0};
    size_t n = tic_encode_32bit(14, TIC_CMD_SET_TARGET_POSITION,
                                 (int32_t)0x499602D2, buf);
    CHECK_EQ(n, 8);
    CHECK_EQ(buf[0], 0xAA);
    CHECK_EQ(buf[1], 14);
    CHECK_EQ(buf[2], TIC_CMD_SET_TARGET_POSITION & 0x7F);
    CHECK_EQ(buf[3], 0x05);
    CHECK_EQ(buf[4], 0x52);
    CHECK_EQ(buf[5], 0x02);
    CHECK_EQ(buf[6], 0x16);
    CHECK_EQ(buf[7], 0x49);
    return 1;
}

static int test_encode_32bit_negative(void)
{
    /* -1 = 0xFFFFFFFF: all MSBs set -> 0x0F, all data bytes 0x7F. */
    uint8_t buf[8] = {0};
    (void)tic_encode_32bit(14, TIC_CMD_SET_TARGET_VELOCITY, -1, buf);
    CHECK_EQ(buf[3], 0x0F);
    CHECK_EQ(buf[4], 0x7F);
    CHECK_EQ(buf[5], 0x7F);
    CHECK_EQ(buf[6], 0x7F);
    CHECK_EQ(buf[7], 0x7F);
    return 1;
}

static int test_encode_block_read(void)
{
    uint8_t buf[8] = {0};
    size_t n = tic_encode_block_read(14, TIC_VAR_CURRENT_POSITION, 4, buf);
    CHECK_EQ(n, 5);
    CHECK_EQ(buf[0], 0xAA);
    CHECK_EQ(buf[1], 14);
    CHECK_EQ(buf[2], TIC_CMD_GET_VARIABLE & 0x7F);
    CHECK_EQ(buf[3], TIC_VAR_CURRENT_POSITION);
    CHECK_EQ(buf[4], 4);
    return 1;
}

static int test_decode_helpers(void)
{
    uint8_t le_i32[4] = { 0xD2, 0x02, 0x96, 0x49 };
    CHECK_EQ(tic_decode_i32(le_i32), (int32_t)0x499602D2);

    uint8_t le_neg[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
    CHECK_EQ(tic_decode_i32(le_neg), -1);

    uint8_t le_u16[2] = { 0x88, 0x13 };  /* 5000 */
    CHECK_EQ(tic_decode_u16(le_u16), 5000);
    return 1;
}

/* ── Connection state machine ──────────────────────────────────── */

static int test_mark_response_first_success_connects(void)
{
    reset_state();
    CHECK(!units[0].connected);
    mark_unit_response(0, true);
    CHECK(units[0].connected);
    CHECK_EQ(units[0].consecutive_misses, 0);
    CHECK_EQ(units[0].last_response_ms, 1000);
    CHECK_LOG("connected");
    return 1;
}

static int test_mark_response_drops_after_threshold(void)
{
    reset_state();
    mark_unit_response(0, true);
    log_count = 0;

    mark_unit_response(0, false);
    CHECK(units[0].connected);
    mark_unit_response(0, false);
    CHECK(units[0].connected);
    mark_unit_response(0, false);
    CHECK(!units[0].connected);
    CHECK_LOG("dropped");
    return 1;
}

static int test_mark_response_recovers_after_drop(void)
{
    reset_state();
    mark_unit_response(0, true);
    mark_unit_response(0, false);
    mark_unit_response(0, false);
    mark_unit_response(0, false);  /* dropped */
    CHECK(!units[0].connected);
    log_count = 0;

    test_now_ms += 1000;
    mark_unit_response(0, true);
    CHECK(units[0].connected);
    CHECK_EQ(units[0].consecutive_misses, 0);
    CHECK_LOG("recovered");
    CHECK_LOG("connected");
    return 1;
}

static int test_mark_response_never_connected_logs_once(void)
{
    reset_state();
    mark_unit_response(0, false);
    mark_unit_response(0, false);
    CHECK_EQ(log_count, 0);
    mark_unit_response(0, false);
    CHECK_LOG("no response after");

    int log_count_before = log_count;
    for (int i = 0; i < 10; i++) mark_unit_response(0, false);
    CHECK_EQ(log_count, log_count_before);
    return 1;
}

/* ── JSON parsing ──────────────────────────────────────────────── */

static int test_parse_json_extracts_fields(void)
{
    reset_state();
    const char* json = "{ \"address\": 17, \"max_position\": 50000, "
                       "\"max_speed_pps\": 2500 }";
    pin_config_t cfg = {0};
    cfg.gpio = TIC_VIRTUAL_GPIO_BASE + 7;  /* mid-channel; no log */

    tic_drv_parse_json(json, json + strlen(json), &cfg);

    CHECK_EQ(cfg.params.tic.address, 17);
    CHECK_EQ(cfg.params.tic.max_position, 50000);
    CHECK_EQ(cfg.params.tic.max_speed_pps, 2500);
    return 1;
}

static int test_parse_json_logs_only_on_channel_0(void)
{
    reset_state();
    test_pair_json_payload = "stub";
    test_pair_json_tx = 4;
    test_pair_json_rx = 5;
    test_pair_json_inst = 1;

    const char* json = "{ \"address\": 14 }";
    pin_config_t cfg = {0};

    cfg.gpio = (uint8_t)(TIC_VIRTUAL_GPIO_BASE + 3);
    tic_drv_parse_json(json, json + strlen(json), &cfg);
    CHECK_EQ(log_count, 0);

    cfg.gpio = (uint8_t)TIC_VIRTUAL_GPIO_BASE;
    tic_drv_parse_json(json, json + strlen(json), &cfg);
    CHECK_EQ(log_count, 1);
    CHECK_LOG("Tic sync");
    CHECK_LOG("TX=4");
    CHECK_LOG("RX=5");
    return 1;
}

static int test_parse_json_missing_pins_warns(void)
{
    reset_state();
    const char* json = "{ \"address\": 14 }";
    pin_config_t cfg = {0};
    cfg.gpio = (uint8_t)TIC_VIRTUAL_GPIO_BASE;

    tic_drv_parse_json(json, json + strlen(json), &cfg);

    CHECK_LOG("[warn]");
    CHECK_LOG("didn't find uart_tx/uart_rx");
    return 1;
}

/* ── Driver vtable lifecycle ───────────────────────────────────── */

static int test_drv_init_is_noop(void)
{
    reset_state();
    bool ok = tic_drv_init();
    CHECK(ok);
    CHECK_EQ(log_count, 0);
    CHECK(!port_initialized);
    CHECK_EQ(unit_count, 0);
    return 1;
}

static int test_drv_apply_config_updates_unit_state(void)
{
    reset_state();
    pin_config_t cfg = {0};
    cfg.params.tic.address = 17;
    cfg.params.tic.max_position = 50000;
    cfg.params.tic.max_speed_pps = 2500;

    uint8_t ch = 2 * TIC_CHANNELS_PER_UNIT;
    bool ok = tic_drv_apply_config(ch, &cfg);

    CHECK(ok);
    CHECK_EQ(units[2].address, 17);
    CHECK_EQ(units[2].max_position, 50000);
    CHECK_EQ(units[2].max_speed_pps, 2500);
    CHECK_EQ(unit_count, 3);
    return 1;
}

static int test_drv_apply_config_out_of_range_rejected(void)
{
    reset_state();
    pin_config_t cfg = {0};
    CHECK(!tic_drv_apply_config(255, &cfg));
    CHECK_EQ(unit_count, 0);
    return 1;
}

/* Boot-reload guard: apply_hardware sweep with zero-init params must
 * NOT clobber values drv_load just restored. */
static int test_apply_config_boot_reload_preserves_unit_state(void)
{
    reset_state();
    /* Stand in for drv_load: populate unit 0 from "flash". */
    units[0].address = 17;
    units[0].max_position = 50000;
    units[0].max_speed_pps = 2500;
    unit_count = 1;

    /* apply_hardware sweep — zero params, mode set. */
    pin_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode = PIN_MODE_TIC_STEPPER;

    CHECK(tic_drv_apply_config(0, &cfg));

    /* Field copy must be skipped because address==0 in cfg. */
    CHECK_EQ(units[0].address, 17);
    CHECK_EQ(units[0].max_position, 50000);
    CHECK_EQ(units[0].max_speed_pps, 2500);
    return 1;
}

/* ── Invalid pin pair fallback ─────────────────────────────────── */

static int test_invalid_pin_pair_warns(void)
{
    reset_state();
    test_pair_reject_tx = 5;
    test_pair_reject_rx = 6;
    tic_tx_pin = 5;
    tic_rx_pin = 6;

    tic_init();

    CHECK_LOG("[warn]");
    CHECK_LOG("isn't a valid UART pair");
    CHECK_EQ(tic_tx_pin, TIC_DEFAULT_TX_PIN);
    CHECK_EQ(tic_rx_pin, TIC_DEFAULT_RX_PIN);
    return 1;
}

/* ── set/get value ─────────────────────────────────────────────── */

static int test_set_value_position_scales(void)
{
    reset_state();
    port_initialized = true;
    units[0].max_position = 10000;

    uint8_t ch = TIC_SUB_TARGET_POSITION;
    CHECK(tic_drv_set_value(ch, 0.5f));
    CHECK_EQ(units[0].target_position, 5000);

    CHECK(tic_drv_set_value(ch, -1.0f));
    CHECK_EQ(units[0].target_position, -10000);

    /* Clamping. */
    CHECK(tic_drv_set_value(ch, 5.0f));
    CHECK_EQ(units[0].target_position, 10000);
    return 1;
}

static int test_set_value_velocity_scales(void)
{
    reset_state();
    port_initialized = true;
    units[0].max_speed_pps = 1000;

    uint8_t ch = TIC_SUB_TARGET_VELOCITY;
    CHECK(tic_drv_set_value(ch, 0.25f));
    CHECK_EQ(units[0].target_velocity, 250);

    CHECK(tic_drv_set_value(ch, -1.0f));
    CHECK_EQ(units[0].target_velocity, -1000);
    return 1;
}

static int test_set_value_readonly_subchans_reject(void)
{
    reset_state();
    port_initialized = true;
    CHECK(!tic_drv_set_value(TIC_SUB_CURRENT_POSITION, 0.5f));
    CHECK(!tic_drv_set_value(TIC_SUB_CURRENT_VELOCITY, 0.5f));
    CHECK(!tic_drv_set_value(TIC_SUB_VIN_VOLTAGE,      0.5f));
    CHECK(!tic_drv_set_value(TIC_SUB_ERROR_STATUS,     0.5f));
    return 1;
}

static int test_get_value_returns_correct_field(void)
{
    reset_state();
    units[3].target_position  = 5000;
    units[3].target_velocity  = 250;
    units[3].current_position = 4321;
    units[3].current_velocity = 987;
    units[3].vin_voltage_mv   = 24500;   /* 24.5 V */
    units[3].error_status     = 0x0042;
    units[3].max_position     = 10000;
    units[3].max_speed_pps    = 1000;

    uint8_t base = 3 * TIC_CHANNELS_PER_UNIT;
    float v;

    CHECK(tic_drv_get_value(base + TIC_SUB_TARGET_POSITION, &v));
    CHECK_FLOAT(v, 0.5f);

    CHECK(tic_drv_get_value(base + TIC_SUB_TARGET_VELOCITY, &v));
    CHECK_FLOAT(v, 0.25f);

    CHECK(tic_drv_get_value(base + TIC_SUB_CURRENT_POSITION, &v));
    CHECK_FLOAT(v, 4321.0f);

    CHECK(tic_drv_get_value(base + TIC_SUB_CURRENT_VELOCITY, &v));
    CHECK_FLOAT(v, 987.0f);

    CHECK(tic_drv_get_value(base + TIC_SUB_VIN_VOLTAGE, &v));
    CHECK_FLOAT(v, 24.5f);

    CHECK(tic_drv_get_value(base + TIC_SUB_ERROR_STATUS, &v));
    CHECK_FLOAT(v, 66.0f);  /* 0x42 = 66 */
    return 1;
}

/* ── save / load ───────────────────────────────────────────────── */

static int test_save_load_roundtrip(void)
{
    reset_state();
    tic_tx_pin = 8;
    tic_rx_pin = 9;
    configured_serial_port = 1;
    configured_baud = 9600;
    units[0].address       = 14;
    units[0].max_speed_pps = 1500;
    units[0].max_position  = 25000;
    units[1].address       = 15;
    units[1].max_speed_pps = 2000;
    units[1].max_position  = 30000;
    unit_count = 2;

    flash_storage_data_t flash;
    memset(&flash, 0, sizeof(flash));

    CHECK(tic_drv_save(&flash));
    CHECK_EQ(flash.tic_config.unit_count, 2);
    CHECK_EQ(flash.tic_config.serial_port, 1);
    CHECK_EQ(flash.tic_config.baud_rate, 9600);
    CHECK_EQ(flash.uart_pins.tic_tx_pin, 8);
    CHECK_EQ(flash.uart_pins.tic_rx_pin, 9);
    CHECK_EQ(flash.tic_config.units[0].address, 14);
    CHECK_EQ(flash.tic_config.units[0].max_speed_pps, 1500);
    CHECK_EQ(flash.tic_config.units[0].max_position, 25000);
    CHECK_EQ(flash.tic_config.units[1].address, 15);

    reset_state();
    CHECK(tic_drv_load(&flash));
    CHECK_EQ(unit_count, 2);
    CHECK_EQ(configured_baud, 9600);
    CHECK_EQ(tic_tx_pin, 8);
    CHECK_EQ(tic_rx_pin, 9);
    CHECK_EQ(units[0].address, 14);
    CHECK_EQ(units[0].max_speed_pps, 1500);
    CHECK_EQ(units[0].max_position, 25000);
    CHECK_EQ(units[1].address, 15);
    CHECK_LOG("restored");
    return 1;
}

static int test_load_invalid_pin_pair_warns(void)
{
    reset_state();
    flash_storage_data_t flash;
    memset(&flash, 0, sizeof(flash));
    flash.uart_pins.tic_tx_pin = 99;
    flash.uart_pins.tic_rx_pin = 100;
    flash.tic_config.unit_count = 0;

    test_pair_reject_tx = 99;
    test_pair_reject_rx = 100;

    tic_drv_load(&flash);
    CHECK_LOG("[warn]");
    CHECK_LOG("stored pin pair");
    CHECK_LOG("isn't a valid UART pair");
    CHECK_EQ(tic_tx_pin, TIC_DEFAULT_TX_PIN);
    CHECK_EQ(tic_rx_pin, TIC_DEFAULT_RX_PIN);
    return 1;
}

/* ── Keepalive ─────────────────────────────────────────────────── */

static void setup_energized_unit(uint8_t unit_idx)
{
    units[unit_idx].address = TIC_DEFAULT_DEVICE_ID + unit_idx;
    units[unit_idx].energized = true;
    units[unit_idx].connected = true;
    units[unit_idx].last_command_ms = test_now_ms;
    if (unit_idx >= unit_count) unit_count = unit_idx + 1;
    port_initialized = true;
}

static int test_keepalive_skips_when_not_initialized(void)
{
    reset_state();
    units[0].energized = true;
    /* port_initialized stays false. */
    CHECK(!maybe_send_keepalive());
    return 1;
}

static int test_keepalive_skips_deenergized(void)
{
    reset_state();
    setup_energized_unit(0);
    units[0].energized = false;
    test_now_ms += TIC_KEEPALIVE_MS * 5;
    CHECK(!maybe_send_keepalive());
    return 1;
}

static int test_keepalive_skips_within_window(void)
{
    reset_state();
    setup_energized_unit(0);
    test_now_ms += TIC_KEEPALIVE_MS - 1;
    CHECK(!maybe_send_keepalive());
    return 1;
}

static int test_keepalive_fires_after_window(void)
{
    reset_state();
    setup_energized_unit(0);
    test_now_ms += TIC_KEEPALIVE_MS + 1;
    CHECK(maybe_send_keepalive());
    CHECK_EQ(units[0].last_command_ms, test_now_ms);
    CHECK(!maybe_send_keepalive());
    return 1;
}

static int test_keepalive_multi_unit_one_per_call(void)
{
    reset_state();
    setup_energized_unit(0);
    setup_energized_unit(1);
    test_now_ms += TIC_KEEPALIVE_MS + 1;

    CHECK(maybe_send_keepalive());
    CHECK_EQ(units[0].last_command_ms, test_now_ms);
    CHECK(units[1].last_command_ms < test_now_ms);

    CHECK(maybe_send_keepalive());
    CHECK_EQ(units[1].last_command_ms, test_now_ms);

    CHECK(!maybe_send_keepalive());
    return 1;
}

/* ── Test runner ───────────────────────────────────────────────── */

typedef int (*test_fn)(void);
typedef struct { const char* name; test_fn fn; } test_entry_t;

static const test_entry_t TESTS[] = {
    {"encode_quick",                                 test_encode_quick},
    {"encode_32bit_packs_msbs",                      test_encode_32bit_packs_msbs},
    {"encode_32bit_negative",                        test_encode_32bit_negative},
    {"encode_block_read",                            test_encode_block_read},
    {"decode_helpers",                               test_decode_helpers},

    {"mark_response_first_success_connects",         test_mark_response_first_success_connects},
    {"mark_response_drops_after_threshold",          test_mark_response_drops_after_threshold},
    {"mark_response_recovers_after_drop",            test_mark_response_recovers_after_drop},
    {"mark_response_never_connected_logs_once",      test_mark_response_never_connected_logs_once},

    {"parse_json_extracts_fields",                   test_parse_json_extracts_fields},
    {"parse_json_logs_only_on_channel_0",            test_parse_json_logs_only_on_channel_0},
    {"parse_json_missing_pins_warns",                test_parse_json_missing_pins_warns},

    {"drv_init_is_noop",                             test_drv_init_is_noop},
    {"drv_apply_config_updates_unit_state",          test_drv_apply_config_updates_unit_state},
    {"drv_apply_config_out_of_range_rejected",       test_drv_apply_config_out_of_range_rejected},
    {"apply_config_boot_reload_preserves_unit_state", test_apply_config_boot_reload_preserves_unit_state},

    {"invalid_pin_pair_warns",                       test_invalid_pin_pair_warns},

    {"set_value_position_scales",                    test_set_value_position_scales},
    {"set_value_velocity_scales",                    test_set_value_velocity_scales},
    {"set_value_readonly_subchans_reject",           test_set_value_readonly_subchans_reject},
    {"get_value_returns_correct_field",              test_get_value_returns_correct_field},

    {"save_load_roundtrip",                          test_save_load_roundtrip},
    {"load_invalid_pin_pair_warns",                  test_load_invalid_pin_pair_warns},

    {"keepalive_skips_when_not_initialized",         test_keepalive_skips_when_not_initialized},
    {"keepalive_skips_deenergized",                  test_keepalive_skips_deenergized},
    {"keepalive_skips_within_window",                test_keepalive_skips_within_window},
    {"keepalive_fires_after_window",                 test_keepalive_fires_after_window},
    {"keepalive_multi_unit_one_per_call",            test_keepalive_multi_unit_one_per_call},
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
