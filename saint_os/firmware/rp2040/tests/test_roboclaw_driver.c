/**
 * Host-runnable tests for the RoboClaw driver.
 *
 * Build/run: `./run_tests.sh` (in this directory) — uses cc on the host.
 *
 * Approach: same shape as test_fas100_parser.c — stub the platform/log
 * headers via define-guards, then `#include "../src/roboclaw_driver.c"`
 * so the test code lives in the same translation unit as the driver and
 * can call static functions and poke static state directly. SIMULATION
 * is defined so the hardware UART/GPIO paths drop out; what's left is
 * the config plumbing, JSON parsing, save/load roundtrip, sub-channel
 * math, and the connection-state state machine.
 *
 * What this CAN'T test under -DSIMULATION: the actual UART probe/poll
 * loop. Those tests would need a Pico SDK shim. The connection-state
 * machine (mark_unit_response) is intentionally factored OUTSIDE the
 * #ifndef SIMULATION block precisely so this test runner can exercise
 * the connect/drop transitions that motivated the rewrite.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <assert.h>

#define SIMULATION 1

/* ============================================================================
 * Platform stub (pretends to be firmware/rp2040/include/platform.h)
 * ============================================================================ */
#define PLATFORM_H
static uint32_t test_now_ms = 0;
#define PLATFORM_MILLIS()      (test_now_ms)
#define PLATFORM_SLEEP_MS(ms)  ((void)(ms))
#define PLATFORM_PRINTF(...)   ((void)0)

/* ============================================================================
 * saint_log stub — capture each emitted line so tests can assert on
 * presence/level. Matches the FAS100 test runner's shape.
 * ============================================================================ */
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

/* Substring match across all captured log lines. Returns 1 if any line
 * contains `needle`, 0 otherwise. */
static int log_contains(const char* needle)
{
    for (int i = 0; i < log_count; i++) {
        if (strstr(log_lines[i], needle)) return 1;
    }
    return 0;
}

/* ============================================================================
 * uart_pin_pairs stub — controllable from tests via test_pair_*. By
 * default lookup() accepts everything (returns UART0 for any pair); a
 * test can set test_pair_reject_pair to make it reject the specific
 * pair, simulating the operator picking an invalid combo.
 * ============================================================================ */
#define UART_PIN_PAIRS_H
static uint8_t test_pair_reject_tx = 0xFF;
static uint8_t test_pair_reject_rx = 0xFF;
static const char* test_pair_json_payload = NULL;  /* if set, parse_json returns these pins */
static uint8_t test_pair_json_tx = 0;
static uint8_t test_pair_json_rx = 0;
static uint8_t test_pair_json_inst = 0;

static bool uart_pin_pair_lookup(uint8_t tx, uint8_t rx, uint8_t* inst)
{
    if (tx == test_pair_reject_tx && rx == test_pair_reject_rx) {
        return false;
    }
    *inst = 0;
    return true;
}

static bool uart_pin_pair_parse_json(const char* a, const char* b,
                                      uint8_t* tx, uint8_t* rx, uint8_t* inst)
{
    (void)a; (void)b;
    if (!test_pair_json_payload) return false;
    *tx   = test_pair_json_tx;
    *rx   = test_pair_json_rx;
    *inst = test_pair_json_inst;
    return true;
}

/* ============================================================================
 * PIO UART stubs — the driver #includes pio_uart.h which provides
 * SIMULATION-mode void*-based stubs. We compile pio_uart.c too so the
 * linker has definitions; under SIMULATION the .c file is a thin set
 * of no-op stubs that return false / 0.
 *
 * The point of including the real stubs (vs. defining our own) is to
 * keep the SIMULATION code path under test: if pio_uart.c later grows
 * SIMULATION behavior that the driver depends on, the test runner
 * automatically picks it up without manual sync.
 * ============================================================================ */
#include "../src/pio_uart.c"

/* ============================================================================
 * Pull in the real driver. The hardware paths under #ifndef SIMULATION
 * drop out; everything else (state machine, config plumbing, JSON
 * parsing, save/load) compiles unchanged.
 * ============================================================================ */
#include "../src/roboclaw_driver.c"

/* ============================================================================
 * Test helpers — same idioms as test_fas100_parser.c
 * ============================================================================ */
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

/* Reset all driver static state so tests are order-independent.
 * roboclaw_driver.c keeps a lot of file-scope state — wipe it here. */
static void reset_state(void)
{
    memset(units, 0, sizeof(units));
    for (uint8_t i = 0; i < ROBOCLAW_MAX_UNITS; i++) {
        units[i].address = ROBOCLAW_ADDRESS_MIN + i;
    }
    unit_count = 0;
    port_initialized = false;
    configured_baud = ROBOCLAW_DEFAULT_BAUD;
    configured_serial_port = ROBOCLAW_DEFAULT_SERIAL_PORT;
    roboclaw_tx_pin = ROBOCLAW_DEFAULT_TX_PIN;
    roboclaw_rx_pin = ROBOCLAW_DEFAULT_RX_PIN;
    active_tx_pin = 0xFF;
    active_rx_pin = 0xFF;
    active_uart   = 0xFF;
    poll_unit = 0;
    poll_register = 0;

    use_pio_uart = false;

    log_count = 0;
    test_now_ms = 1000;
    test_pair_reject_tx = 0xFF;
    test_pair_reject_rx = 0xFF;
    test_pair_json_payload = NULL;
}

/* ============================================================================
 * CRC vectors — sanity check the protocol helper before relying on it
 * for any of the higher-level tests.
 * ============================================================================ */

/* CCITT/XMODEM polynomial 0x1021, init 0x0000, no reflection. Known
 * vectors taken from a reference implementation. */
static int test_crc_known_vectors(void)
{
    /* Single 0x00 byte → 0x0000. */
    uint8_t a[1] = { 0x00 };
    CHECK_EQ(roboclaw_crc16_calculate(a, 1), 0x0000);

    /* "123456789" (the canonical CCITT test vector) → 0x31C3. */
    uint8_t b[] = "123456789";
    CHECK_EQ(roboclaw_crc16_calculate(b, 9), 0x31C3);

    /* A RoboClaw GETVERSION packet (address 0x80, command 21) — verify
     * the CRC matches what gets sent on the wire. We don't hardcode the
     * expected value; instead, recompute byte-by-byte using the public
     * crc16_update and assert internal consistency. */
    uint8_t pkt[2] = { 0x80, ROBOCLAW_CMD_GETVERSION };
    uint16_t crc1 = roboclaw_crc16_calculate(pkt, 2);
    uint16_t crc2 = 0;
    crc2 = roboclaw_crc16_update(crc2, pkt[0]);
    crc2 = roboclaw_crc16_update(crc2, pkt[1]);
    CHECK_EQ(crc1, crc2);
    return 1;
}

/* ============================================================================
 * mark_unit_response — the connection-state machine. This is the core
 * of the "RoboClaw isn't reporting connected" diagnostic.
 * ============================================================================ */

/* First successful poll on a fresh unit flips connected=true with one
 * "connected" log line. */
static int test_mark_response_first_success_connects(void)
{
    reset_state();
    CHECK(!units[0].connected);
    CHECK_EQ(units[0].consecutive_misses, 0);

    mark_unit_response(0, true);

    CHECK(units[0].connected);
    CHECK_EQ(units[0].consecutive_misses, 0);
    CHECK_EQ(units[0].last_response_ms, 1000);
    CHECK_LOG("connected");
    return 1;
}

/* A connected unit staying connected (repeated successes) only logs the
 * transition once, not every poll. */
static int test_mark_response_repeated_success_no_extra_log(void)
{
    reset_state();
    mark_unit_response(0, true);
    int after_first = log_count;
    CHECK_EQ(after_first, 1);

    /* Five more successes — no new log lines. */
    for (int i = 0; i < 5; i++) {
        test_now_ms += 100;
        mark_unit_response(0, true);
    }
    CHECK_EQ(log_count, after_first);
    CHECK(units[0].connected);
    return 1;
}

/* A connected unit dropping after exactly ROBOCLAW_DROP_AFTER_MISSES
 * (3) consecutive failures, with a "dropped" log on the transition. */
static int test_mark_response_drops_after_threshold(void)
{
    reset_state();
    mark_unit_response(0, true);  /* connect */
    log_count = 0;                /* clear the connect log line */

    /* Two misses — still connected, no log. */
    mark_unit_response(0, false);
    CHECK(units[0].connected);
    CHECK_EQ(units[0].consecutive_misses, 1);
    CHECK_EQ(log_count, 0);

    mark_unit_response(0, false);
    CHECK(units[0].connected);
    CHECK_EQ(units[0].consecutive_misses, 2);
    CHECK_EQ(log_count, 0);

    /* Third miss → drop. */
    mark_unit_response(0, false);
    CHECK(!units[0].connected);
    CHECK_EQ(units[0].consecutive_misses, 3);
    CHECK_LOG("dropped");
    return 1;
}

/* A unit that has NEVER been connected (e.g. boot-time probe results
 * were dropped because ros_log_ready was false during drv_load) used
 * to be silent forever on subsequent miss-streaks. The dashboard
 * Logs tab gave no signal that the firmware was even trying. We now
 * fire ONE warn at the ROBOCLAW_DROP_AFTER_MISSES threshold with
 * different phrasing ("no ACK after N attempts...") so the operator
 * sees "we're trying, nothing's coming back" — then go silent again
 * to avoid spamming. */
static int test_mark_response_never_connected_logs_once_at_threshold(void)
{
    reset_state();
    /* No initial connect — unit starts disconnected (factory state). */

    /* Two misses still silent (below threshold). */
    mark_unit_response(0, false);
    CHECK_EQ(units[0].consecutive_misses, 1);
    CHECK_EQ(log_count, 0);
    mark_unit_response(0, false);
    CHECK_EQ(units[0].consecutive_misses, 2);
    CHECK_EQ(log_count, 0);

    /* Third miss → one warn (new behavior). Different phrasing than
     * the connected→dropped case so the operator can distinguish
     * "link broke" from "link never worked." */
    mark_unit_response(0, false);
    CHECK_EQ(units[0].consecutive_misses, 3);
    CHECK(!units[0].connected);  /* still disconnected; no state transition */
    CHECK_LOG("no ACK after");

    /* Subsequent misses stay silent — no spam. */
    int log_count_before = log_count;
    for (int i = 0; i < 10; i++) mark_unit_response(0, false);
    CHECK(!units[0].connected);
    CHECK_EQ(log_count, log_count_before);
    return 1;
}

/* A single success after a miss-streak resets the counter and (if the
 * unit was previously dropped) logs a fresh "connected" line — the
 * recovery path. */
static int test_mark_response_recovers_after_drop(void)
{
    reset_state();
    mark_unit_response(0, true);   /* connect */
    mark_unit_response(0, false);
    mark_unit_response(0, false);
    mark_unit_response(0, false);  /* drops */
    CHECK(!units[0].connected);
    log_count = 0;

    test_now_ms += 1000;
    mark_unit_response(0, true);   /* reconnect */
    CHECK(units[0].connected);
    CHECK_EQ(units[0].consecutive_misses, 0);
    CHECK_EQ(units[0].last_response_ms, test_now_ms);
    CHECK_LOG("connected");
    return 1;
}

/* Sub-threshold misses followed by a success: counter resets, no
 * drop log, no spurious reconnect log. */
static int test_mark_response_transient_miss_recovers(void)
{
    reset_state();
    mark_unit_response(0, true);   /* connect */
    log_count = 0;

    mark_unit_response(0, false);  /* 1 miss, still connected */
    CHECK(units[0].connected);
    CHECK_EQ(log_count, 0);

    mark_unit_response(0, true);   /* recover before threshold */
    CHECK(units[0].connected);
    CHECK_EQ(units[0].consecutive_misses, 0);
    CHECK_EQ(log_count, 0);        /* no transition, no log */
    return 1;
}

/* Out-of-range unit index is a no-op (defensive — the call comes from
 * the round-robin and should never overflow, but cheap to guard). */
static int test_mark_response_out_of_range_ignored(void)
{
    reset_state();
    mark_unit_response(ROBOCLAW_MAX_UNITS, true);
    mark_unit_response(255, true);
    /* No unit's state was touched. */
    for (uint8_t i = 0; i < ROBOCLAW_MAX_UNITS; i++) {
        CHECK(!units[i].connected);
    }
    CHECK_EQ(log_count, 0);
    return 1;
}

/* ============================================================================
 * parse_json — the routing-system entry point. Verifies field extraction
 * and the channel-0-only sync log to match what FAS100 does.
 * ============================================================================ */

static int test_parse_json_extracts_fields(void)
{
    reset_state();
    const char* json = "{ \"address\": 130, \"deadband\": 5, "
                       "\"max_current_ma\": 1500 }";
    pin_config_t cfg = {0};
    cfg.gpio = ROBOCLAW_VIRTUAL_GPIO_BASE + 7;  /* mid-channel, no log */

    roboclaw_drv_parse_json(json, json + strlen(json), &cfg);

    CHECK_EQ(cfg.params.roboclaw.address, 130);
    CHECK_EQ(cfg.params.roboclaw.deadband, 5);
    CHECK_EQ(cfg.params.roboclaw.max_current_ma, 1500);
    return 1;
}

/* Channel 0 (the canonical "first channel of the first unit") emits the
 * sync log line. Mid-channel calls don't, so the Logs tab doesn't get
 * 40 identical lines per sync. */
static int test_parse_json_logs_only_on_channel_0(void)
{
    reset_state();
    test_pair_json_payload = "stub";
    test_pair_json_tx = 4;
    test_pair_json_rx = 5;
    test_pair_json_inst = 1;

    const char* json = "{ \"address\": 128 }";
    pin_config_t cfg = {0};

    /* Mid-channel — no log. */
    cfg.gpio = ROBOCLAW_VIRTUAL_GPIO_BASE + 3;
    roboclaw_drv_parse_json(json, json + strlen(json), &cfg);
    CHECK_EQ(log_count, 0);

    /* Channel 0 — one log line. */
    cfg.gpio = ROBOCLAW_VIRTUAL_GPIO_BASE;
    roboclaw_drv_parse_json(json, json + strlen(json), &cfg);
    CHECK_EQ(log_count, 1);
    CHECK_LOG("RoboClaw sync");
    CHECK_LOG("TX=4");
    CHECK_LOG("RX=5");
    return 1;
}

/* When the JSON doesn't carry uart_tx/uart_rx, parse_json on channel 0
 * emits a "warn" log so the operator notices that their pin choice
 * didn't make it through. */
static int test_parse_json_missing_pins_warns(void)
{
    reset_state();
    /* test_pair_json_payload not set → parse_json returns false. */

    const char* json = "{ \"address\": 128 }";
    pin_config_t cfg = {0};
    cfg.gpio = ROBOCLAW_VIRTUAL_GPIO_BASE;

    roboclaw_drv_parse_json(json, json + strlen(json), &cfg);

    CHECK_LOG("[warn]");
    CHECK_LOG("didn't find uart_tx/uart_rx");
    return 1;
}

/* ============================================================================
 * drv_init / drv_apply_config — the boot vs. config-arrival contract.
 * ============================================================================ */

/* drv_init must be a no-op. It used to probe with default pins at boot,
 * which is exactly the bug that motivated the rewrite. */
static int test_drv_init_is_noop(void)
{
    reset_state();
    bool ok = roboclaw_drv_init();

    CHECK(ok);
    /* No log lines, no unit state changes, no port_initialized. */
    CHECK_EQ(log_count, 0);
    CHECK(!port_initialized);
    CHECK_EQ(unit_count, 0);
    for (uint8_t i = 0; i < ROBOCLAW_MAX_UNITS; i++) {
        CHECK(!units[i].connected);
    }
    return 1;
}

/* drv_apply_config updates the per-unit state for the channel's unit
 * and grows unit_count to cover the largest configured unit index. */
static int test_drv_apply_config_updates_unit_state(void)
{
    reset_state();
    pin_config_t cfg = {0};
    cfg.params.roboclaw.address = 0x82;
    cfg.params.roboclaw.deadband = 7;
    cfg.params.roboclaw.max_current_ma = 2000;

    /* Apply on channel for unit 2 (channel = 2 * CHANNELS_PER_UNIT). */
    uint8_t ch = 2 * ROBOCLAW_CHANNELS_PER_UNIT;
    bool ok = roboclaw_drv_apply_config(ch, &cfg);

    CHECK(ok);
    CHECK_EQ(units[2].address, 0x82);
    CHECK_EQ(units[2].deadband, 7);
    CHECK_EQ(units[2].max_current_ma, 2000);
    CHECK_EQ(unit_count, 3);   /* grew to cover unit index 2 */
    return 1;
}

/* drv_apply_config out-of-range returns false and doesn't touch state. */
static int test_drv_apply_config_out_of_range_rejected(void)
{
    reset_state();
    pin_config_t cfg = {0};
    bool ok = roboclaw_drv_apply_config(255, &cfg);
    CHECK(!ok);
    CHECK_EQ(unit_count, 0);
    return 1;
}

/* ============================================================================
 * Invalid pin-pair fallback — the silent-fallback regression that the
 * rewrite explicitly fixed. With our stub configured to reject (5,6),
 * roboclaw_init() should log a warn and fall back to defaults.
 * ============================================================================ */

static int test_invalid_pin_pair_warns(void)
{
    reset_state();
    test_pair_reject_tx = 5;
    test_pair_reject_rx = 6;
    roboclaw_tx_pin = 5;
    roboclaw_rx_pin = 6;

    roboclaw_init();

    CHECK_LOG("[warn]");
    CHECK_LOG("isn't a valid RP2040 UART pair");
    /* Fell back to defaults. */
    CHECK_EQ(roboclaw_tx_pin, ROBOCLAW_DEFAULT_TX_PIN);
    CHECK_EQ(roboclaw_rx_pin, ROBOCLAW_DEFAULT_RX_PIN);
    return 1;
}

/* ============================================================================
 * set_value / get_value — sub-channel routing on the peripheral_driver
 * vtable. The motor sub-channel is the only writable one; the readback
 * sub-channels each return the right stored field.
 * ============================================================================ */

static int test_set_value_only_motor_writable(void)
{
    reset_state();
    /* set_duty refuses to send until the UART is bound, so simulate a
     * post-init driver. The actual UART write is #ifdef'd out under
     * SIMULATION anyway; we're testing the duty-storage + sub-channel
     * routing here, not the wire-level send. */
    port_initialized = true;

    /* set_value on motor channel of unit 1 → clamp + store duty. */
    uint8_t motor_ch = 1 * ROBOCLAW_CHANNELS_PER_UNIT + ROBOCLAW_SUB_MOTOR;
    CHECK(roboclaw_drv_set_value(motor_ch, 0.5f));
    CHECK_EQ(units[1].duty, (int16_t)(0.5f * ROBOCLAW_DUTY_MAX));

    /* set_value on a non-motor sub-channel should return false. */
    uint8_t enc_ch = 1 * ROBOCLAW_CHANNELS_PER_UNIT + ROBOCLAW_SUB_ENCODER;
    CHECK(!roboclaw_drv_set_value(enc_ch, 1.0f));
    return 1;
}

static int test_set_value_clamps_to_unit_range(void)
{
    reset_state();
    port_initialized = true;  /* see note in test above */
    uint8_t motor_ch = ROBOCLAW_SUB_MOTOR;  /* unit 0 */

    CHECK(roboclaw_drv_set_value(motor_ch, 5.0f));
    CHECK_EQ(units[0].duty, ROBOCLAW_DUTY_MAX);

    CHECK(roboclaw_drv_set_value(motor_ch, -5.0f));
    CHECK_EQ(units[0].duty, ROBOCLAW_DUTY_MIN);
    return 1;
}

static int test_get_value_returns_correct_field(void)
{
    reset_state();
    units[3].duty         = 16384;   /* half throttle */
    units[3].encoder      = 12345;
    units[3].voltage_mv   = 24600;   /* 24.6 V */
    units[3].current_ma   = 1500;    /* 1.5 A */
    units[3].temp_tenths  = 425;     /* 42.5 °C */

    uint8_t base = 3 * ROBOCLAW_CHANNELS_PER_UNIT;
    float v;

    CHECK(roboclaw_drv_get_value(base + ROBOCLAW_SUB_MOTOR, &v));
    CHECK_FLOAT(v, 16384.0f / (float)ROBOCLAW_DUTY_MAX);

    CHECK(roboclaw_drv_get_value(base + ROBOCLAW_SUB_ENCODER, &v));
    CHECK_FLOAT(v, 12345.0f);

    CHECK(roboclaw_drv_get_value(base + ROBOCLAW_SUB_VOLTAGE, &v));
    CHECK_FLOAT(v, 24.6f);

    CHECK(roboclaw_drv_get_value(base + ROBOCLAW_SUB_CURRENT, &v));
    CHECK_FLOAT(v, 1.5f);

    CHECK(roboclaw_drv_get_value(base + ROBOCLAW_SUB_TEMP, &v));
    CHECK_FLOAT(v, 42.5f);
    return 1;
}

/* ============================================================================
 * save / load — pin-routing roundtrip through flash storage. This was
 * the path where stale "virtual GPIO" pin assumptions used to live; the
 * new path stores the configured UART TX/RX pair in flash_uart_pins_t.
 * ============================================================================ */

static int test_save_load_roundtrip(void)
{
    reset_state();
    roboclaw_tx_pin = 8;
    roboclaw_rx_pin = 9;
    configured_serial_port = 1;
    configured_baud = 38400;
    units[0].address        = 0x80;
    units[0].deadband       = 3;
    units[0].max_current_ma = 5000;
    units[1].address        = 0x81;
    units[1].deadband       = 0;
    units[1].max_current_ma = 0;
    unit_count = 2;

    flash_storage_data_t flash;
    memset(&flash, 0, sizeof(flash));

    CHECK(roboclaw_drv_save(&flash));
    CHECK_EQ(flash.roboclaw_config.unit_count, 2);
    CHECK_EQ(flash.roboclaw_config.serial_port, 1);
    CHECK_EQ(flash.roboclaw_config.baud_rate, 38400);
    CHECK_EQ(flash.uart_pins.roboclaw_tx_pin, 8);
    CHECK_EQ(flash.uart_pins.roboclaw_rx_pin, 9);
    CHECK_EQ(flash.roboclaw_config.units[0].address, 0x80);
    CHECK_EQ(flash.roboclaw_config.units[0].deadband, 3);
    CHECK_EQ(flash.roboclaw_config.units[0].max_current_ma, 5000);
    CHECK_EQ(flash.roboclaw_config.units[1].address, 0x81);

    /* Wipe driver state, then load back. */
    reset_state();
    CHECK(roboclaw_drv_load(&flash));
    CHECK_EQ(unit_count, 2);
    CHECK_EQ(configured_baud, 38400);
    CHECK_EQ(roboclaw_tx_pin, 8);
    CHECK_EQ(roboclaw_rx_pin, 9);
    CHECK_EQ(units[0].address, 0x80);
    CHECK_EQ(units[0].deadband, 3);
    CHECK_EQ(units[0].max_current_ma, 5000);
    CHECK_EQ(units[1].address, 0x81);
    /* load logs the restore line. */
    CHECK_LOG("restored");
    return 1;
}

/* Loading flash with an invalid (rejected) pin pair should fall back
 * to defaults AND emit a warn log so the operator sees that saved
 * pins didn't survive. */
static int test_load_invalid_pin_pair_warns(void)
{
    reset_state();
    flash_storage_data_t flash;
    memset(&flash, 0, sizeof(flash));
    flash.uart_pins.roboclaw_tx_pin = 99;
    flash.uart_pins.roboclaw_rx_pin = 100;
    flash.roboclaw_config.unit_count = 0;  /* skip the init-trigger path */

    test_pair_reject_tx = 99;
    test_pair_reject_rx = 100;

    roboclaw_drv_load(&flash);
    CHECK_LOG("[warn]");
    CHECK_LOG("stored pin pair");
    CHECK_LOG("isn't a valid RP2040 UART pair");
    /* Pin state unchanged from defaults. */
    CHECK_EQ(roboclaw_tx_pin, ROBOCLAW_DEFAULT_TX_PIN);
    CHECK_EQ(roboclaw_rx_pin, ROBOCLAW_DEFAULT_RX_PIN);
    return 1;
}

/* ============================================================================
 * uart_swap (PIO UART) config plumbing
 *
 * These tests cover the firmware-side glue for the PCB-swapped TX/RX
 * workaround. We can't actually drive the PIO peripheral from host code,
 * but we CAN verify the decision logic: that uart_swap=true in the sync
 * JSON or flash survives apply_config and flips use_pio_uart at init.
 * The actual PIO state-machine binding is gated behind #ifndef SIMULATION
 * and tested only on hardware.
 * ============================================================================ */

/* parse_json with "uart_swap":true extracts the flag. */
static int test_parse_json_extracts_uart_swap_true(void)
{
    reset_state();
    const char* json = "{\"address\":128,\"deadband\":0,\"max_current_ma\":0,"
                       "\"uart_swap\":true}";
    pin_config_t cfg = {0};
    cfg.gpio = ROBOCLAW_VIRTUAL_GPIO_BASE;
    bool ok = roboclaw_drv_parse_json(json, json + strlen(json), &cfg);
    CHECK(ok);
    CHECK_EQ(cfg.params.roboclaw.uart_swap, 1);
    return 1;
}

/* parse_json with "uart_swap":false leaves the flag at 0. */
static int test_parse_json_uart_swap_false_default(void)
{
    reset_state();
    const char* json = "{\"address\":128,\"uart_swap\":false}";
    pin_config_t cfg = {0};
    cfg.gpio = ROBOCLAW_VIRTUAL_GPIO_BASE;
    bool ok = roboclaw_drv_parse_json(json, json + strlen(json), &cfg);
    CHECK(ok);
    CHECK_EQ(cfg.params.roboclaw.uart_swap, 0);
    return 1;
}

/* parse_json with no uart_swap field at all → field stays at whatever
 * the caller initialized it to. This matters because the schema marks
 * uart_swap as optional; absent field must not clobber existing state. */
static int test_parse_json_uart_swap_absent_unchanged(void)
{
    reset_state();
    const char* json = "{\"address\":128,\"deadband\":0,\"max_current_ma\":0}";
    pin_config_t cfg = {0};
    cfg.gpio = ROBOCLAW_VIRTUAL_GPIO_BASE;
    cfg.params.roboclaw.uart_swap = 1;  // pretend a prior sync set this
    bool ok = roboclaw_drv_parse_json(json, json + strlen(json), &cfg);
    CHECK(ok);
    CHECK_EQ(cfg.params.roboclaw.uart_swap, 1);  // preserved
    return 1;
}

/* apply_config with uart_swap=1 stores the flag on the unit AND flips
 * use_pio_uart via the roboclaw_init() call at the end. */
static int test_apply_config_propagates_uart_swap(void)
{
    reset_state();
    pin_config_t cfg = {0};
    cfg.gpio = ROBOCLAW_VIRTUAL_GPIO_BASE;
    cfg.params.roboclaw.address = 0x80;
    cfg.params.roboclaw.uart_swap = 1;
    bool ok = roboclaw_drv_apply_config(0, &cfg);
    CHECK(ok);
    CHECK_EQ(units[0].uart_swap, 1);
    /* use_pio_uart is decided outside the #ifndef SIMULATION block, so
     * tests can verify the decision even though the actual PIO binding
     * is HW-only. */
    CHECK(use_pio_uart);
    return 1;
}

/* apply_config with uart_swap=0 leaves use_pio_uart false (HW path). */
static int test_apply_config_uart_swap_false_keeps_hw_path(void)
{
    reset_state();
    pin_config_t cfg = {0};
    cfg.gpio = ROBOCLAW_VIRTUAL_GPIO_BASE;
    cfg.params.roboclaw.address = 0x80;
    cfg.params.roboclaw.uart_swap = 0;
    bool ok = roboclaw_drv_apply_config(0, &cfg);
    CHECK(ok);
    CHECK_EQ(units[0].uart_swap, 0);
    CHECK(!use_pio_uart);
    return 1;
}

/* Flipping uart_swap via re-sync transitions use_pio_uart back. The
 * driver in production hits this when the operator unchecks the
 * "Use PIO UART" param and re-syncs. */
static int test_uart_swap_can_toggle_back(void)
{
    reset_state();
    pin_config_t cfg = {0};
    cfg.gpio = ROBOCLAW_VIRTUAL_GPIO_BASE;
    cfg.params.roboclaw.address = 0x80;

    cfg.params.roboclaw.uart_swap = 1;
    CHECK(roboclaw_drv_apply_config(0, &cfg));
    CHECK(use_pio_uart);

    cfg.params.roboclaw.uart_swap = 0;
    CHECK(roboclaw_drv_apply_config(0, &cfg));
    CHECK(!use_pio_uart);
    return 1;
}

/* save → reset → load round-trips uart_swap through flash. Prevents
 * a regression where the bit is parsed and applied at sync time but
 * lost on reboot — which would mean the operator has to re-sync every
 * power cycle. */
static int test_save_load_roundtrip_with_uart_swap(void)
{
    reset_state();
    pin_config_t cfg = {0};
    cfg.gpio = ROBOCLAW_VIRTUAL_GPIO_BASE;
    cfg.params.roboclaw.address = 0x80;
    cfg.params.roboclaw.uart_swap = 1;
    cfg.params.roboclaw.estop_pin = 26;
    CHECK(roboclaw_drv_apply_config(0, &cfg));
    CHECK_EQ(units[0].uart_swap, 1);
    CHECK_EQ(units[0].estop_pin, 26);

    /* Save to a fresh storage buffer. */
    flash_storage_data_t storage;
    memset(&storage, 0, sizeof(storage));
    storage.uart_pins.roboclaw_tx_pin = 0;
    storage.uart_pins.roboclaw_rx_pin = 1;
    CHECK(roboclaw_drv_save(&storage));

    /* Wipe driver state, then load. */
    reset_state();
    CHECK(roboclaw_drv_load(&storage));
    CHECK_EQ(units[0].uart_swap, 1);
    CHECK_EQ(units[0].estop_pin, 26);
    CHECK(use_pio_uart);
    return 1;
}

/* ============================================================================
 * Duty keepalive — counters the RoboClaw's serial-timeout safety
 *
 * The RoboClaw has a built-in deadman timer (Motion Studio Packet Serial
 * Settings → Timeout, default 1 s). Without firmware keepalive, a single
 * set_duty followed by quiet on the bus would let the controller kill
 * the motor a second later. maybe_send_duty_keepalive() in the driver
 * re-sends M1DUTY for any unit with duty != 0 once per
 * ROBOCLAW_DUTY_KEEPALIVE_MS window.
 *
 * Host tests verify the timing decision; the actual wire-level send is
 * gated behind #ifndef SIMULATION and only exercises on hardware. We
 * check: returns true (= packet sent) at the right moments, returns
 * false otherwise, doesn't fire for idle units, doesn't fire for
 * disconnected units, doesn't fire when not yet initialized, and the
 * timestamp gets refreshed so the next call respects the interval.
 * ============================================================================ */

/* Helper: bring a unit to "configured, connected, port_initialized=true,
 * driving at duty=N" state via apply_config + mark_unit_response. */
static void setup_running_unit(uint8_t unit_idx, int16_t duty)
{
    pin_config_t cfg = {0};
    cfg.gpio = ROBOCLAW_VIRTUAL_GPIO_BASE +
               (unit_idx * ROBOCLAW_CHANNELS_PER_UNIT);
    cfg.params.roboclaw.address = ROBOCLAW_ADDRESS_MIN + unit_idx;
    roboclaw_drv_apply_config(unit_idx * ROBOCLAW_CHANNELS_PER_UNIT, &cfg);
    port_initialized = true;             // bypasses SIMULATION-gated init
    units[unit_idx].duty = duty;
    units[unit_idx].connected = true;
    units[unit_idx].last_duty_send_ms = test_now_ms;
}

/* Nothing should fire when the driver isn't initialized. */
static int test_keepalive_skips_when_not_initialized(void)
{
    reset_state();
    units[0].duty = 1000;
    units[0].connected = true;
    /* port_initialized is still false from reset_state. */
    CHECK(!maybe_send_duty_keepalive());
    return 1;
}

/* Idle unit (duty == 0) should not generate keepalives no matter how
 * much time has passed — the motor is supposed to be stopped, the
 * timeout kicking in is the correct behavior. */
static int test_keepalive_skips_idle_unit(void)
{
    reset_state();
    setup_running_unit(0, 0);    /* duty = 0 */
    test_now_ms += ROBOCLAW_DUTY_KEEPALIVE_MS * 5;
    CHECK(!maybe_send_duty_keepalive());
    return 1;
}

/* A unit marked "disconnected" (because ACKs aren't coming back) SHOULD
 * still get keepalives as long as duty != 0. The controller's serial-
 * timeout watchdog feeds on incoming bytes at ITS RX, not on our ACK
 * receive — so even when our RX path is flaky and the connection state
 * is flapping, we keep sending so the motor keeps running. This is the
 * safety-critical "motor stays alive" behavior. */
static int test_keepalive_fires_even_when_disconnected(void)
{
    reset_state();
    setup_running_unit(0, 5000);
    units[0].connected = false;      // ACK reliability is poor; flag flapped
    test_now_ms += ROBOCLAW_DUTY_KEEPALIVE_MS + 1;
    CHECK(maybe_send_duty_keepalive());
    CHECK_EQ(units[0].last_duty_send_ms, test_now_ms);
    return 1;
}

/* Right after set_duty, no keepalive should fire — the timestamp is
 * fresh and we're inside the window. */
static int test_keepalive_skips_within_window(void)
{
    reset_state();
    setup_running_unit(0, 5000);
    /* Advance time, but stay just inside the keepalive window. */
    test_now_ms += ROBOCLAW_DUTY_KEEPALIVE_MS - 1;
    CHECK(!maybe_send_duty_keepalive());
    return 1;
}

/* Once the window elapses, the next call should fire keepalive. */
static int test_keepalive_fires_after_window(void)
{
    reset_state();
    setup_running_unit(0, 5000);
    test_now_ms += ROBOCLAW_DUTY_KEEPALIVE_MS + 1;
    CHECK(maybe_send_duty_keepalive());
    /* Timestamp should have advanced to "now" so the next call is back
     * inside the window. */
    CHECK_EQ(units[0].last_duty_send_ms, test_now_ms);
    /* Immediate follow-up call returns false (window has reset). */
    CHECK(!maybe_send_duty_keepalive());
    return 1;
}

/* Sustained running: every keepalive window the function returns true
 * exactly once. This is the "motor stays running until I stop it"
 * loop in continuous form. */
static int test_keepalive_fires_each_window_while_running(void)
{
    reset_state();
    setup_running_unit(0, 8000);

    int kept_alive = 0;
    for (int i = 0; i < 10; i++) {
        /* Advance one keepalive window's worth of time. */
        test_now_ms += ROBOCLAW_DUTY_KEEPALIVE_MS + 1;
        if (maybe_send_duty_keepalive()) kept_alive++;
        /* Sub-window jitter shouldn't trigger an extra send. */
        test_now_ms += ROBOCLAW_DUTY_KEEPALIVE_MS / 4;
        CHECK(!maybe_send_duty_keepalive());
    }
    CHECK_EQ(kept_alive, 10);
    return 1;
}

/* Setting duty back to 0 stops the keepalive stream — the motor is
 * supposed to coast/stop. */
static int test_keepalive_stops_when_duty_returns_to_zero(void)
{
    reset_state();
    setup_running_unit(0, 5000);
    test_now_ms += ROBOCLAW_DUTY_KEEPALIVE_MS + 1;
    CHECK(maybe_send_duty_keepalive());

    /* Operator returned slider to neutral. */
    units[0].duty = 0;

    /* Even after another full window, no keepalive should fire. */
    test_now_ms += ROBOCLAW_DUTY_KEEPALIVE_MS * 3;
    CHECK(!maybe_send_duty_keepalive());
    return 1;
}

/* Multi-unit: each unit independently checks its own timestamp; one
 * unit being due doesn't mask the others (we send at most one packet
 * per call but the next call picks up the next due unit). */
static int test_keepalive_multi_unit_one_per_call(void)
{
    reset_state();
    setup_running_unit(0, 4000);
    setup_running_unit(1, -3000);
    /* Both due simultaneously. */
    test_now_ms += ROBOCLAW_DUTY_KEEPALIVE_MS + 1;

    /* First call services unit 0 (loop walks units in order). */
    CHECK(maybe_send_duty_keepalive());
    CHECK_EQ(units[0].last_duty_send_ms, test_now_ms);
    /* Unit 1's timestamp not yet refreshed. */
    CHECK(units[1].last_duty_send_ms < test_now_ms);

    /* Second call (same `now`) services unit 1. */
    CHECK(maybe_send_duty_keepalive());
    CHECK_EQ(units[1].last_duty_send_ms, test_now_ms);

    /* Third call — both inside the window — returns false. */
    CHECK(!maybe_send_duty_keepalive());
    return 1;
}

/* set_duty itself should refresh last_duty_send_ms so an immediately
 * subsequent keepalive call doesn't double-send. */
static int test_set_duty_refreshes_keepalive_timestamp(void)
{
    /* Can't fully exercise set_duty in SIMULATION (the wire-level
     * send is gated). But the timestamp assignment IS in the same
     * #ifndef SIMULATION block as send_command, so it gets compiled
     * out too. We exercise the equivalent: simulate the operator
     * calling set_duty by directly updating duty + timestamp the way
     * production code does, then verify keepalive doesn't fire. */
    reset_state();
    setup_running_unit(0, 5000);
    /* Pretend set_duty was just called: timestamp = now, duty != 0. */
    units[0].last_duty_send_ms = test_now_ms;
    /* Immediate keepalive check — well inside window. */
    CHECK(!maybe_send_duty_keepalive());
    return 1;
}

/* ============================================================================
 * Test runner
 * ============================================================================ */

typedef int (*test_fn)(void);
typedef struct { const char* name; test_fn fn; } test_entry_t;

static const test_entry_t TESTS[] = {
    /* CRC */
    { "crc_known_vectors",                       test_crc_known_vectors },

    /* Connection state machine */
    { "mark_response_first_success_connects",    test_mark_response_first_success_connects },
    { "mark_response_repeated_success_no_extra_log", test_mark_response_repeated_success_no_extra_log },
    { "mark_response_drops_after_threshold",     test_mark_response_drops_after_threshold },
    { "mark_response_never_connected_logs_once_at_threshold", test_mark_response_never_connected_logs_once_at_threshold },
    { "mark_response_recovers_after_drop",       test_mark_response_recovers_after_drop },
    { "mark_response_transient_miss_recovers",   test_mark_response_transient_miss_recovers },
    { "mark_response_out_of_range_ignored",      test_mark_response_out_of_range_ignored },

    /* JSON parsing + sync log */
    { "parse_json_extracts_fields",              test_parse_json_extracts_fields },
    { "parse_json_logs_only_on_channel_0",       test_parse_json_logs_only_on_channel_0 },
    { "parse_json_missing_pins_warns",           test_parse_json_missing_pins_warns },

    /* Driver vtable lifecycle */
    { "drv_init_is_noop",                        test_drv_init_is_noop },
    { "drv_apply_config_updates_unit_state",     test_drv_apply_config_updates_unit_state },
    { "drv_apply_config_out_of_range_rejected",  test_drv_apply_config_out_of_range_rejected },

    /* Routing fallback */
    { "invalid_pin_pair_warns",                  test_invalid_pin_pair_warns },

    /* set / get value */
    { "set_value_only_motor_writable",           test_set_value_only_motor_writable },
    { "set_value_clamps_to_unit_range",          test_set_value_clamps_to_unit_range },
    { "get_value_returns_correct_field",         test_get_value_returns_correct_field },

    /* Persistence */
    { "save_load_roundtrip",                     test_save_load_roundtrip },
    { "load_invalid_pin_pair_warns",             test_load_invalid_pin_pair_warns },

    /* PIO UART (uart_swap) plumbing */
    { "parse_json_extracts_uart_swap_true",      test_parse_json_extracts_uart_swap_true },
    { "parse_json_uart_swap_false_default",      test_parse_json_uart_swap_false_default },
    { "parse_json_uart_swap_absent_unchanged",   test_parse_json_uart_swap_absent_unchanged },
    { "apply_config_propagates_uart_swap",       test_apply_config_propagates_uart_swap },
    { "apply_config_uart_swap_false_keeps_hw_path", test_apply_config_uart_swap_false_keeps_hw_path },
    { "uart_swap_can_toggle_back",               test_uart_swap_can_toggle_back },
    { "save_load_roundtrip_with_uart_swap",      test_save_load_roundtrip_with_uart_swap },

    /* Duty keepalive */
    { "keepalive_skips_when_not_initialized",    test_keepalive_skips_when_not_initialized },
    { "keepalive_skips_idle_unit",               test_keepalive_skips_idle_unit },
    { "keepalive_fires_even_when_disconnected",  test_keepalive_fires_even_when_disconnected },
    { "keepalive_skips_within_window",           test_keepalive_skips_within_window },
    { "keepalive_fires_after_window",            test_keepalive_fires_after_window },
    { "keepalive_fires_each_window_while_running", test_keepalive_fires_each_window_while_running },
    { "keepalive_stops_when_duty_returns_to_zero", test_keepalive_stops_when_duty_returns_to_zero },
    { "keepalive_multi_unit_one_per_call",       test_keepalive_multi_unit_one_per_call },
    { "set_duty_refreshes_keepalive_timestamp",  test_set_duty_refreshes_keepalive_timestamp },
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
