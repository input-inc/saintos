/**
 * Host-runnable tests for the generic switch / sensor input driver.
 *
 * Build/run: `./run_tests.sh` (in this directory). Same shape as
 * test_kangaroo_driver.c — stub the platform/log/manager headers, then
 * `#include "../../shared/src/switch_input_driver.c"` so static state
 * and helpers are reachable.
 *
 * The interesting behaviour here is all timing and edges: debounce,
 * latch, hysteresis, and the local interlock fan-out. See
 * docs/SENSOR_INPUTS.md.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <assert.h>

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

/* ── Pin-read stubs (the per-platform half) ────────────────────── */
static bool     stub_digital_level = false;
static uint16_t stub_analog_mv     = 0;
static bool     stub_analog_ok     = true;

bool switch_input_read_digital_pin(uint8_t pin, bool pull_up)
{
    (void)pin; (void)pull_up;
    return stub_digital_level;
}
bool switch_input_read_analog_mv(uint8_t pin, uint16_t* out_mv)
{
    (void)pin;
    if (!stub_analog_ok) return false;
    if (out_mv) *out_mv = stub_analog_mv;
    return true;
}

/* ── peripheral_manager stubs ──────────────────────────────────── */
/* Record what the interlock tried to stop, and let a test decide
 * whether the target "exists". */
static char stopped_ids[8][32];
static int  stopped_count = 0;
static int  estop_all_count = 0;
static bool stub_dispatch_result = true;

bool peripheral_dispatch_command(const char* peripheral_id, const char* command,
                                 const char* args_json, const char* args_json_end)
{
    (void)args_json; (void)args_json_end;
    if (stopped_count < 8 && strcmp(command, "estop") == 0) {
        snprintf(stopped_ids[stopped_count], sizeof(stopped_ids[0]),
                 "%s", peripheral_id);
        stopped_count++;
    }
    return stub_dispatch_result;
}
void peripheral_estop_all(void) { estop_all_count++; }
int peripheral_state_append_channel(char* buf, size_t cap, bool* first,
                                    const char* peripheral_id,
                                    const char* channel_id, float value)
{
    (void)peripheral_id; (void)channel_id; (void)value;
    if (first) *first = false;
    if (cap > 0 && buf) buf[0] = '\0';
    return 0;
}

/* ── Pull in the driver ────────────────────────────────────────── */
#include "../../shared/src/switch_input_driver.c"

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
    last_poll_ms = 0;
    pending_target_count = 0;
    test_now_ms = 1000;
    log_count = 0;
    stub_digital_level = false;
    stub_analog_mv = 0;
    stub_analog_ok = true;
    stopped_count = 0;
    estop_all_count = 0;
    stub_dispatch_result = true;
}

/* Configure unit 0 directly — apply_config's own path is covered
 * separately by the JSON tests. */
static void mkunit(uint8_t sense, bool active_low, bool latch,
                   uint16_t debounce_ms, uint8_t on_trip)
{
    switch_unit_t* u = &units[0];
    u->configured   = true;
    snprintf(u->peripheral_id, sizeof(u->peripheral_id), "limit-1");
    u->pin          = 26;
    u->sense        = sense;
    u->active_low   = active_low;
    u->latch        = latch;
    u->debounce_ms  = debounce_ms;
    u->threshold_mv = SWITCH_DEFAULT_THRESHOLD_MV;
    u->hysteresis_mv = SWITCH_DEFAULT_HYSTERESIS_MV;
    u->on_trip      = on_trip;
    unit_count = 1;
}

/* Advance the clock and poll, in poll-interval steps, so debounce timing
 * behaves as it would on a real loop. */
static void advance(uint32_t ms)
{
    for (uint32_t t = 0; t < ms; t += SWITCH_POLL_INTERVAL_MS) {
        test_now_ms += SWITCH_POLL_INTERVAL_MS;
        switch_input_update();
    }
}

/* ── Debounce ──────────────────────────────────────────────────── */

static int test_debounce_suppresses_short_glitch(void)
{
    reset_state();
    mkunit(SWITCH_SENSE_DIGITAL, false, false, 20, SWITCH_TRIP_NONE);

    stub_digital_level = true;      /* glitch high... */
    advance(10);
    stub_digital_level = false;     /* ...gone before the window closes */
    advance(30);

    CHECK(!units[0].state);
    CHECK_EQ(units[0].trip_count, 0);
    return 1;
}

static int test_debounce_accepts_stable_assert(void)
{
    reset_state();
    mkunit(SWITCH_SENSE_DIGITAL, false, false, 20, SWITCH_TRIP_NONE);

    stub_digital_level = true;
    advance(50);

    CHECK(units[0].state);
    CHECK_EQ(units[0].trip_count, 1);
    return 1;
}

/* A magnet sweeping past a reed is a pulse. As long as it is longer than
 * the debounce window it must be caught — this is the whole reason
 * debounce and latch live in firmware rather than server-side. */
static int test_brief_pulse_is_latched(void)
{
    reset_state();
    mkunit(SWITCH_SENSE_DIGITAL, false, true, 5, SWITCH_TRIP_NONE);

    stub_digital_level = true;
    advance(20);                    /* pulse present */
    stub_digital_level = false;
    advance(50);                    /* long gone */

    CHECK(!units[0].state);         /* live state follows the pin */
    CHECK(units[0].latched);        /* but the event is retained */
    CHECK_EQ(units[0].trip_count, 1);
    return 1;
}

static int test_release_is_not_a_trip(void)
{
    reset_state();
    mkunit(SWITCH_SENSE_DIGITAL, false, false, 5, SWITCH_TRIP_STOP_TARGETS);
    snprintf(units[0].targets[0], 32, "kangaroo-1");
    units[0].target_count = 1;

    stub_digital_level = true;
    advance(30);
    CHECK_EQ(stopped_count, 1);

    stub_digital_level = false;
    advance(30);
    CHECK_EQ(stopped_count, 1);     /* releasing must not re-fire */
    return 1;
}

/* ── Polarity ──────────────────────────────────────────────────── */

/* The PSR-2 is normally CLOSED: it asserts when the contact OPENS, so a
 * severed cable reads as tripped. That is the correct fail-safe
 * direction and the opposite of a typical normally-open input. */
static int test_active_low_asserts_on_open(void)
{
    reset_state();
    mkunit(SWITCH_SENSE_DIGITAL, true, false, 5, SWITCH_TRIP_NONE);

    stub_digital_level = true;      /* contact closed, pin high */
    advance(30);
    CHECK(!units[0].state);

    stub_digital_level = false;     /* contact opens (or cable cut) */
    advance(30);
    CHECK(units[0].state);
    return 1;
}

/* ── Analog sense + hysteresis ─────────────────────────────────── */

static int test_analog_threshold_crossing(void)
{
    reset_state();
    mkunit(SWITCH_SENSE_ANALOG, false, false, 5, SWITCH_TRIP_NONE);

    stub_analog_mv = 1700;          /* sensor conducting — below */
    advance(30);
    CHECK(!units[0].state);

    stub_analog_mv = 3300;          /* open circuit — above */
    advance(30);
    CHECK(units[0].state);
    return 1;
}

/* Without hysteresis a sensor resting at the threshold would chatter,
 * and every chatter edge is a trip. */
static int test_analog_hysteresis_blocks_chatter(void)
{
    reset_state();
    mkunit(SWITCH_SENSE_ANALOG, false, false, 5, SWITCH_TRIP_NONE);
    units[0].threshold_mv  = 2500;
    units[0].hysteresis_mv = 200;

    stub_analog_mv = 2600;          /* above threshold but inside the band */
    advance(30);
    CHECK(!units[0].state);         /* needs >= 2700 to assert */

    stub_analog_mv = 2750;
    advance(30);
    CHECK(units[0].state);

    stub_analog_mv = 2450;          /* below threshold but inside the band */
    advance(30);
    CHECK(units[0].state);          /* needs <= 2300 to release */

    stub_analog_mv = 2200;
    advance(30);
    CHECK(!units[0].state);
    return 1;
}

/* A failed ADC read must hold the last state, not read as 0 V — which
 * with active_low would look like an assert and stop the machine. */
static int test_analog_read_failure_holds_state(void)
{
    reset_state();
    mkunit(SWITCH_SENSE_ANALOG, false, false, 5, SWITCH_TRIP_NONE);

    stub_analog_mv = 3300;
    advance(30);
    CHECK(units[0].state);

    stub_analog_ok = false;
    advance(50);
    CHECK(units[0].state);          /* unchanged */
    return 1;
}

/* ── Local interlock ───────────────────────────────────────────── */

static int test_trip_stops_all_targets(void)
{
    reset_state();
    mkunit(SWITCH_SENSE_DIGITAL, false, true, 5, SWITCH_TRIP_STOP_TARGETS);
    snprintf(units[0].targets[0], 32, "kangaroo-1");
    snprintf(units[0].targets[1], 32, "roboclaw-2");
    units[0].target_count = 2;

    stub_digital_level = true;
    advance(30);

    CHECK_EQ(stopped_count, 2);
    CHECK(strcmp(stopped_ids[0], "kangaroo-1") == 0);
    CHECK(strcmp(stopped_ids[1], "roboclaw-2") == 0);
    return 1;
}

/* A target nobody claims is a configuration error the operator must see
 * — silently not stopping is the worst possible outcome here. */
static int test_unclaimed_target_logs_error(void)
{
    reset_state();
    mkunit(SWITCH_SENSE_DIGITAL, false, true, 5, SWITCH_TRIP_STOP_TARGETS);
    snprintf(units[0].targets[0], 32, "ghost-9");
    units[0].target_count = 1;
    stub_dispatch_result = false;

    stub_digital_level = true;
    advance(30);

    CHECK_LOG("could NOT stop");
    return 1;
}

static int test_stop_targets_with_no_targets_warns(void)
{
    reset_state();
    mkunit(SWITCH_SENSE_DIGITAL, false, true, 5, SWITCH_TRIP_STOP_TARGETS);
    units[0].target_count = 0;

    stub_digital_level = true;
    advance(30);

    CHECK_EQ(stopped_count, 0);
    CHECK_LOG("no targets configured");
    return 1;
}

static int test_estop_node_calls_estop_all(void)
{
    reset_state();
    mkunit(SWITCH_SENSE_DIGITAL, false, true, 5, SWITCH_TRIP_ESTOP_NODE);

    stub_digital_level = true;
    advance(30);

    CHECK_EQ(estop_all_count, 1);
    CHECK_EQ(stopped_count, 0);
    return 1;
}

static int test_trip_none_reports_only(void)
{
    reset_state();
    mkunit(SWITCH_SENSE_DIGITAL, false, true, 5, SWITCH_TRIP_NONE);
    snprintf(units[0].targets[0], 32, "kangaroo-1");
    units[0].target_count = 1;

    stub_digital_level = true;
    advance(30);

    CHECK(units[0].latched);        /* still recorded... */
    CHECK_EQ(stopped_count, 0);     /* ...but nothing stopped */
    CHECK_EQ(estop_all_count, 0);
    return 1;
}

/* ── Latch clearing ────────────────────────────────────────────── */

static int test_clear_latch_refused_while_asserted(void)
{
    reset_state();
    mkunit(SWITCH_SENSE_DIGITAL, false, true, 5, SWITCH_TRIP_NONE);

    stub_digital_level = true;
    advance(30);
    CHECK(units[0].latched);

    CHECK(!switch_input_clear_latch(0));
    CHECK(units[0].latched);
    CHECK_LOG("still asserted");
    return 1;
}

static int test_clear_latch_succeeds_once_released(void)
{
    reset_state();
    mkunit(SWITCH_SENSE_DIGITAL, false, true, 5, SWITCH_TRIP_NONE);

    stub_digital_level = true;
    advance(30);
    stub_digital_level = false;
    advance(30);

    CHECK(units[0].latched);
    CHECK(switch_input_clear_latch(0));
    CHECK(!units[0].latched);
    CHECK(!switch_input_any_latched());
    return 1;
}

/* ── Startup seeding ───────────────────────────────────────────── */

/* An input already asserted at boot must be seen immediately. Waiting
 * for an edge would silently arm a machine with a limit already tripped. */
static int test_init_seeds_already_asserted(void)
{
    reset_state();
    mkunit(SWITCH_SENSE_DIGITAL, false, true, 20, SWITCH_TRIP_NONE);
    stub_digital_level = true;

    switch_input_init();

    CHECK(units[0].state);
    CHECK(units[0].latched);
    CHECK_LOG("ALREADY asserted");
    return 1;
}

/* ── JSON param parsing ────────────────────────────────────────── */

static int test_parse_json_reads_params_and_targets(void)
{
    reset_state();
    pin_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    const char* json =
        "{\"sense_analog\":true,\"active_low\":false,\"latch\":true,"
        "\"debounce_ms\":25,\"threshold_mv\":2400,\"hysteresis_mv\":150,"
        "\"on_trip\":1,\"targets\":[\"kangaroo-1\",\"roboclaw-2\"]}";
    CHECK(drv_parse_json(json, json + strlen(json), &cfg));

    CHECK_EQ(cfg.params.switch_input.sense, SWITCH_SENSE_ANALOG);
    CHECK_EQ(cfg.params.switch_input.active_low, 0);
    CHECK_EQ(cfg.params.switch_input.latch, 1);
    CHECK_EQ(cfg.params.switch_input.debounce_ms, 25);
    CHECK_EQ(cfg.params.switch_input.threshold_mv, 2400);
    CHECK_EQ(cfg.params.switch_input.hysteresis_mv, 150);
    CHECK_EQ(cfg.params.switch_input.on_trip, SWITCH_TRIP_STOP_TARGETS);

    CHECK_EQ(pending_target_count, 2);
    CHECK(strcmp(pending_targets[0], "kangaroo-1") == 0);
    CHECK(strcmp(pending_targets[1], "roboclaw-2") == 0);
    return 1;
}

/* The targets array must not run on into a later param's strings. */
static int test_parse_json_targets_stop_at_bracket(void)
{
    reset_state();
    pin_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    const char* json =
        "{\"targets\":[\"kangaroo-1\"],\"label\":\"Front limit\","
        "\"note\":\"do not touch\"}";
    CHECK(drv_parse_json(json, json + strlen(json), &cfg));

    CHECK_EQ(pending_target_count, 1);
    CHECK(strcmp(pending_targets[0], "kangaroo-1") == 0);
    return 1;
}

static int test_parse_json_clamps_out_of_range(void)
{
    reset_state();
    pin_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    const char* json = "{\"debounce_ms\":999999,\"on_trip\":99}";
    CHECK(drv_parse_json(json, json + strlen(json), &cfg));

    CHECK_EQ(cfg.params.switch_input.debounce_ms, 5000);
    /* An unrecognised action must fall back to the harmless one, not to
     * whatever integer happened to arrive. */
    CHECK_EQ(cfg.params.switch_input.on_trip, SWITCH_TRIP_NONE);
    return 1;
}

/* ── Channels are read-only ────────────────────────────────────── */

static int test_set_value_rejected(void)
{
    reset_state();
    mkunit(SWITCH_SENSE_DIGITAL, false, true, 5, SWITCH_TRIP_NONE);
    CHECK(!drv_set_value(SWITCH_SUB_STATE, 1.0f));
    CHECK(!drv_set_value(SWITCH_SUB_LATCHED, 0.0f));
    return 1;
}

static int test_get_value_exposes_channels(void)
{
    reset_state();
    mkunit(SWITCH_SENSE_ANALOG, false, true, 5, SWITCH_TRIP_NONE);
    stub_analog_mv = 3300;
    advance(30);

    float v = 0;
    CHECK(drv_get_value(SWITCH_SUB_STATE, &v));      CHECK_EQ((long)v, 1);
    CHECK(drv_get_value(SWITCH_SUB_LATCHED, &v));    CHECK_EQ((long)v, 1);
    CHECK(drv_get_value(SWITCH_SUB_TRIP_COUNT, &v)); CHECK_EQ((long)v, 1);
    CHECK(drv_get_value(SWITCH_SUB_VOLTAGE, &v));
    CHECK(v > 3.2f && v < 3.4f);
    return 1;
}

/* ── Runner ────────────────────────────────────────────────────── */

typedef int (*test_fn)(void);
typedef struct { const char* name; test_fn fn; } test_entry_t;

static const test_entry_t TESTS[] = {
    {"debounce_suppresses_short_glitch",  test_debounce_suppresses_short_glitch},
    {"debounce_accepts_stable_assert",    test_debounce_accepts_stable_assert},
    {"brief_pulse_is_latched",            test_brief_pulse_is_latched},
    {"release_is_not_a_trip",             test_release_is_not_a_trip},
    {"active_low_asserts_on_open",        test_active_low_asserts_on_open},
    {"analog_threshold_crossing",         test_analog_threshold_crossing},
    {"analog_hysteresis_blocks_chatter",  test_analog_hysteresis_blocks_chatter},
    {"analog_read_failure_holds_state",   test_analog_read_failure_holds_state},
    {"trip_stops_all_targets",            test_trip_stops_all_targets},
    {"unclaimed_target_logs_error",       test_unclaimed_target_logs_error},
    {"stop_targets_with_no_targets_warns", test_stop_targets_with_no_targets_warns},
    {"estop_node_calls_estop_all",        test_estop_node_calls_estop_all},
    {"trip_none_reports_only",            test_trip_none_reports_only},
    {"clear_latch_refused_while_asserted", test_clear_latch_refused_while_asserted},
    {"clear_latch_succeeds_once_released", test_clear_latch_succeeds_once_released},
    {"init_seeds_already_asserted",       test_init_seeds_already_asserted},
    {"parse_json_reads_params_and_targets", test_parse_json_reads_params_and_targets},
    {"parse_json_targets_stop_at_bracket", test_parse_json_targets_stop_at_bracket},
    {"parse_json_clamps_out_of_range",    test_parse_json_clamps_out_of_range},
    {"set_value_rejected",                test_set_value_rejected},
    {"get_value_exposes_channels",        test_get_value_exposes_channels},
};

int main(void)
{
    int passed = 0, failed = 0;
    size_t n = sizeof(TESTS) / sizeof(TESTS[0]);
    for (size_t i = 0; i < n; i++) {
        int ok = TESTS[i].fn();
        if (ok) { printf("  ok   %s\n", TESTS[i].name); passed++; }
        else    { printf("  FAIL %s\n", TESTS[i].name); failed++; }
    }
    printf("\n%d passed, %d failed (%zu total)\n", passed, failed, n);
    return failed == 0 ? 0 : 1;
}
