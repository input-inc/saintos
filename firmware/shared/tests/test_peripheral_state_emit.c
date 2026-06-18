/**
 * Host-runnable test for the peripheral-first state-emission helpers
 * in peripheral_manager.cpp (Phase 1 of
 * docs/PERIPHERAL_FIRST_MIGRATION.md).
 *
 * Strategy: short-circuit the shared platform.h stub (which #errors
 * by design) with a #define and a local PLATFORM_PRINTF, then
 * include peripheral_manager.cpp directly so the test exercises the
 * real implementation alongside a couple of fake drivers.
 *
 * Coverage:
 *   - peripheral_state_append_channel formats one record correctly
 *   - first==true emits no leading comma; first becomes false after
 *   - second call (first==false) prepends a comma
 *   - overflow returns -1 and doesn't write past cap
 *   - peripheral_state_emit_all_channels iterates registered drivers
 *   - drivers with state_emit_channels == NULL are skipped
 *   - drivers that emit multiple records share the array correctly
 *   - mixed (some NULL, some implemented) still produces valid JSON
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

/* ── platform.h short-circuit + PLATFORM_PRINTF stub ─────────────── */

#define PLATFORM_H   /* skip the shared stub header's #error */
#define PLATFORM_PRINTF(...) ((void)0)   /* silence registration logs */

/* peripheral_manager.cpp wraps no C++-only features (despite the
 * extension), so it compiles cleanly as C. */
#include "../src/peripheral_manager.cpp"

/* ── EXPECT macro: same shape as test_maestro_driver.c ────────────── */

static int failures = 0;
#define EXPECT(cond, label) do {                                            \
    if (!(cond)) {                                                          \
        fprintf(stderr, "  FAIL: %s  (%s:%d)\n", label, __FILE__, __LINE__);\
        failures++;                                                         \
    }                                                                       \
} while (0)

/* ── Fake driver fixtures ────────────────────────────────────────── */

/* Each fake driver emits a deterministic set of records when its
 * state_emit_channels callback fires. The test asserts the resulting
 * JSON fragments concatenated correctly. */

static int fake_emit_maestro(char* buf, size_t cap, bool* first)
{
    int total = 0;
    int n;
    n = peripheral_state_append_channel(buf + total, cap - (size_t)total, first,
                                        "maestro-1", "connected", 1.0f);
    if (n < 0) return -1;
    total += n;
    n = peripheral_state_append_channel(buf + total, cap - (size_t)total, first,
                                        "maestro-1", "errors", 0.0f);
    if (n < 0) return -1;
    total += n;
    return total;
}

static int fake_emit_syren(char* buf, size_t cap, bool* first)
{
    return peripheral_state_append_channel(buf, cap, first,
                                           "syren-1", "current", 2.5f);
}

static const peripheral_driver_t fake_maestro = {
    .name = "maestro", .mode_string = "maestro_servo",
    .state_emit_channels = fake_emit_maestro,
};
static const peripheral_driver_t fake_syren = {
    .name = "syren", .mode_string = "syren_motor",
    .state_emit_channels = fake_emit_syren,
};
static const peripheral_driver_t fake_legacy = {
    .name = "legacy", .mode_string = "legacy_mode",
    .state_emit_channels = NULL,   /* hasn't migrated */
};

/* ── Test helpers ────────────────────────────────────────────────── */

/* peripheral_manager.cpp keeps its registered drivers in a file-
 * scope static; there's no public unregister. Each test resets by
 * memset-ing the file-scope arrays directly (they're visible because
 * we #included the .cpp into this translation unit). */
extern const peripheral_driver_t* drivers[];
extern uint8_t driver_count;

static void reset_registry(void)
{
    memset(drivers, 0, sizeof(*drivers) * PERIPHERAL_MAX_DRIVERS);
    driver_count = 0;
}

/* ── Tests: peripheral_state_append_channel ──────────────────────── */

static void test_append_first_record(void)
{
    printf("test_append_first_record\n");
    char buf[256] = {0};
    bool first = true;
    int n = peripheral_state_append_channel(buf, sizeof(buf), &first,
                                            "maestro-1", "connected", 1.5f);
    EXPECT(n > 0,           "returns positive byte count");
    EXPECT(first == false,  "flips first to false");
    EXPECT(strstr(buf, "\"peripheral_id\":\"maestro-1\"") != NULL,
                            "peripheral_id present");
    EXPECT(strstr(buf, "\"channel_id\":\"connected\"") != NULL,
                            "channel_id present");
    EXPECT(strstr(buf, "1.5") != NULL,
                            "value rendered");
    EXPECT(buf[0] == '{',   "no leading comma when first==true");
}

static void test_append_subsequent_record(void)
{
    printf("test_append_subsequent_record\n");
    char buf[256] = {0};
    bool first = false;   /* simulate "not the first call this tick" */
    int n = peripheral_state_append_channel(buf, sizeof(buf), &first,
                                            "p", "c", 7.0f);
    EXPECT(n > 0,         "returns positive byte count");
    EXPECT(buf[0] == ',', "prepends a comma when first==false");
}

static void test_append_overflow_returns_neg(void)
{
    printf("test_append_overflow_returns_neg\n");
    char buf[8] = {0};   /* far too small for any real record */
    bool first = true;
    int n = peripheral_state_append_channel(buf, sizeof(buf), &first,
                                            "p", "c", 1.0f);
    EXPECT(n == -1,        "returns -1 on overflow");
    EXPECT(first == true,  "leaves first unchanged on failure");
}

static void test_append_rejects_null_args(void)
{
    printf("test_append_rejects_null_args\n");
    char buf[64];
    bool first = true;
    EXPECT(peripheral_state_append_channel(NULL, 64, &first, "p", "c", 1.0f) == -1,
           "null buf rejected");
    EXPECT(peripheral_state_append_channel(buf, 64, NULL, "p", "c", 1.0f) == -1,
           "null first rejected");
    EXPECT(peripheral_state_append_channel(buf, 64, &first, NULL, "c", 1.0f) == -1,
           "null peripheral_id rejected");
    EXPECT(peripheral_state_append_channel(buf, 64, &first, "p", NULL, 1.0f) == -1,
           "null channel_id rejected");
}

/* ── Tests: peripheral_state_emit_all_channels ───────────────────── */

static void test_emit_empty_registry(void)
{
    printf("test_emit_empty_registry\n");
    reset_registry();
    char buf[256] = {0};
    int n = peripheral_state_emit_all_channels(buf, sizeof(buf));
    EXPECT(n == 0,         "no drivers → no bytes written");
    EXPECT(buf[0] == '\0', "buffer untouched");
}

static void test_emit_single_driver(void)
{
    printf("test_emit_single_driver\n");
    reset_registry();
    peripheral_register(&fake_maestro);

    char buf[512] = {0};
    int n = peripheral_state_emit_all_channels(buf, sizeof(buf));
    EXPECT(n > 0,                                          "wrote some bytes");
    EXPECT(strstr(buf, "\"maestro-1\"") != NULL,           "maestro-1 record present");
    EXPECT(strstr(buf, "\"connected\"") != NULL,           "connected channel present");
    EXPECT(strstr(buf, "\"errors\"")    != NULL,           "errors channel present");
    /* Both records in one buffer should be separated by a comma. */
    EXPECT(strstr(buf, "},{") != NULL,                     "records separated by comma");
    EXPECT(buf[0] == '{',                                  "no leading comma overall");
}

static void test_emit_multiple_drivers(void)
{
    printf("test_emit_multiple_drivers\n");
    reset_registry();
    peripheral_register(&fake_maestro);
    peripheral_register(&fake_syren);

    char buf[512] = {0};
    int n = peripheral_state_emit_all_channels(buf, sizeof(buf));
    EXPECT(n > 0,                                          "wrote some bytes");
    /* All three records present across the two drivers. */
    EXPECT(strstr(buf, "\"connected\"") != NULL,           "maestro connected present");
    EXPECT(strstr(buf, "\"errors\"")    != NULL,           "maestro errors present");
    EXPECT(strstr(buf, "\"syren-1\"")   != NULL,           "syren record present");
    /* The shared `first` flag should keep the array a single
     * well-formed JSON sequence with N-1 commas for N records. */
    int comma_count = 0;
    for (const char* p = buf; *p; p++) if (*p == ',' && *(p+1) == '{') comma_count++;
    EXPECT(comma_count == 2,                               "exactly 2 inter-record commas for 3 records");
}

static void test_emit_skips_null_callback(void)
{
    printf("test_emit_skips_null_callback\n");
    reset_registry();
    peripheral_register(&fake_legacy);     /* state_emit_channels == NULL */
    peripheral_register(&fake_maestro);

    char buf[512] = {0};
    int n = peripheral_state_emit_all_channels(buf, sizeof(buf));
    EXPECT(n > 0,                                          "migrated driver still emits");
    EXPECT(strstr(buf, "\"maestro-1\"") != NULL,           "maestro-1 records present");
    /* And critically, the legacy driver didn't crash or insert a
     * stray comma. The maestro records are the only ones present. */
    EXPECT(buf[0] == '{',                                  "no leading comma from skipped driver");
}

static void test_emit_overflow_returns_neg(void)
{
    printf("test_emit_overflow_returns_neg\n");
    reset_registry();
    peripheral_register(&fake_maestro);
    char buf[16] = {0};
    int n = peripheral_state_emit_all_channels(buf, sizeof(buf));
    EXPECT(n == -1, "emit_all propagates -1 on overflow");
}

/* ── Main ─────────────────────────────────────────────────────────── */

int main(void)
{
    test_append_first_record();
    test_append_subsequent_record();
    test_append_overflow_returns_neg();
    test_append_rejects_null_args();

    test_emit_empty_registry();
    test_emit_single_driver();
    test_emit_multiple_drivers();
    test_emit_skips_null_callback();
    test_emit_overflow_returns_neg();

    if (failures == 0) {
        printf("PASS\n");
        return 0;
    } else {
        printf("FAIL: %d assertion(s) failed\n", failures);
        return 1;
    }
}
