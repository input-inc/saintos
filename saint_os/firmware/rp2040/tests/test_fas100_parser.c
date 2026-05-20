/**
 * Host-runnable test for the FAS100 driver's S.Port / FBUS parsers.
 *
 * Build/run: `./run_tests.sh` (in this directory) — uses cc on the host.
 *
 * Approach: pre-define a few header guards + stubs that the driver
 * needs, then #include "fas100_driver.c" so the test code is in the
 * SAME translation unit as the driver and can poke at static state
 * + call static functions. SIMULATION is defined so the hardware
 * UART/GPIO paths drop out.
 *
 * Tests focus on the parser layer — the parts that handle bytes
 * coming back from the sensor, including poll-echo on the shared
 * TX/RX wire, byte-stuffing, and re-alignment when garbage bytes
 * desync the frame accumulator. These are the exact failure modes
 * we observed on hardware ("first response (data_id=0x6602)" being
 * a lucky-CRC match on misaligned bytes).
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
 * saint_log stub (pretends to be firmware/shared/include/saint_log.h)
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

/* ============================================================================
 * uart_pin_pairs stub
 * ============================================================================ */
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

/* ============================================================================
 * Pull in the real driver. Hardware paths under #ifndef SIMULATION
 * are excluded, leaving the parser + state machine.
 * ============================================================================ */
#include "../src/fas100_driver.c"

/* ============================================================================
 * Test helpers
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

static void reset_state(void)
{
    rx_pos = 0;
    rx_in_stuff = false;
    sport_skip_addr = false;
    sensor_responded = false;
    current_amps = 0;
    voltage_volts = 0;
    temp1_celsius = 0;
    temp2_celsius = 0;
    last_response_ms = 0;
    last_byte_ms = 0;
    last_soft_resync_ms = 0;
    stat_polls_sent = 0;
    stat_echo_bytes = 0;
    stat_echo_missed = 0;
    stat_frames_ok = 0;
    stat_frames_crc_bad = 0;
    stat_resyncs = 0;
    stat_soft_resyncs = 0;
    stat_unknown_id = 0;
    log_count = 0;
    test_now_ms = 1000;
    port_initialized = true;
}

static void feed_sport(const uint8_t* bytes, size_t n)
{
    for (size_t i = 0; i < n; i++) sport_feed_byte(bytes[i]);
}

static void feed_fbus(const uint8_t* bytes, size_t n)
{
    for (size_t i = 0; i < n; i++) fbus_feed_byte(bytes[i]);
}

/* Build a valid S.Port response frame [0x10, dl, dh, v0, v1, v2, v3, crc]. */
static void build_sport_response(uint16_t data_id, uint32_t value, uint8_t out[8])
{
    out[0] = SPORT_DATA_HEADER;
    out[1] = (uint8_t)(data_id & 0xFF);
    out[2] = (uint8_t)(data_id >> 8);
    out[3] = (uint8_t)(value & 0xFF);
    out[4] = (uint8_t)(value >> 8);
    out[5] = (uint8_t)(value >> 16);
    out[6] = (uint8_t)(value >> 24);
    out[7] = sport_crc_calculate(out, 7);
}

/* Build a valid FBUS response frame
 * [0x08, sensor_id, 0x10, dl, dh, v0, v1, v2, v3, crc]. */
static void build_fbus_response(uint16_t data_id, uint32_t value, uint8_t out[10])
{
    out[0] = FBUS_LEN_BYTE;
    out[1] = FBUS_FAS100_SENSOR_ID;
    out[2] = FBUS_FRAME_ID_DATA;
    out[3] = (uint8_t)(data_id & 0xFF);
    out[4] = (uint8_t)(data_id >> 8);
    out[5] = (uint8_t)(value & 0xFF);
    out[6] = (uint8_t)(value >> 8);
    out[7] = (uint8_t)(value >> 16);
    out[8] = (uint8_t)(value >> 24);
    out[9] = sport_crc_calculate(out, 9);
}

/* ============================================================================
 * S.Port tests
 * ============================================================================ */

/* A clean response with the correct CRC parses and updates the right channel. */
static int test_sport_basic_current(void)
{
    reset_state();
    /* Current = 5.0 A → raw value 50 (×10), data_id = 0x0200. */
    uint8_t frame[8];
    build_sport_response(SPORT_DATA_ID_CURRENT, 50, frame);
    feed_sport(frame, 8);

    CHECK_EQ(stat_frames_ok, 1);
    CHECK_EQ(stat_frames_crc_bad, 0);
    CHECK_FLOAT(current_amps, 5.0f);
    CHECK(sensor_responded);
    return 1;
}

static int test_sport_basic_voltage(void)
{
    reset_state();
    /* Voltage = 12.34 V → raw value 1234 (×100), data_id = 0x0210. */
    uint8_t frame[8];
    build_sport_response(SPORT_DATA_ID_VOLTAGE, 1234, frame);
    feed_sport(frame, 8);

    CHECK_EQ(stat_frames_ok, 1);
    CHECK_FLOAT(voltage_volts, 12.34f);
    return 1;
}

/* A frame with a tampered CRC byte is rejected and channel storage
 * doesn't change. */
static int test_sport_bad_crc_rejected(void)
{
    reset_state();
    uint8_t frame[8];
    build_sport_response(SPORT_DATA_ID_CURRENT, 50, frame);
    frame[7] ^= 0xFF;  /* flip every CRC bit */
    feed_sport(frame, 8);

    CHECK_EQ(stat_frames_ok, 0);
    CHECK_EQ(stat_frames_crc_bad, 1);
    CHECK_FLOAT(current_amps, 0.0f);
    CHECK(!sensor_responded);
    return 1;
}

/* The standard happy-path on the wire: poll echo arrives in RX, then
 * the sensor response. 0x7E hard-resets the parser, the next byte is
 * the physical-ID (discarded), and the 8-byte frame parses cleanly. */
static int test_sport_echo_then_response(void)
{
    reset_state();
    uint8_t echo[2] = { SPORT_POLL_HEADER, SPORT_FAS100_PHYSICAL_ID };
    feed_sport(echo, 2);

    /* After the echo: rx_pos should be 0, sport_skip_addr should be false
     * (already consumed the address byte). */
    CHECK_EQ(rx_pos, 0);
    CHECK(!sport_skip_addr);

    uint8_t frame[8];
    build_sport_response(SPORT_DATA_ID_TEMP1, 25, frame);
    feed_sport(frame, 8);

    CHECK_EQ(stat_frames_ok, 1);
    CHECK_FLOAT(temp1_celsius, 25.0f);
    return 1;
}

/* Pre-load the parser with garbage (simulates a previous misaligned
 * frame), then send poll-echo + clean response. The 0x7E in the echo
 * should hard-reset the parser, count as a resync, and the response
 * should still parse correctly. This is the bug we're chasing on
 * hardware: the parser was lucky-CRC'ing misaligned bytes and accepting
 * frames with bogus data_ids (e.g. 0x6602). */
static int test_sport_resync_after_garbage(void)
{
    reset_state();
    uint8_t garbage[5] = { 0x42, 0x99, 0xAA, 0xBB, 0xCC };
    feed_sport(garbage, 5);

    /* Parser is now mid-frame with rx_pos=5 holding nonsense. */
    CHECK_EQ(rx_pos, 5);

    /* Echo arrives. 0x7E should trigger a resync. */
    uint8_t echo[2] = { SPORT_POLL_HEADER, SPORT_FAS100_PHYSICAL_ID };
    feed_sport(echo, 2);
    CHECK_EQ(stat_resyncs, 1);
    CHECK_EQ(rx_pos, 0);

    /* Then the response. Should parse cleanly thanks to the resync. */
    uint8_t frame[8];
    build_sport_response(SPORT_DATA_ID_VOLTAGE, 100, frame);
    feed_sport(frame, 8);
    CHECK_EQ(stat_frames_ok, 1);
    CHECK_FLOAT(voltage_volts, 1.0f);
    return 1;
}

/* Byte stuffing: a sensor payload value of 0x7E is sent as [0x7D, 0x5E]
 * on the wire. Parser must un-stuff and validate correctly. */
static int test_sport_byte_stuffing_payload(void)
{
    reset_state();
    /* Build a frame where one of the value bytes is 0x7E. */
    uint8_t logical[8];
    build_sport_response(SPORT_DATA_ID_CURRENT, 0x0000007E, logical);
    CHECK_EQ(logical[3], 0x7E);  /* v0 = 0x7E */

    /* Now byte-stuff it: replace 0x7E with [0x7D, 0x5E], 0x7D with
     * [0x7D, 0x5D]. CRC isn't stuffed (it's computed pre-stuffing). */
    uint8_t wire[16];
    size_t wire_n = 0;
    for (size_t i = 0; i < 8; i++) {
        uint8_t b = logical[i];
        if (b == SPORT_POLL_HEADER || b == SPORT_STUFF_MARKER) {
            wire[wire_n++] = SPORT_STUFF_MARKER;
            wire[wire_n++] = b ^ SPORT_STUFF_MASK;
        } else {
            wire[wire_n++] = b;
        }
    }

    feed_sport(wire, wire_n);
    CHECK_EQ(stat_frames_ok, 1);
    /* Value 0x7E → 126 / 10.0 = 12.6 A */
    CHECK_FLOAT(current_amps, 12.6f);
    return 1;
}

/* A CRC-valid response with a data_id outside the FAS100 set
 * (e.g. an upstream FAS100 firmware that started sending more IDs) is
 * counted as unknown_id but DOES refresh sensor_responded — we still
 * know the sensor is alive. */
static int test_sport_unknown_data_id(void)
{
    reset_state();
    uint8_t frame[8];
    build_sport_response(0x6602, 12345, frame);  /* the bogus ID we saw on hardware */
    feed_sport(frame, 8);

    CHECK_EQ(stat_frames_ok, 1);
    CHECK_EQ(stat_unknown_id, 1);
    CHECK(sensor_responded);
    /* No known channel should have been updated. */
    CHECK_FLOAT(current_amps, 0.0f);
    return 1;
}

/* Two responses back-to-back. The parser must reset after a complete
 * frame and accept the next one. */
static int test_sport_two_responses_back_to_back(void)
{
    reset_state();
    uint8_t f1[8], f2[8];
    build_sport_response(SPORT_DATA_ID_CURRENT, 100, f1);
    build_sport_response(SPORT_DATA_ID_VOLTAGE, 1200, f2);

    /* Wire: echo, response1, echo, response2 */
    uint8_t echo[2] = { SPORT_POLL_HEADER, SPORT_FAS100_PHYSICAL_ID };
    feed_sport(echo, 2);
    feed_sport(f1, 8);
    feed_sport(echo, 2);
    feed_sport(f2, 8);

    CHECK_EQ(stat_frames_ok, 2);
    CHECK_FLOAT(current_amps, 10.0f);
    CHECK_FLOAT(voltage_volts, 12.0f);
    return 1;
}

/* If a poll-echo's tail byte (the address byte 0x22) somehow leaks
 * past the drain and lands at the start of what the parser thinks is
 * a new frame, the parser's rx_buf[0]==SPORT_DATA_HEADER check will
 * reject it and the next 0x7E re-syncs. */
static int test_sport_leaked_addr_byte_rejected(void)
{
    reset_state();
    /* Address byte arrives without preceding 0x7E. Parser accumulates. */
    uint8_t leak[1] = { SPORT_FAS100_PHYSICAL_ID };
    feed_sport(leak, 1);
    CHECK_EQ(rx_pos, 1);
    CHECK(rx_buf[0] != SPORT_DATA_HEADER);

    /* Now a full valid response arrives. With rx_buf[0]=0x22 already in,
     * the parser will fill 7 more bytes (the full real response) and at
     * rx_pos=8 will see rx_buf[0]=0x22 != 0x10 → reject as crc_bad. */
    uint8_t frame[8];
    build_sport_response(SPORT_DATA_ID_CURRENT, 50, frame);
    feed_sport(frame, 8);
    CHECK_EQ(stat_frames_crc_bad, 1);
    CHECK_EQ(stat_frames_ok, 0);

    /* Next cycle: echo arrives, 0x7E resyncs. New response parses cleanly. */
    uint8_t echo[2] = { SPORT_POLL_HEADER, SPORT_FAS100_PHYSICAL_ID };
    feed_sport(echo, 2);
    feed_sport(frame, 8);
    CHECK_EQ(stat_frames_ok, 1);
    return 1;
}

/* A 0x7E arriving mid-frame (e.g. our next poll's echo while we were
 * still expecting more response bytes) hard-resets the parser. */
static int test_sport_mid_frame_sync(void)
{
    reset_state();
    /* Inject the first 4 bytes of a response, then an early 0x7E. */
    uint8_t partial[4] = { 0x10, 0x00, 0x02, 0xAA };
    feed_sport(partial, 4);
    CHECK_EQ(rx_pos, 4);

    /* Mid-frame 0x7E should reset the accumulator and count as resync. */
    sport_feed_byte(SPORT_POLL_HEADER);
    CHECK_EQ(rx_pos, 0);
    CHECK_EQ(stat_resyncs, 1);
    CHECK(sport_skip_addr);

    /* Send the address byte (skipped) then a full frame. */
    sport_feed_byte(SPORT_FAS100_PHYSICAL_ID);
    uint8_t frame[8];
    build_sport_response(SPORT_DATA_ID_TEMP2, 99, frame);
    feed_sport(frame, 8);
    CHECK_EQ(stat_frames_ok, 1);
    CHECK_FLOAT(temp2_celsius, 99.0f);
    return 1;
}

/* ============================================================================
 * FBUS tests
 * ============================================================================ */

static int test_fbus_basic_response(void)
{
    reset_state();
    uint8_t frame[10];
    build_fbus_response(SPORT_DATA_ID_VOLTAGE, 1500, frame);
    feed_fbus(frame, 10);
    CHECK_EQ(stat_frames_ok, 1);
    CHECK_FLOAT(voltage_volts, 15.0f);
    return 1;
}

/* FBUS poll-echo and response share the same 3-byte prefix
 * [0x08, sensor_id, 0x10]. With our drain working, the parser only
 * sees the response. Without the drain, the parser would consume the
 * echo as if it were the start of a response and CRC-fail. This test
 * verifies the parser's per-byte alignment on a clean response. */
static int test_fbus_parser_alignment(void)
{
    reset_state();
    uint8_t frame[10];
    build_fbus_response(SPORT_DATA_ID_TEMP1, 42, frame);

    /* Feed byte-by-byte and assert state transitions. */
    fbus_feed_byte(frame[0]);  /* 0x08 → state advances */
    CHECK_EQ(rx_pos, 1);

    fbus_feed_byte(frame[1]);  /* 0x22 */
    CHECK_EQ(rx_pos, 2);

    fbus_feed_byte(frame[2]);  /* 0x10 */
    CHECK_EQ(rx_pos, 3);

    for (int i = 3; i < 10; i++) fbus_feed_byte(frame[i]);
    CHECK_EQ(stat_frames_ok, 1);
    CHECK_FLOAT(temp1_celsius, 42.0f);
    return 1;
}

/* If the FBUS prefix is mis-started (a bad first byte that's not 0x08),
 * the parser stays in the wait-for-len state and discards. */
static int test_fbus_bad_prefix_dropped(void)
{
    reset_state();
    fbus_feed_byte(0x55);   /* not 0x08 → stay at rx_pos=0 */
    CHECK_EQ(rx_pos, 0);

    /* Now a real frame. Should parse. */
    uint8_t frame[10];
    build_fbus_response(SPORT_DATA_ID_CURRENT, 200, frame);
    feed_fbus(frame, 10);
    CHECK_EQ(stat_frames_ok, 1);
    CHECK_FLOAT(current_amps, 20.0f);
    return 1;
}

/* If we see 0x08 followed by NOT the sensor id, the parser should
 * recover. If the wrong byte is itself 0x08, restart with that as
 * the new len byte. */
static int test_fbus_restart_on_double_len(void)
{
    reset_state();
    fbus_feed_byte(FBUS_LEN_BYTE);  /* 0x08, rx_pos=1 */
    CHECK_EQ(rx_pos, 1);

    fbus_feed_byte(FBUS_LEN_BYTE);  /* 0x08 again — treat as new start */
    CHECK_EQ(rx_pos, 1);
    CHECK_EQ(rx_buf[0], FBUS_LEN_BYTE);
    return 1;
}

/* CRC is the same algorithm as S.Port (sum-with-carry-fold, complemented). */
static int test_crc_known_vectors(void)
{
    /* A frame of all zeros gives CRC = 0xFF. */
    uint8_t a[7] = { 0 };
    CHECK_EQ(sport_crc_calculate(a, 7), 0xFF);

    /* Single 0xFF gives CRC = 0x00. */
    uint8_t b[1] = { 0xFF };
    CHECK_EQ(sport_crc_calculate(b, 1), 0x00);

    /* Two 0x80s: 0x80+0x80 = 0x100 → carry-fold to 0x01 → ~0x01 = 0xFE. */
    uint8_t c[2] = { 0x80, 0x80 };
    CHECK_EQ(sport_crc_calculate(c, 2), 0xFE);
    return 1;
}

/* ============================================================================
 * Connection-stickiness + lost-link tests
 * ============================================================================ */

/* is_connected() reports false on a fresh boot before any frame has
 * been seen, even if some time has elapsed. */
static int test_is_connected_false_before_first_frame(void)
{
    reset_state();
    test_now_ms = 10000;
    CHECK(!fas100_is_connected());
    return 1;
}

/* After a valid frame, is_connected() is true. */
static int test_is_connected_after_first_frame(void)
{
    reset_state();
    uint8_t frame[8];
    build_sport_response(SPORT_DATA_ID_CURRENT, 50, frame);
    test_now_ms = 5000;
    feed_sport(frame, 8);
    CHECK(fas100_is_connected());
    return 1;
}

/* The sticky window keeps is_connected() returning true for
 * FAS100_STICKY_CONNECTED_MS after sensor_responded is cleared (as
 * happens when enter_phase() drops us out of PHASE_LOCKED for a
 * re-probe). Past the window, is_connected() returns false. */
static int test_is_connected_sticky_after_response_cleared(void)
{
    reset_state();
    uint8_t frame[8];
    build_sport_response(SPORT_DATA_ID_VOLTAGE, 1234, frame);
    test_now_ms = 5000;
    feed_sport(frame, 8);
    CHECK(fas100_is_connected());

    /* Simulate a re-probe: clear sensor_responded but leave the
     * last_response_ms timestamp behind. */
    sensor_responded = false;

    /* Still inside the sticky window — should report connected. */
    test_now_ms = 5000 + (FAS100_STICKY_CONNECTED_MS - 100);
    CHECK(fas100_is_connected());

    /* Past the sticky window — disconnected. */
    test_now_ms = 5000 + FAS100_STICKY_CONNECTED_MS + 100;
    CHECK(!fas100_is_connected());
    return 1;
}

/* sport_feed_byte does NOT update last_byte_ms — only the UART read
 * loop in fas100_update() does. So feeding bytes through the parser
 * in tests should not affect last_byte_ms. We exercise this by
 * manually setting last_byte_ms (as the real read loop would) and
 * confirming the byte_silence calculation in the LOCKED branch uses
 * it correctly. This is just a sanity check of the semantics. */
static int test_last_byte_ms_independent_of_feed(void)
{
    reset_state();
    last_byte_ms = 0;
    uint8_t frame[8];
    build_sport_response(SPORT_DATA_ID_CURRENT, 50, frame);
    feed_sport(frame, 8);
    CHECK_EQ(last_byte_ms, 0);  /* feed_sport doesn't touch it */
    return 1;
}

/* ============================================================================
 * Test runner
 * ============================================================================ */

typedef int (*test_fn)(void);
typedef struct { const char* name; test_fn fn; } test_entry_t;

static const test_entry_t TESTS[] = {
    { "sport_basic_current",            test_sport_basic_current },
    { "sport_basic_voltage",            test_sport_basic_voltage },
    { "sport_bad_crc_rejected",         test_sport_bad_crc_rejected },
    { "sport_echo_then_response",       test_sport_echo_then_response },
    { "sport_resync_after_garbage",     test_sport_resync_after_garbage },
    { "sport_byte_stuffing_payload",    test_sport_byte_stuffing_payload },
    { "sport_unknown_data_id",          test_sport_unknown_data_id },
    { "sport_two_responses_back_to_back", test_sport_two_responses_back_to_back },
    { "sport_leaked_addr_byte_rejected", test_sport_leaked_addr_byte_rejected },
    { "sport_mid_frame_sync",           test_sport_mid_frame_sync },
    { "fbus_basic_response",            test_fbus_basic_response },
    { "fbus_parser_alignment",          test_fbus_parser_alignment },
    { "fbus_bad_prefix_dropped",        test_fbus_bad_prefix_dropped },
    { "fbus_restart_on_double_len",     test_fbus_restart_on_double_len },
    { "crc_known_vectors",              test_crc_known_vectors },
    { "is_connected_false_before_first_frame",   test_is_connected_false_before_first_frame },
    { "is_connected_after_first_frame",          test_is_connected_after_first_frame },
    { "is_connected_sticky_after_response_cleared", test_is_connected_sticky_after_response_cleared },
    { "last_byte_ms_independent_of_feed",        test_last_byte_ms_independent_of_feed },
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
