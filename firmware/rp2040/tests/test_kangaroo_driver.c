/**
 * Host-runnable tests for the Dimension Engineering Kangaroo X2 driver.
 *
 * Build/run: `./run_tests.sh` (in this directory). Same shape as
 * test_tic_driver.c — stub the platform/log/uart headers, then
 * `#include "../../shared/src/kangaroo_driver.c"` so static state and
 * helpers are reachable.
 *
 * Unlike the Tic test, the transport stub is READ-CAPABLE: a write
 * stages a canned reply into the RX buffer (simulating the Kangaroo
 * answering a Get request), so the packet/ASCII reply parsers can be
 * exercised end to end via query().
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <assert.h>

/* NOTE: SIMULATION is intentionally NOT defined — we want the wire
 * paths (query / read_packet_reply / read_simple_reply) compiled in so
 * the stub transport can drive them. The stub's write() is a no-op
 * other than staging canned replies, so nothing touches real hardware. */

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

/* ── Read-capable transport stub ───────────────────────────────── */
#define SAINT_KANGAROO_TRANSPORT_H
typedef struct kangaroo_transport_ops {
    const char* name;
    bool (*open)(uint8_t tx_pin, uint8_t rx_pin, uint32_t baud);
    bool (*is_open)(void);
    bool (*write)(const uint8_t* data, size_t len);
    size_t (*read)(uint8_t* data, size_t max_len);
    uint8_t (*resolved_instance)(void);
} kangaroo_transport_ops_t;

static bool    stub_open_flag = true;
static uint8_t stub_rx[64];
static size_t  stub_rx_n = 0, stub_rx_i = 0;
static uint8_t stub_canned[64];      /* reply staged by the next write() */
static size_t  stub_canned_n = 0;
static uint8_t stub_last_tx[64];     /* bytes the driver last wrote      */
static size_t  stub_last_tx_n = 0;

static bool stub_open(uint8_t tx, uint8_t rx, uint32_t baud)
{ (void)tx; (void)rx; (void)baud; return stub_open_flag; }
static bool stub_is_open(void) { return stub_open_flag; }
static bool stub_write(const uint8_t* d, size_t n)
{
    if (n <= sizeof(stub_last_tx)) { memcpy(stub_last_tx, d, n); stub_last_tx_n = n; }
    /* Device "responds": stage the canned reply for subsequent reads. */
    if (stub_canned_n) {
        memcpy(stub_rx, stub_canned, stub_canned_n);
        stub_rx_n = stub_canned_n;
        stub_rx_i = 0;
    }
    return true;
}
static size_t stub_read(uint8_t* d, size_t max)
{
    size_t k = 0;
    while (k < max && stub_rx_i < stub_rx_n) d[k++] = stub_rx[stub_rx_i++];
    /* The test clock is frozen, so the driver's read-timeout busy-wait
     * (while !has_byte: if MILLIS()-start > limit ...) would spin
     * forever on an empty wire. Advance the clock on an empty read so
     * those timeouts fire just like real elapsed time would make them. */
    if (k == 0) test_now_ms += 5;
    return k;
}
static uint8_t stub_inst(void) { return 0; }

static const kangaroo_transport_ops_t stub_ops = {
    "stub", stub_open, stub_is_open, stub_write, stub_read, stub_inst,
};
const kangaroo_transport_ops_t* kangaroo_get_transport(void) { return &stub_ops; }

/* ── Pull in the shared driver ─────────────────────────────────── */
#include "../../shared/src/kangaroo_driver.c"

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

static void clear_wire(void)
{
    stub_rx_n = stub_rx_i = 0;
    stub_canned_n = 0;
    stub_last_tx_n = 0;
    rx_look_n = rx_look_i = 0;
}

static void set_canned(const uint8_t* b, size_t n)
{
    memcpy(stub_canned, b, n);
    stub_canned_n = n;
}

static void reset_state(void)
{
    memset(units, 0, sizeof(units));
    unit_count = 0;
    port_initialized = false;
    configured_baud = KANGAROO_DEFAULT_BAUD;
    configured_serial_port = KANGAROO_DEFAULT_SERIAL_PORT;
    kangaroo_tx_pin = KANGAROO_DEFAULT_TX_PIN;
    kangaroo_rx_pin = KANGAROO_DEFAULT_RX_PIN;
    active_tx_pin = 0xFF;
    active_rx_pin = 0xFF;
    active_uart   = 0xFF;
    poll_unit = 0;
    poll_param = 0;
    log_count = 0;
    test_now_ms = 1000;
    test_pair_reject_tx = 0xFF;
    test_pair_reject_rx = 0xFF;
    test_pair_json_payload = NULL;
    stub_open_flag = true;
    clear_wire();
}

/* ── CRC-14 (verbatim DE) known-answer ─────────────────────────── */

static int test_crc14_known_answer(void)
{
    /* Start packet body for channel '1': addr 128, cmd 32, len 2,
     * data [0x31, 0x00]. Reference value computed from DE's algorithm. */
    uint8_t body[5] = { 128, 32, 2, 0x31, 0x00 };
    uint16_t crc = kangaroo_crc14(body, sizeof(body));
    CHECK_EQ(crc, 0x2222);
    CHECK_EQ(crc & 0x7f, 0x22);
    CHECK_EQ((crc >> 7) & 0x7f, 0x44);
    /* Determinism + sensitivity. */
    CHECK_EQ(kangaroo_crc14(body, sizeof(body)), 0x2222);
    body[3] ^= 0x01;
    CHECK(kangaroo_crc14(body, sizeof(body)) != 0x2222);
    return 1;
}

/* ── bit-packed numbers ────────────────────────────────────────── */

static int test_bitpack_known_vectors(void)
{
    uint8_t b[5];
    CHECK_EQ(kangaroo_bitpack(b, 0), 1);   CHECK_EQ(b[0], 0x00);
    CHECK_EQ(kangaroo_bitpack(b, 1), 1);   CHECK_EQ(b[0], 0x02);
    CHECK_EQ(kangaroo_bitpack(b, -1), 1);  CHECK_EQ(b[0], 0x03);
    CHECK_EQ(kangaroo_bitpack(b, 31), 1);  CHECK_EQ(b[0], 0x3e);   /* 31*2=62 < 64 -> 1 byte */
    CHECK_EQ(kangaroo_bitpack(b, 63), 2);  CHECK_EQ(b[0], 0x7e); CHECK_EQ(b[1], 0x01); /* 63*2=126 -> continuation */
    CHECK_EQ(kangaroo_bitpack(b, 64), 2);  CHECK_EQ(b[0], 0x40); CHECK_EQ(b[1], 0x02);
    CHECK_EQ(kangaroo_bitpack(b, -64), 2); CHECK_EQ(b[0], 0x41); CHECK_EQ(b[1], 0x02);
    CHECK_EQ(kangaroo_bitpack(b, 1000), 2);CHECK_EQ(b[0], 0x50); CHECK_EQ(b[1], 0x1f);
    CHECK_EQ(kangaroo_bitpack(b, KANGAROO_BITPACK_MAX), 5);
    return 1;
}

static int test_bitpack_roundtrip(void)
{
    int32_t cases[] = { 0, 1, -1, 63, 64, -64, 4095, 4096, -4096,
                        1000, -1000, 100000, -100000, KANGAROO_BITPACK_MAX,
                        -KANGAROO_BITPACK_MAX };
    for (size_t i = 0; i < sizeof(cases)/sizeof(cases[0]); i++) {
        uint8_t b[5];
        size_t n = kangaroo_bitpack(b, cases[i]);
        size_t idx = 0;
        int32_t out = kangaroo_bitunpack(b, n, &idx);
        CHECK_EQ(idx, n);
        CHECK_EQ(out, cases[i]);
    }
    return 1;
}

/* ── Packet framing ────────────────────────────────────────────── */

static int test_write_command_crc_placement(void)
{
    uint8_t data[2] = { 0x31, 0x00 };
    uint8_t buf[16];
    size_t n = kangaroo_write_command(128, KANGAROO_CMD_START, data, 2, buf);
    CHECK_EQ(n, 7);                       /* 5 + 2 */
    CHECK_EQ(buf[0], 128);                /* address (high bit set) */
    CHECK_EQ(buf[1], KANGAROO_CMD_START);
    CHECK_EQ(buf[2], 2);
    CHECK_EQ(buf[3], 0x31);
    CHECK_EQ(buf[4], 0x00);
    CHECK_EQ(buf[5], 0x22);               /* crc lo */
    CHECK_EQ(buf[6], 0x44);               /* crc hi */
    return 1;
}

static int test_build_move_position_with_limit(void)
{
    /* chan '1', pos 1000, speed limit 200 (reference computed from DE). */
    uint8_t buf[24];
    size_t n = kangaroo_build_move_position(128, '1', 1000, 200, buf);
    CHECK_EQ(buf[1], KANGAROO_CMD_MOVE);
    CHECK_EQ(buf[2], 8);                  /* data length */
    CHECK_EQ(buf[3], '1');
    CHECK_EQ(buf[4], 0x00);               /* move flags */
    CHECK_EQ(buf[5], KANGAROO_MOVE_POSITION);
    CHECK_EQ(buf[6], 0x50);               /* bitpack(1000)[0] */
    CHECK_EQ(buf[7], 0x1f);               /* bitpack(1000)[1] */
    CHECK_EQ(buf[8], KANGAROO_MOVE_SPEED);
    /* CRC matches an independent recompute over the first 3+len bytes. */
    uint16_t crc = kangaroo_crc14(buf, 3 + buf[2]);
    CHECK_EQ(buf[n - 2], crc & 0x7f);
    CHECK_EQ(buf[n - 1], (crc >> 7) & 0x7f);
    return 1;
}

static int test_build_move_position_no_limit_omits_speed(void)
{
    uint8_t buf[24];
    (void)kangaroo_build_move_position(128, '1', 1000, -1, buf);
    /* data = [chan, flags, type, bitpack(1000)] -> len 5, no speed param. */
    CHECK_EQ(buf[2], 5);
    return 1;
}

static int test_build_get(void)
{
    uint8_t buf[8];
    size_t n = kangaroo_build_get(128, '2', KANGAROO_GET_SPEED, buf);
    CHECK_EQ(n, 8);                       /* 5 + 3 data */
    CHECK_EQ(buf[1], KANGAROO_CMD_STATUS);
    CHECK_EQ(buf[2], 3);
    CHECK_EQ(buf[3], '2');
    CHECK_EQ(buf[4], 0x00);
    CHECK_EQ(buf[5], KANGAROO_GET_SPEED);
    return 1;
}

/* ── Packet reply parsing (via query) ──────────────────────────── */

static int test_query_packet_position_reply(void)
{
    reset_state();
    units[0].address = 128;
    units[0].channel_name = '1';
    units[0].protocol = KANGAROO_PROTO_PACKET;

    /* Reference reply: chan '1', flags 0, param 1, value 4321. */
    uint8_t reply[] = { 0x80, 0x43, 0x06, 0x31, 0x00, 0x01,
                        0x42, 0x47, 0x02, 0x09, 0x52 };
    set_canned(reply, sizeof(reply));

    kangaroo_reply_t r = {0};
    CHECK(query(&units[0], KANGAROO_GET_POSITION, &r));
    CHECK_EQ(r.flags, 0);
    CHECK_EQ(r.param, 1);
    CHECK_EQ(r.value, 4321);
    return 1;
}

static int test_query_packet_error_reply(void)
{
    reset_state();
    units[0].address = 128;
    units[0].channel_name = '1';
    units[0].protocol = KANGAROO_PROTO_PACKET;

    /* Reference reply: flags ERROR, value 1 (not started). */
    uint8_t reply[] = { 0x80, 0x43, 0x04, 0x31, 0x01, 0x01,
                        0x02, 0x45, 0x6a };
    set_canned(reply, sizeof(reply));

    kangaroo_reply_t r = {0};
    CHECK(query(&units[0], KANGAROO_GET_POSITION, &r));
    CHECK(r.flags & KANGAROO_STATUS_ERROR);
    CHECK_EQ(r.value, 1);
    return 1;
}

static int test_query_packet_bad_crc_rejected(void)
{
    reset_state();
    units[0].address = 128;
    units[0].channel_name = '1';
    units[0].protocol = KANGAROO_PROTO_PACKET;

    uint8_t reply[] = { 0x80, 0x43, 0x06, 0x31, 0x00, 0x01,
                        0x42, 0x47, 0x02, 0x09, 0x53 /* corrupted crc */ };
    set_canned(reply, sizeof(reply));

    kangaroo_reply_t r = {0};
    CHECK(!query(&units[0], KANGAROO_GET_POSITION, &r));
    return 1;
}

/* ── Simplified-serial reply parsing (via query) ───────────────── */

static int test_query_simple_position_done(void)
{
    reset_state();
    units[0].channel_name = '1';
    units[0].protocol = KANGAROO_PROTO_SIMPLE;

    const char* line = "1,P4321\r\n";
    set_canned((const uint8_t*)line, strlen(line));

    kangaroo_reply_t r = {0};
    CHECK(query(&units[0], KANGAROO_GET_POSITION, &r));
    CHECK_EQ(r.flags & KANGAROO_STATUS_BUSY, 0);  /* uppercase P = done */
    CHECK_EQ(r.value, 4321);
    return 1;
}

static int test_query_simple_position_moving(void)
{
    reset_state();
    units[0].channel_name = '1';
    units[0].protocol = KANGAROO_PROTO_SIMPLE;

    const char* line = "1,p1234\r\n";
    set_canned((const uint8_t*)line, strlen(line));

    kangaroo_reply_t r = {0};
    CHECK(query(&units[0], KANGAROO_GET_POSITION, &r));
    CHECK(r.flags & KANGAROO_STATUS_BUSY);        /* lowercase p = moving */
    CHECK_EQ(r.value, 1234);
    return 1;
}

static int test_query_simple_error(void)
{
    reset_state();
    units[0].channel_name = '2';
    units[0].protocol = KANGAROO_PROTO_SIMPLE;

    const char* line = "2,E1\r\n";
    set_canned((const uint8_t*)line, strlen(line));

    kangaroo_reply_t r = {0};
    CHECK(query(&units[0], KANGAROO_GET_POSITION, &r));
    CHECK(r.flags & KANGAROO_STATUS_ERROR);
    CHECK_EQ(r.value, 1);
    return 1;
}

/* ── set / get value ───────────────────────────────────────────── */

static int test_set_value_position_scales(void)
{
    reset_state();
    port_initialized = true;
    units[0].max_position = 10000;
    units[0].channel_name = '1';
    units[0].protocol = KANGAROO_PROTO_PACKET;

    CHECK(drv_set_value(KANGAROO_SUB_TARGET_POSITION, 0.5f));
    CHECK_EQ(units[0].target_position, 5000);
    CHECK(drv_set_value(KANGAROO_SUB_TARGET_POSITION, -1.0f));
    CHECK_EQ(units[0].target_position, -10000);
    CHECK(drv_set_value(KANGAROO_SUB_TARGET_POSITION, 5.0f));   /* clamp */
    CHECK_EQ(units[0].target_position, 10000);
    return 1;
}

static int test_set_value_speed_scales(void)
{
    reset_state();
    port_initialized = true;
    units[0].max_speed = 1000;
    units[0].channel_name = '1';
    units[0].protocol = KANGAROO_PROTO_PACKET;

    CHECK(drv_set_value(KANGAROO_SUB_TARGET_SPEED, 0.25f));
    CHECK_EQ(units[0].target_speed, 250);
    CHECK(drv_set_value(KANGAROO_SUB_TARGET_SPEED, -1.0f));
    CHECK_EQ(units[0].target_speed, -1000);
    return 1;
}

static int test_set_value_readonly_subchans_reject(void)
{
    reset_state();
    port_initialized = true;
    CHECK(!drv_set_value(KANGAROO_SUB_CURRENT_POSITION, 0.5f));
    CHECK(!drv_set_value(KANGAROO_SUB_CURRENT_SPEED,    0.5f));
    CHECK(!drv_set_value(KANGAROO_SUB_MOVING,           0.5f));
    CHECK(!drv_set_value(KANGAROO_SUB_ERROR_STATUS,     0.5f));
    return 1;
}

static int test_get_value_returns_correct_field(void)
{
    reset_state();
    units[3].target_position  = 5000;
    units[3].target_speed     = 250;
    units[3].current_position = 4321;
    units[3].current_speed    = 987;
    units[3].moving           = 1;
    units[3].error_status     = 6;
    units[3].max_position     = 10000;
    units[3].max_speed        = 1000;

    uint8_t base = 3 * KANGAROO_CHANNELS_PER_UNIT;
    float v;
    CHECK(drv_get_value(base + KANGAROO_SUB_TARGET_POSITION, &v));  CHECK_FLOAT(v, 0.5f);
    CHECK(drv_get_value(base + KANGAROO_SUB_TARGET_SPEED, &v));     CHECK_FLOAT(v, 0.25f);
    CHECK(drv_get_value(base + KANGAROO_SUB_CURRENT_POSITION, &v)); CHECK_FLOAT(v, 4321.0f);
    CHECK(drv_get_value(base + KANGAROO_SUB_CURRENT_SPEED, &v));    CHECK_FLOAT(v, 987.0f);
    CHECK(drv_get_value(base + KANGAROO_SUB_MOVING, &v));           CHECK_FLOAT(v, 1.0f);
    CHECK(drv_get_value(base + KANGAROO_SUB_ERROR_STATUS, &v));     CHECK_FLOAT(v, 6.0f);
    return 1;
}

/* ── JSON parsing ──────────────────────────────────────────────── */

static int test_parse_json_extracts_fields(void)
{
    reset_state();
    const char* json = "{ \"address\": 130, \"channel\": \"T\", "
                       "\"protocol\": \"simple\", \"home_on_start\": true, "
                       "\"max_position\": 50000, \"max_speed\": 2500 }";
    pin_config_t cfg = {0};
    drv_parse_json(json, json + strlen(json), &cfg);

    CHECK_EQ(cfg.params.kangaroo.address, 130);
    CHECK_EQ(cfg.params.kangaroo.channel_name, 'T');
    CHECK_EQ(cfg.params.kangaroo.protocol, KANGAROO_PROTO_SIMPLE);
    CHECK_EQ(cfg.params.kangaroo.home_on_start, 1);
    CHECK_EQ(cfg.params.kangaroo.max_position, 50000);
    CHECK_EQ(cfg.params.kangaroo.max_speed, 2500);
    return 1;
}

static int test_parse_json_protocol_defaults_packet(void)
{
    reset_state();
    const char* json = "{ \"address\": 128, \"protocol\": \"packet\" }";
    pin_config_t cfg = {0};
    drv_parse_json(json, json + strlen(json), &cfg);
    CHECK_EQ(cfg.params.kangaroo.protocol, KANGAROO_PROTO_PACKET);
    return 1;
}

/* ── apply_config + boot-reload guard ──────────────────────────── */

static int test_apply_config_updates_unit_state(void)
{
    reset_state();
    pin_config_t cfg = {0};
    cfg.params.kangaroo.address = 129;
    cfg.params.kangaroo.channel_name = '2';
    cfg.params.kangaroo.protocol = KANGAROO_PROTO_SIMPLE;
    cfg.params.kangaroo.max_position = 50000;
    cfg.params.kangaroo.max_speed = 2500;

    uint8_t ch = 2 * KANGAROO_CHANNELS_PER_UNIT;
    CHECK(drv_apply_config(ch, &cfg));
    CHECK_EQ(units[2].address, 129);
    CHECK_EQ(units[2].channel_name, '2');
    CHECK_EQ(units[2].protocol, KANGAROO_PROTO_SIMPLE);
    CHECK_EQ(units[2].max_position, 50000);
    CHECK_EQ(units[2].max_speed, 2500);
    CHECK_EQ(unit_count, 3);
    return 1;
}

static int test_apply_config_boot_reload_preserves_state(void)
{
    reset_state();
    units[0].address = 131;
    units[0].channel_name = 'D';
    units[0].max_position = 50000;
    unit_count = 1;

    pin_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode = PIN_MODE_KANGAROO;   /* zero params = boot-reload sweep */

    CHECK(drv_apply_config(0, &cfg));
    CHECK_EQ(units[0].address, 131);          /* not clobbered */
    CHECK_EQ(units[0].channel_name, 'D');
    CHECK_EQ(units[0].max_position, 50000);
    return 1;
}

/* ── save / load ───────────────────────────────────────────────── */

static int test_save_load_roundtrip(void)
{
    reset_state();
    kangaroo_tx_pin = 8;
    kangaroo_rx_pin = 9;
    configured_serial_port = 1;
    configured_baud = 38400;
    units[0].address = 128; units[0].channel_name = '1';
    units[0].protocol = KANGAROO_PROTO_PACKET;
    units[0].max_position = 25000; units[0].max_speed = 1500;
    units[1].address = 128; units[1].channel_name = '2';
    units[1].protocol = KANGAROO_PROTO_SIMPLE;
    units[1].max_position = 30000; units[1].max_speed = 2000;
    unit_count = 2;

    flash_storage_data_t flash;
    memset(&flash, 0, sizeof(flash));

    CHECK(drv_save(&flash));
    CHECK_EQ(flash.kangaroo_config.unit_count, 2);
    CHECK_EQ(flash.kangaroo_config.baud_rate, 38400);
    CHECK_EQ(flash.uart_pins.kangaroo_tx_pin, 8);
    CHECK_EQ(flash.uart_pins.kangaroo_rx_pin, 9);
    CHECK_EQ(flash.kangaroo_config.units[1].channel_name, '2');
    CHECK_EQ(flash.kangaroo_config.units[1].protocol, KANGAROO_PROTO_SIMPLE);

    reset_state();
    CHECK(drv_load(&flash));
    CHECK_EQ(unit_count, 2);
    CHECK_EQ(configured_baud, 38400);
    CHECK_EQ(kangaroo_tx_pin, 8);
    CHECK_EQ(units[0].channel_name, '1');
    CHECK_EQ(units[1].channel_name, '2');
    CHECK_EQ(units[1].max_position, 30000);
    CHECK_LOG("restored");
    return 1;
}

/* ── invalid pin pair fallback ─────────────────────────────────── */

static int test_invalid_pin_pair_warns(void)
{
    reset_state();
    test_pair_reject_tx = 5;
    test_pair_reject_rx = 6;
    kangaroo_tx_pin = 5;
    kangaroo_rx_pin = 6;

    kangaroo_init();

    CHECK_LOG("[warn]");
    CHECK_LOG("isn't a valid UART pair");
    CHECK_EQ(kangaroo_tx_pin, KANGAROO_DEFAULT_TX_PIN);
    CHECK_EQ(kangaroo_rx_pin, KANGAROO_DEFAULT_RX_PIN);
    return 1;
}

/* ── estop ─────────────────────────────────────────────────────── */

static int test_estop_clears_started(void)
{
    reset_state();
    port_initialized = true;
    units[0].channel_name = '1'; units[0].started = true; units[0].target_speed = 500;
    unit_count = 1;

    drv_estop();
    CHECK(!units[0].started);
    CHECK_EQ(units[0].target_speed, 0);
    CHECK_LOG("ESTOP");
    return 1;
}

/* ── Teach tune ────────────────────────────────────────────────── */
/*
 * Byte layouts below are checked against the Kangaroo Packet Serial
 * Reference Manual pp.13-15 (System data = channel, flags, sub-command,
 * then bit-packed parameters). Flags must stay 0: the manual warns that
 * tuning commands have unpredictable effects on sequence codes.
 */

/* Stage a unit that is live enough to accept tune commands. */
static void tune_ready_unit(void)
{
    reset_state();
    port_initialized = true;
    unit_count = 1;
    units[0].address = 128;
    units[0].channel_name = '1';
    units[0].protocol = KANGAROO_PROTO_PACKET;
    units[0].connected = true;
}

/* flags 0, param 1 (position), value 4321, BUSY clear. */
static const uint8_t TUNE_REPLY_IDLE[] = {
    0x80, 0x43, 0x06, 0x31, 0x00, 0x01, 0x42, 0x47, 0x02, 0x09, 0x52
};
/* flags ERROR, value 1. */
static const uint8_t TUNE_REPLY_ERROR[] = {
    0x80, 0x43, 0x04, 0x31, 0x01, 0x01, 0x02, 0x45, 0x6a
};

static int test_tune_builders_match_reference(void)
{
    uint8_t b[24];

    (void)kangaroo_build_tune_enter_mode(128, '1', KANGAROO_TUNE_MODE_TEACH, b);
    CHECK_EQ(b[1], KANGAROO_CMD_SYSTEM);
    CHECK_EQ(b[2], 4);                    /* chan, flags, sub, bitpack(1) */
    CHECK_EQ(b[3], '1');
    CHECK_EQ(b[4], 0x00);                 /* flags — never a sequence code */
    CHECK_EQ(b[5], KANGAROO_SYS_TUNE_ENTER_MODE);
    CHECK_EQ(b[6], 0x02);                 /* bitpack(1) = 1 << 1 */

    (void)kangaroo_build_tune_set_disabled_channels(128, '1', 0, b);
    CHECK_EQ(b[2], 4);
    CHECK_EQ(b[5], KANGAROO_SYS_TUNE_SET_DISABLED_CH);
    CHECK_EQ(b[6], 0x00);                 /* mask 0 = enable all */

    (void)kangaroo_build_tune_open_loop(128, '1', 1000, b);
    CHECK_EQ(b[2], 5);                    /* 1000 bit-packs to 2 bytes */
    CHECK_EQ(b[5], KANGAROO_SYS_TUNE_CONTROL_OPEN_LOOP);
    CHECK_EQ(b[6], 0x50);                 /* (2000 & 0x3f) | 0x40 */
    CHECK_EQ(b[7], 0x1F);                 /* 2000 >> 6 */

    (void)kangaroo_build_tune_go(128, '1', b);
    CHECK_EQ(b[2], 3);                    /* no parameters */
    CHECK_EQ(b[5], KANGAROO_SYS_TUNE_GO);

    (void)kangaroo_build_tune_abort(128, '1', b);
    CHECK_EQ(b[2], 3);
    CHECK_EQ(b[5], KANGAROO_SYS_TUNE_ABORT);
    return 1;
}

/* Control Open Loop's range is 2^28-1, NARROWER than the bit-packer's
 * 2^29-1. Clamping with the wrong constant would let a caller command
 * double the intended power on the one command with no feedback and no
 * travel limits. */
static int test_tune_open_loop_clamps_below_bitpack_max(void)
{
    uint8_t at_max[24], over[24], bitpack_max[24];
    size_t n_at  = kangaroo_build_tune_open_loop(128, '1', KANGAROO_OPEN_LOOP_MAX, at_max);
    size_t n_ov  = kangaroo_build_tune_open_loop(128, '1', 2147483647L, over);
    size_t n_bp  = kangaroo_build_tune_open_loop(128, '1', KANGAROO_BITPACK_MAX, bitpack_max);

    CHECK(KANGAROO_OPEN_LOOP_MAX < KANGAROO_BITPACK_MAX);
    CHECK_EQ(n_ov, n_at);
    CHECK_EQ(memcmp(over, at_max, n_at), 0);
    CHECK_EQ(n_bp, n_at);
    CHECK_EQ(memcmp(bitpack_max, at_max, n_at), 0);

    /* And symmetrically on the negative side. */
    uint8_t neg_at[24], neg_over[24];
    size_t n_nat = kangaroo_build_tune_open_loop(128, '1', -KANGAROO_OPEN_LOOP_MAX, neg_at);
    size_t n_nov = kangaroo_build_tune_open_loop(128, '1', -2147483647L, neg_over);
    CHECK_EQ(n_nov, n_nat);
    CHECK_EQ(memcmp(neg_over, neg_at, n_nat), 0);
    return 1;
}

static int test_tune_enter_moves_to_jog(void)
{
    tune_ready_unit();
    CHECK(kangaroo_tune_enter(0));
    CHECK_EQ(kangaroo_tune_get_state(0), KANGAROO_TUNE_JOG);
    CHECK_EQ(units[0].jog_power, 0);
    return 1;
}

static int test_tune_enter_rejects_simple_protocol(void)
{
    tune_ready_unit();
    units[0].protocol = KANGAROO_PROTO_SIMPLE;
    CHECK(!kangaroo_tune_enter(0));
    CHECK_EQ(kangaroo_tune_get_state(0), KANGAROO_TUNE_IDLE);
    CHECK_LOG("packet serial");
    return 1;
}

static int test_tune_enter_rejects_disconnected(void)
{
    tune_ready_unit();
    units[0].connected = false;
    CHECK(!kangaroo_tune_enter(0));
    CHECK_EQ(kangaroo_tune_get_state(0), KANGAROO_TUNE_IDLE);
    CHECK_LOG("not responding");
    return 1;
}

static int test_tune_jog_scales_by_power_cap(void)
{
    tune_ready_unit();
    CHECK(kangaroo_tune_enter(0));

    units[0].jog_pct = 50;
    CHECK(kangaroo_tune_jog(0, 1.0f));
    /* ~half of full scale; float rounding on a 28-bit value means this
     * can't be an exact compare. */
    CHECK(units[0].jog_power > 134000000L);
    CHECK(units[0].jog_power < 134500000L);

    CHECK(kangaroo_tune_jog(0, -1.0f));
    CHECK(units[0].jog_power < -134000000L);

    CHECK(kangaroo_tune_jog(0, 0.0f));
    CHECK_EQ(units[0].jog_power, 0);
    return 1;
}

/* An unset cap must not mean "unlimited" — it falls back to the low
 * default, because the first jog on an untuned axis is the most
 * dangerous moment in the procedure. */
static int test_tune_jog_unset_cap_uses_safe_default(void)
{
    tune_ready_unit();
    CHECK(kangaroo_tune_enter(0));
    units[0].jog_pct = 0;

    CHECK(kangaroo_tune_jog(0, 1.0f));
    long expect = (long)(KANGAROO_OPEN_LOOP_MAX / 10);   /* 10% default */
    CHECK(units[0].jog_power < expect + (expect / 100));
    CHECK(units[0].jog_power > expect - (expect / 100));
    return 1;
}

static int test_tune_jog_rejected_outside_jog_stage(void)
{
    tune_ready_unit();
    CHECK(!kangaroo_tune_jog(0, 1.0f));      /* still IDLE */
    CHECK_EQ(units[0].jog_power, 0);
    CHECK_LOG("not in the jog stage");
    return 1;
}

static int test_tune_jog_deadman_zeroes_power(void)
{
    tune_ready_unit();
    CHECK(kangaroo_tune_enter(0));
    CHECK(kangaroo_tune_jog(0, 1.0f));
    CHECK(units[0].jog_power != 0);

    /* Hold off the keep-alive so only the dead-man can act. */
    test_now_ms += KANGAROO_JOG_DEADMAN_MS + 10;
    units[0].tune_keepalive_ms = test_now_ms;
    tune_tick(0);

    CHECK_EQ(units[0].jog_power, 0);
    CHECK_LOG("dead-man");
    return 1;
}

static int test_tune_jog_refresh_holds_off_deadman(void)
{
    tune_ready_unit();
    CHECK(kangaroo_tune_enter(0));
    CHECK(kangaroo_tune_jog(0, 1.0f));
    long armed = units[0].jog_power;

    /* Refresh just inside the window, twice — power must survive. */
    for (int i = 0; i < 2; i++) {
        test_now_ms += KANGAROO_JOG_DEADMAN_MS - 50;
        CHECK(kangaroo_tune_jog(0, 1.0f));
        units[0].tune_keepalive_ms = test_now_ms;
        tune_tick(0);
    }
    CHECK_EQ(units[0].jog_power, armed);
    return 1;
}

static int test_tune_go_requires_jog_stage(void)
{
    tune_ready_unit();
    CHECK(!kangaroo_tune_go(0));
    CHECK_LOG("must be in the jog stage");

    CHECK(kangaroo_tune_enter(0));
    CHECK(kangaroo_tune_jog(0, 1.0f));
    CHECK(kangaroo_tune_go(0));
    CHECK_EQ(kangaroo_tune_get_state(0), KANGAROO_TUNE_GOING);
    /* Go must never hand over with jog power still applied. */
    CHECK_EQ(units[0].jog_power, 0);
    return 1;
}

static int test_tune_abort_fails_the_tune(void)
{
    tune_ready_unit();
    CHECK(kangaroo_tune_enter(0));
    units[0].started = true;

    CHECK(kangaroo_tune_abort(0));
    CHECK_EQ(kangaroo_tune_get_state(0), KANGAROO_TUNE_FAILED);
    CHECK_EQ(units[0].jog_power, 0);
    /* Error 6 clears with a Start, so the channel must re-Start before
     * normal motion is accepted again. */
    CHECK(!units[0].started);
    CHECK_LOG("aborted by operator");
    return 1;
}

static int test_tune_blocks_routed_motion(void)
{
    tune_ready_unit();
    CHECK(kangaroo_tune_enter(0));
    units[0].max_position = 10000;

    CHECK(!kangaroo_set_position(0, 500));
    CHECK(!kangaroo_set_speed(0, 500));
    CHECK_LOG("tune in progress");
    /* The refused setpoint must not be recorded either. */
    CHECK_EQ(units[0].target_position, 0);
    return 1;
}

static int test_estop_aborts_tune(void)
{
    tune_ready_unit();
    CHECK(kangaroo_tune_enter(0));
    CHECK(kangaroo_tune_jog(0, 1.0f));

    drv_estop();
    CHECK_EQ(kangaroo_tune_get_state(0), KANGAROO_TUNE_FAILED);
    CHECK_EQ(units[0].jog_power, 0);
    CHECK_LOG("e-stop");
    return 1;
}

static int test_tune_error_reply_fails_tune(void)
{
    tune_ready_unit();
    CHECK(kangaroo_tune_enter(0));
    CHECK(kangaroo_tune_jog(0, 1.0f));
    CHECK(kangaroo_tune_go(0));

    set_canned(TUNE_REPLY_ERROR, sizeof(TUNE_REPLY_ERROR));
    test_now_ms += KANGAROO_TUNE_KEEPALIVE_MS + 10;
    tune_tick(0);

    CHECK_EQ(kangaroo_tune_get_state(0), KANGAROO_TUNE_FAILED);
    CHECK_LOG("reported an error");
    return 1;
}

static int test_tune_completes_after_quiet_period(void)
{
    tune_ready_unit();
    CHECK(kangaroo_tune_enter(0));
    CHECK(kangaroo_tune_go(0));
    set_canned(TUNE_REPLY_IDLE, sizeof(TUNE_REPLY_IDLE));

    /* Not-busy before the cycle has had time to start proves nothing —
     * the axis is idle at that point anyway. */
    test_now_ms += KANGAROO_TUNE_KEEPALIVE_MS + 10;
    tune_tick(0);
    CHECK_EQ(kangaroo_tune_get_state(0), KANGAROO_TUNE_GOING);

    /* Past the minimum, then quiet for long enough. Completion needs
     * KANGAROO_TUNE_QUIET_MS of continuous not-busy, which is many
     * keep-alive intervals — iterate well past that so the test doesn't
     * become brittle if either constant is retuned. */
    test_now_ms += KANGAROO_TUNE_MIN_MS;
    int ticks = (KANGAROO_TUNE_QUIET_MS / KANGAROO_TUNE_KEEPALIVE_MS) + 5;
    for (int i = 0; i < ticks; i++) {
        test_now_ms += KANGAROO_TUNE_KEEPALIVE_MS + 10;
        tune_tick(0);
    }
    CHECK_EQ(kangaroo_tune_get_state(0), KANGAROO_TUNE_DONE);
    CHECK_LOG("POWER CYCLE REQUIRED");
    return 1;
}

static int test_tune_times_out(void)
{
    tune_ready_unit();
    CHECK(kangaroo_tune_enter(0));
    CHECK(kangaroo_tune_go(0));

    test_now_ms += KANGAROO_TUNE_MAX_MS + 1;
    tune_tick(0);

    CHECK_EQ(kangaroo_tune_get_state(0), KANGAROO_TUNE_FAILED);
    CHECK_LOG("time limit");
    return 1;
}

/* ── Safety interlock (estop / clear_estop command verbs) ──────── */
/*
 * These come in through drv_command rather than the estop() vtable entry
 * because estop() is driver-WIDE: it stops every Kangaroo the driver
 * owns. A limit switch guarding one axis must stop only that axis, so
 * the switch_input driver routes by peripheral_id. See
 * docs/SENSOR_INPUTS.md.
 */

static void interlock_ready_unit(const char* id)
{
    tune_ready_unit();
    snprintf(units[0].peripheral_id, sizeof(units[0].peripheral_id), "%s", id);
    units[0].max_position = 10000;
    units[0].max_speed = 1000;
}

static int test_estop_command_latches_and_blocks_motion(void)
{
    interlock_ready_unit("kangaroo-1");

    CHECK(drv_command("kangaroo-1", "estop", NULL, NULL));
    CHECK(units[0].interlocked);

    /* A plain powerdown would be undone by the very next setpoint —
     * that's why the latch exists. */
    /* A bare estop with no direction argument blocks everything — the
     * conservative reading, and what an operator-driven stop should do. */
    CHECK_EQ(units[0].interlock_block, SWITCH_BLOCK_BOTH);
    CHECK(!kangaroo_set_position(0, 500));
    CHECK(!kangaroo_set_speed(0, 500));
    CHECK(!kangaroo_set_speed(0, -500));
    CHECK_LOG("interlock blocks");
    CHECK_EQ(units[0].target_position, 0);
    return 1;
}

/* ── Soft limits (usable travel inside the taught range) ───────── */

static int test_soft_limits_clamp_position(void)
{
    interlock_ready_unit("kangaroo-1");
    units[0].soft_valid = true;
    units[0].soft_min = 1000;
    units[0].soft_max = 9000;

    /* Clamped, not refused: a routed animation that overshoots slightly
     * should ride the limit rather than drop the move entirely. */
    CHECK(kangaroo_set_position(0, 12000));
    CHECK_EQ(units[0].target_position, 9000);
    CHECK(kangaroo_set_position(0, -500));
    CHECK_EQ(units[0].target_position, 1000);
    CHECK(kangaroo_set_position(0, 5000));
    CHECK_EQ(units[0].target_position, 5000);
    CHECK_LOG("clamped");
    return 1;
}

/* Without a config sync there are no soft limits, and the driver must
 * behave exactly as it did before rather than clamping to zero. */
static int test_soft_limits_inactive_until_synced(void)
{
    interlock_ready_unit("kangaroo-1");
    units[0].soft_valid = false;
    units[0].soft_min = 0;
    units[0].soft_max = 0;

    CHECK(kangaroo_set_position(0, 12000));
    CHECK_EQ(units[0].target_position, 12000);
    return 1;
}

/* All three values must arrive together, and min must be below max —
 * a half-configured window could park the axis somewhere the operator
 * never asked for. */
static int test_soft_limits_reject_partial_or_inverted(void)
{
    pin_config_t cfg;

    memset(&cfg, 0, sizeof(cfg));
    const char* partial = "{\"soft_min\":1000,\"soft_max\":9000}";
    CHECK(drv_parse_json(partial, partial + strlen(partial), &cfg));
    CHECK_EQ(cfg.params.kangaroo.soft_valid, 0);

    memset(&cfg, 0, sizeof(cfg));
    const char* inverted =
        "{\"soft_min\":9000,\"soft_max\":1000,\"soft_center\":5000}";
    CHECK(drv_parse_json(inverted, inverted + strlen(inverted), &cfg));
    CHECK_EQ(cfg.params.kangaroo.soft_valid, 0);

    memset(&cfg, 0, sizeof(cfg));
    const char* good =
        "{\"soft_min\":1000,\"soft_max\":9000,\"soft_center\":4200}";
    CHECK(drv_parse_json(good, good + strlen(good), &cfg));
    CHECK_EQ(cfg.params.kangaroo.soft_valid, 1);
    CHECK_EQ(cfg.params.kangaroo.soft_min, 1000);
    CHECK_EQ(cfg.params.kangaroo.soft_max, 9000);
    CHECK_EQ(cfg.params.kangaroo.soft_center, 4200);
    return 1;
}

/* The clamp must not let a move sneak past the interlock: clamping
 * happens first so the direction check sees the real commanded value. */
static int test_soft_clamp_runs_before_interlock_check(void)
{
    interlock_ready_unit("kangaroo-1");
    units[0].soft_valid = true;
    units[0].soft_min = 1000;
    units[0].soft_max = 9000;
    units[0].current_position = 5000;

    const char* args = "{\"block\":1}";      /* block extend */
    CHECK(drv_command("kangaroo-1", "estop", args, args + strlen(args)));

    /* Clamps to 9000, which is still an extend from 5000 → refused. */
    CHECK(!kangaroo_set_position(0, 99999));
    /* Clamps to 1000, a retreat → allowed. */
    CHECK(kangaroo_set_position(0, -99999));
    CHECK_EQ(units[0].target_position, 1000);
    return 1;
}

/* ── Directional interlock ─────────────────────────────────────── */
/*
 * An end-of-travel switch blocks motion INTO itself but must let the
 * axis retreat — otherwise tripping a limit strands the mechanism on the
 * switch with no way off but a manual clear.
 */

static int test_interlock_positive_allows_retreat(void)
{
    interlock_ready_unit("kangaroo-1");
    units[0].current_position = 5000;

    const char* args = "{\"block\":1}";      /* SWITCH_BLOCK_POSITIVE */
    CHECK(drv_command("kangaroo-1", "estop", args, args + strlen(args)));
    CHECK_EQ(units[0].interlock_block, SWITCH_BLOCK_POSITIVE);

    /* Blocked: extend. Allowed: retract. */
    CHECK(!kangaroo_set_speed(0, 400));
    CHECK(kangaroo_set_speed(0, -400));
    CHECK_LOG("extend/forward");
    return 1;
}

static int test_interlock_negative_allows_extend(void)
{
    interlock_ready_unit("kangaroo-1");
    units[0].current_position = 5000;

    const char* args = "{\"block\":2}";      /* SWITCH_BLOCK_NEGATIVE */
    CHECK(drv_command("kangaroo-1", "estop", args, args + strlen(args)));

    CHECK(!kangaroo_set_speed(0, -400));
    CHECK(kangaroo_set_speed(0, 400));
    return 1;
}

/* Position commands are judged against the CURRENT position, so a move
 * that retreats is allowed and one that drives further in is not. */
static int test_interlock_position_compares_against_current(void)
{
    interlock_ready_unit("kangaroo-1");
    units[0].current_position = 5000;

    const char* args = "{\"block\":1}";      /* block extend */
    CHECK(drv_command("kangaroo-1", "estop", args, args + strlen(args)));

    CHECK(!kangaroo_set_position(0, 6000));  /* further out — refused */
    CHECK(kangaroo_set_position(0, 4000));   /* back off  — allowed  */
    CHECK_EQ(units[0].target_position, 4000);
    return 1;
}

/* Zero must always get through regardless of direction, or the interlock
 * could not actually stop the axis. */
static int test_interlock_never_blocks_a_stop(void)
{
    interlock_ready_unit("kangaroo-1");
    CHECK(drv_command("kangaroo-1", "estop", NULL, NULL));   /* blocks both */
    CHECK(kangaroo_set_speed(0, 0));
    return 1;
}

/* A malformed or out-of-range direction must fall back to blocking
 * everything, never to blocking nothing. */
static int test_interlock_bad_direction_blocks_both(void)
{
    interlock_ready_unit("kangaroo-1");
    const char* args = "{\"block\":99}";
    CHECK(drv_command("kangaroo-1", "estop", args, args + strlen(args)));
    CHECK_EQ(units[0].interlock_block, SWITCH_BLOCK_BOTH);
    CHECK(!kangaroo_set_speed(0, 400));
    CHECK(!kangaroo_set_speed(0, -400));
    return 1;
}

static int test_estop_command_only_hits_named_instance(void)
{
    interlock_ready_unit("kangaroo-1");
    /* Second unit on the same driver — must be untouched. */
    units[1] = units[0];
    units[1].interlocked = false;
    snprintf(units[1].peripheral_id, sizeof(units[1].peripheral_id),
             "kangaroo-2");
    unit_count = 2;

    CHECK(drv_command("kangaroo-1", "estop", NULL, NULL));
    CHECK(units[0].interlocked);
    CHECK(!units[1].interlocked);
    return 1;
}

static int test_estop_command_unknown_id_not_claimed(void)
{
    interlock_ready_unit("kangaroo-1");
    /* Returning false is what lets the manager offer it to the next
     * driver instead of swallowing it. */
    CHECK(!drv_command("not-ours", "estop", NULL, NULL));
    CHECK(!units[0].interlocked);
    return 1;
}

static int test_estop_command_aborts_running_tune(void)
{
    interlock_ready_unit("kangaroo-1");
    CHECK(kangaroo_tune_enter(0));
    CHECK(kangaroo_tune_jog(0, 1.0f));
    CHECK(kangaroo_tune_go(0));

    CHECK(drv_command("kangaroo-1", "estop", NULL, NULL));
    CHECK_EQ(kangaroo_tune_get_state(0), KANGAROO_TUNE_FAILED);
    CHECK_EQ(units[0].jog_power, 0);
    CHECK(units[0].interlocked);
    return 1;
}

/* Starting a tune with a limit tripped is the worst case: jogging is
 * open loop, so it would drive with no feedback past a point something
 * already decided was out of bounds. */
static int test_interlock_blocks_starting_a_tune(void)
{
    interlock_ready_unit("kangaroo-1");
    CHECK(drv_command("kangaroo-1", "estop", NULL, NULL));

    CHECK(!kangaroo_tune_enter(0));
    CHECK_EQ(kangaroo_tune_get_state(0), KANGAROO_TUNE_IDLE);
    CHECK_LOG("interlock latched");
    return 1;
}

static int test_clear_estop_re_enables_motion(void)
{
    interlock_ready_unit("kangaroo-1");
    CHECK(drv_command("kangaroo-1", "estop", NULL, NULL));
    CHECK(!kangaroo_set_position(0, 500));

    CHECK(drv_command("kangaroo-1", "clear_estop", NULL, NULL));
    CHECK(!units[0].interlocked);
    /* Clearing must not itself command motion — the channel stays
     * un-started until a real setpoint arrives. */
    CHECK(!units[0].started);
    CHECK(kangaroo_set_position(0, 500));
    return 1;
}

static int test_clear_estop_when_not_latched_is_a_noop(void)
{
    interlock_ready_unit("kangaroo-1");
    CHECK(drv_command("kangaroo-1", "clear_estop", NULL, NULL));
    CHECK(!units[0].interlocked);
    return 1;
}

/* ── Test runner ───────────────────────────────────────────────── */

typedef int (*test_fn)(void);
typedef struct { const char* name; test_fn fn; } test_entry_t;

static const test_entry_t TESTS[] = {
    {"crc14_known_answer",                  test_crc14_known_answer},
    {"bitpack_known_vectors",               test_bitpack_known_vectors},
    {"bitpack_roundtrip",                   test_bitpack_roundtrip},
    {"write_command_crc_placement",         test_write_command_crc_placement},
    {"build_move_position_with_limit",      test_build_move_position_with_limit},
    {"build_move_position_no_limit",        test_build_move_position_no_limit_omits_speed},
    {"build_get",                           test_build_get},

    {"query_packet_position_reply",         test_query_packet_position_reply},
    {"query_packet_error_reply",            test_query_packet_error_reply},
    {"query_packet_bad_crc_rejected",       test_query_packet_bad_crc_rejected},
    {"query_simple_position_done",          test_query_simple_position_done},
    {"query_simple_position_moving",        test_query_simple_position_moving},
    {"query_simple_error",                  test_query_simple_error},

    {"set_value_position_scales",           test_set_value_position_scales},
    {"set_value_speed_scales",              test_set_value_speed_scales},
    {"set_value_readonly_subchans_reject",  test_set_value_readonly_subchans_reject},
    {"get_value_returns_correct_field",     test_get_value_returns_correct_field},

    {"parse_json_extracts_fields",          test_parse_json_extracts_fields},
    {"parse_json_protocol_defaults_packet", test_parse_json_protocol_defaults_packet},

    {"apply_config_updates_unit_state",     test_apply_config_updates_unit_state},
    {"apply_config_boot_reload_preserves",  test_apply_config_boot_reload_preserves_state},

    {"save_load_roundtrip",                 test_save_load_roundtrip},
    {"invalid_pin_pair_warns",              test_invalid_pin_pair_warns},
    {"estop_clears_started",                test_estop_clears_started},

    {"tune_builders_match_reference",       test_tune_builders_match_reference},
    {"tune_open_loop_clamps_below_bitpack", test_tune_open_loop_clamps_below_bitpack_max},
    {"tune_enter_moves_to_jog",             test_tune_enter_moves_to_jog},
    {"tune_enter_rejects_simple_protocol",  test_tune_enter_rejects_simple_protocol},
    {"tune_enter_rejects_disconnected",     test_tune_enter_rejects_disconnected},
    {"tune_jog_scales_by_power_cap",        test_tune_jog_scales_by_power_cap},
    {"tune_jog_unset_cap_uses_default",     test_tune_jog_unset_cap_uses_safe_default},
    {"tune_jog_rejected_outside_jog_stage", test_tune_jog_rejected_outside_jog_stage},
    {"tune_jog_deadman_zeroes_power",       test_tune_jog_deadman_zeroes_power},
    {"tune_jog_refresh_holds_off_deadman",  test_tune_jog_refresh_holds_off_deadman},
    {"tune_go_requires_jog_stage",          test_tune_go_requires_jog_stage},
    {"tune_abort_fails_the_tune",           test_tune_abort_fails_the_tune},
    {"tune_blocks_routed_motion",           test_tune_blocks_routed_motion},
    {"estop_aborts_tune",                   test_estop_aborts_tune},
    {"tune_error_reply_fails_tune",         test_tune_error_reply_fails_tune},
    {"tune_completes_after_quiet_period",   test_tune_completes_after_quiet_period},
    {"tune_times_out",                      test_tune_times_out},

    {"estop_cmd_latches_blocks_motion",     test_estop_command_latches_and_blocks_motion},
    {"estop_cmd_only_named_instance",       test_estop_command_only_hits_named_instance},
    {"estop_cmd_unknown_id_not_claimed",    test_estop_command_unknown_id_not_claimed},
    {"estop_cmd_aborts_running_tune",       test_estop_command_aborts_running_tune},
    {"interlock_blocks_starting_a_tune",    test_interlock_blocks_starting_a_tune},
    {"clear_estop_re_enables_motion",       test_clear_estop_re_enables_motion},
    {"clear_estop_not_latched_noop",        test_clear_estop_when_not_latched_is_a_noop},

    {"soft_limits_clamp_position",          test_soft_limits_clamp_position},
    {"soft_limits_inactive_until_synced",   test_soft_limits_inactive_until_synced},
    {"soft_limits_reject_partial",          test_soft_limits_reject_partial_or_inverted},
    {"soft_clamp_before_interlock_check",   test_soft_clamp_runs_before_interlock_check},

    {"interlock_positive_allows_retreat",   test_interlock_positive_allows_retreat},
    {"interlock_negative_allows_extend",    test_interlock_negative_allows_extend},
    {"interlock_position_vs_current",       test_interlock_position_compares_against_current},
    {"interlock_never_blocks_a_stop",       test_interlock_never_blocks_a_stop},
    {"interlock_bad_direction_blocks_both", test_interlock_bad_direction_blocks_both},
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
