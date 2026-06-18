/**
 * Host-runnable test for shared maestro_driver core.
 *
 * Strategy: provide fake maestro_transport_ops tables (one tagged
 * "usb_host", one tagged "uart") plus a small saint_log stub, then
 * include the unit under test so its file-scope statics and the
 * peripheral_driver_t glue land in this translation unit.
 *
 * Coverage:
 *   - compact-protocol byte assembly for set_target / set_speed /
 *     set_acceleration / get_position / get_errors / go_home matches
 *     the Pololu spec exactly
 *   - get_position assembles the 2-byte LE reply correctly
 *   - get_errors timeout path returns 0xFFFF (not 0)
 *   - channel-config round-trip through set/get
 *   - drv_save -> drv_load preserves channel values and transport_mode
 *   - ensure_state_init idempotency (load-before-init doesn't reset
 *     freshly loaded channel configs back to defaults — the bug I just
 *     ran into during the port)
 *   - parse_json picks "transport":"uart" and "transport":"usb_host"
 *   - default transport selection prefers USB host when both ops are
 *     available; falls back to UART when USB returns NULL
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

/* ── saint_log stub ───────────────────────────────────────────────── */

#define SAINT_LOG_H  /* short-circuit the real header */
static void saint_log_publish(const char* level, const char* fmt, ...)
{
    (void)level; (void)fmt;
}
static void saint_log_boot_queue(const char* level, const char* fmt, ...)
{ (void)level; (void)fmt; }
static void saint_log_set_ros_ready(bool ready) { (void)ready; }
static void saint_log_drain_boot_queue(void) {}
static void saint_log_emit_local(const char* level, const char* text)
{ (void)level; (void)text; }
static bool saint_log_emit_ros(const char* json, size_t len)
{ (void)json; (void)len; return true; }
static uint32_t saint_log_uptime_ms(void) { return 0; }

/* ── Transport observation state + fakes ─────────────────────────── */

#define BUS_CAP 512
static uint8_t  bus[BUS_CAP];
static size_t   bus_len = 0;
static uint8_t  reply[BUS_CAP];
static size_t   reply_len = 0;
static size_t   reply_pos = 0;
static bool     reply_should_timeout = false;
static int      update_calls = 0;
static int      open_calls   = 0;
static bool     fake_connected = true;
static const char* opened_name = NULL;

static void bus_reset(void)
{
    bus_len = 0;
    reply_len = 0;
    reply_pos = 0;
    reply_should_timeout = false;
    update_calls = 0;
    open_calls   = 0;
    fake_connected = true;
    opened_name = NULL;
}

static void prime_reply(const uint8_t* data, size_t len)
{
    if (len > sizeof(reply)) len = sizeof(reply);
    memcpy(reply, data, len);
    reply_len = len;
    reply_pos = 0;
}

/* Two transport tables — one tagged usb_host, one uart. Both share
 * the same buffers so tests can assert against `bus` regardless of
 * which one's active. The point of having two is to verify the
 * shared driver actually dispatches through the right table when
 * transport_mode changes. */

#include "flash_types.h"
#include "maestro_transport.h"

static bool fake_open(const flash_storage_data_t* s);
static void fake_update(void);
static bool fake_is_connected(void);
static bool fake_write(const uint8_t* data, size_t len);
static size_t fake_read(uint8_t* data, size_t len, uint32_t timeout_ms);

static const maestro_transport_ops_t usb_ops = {
    .name             = "usb_host",
    .open             = fake_open,
    .update           = fake_update,
    .is_connected     = fake_is_connected,
    .write            = fake_write,
    .read             = fake_read,
    .supports_hotplug = NULL,    /* tested with hotplug off — connect/disconnect
                                    log lines aren't part of this suite */
};
static const maestro_transport_ops_t uart_ops = {
    .name             = "uart",
    .open             = fake_open,
    .update           = fake_update,
    .is_connected     = fake_is_connected,
    .write            = fake_write,
    .read             = fake_read,
    .supports_hotplug = NULL,
};

/* Per-test toggles for what the lookups return. */
static bool usb_available  = true;
static bool uart_available = true;

const maestro_transport_ops_t* maestro_get_transport_usb_cdc(void)
{
    return usb_available ? &usb_ops : NULL;
}
const maestro_transport_ops_t* maestro_get_transport_usb_vendor(void)
{
    return NULL;  /* not exercised by shared tests */
}
const maestro_transport_ops_t* maestro_get_transport_usb_host(void)
{
    return maestro_get_transport_usb_cdc();  /* legacy alias */
}
const maestro_transport_ops_t* maestro_get_transport_uart(void)
{
    return uart_available ? &uart_ops : NULL;
}

static bool fake_open(const flash_storage_data_t* s)
{
    open_calls++;
    /* Stash which name was open()d to verify the right transport was picked. */
    opened_name = (s && s->maestro_config.transport_mode
                   == FLASH_MAESTRO_TRANSPORT_UART) ? "uart" : "usb_host";
    return true;
}
static void fake_update(void) { update_calls++; }
static bool fake_is_connected(void) { return fake_connected; }
static bool fake_write(const uint8_t* data, size_t len)
{
    if (bus_len + len > sizeof(bus)) return false;
    memcpy(bus + bus_len, data, len);
    bus_len += len;
    return true;
}
static size_t fake_read(uint8_t* data, size_t len, uint32_t timeout_ms)
{
    (void)timeout_ms;
    if (reply_should_timeout) return 0;
    size_t avail = reply_len - reply_pos;
    size_t take  = avail < len ? avail : len;
    memcpy(data, reply + reply_pos, take);
    reply_pos += take;
    return take;
}

/* Now pull in the unit under test. */
#include "../src/maestro_driver.c"

/* ── Test plumbing ────────────────────────────────────────────────── */

static int fail_count = 0;
#define EXPECT(cond, label) do {                                                \
    if (cond) printf("  ok   %s\n", label);                                     \
    else { printf("  FAIL %s\n", label); fail_count++; }                        \
} while (0)

/* Force the shared module + bus back to clean state between cases. */
static void reset_all(void)
{
    bus_reset();
    g_initialized   = false;
    g_was_connected = false;
    g_transport     = NULL;
    g_transport_mode = FLASH_MAESTRO_TRANSPORT_USB_HOST;
    usb_available  = true;
    uart_available = true;
}

/* Bind a transport explicitly without going through a full flash
 * struct (some tests want to drive write/read directly without
 * exercising drv_load's branching). */
static void force_bind_transport(const maestro_transport_ops_t* ops)
{
    g_transport = ops;
}

/* ── Cases ────────────────────────────────────────────────────────── */

static void case_compact_protocol_bytes(void)
{
    printf("case_compact_protocol_bytes\n");
    reset_all();
    maestro_init();
    force_bind_transport(&uart_ops);

    /* Set target — channel 3, 6000 quarter-µs (= 1500µs neutral). The
     * Pololu spec splits the 14-bit value into two 7-bit halves: low7
     * in byte[2], high7 in byte[3]. 6000 = 0x1770 → low7=0x70, high7=0x2E. */
    bus_reset();
    EXPECT(maestro_set_target(3, 6000), "set_target returned true");
    EXPECT(bus_len == 4,                "set_target wrote 4 bytes");
    EXPECT(bus[0] == 0x84,              "byte0 == 0x84 (Set Target)");
    EXPECT(bus[1] == 3,                 "byte1 == channel");
    EXPECT(bus[2] == 0x70,              "byte2 == low7 of 6000");
    EXPECT(bus[3] == 0x2E,              "byte3 == high7 of 6000");

    /* set_speed: command 0x87, same split as target. */
    bus_reset();
    EXPECT(maestro_set_speed(5, 0x3FFF), "set_speed returned true");
    EXPECT(bus[0] == 0x87 && bus[1] == 5,           "set_speed header");
    EXPECT(bus[2] == 0x7F && bus[3] == 0x7F,        "set_speed payload (max 14-bit)");

    /* set_acceleration: command 0x89. */
    bus_reset();
    EXPECT(maestro_set_acceleration(0, 1), "set_acceleration returned true");
    EXPECT(bus[0] == 0x89 && bus[1] == 0,           "set_accel header");
    EXPECT(bus[2] == 1    && bus[3] == 0,           "set_accel payload");

    /* go_home: single byte 0xA2 */
    bus_reset();
    maestro_go_home();
    EXPECT(bus_len == 1 && bus[0] == 0xA2, "go_home == 0xA2");

    /* get_position: 2-byte LE reply assembled correctly */
    bus_reset();
    uint8_t pos_reply[2] = { 0x70, 0x2E };  /* 0x2E70 = 11888 = 1500µs * 4 + 8888 nope; just an arbitrary value */
    prime_reply(pos_reply, 2);
    uint16_t pos = maestro_get_position(7);
    EXPECT(bus_len == 2,                  "get_position wrote 2-byte query");
    EXPECT(bus[0] == 0x90 && bus[1] == 7, "get_position header");
    EXPECT(pos == 0x2E70,                 "get_position assembled LE bytes");

    /* get_errors: timeout -> 0xFFFF (NOT 0 — 0 means "no errors" which
     * is misleading). */
    bus_reset();
    reply_should_timeout = true;
    EXPECT(maestro_get_errors() == 0xFFFF, "get_errors timeout -> 0xFFFF");
}

static void case_channel_config_roundtrip(void)
{
    printf("case_channel_config_roundtrip\n");
    reset_all();
    maestro_init();
    force_bind_transport(&uart_ops);

    maestro_channel_config_t cfg = {
        .min_pulse_us = 1000,
        .max_pulse_us = 2000,
        .neutral_us   = 1500,
        .speed        = 50,
        .acceleration = 10,
        .home_us      = 1500,
    };
    maestro_set_channel_config(2, &cfg);

    const maestro_channel_config_t* got = maestro_get_channel_config(2);
    EXPECT(got != NULL,                       "get returned non-null");
    EXPECT(got->min_pulse_us == 1000,         "min_pulse_us round-trip");
    EXPECT(got->max_pulse_us == 2000,         "max_pulse_us round-trip");
    EXPECT(got->neutral_us   == 1500,         "neutral_us round-trip");
    EXPECT(got->speed        == 50,           "speed round-trip");
    EXPECT(got->acceleration == 10,           "acceleration round-trip");
    EXPECT(got->home_us      == 1500,         "home_us round-trip");

    /* Speed + acceleration are stored as the channel's default (only
     * applied on Pose transitions) — they are NOT pushed to the
     * device on config save. Animation/control input snaps to target
     * at full speed; deliberate change in Phase 1 of the Maestro
     * per-channel-config work. See TODO comment in
     * firmware/shared/src/maestro_driver.c::maestro_set_channel_config. */
    EXPECT(bus_len == 0, "set_channel_config does NOT push speed/accel on config save");
}

static void case_save_load_roundtrip(void)
{
    printf("case_save_load_roundtrip\n");
    reset_all();
    maestro_init();
    force_bind_transport(&uart_ops);
    g_transport_mode = FLASH_MAESTRO_TRANSPORT_UART;

    maestro_channel_config_t cfg = {
        .min_pulse_us = 800,
        .max_pulse_us = 2200,
        .neutral_us   = 1500,
        .speed        = 100,
        .acceleration = 20,
        .home_us      = 1000,
    };
    maestro_set_channel_config(0, &cfg);
    maestro_set_channel_config(5, &cfg);

    flash_storage_data_t storage;
    memset(&storage, 0, sizeof(storage));

    EXPECT(maestro_peripheral.save_config(&storage),
           "save_config returned true");
    EXPECT(storage.maestro_config.channel_count == MAESTRO_MAX_CHANNELS,
           "channel_count saved");
    EXPECT(storage.maestro_config.transport_mode
            == FLASH_MAESTRO_TRANSPORT_UART,
           "transport_mode saved");
    EXPECT(storage.maestro_config.channels[5].min_pulse_us == 800,
           "channel 5 min_pulse saved");

    /* Now fresh state, load, verify it came back. */
    reset_all();
    force_bind_transport(NULL);
    /* Bypass init so load runs against an "uninitialized" module —
     * the bug we're guarding against here is load-before-init
     * clobbering. */
    EXPECT(maestro_peripheral.load_config(&storage),
           "load_config returned true");
    const maestro_channel_config_t* got = maestro_get_channel_config(5);
    EXPECT(got != NULL,                "load_config produced channel 5");
    EXPECT(got->min_pulse_us == 800,   "channel 5 min_pulse restored");
    EXPECT(got->max_pulse_us == 2200,  "channel 5 max_pulse restored");
    EXPECT(g_transport_mode == FLASH_MAESTRO_TRANSPORT_UART,
           "transport_mode restored");
    EXPECT(g_transport == &uart_ops,
           "transport bound to uart ops after load");
}

static void case_load_then_init_no_clobber(void)
{
    printf("case_load_then_init_no_clobber\n");
    reset_all();

    /* Manually build a storage struct with non-default channel values
     * (mimics what the framework does on real boot: register → load
     * → init). The order matters: load runs first, init runs after.
     * If maestro_init reset-to-defaults regardless of g_initialized,
     * the loaded values would be lost. */
    flash_storage_data_t storage;
    memset(&storage, 0, sizeof(storage));
    storage.maestro_config.channel_count  = MAESTRO_MAX_CHANNELS;
    storage.maestro_config.transport_mode = FLASH_MAESTRO_TRANSPORT_UART;
    storage.maestro_config.channels[0].min_pulse_us = 555;
    storage.maestro_config.channels[0].max_pulse_us = 2444;

    /* Simulate: register → load (no init yet). */
    EXPECT(maestro_peripheral.load_config(&storage),
           "load_config returned true");
    EXPECT(g_initialized, "load_config ran ensure_state_init");
    EXPECT(maestro_get_channel_config(0)->min_pulse_us == 555,
           "channel 0 min_pulse loaded");

    /* Now: peripheral_init_all → drv_init → maestro_init. Must NOT
     * clobber the freshly loaded value. */
    maestro_init();
    EXPECT(maestro_get_channel_config(0)->min_pulse_us == 555,
           "channel 0 still 555 after maestro_init");
    EXPECT(maestro_get_channel_config(0)->max_pulse_us == 2444,
           "channel 0 still 2444 after maestro_init");
}

static void case_parse_json_transport(void)
{
    printf("case_parse_json_transport\n");
    reset_all();
    maestro_init();
    g_transport_mode = FLASH_MAESTRO_TRANSPORT_USB_HOST;

    pin_config_t pc;
    memset(&pc, 0, sizeof(pc));
    const char* json = "{\"min_pulse_us\":1000,\"transport\":\"uart\"}";
    EXPECT(maestro_peripheral.parse_json_params(json, json + strlen(json), &pc),
           "parse_json returned true");
    EXPECT(g_transport_mode == FLASH_MAESTRO_TRANSPORT_UART,
           "transport=uart parsed");
    EXPECT(pc.params.maestro.min_pulse_us == 1000,
           "min_pulse_us parsed alongside transport");

    g_transport_mode = FLASH_MAESTRO_TRANSPORT_UART;
    const char* json2 = "{\"transport\":\"usb_host\"}";
    EXPECT(maestro_peripheral.parse_json_params(json2, json2 + strlen(json2), &pc),
           "parse_json(usb_host) returned true");
    EXPECT(g_transport_mode == FLASH_MAESTRO_TRANSPORT_USB_HOST,
           "transport=usb_host parsed");
}

static void case_parse_json_per_channel(void)
{
    /* Per-channel override path: when the incoming JSON has a
     * "channels" array, the driver should pick out the N-th entry
     * (where N is derived from pcfg->gpio - MAESTRO_VIRTUAL_GPIO_BASE)
     * and use those values instead of the peripheral-level defaults.
     * Mirrors what apply_one_peripheral does on the host side: same
     * JSON, different pcfg per channel. */
    printf("case_parse_json_per_channel\n");
    reset_all();
    maestro_init();

    /* JSON with peripheral-level defaults AND a channels array.
     * Channel 0 overrides home_us to 1100, channel 2 overrides
     * min_pulse_us to 1200, channel 1 leaves everything default. */
    const char* json =
        "{\"min_pulse_us\":1000,\"max_pulse_us\":2000,\"home_us\":1500,"
         "\"channels\":["
            "{\"label\":\"a\",\"home_us\":1100,\"min_pulse_us\":1000,\"max_pulse_us\":2000},"
            "{\"label\":\"b\"},"
            "{\"label\":\"c\",\"min_pulse_us\":1200,\"home_us\":1500}"
         "]}";

    pin_config_t pc;

    /* Channel 0: expect home_us=1100 from the channels override. */
    memset(&pc, 0, sizeof(pc));
    pc.gpio = MAESTRO_VIRTUAL_GPIO_BASE + 0;
    EXPECT(maestro_peripheral.parse_json_params(json, json + strlen(json), &pc),
           "parse(ch0): returned true");
    EXPECT(pc.params.maestro.home_us == 1100,
           "parse(ch0): channels[0].home_us overrides peripheral default");
    EXPECT(pc.params.maestro.min_pulse_us == 1000,
           "parse(ch0): min_pulse_us also from channels[0]");

    /* Channel 1: channels[1] is empty so peripheral defaults apply. */
    memset(&pc, 0, sizeof(pc));
    pc.gpio = MAESTRO_VIRTUAL_GPIO_BASE + 1;
    EXPECT(maestro_peripheral.parse_json_params(json, json + strlen(json), &pc),
           "parse(ch1): returned true");
    EXPECT(pc.params.maestro.home_us == 1500,
           "parse(ch1): empty channels[1] → peripheral home_us applies");
    EXPECT(pc.params.maestro.min_pulse_us == 1000,
           "parse(ch1): empty channels[1] → peripheral min_pulse_us applies");

    /* Channel 2: min_pulse_us from channels[2]. */
    memset(&pc, 0, sizeof(pc));
    pc.gpio = MAESTRO_VIRTUAL_GPIO_BASE + 2;
    EXPECT(maestro_peripheral.parse_json_params(json, json + strlen(json), &pc),
           "parse(ch2): returned true");
    EXPECT(pc.params.maestro.min_pulse_us == 1200,
           "parse(ch2): channels[2].min_pulse_us overrides peripheral default");

    /* Missing channels[] array → peripheral defaults for any channel. */
    const char* json_no_arr = "{\"min_pulse_us\":900,\"home_us\":1400}";
    memset(&pc, 0, sizeof(pc));
    pc.gpio = MAESTRO_VIRTUAL_GPIO_BASE + 0;
    EXPECT(maestro_peripheral.parse_json_params(json_no_arr, json_no_arr + strlen(json_no_arr), &pc),
           "parse(no channels[]): returned true");
    EXPECT(pc.params.maestro.min_pulse_us == 900,
           "parse(no channels[]): peripheral min_pulse_us applies");
    EXPECT(pc.params.maestro.home_us == 1400,
           "parse(no channels[]): peripheral home_us applies");
}

static void case_parse_json_full_maestro_payload(void)
{
    /* Regression test: with the new params["channels"] array, a full
     * 24-channel Maestro peripheral serializes to ~3500 bytes — well
     * past the firmware's 2048-byte config_buffer (and waaay past the
     * 512-byte XRCE-DDS MTU, which forces fragmentation). The crash
     * the operator sees happens at the rmw_microxrcedds reception
     * layer when the reassembled message overruns config_msg.data
     * capacity. The parser layer itself should be safe regardless.
     *
     * This test builds a realistic full payload and confirms:
     *   - Parser doesn't crash
     *   - Per-channel parsing scales to 24 entries
     *   - Picking any given channel from the array works
     *
     * The buffer-size fix lives in firmware/teensy41/src/main.cpp
     * and firmware/rp2040/src/main.c (config_buffer + capacity bumps).
     * See docs/MAESTRO_BRINGUP.md for the wire-size constraints. */
    printf("case_parse_json_full_maestro_payload\n");
    reset_all();
    maestro_init();

    /* Build a 24-channel payload mirroring what the server emits. */
    char json[8192];
    int n = snprintf(json, sizeof(json),
        "{\"min_pulse_us\":1000,\"max_pulse_us\":2000,"
         "\"channel_count\":24,\"channels\":[");
    for (int ch = 0; ch < 24; ch++) {
        n += snprintf(json + n, sizeof(json) - n,
            "%s{\"label\":\"Ch %d\",\"min_pulse_us\":%d,\"max_pulse_us\":%d,"
             "\"neutral_us\":1500,\"home_us\":%d,\"speed\":0,\"acceleration\":0}",
            ch ? "," : "",
            ch,
            1000 + ch * 5,        /* sneak ch index in via min so we can verify */
            1800 + ch * 5,
            1400 + ch * 10);      /* and via home */
    }
    n += snprintf(json + n, sizeof(json) - n, "]}");

    /* The payload should be larger than the current 2048-byte
     * config_buffer — that's the actual bug.  We don't fail the test
     * on size (since the parser itself is fine), but log it for
     * humans reading the output. */
    printf("  payload size = %d bytes (config_buffer was 2048)\n", n);
    EXPECT(n > 2048, "payload exceeds legacy 2048-byte config_buffer");

    pin_config_t pc;
    /* Probe a few channels — verifies the per-channel array walk
     * doesn't trip on the large input. */
    const int probes[] = {0, 7, 15, 23};
    for (size_t i = 0; i < sizeof(probes)/sizeof(probes[0]); i++) {
        int probe = probes[i];
        memset(&pc, 0, sizeof(pc));
        pc.gpio = MAESTRO_VIRTUAL_GPIO_BASE + probe;
        EXPECT(maestro_peripheral.parse_json_params(json, json + n, &pc),
               "parse(full payload): returned true");
        uint16_t expected_min  = (uint16_t)(1000 + probe * 5);
        uint16_t expected_max  = (uint16_t)(1800 + probe * 5);
        uint16_t expected_home = (uint16_t)(1400 + probe * 10);
        EXPECT(pc.params.maestro.min_pulse_us == expected_min,
               "parse(full payload, ch N): min_pulse_us matches");
        EXPECT(pc.params.maestro.max_pulse_us == expected_max,
               "parse(full payload, ch N): max_pulse_us matches");
        EXPECT(pc.params.maestro.home_us == expected_home,
               "parse(full payload, ch N): home_us matches");
    }
}

static void case_parse_json_truncated(void)
{
    /* Regression test: simulate what happens when the wire payload is
     * truncated mid-array (the legacy bug: buffer-too-small + null-
     * terminate-at-capacity-boundary mid-JSON). The parser must NOT
     * crash, must NOT walk past json_end, and may silently fall back
     * to peripheral-level defaults.
     *
     * This test won't catch the rmw-layer crash that's the actual
     * operator-visible symptom (that's a buffer overrun deeper in
     * micro-ROS), but it does prove the parser is safe to call on
     * any truncation point. */
    printf("case_parse_json_truncated\n");
    reset_all();
    maestro_init();

    char json[2048];
    int n = snprintf(json, sizeof(json),
        "{\"min_pulse_us\":1234,\"home_us\":1567,\"channels\":[");
    for (int ch = 0; ch < 24; ch++) {
        n += snprintf(json + n, sizeof(json) - n,
            "%s{\"label\":\"Ch %d\",\"min_pulse_us\":%d,\"max_pulse_us\":%d,"
             "\"neutral_us\":1500,\"home_us\":1500,\"speed\":0,\"acceleration\":0}",
            ch ? "," : "",
            ch, 1000 + ch, 2000 + ch);
        if (n > 1800) break;  /* truncate before we'd close the array */
    }
    /* Note: deliberately NO closing "]}" — simulates wire truncation. */
    json[n] = '\0';
    EXPECT(n > 1500, "truncated test set up payload large enough to matter");

    pin_config_t pc;
    memset(&pc, 0, sizeof(pc));
    pc.gpio = MAESTRO_VIRTUAL_GPIO_BASE + 0;
    /* Survive without crashing. ASan / UBSan would catch out-of-bound
     * reads here if they happened. */
    bool ok = maestro_peripheral.parse_json_params(json, json + n, &pc);
    EXPECT(ok, "parse(truncated): returned without crashing");

    /* If the parser COULDN'T find a complete channels[0] entry it
     * falls back to peripheral-level. Either outcome is acceptable;
     * what matters is no memory corruption. */
    bool fell_back = (pc.params.maestro.min_pulse_us == 1234);
    bool used_ch0  = (pc.params.maestro.min_pulse_us == 1000);
    EXPECT(fell_back || used_ch0,
           "parse(truncated): produced a reasonable value (either peripheral default or ch0 override)");
}

static void case_default_transport_pick(void)
{
    printf("case_default_transport_pick\n");

    /* Both available → prefer USB host. */
    reset_all();
    usb_available = true;
    uart_available = true;
    maestro_init();
    EXPECT(g_transport_mode == FLASH_MAESTRO_TRANSPORT_USB_HOST,
           "default = USB host when both available");

    /* USB unavailable → fall through to UART. */
    reset_all();
    usb_available = false;
    uart_available = true;
    maestro_init();
    EXPECT(g_transport_mode == FLASH_MAESTRO_TRANSPORT_UART,
           "default = UART when USB unavailable");

    /* Neither available → stays at whatever was last set. The shared
     * driver doesn't try to invent a working transport on a platform
     * that can't provide one; it just stays inert. */
    reset_all();
    usb_available = false;
    uart_available = false;
    g_transport_mode = FLASH_MAESTRO_TRANSPORT_USB_HOST;
    maestro_init();
    /* No useful claim to make here other than "didn't crash". */
}

static void case_load_unsupported_transport(void)
{
    printf("case_load_unsupported_transport\n");
    reset_all();
    usb_available  = false;   /* simulate RP2040 */
    uart_available = true;

    flash_storage_data_t storage;
    memset(&storage, 0, sizeof(storage));
    storage.maestro_config.channel_count  = MAESTRO_MAX_CHANNELS;
    storage.maestro_config.transport_mode = FLASH_MAESTRO_TRANSPORT_USB_HOST;
    storage.maestro_config.channels[0].min_pulse_us = 1500;

    /* load_config should not crash, should return true, and should
     * leave g_transport as NULL (inert) — channel state still loads. */
    EXPECT(maestro_peripheral.load_config(&storage),
           "load_config returned true even with unsupported mode");
    EXPECT(g_transport == NULL,
           "transport NULL when platform can't supply the mode");
    EXPECT(maestro_get_channel_config(0)->min_pulse_us == 1500,
           "channel data still loaded");
}

/* Erased / never-saved flash slot — drv_load must NOT bind a
 * transport and must NOT log an error. Regression for the post-OTA
 * "Maestro: no transport for mode=255" log on nodes with no Maestro
 * configured: the maestro_config bytes are 0xFF (erased) or zero
 * (never written), and the driver was applying them as if they were
 * a real saved record. */
static void case_load_no_record(void)
{
    printf("case_load_no_record\n");

    /* Erased-flash slot (0xFF everywhere). */
    reset_all();
    usb_available  = true;
    uart_available = true;
    flash_storage_data_t storage;
    memset(&storage, 0xFF, sizeof(storage));
    EXPECT(maestro_peripheral.load_config(&storage),
           "erased slot: load_config returns true");
    EXPECT(g_transport == NULL,
           "erased slot: no transport bound");

    /* Zero-initialized slot (struct was written but Maestro region
     * untouched — channel_count==0 sentinel). */
    reset_all();
    usb_available  = true;
    uart_available = true;
    memset(&storage, 0, sizeof(storage));
    EXPECT(maestro_peripheral.load_config(&storage),
           "zero slot: load_config returns true");
    EXPECT(g_transport == NULL,
           "zero slot: no transport bound");
}

int main(void)
{
    case_compact_protocol_bytes();
    case_channel_config_roundtrip();
    case_parse_json_per_channel();
    case_parse_json_full_maestro_payload();
    case_parse_json_truncated();
    case_save_load_roundtrip();
    case_load_then_init_no_clobber();
    case_parse_json_transport();
    case_default_transport_pick();
    case_load_unsupported_transport();
    case_load_no_record();

    if (fail_count) {
        printf("\ntest_maestro_driver: %d failure(s)\n", fail_count);
        return 1;
    }
    printf("\ntest_maestro_driver: OK\n");
    return 0;
}
