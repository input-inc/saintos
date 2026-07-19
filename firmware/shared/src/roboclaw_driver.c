/**
 * SAINT.OS Firmware - RoboClaw Solo 60A Motor Controller Driver (shared core)
 *
 * Drives up to 8 BasicMicro RoboClaw Solo 60A motor controllers over
 * CRC16 packet serial (addresses 0x80-0x87). All UART I/O dispatches
 * through the per-platform transport ops table (shared/include/
 * roboclaw_transport.h), so the protocol layer, connection state
 * machine, duty keepalive, save/load, and wire-debug passthrough live
 * once and work identically on RP2040 and Teensy 4.1.
 *
 * Per-unit virtual GPIO layout: 5 channels per unit (motor, encoder,
 * voltage, current, temp). Telemetry is polled round-robin from
 * roboclaw_update() to avoid blocking; disconnected units are
 * re-probed on a slower pace.
 *
 * Wiring concerns the shared driver knows about but the transport
 * resolves:
 *   - HW UART vs PIO UART: RP2040 PCBs that route TX→TX (not
 *     crossed) need the PIO path so the wires can be on any GPIO.
 *     The shared driver decides based on units[0].uart_swap and
 *     passes pio_swap to transport.open().
 *   - GPIO function clobbering: at boot some other peripheral's
 *     init can re-route our pads — RP2040 transport.verify_binding
 *     catches this and forces a re-bind.
 *   - Inverted signaling: not used (RoboClaw is 8N1 non-inverted).
 *
 * E-stop pin handling is platform-specific GPIO output — kept here
 * with a per-platform helper (apply_estop_pin / assert_estop_pin)
 * gated on the SAINT_ROBOCLAW_HAS_ESTOP_GPIO macro. RP2040 implements
 * the GPIO calls; the shared core's no-op implementation is used on
 * platforms without GPIO output convention (Teensy follows the same
 * pattern but the API isn't wired here yet — see roboclaw_drv_estop
 * which still works without per-unit pin assertion).
 */

#include "roboclaw_driver.h"
#include "roboclaw_transport.h"

#include "flash_types.h"
#include "peripheral_driver.h"
#include "pin_types.h"
#include "platform.h"
#include "saint_log.h"
#include "uart_pin_pairs.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ── Configuration ──────────────────────────────────────────────── */

#define ROBOCLAW_DEFAULT_TX_PIN     0
#define ROBOCLAW_DEFAULT_RX_PIN     1
#define ROBOCLAW_DEFAULT_SERIAL_PORT 0

#define ROBOCLAW_DROP_AFTER_MISSES  3
#define ROBOCLAW_REPROBE_INTERVAL_MS 2000
#define ROBOCLAW_DUTY_KEEPALIVE_MS  400

static uint8_t roboclaw_tx_pin = ROBOCLAW_DEFAULT_TX_PIN;
static uint8_t roboclaw_rx_pin = ROBOCLAW_DEFAULT_RX_PIN;

static uint8_t active_tx_pin = 0xFF;
static uint8_t active_rx_pin = 0xFF;
static uint8_t active_uart   = 0xFF;

/* ── Per-Unit State ─────────────────────────────────────────────── */

typedef struct {
    uint8_t  address;
    uint8_t  deadband;
    uint16_t max_current_ma;
    int16_t  duty;
    int32_t  encoder;
    uint16_t voltage_mv;
    int16_t  current_ma;
    int16_t  temp_tenths;
    bool     connected;
    bool     first_duty_logged;
    uint32_t last_response_ms;
    uint8_t  consecutive_misses;
    uint32_t last_reprobe_ms;
    uint8_t  estop_pin;
    uint8_t  uart_swap;
    uint8_t  invert_direction;
    uint32_t last_duty_send_ms;
    /* Fault status (Read Status, cmd 90). fault_flags is the canonical
     * ROBOCLAW_FAULT_* set, normalized from whichever raw word the
     * controller returns. prev_fault_flags drives edge-triggered logging;
     * status_len caches the auto-detected response length (2 or 4 bytes,
     * 0 = not yet detected). peripheral_id labels this unit's records on
     * the peripheral-first state_emit path (no virtual GPIO involved). */
    uint16_t fault_flags;
    uint16_t prev_fault_flags;
    bool     status_valid;
    uint8_t  status_len;
    char     peripheral_id[32];
} roboclaw_unit_t;

/* ── Driver State ───────────────────────────────────────────────── */

static roboclaw_unit_t units[ROBOCLAW_MAX_UNITS];
static uint8_t unit_count = 0;
static bool port_initialized = false;
static uint16_t configured_baud = ROBOCLAW_DEFAULT_BAUD;
static uint8_t configured_serial_port = ROBOCLAW_DEFAULT_SERIAL_PORT;

static uint8_t poll_unit = 0;
static uint8_t poll_register = 0;

/* True when the transport is using the PIO UART path; false when it's
 * using the hardware UART. Mirrors what we asked transport.open to do
 * (and, on RP2040, whether the transport fell back to HW after a PIO
 * init failure). Set here in roboclaw_init() so host tests can verify
 * the decision logic without running the actual PIO peripheral. */
static bool use_pio_uart = false;

/* ── Wire-level diagnostic counters ─────────────────────────────── */

static uint32_t wire_tx_packets   = 0;
static uint32_t wire_tx_bytes     = 0;
static uint32_t wire_ack_ok       = 0;
static uint32_t wire_ack_timeout  = 0;
static uint32_t wire_ack_wrong    = 0;
static uint32_t wire_resp_ok      = 0;
static uint32_t wire_resp_short   = 0;
static uint32_t wire_resp_crc_bad = 0;
static uint8_t  wire_ack_last     = 0;
static uint32_t wire_stats_last_dump_ms = 0;
#define WIRE_STATS_DUMP_MS  5000

static const roboclaw_transport_ops_t* transport(void)
{
    return roboclaw_get_transport();
}

static void wire_stats_maybe_dump(uint32_t now)
{
    if (wire_tx_packets == 0) return;
    if (wire_stats_last_dump_ms == 0) {
        wire_stats_last_dump_ms = now;
        return;
    }
    if (now - wire_stats_last_dump_ms < WIRE_STATS_DUMP_MS) return;

    uint32_t window_ms = now - wire_stats_last_dump_ms;
    saint_log_publish("info",
        "RoboClaw wire (%lu ms): tx=%lu pkts/%lu bytes, ack=%lu ok/"
        "%lu timeout/%lu wrong (last=0x%02X), resp=%lu ok/%lu short/"
        "%lu crc_bad",
        (unsigned long)window_ms,
        (unsigned long)wire_tx_packets, (unsigned long)wire_tx_bytes,
        (unsigned long)wire_ack_ok, (unsigned long)wire_ack_timeout,
        (unsigned long)wire_ack_wrong, (unsigned)wire_ack_last,
        (unsigned long)wire_resp_ok, (unsigned long)wire_resp_short,
        (unsigned long)wire_resp_crc_bad);

    wire_tx_packets = 0;
    wire_tx_bytes = 0;
    wire_ack_ok = 0;
    wire_ack_timeout = 0;
    wire_ack_wrong = 0;
    wire_resp_ok = 0;
    wire_resp_short = 0;
    wire_resp_crc_bad = 0;
    wire_stats_last_dump_ms = now;
}

static void temp_probe_diagnostic(uint32_t now);

/* ── Internal Helpers — wire I/O abstraction ────────────────────── */

static inline bool wire_ready(void)
{
    const roboclaw_transport_ops_t* t = transport();
    return t && t->is_open();
}

static inline void wire_write_blocking(const uint8_t* buf, size_t len)
{
    const roboclaw_transport_ops_t* t = transport();
    if (t && t->is_open()) (void)t->write(buf, len);
}

static inline bool wire_is_readable(void)
{
    const roboclaw_transport_ops_t* t = transport();
    if (!t || !t->is_open()) return false;
    /* Peek by attempting a 0-byte read isn't portable; the transports
     * expose a single read primitive that returns 0 when nothing is
     * available. The helper below buffers one byte for the read-byte
     * polling style the protocol layer uses below. */
    return true;
}

/* Read a single byte if available; returns -1 if not. The transport's
 * read() function fills as much as is available, but the protocol
 * layer prefers a one-at-a-time pattern with per-byte timeouts. We
 * adapt by maintaining a tiny look-ahead buffer. */
static uint8_t wire_lookahead_buf[64];
static size_t  wire_lookahead_n = 0;
static size_t  wire_lookahead_i = 0;

static int wire_try_getc(void)
{
    const roboclaw_transport_ops_t* t = transport();
    if (!t || !t->is_open()) return -1;
    if (wire_lookahead_i >= wire_lookahead_n) {
        wire_lookahead_n = t->read(wire_lookahead_buf,
                                   sizeof(wire_lookahead_buf));
        wire_lookahead_i = 0;
        if (wire_lookahead_n == 0) return -1;
    }
    return (int)wire_lookahead_buf[wire_lookahead_i++];
}

static inline uint8_t wire_getc(void)
{
    int b = wire_try_getc();
    return (b < 0) ? 0 : (uint8_t)b;
}

/* True if there's a byte available to read RIGHT NOW (re-fills the
 * look-ahead buffer from the transport so the protocol layer's
 * "while (!wire_is_readable()) timeout" loop sees fresh state every
 * iteration). */
static bool wire_has_byte(void)
{
    if (wire_lookahead_i < wire_lookahead_n) return true;
    const roboclaw_transport_ops_t* t = transport();
    if (!t || !t->is_open()) return false;
    wire_lookahead_n = t->read(wire_lookahead_buf,
                               sizeof(wire_lookahead_buf));
    wire_lookahead_i = 0;
    return wire_lookahead_n > 0;
}

static void send_command(uint8_t address, uint8_t command,
                         const uint8_t* data, uint8_t data_len)
{
    if (!wire_ready()) return;

    uint8_t packet[64];
    uint8_t idx = 0;

    packet[idx++] = address;
    packet[idx++] = command;
    for (uint8_t i = 0; i < data_len; i++) {
        packet[idx++] = data[i];
    }

    uint16_t crc = roboclaw_crc16_calculate(packet, idx);
    packet[idx++] = (uint8_t)(crc >> 8);
    packet[idx++] = (uint8_t)(crc & 0xFF);

    wire_write_blocking(packet, idx);
    wire_tx_packets++;
    wire_tx_bytes += idx;
}

static bool read_response(uint8_t* buffer, uint8_t expected_len,
                           uint8_t address, uint8_t command)
{
    if (!wire_ready()) return false;

    uint8_t total = expected_len + 2;  /* data + 2 CRC bytes */
    uint32_t start = PLATFORM_MILLIS();

    for (uint8_t i = 0; i < total; i++) {
        while (!wire_has_byte()) {
            if (PLATFORM_MILLIS() - start > ROBOCLAW_RESPONSE_TIMEOUT_MS) {
                wire_resp_short++;
                return false;
            }
        }
        buffer[i] = wire_getc();
        start = PLATFORM_MILLIS();
    }

    uint16_t crc = 0;
    crc = roboclaw_crc16_update(crc, address);
    crc = roboclaw_crc16_update(crc, command);
    for (uint8_t i = 0; i < expected_len; i++) {
        crc = roboclaw_crc16_update(crc, buffer[i]);
    }

    uint16_t received_crc = ((uint16_t)buffer[expected_len] << 8) |
                             buffer[expected_len + 1];
    if (crc == received_crc) {
        wire_resp_ok++;
        return true;
    }
    wire_resp_crc_bad++;
    return false;
}

static bool read_ack(void)
{
    if (!wire_ready()) return false;

    uint32_t start = PLATFORM_MILLIS();
    while (!wire_has_byte()) {
        if (PLATFORM_MILLIS() - start > ROBOCLAW_RESPONSE_TIMEOUT_MS) {
            wire_ack_timeout++;
            return false;
        }
    }
    uint8_t got = wire_getc();
    if (got == ROBOCLAW_ACK_BYTE) {
        wire_ack_ok++;
        return true;
    }
    wire_ack_wrong++;
    wire_ack_last = got;
    return false;
}

/* ── Standalone GETTEMP diagnostic ──────────────────────────────── */
/* See the rationale in firmware/rp2040/src/roboclaw_driver.c (this is
 * the same helper, ported to the wire_*() abstraction). Bypasses the
 * round-robin's connected-state gate so any byte the controller
 * sends shows up in the log verbatim. */
#define TEMP_PROBE_INTERVAL_MS 1000

static uint32_t temp_probe_last_ms = 0;

static void temp_probe_diagnostic(uint32_t now)
{
    if (!port_initialized || unit_count == 0) return;
    if (!wire_ready()) return;
    if (now - temp_probe_last_ms < TEMP_PROBE_INTERVAL_MS) return;
    temp_probe_last_ms = now;

    uint8_t addr = units[0].address;

    uint8_t pre_drain = 0;
    uint8_t pre_drain_bytes[8] = {0};
    while (wire_has_byte()) {
        uint8_t b = wire_getc();
        if (pre_drain < (uint8_t)(sizeof(pre_drain_bytes))) {
            pre_drain_bytes[pre_drain] = b;
        }
        pre_drain++;
    }
    if (pre_drain > 0) {
        char buf[40];
        int n = snprintf(buf, sizeof(buf), "%u byte%s (",
                         (unsigned)pre_drain, pre_drain == 1 ? "" : "s");
        for (uint8_t i = 0; i < pre_drain && i < 8 && n < (int)sizeof(buf) - 4; i++) {
            n += snprintf(buf + n, sizeof(buf) - n, "%s%02X",
                          i == 0 ? "" : " ", (unsigned)pre_drain_bytes[i]);
        }
        if (n < (int)sizeof(buf) - 4) snprintf(buf + n, sizeof(buf) - n, "%s)",
            pre_drain > 8 ? " …" : "");
        saint_log_publish("warn",
            "RoboClaw temp-probe: drained %s of stale RX BEFORE sending — "
            "previous exchange left orphans OR noise on idle RX",
            buf);
    }

    send_command(addr, ROBOCLAW_CMD_GETTEMP, NULL, 0);

    uint8_t  buf[4] = {0, 0, 0, 0};
    uint8_t  got    = 0;
    uint32_t start  = PLATFORM_MILLIS();
    while (got < 4) {
        while (!wire_has_byte()) {
            uint32_t waited = PLATFORM_MILLIS() - start;
            uint32_t limit  = (got == 0) ? ROBOCLAW_RESPONSE_TIMEOUT_MS
                                         : ROBOCLAW_BYTE_TIMEOUT_MS;
            if (waited > limit) goto report;
        }
        buf[got++] = wire_getc();
        start = PLATFORM_MILLIS();
    }

report:
    if (got == 0) {
        uint8_t  late[8] = {0};
        uint8_t  late_n  = 0;
        uint32_t late_deadline = PLATFORM_MILLIS() + 10;
        while (PLATFORM_MILLIS() < late_deadline && late_n < 8) {
            if (wire_has_byte()) {
                late[late_n++] = wire_getc();
            }
        }
        if (late_n == 0) {
            saint_log_publish("info",
                "RoboClaw temp-probe: sent [0x%02X, 82] to unit 0 — NO bytes "
                "received within %u ms NOR in the 10 ms grace window "
                "(controller silent or wiring blocked)",
                (unsigned)addr, (unsigned)ROBOCLAW_RESPONSE_TIMEOUT_MS);
        } else {
            char lbuf[40];
            int ln = snprintf(lbuf, sizeof(lbuf), "%u byte%s [",
                              (unsigned)late_n, late_n == 1 ? "" : "s");
            for (uint8_t i = 0; i < late_n && ln < (int)sizeof(lbuf) - 4; i++) {
                ln += snprintf(lbuf + ln, sizeof(lbuf) - ln, "%s%02X",
                               i == 0 ? "" : " ", (unsigned)late[i]);
            }
            if (ln < (int)sizeof(lbuf) - 2) snprintf(lbuf + ln, sizeof(lbuf) - ln, "]");
            saint_log_publish("warn",
                "RoboClaw temp-probe: sent [0x%02X, 82] to unit 0 — 0 bytes "
                "in %u ms BUT %s arrived in the grace window. Controller IS "
                "alive; our timeout/framing is slipping. Check the "
                "configured baud (%u) and bus integrity.",
                (unsigned)addr, (unsigned)ROBOCLAW_RESPONSE_TIMEOUT_MS,
                lbuf, (unsigned)configured_baud);
        }
        return;
    }

    bool        crc_ok        = false;
    uint16_t    expected_crc  = 0;
    uint16_t    received_crc  = 0;
    int16_t     temp_tenths_c = 0;
    if (got == 4) {
        uint16_t crc = 0;
        crc = roboclaw_crc16_update(crc, addr);
        crc = roboclaw_crc16_update(crc, ROBOCLAW_CMD_GETTEMP);
        crc = roboclaw_crc16_update(crc, buf[0]);
        crc = roboclaw_crc16_update(crc, buf[1]);
        expected_crc  = crc;
        received_crc  = ((uint16_t)buf[2] << 8) | buf[3];
        crc_ok        = (expected_crc == received_crc);
        temp_tenths_c = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    }

    if (crc_ok) {
        saint_log_publish("info",
            "RoboClaw temp-probe: unit 0 (0x%02X) got %u bytes "
            "[%02X %02X %02X %02X] — CRC OK, temp=%d.%d °C",
            (unsigned)addr, (unsigned)got,
            (unsigned)buf[0], (unsigned)buf[1],
            (unsigned)buf[2], (unsigned)buf[3],
            (int)(temp_tenths_c / 10),
            (int)((temp_tenths_c < 0 ? -temp_tenths_c : temp_tenths_c) % 10));
    } else if (got == 4) {
        saint_log_publish("warn",
            "RoboClaw temp-probe: unit 0 (0x%02X) got %u bytes "
            "[%02X %02X %02X %02X] — CRC MISMATCH "
            "(expected 0x%04X, received 0x%04X). "
            "Bytes ARE arriving but the wire is corrupting them.",
            (unsigned)addr, (unsigned)got,
            (unsigned)buf[0], (unsigned)buf[1],
            (unsigned)buf[2], (unsigned)buf[3],
            (unsigned)expected_crc, (unsigned)received_crc);
    } else {
        saint_log_publish("warn",
            "RoboClaw temp-probe: unit 0 (0x%02X) got %u of 4 bytes "
            "[%02X %02X %02X %02X] then timed out — short response, "
            "controller likely started replying but lost framing.",
            (unsigned)addr, (unsigned)got,
            (unsigned)buf[0], (unsigned)buf[1],
            (unsigned)buf[2], (unsigned)buf[3]);
    }
}

/* ── Wire-level debug passthrough ───────────────────────────────── */

static uint8_t debug_parse_hex(const char* s, const char* end,
                                uint8_t* out, uint8_t out_max)
{
    uint8_t n = 0;
    while (s < end && n < out_max) {
        while (s < end && (*s == ' ' || *s == ',')) s++;
        if (s + 1 >= end) break;
        char h = s[0], l = s[1];
        int hi = (h >= '0' && h <= '9') ? h - '0'
               : (h >= 'a' && h <= 'f') ? h - 'a' + 10
               : (h >= 'A' && h <= 'F') ? h - 'A' + 10 : -1;
        int lo = (l >= '0' && l <= '9') ? l - '0'
               : (l >= 'a' && l <= 'f') ? l - 'a' + 10
               : (l >= 'A' && l <= 'F') ? l - 'A' + 10 : -1;
        if (hi < 0 || lo < 0) break;
        out[n++] = (uint8_t)((hi << 4) | lo);
        s += 2;
    }
    return n;
}

static void debug_format_hex(const uint8_t* in, uint8_t in_len,
                              char* out, size_t out_size)
{
    size_t pos = 0;
    for (uint8_t i = 0; i < in_len && pos + 3 < out_size; i++) {
        pos += (size_t)snprintf(out + pos, out_size - pos,
                                 "%s%02X", i == 0 ? "" : " ",
                                 (unsigned)in[i]);
    }
    if (pos < out_size) out[pos] = '\0';
    else if (out_size > 0) out[out_size - 1] = '\0';
}

static void debug_op_raw(const char* json)
{
    const char* p = strstr(json, "\"tx_hex\"");
    if (!p) {
        saint_log_publish("warn", "roboclaw_dbg: raw missing tx_hex");
        return;
    }
    p = strchr(p, ':');
    if (!p) return;
    p++;
    while (*p == ' ') p++;
    if (*p != '"') {
        saint_log_publish("warn", "roboclaw_dbg: raw tx_hex must be a string");
        return;
    }
    p++;
    const char* tx_end = strchr(p, '"');
    if (!tx_end) return;
    uint8_t tx[32];
    uint8_t tx_len = debug_parse_hex(p, tx_end, tx, sizeof(tx));
    if (tx_len == 0) {
        saint_log_publish("warn", "roboclaw_dbg: raw tx_hex parsed 0 bytes");
        return;
    }

    uint8_t read_len = 4;
    const char* q = strstr(json, "\"read_len\"");
    if (q && (q = strchr(q, ':'))) {
        q++; while (*q == ' ') q++;
        int v = atoi(q);
        if (v > 0 && v <= 64) read_len = (uint8_t)v;
    }

    uint16_t timeout_ms = 100;
    q = strstr(json, "\"timeout_ms\"");
    if (q && (q = strchr(q, ':'))) {
        q++; while (*q == ' ') q++;
        int v = atoi(q);
        if (v > 0 && v <= 10000) timeout_ms = (uint16_t)v;
    }

    if (!wire_ready()) {
        saint_log_publish("warn", "roboclaw_dbg: raw — UART not bound");
        return;
    }

    uint8_t drain[16];
    uint8_t drain_n = 0;
    while (wire_has_byte() && drain_n < (uint8_t)sizeof(drain)) {
        drain[drain_n++] = wire_getc();
    }
    char drain_hex[64];
    debug_format_hex(drain, drain_n, drain_hex, sizeof(drain_hex));

    char tx_hex[80];
    debug_format_hex(tx, tx_len, tx_hex, sizeof(tx_hex));
    uint32_t t_send = PLATFORM_MILLIS();
    wire_write_blocking(tx, tx_len);

    uint8_t rx[64];
    uint8_t rx_n = 0;
    uint32_t t_first = 0;
    uint32_t t_last = 0;
    uint32_t deadline = t_send + timeout_ms;
    while (rx_n < read_len && PLATFORM_MILLIS() < deadline) {
        if (wire_has_byte()) {
            if (rx_n == 0) t_first = PLATFORM_MILLIS();
            rx[rx_n++] = wire_getc();
            t_last = PLATFORM_MILLIS();
        }
    }
    uint32_t t_done = PLATFORM_MILLIS();

    char rx_hex[200];
    debug_format_hex(rx, rx_n, rx_hex, sizeof(rx_hex));

    saint_log_publish("info",
        "roboclaw_dbg: raw tx=[%s] drained=[%s](%u) rx=[%s] "
        "got=%u/%u in %lu ms first@+%lu last@+%lu",
        tx_hex, drain_hex, (unsigned)drain_n, rx_hex,
        (unsigned)rx_n, (unsigned)read_len,
        (unsigned long)(t_done - t_send),
        (unsigned long)(rx_n > 0 ? t_first - t_send : 0),
        (unsigned long)(rx_n > 0 ? t_last - t_send : 0));
}

static void debug_op_sniff(const char* json)
{
    uint16_t duration_ms = 1000;
    const char* p = strstr(json, "\"duration_ms\"");
    if (p && (p = strchr(p, ':'))) {
        p++; while (*p == ' ') p++;
        int v = atoi(p);
        if (v > 0 && v <= 10000) duration_ms = (uint16_t)v;
    }

    if (!wire_ready()) {
        saint_log_publish("warn", "roboclaw_dbg: sniff — UART not bound");
        return;
    }

    uint8_t buf[128];
    uint8_t n = 0;
    uint32_t deadline = PLATFORM_MILLIS() + duration_ms;
    while (PLATFORM_MILLIS() < deadline && n < (uint8_t)sizeof(buf)) {
        if (wire_has_byte()) {
            buf[n++] = wire_getc();
        }
    }

    char hex[400];
    debug_format_hex(buf, n, hex, sizeof(hex));
    saint_log_publish("info",
        "roboclaw_dbg: sniff duration=%u ms captured=%u byte%s [%s]",
        (unsigned)duration_ms, (unsigned)n, n == 1 ? "" : "s", hex);
}

static void debug_op_reconfigure(const char* json)
{
    long baud = (long)configured_baud;
    int swap_override = -1;
    const char* p = strstr(json, "\"baud\"");
    if (p && (p = strchr(p, ':'))) {
        p++; while (*p == ' ') p++;
        long v = strtol(p, NULL, 10);
        if (v >= 1200 && v <= 57600) baud = v;
    }
    p = strstr(json, "\"swap\"");
    if (p && (p = strchr(p, ':'))) {
        p++; while (*p == ' ') p++;
        swap_override = atoi(p) ? 1 : 0;
    }

    configured_baud = (uint16_t)baud;
    if (swap_override == 0 || swap_override == 1) {
        if (unit_count == 0) {
            units[0].address = ROBOCLAW_DEFAULT_ADDRESS;
            unit_count = 1;
        }
        units[0].uart_swap = (uint8_t)swap_override;
    }

    /* Force the idempotent fast-path to miss so the rebind actually
     * runs even when the pin pair didn't change. */
    active_tx_pin = 0xFF;
    active_rx_pin = 0xFF;
    active_uart   = 0xFF;
    roboclaw_init();

    saint_log_publish("info",
        "roboclaw_dbg: reconfigured baud=%ld swap=%d (active baud=%u, pio=%d)",
        baud, swap_override,
        (unsigned)configured_baud, (int)use_pio_uart);
}

void roboclaw_debug_handle_json(const char* json)
{
    if (!json) return;
    if (strstr(json, "\"op\":\"raw\"")
        || strstr(json, "\"op\": \"raw\"")) {
        debug_op_raw(json);
    } else if (strstr(json, "\"op\":\"sniff\"")
               || strstr(json, "\"op\": \"sniff\"")) {
        debug_op_sniff(json);
    } else if (strstr(json, "\"op\":\"reconfigure\"")
               || strstr(json, "\"op\": \"reconfigure\"")) {
        debug_op_reconfigure(json);
    } else {
        saint_log_publish("warn",
            "roboclaw_dbg: unknown op (expected raw|sniff|reconfigure)");
    }
}

/* ── Probe / connection tracking ────────────────────────────────── */

static bool probe_unit(uint8_t unit_idx, char* version_out, size_t version_out_size)
{
    if (!wire_ready() || unit_idx >= ROBOCLAW_MAX_UNITS) return false;

    /* Drain any stale bytes from a previous failed exchange so the
     * version parser doesn't latch onto leftover garbage. */
    while (wire_has_byte()) (void)wire_getc();

    send_command(units[unit_idx].address, ROBOCLAW_CMD_GETVERSION, NULL, 0);

    uint8_t resp[ROBOCLAW_VERSION_MAX_LEN + 2];
    uint32_t start = PLATFORM_MILLIS();
    uint8_t len = 0;
    bool got_null = false;
    while (len < ROBOCLAW_VERSION_MAX_LEN) {
        while (!wire_has_byte()) {
            if (PLATFORM_MILLIS() - start > ROBOCLAW_RESPONSE_TIMEOUT_MS) {
                return false;
            }
        }
        resp[len] = wire_getc();
        if (resp[len] == 0) {
            got_null = true;
            len++;
            break;
        }
        len++;
        start = PLATFORM_MILLIS();
    }
    if (!got_null) return false;

    for (uint8_t j = 0; j < 2; j++) {
        while (!wire_has_byte()) {
            if (PLATFORM_MILLIS() - start > ROBOCLAW_BYTE_TIMEOUT_MS) {
                return false;
            }
        }
        resp[len++] = wire_getc();
        start = PLATFORM_MILLIS();
    }

    uint16_t crc = 0;
    crc = roboclaw_crc16_update(crc, units[unit_idx].address);
    crc = roboclaw_crc16_update(crc, ROBOCLAW_CMD_GETVERSION);
    for (uint8_t i = 0; i < len - 2; i++) {
        crc = roboclaw_crc16_update(crc, resp[i]);
    }
    uint16_t received_crc = ((uint16_t)resp[len - 2] << 8) | resp[len - 1];
    if (crc != received_crc) return false;

    if (version_out && version_out_size > 0) {
        size_t copy_len = (len >= 2) ? (size_t)(len - 2) : 0;
        if (copy_len >= version_out_size) copy_len = version_out_size - 1;
        memcpy(version_out, resp, copy_len);
        version_out[copy_len] = '\0';
    }
    return true;
}

/* ── E-stop pin handling ────────────────────────────────────────── */
/* Platform-specific GPIO output. Each platform's transport.c provides
 * the implementations; host tests link in no-op fallbacks via the
 * SAINT_ROBOCLAW_NO_ESTOP_GPIO compile flag. */

#if defined(SAINT_ROBOCLAW_NO_ESTOP_GPIO)
static void roboclaw_estop_apply_pin(uint8_t pin)  { (void)pin; }
static void roboclaw_estop_assert_pin(uint8_t pin) { (void)pin; }
#else
extern void roboclaw_estop_apply_pin(uint8_t pin);   /* drive LOW */
extern void roboclaw_estop_assert_pin(uint8_t pin);  /* drive HIGH */
#endif

static void apply_estop_pin(uint8_t unit_idx)
{
    if (unit_idx >= ROBOCLAW_MAX_UNITS) return;
    uint8_t pin = units[unit_idx].estop_pin;
    if (pin == 0 || pin > 29) return;
    roboclaw_estop_apply_pin(pin);
    saint_log_publish("info",
        "RoboClaw: unit %u E-stop pin GPIO %u driven LOW (deasserted) "
        "— S3 should now be releasable on the controller",
        (unsigned)unit_idx, (unsigned)pin);
}

static void assert_estop_pin(uint8_t unit_idx)
{
    if (unit_idx >= ROBOCLAW_MAX_UNITS) return;
    uint8_t pin = units[unit_idx].estop_pin;
    if (pin == 0 || pin > 29) return;
    roboclaw_estop_assert_pin(pin);
}

/* ── Connection state machine ───────────────────────────────────── */

static void mark_unit_response(uint8_t unit_idx, bool success)
{
    if (unit_idx >= ROBOCLAW_MAX_UNITS) return;
    roboclaw_unit_t* u = &units[unit_idx];

    if (success) {
        u->last_response_ms = PLATFORM_MILLIS();
        if (u->consecutive_misses >= ROBOCLAW_DROP_AFTER_MISSES) {
            saint_log_publish("info",
                "RoboClaw: unit %u (0x%02X) ACK received — recovered after %u misses",
                (unsigned)unit_idx, (unsigned)u->address,
                (unsigned)u->consecutive_misses);
        }
        u->consecutive_misses = 0;
        if (!u->connected) {
            u->connected = true;
            saint_log_publish("info",
                "RoboClaw: unit %u (0x%02X) connected",
                (unsigned)unit_idx, (unsigned)u->address);
        }
    } else {
        if (u->consecutive_misses < 255) u->consecutive_misses++;
        if (u->consecutive_misses == ROBOCLAW_DROP_AFTER_MISSES) {
            if (u->connected) {
                u->connected = false;
                saint_log_publish("warn",
                    "RoboClaw: unit %u (0x%02X) dropped — %u consecutive missed polls",
                    (unsigned)unit_idx, (unsigned)u->address,
                    (unsigned)u->consecutive_misses);
            } else {
                saint_log_publish("warn",
                    "RoboClaw: unit %u (0x%02X) — no ACK after %u attempts "
                    "(controller hasn't responded once since boot; check wiring, "
                    "baud, address, S3 latch)",
                    (unsigned)unit_idx, (unsigned)u->address,
                    (unsigned)u->consecutive_misses);
            }
        }
    }
}

/* ── Public API ─────────────────────────────────────────────────── */

void roboclaw_init(void)
{
    /* Seed the default address for any unit slot the operator hasn't
     * explicitly touched. Don't touch other fields — drv_apply_config /
     * drv_load populate those before us. */
    for (uint8_t i = 0; i < ROBOCLAW_MAX_UNITS; i++) {
        if (units[i].address == 0) {
            units[i].address = ROBOCLAW_ADDRESS_MIN + i;
        }
    }

    /* Resolve the operator-picked TX/RX into a UART instance. Silent
     * fallback was the regression that caused "configured pair was 4/5
     * but I'm running on 0/1" — log loudly and fall back. */
    uint8_t resolved_inst;
    uint8_t req_tx = roboclaw_tx_pin;
    uint8_t req_rx = roboclaw_rx_pin;
    if (uart_pin_pair_lookup(roboclaw_tx_pin, roboclaw_rx_pin, &resolved_inst)) {
        configured_serial_port = resolved_inst;
    } else {
        saint_log_publish("warn",
            "RoboClaw: requested pair TX=%u RX=%u isn't a valid RP2040 UART pair "
            "— falling back to TX=%u RX=%u (UART0). Check board YAML uart_pairs.",
            (unsigned)req_tx, (unsigned)req_rx,
            (unsigned)ROBOCLAW_DEFAULT_TX_PIN, (unsigned)ROBOCLAW_DEFAULT_RX_PIN);
        roboclaw_tx_pin = ROBOCLAW_DEFAULT_TX_PIN;
        roboclaw_rx_pin = ROBOCLAW_DEFAULT_RX_PIN;
        configured_serial_port = ROBOCLAW_DEFAULT_SERIAL_PORT;
    }

    /* HW UART vs PIO UART decision. All units share the bus, so
     * unit 0's uart_swap is authoritative (catalog schema documents
     * multi-unit setups must agree). Outside the transport call so
     * host tests can verify the decision without running PIO. */
    bool want_pio = (unit_count > 0) && (units[0].uart_swap != 0);
    use_pio_uart  = want_pio;

    const roboclaw_transport_ops_t* t = transport();
    if (!t) {
        /* Test / SIMULATION path: transport unavailable. Still set
         * port_initialized so the rest of the driver doesn't bail. */
        active_tx_pin = roboclaw_tx_pin;
        active_rx_pin = roboclaw_rx_pin;
        active_uart   = configured_serial_port;
        port_initialized = true;
        return;
    }

    /* Idempotent fast-path: same configuration already bound AND the
     * transport still verifies its binding. The verify_binding check
     * catches the rare case where another peripheral's init (or a
     * legacy pin_config_apply_hardware sweep) re-routed our pads to
     * UART/SIO AFTER we'd bound them — without this, a config sync
     * arriving later would find active_* matching and return early
     * while the PIO state machine pushes bytes into pads routed
     * somewhere else. */
    if (active_tx_pin == roboclaw_tx_pin
        && active_rx_pin == roboclaw_rx_pin
        && active_uart   == configured_serial_port
        && port_initialized
        && t->is_open()
        && t->verify_binding()
        && use_pio_uart == t->pio_swap_active()) {
        return;
    }

    if (!t->open(roboclaw_tx_pin, roboclaw_rx_pin,
                 configured_baud, want_pio)) {
        saint_log_publish("error",
            "RoboClaw: %s transport open failed (tx=%u rx=%u baud=%u pio=%d)",
            t->name,
            (unsigned)roboclaw_tx_pin, (unsigned)roboclaw_rx_pin,
            (unsigned)configured_baud, (int)want_pio);
        return;
    }
    use_pio_uart = t->pio_swap_active();
    configured_serial_port = t->resolved_instance();

    active_tx_pin = roboclaw_tx_pin;
    active_rx_pin = roboclaw_rx_pin;
    active_uart   = configured_serial_port;

    /* Bus stabilization delay before the probe sweep. */
    PLATFORM_SLEEP_MS(100);

    if (use_pio_uart) {
        saint_log_publish("info",
            "RoboClaw: bound PIO UART (TX=GP%u, RX=GP%u, swapped from hw map) "
            "@ %u baud — probing 0x%02X..0x%02X",
            (unsigned)roboclaw_rx_pin, (unsigned)roboclaw_tx_pin,
            (unsigned)configured_baud,
            (unsigned)ROBOCLAW_ADDRESS_MIN,
            (unsigned)(ROBOCLAW_ADDRESS_MIN + ROBOCLAW_MAX_UNITS - 1));
    } else {
        saint_log_publish("info",
            "RoboClaw: bound UART%u TX=%u RX=%u @ %u baud — "
            "probing 0x%02X..0x%02X",
            (unsigned)configured_serial_port,
            (unsigned)roboclaw_tx_pin, (unsigned)roboclaw_rx_pin,
            (unsigned)configured_baud,
            (unsigned)ROBOCLAW_ADDRESS_MIN,
            (unsigned)(ROBOCLAW_ADDRESS_MIN + ROBOCLAW_MAX_UNITS - 1));
    }

    uint8_t connected_count = 0;
    for (uint8_t i = 0; i < ROBOCLAW_MAX_UNITS; i++) {
        units[i].consecutive_misses = 0;
        units[i].last_reprobe_ms    = PLATFORM_MILLIS();
        char version[ROBOCLAW_VERSION_MAX_LEN] = {0};
        if (probe_unit(i, version, sizeof(version))) {
            mark_unit_response(i, true);
            saint_log_publish("info",
                "RoboClaw: unit %u (0x%02X) version: %s",
                (unsigned)i, (unsigned)units[i].address, version);
            if (i >= unit_count) unit_count = i + 1;
            connected_count++;
        }
    }
    saint_log_publish(connected_count > 0 ? "info" : "warn",
        "RoboClaw: probe complete — %u of %u addresses responded",
        (unsigned)connected_count, (unsigned)ROBOCLAW_MAX_UNITS);

    port_initialized = true;
}

static bool maybe_send_duty_keepalive(void);

/* ── Fault status (Read Status, cmd 90) ─────────────────────────── */

/* Raw 16-bit status word bits (older RoboClaw firmware). */
#define RC16_M1_OVERCURRENT    0x0001
#define RC16_M2_OVERCURRENT    0x0002
#define RC16_ESTOP             0x0004
#define RC16_TEMP_ERROR        0x0008
#define RC16_TEMP2_ERROR       0x0010
#define RC16_MAIN_BATT_HIGH    0x0020
#define RC16_LOGIC_BATT_HIGH   0x0040
#define RC16_LOGIC_BATT_LOW    0x0080
#define RC16_M1_DRIVER_FAULT   0x0100
#define RC16_M2_DRIVER_FAULT   0x0200
#define RC16_MAIN_BATT_HI_WARN 0x0400
#define RC16_MAIN_BATT_LO_WARN 0x0800
#define RC16_TEMP_WARN         0x1000
#define RC16_TEMP2_WARN        0x2000
/* 0x4000/0x8000 = M1/M2 Home — not faults. */

static uint16_t normalize_status_16(uint16_t s)
{
    uint16_t f = 0;
    if (s & RC16_ESTOP)                                    f |= ROBOCLAW_FAULT_ESTOP;
    if (s & (RC16_M1_OVERCURRENT | RC16_M2_OVERCURRENT))   f |= ROBOCLAW_FAULT_OVERCURRENT;
    if (s & (RC16_TEMP_ERROR | RC16_TEMP2_ERROR))          f |= ROBOCLAW_FAULT_TEMP_ERROR;
    if (s & (RC16_TEMP_WARN | RC16_TEMP2_WARN))            f |= ROBOCLAW_FAULT_TEMP_WARN;
    if (s & (RC16_MAIN_BATT_HIGH | RC16_MAIN_BATT_HI_WARN)) f |= ROBOCLAW_FAULT_MAIN_BATT_HI;
    if (s & RC16_MAIN_BATT_LO_WARN)                        f |= ROBOCLAW_FAULT_MAIN_BATT_LO;
    if (s & (RC16_LOGIC_BATT_HIGH | RC16_LOGIC_BATT_LOW))  f |= ROBOCLAW_FAULT_LOGIC_BATT;
    if (s & (RC16_M1_DRIVER_FAULT | RC16_M2_DRIVER_FAULT)) f |= ROBOCLAW_FAULT_DRIVER_FAULT;
    return f;
}

/* Raw 32-bit status word bits (newer RoboClaw firmware, 4.1.34+). */
#define RC32_ESTOP             0x00000001UL
#define RC32_TEMP_ERROR        0x00000002UL
#define RC32_TEMP2_ERROR       0x00000004UL
#define RC32_MAIN_VOLT_HIGH    0x00000008UL
#define RC32_LOGIC_VOLT_HIGH   0x00000010UL
#define RC32_LOGIC_VOLT_LOW    0x00000020UL
#define RC32_M1_DRIVER_FAULT   0x00000040UL
#define RC32_M2_DRIVER_FAULT   0x00000080UL
#define RC32_M1_SPEED_ERROR    0x00000100UL
#define RC32_M2_SPEED_ERROR    0x00000200UL
#define RC32_M1_POS_ERROR      0x00000400UL
#define RC32_M2_POS_ERROR      0x00000800UL
#define RC32_M1_CURRENT_ERROR  0x00001000UL
#define RC32_M2_CURRENT_ERROR  0x00002000UL
#define RC32_M1_OC_WARN        0x00010000UL
#define RC32_M2_OC_WARN        0x00020000UL
#define RC32_MAIN_VOLT_HI_WARN 0x00040000UL
#define RC32_MAIN_VOLT_LO_WARN 0x00080000UL
#define RC32_TEMP_WARN         0x00100000UL
#define RC32_TEMP2_WARN        0x00200000UL

static uint16_t normalize_status_32(uint32_t s)
{
    uint16_t f = 0;
    if (s & RC32_ESTOP)                                     f |= ROBOCLAW_FAULT_ESTOP;
    if (s & (RC32_M1_CURRENT_ERROR | RC32_M2_CURRENT_ERROR |
             RC32_M1_OC_WARN | RC32_M2_OC_WARN))            f |= ROBOCLAW_FAULT_OVERCURRENT;
    if (s & (RC32_TEMP_ERROR | RC32_TEMP2_ERROR))           f |= ROBOCLAW_FAULT_TEMP_ERROR;
    if (s & (RC32_TEMP_WARN | RC32_TEMP2_WARN))             f |= ROBOCLAW_FAULT_TEMP_WARN;
    if (s & (RC32_MAIN_VOLT_HIGH | RC32_MAIN_VOLT_HI_WARN)) f |= ROBOCLAW_FAULT_MAIN_BATT_HI;
    if (s & RC32_MAIN_VOLT_LO_WARN)                         f |= ROBOCLAW_FAULT_MAIN_BATT_LO;
    if (s & (RC32_LOGIC_VOLT_HIGH | RC32_LOGIC_VOLT_LOW))   f |= ROBOCLAW_FAULT_LOGIC_BATT;
    if (s & (RC32_M1_DRIVER_FAULT | RC32_M2_DRIVER_FAULT))  f |= ROBOCLAW_FAULT_DRIVER_FAULT;
    if (s & (RC32_M1_SPEED_ERROR | RC32_M2_SPEED_ERROR))    f |= ROBOCLAW_FAULT_SPEED_ERROR;
    if (s & (RC32_M1_POS_ERROR | RC32_M2_POS_ERROR))        f |= ROBOCLAW_FAULT_POSITION_ERR;
    return f;
}

/* Render active canonical fault names into buf, comma-separated. */
static void fault_names(uint16_t f, char* buf, size_t cap)
{
    static const struct { uint16_t bit; const char* name; } NAMES[] = {
        { ROBOCLAW_FAULT_ESTOP,        "E-Stop" },
        { ROBOCLAW_FAULT_OVERCURRENT,  "over-current" },
        { ROBOCLAW_FAULT_TEMP_ERROR,   "over-temperature" },
        { ROBOCLAW_FAULT_TEMP_WARN,    "temperature warning" },
        { ROBOCLAW_FAULT_MAIN_BATT_HI, "main battery high" },
        { ROBOCLAW_FAULT_MAIN_BATT_LO, "main battery low" },
        { ROBOCLAW_FAULT_LOGIC_BATT,   "logic battery out of range" },
        { ROBOCLAW_FAULT_DRIVER_FAULT, "driver fault" },
        { ROBOCLAW_FAULT_SPEED_ERROR,  "speed error limit" },
        { ROBOCLAW_FAULT_POSITION_ERR, "position error limit" },
    };
    size_t n = 0; bool first = true;
    buf[0] = '\0';
    for (size_t i = 0; i < sizeof(NAMES)/sizeof(NAMES[0]); i++) {
        if (!(f & NAMES[i].bit)) continue;
        int w = snprintf(buf + n, cap - n, "%s%s", first ? "" : ", ", NAMES[i].name);
        if (w < 0 || (size_t)w >= cap - n) break;
        n += (size_t)w; first = false;
    }
    if (first) snprintf(buf, cap, "none");
}

/* Edge-triggered: log only when the canonical fault set changes, so a
 * standing fault doesn't spam the log every poll. */
static void log_fault_transition(uint8_t u)
{
    uint16_t now = units[u].fault_flags;
    if (now == units[u].prev_fault_flags) return;
    units[u].prev_fault_flags = now;
    const char* id = units[u].peripheral_id[0] ? units[u].peripheral_id : "roboclaw";
    if (now == ROBOCLAW_FAULT_NONE) {
        saint_log_publish("info", "RoboClaw '%s' (unit %u): faults cleared",
                          id, (unsigned)u);
        return;
    }
    char buf[128];
    fault_names(now, buf, sizeof(buf));
    saint_log_publish((now & ROBOCLAW_FAULT_ERROR_MASK) ? "error" : "warn",
        "RoboClaw '%s' (unit %u) FAULT: %s (0x%04X)",
        id, (unsigned)u, buf, (unsigned)now);
}

/* Poll the status register, auto-detecting 16- vs 32-bit response length.
 * Drains stray RX around the exchange so a wrong-length probe can't
 * corrupt the next telemetry read. Best-effort: a failed read leaves the
 * last-known fault_flags untouched and does NOT count as a missed poll
 * (connection tracking stays on the four data reads). */
static bool read_status(uint8_t u)
{
    uint8_t addr = units[u].address;
    uint8_t resp[8];
    uint8_t lens[2]; uint8_t nlens;
    if (units[u].status_len == 2)      { lens[0] = 2; nlens = 1; }
    else if (units[u].status_len == 4) { lens[0] = 4; nlens = 1; }
    else                               { lens[0] = 4; lens[1] = 2; nlens = 2; }

    for (uint8_t k = 0; k < nlens; k++) {
        while (wire_has_byte()) (void)wire_getc();          /* drain before send */
        send_command(addr, ROBOCLAW_CMD_GETSTATUS, NULL, 0);
        if (read_response(resp, lens[k], addr, ROBOCLAW_CMD_GETSTATUS)) {
            units[u].status_len = lens[k];
            if (lens[k] == 4) {
                uint32_t raw = ((uint32_t)resp[0] << 24) | ((uint32_t)resp[1] << 16) |
                               ((uint32_t)resp[2] << 8) | resp[3];
                units[u].fault_flags = normalize_status_32(raw);
            } else {
                uint16_t raw = ((uint16_t)resp[0] << 8) | resp[1];
                units[u].fault_flags = normalize_status_16(raw);
            }
            units[u].status_valid = true;
            log_fault_transition(u);
            return true;
        }
        while (wire_has_byte()) (void)wire_getc();          /* drain leftovers */
    }
    return false;
}

void roboclaw_update(void)
{
    if (!port_initialized || unit_count == 0) return;

    temp_probe_diagnostic(PLATFORM_MILLIS());
    wire_stats_maybe_dump(PLATFORM_MILLIS());

    if (maybe_send_duty_keepalive()) return;

    uint8_t u = poll_unit;
    uint8_t addr = units[u].address;
    uint8_t resp[8];
    bool success = false;

    if (!units[u].connected) {
        uint32_t now = PLATFORM_MILLIS();
        if (now - units[u].last_reprobe_ms >= ROBOCLAW_REPROBE_INTERVAL_MS) {
            units[u].last_reprobe_ms = now;
            char version[ROBOCLAW_VERSION_MAX_LEN] = {0};
            if (probe_unit(u, version, sizeof(version))) {
                mark_unit_response(u, true);
                saint_log_publish("info",
                    "RoboClaw: unit %u (0x%02X) recovered — version: %s",
                    (unsigned)u, (unsigned)addr, version);
            }
        }
        goto next_poll;
    }

    switch (poll_register) {
    case 0:
        send_command(addr, ROBOCLAW_CMD_GETM1ENC, NULL, 0);
        if (read_response(resp, 5, addr, ROBOCLAW_CMD_GETM1ENC)) {
            units[u].encoder = ((int32_t)resp[0] << 24) |
                               ((int32_t)resp[1] << 16) |
                               ((int32_t)resp[2] << 8) |
                               resp[3];
            success = true;
        }
        break;

    case 1:
        send_command(addr, ROBOCLAW_CMD_GETMBATT, NULL, 0);
        if (read_response(resp, 2, addr, ROBOCLAW_CMD_GETMBATT)) {
            uint16_t raw = ((uint16_t)resp[0] << 8) | resp[1];
            units[u].voltage_mv = raw * 100;
            success = true;
        }
        break;

    case 2:
        send_command(addr, ROBOCLAW_CMD_GETCURRENTS, NULL, 0);
        if (read_response(resp, 4, addr, ROBOCLAW_CMD_GETCURRENTS)) {
            int16_t raw = (int16_t)(((uint16_t)resp[0] << 8) | resp[1]);
            units[u].current_ma = raw * 10;
            success = true;
        }
        break;

    case 3:
        send_command(addr, ROBOCLAW_CMD_GETTEMP, NULL, 0);
        if (read_response(resp, 2, addr, ROBOCLAW_CMD_GETTEMP)) {
            units[u].temp_tenths = (int16_t)(((uint16_t)resp[0] << 8) | resp[1]);
            success = true;
        }
        break;

    case 4:
        /* Fault/status register — best-effort, does not gate connection
         * (a controller can be perfectly healthy on an older firmware we
         * can't yet parse). Skips mark_unit_response below. */
        read_status(u);
        break;
    }
    if (poll_register != 4) {
        mark_unit_response(u, success);
    }

next_poll:
    poll_register++;
    if (poll_register > 4) {
        poll_register = 0;
        poll_unit++;
        if (poll_unit >= unit_count) {
            poll_unit = 0;
        }
    }
}

bool roboclaw_is_connected(void)
{
    if (!port_initialized) return false;
    for (uint8_t i = 0; i < unit_count; i++) {
        if (units[i].connected) return true;
    }
    return false;
}

static inline int16_t apply_direction(uint8_t unit, int16_t duty)
{
    if (unit >= ROBOCLAW_MAX_UNITS) return duty;
    return units[unit].invert_direction ? (int16_t)-duty : duty;
}

bool roboclaw_set_duty(uint8_t unit, int16_t duty)
{
    if (unit >= ROBOCLAW_MAX_UNITS) return false;
    if (!port_initialized) {
        saint_log_publish("warn",
            "RoboClaw: set_duty(unit=%u, duty=%d) ignored — driver not "
            "initialized (no peripheral synced; UART not bound).",
            (unsigned)unit, (int)duty);
        return false;
    }

    if (duty > ROBOCLAW_DUTY_MAX) duty = ROBOCLAW_DUTY_MAX;
    if (duty < ROBOCLAW_DUTY_MIN) duty = ROBOCLAW_DUTY_MIN;

    units[unit].duty = duty;

#ifndef SIMULATION
    if (!wire_ready()) {
        saint_log_publish("warn",
            "RoboClaw: set_duty(unit=%u, duty=%d) — transport not bound.",
            (unsigned)unit, (int)duty);
        return false;
    }
    int16_t wire_duty = apply_direction(unit, duty);
    uint8_t data[2];
    data[0] = (uint8_t)((uint16_t)wire_duty >> 8);
    data[1] = (uint8_t)((uint16_t)wire_duty & 0xFF);

    if (!units[unit].first_duty_logged) {
        units[unit].first_duty_logged = true;
        saint_log_publish("info",
            "RoboClaw: unit %u (0x%02X) — dispatching first M1DUTY "
            "packet [0x%02X, 32, 0x%02X, 0x%02X, crc16] via %s @ %u baud, "
            "duty=%d (%.1f%%). Watch for ACK / connected log next.",
            (unsigned)unit, (unsigned)units[unit].address,
            (unsigned)units[unit].address,
            (unsigned)data[0], (unsigned)data[1],
            use_pio_uart ? "PIO UART (TX/RX swapped)" : "HW UART",
            (unsigned)configured_baud,
            (int)duty, (double)duty * 100.0 / (double)ROBOCLAW_DUTY_MAX);
    }

    send_command(units[unit].address, ROBOCLAW_CMD_M1DUTY, data, 2);
    bool ack_ok = read_ack();
    mark_unit_response(unit, ack_ok);
    units[unit].last_duty_send_ms = PLATFORM_MILLIS();
#endif
    return true;
}

/* Re-sends M1DUTY for any unit with duty != 0 every ROBOCLAW_DUTY_
 * KEEPALIVE_MS. The RoboClaw's serial-timeout watchdog feeds on
 * INCOMING bytes — even an unACKed write keeps the motor alive. So
 * we do NOT gate on units[i].connected: ACK reliability is just an
 * observation, but the motor staying alive is safety-critical. */
static bool maybe_send_duty_keepalive(void)
{
    if (!port_initialized || unit_count == 0) return false;

    uint32_t now = PLATFORM_MILLIS();
    for (uint8_t i = 0; i < unit_count; i++) {
        roboclaw_unit_t* u = &units[i];
        if (u->duty == 0) continue;
        uint32_t since_last = now - u->last_duty_send_ms;
        if (since_last < ROBOCLAW_DUTY_KEEPALIVE_MS) continue;

#ifndef SIMULATION
        if (wire_ready()) {
            int16_t wire_duty = apply_direction(i, u->duty);
            uint8_t data[2];
            data[0] = (uint8_t)((uint16_t)wire_duty >> 8);
            data[1] = (uint8_t)((uint16_t)wire_duty & 0xFF);
            send_command(u->address, ROBOCLAW_CMD_M1DUTY, data, 2);
            bool ack_ok = read_ack();
            mark_unit_response(i, ack_ok);
        }
#endif
        u->last_duty_send_ms = now;
        return true;
    }
    return false;
}

bool roboclaw_stop(uint8_t unit)
{
    return roboclaw_set_duty(unit, 0);
}

void roboclaw_stop_all(void)
{
    for (uint8_t i = 0; i < ROBOCLAW_MAX_UNITS; i++) {
        roboclaw_set_duty(i, 0);
    }
}

/* ── peripheral_driver_t glue ───────────────────────────────────── */

static bool roboclaw_drv_init(void)
{
    /* No-op (README item 3 pattern). Real init runs from drv_load
     * (flash had unit_count > 0) or drv_apply_config. */
    return true;
}

static bool roboclaw_drv_set_value(uint8_t channel, float value)
{
    uint8_t unit = channel / ROBOCLAW_CHANNELS_PER_UNIT;
    uint8_t sub = channel % ROBOCLAW_CHANNELS_PER_UNIT;

    if (sub != ROBOCLAW_SUB_MOTOR) return false;

    if (value < -1.0f) value = -1.0f;
    if (value > 1.0f) value = 1.0f;
    int16_t duty = (int16_t)(value * ROBOCLAW_DUTY_MAX);
    return roboclaw_set_duty(unit, duty);
}

static bool roboclaw_drv_get_value(uint8_t channel, float* value)
{
    uint8_t unit = channel / ROBOCLAW_CHANNELS_PER_UNIT;
    uint8_t sub = channel % ROBOCLAW_CHANNELS_PER_UNIT;

    if (unit >= ROBOCLAW_MAX_UNITS) return false;

    switch (sub) {
    case ROBOCLAW_SUB_MOTOR:
        *value = (float)units[unit].duty / (float)ROBOCLAW_DUTY_MAX;
        return true;
    case ROBOCLAW_SUB_ENCODER:
        *value = (float)units[unit].encoder;
        return true;
    case ROBOCLAW_SUB_VOLTAGE:
        *value = (float)units[unit].voltage_mv / 1000.0f;
        return true;
    case ROBOCLAW_SUB_CURRENT:
        *value = (float)units[unit].current_ma / 1000.0f;
        return true;
    case ROBOCLAW_SUB_TEMP:
        *value = (float)units[unit].temp_tenths / 10.0f;
        return true;
    default:
        return false;
    }
}

static void roboclaw_drv_set_defaults(uint8_t channel, pin_config_t* config)
{
    uint8_t unit = channel / ROBOCLAW_CHANNELS_PER_UNIT;
    config->params.roboclaw.address = ROBOCLAW_ADDRESS_MIN + unit;
    config->params.roboclaw.deadband = 0;
    config->params.roboclaw.max_current_ma = 0;
}

static bool roboclaw_drv_apply_config(uint8_t channel, const pin_config_t* config)
{
    uint8_t unit = channel / ROBOCLAW_CHANNELS_PER_UNIT;
    if (unit >= ROBOCLAW_MAX_UNITS) return false;

    /* Remember the operator's instance id so the peripheral-first
     * state_emit path can label this unit's "connected"/"error_flags"
     * records with (peripheral_id, channel_id) — no virtual GPIO. */
    if (config->logical_name[0]) {
        strncpy(units[unit].peripheral_id, config->logical_name,
                sizeof(units[unit].peripheral_id) - 1);
        units[unit].peripheral_id[sizeof(units[unit].peripheral_id) - 1] = '\0';
    }

    /* Address == 0 is the boot-reload fingerprint (apply_hardware sweep
     * with zero params). If we see it AND units[] already has a
     * populated address, trust units[] (drv_load filled it in) and
     * skip the field copy — otherwise we'd clobber uart_swap/estop_pin
     * that drv_load just restored. */
    bool cfg_has_real_params = (config->params.roboclaw.address != 0);
    if (cfg_has_real_params) {
        units[unit].address = config->params.roboclaw.address;
        units[unit].deadband = config->params.roboclaw.deadband;
        units[unit].max_current_ma = config->params.roboclaw.max_current_ma;
        units[unit].estop_pin = config->params.roboclaw.estop_pin;
        units[unit].uart_swap = config->params.roboclaw.uart_swap;
        units[unit].invert_direction = config->params.roboclaw.invert_direction;
    }

    if (unit >= unit_count) {
        unit_count = unit + 1;
    }

    /* Drive E-stop pin LOW BEFORE the probe sweep — the controller
     * may still be latched from boot-time floating S3. */
    apply_estop_pin(unit);

    roboclaw_init();
    return true;
}

static bool roboclaw_drv_parse_json(const char* json_start, const char* json_end,
                                     pin_config_t* config)
{
    const char* p;
    bool got_pins = false;

    p = strstr(json_start, "\"address\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) { p++; while (*p == ' ') p++; config->params.roboclaw.address = (uint8_t)atoi(p); }
    }

    p = strstr(json_start, "\"deadband\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) { p++; while (*p == ' ') p++; config->params.roboclaw.deadband = (uint8_t)atoi(p); }
    }

    p = strstr(json_start, "\"max_current_ma\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) { p++; while (*p == ' ') p++; config->params.roboclaw.max_current_ma = (uint16_t)atoi(p); }
    }

    p = strstr(json_start, "\"estop_pin\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) {
            p++;
            while (*p == ' ') p++;
            int v = atoi(p);
            if (v >= 1 && v <= 29) {
                config->params.roboclaw.estop_pin = (uint8_t)v;
            } else {
                config->params.roboclaw.estop_pin = 0;
            }
        }
    }

    p = strstr(json_start, "\"uart_swap\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) {
            p++;
            while (*p == ' ') p++;
            config->params.roboclaw.uart_swap =
                (strncmp(p, "true", 4) == 0) ? 1 : 0;
        }
    }

    p = strstr(json_start, "\"invert_direction\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) {
            p++;
            while (*p == ' ') p++;
            config->params.roboclaw.invert_direction =
                (strncmp(p, "true", 4) == 0) ? 1 : 0;
        }
    }

    uint8_t tx, rx, inst;
    if (uart_pin_pair_parse_json(json_start, json_end, &tx, &rx, &inst)) {
        roboclaw_tx_pin = tx;
        roboclaw_rx_pin = rx;
        configured_serial_port = inst;
        got_pins = true;
    }

    if (config->gpio == ROBOCLAW_VIRTUAL_GPIO_BASE) {
        char estop_buf[16];
        if (config->params.roboclaw.estop_pin >= 1
            && config->params.roboclaw.estop_pin <= 29) {
            snprintf(estop_buf, sizeof(estop_buf), "GPIO %u",
                     (unsigned)config->params.roboclaw.estop_pin);
        } else {
            snprintf(estop_buf, sizeof(estop_buf), "none");
        }
        if (got_pins) {
            saint_log_publish("info",
                "RoboClaw sync: pins TX=%u RX=%u (UART%u), unit 0 addr=0x%02X "
                "deadband=%u max_current=%u mA, E-stop pin=%s",
                (unsigned)roboclaw_tx_pin, (unsigned)roboclaw_rx_pin,
                (unsigned)configured_serial_port,
                (unsigned)config->params.roboclaw.address,
                (unsigned)config->params.roboclaw.deadband,
                (unsigned)config->params.roboclaw.max_current_ma,
                estop_buf);
        } else {
            saint_log_publish("warn",
                "RoboClaw sync: didn't find uart_tx/uart_rx in JSON — "
                "driver will keep using TX=%u RX=%u (E-stop pin=%s)",
                (unsigned)roboclaw_tx_pin, (unsigned)roboclaw_rx_pin,
                estop_buf);
        }
    }

    return true;
}

static void roboclaw_drv_estop(void)
{
    for (uint8_t i = 0; i < unit_count; i++) {
        assert_estop_pin(i);
    }
    roboclaw_stop_all();
    saint_log_publish("warn",
        "RoboClaw: ESTOP — estop_pin asserted HIGH on %u unit(s), "
        "all duties commanded to 0",
        (unsigned)unit_count);
}

static void roboclaw_drv_clear_estop(void)
{
    for (uint8_t i = 0; i < unit_count; i++) {
        apply_estop_pin(i);
    }
    saint_log_publish("info",
        "RoboClaw: ESTOP RELEASED — estop_pin deasserted on %u unit(s). "
        "Motor duty remains at 0 until the next command.",
        (unsigned)unit_count);
}

static bool roboclaw_drv_save(void* storage_ptr)
{
    flash_storage_data_t* storage = (flash_storage_data_t*)storage_ptr;

    memset(&storage->roboclaw_config, 0, sizeof(storage->roboclaw_config));
    storage->roboclaw_config.unit_count = unit_count;
    storage->roboclaw_config.serial_port = configured_serial_port;
    storage->roboclaw_config.baud_rate = configured_baud;

    storage->uart_pins.roboclaw_tx_pin = roboclaw_tx_pin;
    storage->uart_pins.roboclaw_rx_pin = roboclaw_rx_pin;

    for (uint8_t i = 0; i < ROBOCLAW_MAX_UNITS; i++) {
        storage->roboclaw_config.units[i].address = units[i].address;
        storage->roboclaw_config.units[i].deadband = units[i].deadband;
        storage->roboclaw_config.units[i].max_current_ma = units[i].max_current_ma;
        storage->roboclaw_config.units[i].estop_pin = units[i].estop_pin;
        /* Pack uart_swap (bit 0) + invert_direction (bit 1) into the
         * single uart_swap flash byte to avoid a schema-version bump.
         * v9 saves had bit 1 == 0 — those unpack as invert_direction=0,
         * preserving prior behavior. */
        storage->roboclaw_config.units[i].uart_swap =
            (uint8_t)((units[i].uart_swap         & 0x01)        |
                      ((units[i].invert_direction & 0x01) << 1));
    }

    return true;
}

static bool roboclaw_drv_load(const void* storage_ptr)
{
    const flash_storage_data_t* storage = (const flash_storage_data_t*)storage_ptr;

    uint8_t stored_tx = storage->uart_pins.roboclaw_tx_pin;
    uint8_t stored_rx = storage->uart_pins.roboclaw_rx_pin;
    if (stored_tx != 0 || stored_rx != 0) {
        uint8_t inst;
        if (uart_pin_pair_lookup(stored_tx, stored_rx, &inst)) {
            roboclaw_tx_pin = stored_tx;
            roboclaw_rx_pin = stored_rx;
            configured_serial_port = inst;
        } else {
            saint_log_publish("warn",
                "RoboClaw: stored pin pair TX=%u RX=%u isn't a valid RP2040 "
                "UART pair — using defaults. Re-sync from the dashboard.",
                (unsigned)stored_tx, (unsigned)stored_rx);
        }
    }

    if (storage->roboclaw_config.unit_count == 0) {
        saint_log_publish("info",
            "RoboClaw: no saved peripheral in flash — driver dormant "
            "(no UART traffic). Sync a RoboClaw config from the dashboard "
            "to bind the UART and start polling.");
        return true;
    }

    uint8_t count = storage->roboclaw_config.unit_count;
    if (count > ROBOCLAW_MAX_UNITS) count = ROBOCLAW_MAX_UNITS;
    unit_count = count;

    if (storage->roboclaw_config.baud_rate > 0) {
        configured_baud = storage->roboclaw_config.baud_rate;
    }
    configured_serial_port = storage->roboclaw_config.serial_port;

    for (uint8_t i = 0; i < count; i++) {
        units[i].address = storage->roboclaw_config.units[i].address;
        units[i].deadband = storage->roboclaw_config.units[i].deadband;
        units[i].max_current_ma = storage->roboclaw_config.units[i].max_current_ma;
        units[i].estop_pin = storage->roboclaw_config.units[i].estop_pin;
        {
            uint8_t flags = storage->roboclaw_config.units[i].uart_swap;
            units[i].uart_swap        = (uint8_t)(flags        & 0x01);
            units[i].invert_direction = (uint8_t)((flags >> 1) & 0x01);
        }
        apply_estop_pin(i);
    }

    saint_log_publish("info",
        "RoboClaw: restored %u unit configs from flash (UART%u TX=%u RX=%u @ %u baud)",
        (unsigned)count, (unsigned)configured_serial_port,
        (unsigned)roboclaw_tx_pin, (unsigned)roboclaw_rx_pin,
        (unsigned)configured_baud);

    roboclaw_init();
    return true;
}

/* Peripheral-first telemetry: emit each configured unit's connection +
 * fault status keyed by (peripheral_id, channel_id) — this is the same
 * path the Maestro uses and needs no virtual GPIO. The motor/encoder/
 * voltage/current/temp readings still flow through the runtime-value
 * path; this adds the status the UI decodes into fault badges. */
static int roboclaw_state_emit_channels(char* buf, size_t cap, bool* first)
{
    int total = 0;
    for (uint8_t u = 0; u < unit_count && u < ROBOCLAW_MAX_UNITS; u++) {
        if (!units[u].peripheral_id[0]) continue;   /* not configured */
        int n = peripheral_state_append_channel(
            buf + total, cap - (size_t)total, first,
            units[u].peripheral_id, "connected",
            units[u].connected ? 1.0f : 0.0f);
        if (n < 0) return -1;
        total += n;
        n = peripheral_state_append_channel(
            buf + total, cap - (size_t)total, first,
            units[u].peripheral_id, "error_flags",
            units[u].status_valid ? (float)units[u].fault_flags : 0.0f);
        if (n < 0) return -1;
        total += n;
    }
    return total;
}

static const peripheral_driver_t roboclaw_peripheral = {
    .name              = "roboclaw",
    .mode_string       = "roboclaw_motor",
    .pin_mode          = PIN_MODE_ROBOCLAW_MOTOR,
    .capability_flag   = PIN_CAP_ROBOCLAW_MOTOR,
    .virtual_gpio_base = ROBOCLAW_VIRTUAL_GPIO_BASE,
    .channel_count          = ROBOCLAW_MAX_CHANNELS,
    .channels_per_instance  = ROBOCLAW_CHANNELS_PER_UNIT,
    .init              = roboclaw_drv_init,
    .update            = roboclaw_update,
    .is_connected      = roboclaw_is_connected,
    .state_emit_channels = roboclaw_state_emit_channels,
    .set_value         = roboclaw_drv_set_value,
    .get_value         = roboclaw_drv_get_value,
    .set_defaults      = roboclaw_drv_set_defaults,
    .apply_config      = roboclaw_drv_apply_config,
    .parse_json_params = roboclaw_drv_parse_json,
    .estop             = roboclaw_drv_estop,
    .clear_estop       = roboclaw_drv_clear_estop,
    .save_config       = roboclaw_drv_save,
    .load_config       = roboclaw_drv_load,
};

const peripheral_driver_t* roboclaw_get_peripheral_driver(void)
{
    return &roboclaw_peripheral;
}
