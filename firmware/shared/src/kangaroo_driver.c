/**
 * SAINT.OS Firmware - Dimension Engineering Kangaroo X2 driver (shared core)
 *
 * Closed-loop motion over packet OR simplified serial (per-channel,
 * operator-selected). Up to 8 Kangaroo motor channels share one UART;
 * each is addressed by (board address 128..135, channel name char).
 * Six sub-channels per unit:
 *
 *   0  target_position    write, [-1.0, 1.0] -> ±max_position (units)
 *   1  target_speed       write, [-1.0, 1.0] -> ±max_speed (units/s)
 *   2  current_position   read,  units
 *   3  current_speed      read,  units/s
 *   4  moving             read,  1 = motion pending (Kangaroo "busy")
 *   5  error_status       read,  last Kangaroo error code (0 = OK)
 *
 * Each channel MUST be sent Start after every power-up before it honors
 * motion commands. We Start (and optionally Home) on config / on
 * (re)connect, and auto-resend Start if a reply reports error 1
 * ("not started" — the Kangaroo lost power and came back).
 *
 * Unlike the SyRen (write-only) this driver reads replies, so it shares
 * the Tic's read-capable transport + look-ahead-buffer pattern. Wire
 * framing lives in shared/include/kangaroo_protocol.h (a verbatim port
 * of DE's Arduino/C# library); UART I/O dispatches through the
 * per-platform transport ops.
 */

#include "kangaroo_driver.h"
#include "kangaroo_transport.h"
#include "kangaroo_protocol.h"

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

#define KANGAROO_DEFAULT_TX_PIN      0
#define KANGAROO_DEFAULT_RX_PIN      1
#define KANGAROO_DEFAULT_SERIAL_PORT 0

#define KANGAROO_DROP_AFTER_MISSES   3
#define KANGAROO_REPROBE_INTERVAL_MS 2000
#define KANGAROO_DEFAULT_MAX_POSITION 10000
#define KANGAROO_DEFAULT_MAX_SPEED    1000
#define KANGAROO_DEFAULT_CHANNEL_NAME '1'

/* Reply read budgets (mirror the Tic's). */
#define KANGAROO_RESPONSE_TIMEOUT_MS 50
#define KANGAROO_BYTE_TIMEOUT_MS     10

/* ── Teach-tune timings ─────────────────────────────────────────── */

/* Tuning has an automatic serial timeout and aborts if packets stop
 * (Packet Serial Reference p.14: "You must continually send packets or
 * it will abort. Get commands in a loop will do the job."). We hold a
 * Get loop at this interval for the whole tune — comfortably inside any
 * plausible timeout without saturating a 9600 baud link. */
#define KANGAROO_TUNE_KEEPALIVE_MS   200

/* Dead-man. Open-loop jog has no feedback and no travel limits, so a
 * jog that stops being refreshed must decay to zero rather than stay
 * latched — otherwise a dropped link pins the actuator against a hard
 * stop. Must exceed the UI's jog repeat interval with margin. */
#define KANGAROO_JOG_DEADMAN_MS      250

/* Absolute ceiling on a tune cycle. A slow linear actuator can take
 * minutes; past this something is wrong and we abort rather than leave
 * the axis under the Kangaroo's control indefinitely. */
#define KANGAROO_TUNE_MAX_MS         600000

/* The tune is complete when the axis has reported not-busy continuously
 * for this long, no earlier than KANGAROO_TUNE_MIN_MS after Go (the
 * cycle does not start instantly, so an early not-busy is meaningless). */
#define KANGAROO_TUNE_QUIET_MS       3000
#define KANGAROO_TUNE_MIN_MS         5000

/* Default open-loop jog cap, as a percentage of full scale. Deliberately
 * low: the first jog on an untuned axis is the single most dangerous
 * moment in this whole procedure, and direction is unknown until tried. */
#define KANGAROO_DEFAULT_JOG_PCT     10

/* Get-reply error codes (KANGAROO_ERR_*) now live in kangaroo_protocol.h
 * alongside the rest of the wire constants — the full set, not just the
 * two this file used to declare locally. Do not redeclare them here. */

static uint8_t kangaroo_tx_pin = KANGAROO_DEFAULT_TX_PIN;
static uint8_t kangaroo_rx_pin = KANGAROO_DEFAULT_RX_PIN;

static uint8_t active_tx_pin = 0xFF;
static uint8_t active_rx_pin = 0xFF;
static uint8_t active_uart   = 0xFF;

/* ── Per-Unit State ─────────────────────────────────────────────── */

typedef struct {
    uint8_t  address;            /* 128-135 */
    uint8_t  channel_name;       /* '1'/'2'/'D'/'T' */
    uint8_t  protocol;           /* KANGAROO_PROTO_PACKET | _SIMPLE */
    uint8_t  home_on_start;

    int32_t  max_position;       /* scaling for target_position */
    int32_t  max_speed;          /* scaling for target_speed (units/s) */

    int32_t  target_position;    /* last commanded position */
    int32_t  target_speed;       /* last commanded speed */
    int32_t  current_position;   /* read back via getp */
    int32_t  current_speed;      /* read back via gets */
    uint8_t  moving;             /* 1 if last reply had BUSY set */
    uint16_t error_status;       /* last Kangaroo error code (0 = OK) */

    bool     started;            /* sent Start (and Home) this session */
    bool     connected;          /* answered a recent Get */
    bool     first_cmd_logged;
    uint8_t  consecutive_misses;
    uint32_t last_reprobe_ms;

    /* ── Teach tune (see kangaroo_driver.h) ─────────────────────── */
    uint8_t  tune_state;         /* kangaroo_tune_state_t */
    uint8_t  jog_pct;            /* open-loop power cap, % of full scale */
    int32_t  jog_power;          /* last open-loop power actually sent */
    uint32_t jog_refresh_ms;     /* last kangaroo_tune_jog() call */
    uint32_t tune_keepalive_ms;  /* last keep-alive Get during a tune */
    uint32_t tune_go_ms;         /* when Go was sent */
    uint32_t tune_quiet_since;   /* first not-busy poll since Go (0 = busy) */

    int32_t  taught_min;         /* Get 8 — read-only on the wire */
    int32_t  taught_max;         /* Get 9 */
    bool     extents_valid;

    /* Operator-assigned id from the peripheral JSON. Routes inbound
     * peripheral_commands to this unit — same use as the RoboClaw
     * driver's field of the same name. */
    char     peripheral_id[32];

    /* Latched safety interlock, asserted by an external source (today a
     * switch_input peripheral tripping — see docs/SENSOR_INPUTS.md).
     * Distinct from a plain powerdown: powerdown alone is undone by the
     * very next routed setpoint, which is not an interlock. This blocks
     * motion until something explicitly clears it. */
    bool     interlocked;
} kangaroo_unit_t;

/* True while a unit owns the bus for tuning. Normal position/speed
 * polling is suspended for these — the tune's keep-alive is the only
 * traffic that may go out, and interleaving Moves would fight it. */
static inline bool tune_is_active(const kangaroo_unit_t* u)
{
    return u->tune_state == KANGAROO_TUNE_ENTERING
        || u->tune_state == KANGAROO_TUNE_JOG
        || u->tune_state == KANGAROO_TUNE_GOING;
}

/* ── Driver State ───────────────────────────────────────────────── */

static kangaroo_unit_t units[KANGAROO_MAX_UNITS];
static uint8_t  unit_count = 0;
static bool     port_initialized = false;
static uint32_t configured_baud = KANGAROO_DEFAULT_BAUD;
static uint8_t  configured_serial_port = KANGAROO_DEFAULT_SERIAL_PORT;

static uint8_t poll_unit = 0;
static uint8_t poll_param = 0;  /* 0 = position, 1 = speed */

static const kangaroo_transport_ops_t* transport(void)
{
    return kangaroo_get_transport();
}

/* ── Wire helpers (read look-ahead, mirror tic_driver.c) ────────── */

static inline bool wire_ready(void)
{
    const kangaroo_transport_ops_t* t = transport();
    return t && t->is_open();
}

static bool wire_write(const uint8_t* buf, size_t len)
{
    const kangaroo_transport_ops_t* t = transport();
    if (!t || !t->is_open()) return false;
    return t->write(buf, len);
}

static uint8_t  rx_look[48];
static size_t   rx_look_n = 0;
static size_t   rx_look_i = 0;

static bool wire_has_byte(void)
{
    if (rx_look_i < rx_look_n) return true;
    const kangaroo_transport_ops_t* t = transport();
    if (!t || !t->is_open()) return false;
    rx_look_n = t->read(rx_look, sizeof(rx_look));
    rx_look_i = 0;
    return rx_look_n > 0;
}

static uint8_t wire_getc(void)
{
    if (rx_look_i >= rx_look_n) {
        if (!wire_has_byte()) return 0;
    }
    return rx_look[rx_look_i++];
}

static void wire_drain(void)
{
    while (wire_has_byte()) (void)wire_getc();
}

/* Read one byte, waiting up to limit_ms. Returns false on timeout. */
static bool read_byte(uint32_t limit_ms, uint8_t* out)
{
    uint32_t start = PLATFORM_MILLIS();
    while (!wire_has_byte()) {
        if (PLATFORM_MILLIS() - start > limit_ms) return false;
    }
    *out = wire_getc();
    return true;
}

/* ── Reply parsing ──────────────────────────────────────────────── */

typedef struct {
    uint8_t flags;    /* KANGAROO_STATUS_* bits */
    uint8_t param;    /* echoed parameter (1=pos, 2=speed) */
    int32_t value;    /* bit-packed value (or error code if flags&ERROR) */
} kangaroo_reply_t;

/* Read & validate one packet-serial Get reply (command 67). Returns
 * true on a CRC-good reply, populating `out`. */
static bool read_packet_reply(kangaroo_reply_t* out)
{
    /* Sync to an address byte (high bit set). */
    uint8_t b;
    if (!read_byte(KANGAROO_RESPONSE_TIMEOUT_MS, &b)) return false;
    uint32_t deadline = PLATFORM_MILLIS() + KANGAROO_RESPONSE_TIMEOUT_MS;
    while (!(b & 0x80)) {
        if (PLATFORM_MILLIS() > deadline) return false;
        if (!read_byte(KANGAROO_BYTE_TIMEOUT_MS, &b)) return false;
    }

    uint8_t frame[3 + 32];   /* addr, cmd, len, up to 32 data bytes */
    frame[0] = b;            /* address */
    if (!read_byte(KANGAROO_BYTE_TIMEOUT_MS, &frame[1])) return false;  /* command */
    if (!read_byte(KANGAROO_BYTE_TIMEOUT_MS, &frame[2])) return false;  /* length  */
    uint8_t len = frame[2];
    if (len > 32) return false;

    uint8_t crc_lo, crc_hi;
    for (uint8_t i = 0; i < len; i++) {
        if (!read_byte(KANGAROO_BYTE_TIMEOUT_MS, &frame[3 + i])) return false;
    }
    if (!read_byte(KANGAROO_BYTE_TIMEOUT_MS, &crc_lo)) return false;
    if (!read_byte(KANGAROO_BYTE_TIMEOUT_MS, &crc_hi)) return false;

    if (frame[1] != KANGAROO_RC_STATUS) return false;  /* not a Get reply */

    uint16_t want = kangaroo_crc14(frame, (size_t)(3 + len));
    uint16_t got  = (uint16_t)((crc_lo & 0x7f) | ((uint16_t)(crc_hi & 0x7f) << 7));
    if (want != got) return false;

    /* data: [channel][flags]([echo])([seq])[param][value...] */
    const uint8_t* data = &frame[3];
    size_t idx = 0;
    if (idx >= len) return false;
    idx++;                                   /* channel */
    if (idx >= len) return false;
    uint8_t flags = data[idx++];
    if (flags & KANGAROO_STATUS_ECHO_CODE) { if (idx >= len) return false; idx++; }
    if (flags & KANGAROO_STATUS_SEQUENCE)  { if (idx >= len) return false; idx++; }
    if (idx >= len) return false;
    uint8_t param = data[idx++];
    int32_t value = kangaroo_bitunpack(data, len, &idx);

    out->flags = flags;
    out->param = param;
    out->value = value;
    return true;
}

/* Read one simplified-serial reply line "<ch>,<L><number>\r\n".
 * The status letter L encodes done/busy/error; we don't know which
 * parameter was queried from the line alone, so the caller passes the
 * param it requested. */
static bool read_simple_reply(uint8_t param, kangaroo_reply_t* out)
{
    char line[24];
    size_t n = 0;
    uint8_t c;
    /* First byte gets the full response budget; the rest are quick. */
    if (!read_byte(KANGAROO_RESPONSE_TIMEOUT_MS, &c)) return false;
    while (c != '\n') {
        if (n < sizeof(line) - 1 && c != '\r') line[n++] = (char)c;
        if (!read_byte(KANGAROO_BYTE_TIMEOUT_MS, &c)) break;
    }
    line[n] = '\0';

    /* Locate the comma, then the status letter + number. */
    char* comma = strchr(line, ',');
    if (!comma || !comma[1]) return false;
    char letter = comma[1];
    const char* num = &comma[2];

    uint8_t flags = 0;
    if (letter == 'E' || letter == 'e') {
        flags |= KANGAROO_STATUS_ERROR;
        if (letter == 'e') flags |= KANGAROO_STATUS_BUSY;  /* lowercase = pending */
    } else if (letter == 'p' || letter == 's') {
        flags |= KANGAROO_STATUS_BUSY;                     /* lowercase = still moving */
    } else if (letter != 'P' && letter != 'S') {
        return false;                                      /* unrecognized */
    }

    out->flags = flags;
    out->param = param;
    out->value = (int32_t)atol(num);
    return true;
}

/* ── Protocol-aware senders ─────────────────────────────────────── */

static bool send_start(kangaroo_unit_t* u)
{
    if (!wire_ready()) return false;
    if (u->protocol == KANGAROO_PROTO_SIMPLE) {
        char cmd[16];
        int len = snprintf(cmd, sizeof(cmd), "%c,start\n", (char)u->channel_name);
        bool ok = wire_write((const uint8_t*)cmd, (size_t)len);
        if (ok && u->home_on_start) {
            len = snprintf(cmd, sizeof(cmd), "%c,home\n", (char)u->channel_name);
            wire_write((const uint8_t*)cmd, (size_t)len);
        }
        return ok;
    }
    uint8_t buf[8];
    size_t n = kangaroo_build_start(u->address, u->channel_name, buf);
    bool ok = wire_write(buf, n);
    if (ok && u->home_on_start) {
        n = kangaroo_build_home(u->address, u->channel_name, buf);
        wire_write(buf, n);
    }
    return ok;
}

static bool send_move_position(kangaroo_unit_t* u, int32_t position,
                               int32_t speed_limit)
{
    if (!wire_ready()) return false;
    if (u->protocol == KANGAROO_PROTO_SIMPLE) {
        char cmd[32];
        int len;
        if (speed_limit >= 0) {
            len = snprintf(cmd, sizeof(cmd), "%c,p%ld s%ld\n",
                           (char)u->channel_name, (long)position, (long)speed_limit);
        } else {
            len = snprintf(cmd, sizeof(cmd), "%c,p%ld\n",
                           (char)u->channel_name, (long)position);
        }
        return wire_write((const uint8_t*)cmd, (size_t)len);
    }
    uint8_t buf[24];
    size_t n = kangaroo_build_move_position(u->address, u->channel_name,
                                            position, speed_limit, buf);
    return wire_write(buf, n);
}

static bool send_move_speed(kangaroo_unit_t* u, int32_t speed)
{
    if (!wire_ready()) return false;
    if (u->protocol == KANGAROO_PROTO_SIMPLE) {
        char cmd[24];
        int len = snprintf(cmd, sizeof(cmd), "%c,s%ld\n",
                           (char)u->channel_name, (long)speed);
        return wire_write((const uint8_t*)cmd, (size_t)len);
    }
    uint8_t buf[16];
    size_t n = kangaroo_build_move_speed(u->address, u->channel_name, speed, buf);
    return wire_write(buf, n);
}

static bool send_powerdown(kangaroo_unit_t* u)
{
    if (!wire_ready()) return false;
    if (u->protocol == KANGAROO_PROTO_SIMPLE) {
        char cmd[20];
        int len = snprintf(cmd, sizeof(cmd), "%c,powerdown\n", (char)u->channel_name);
        return wire_write((const uint8_t*)cmd, (size_t)len);
    }
    uint8_t buf[8];
    size_t n = kangaroo_build_powerdown(u->address, u->channel_name, buf);
    return wire_write(buf, n);
}

/* Send a System command. Packet protocol only — the tune sub-commands
 * have no simplified-serial equivalent, and silently doing nothing on a
 * SIMPLE channel would leave an operator jogging a dead UI. */
static bool send_system(kangaroo_unit_t* u, uint8_t sub, bool has_param,
                        int32_t param)
{
    if (!wire_ready()) return false;
    if (u->protocol != KANGAROO_PROTO_PACKET) return false;
    uint8_t buf[16];
    size_t n = kangaroo_build_system(u->address, u->channel_name, sub,
                                     has_param, param, buf);
    return wire_write(buf, n);
}

/* Issue a Get for `param` and read back the reply. Returns true on a
 * well-formed reply (which may itself carry an error flag). */
static bool query(kangaroo_unit_t* u, uint8_t param, kangaroo_reply_t* out)
{
    if (!wire_ready()) return false;
    wire_drain();   /* flush stale bytes from a prior failed exchange */

    if (u->protocol == KANGAROO_PROTO_SIMPLE) {
        char cmd[16];
        const char* q = (param == KANGAROO_GET_SPEED) ? "gets" : "getp";
        int len = snprintf(cmd, sizeof(cmd), "%c,%s\n", (char)u->channel_name, q);
        if (!wire_write((const uint8_t*)cmd, (size_t)len)) return false;
        return read_simple_reply(param, out);
    }
    uint8_t buf[8];
    size_t n = kangaroo_build_get(u->address, u->channel_name, param, buf);
    if (!wire_write(buf, n)) return false;
    return read_packet_reply(out);
}

/* ── Connection state machine (mirror tic_driver.c) ─────────────── */

static void mark_unit_response(uint8_t idx, bool success)
{
    if (idx >= KANGAROO_MAX_UNITS) return;
    kangaroo_unit_t* u = &units[idx];
    if (success) {
        u->consecutive_misses = 0;
        if (!u->connected) {
            u->connected = true;
            saint_log_publish("info", "Kangaroo: unit %u (addr=%u ch=%c) connected",
                (unsigned)idx, (unsigned)u->address, (char)u->channel_name);
        }
    } else {
        if (u->consecutive_misses < 255) u->consecutive_misses++;
        if (u->consecutive_misses == KANGAROO_DROP_AFTER_MISSES && u->connected) {
            u->connected = false;
            u->started = false;   /* force a fresh Start on reconnect */
            saint_log_publish("warn",
                "Kangaroo: unit %u (addr=%u ch=%c) dropped — %u missed reads",
                (unsigned)idx, (unsigned)u->address, (char)u->channel_name,
                (unsigned)u->consecutive_misses);
        }
    }
}

/* Send Start (+ Home) once per session. Idempotent on u->started so
 * per-channel apply_config sweeps don't re-Start repeatedly. */
static void start_unit(uint8_t idx)
{
    if (idx >= KANGAROO_MAX_UNITS || !wire_ready()) return;
    kangaroo_unit_t* u = &units[idx];
    if (send_start(u)) u->started = true;
}

/* ── Teach tune ─────────────────────────────────────────────────── */

static void tune_set_state(uint8_t idx, kangaroo_tune_state_t st,
                           const char* level, const char* why)
{
    kangaroo_unit_t* u = &units[idx];
    u->tune_state = (uint8_t)st;
    saint_log_publish(level, "Kangaroo: unit %u tune → %s (%s)",
        (unsigned)idx,
        st == KANGAROO_TUNE_IDLE     ? "idle"     :
        st == KANGAROO_TUNE_ENTERING ? "entering" :
        st == KANGAROO_TUNE_JOG      ? "jog"      :
        st == KANGAROO_TUNE_GOING    ? "running"  :
        st == KANGAROO_TUNE_DONE     ? "done"     : "failed",
        why);
}

/* Stop the axis and leave tune mode. Safe to call from any state — this
 * is the path e-stop, disconnect and timeout all funnel through. */
static void tune_stop(uint8_t idx, kangaroo_tune_state_t end_state,
                      const char* level, const char* why)
{
    kangaroo_unit_t* u = &units[idx];
#ifndef SIMULATION
    /* Zero power before Abort: Abort ends the tune but says nothing
     * about the open-loop power we may have left running. */
    (void)send_system(u, KANGAROO_SYS_TUNE_CONTROL_OPEN_LOOP, true, 0);
    (void)send_system(u, KANGAROO_SYS_TUNE_ABORT, false, 0);
#endif
    u->jog_power = 0;
    u->tune_quiet_since = 0;
    /* The tune leaves the channel in an indeterminate control state and
     * error 6 (serial timeout) clears with a Start, so force a fresh
     * Start before any normal motion command is accepted again. */
    u->started = false;
    tune_set_state(idx, end_state, level, why);
}

/* Serviced from kangaroo_update() for whichever unit is mid-tune. */
static void tune_tick(uint8_t idx)
{
    kangaroo_unit_t* u = &units[idx];
    uint32_t now = PLATFORM_MILLIS();

    /* Dead-man applies the moment we are jogging, regardless of state. */
    if (u->jog_power != 0
        && (uint32_t)(now - u->jog_refresh_ms) >= KANGAROO_JOG_DEADMAN_MS) {
        (void)send_system(u, KANGAROO_SYS_TUNE_CONTROL_OPEN_LOOP, true, 0);
        u->jog_power = 0;
        saint_log_publish("warn",
            "Kangaroo: unit %u jog dead-man fired — no refresh for %u ms, "
            "open-loop power zeroed", (unsigned)idx,
            (unsigned)KANGAROO_JOG_DEADMAN_MS);
    }

    if (u->tune_state == KANGAROO_TUNE_GOING
        && (uint32_t)(now - u->tune_go_ms) >= KANGAROO_TUNE_MAX_MS) {
        tune_stop(idx, KANGAROO_TUNE_FAILED, "error", "exceeded time limit");
        return;
    }

    if ((uint32_t)(now - u->tune_keepalive_ms) < KANGAROO_TUNE_KEEPALIVE_MS) {
        return;
    }
    u->tune_keepalive_ms = now;

    /* The keep-alive doubles as telemetry, so position stays live in the
     * UI while the Kangaroo drives the axis around. */
    kangaroo_reply_t r;
    if (!query(u, KANGAROO_GET_POSITION, &r)) {
        mark_unit_response(idx, false);
        /* Don't fail the tune on one missed reply — a single dropped
         * frame is normal. Losing the channel outright is handled by
         * the disconnect check in kangaroo_update(). */
        return;
    }
    mark_unit_response(idx, true);
    u->moving = (r.flags & KANGAROO_STATUS_BUSY) ? 1 : 0;

    if (r.flags & KANGAROO_STATUS_ERROR) {
        u->error_status = (uint16_t)r.value;
        /* Error 6 during a tune means the keep-alive lost the race and
         * the Kangaroo already aborted — reporting anything else would
         * be a lie. Every other error is equally terminal here. */
        tune_stop(idx, KANGAROO_TUNE_FAILED, "error",
                  r.value == KANGAROO_ERR_SERIAL_TIMEOUT
                      ? "Kangaroo aborted on serial timeout"
                      : "Kangaroo reported an error");
        return;
    }
    u->error_status = 0;
    u->current_position = r.value;

    if (u->tune_state != KANGAROO_TUNE_GOING) return;

    /* Completion: the wire gives us no explicit "tune finished" signal,
     * so infer it from the axis going quiet — but only after the cycle
     * has had time to start, since not-busy is the initial state too. */
    if (u->moving) {
        u->tune_quiet_since = 0;
        return;
    }
    if ((uint32_t)(now - u->tune_go_ms) < KANGAROO_TUNE_MIN_MS) return;
    if (u->tune_quiet_since == 0) { u->tune_quiet_since = now; return; }
    if ((uint32_t)(now - u->tune_quiet_since) < KANGAROO_TUNE_QUIET_MS) return;

    u->started = false;
    u->tune_state = (uint8_t)KANGAROO_TUNE_DONE;
    saint_log_publish("info",
        "Kangaroo: unit %u tune → done. POWER CYCLE REQUIRED before the "
        "new tune takes effect; taught extents readable after that.",
        (unsigned)idx);
}

/* ── Public API ─────────────────────────────────────────────────── */

void kangaroo_init(void)
{
    for (uint8_t i = 0; i < KANGAROO_MAX_UNITS; i++) {
        if (units[i].address == 0)       units[i].address = KANGAROO_DEFAULT_ADDRESS + i;
        if (units[i].channel_name == 0)  units[i].channel_name = KANGAROO_DEFAULT_CHANNEL_NAME;
        if (units[i].max_position == 0)  units[i].max_position = KANGAROO_DEFAULT_MAX_POSITION;
        if (units[i].max_speed == 0)     units[i].max_speed = KANGAROO_DEFAULT_MAX_SPEED;
    }

    uint8_t resolved_inst;
    uint8_t req_tx = kangaroo_tx_pin, req_rx = kangaroo_rx_pin;
    if (uart_pin_pair_lookup(kangaroo_tx_pin, kangaroo_rx_pin, &resolved_inst)) {
        configured_serial_port = resolved_inst;
    } else {
        saint_log_publish("warn",
            "Kangaroo: requested pair TX=%u RX=%u isn't a valid UART pair "
            "— falling back to TX=%u RX=%u. Check board YAML uart_pairs.",
            (unsigned)req_tx, (unsigned)req_rx,
            (unsigned)KANGAROO_DEFAULT_TX_PIN, (unsigned)KANGAROO_DEFAULT_RX_PIN);
        kangaroo_tx_pin = KANGAROO_DEFAULT_TX_PIN;
        kangaroo_rx_pin = KANGAROO_DEFAULT_RX_PIN;
        configured_serial_port = KANGAROO_DEFAULT_SERIAL_PORT;
    }

    const kangaroo_transport_ops_t* t = transport();
    if (!t) {
        /* Test / SIMULATION path: no transport. */
        active_tx_pin = kangaroo_tx_pin;
        active_rx_pin = kangaroo_rx_pin;
        active_uart   = configured_serial_port;
        port_initialized = true;
        return;
    }

    if (active_tx_pin == kangaroo_tx_pin
        && active_rx_pin == kangaroo_rx_pin
        && active_uart   == configured_serial_port
        && port_initialized && t->is_open()) {
        return;
    }

    if (!t->open(kangaroo_tx_pin, kangaroo_rx_pin, configured_baud)) {
        saint_log_publish("error",
            "Kangaroo: %s transport open failed (tx=%u rx=%u baud=%lu)",
            t->name, (unsigned)kangaroo_tx_pin, (unsigned)kangaroo_rx_pin,
            (unsigned long)configured_baud);
        return;
    }
    configured_serial_port = t->resolved_instance();
    active_tx_pin = kangaroo_tx_pin;
    active_rx_pin = kangaroo_rx_pin;
    active_uart   = configured_serial_port;

    PLATFORM_SLEEP_MS(50);

    saint_log_publish("info",
        "Kangaroo: bound UART%u TX=%u RX=%u @ %lu baud — starting %u channel(s)",
        (unsigned)configured_serial_port,
        (unsigned)kangaroo_tx_pin, (unsigned)kangaroo_rx_pin,
        (unsigned long)configured_baud, (unsigned)unit_count);

    uint8_t connected_count = 0;
    for (uint8_t i = 0; i < unit_count; i++) {
        units[i].consecutive_misses = 0;
        units[i].last_reprobe_ms = PLATFORM_MILLIS();
        start_unit(i);
        kangaroo_reply_t r;
        if (query(&units[i], KANGAROO_GET_POSITION, &r)) {
            mark_unit_response(i, true);
            connected_count++;
        }
    }
    saint_log_publish(connected_count > 0 ? "info" : "warn",
        "Kangaroo: probe complete — %u of %u channel(s) responded",
        (unsigned)connected_count, (unsigned)unit_count);

    port_initialized = true;
}

void kangaroo_update(void)
{
    if (!port_initialized || unit_count == 0) return;

#ifndef SIMULATION
    /* A tune owns the bus. Its keep-alive is the only traffic allowed
     * out — a normal Move or an interleaved Get on another channel would
     * fight the tune and risk tripping its serial timeout — so service
     * the tuning unit and return without touching the round-robin. */
    for (uint8_t t = 0; t < unit_count; t++) {
        if (!tune_is_active(&units[t])) continue;
        if (!units[t].connected) {
            tune_stop(t, KANGAROO_TUNE_FAILED, "error",
                      "channel dropped mid-tune");
            continue;
        }
        tune_tick(t);
        return;
    }

    uint8_t i = poll_unit;
    kangaroo_unit_t* u = &units[i];

    if (!u->connected) {
        uint32_t now = PLATFORM_MILLIS();
        if (now - u->last_reprobe_ms >= KANGAROO_REPROBE_INTERVAL_MS) {
            u->last_reprobe_ms = now;
            if (!u->started) start_unit(i);
            kangaroo_reply_t r;
            if (query(u, KANGAROO_GET_POSITION, &r)) mark_unit_response(i, true);
        }
        goto next_poll;
    }

    {
        uint8_t param = (poll_param == 0) ? KANGAROO_GET_POSITION : KANGAROO_GET_SPEED;
        kangaroo_reply_t r;
        bool ok = query(u, param, &r);
        if (ok) {
            u->moving = (r.flags & KANGAROO_STATUS_BUSY) ? 1 : 0;
            if (r.flags & KANGAROO_STATUS_ERROR) {
                u->error_status = (uint16_t)r.value;
                /* Auto-recover: the channel lost power and forgot it was
                 * started. Re-Start so the next setpoint takes. */
                if (r.value == KANGAROO_ERR_NOT_STARTED) {
                    u->started = false;
                    start_unit(i);
                }
            } else {
                u->error_status = 0;
                if (param == KANGAROO_GET_POSITION) u->current_position = r.value;
                else                                u->current_speed = r.value;
            }
        }
        mark_unit_response(i, ok);
    }

next_poll:
    poll_param ^= 1;
    if (poll_param == 0) {
        poll_unit++;
        if (poll_unit >= unit_count) poll_unit = 0;
    }
#endif
}

bool kangaroo_is_connected(void)
{
    if (!port_initialized) return false;
    for (uint8_t i = 0; i < unit_count; i++) {
        if (units[i].connected) return true;
    }
    return false;
}

bool kangaroo_set_position(uint8_t unit, int32_t position)
{
    if (unit >= KANGAROO_MAX_UNITS) return false;
    if (!port_initialized) {
        saint_log_publish("warn",
            "Kangaroo: set_position(unit=%u, pos=%ld) ignored — not initialized.",
            (unsigned)unit, (long)position);
        return false;
    }
    if (units[unit].interlocked) {
        saint_log_publish("warn",
            "Kangaroo: set_position(unit=%u) refused — safety interlock "
            "latched; clear it before commanding motion", (unsigned)unit);
        return false;
    }
    /* Refuse routed motion while tuning. The Kangaroo is driving the axis
     * itself and a Move here would fight the tune — and during the jog
     * stage it would move an axis whose travel limits are not yet known. */
    if (tune_is_active(&units[unit])) {
        saint_log_publish("warn",
            "Kangaroo: set_position(unit=%u) refused — tune in progress",
            (unsigned)unit);
        return false;
    }
    units[unit].target_position = position;

#ifndef SIMULATION
    if (!wire_ready()) return false;
    if (!units[unit].started) start_unit(unit);
    if (!units[unit].first_cmd_logged) {
        units[unit].first_cmd_logged = true;
        saint_log_publish("info",
            "Kangaroo: unit %u (addr=%u ch=%c) — first position move %ld",
            (unsigned)unit, (unsigned)units[unit].address,
            (char)units[unit].channel_name, (long)position);
    }
    /* Cap moves at the channel's configured max_speed (a non-negative
     * speed limit). */
    int32_t limit = units[unit].max_speed > 0 ? units[unit].max_speed : -1;
    (void)send_move_position(&units[unit], position, limit);
#endif
    return true;
}

bool kangaroo_set_speed(uint8_t unit, int32_t speed)
{
    if (unit >= KANGAROO_MAX_UNITS) return false;
    if (!port_initialized) {
        saint_log_publish("warn",
            "Kangaroo: set_speed(unit=%u, spd=%ld) ignored — not initialized.",
            (unsigned)unit, (long)speed);
        return false;
    }
    if (units[unit].interlocked) {
        saint_log_publish("warn",
            "Kangaroo: set_speed(unit=%u) refused — safety interlock latched",
            (unsigned)unit);
        return false;
    }
    if (tune_is_active(&units[unit])) {
        saint_log_publish("warn",
            "Kangaroo: set_speed(unit=%u) refused — tune in progress",
            (unsigned)unit);
        return false;
    }
    units[unit].target_speed = speed;

#ifndef SIMULATION
    if (!wire_ready()) return false;
    if (!units[unit].started) start_unit(unit);
    if (!units[unit].first_cmd_logged) {
        units[unit].first_cmd_logged = true;
        saint_log_publish("info",
            "Kangaroo: unit %u (addr=%u ch=%c) — first speed move %ld",
            (unsigned)unit, (unsigned)units[unit].address,
            (char)units[unit].channel_name, (long)speed);
    }
    (void)send_move_speed(&units[unit], speed);
#endif
    return true;
}

bool kangaroo_powerdown(uint8_t unit)
{
    if (unit >= KANGAROO_MAX_UNITS) return false;
    units[unit].target_speed = 0;
#ifndef SIMULATION
    if (!wire_ready()) return false;
    (void)send_powerdown(&units[unit]);
#endif
    units[unit].started = false;  /* powerdown drops the control loop */
    return true;
}

/* ── Teach tune, public entry points ────────────────────────────── */

/* Shared guard: tuning needs a live, initialized, packet-protocol
 * channel. Logs the specific reason — a silently-refused tune command
 * leaves an operator jogging a dead UI wondering why nothing moves. */
static bool tune_precheck(uint8_t unit, const char* what)
{
    if (unit >= KANGAROO_MAX_UNITS) return false;
    if (!port_initialized) {
        saint_log_publish("warn", "Kangaroo: tune %s(unit=%u) ignored — "
            "driver not initialized", what, (unsigned)unit);
        return false;
    }
    if (unit >= unit_count) {
        saint_log_publish("warn", "Kangaroo: tune %s(unit=%u) ignored — "
            "no such unit (%u configured)", what, (unsigned)unit,
            (unsigned)unit_count);
        return false;
    }
    if (units[unit].protocol != KANGAROO_PROTO_PACKET) {
        saint_log_publish("warn", "Kangaroo: tune %s(unit=%u) ignored — "
            "tuning requires packet serial; this channel is on simplified "
            "serial", what, (unsigned)unit);
        return false;
    }
    if (!units[unit].connected) {
        saint_log_publish("warn", "Kangaroo: tune %s(unit=%u) ignored — "
            "channel not responding", what, (unsigned)unit);
        return false;
    }
    /* Starting a tune with a limit tripped is the worst case: jogging is
     * open loop, so the axis would be driven with no feedback past a
     * point something already decided was out of bounds. */
    if (units[unit].interlocked) {
        saint_log_publish("warn", "Kangaroo: tune %s(unit=%u) refused — "
            "safety interlock latched", what, (unsigned)unit);
        return false;
    }
    return true;
}

bool kangaroo_tune_enter(uint8_t unit)
{
    if (!tune_precheck(unit, "enter")) return false;
    kangaroo_unit_t* u = &units[unit];

    if (tune_is_active(u)) {
        saint_log_publish("warn", "Kangaroo: tune enter(unit=%u) ignored — "
            "a tune is already in progress", (unsigned)unit);
        return false;
    }

#ifndef SIMULATION
    if (!send_system(u, KANGAROO_SYS_TUNE_ENTER_MODE, true,
                     KANGAROO_TUNE_MODE_TEACH)) {
        return false;
    }
    /* Every channel comes up disabled after Enter Mode and will not move
     * until the mask is cleared (Reference Manual p.14). Skipping this is
     * indistinguishable from a wiring fault at the jog step. */
    if (!send_system(u, KANGAROO_SYS_TUNE_SET_DISABLED_CH, true, 0)) {
        tune_stop(unit, KANGAROO_TUNE_FAILED, "error",
                  "could not clear the disabled-channel mask");
        return false;
    }
#endif

    u->jog_power = 0;
    u->jog_refresh_ms = PLATFORM_MILLIS();
    u->tune_keepalive_ms = PLATFORM_MILLIS();
    u->tune_quiet_since = 0;
    u->extents_valid = false;
    tune_set_state(unit, KANGAROO_TUNE_JOG, "info",
                   "Mode 1 Teach — jog to each end, then centre");
    return true;
}

bool kangaroo_tune_jog(uint8_t unit, float fraction)
{
    if (!tune_precheck(unit, "jog")) return false;
    kangaroo_unit_t* u = &units[unit];

    if (u->tune_state != KANGAROO_TUNE_JOG) {
        saint_log_publish("warn", "Kangaroo: tune jog(unit=%u) ignored — "
            "not in the jog stage", (unsigned)unit);
        return false;
    }

    if (fraction < -1.0f) fraction = -1.0f;
    if (fraction >  1.0f) fraction =  1.0f;

    /* Scale into the unit's power cap before the builder's own clamp.
     * Open loop has no protection of its own, so this cap is the only
     * thing bounding how hard an untuned axis is driven. */
    uint8_t pct = u->jog_pct > 0 ? u->jog_pct : KANGAROO_DEFAULT_JOG_PCT;
    if (pct > 100) pct = 100;
    int32_t power = (int32_t)(fraction * ((float)pct / 100.0f)
                              * (float)KANGAROO_OPEN_LOOP_MAX);

    u->jog_refresh_ms = PLATFORM_MILLIS();

#ifndef SIMULATION
    if (!send_system(u, KANGAROO_SYS_TUNE_CONTROL_OPEN_LOOP, true, power)) {
        return false;
    }
#endif
    u->jog_power = power;
    return true;
}

bool kangaroo_tune_go(uint8_t unit)
{
    if (!tune_precheck(unit, "go")) return false;
    kangaroo_unit_t* u = &units[unit];

    if (u->tune_state != KANGAROO_TUNE_JOG) {
        saint_log_publish("warn", "Kangaroo: tune go(unit=%u) ignored — "
            "must be in the jog stage (enter tune mode first)",
            (unsigned)unit);
        return false;
    }

#ifndef SIMULATION
    /* Never hand over to the tune cycle with jog power still applied. */
    (void)send_system(u, KANGAROO_SYS_TUNE_CONTROL_OPEN_LOOP, true, 0);
    u->jog_power = 0;
    if (!send_system(u, KANGAROO_SYS_TUNE_GO, false, 0)) return false;
#else
    u->jog_power = 0;
#endif

    uint32_t now = PLATFORM_MILLIS();
    u->tune_go_ms = now;
    u->tune_keepalive_ms = now;
    u->tune_quiet_since = 0;
    tune_set_state(unit, KANGAROO_TUNE_GOING, "warn",
                   "STAND CLEAR — the axis now moves under its own control");
    return true;
}

bool kangaroo_tune_abort(uint8_t unit)
{
    if (unit >= KANGAROO_MAX_UNITS) return false;
    /* Deliberately skips tune_precheck: abort must work even when the
     * channel looks unhealthy, since that is exactly when it is wanted. */
    if (!tune_is_active(&units[unit])) return false;
    tune_stop(unit, KANGAROO_TUNE_FAILED, "warn", "aborted by operator");
    return true;
}

bool kangaroo_tune_read_extents(uint8_t unit, int32_t* out_min, int32_t* out_max)
{
    if (!tune_precheck(unit, "read_extents")) return false;
    kangaroo_unit_t* u = &units[unit];

    if (tune_is_active(u)) {
        saint_log_publish("warn", "Kangaroo: read_extents(unit=%u) ignored — "
            "tune still in progress", (unsigned)unit);
        return false;
    }

#ifndef SIMULATION
    if (!u->started) start_unit(unit);

    kangaroo_reply_t r;
    if (!query(u, KANGAROO_GET_ABS_MIN, &r) || (r.flags & KANGAROO_STATUS_ERROR)) {
        saint_log_publish("warn", "Kangaroo: unit %u could not read minimum "
            "position%s", (unsigned)unit,
            (r.flags & KANGAROO_STATUS_ERROR) ? " (channel reported an error)" : "");
        return false;
    }
    int32_t lo = r.value;

    if (!query(u, KANGAROO_GET_ABS_MAX, &r) || (r.flags & KANGAROO_STATUS_ERROR)) {
        saint_log_publish("warn", "Kangaroo: unit %u could not read maximum "
            "position", (unsigned)unit);
        return false;
    }
    u->taught_min = lo;
    u->taught_max = r.value;
#endif

    u->extents_valid = true;
    if (out_min) *out_min = u->taught_min;
    if (out_max) *out_max = u->taught_max;
    saint_log_publish("info", "Kangaroo: unit %u taught travel %ld … %ld",
        (unsigned)unit, (long)u->taught_min, (long)u->taught_max);
    return true;
}

kangaroo_tune_state_t kangaroo_tune_get_state(uint8_t unit)
{
    if (unit >= KANGAROO_MAX_UNITS) return KANGAROO_TUNE_IDLE;
    return (kangaroo_tune_state_t)units[unit].tune_state;
}

void kangaroo_powerdown_all(void)
{
    for (uint8_t i = 0; i < KANGAROO_MAX_UNITS; i++) {
        (void)kangaroo_powerdown(i);
    }
}

/* ── peripheral_driver_t glue ───────────────────────────────────── */

static bool drv_init(void)
{
    /* No-op (README item 3 pattern). Real init runs from drv_load
     * (flash had unit_count > 0) or drv_apply_config. */
    return true;
}

static bool drv_set_value(uint8_t channel, float value)
{
    uint8_t unit = channel / KANGAROO_CHANNELS_PER_UNIT;
    uint8_t sub  = channel % KANGAROO_CHANNELS_PER_UNIT;
    if (unit >= KANGAROO_MAX_UNITS) return false;
    if (value < -1.0f) value = -1.0f;
    if (value >  1.0f) value =  1.0f;

    switch (sub) {
    case KANGAROO_SUB_TARGET_POSITION: {
        int32_t pos = (int32_t)(value * (float)units[unit].max_position);
        return kangaroo_set_position(unit, pos);
    }
    case KANGAROO_SUB_TARGET_SPEED: {
        int32_t spd = (int32_t)(value * (float)units[unit].max_speed);
        return kangaroo_set_speed(unit, spd);
    }
    case KANGAROO_SUB_JOG:
        /* Only meaningful mid-tune; kangaroo_tune_jog rejects it
         * otherwise, so a stray routed value can't drive the axis open
         * loop outside a tune. */
        return kangaroo_tune_jog(unit, value);
    default:
        return false;  /* read-only sub-channels */
    }
}

static bool drv_get_value(uint8_t channel, float* value)
{
    if (!value) return false;
    uint8_t unit = channel / KANGAROO_CHANNELS_PER_UNIT;
    uint8_t sub  = channel % KANGAROO_CHANNELS_PER_UNIT;
    if (unit >= KANGAROO_MAX_UNITS) return false;

    switch (sub) {
    case KANGAROO_SUB_TARGET_POSITION:
        *value = (units[unit].max_position == 0) ? 0.0f
            : (float)units[unit].target_position / (float)units[unit].max_position;
        return true;
    case KANGAROO_SUB_TARGET_SPEED:
        *value = (units[unit].max_speed == 0) ? 0.0f
            : (float)units[unit].target_speed / (float)units[unit].max_speed;
        return true;
    case KANGAROO_SUB_CURRENT_POSITION:
        *value = (float)units[unit].current_position;
        return true;
    case KANGAROO_SUB_CURRENT_SPEED:
        *value = (float)units[unit].current_speed;
        return true;
    case KANGAROO_SUB_MOVING:
        *value = (float)units[unit].moving;
        return true;
    case KANGAROO_SUB_ERROR_STATUS:
        *value = (float)units[unit].error_status;
        return true;
    case KANGAROO_SUB_JOG:
        *value = (KANGAROO_OPEN_LOOP_MAX == 0) ? 0.0f
            : (float)units[unit].jog_power / (float)KANGAROO_OPEN_LOOP_MAX;
        return true;
    case KANGAROO_SUB_TUNE_STATE:
        *value = (float)units[unit].tune_state;
        return true;
    case KANGAROO_SUB_TAUGHT_MIN:
        *value = (float)units[unit].taught_min;
        return true;
    case KANGAROO_SUB_TAUGHT_MAX:
        *value = (float)units[unit].taught_max;
        return true;
    default:
        return false;
    }
}

static void drv_set_defaults(uint8_t channel, pin_config_t* config)
{
    uint8_t unit = channel / KANGAROO_CHANNELS_PER_UNIT;
    config->params.kangaroo.address       = KANGAROO_DEFAULT_ADDRESS;
    config->params.kangaroo.channel_name  = (unit & 1) ? '2' : '1';
    config->params.kangaroo.protocol      = KANGAROO_PROTO_PACKET;
    config->params.kangaroo.home_on_start = 0;
    config->params.kangaroo.max_position  = KANGAROO_DEFAULT_MAX_POSITION;
    config->params.kangaroo.max_speed     = KANGAROO_DEFAULT_MAX_SPEED;
}

static bool drv_apply_config(uint8_t channel, const pin_config_t* config)
{
    uint8_t unit = channel / KANGAROO_CHANNELS_PER_UNIT;
    if (unit >= KANGAROO_MAX_UNITS) return false;

    /* address == 0 fingerprint = boot-reload sweep with zero params;
     * trust whatever drv_load already populated. */
    if (config->params.kangaroo.address != 0) {
        units[unit].address       = config->params.kangaroo.address;
        units[unit].channel_name  = config->params.kangaroo.channel_name
                                        ? config->params.kangaroo.channel_name
                                        : KANGAROO_DEFAULT_CHANNEL_NAME;
        units[unit].protocol      = config->params.kangaroo.protocol;
        units[unit].home_on_start = config->params.kangaroo.home_on_start;
        units[unit].max_position  = config->params.kangaroo.max_position;
        units[unit].max_speed     = config->params.kangaroo.max_speed;
        units[unit].jog_pct       = config->params.kangaroo.jog_pct;
    }

    /* Record the operator's id so peripheral_commands (the teach tune)
     * can be routed back to this unit. Guarded like the params above:
     * a boot-reload sweep passes an empty name and must not clobber it. */
    if (config->logical_name[0]) {
        strncpy(units[unit].peripheral_id, config->logical_name,
                sizeof(units[unit].peripheral_id) - 1);
        units[unit].peripheral_id[sizeof(units[unit].peripheral_id) - 1] = '\0';
    }

    if (unit >= unit_count) unit_count = unit + 1;

    kangaroo_init();
    /* Fresh sync — (re)Start this channel so the first setpoint takes. */
    units[unit].started = false;
    start_unit(unit);
    return true;
}

static bool drv_parse_json(const char* json_start, const char* json_end,
                           pin_config_t* config)
{
    const char* p;

    p = strstr(json_start, "\"address\"");
    if (p && p < json_end) { p = strchr(p, ':');
        if (p) { p++; while (*p == ' ') p++;
                 config->params.kangaroo.address = (uint8_t)atoi(p); } }

    /* channel: a JSON string like "1" / "2" / "D" / "T" — take its
     * first character. */
    p = strstr(json_start, "\"channel\"");
    if (p && p < json_end) { p = strchr(p, ':');
        if (p) { p++; while (*p == ' ' || *p == '"') p++;
                 if (*p) config->params.kangaroo.channel_name = (uint8_t)*p; } }

    /* protocol: string "packet" | "simple". */
    p = strstr(json_start, "\"protocol\"");
    if (p && p < json_end) { p = strchr(p, ':');
        if (p) { p++; while (*p == ' ' || *p == '"') p++;
                 config->params.kangaroo.protocol =
                     (*p == 's' || *p == 'S') ? KANGAROO_PROTO_SIMPLE
                                              : KANGAROO_PROTO_PACKET; } }

    p = strstr(json_start, "\"home_on_start\"");
    if (p && p < json_end) { p = strchr(p, ':');
        if (p) { p++; while (*p == ' ') p++;
                 config->params.kangaroo.home_on_start =
                     (*p == 't' || *p == '1') ? 1 : 0; } }

    p = strstr(json_start, "\"max_position\"");
    if (p && p < json_end) { p = strchr(p, ':');
        if (p) { p++; while (*p == ' ') p++;
                 config->params.kangaroo.max_position = (int32_t)atol(p); } }

    p = strstr(json_start, "\"max_speed\"");
    if (p && p < json_end) { p = strchr(p, ':');
        if (p) { p++; while (*p == ' ') p++;
                 config->params.kangaroo.max_speed = (int32_t)atol(p); } }

    /* Teach-tune open-loop power cap. Clamped to 1..100 here rather
     * than trusted: this bounds how hard an untuned axis is driven with
     * no feedback and no travel limits, so a malformed value must not
     * widen it. Absent leaves 0, which the driver reads as "use the low
     * default" — never as unlimited. */
    p = strstr(json_start, "\"jog_power_pct\"");
    if (p && p < json_end) { p = strchr(p, ':');
        if (p) { p++; while (*p == ' ') p++;
                 long v = atol(p);
                 if (v < 0)   v = 0;
                 if (v > 100) v = 100;
                 config->params.kangaroo.jog_pct = (uint8_t)v; } }

    /* Optional baud override (Kangaroo can run up to 115200). */
    p = strstr(json_start, "\"baud\"");
    if (p && p < json_end) { p = strchr(p, ':');
        if (p) { p++; while (*p == ' ') p++;
                 uint32_t b = (uint32_t)atol(p); if (b > 0) configured_baud = b; } }

    uint8_t tx, rx, inst;
    if (uart_pin_pair_parse_json(json_start, json_end, &tx, &rx, &inst)) {
        kangaroo_tx_pin = tx;
        kangaroo_rx_pin = rx;
        configured_serial_port = inst;
    }
    return true;
}

/* ── Out-of-band commands (the teach tune) ──────────────────────── */

/* Pull a numeric field out of the raw "args" object. Hand-rolled for
 * the same reason drv_parse_json is: there is no JSON parser on the
 * MCU. Returns false when the key is absent so callers can default. */
static bool args_get_float(const char* json, const char* json_end,
                           const char* key, float* out)
{
    if (!json || !json_end || json_end <= json) return false;
    char pat[32];
    int n = snprintf(pat, sizeof(pat), "\"%s\"", key);
    if (n <= 0 || (size_t)n >= sizeof(pat)) return false;

    const char* p = strstr(json, pat);
    if (!p || p >= json_end) return false;
    p = strchr(p + n, ':');
    if (!p || p >= json_end) return false;
    p++;
    while (p < json_end && (*p == ' ' || *p == '\t')) p++;
    if (p >= json_end) return false;
    *out = (float)atof(p);
    return true;
}

static bool drv_command(const char* peripheral_id, const char* command,
                        const char* args_json, const char* args_json_end)
{
    if (!peripheral_id || !command) return false;

    /* Resolve the operator's id to one of our units. Returning false for
     * an id we don't own is what lets the manager try the next driver. */
    uint8_t unit = 0xFF;
    for (uint8_t i = 0; i < unit_count; i++) {
        if (units[i].peripheral_id[0]
            && strcmp(units[i].peripheral_id, peripheral_id) == 0) {
            unit = i;
            break;
        }
    }
    if (unit == 0xFF) return false;

    if (strcmp(command, "tune_enter") == 0) {
        return kangaroo_tune_enter(unit);
    }
    if (strcmp(command, "tune_jog") == 0) {
        /* Absent//unparseable power means stop, not full speed — this
         * runs open loop with no travel limits, so the safe reading of a
         * malformed jog is zero. */
        float power = 0.0f;
        if (!args_get_float(args_json, args_json_end, "power", &power)) {
            saint_log_publish("warn",
                "Kangaroo: tune_jog for '%s' had no 'power' argument — "
                "treating as stop", peripheral_id);
            power = 0.0f;
        }
        return kangaroo_tune_jog(unit, power);
    }
    if (strcmp(command, "tune_go") == 0) {
        return kangaroo_tune_go(unit);
    }
    if (strcmp(command, "tune_abort") == 0) {
        return kangaroo_tune_abort(unit);
    }
    if (strcmp(command, "tune_read_extents") == 0) {
        return kangaroo_tune_read_extents(unit, NULL, NULL);
    }

    /* Per-instance safety interlock. The peripheral_driver_t estop()
     * vtable entry is driver-WIDE — it stops every Kangaroo the driver
     * owns — so a limit switch guarding one axis has to come in through
     * the command path, which routes by peripheral_id. See
     * docs/SENSOR_INPUTS.md. */
    if (strcmp(command, "estop") == 0) {
        units[unit].interlocked = true;
        if (tune_is_active(&units[unit])) {
            tune_stop(unit, KANGAROO_TUNE_FAILED, "warn",
                      "safety interlock tripped");
        }
        (void)kangaroo_powerdown(unit);
        saint_log_publish("warn",
            "Kangaroo: '%s' INTERLOCK — powered down; motion blocked until "
            "cleared", peripheral_id);
        return true;
    }
    if (strcmp(command, "clear_estop") == 0) {
        if (!units[unit].interlocked) return true;
        units[unit].interlocked = false;
        /* Leave the channel un-started: the next setpoint re-Starts it.
         * Clearing the latch must not itself command motion. */
        units[unit].started = false;
        saint_log_publish("info",
            "Kangaroo: '%s' interlock cleared — motion re-enabled",
            peripheral_id);
        return true;
    }

    saint_log_publish("warn",
        "Kangaroo: '%s' ignoring unknown command '%s'",
        peripheral_id, command);
    /* Claimed the peripheral, just not the verb — returning true stops
     * the manager from also logging "no driver claimed it". */
    return true;
}

static void drv_estop(void)
{
    /* Stop motion (speed 0) then power down each channel. Speed-0 is
     * defense in depth in case the power-down packet is delayed. */
    for (uint8_t i = 0; i < unit_count; i++) {
        /* An in-flight tune has the Kangaroo driving the axis on its own
         * schedule; Move/powerdown alone would not end that, so abort the
         * tune first. tune_stop also zeroes any open-loop jog power. */
        if (tune_is_active(&units[i])) {
            tune_stop(i, KANGAROO_TUNE_FAILED, "warn", "e-stop");
        }
#ifndef SIMULATION
        if (wire_ready()) {
            (void)send_move_speed(&units[i], 0);
            (void)send_powerdown(&units[i]);
        }
#endif
        units[i].started = false;
        units[i].target_speed = 0;
    }
    saint_log_publish("warn",
        "Kangaroo: ESTOP — speed 0 + power down sent to %u channel(s)",
        (unsigned)unit_count);
}

static bool drv_save(void* storage_ptr)
{
    flash_storage_data_t* storage = (flash_storage_data_t*)storage_ptr;

    memset(&storage->kangaroo_config, 0, sizeof(storage->kangaroo_config));
    storage->kangaroo_config.unit_count  = unit_count;
    storage->kangaroo_config.serial_port = configured_serial_port;
    storage->kangaroo_config.baud_rate   = configured_baud;

    storage->uart_pins.kangaroo_tx_pin = kangaroo_tx_pin;
    storage->uart_pins.kangaroo_rx_pin = kangaroo_rx_pin;

    for (uint8_t i = 0; i < KANGAROO_MAX_UNITS; i++) {
        storage->kangaroo_config.units[i].address       = units[i].address;
        storage->kangaroo_config.units[i].channel_name  = units[i].channel_name;
        storage->kangaroo_config.units[i].protocol      = units[i].protocol;
        storage->kangaroo_config.units[i].home_on_start = units[i].home_on_start;
        storage->kangaroo_config.units[i].max_position  = units[i].max_position;
        storage->kangaroo_config.units[i].max_speed     = units[i].max_speed;
    }
    return true;
}

static bool drv_load(const void* storage_ptr)
{
    const flash_storage_data_t* storage = (const flash_storage_data_t*)storage_ptr;

    uint8_t stored_tx = storage->uart_pins.kangaroo_tx_pin;
    uint8_t stored_rx = storage->uart_pins.kangaroo_rx_pin;
    if (stored_tx != 0 || stored_rx != 0) {
        uint8_t inst;
        if (uart_pin_pair_lookup(stored_tx, stored_rx, &inst)) {
            kangaroo_tx_pin = stored_tx;
            kangaroo_rx_pin = stored_rx;
            configured_serial_port = inst;
        } else {
            saint_log_publish("warn",
                "Kangaroo: stored pin pair TX=%u RX=%u isn't a valid UART pair "
                "— using defaults. Re-sync from the dashboard.",
                (unsigned)stored_tx, (unsigned)stored_rx);
        }
    }

    if (storage->kangaroo_config.unit_count == 0) {
        saint_log_publish("info",
            "Kangaroo: no saved peripheral in flash — driver dormant. "
            "Sync a Kangaroo config from the dashboard to bind the UART.");
        return true;
    }

    uint8_t count = storage->kangaroo_config.unit_count;
    if (count > KANGAROO_MAX_UNITS) count = KANGAROO_MAX_UNITS;
    unit_count = count;

    if (storage->kangaroo_config.baud_rate > 0) {
        configured_baud = storage->kangaroo_config.baud_rate;
    }
    configured_serial_port = storage->kangaroo_config.serial_port;

    for (uint8_t i = 0; i < count; i++) {
        units[i].address       = storage->kangaroo_config.units[i].address;
        units[i].channel_name  = storage->kangaroo_config.units[i].channel_name;
        units[i].protocol      = storage->kangaroo_config.units[i].protocol;
        units[i].home_on_start = storage->kangaroo_config.units[i].home_on_start;
        units[i].max_position  = storage->kangaroo_config.units[i].max_position;
        units[i].max_speed     = storage->kangaroo_config.units[i].max_speed;
    }

    saint_log_publish("info",
        "Kangaroo: restored %u channel configs from flash (UART%u TX=%u RX=%u @ %lu baud)",
        (unsigned)count, (unsigned)configured_serial_port,
        (unsigned)kangaroo_tx_pin, (unsigned)kangaroo_rx_pin,
        (unsigned long)configured_baud);

    kangaroo_init();
    return true;
}

static const peripheral_driver_t kangaroo_peripheral = {
    .name              = "kangaroo",
    .mode_string       = "kangaroo_motion",
    .pin_mode          = PIN_MODE_KANGAROO,
    .capability_flag   = PIN_CAP_KANGAROO,
    .virtual_gpio_base = KANGAROO_VIRTUAL_GPIO_BASE,
    .channel_count          = KANGAROO_MAX_CHANNELS,
    .channels_per_instance  = KANGAROO_CHANNELS_PER_UNIT,
    .init              = drv_init,
    .update            = kangaroo_update,
    .is_connected      = kangaroo_is_connected,
    .set_value         = drv_set_value,
    .get_value         = drv_get_value,
    .set_defaults      = drv_set_defaults,
    .apply_config      = drv_apply_config,
    .parse_json_params = drv_parse_json,
    .command           = drv_command,
    .estop             = drv_estop,
    .save_config       = drv_save,
    .load_config       = drv_load,
};

const peripheral_driver_t* kangaroo_get_peripheral_driver(void)
{
    return &kangaroo_peripheral;
}
