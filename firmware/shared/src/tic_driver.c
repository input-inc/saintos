/**
 * SAINT.OS Firmware - Pololu Tic Stepper Controller Driver (shared core)
 *
 * Pololu binary protocol over TTL serial. Up to 8 Tics share one
 * UART; per-unit device IDs (1..127) disambiguate addressing. Six
 * sub-channels per unit:
 *
 *   0  target_position    write, [-1.0, 1.0] → ±max_position steps
 *   1  target_velocity    write, [-1.0, 1.0] → ±max_speed_pps pulses/s
 *   2  current_position   read,  steps
 *   3  current_velocity   read,  pulses/s
 *   4  vin_voltage        read,  volts
 *   5  error_status       read,  active error bitset cast to float
 *
 * The Tic has a configurable serial command-timeout watchdog (default
 * 1 s); without keepalive packets the controller kills motion after
 * that window. We send Reset Command Timeout (0x8C, quick) on a
 * tic_DUTY_KEEPALIVE_MS cadence to keep energized units alive between
 * operator-initiated commands.
 *
 * On boot / config sync we energize each configured unit and issue
 * Exit Safe Start so the first target_position / target_velocity
 * command moves the motor without the operator having to manually
 * clear safe-start from the dashboard.
 *
 * Step mode and current limit are NOT set by the firmware — operators
 * preconfigure those via Pololu Tic Control Center over USB. This
 * driver focuses on motion control + telemetry.
 */

#include "tic_driver.h"
#include "tic_transport.h"
#include "tic_protocol.h"

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

#define TIC_DEFAULT_TX_PIN     0
#define TIC_DEFAULT_RX_PIN     1
#define TIC_DEFAULT_SERIAL_PORT 0

#define TIC_DROP_AFTER_MISSES  3
#define TIC_REPROBE_INTERVAL_MS 2000
#define TIC_KEEPALIVE_MS       400   /* under default 1 s controller timeout */
#define TIC_DEFAULT_MAX_POSITION  10000
#define TIC_DEFAULT_MAX_SPEED_PPS 1000

static uint8_t tic_tx_pin = TIC_DEFAULT_TX_PIN;
static uint8_t tic_rx_pin = TIC_DEFAULT_RX_PIN;

static uint8_t active_tx_pin = 0xFF;
static uint8_t active_rx_pin = 0xFF;
static uint8_t active_uart   = 0xFF;

/* ── Per-Unit State ─────────────────────────────────────────────── */

typedef struct {
    uint8_t  address;            /* Tic device ID, 1..127 */
    int32_t  max_position;       /* scaling for target_position */
    uint16_t max_speed_pps;      /* scaling for target_velocity */

    int32_t  target_position;    /* last commanded position (steps) */
    int32_t  target_velocity;    /* last commanded velocity (pulses/s) */
    int32_t  current_position;   /* read from TIC_VAR_CURRENT_POSITION */
    int32_t  current_velocity;   /* pulses/s, converted from wire */
    uint16_t vin_voltage_mv;
    uint16_t error_status;

    bool     energized;          /* sent ENERGIZE + EXIT_SAFE_START */
    bool     connected;          /* responded to recent Get Variable */
    bool     first_cmd_logged;
    uint8_t  consecutive_misses;
    uint32_t last_response_ms;
    uint32_t last_reprobe_ms;
    uint32_t last_command_ms;    /* any TX to this unit — for keepalive throttling */
} tic_unit_t;

/* ── Driver State ───────────────────────────────────────────────── */

static tic_unit_t units[TIC_MAX_UNITS];
static uint8_t unit_count = 0;
static bool port_initialized = false;
static uint16_t configured_baud = TIC_DEFAULT_BAUD;
static uint8_t configured_serial_port = TIC_DEFAULT_SERIAL_PORT;

static uint8_t poll_unit = 0;
static uint8_t poll_register = 0;  /* 0=pos 1=vel 2=vin 3=err */

static const tic_transport_ops_t* transport(void)
{
    return tic_get_transport();
}

/* ── Wire helpers ───────────────────────────────────────────────── */

static inline bool wire_ready(void)
{
    const tic_transport_ops_t* t = transport();
    return t && t->is_open();
}

static bool wire_write(const uint8_t* buf, size_t len)
{
    const tic_transport_ops_t* t = transport();
    if (!t || !t->is_open()) return false;
    return t->write(buf, len);
}

/* Look-ahead buffer for the one-byte-at-a-time read pattern. */
static uint8_t  rx_look[32];
static size_t   rx_look_n = 0;
static size_t   rx_look_i = 0;

static bool wire_has_byte(void)
{
    if (rx_look_i < rx_look_n) return true;
    const tic_transport_ops_t* t = transport();
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

/* ── Protocol I/O ───────────────────────────────────────────────── */

static bool send_quick(uint8_t device_id, uint8_t cmd)
{
    uint8_t buf[3];
    size_t n = tic_encode_quick(device_id, cmd, buf);
    return wire_write(buf, n);
}

static bool send_32bit(uint8_t device_id, uint8_t cmd, int32_t value)
{
    uint8_t buf[8];
    size_t n = tic_encode_32bit(device_id, cmd, value, buf);
    return wire_write(buf, n);
}

/* Issue Get Variable for (offset, length) on `device_id` and read
 * back `length` little-endian bytes into `out`. Returns true if all
 * bytes arrived before TIC_RESPONSE_TIMEOUT_MS / TIC_BYTE_TIMEOUT_MS.
 * The Tic does NOT echo the request — first response byte is the
 * low byte of the variable, no header. */
static bool get_var(uint8_t device_id, uint8_t offset, uint8_t length,
                    uint8_t* out)
{
    if (!wire_ready() || length == 0 || length > 15) return false;

    /* Drain stale bytes so a previous failed exchange doesn't poison
     * this read. */
    wire_drain();

    uint8_t req[5];
    size_t  req_n = tic_encode_block_read(device_id, offset, length, req);
    if (!wire_write(req, req_n)) return false;

    uint32_t start = PLATFORM_MILLIS();
    uint8_t i = 0;
    while (i < length) {
        while (!wire_has_byte()) {
            uint32_t waited = PLATFORM_MILLIS() - start;
            uint32_t limit = (i == 0) ? TIC_RESPONSE_TIMEOUT_MS
                                      : TIC_BYTE_TIMEOUT_MS;
            if (waited > limit) return false;
        }
        out[i++] = wire_getc();
        start = PLATFORM_MILLIS();
    }
    return true;
}

static bool get_var_i32(uint8_t device_id, uint8_t offset, int32_t* out)
{
    uint8_t buf[4];
    if (!get_var(device_id, offset, 4, buf)) return false;
    *out = tic_decode_i32(buf);
    return true;
}

static bool get_var_u16(uint8_t device_id, uint8_t offset, uint16_t* out)
{
    uint8_t buf[2];
    if (!get_var(device_id, offset, 2, buf)) return false;
    *out = tic_decode_u16(buf);
    return true;
}

/* ── Connection state machine ───────────────────────────────────── */

static void mark_unit_response(uint8_t unit_idx, bool success)
{
    if (unit_idx >= TIC_MAX_UNITS) return;
    tic_unit_t* u = &units[unit_idx];

    if (success) {
        u->last_response_ms = PLATFORM_MILLIS();
        if (u->consecutive_misses >= TIC_DROP_AFTER_MISSES) {
            saint_log_publish("info",
                "Tic: unit %u (id=%u) recovered after %u misses",
                (unsigned)unit_idx, (unsigned)u->address,
                (unsigned)u->consecutive_misses);
        }
        u->consecutive_misses = 0;
        if (!u->connected) {
            u->connected = true;
            saint_log_publish("info",
                "Tic: unit %u (id=%u) connected",
                (unsigned)unit_idx, (unsigned)u->address);
        }
    } else {
        if (u->consecutive_misses < 255) u->consecutive_misses++;
        if (u->consecutive_misses == TIC_DROP_AFTER_MISSES) {
            if (u->connected) {
                u->connected = false;
                saint_log_publish("warn",
                    "Tic: unit %u (id=%u) dropped — %u consecutive missed reads",
                    (unsigned)unit_idx, (unsigned)u->address,
                    (unsigned)u->consecutive_misses);
            } else {
                saint_log_publish("warn",
                    "Tic: unit %u (id=%u) — no response after %u attempts "
                    "(check wiring, baud, device ID, USB still attached?)",
                    (unsigned)unit_idx, (unsigned)u->address,
                    (unsigned)u->consecutive_misses);
            }
        }
    }
}

/* Probe a unit by reading TIC_VAR_OPERATION_STATE (1 byte). Cheap
 * sanity check; if the Tic responds with anything, link is alive. */
static bool probe_unit(uint8_t unit_idx)
{
    if (unit_idx >= TIC_MAX_UNITS) return false;
    uint8_t op_state;
    return get_var(units[unit_idx].address,
                   TIC_VAR_OPERATION_STATE, 1, &op_state);
}

/* Energize + Exit Safe Start. Called from drv_load (after flash
 * restore) and drv_apply_config (fresh sync). Idempotent on the
 * unit's `energized` flag so per-channel apply_config sweeps don't
 * fire ENERGIZE 6× per unit. */
static void energize_unit(uint8_t unit_idx)
{
    if (unit_idx >= TIC_MAX_UNITS) return;
    if (!wire_ready()) return;
    tic_unit_t* u = &units[unit_idx];
    (void)send_quick(u->address, TIC_CMD_ENERGIZE);
    (void)send_quick(u->address, TIC_CMD_EXIT_SAFE_START);
    u->energized = true;
    u->last_command_ms = PLATFORM_MILLIS();
}

/* Unused at the moment — the only deenergize path (tic_drv_estop)
 * inlines its own halt-and-deenergize sequence with the unit-level
 * defense-in-depth comment. Kept as the parallel "energize_unit"
 * primitive so anyone adding an operator-triggered deenergize
 * action has a one-call entry point. __attribute__((unused))
 * silences -Wunused-function without us having to rewrite the
 * estop loop. */
__attribute__((unused))
static void deenergize_unit(uint8_t unit_idx)
{
    if (unit_idx >= TIC_MAX_UNITS) return;
    if (!wire_ready()) return;
    tic_unit_t* u = &units[unit_idx];
    (void)send_quick(u->address, TIC_CMD_DEENERGIZE);
    u->energized = false;
    u->last_command_ms = PLATFORM_MILLIS();
}

/* ── Public API ─────────────────────────────────────────────────── */

void tic_init(void)
{
    /* Seed default device IDs for any never-touched unit slot. Default
     * Tic ID is 14; for multi-drop layouts the operator pre-assigns
     * 1..127 via Control Center. */
    for (uint8_t i = 0; i < TIC_MAX_UNITS; i++) {
        if (units[i].address == 0) {
            units[i].address = TIC_DEFAULT_DEVICE_ID;
        }
        if (units[i].max_position == 0) {
            units[i].max_position = TIC_DEFAULT_MAX_POSITION;
        }
        if (units[i].max_speed_pps == 0) {
            units[i].max_speed_pps = TIC_DEFAULT_MAX_SPEED_PPS;
        }
    }

    /* Validate pin pair, log loudly on fallback (same regression
     * pattern as RoboClaw/FAS100). */
    uint8_t resolved_inst;
    uint8_t req_tx = tic_tx_pin;
    uint8_t req_rx = tic_rx_pin;
    if (uart_pin_pair_lookup(tic_tx_pin, tic_rx_pin, &resolved_inst)) {
        configured_serial_port = resolved_inst;
    } else {
        saint_log_publish("warn",
            "Tic: requested pair TX=%u RX=%u isn't a valid UART pair "
            "— falling back to TX=%u RX=%u (UART0). Check board YAML uart_pairs.",
            (unsigned)req_tx, (unsigned)req_rx,
            (unsigned)TIC_DEFAULT_TX_PIN, (unsigned)TIC_DEFAULT_RX_PIN);
        tic_tx_pin = TIC_DEFAULT_TX_PIN;
        tic_rx_pin = TIC_DEFAULT_RX_PIN;
        configured_serial_port = TIC_DEFAULT_SERIAL_PORT;
    }

    const tic_transport_ops_t* t = transport();
    if (!t) {
        /* Test / SIMULATION path: no transport. Set state and bail. */
        active_tx_pin = tic_tx_pin;
        active_rx_pin = tic_rx_pin;
        active_uart   = configured_serial_port;
        port_initialized = true;
        return;
    }

    /* Idempotent fast-path. */
    if (active_tx_pin == tic_tx_pin
        && active_rx_pin == tic_rx_pin
        && active_uart   == configured_serial_port
        && port_initialized
        && t->is_open()) {
        return;
    }

    if (!t->open(tic_tx_pin, tic_rx_pin, configured_baud)) {
        saint_log_publish("error",
            "Tic: %s transport open failed (tx=%u rx=%u baud=%u)",
            t->name,
            (unsigned)tic_tx_pin, (unsigned)tic_rx_pin,
            (unsigned)configured_baud);
        return;
    }
    configured_serial_port = t->resolved_instance();

    active_tx_pin = tic_tx_pin;
    active_rx_pin = tic_rx_pin;
    active_uart   = configured_serial_port;

    PLATFORM_SLEEP_MS(50);

    saint_log_publish("info",
        "Tic: bound UART%u TX=%u RX=%u @ %u baud — probing %u unit(s)",
        (unsigned)configured_serial_port,
        (unsigned)tic_tx_pin, (unsigned)tic_rx_pin,
        (unsigned)configured_baud, (unsigned)unit_count);

    uint8_t connected_count = 0;
    for (uint8_t i = 0; i < unit_count; i++) {
        units[i].consecutive_misses = 0;
        units[i].last_reprobe_ms = PLATFORM_MILLIS();
        if (probe_unit(i)) {
            mark_unit_response(i, true);
            /* Energize + exit safe start so first motion command moves
             * without a separate operator action. */
            energize_unit(i);
            connected_count++;
        }
    }
    saint_log_publish(connected_count > 0 ? "info" : "warn",
        "Tic: probe complete — %u of %u unit(s) responded",
        (unsigned)connected_count, (unsigned)unit_count);

    port_initialized = true;
}

static bool maybe_send_keepalive(void)
{
    if (!port_initialized || unit_count == 0) return false;
    uint32_t now = PLATFORM_MILLIS();
    for (uint8_t i = 0; i < unit_count; i++) {
        tic_unit_t* u = &units[i];
        if (!u->energized) continue;
        if (now - u->last_command_ms < TIC_KEEPALIVE_MS) continue;

#ifndef SIMULATION
        if (!wire_ready()) return false;
        (void)send_quick(u->address, TIC_CMD_RESET_COMMAND_TIMEOUT);
#endif
        u->last_command_ms = now;
        return true;   /* one packet per update tick */
    }
    return false;
}

void tic_update(void)
{
    if (!port_initialized || unit_count == 0) return;

    if (maybe_send_keepalive()) return;

#ifndef SIMULATION
    uint8_t u = poll_unit;
    bool success = false;

    if (!units[u].connected) {
        uint32_t now = PLATFORM_MILLIS();
        if (now - units[u].last_reprobe_ms >= TIC_REPROBE_INTERVAL_MS) {
            units[u].last_reprobe_ms = now;
            if (probe_unit(u)) {
                mark_unit_response(u, true);
                energize_unit(u);
            }
        }
        goto next_poll;
    }

    switch (poll_register) {
    case 0: {
        int32_t v;
        if (get_var_i32(units[u].address, TIC_VAR_CURRENT_POSITION, &v)) {
            units[u].current_position = v;
            success = true;
        }
        break;
    }
    case 1: {
        int32_t v;
        if (get_var_i32(units[u].address, TIC_VAR_CURRENT_VELOCITY, &v)) {
            /* Wire is pulses per 10000 seconds — convert to PPS. */
            units[u].current_velocity = v / TIC_VELOCITY_WIRE_SCALE;
            success = true;
        }
        break;
    }
    case 2: {
        uint16_t v;
        if (get_var_u16(units[u].address, TIC_VAR_VIN_VOLTAGE, &v)) {
            units[u].vin_voltage_mv = v;
            success = true;
        }
        break;
    }
    case 3: {
        uint16_t v;
        if (get_var_u16(units[u].address, TIC_VAR_ERROR_STATUS, &v)) {
            units[u].error_status = v;
            success = true;
        }
        break;
    }
    }
    mark_unit_response(u, success);

next_poll:
    poll_register++;
    if (poll_register > 3) {
        poll_register = 0;
        poll_unit++;
        if (poll_unit >= unit_count) poll_unit = 0;
    }
#endif
}

bool tic_is_connected(void)
{
    if (!port_initialized) return false;
    for (uint8_t i = 0; i < unit_count; i++) {
        if (units[i].connected) return true;
    }
    return false;
}

bool tic_set_target_position(uint8_t unit, int32_t position)
{
    if (unit >= TIC_MAX_UNITS) return false;
    if (!port_initialized) {
        saint_log_publish("warn",
            "Tic: set_target_position(unit=%u, pos=%ld) ignored — driver "
            "not initialized.", (unsigned)unit, (long)position);
        return false;
    }
    units[unit].target_position = position;

#ifndef SIMULATION
    if (!wire_ready()) return false;
    if (!units[unit].energized) energize_unit(unit);
    if (!units[unit].first_cmd_logged) {
        units[unit].first_cmd_logged = true;
        saint_log_publish("info",
            "Tic: unit %u (id=%u) — dispatching first SET_TARGET_POSITION %ld",
            (unsigned)unit, (unsigned)units[unit].address, (long)position);
    }
    (void)send_32bit(units[unit].address, TIC_CMD_SET_TARGET_POSITION, position);
    units[unit].last_command_ms = PLATFORM_MILLIS();
#endif
    return true;
}

bool tic_set_target_velocity(uint8_t unit, int32_t pulses_per_sec)
{
    if (unit >= TIC_MAX_UNITS) return false;
    if (!port_initialized) {
        saint_log_publish("warn",
            "Tic: set_target_velocity(unit=%u, pps=%ld) ignored — driver "
            "not initialized.", (unsigned)unit, (long)pulses_per_sec);
        return false;
    }
    units[unit].target_velocity = pulses_per_sec;

#ifndef SIMULATION
    if (!wire_ready()) return false;
    if (!units[unit].energized) energize_unit(unit);
    if (!units[unit].first_cmd_logged) {
        units[unit].first_cmd_logged = true;
        saint_log_publish("info",
            "Tic: unit %u (id=%u) — dispatching first SET_TARGET_VELOCITY "
            "%ld pps", (unsigned)unit, (unsigned)units[unit].address,
            (long)pulses_per_sec);
    }
    /* Wire unit is pulses per 10000 seconds. Multiply, watching for
     * overflow at the int32 boundary — operator scaling caps PPS at
     * max_speed_pps so this is bounded in practice. */
    int64_t wire = (int64_t)pulses_per_sec * (int64_t)TIC_VELOCITY_WIRE_SCALE;
    if (wire >  INT32_MAX) wire = INT32_MAX;
    if (wire < -INT32_MAX) wire = -INT32_MAX;
    (void)send_32bit(units[unit].address, TIC_CMD_SET_TARGET_VELOCITY,
                     (int32_t)wire);
    units[unit].last_command_ms = PLATFORM_MILLIS();
#endif
    return true;
}

bool tic_halt(uint8_t unit)
{
    if (unit >= TIC_MAX_UNITS) return false;
    units[unit].target_velocity = 0;
#ifndef SIMULATION
    if (!wire_ready()) return false;
    (void)send_quick(units[unit].address, TIC_CMD_HALT_AND_HOLD);
    units[unit].last_command_ms = PLATFORM_MILLIS();
#endif
    return true;
}

void tic_halt_all(void)
{
    for (uint8_t i = 0; i < TIC_MAX_UNITS; i++) {
        (void)tic_halt(i);
    }
}

/* ── peripheral_driver_t glue ───────────────────────────────────── */

static bool tic_drv_init(void)
{
    /* No-op (README item 3 pattern). Real init runs from drv_load
     * (flash had unit_count > 0) or drv_apply_config. */
    return true;
}

static bool tic_drv_set_value(uint8_t channel, float value)
{
    uint8_t unit = channel / TIC_CHANNELS_PER_UNIT;
    uint8_t sub = channel % TIC_CHANNELS_PER_UNIT;
    if (unit >= TIC_MAX_UNITS) return false;
    if (value < -1.0f) value = -1.0f;
    if (value >  1.0f) value =  1.0f;

    switch (sub) {
    case TIC_SUB_TARGET_POSITION: {
        int32_t pos = (int32_t)(value * (float)units[unit].max_position);
        return tic_set_target_position(unit, pos);
    }
    case TIC_SUB_TARGET_VELOCITY: {
        int32_t pps = (int32_t)(value * (float)units[unit].max_speed_pps);
        return tic_set_target_velocity(unit, pps);
    }
    default:
        return false;
    }
}

static bool tic_drv_get_value(uint8_t channel, float* value)
{
    if (!value) return false;
    uint8_t unit = channel / TIC_CHANNELS_PER_UNIT;
    uint8_t sub = channel % TIC_CHANNELS_PER_UNIT;
    if (unit >= TIC_MAX_UNITS) return false;

    switch (sub) {
    case TIC_SUB_TARGET_POSITION:
        *value = (units[unit].max_position == 0) ? 0.0f
            : (float)units[unit].target_position / (float)units[unit].max_position;
        return true;
    case TIC_SUB_TARGET_VELOCITY:
        *value = (units[unit].max_speed_pps == 0) ? 0.0f
            : (float)units[unit].target_velocity / (float)units[unit].max_speed_pps;
        return true;
    case TIC_SUB_CURRENT_POSITION:
        *value = (float)units[unit].current_position;
        return true;
    case TIC_SUB_CURRENT_VELOCITY:
        *value = (float)units[unit].current_velocity;
        return true;
    case TIC_SUB_VIN_VOLTAGE:
        *value = (float)units[unit].vin_voltage_mv / 1000.0f;
        return true;
    case TIC_SUB_ERROR_STATUS:
        *value = (float)units[unit].error_status;
        return true;
    default:
        return false;
    }
}

static void tic_drv_set_defaults(uint8_t channel, pin_config_t* config)
{
    (void)channel;
    config->params.tic.address = TIC_DEFAULT_DEVICE_ID;
    config->params.tic.max_position = TIC_DEFAULT_MAX_POSITION;
    config->params.tic.max_speed_pps = TIC_DEFAULT_MAX_SPEED_PPS;
}

static bool tic_drv_apply_config(uint8_t channel, const pin_config_t* config)
{
    uint8_t unit = channel / TIC_CHANNELS_PER_UNIT;
    if (unit >= TIC_MAX_UNITS) return false;

    /* Address == 0 fingerprint = boot-reload sweep with zero params;
     * trust whatever drv_load already populated. */
    if (config->params.tic.address != 0) {
        units[unit].address       = config->params.tic.address;
        units[unit].max_position  = config->params.tic.max_position;
        units[unit].max_speed_pps = config->params.tic.max_speed_pps;
    }

    if (unit >= unit_count) unit_count = unit + 1;

    tic_init();
    return true;
}

static bool tic_drv_parse_json(const char* json_start, const char* json_end,
                                pin_config_t* config)
{
    const char* p;
    bool got_pins = false;

    p = strstr(json_start, "\"address\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) { p++; while (*p == ' ') p++;
                 config->params.tic.address = (uint8_t)atoi(p); }
    }

    p = strstr(json_start, "\"max_position\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) { p++; while (*p == ' ') p++;
                 config->params.tic.max_position = (int32_t)atol(p); }
    }

    p = strstr(json_start, "\"max_speed_pps\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) { p++; while (*p == ' ') p++;
                 config->params.tic.max_speed_pps = (uint16_t)atoi(p); }
    }

    uint8_t tx, rx, inst;
    if (uart_pin_pair_parse_json(json_start, json_end, &tx, &rx, &inst)) {
        tic_tx_pin = tx;
        tic_rx_pin = rx;
        configured_serial_port = inst;
        got_pins = true;
    }

    /* Log once per sync, on the first channel of the first unit.
     * pin_config_t.gpio is uint8_t, so TIC_VIRTUAL_GPIO_BASE (300)
     * truncates to (uint8_t)44 when stored. Compare against the
     * truncated value so the match still works for bases > 255. */
    if (config->gpio == (uint8_t)TIC_VIRTUAL_GPIO_BASE) {
        if (got_pins) {
            saint_log_publish("info",
                "Tic sync: pins TX=%u RX=%u (UART%u), unit 0 id=%u "
                "max_pos=%ld max_pps=%u",
                (unsigned)tic_tx_pin, (unsigned)tic_rx_pin,
                (unsigned)configured_serial_port,
                (unsigned)config->params.tic.address,
                (long)config->params.tic.max_position,
                (unsigned)config->params.tic.max_speed_pps);
        } else {
            saint_log_publish("warn",
                "Tic sync: didn't find uart_tx/uart_rx in JSON — "
                "driver will keep using TX=%u RX=%u",
                (unsigned)tic_tx_pin, (unsigned)tic_rx_pin);
        }
    }
    return true;
}

static void tic_drv_estop(void)
{
    /* Auto-energize / deenergize-on-estop pattern: ESTOP deenergizes
     * every configured unit AND commands halt-and-hold to all units
     * (defense in depth — even if deenergize is delayed by the wire,
     * the motor is already commanded to a stop). */
    for (uint8_t i = 0; i < unit_count; i++) {
#ifndef SIMULATION
        if (wire_ready()) {
            (void)send_quick(units[i].address, TIC_CMD_HALT_AND_HOLD);
            (void)send_quick(units[i].address, TIC_CMD_DEENERGIZE);
        }
#endif
        units[i].energized = false;
    }
    saint_log_publish("warn",
        "Tic: ESTOP — deenergized + halt-and-hold sent to %u unit(s)",
        (unsigned)unit_count);
}

static bool tic_drv_save(void* storage_ptr)
{
    flash_storage_data_t* storage = (flash_storage_data_t*)storage_ptr;

    memset(&storage->tic_config, 0, sizeof(storage->tic_config));
    storage->tic_config.unit_count = unit_count;
    storage->tic_config.serial_port = configured_serial_port;
    storage->tic_config.baud_rate = configured_baud;

    storage->uart_pins.tic_tx_pin = tic_tx_pin;
    storage->uart_pins.tic_rx_pin = tic_rx_pin;

    for (uint8_t i = 0; i < TIC_MAX_UNITS; i++) {
        storage->tic_config.units[i].address       = units[i].address;
        storage->tic_config.units[i].max_speed_pps = units[i].max_speed_pps;
        storage->tic_config.units[i].max_position  = units[i].max_position;
    }
    return true;
}

static bool tic_drv_load(const void* storage_ptr)
{
    const flash_storage_data_t* storage = (const flash_storage_data_t*)storage_ptr;

    uint8_t stored_tx = storage->uart_pins.tic_tx_pin;
    uint8_t stored_rx = storage->uart_pins.tic_rx_pin;
    if (stored_tx != 0 || stored_rx != 0) {
        uint8_t inst;
        if (uart_pin_pair_lookup(stored_tx, stored_rx, &inst)) {
            tic_tx_pin = stored_tx;
            tic_rx_pin = stored_rx;
            configured_serial_port = inst;
        } else {
            saint_log_publish("warn",
                "Tic: stored pin pair TX=%u RX=%u isn't a valid UART pair "
                "— using defaults. Re-sync from the dashboard.",
                (unsigned)stored_tx, (unsigned)stored_rx);
        }
    }

    if (storage->tic_config.unit_count == 0) {
        saint_log_publish("info",
            "Tic: no saved peripheral in flash — driver dormant. "
            "Sync a Tic config from the dashboard to bind the UART.");
        return true;
    }

    uint8_t count = storage->tic_config.unit_count;
    if (count > TIC_MAX_UNITS) count = TIC_MAX_UNITS;
    unit_count = count;

    if (storage->tic_config.baud_rate > 0) {
        configured_baud = storage->tic_config.baud_rate;
    }
    configured_serial_port = storage->tic_config.serial_port;

    for (uint8_t i = 0; i < count; i++) {
        units[i].address       = storage->tic_config.units[i].address;
        units[i].max_speed_pps = storage->tic_config.units[i].max_speed_pps;
        units[i].max_position  = storage->tic_config.units[i].max_position;
    }

    saint_log_publish("info",
        "Tic: restored %u unit configs from flash (UART%u TX=%u RX=%u @ %u baud)",
        (unsigned)count, (unsigned)configured_serial_port,
        (unsigned)tic_tx_pin, (unsigned)tic_rx_pin,
        (unsigned)configured_baud);

    tic_init();
    return true;
}

static const peripheral_driver_t tic_peripheral = {
    .name              = "tic",
    .mode_string       = "tic_stepper",
    .pin_mode          = PIN_MODE_TIC_STEPPER,
    .capability_flag   = PIN_CAP_TIC_STEPPER,
    .virtual_gpio_base = TIC_VIRTUAL_GPIO_BASE,
    .channel_count          = TIC_MAX_CHANNELS,
    .channels_per_instance  = TIC_CHANNELS_PER_UNIT,
    .init              = tic_drv_init,
    .update            = tic_update,
    .is_connected      = tic_is_connected,
    .set_value         = tic_drv_set_value,
    .get_value         = tic_drv_get_value,
    .set_defaults      = tic_drv_set_defaults,
    .apply_config      = tic_drv_apply_config,
    .parse_json_params = tic_drv_parse_json,
    .estop             = tic_drv_estop,
    .save_config       = tic_drv_save,
    .load_config       = tic_drv_load,
};

const peripheral_driver_t* tic_get_peripheral_driver(void)
{
    return &tic_peripheral;
}
