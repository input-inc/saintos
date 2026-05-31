/**
 * SAINT.OS Firmware - TMC2208 Stepper Driver (shared core)
 *
 * Up to 4 TMC2208 chips on one UART (slave addresses 0..3 via MS1/MS2
 * pin strapping). The MCU generates STEP/DIR pulses via the per-
 * platform transport, and configures each chip's microsteps, current,
 * and stealthChop mode via the UART register-write protocol.
 *
 * Motion model: constant velocity, no acceleration ramp. When the
 * operator commands target_position, the firmware computes:
 *   steps_to_go = target_position - current_position
 *   direction   = sign(steps_to_go)
 * sets the DIR pin, calls transport.axis_set_rate(..., target_velocity),
 * and lets the platform-side timer/PIO emit pulses. Each emitted pulse
 * fires tmc2208_step_done(axis) which decrements steps_to_go and
 * stops the axis when it hits zero.
 *
 * Wire-level config sent at apply_config / drv_load:
 *   - GCONF: pdn_disable=1, mstep_reg_select=1, en_spreadCycle from operator
 *   - CHOPCONF: TOFF=3 etc + MRES from microsteps choice
 *   - IHOLD_IRUN: IRUN/IHOLD computed from run/hold mA + Rsense
 *   - TPOWERDOWN: 20 (~5s power-down delay after standstill)
 */

#include "tmc2208_driver.h"
#include "tmc2208_transport.h"

#include "flash_types.h"
#include "peripheral_driver.h"
#include "pin_types.h"
#include "platform.h"
#include "saint_log.h"
#include "uart_pin_pairs.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef INT32_MAX
#define INT32_MAX 0x7FFFFFFF
#endif

/* ── Configuration ──────────────────────────────────────────────── */

#define TMC2208_DEFAULT_TX_PIN     0
#define TMC2208_DEFAULT_RX_PIN     1
#define TMC2208_DEFAULT_RSENSE_MOHM  110   /* 0.11Ω stock */
#define TMC2208_DEFAULT_MICROSTEPS   16
#define TMC2208_DEFAULT_RUN_MA       500
#define TMC2208_DEFAULT_HOLD_MA      200
#define TMC2208_DEFAULT_MAX_POSITION 10000
#define TMC2208_DEFAULT_MAX_SPEED    1000
#define TMC2208_TPOWERDOWN_DEFAULT   20    /* ~5.6s after last step */
#define TMC2208_PROBE_INTERVAL_MS    2000

static uint8_t tmc2208_tx_pin = TMC2208_DEFAULT_TX_PIN;
static uint8_t tmc2208_rx_pin = TMC2208_DEFAULT_RX_PIN;

static uint8_t active_tx_pin = 0xFF;
static uint8_t active_rx_pin = 0xFF;
static uint8_t active_uart   = 0xFF;

/* ── Per-Axis State ─────────────────────────────────────────────── */

typedef struct {
    uint8_t  address;          /* TMC2208 slave addr 0..3 */
    uint8_t  step_pin;
    uint8_t  dir_pin;
    uint16_t rsense_milliohm;
    uint16_t microsteps;
    uint16_t run_current_ma;
    uint16_t hold_current_ma;
    uint8_t  stealth_chop;     /* 0 = SpreadCycle, 1 = StealthChop */

    int32_t  max_position;
    uint16_t max_speed_pps;

    /* Runtime state */
    int32_t  target_position;
    int32_t  target_velocity;  /* signed PPS; sign carries direction */
    volatile int32_t  current_position;  /* updated from step ISR */
    volatile int32_t  steps_remaining;   /* signed; ISR decrements toward 0 */
    int8_t   active_direction;           /* +1 / -1 / 0 (idle) */

    uint32_t error_flags;      /* latest DRV_STATUS read */
    bool     attached;         /* axis_attach() succeeded */
    bool     configured;       /* chip-side config registers written */
    bool     connected;        /* responded to last GCONF readback */
    uint8_t  consecutive_misses;
    uint32_t last_response_ms;
    uint32_t last_reprobe_ms;
    bool     first_cmd_logged;
} tmc2208_axis_t;

/* ── Driver State ───────────────────────────────────────────────── */

static tmc2208_axis_t axes[TMC2208_MAX_AXES];
static uint8_t axis_count = 0;
static bool port_initialized = false;
static uint32_t configured_baud = TMC2208_DEFAULT_BAUD;
static uint8_t configured_serial_port = 0;

/* Round-robin telemetry poll across attached axes (only error_flags
 * is polled here — current_position is maintained by the step ISR). */
static uint8_t poll_axis = 0;

static const tmc2208_transport_ops_t* transport(void)
{
    return tmc2208_get_transport();
}

/* ── Register I/O ───────────────────────────────────────────────── */

static bool reg_write(uint8_t slave_addr, uint8_t reg, uint32_t value)
{
    const tmc2208_transport_ops_t* t = transport();
    if (!t || !t->is_open()) return false;
    uint8_t frame[8];
    tmc2208_build_write(slave_addr, reg, value, frame);
    /* Write frames don't get an explicit reply (the next IFCNT read
     * is how callers confirm). xfer drains the echo for us. */
    (void)t->xfer(frame, TMC2208_WRITE_FRAME_BYTES, NULL, 0);
    return true;
}

static bool reg_read(uint8_t slave_addr, uint8_t reg, uint32_t* out)
{
    const tmc2208_transport_ops_t* t = transport();
    if (!t || !t->is_open()) return false;

    uint8_t req[4];
    tmc2208_build_read_request(slave_addr, reg, req);
    uint8_t reply[8] = {0};
    size_t got = t->xfer(req, TMC2208_READ_REQUEST_BYTES,
                         reply, TMC2208_READ_REPLY_BYTES);
    if (got < TMC2208_READ_REPLY_BYTES) return false;
    return tmc2208_parse_read_reply(reply, reg, out);
}

/* ── Chip configuration ─────────────────────────────────────────── */

static bool configure_axis(uint8_t axis_idx)
{
    if (axis_idx >= TMC2208_MAX_AXES) return false;
    tmc2208_axis_t* a = &axes[axis_idx];

    /* GCONF: enable UART control, register-driven microsteps, optional
     * SpreadCycle. pdn_disable=1 keeps the chip listening on UART
     * regardless of the PDN_UART pin level. */
    uint32_t gconf = TMC2208_GCONF_PDN_DISABLE
                   | TMC2208_GCONF_MSTEP_REG_SELECT
                   | TMC2208_GCONF_MULTISTEP_FILT;
    if (!a->stealth_chop) gconf |= TMC2208_GCONF_EN_SPREADCYCLE;
    if (!reg_write(a->address, TMC2208_REG_GCONF, gconf)) return false;

    /* CHOPCONF: default chopper + MRES from microsteps + INTPOL on
     * (smooths low-microstep operation). */
    uint8_t mres = tmc2208_mres_for_microsteps(a->microsteps);
    uint32_t chopconf = TMC2208_CHOPCONF_DEFAULT
                      | (((uint32_t)mres) << TMC2208_CHOPCONF_MRES_SHIFT)
                      | TMC2208_CHOPCONF_INTPOL;
    if (!reg_write(a->address, TMC2208_REG_CHOPCONF, chopconf)) return false;

    /* IHOLD_IRUN: CS values from operator-supplied currents + Rsense.
     * Log when clamping at the high end so the operator knows their
     * request was capped (typical motors at 0.11Ω cap at ~1700mA RMS). */
    uint8_t irun_cs  = tmc2208_current_to_cs(a->run_current_ma,  a->rsense_milliohm);
    uint8_t ihold_cs = tmc2208_current_to_cs(a->hold_current_ma, a->rsense_milliohm);
    if (irun_cs >= 31 && a->run_current_ma > 0) {
        saint_log_publish("warn",
            "TMC2208 axis %u: requested %u mA RUN exceeds Rsense=%u mΩ range "
            "(VSENSE=0); clamped to CS=31 (max for this sense resistor)",
            (unsigned)axis_idx,
            (unsigned)a->run_current_ma, (unsigned)a->rsense_milliohm);
    }
    uint32_t ihold_irun = tmc2208_pack_ihold_irun(ihold_cs, irun_cs, /*delay*/ 8);
    if (!reg_write(a->address, TMC2208_REG_IHOLD_IRUN, ihold_irun)) return false;

    /* TPOWERDOWN: auto power-down delay after standstill. */
    if (!reg_write(a->address, TMC2208_REG_TPOWERDOWN, TMC2208_TPOWERDOWN_DEFAULT))
        return false;

    a->configured = true;
    saint_log_publish("info",
        "TMC2208 axis %u (slave %u) configured: %u µstep, IRUN=%u/IHOLD=%u "
        "(%u/%u mA req, Rsense=%u mΩ), %s",
        (unsigned)axis_idx, (unsigned)a->address,
        (unsigned)a->microsteps, (unsigned)irun_cs, (unsigned)ihold_cs,
        (unsigned)a->run_current_ma, (unsigned)a->hold_current_ma,
        (unsigned)a->rsense_milliohm,
        a->stealth_chop ? "StealthChop" : "SpreadCycle");
    return true;
}

/* Read GCONF back as a connectivity probe — any successful read
 * with matching slave-reply address validates the link. */
static bool probe_axis(uint8_t axis_idx)
{
    if (axis_idx >= TMC2208_MAX_AXES) return false;
    uint32_t gconf;
    return reg_read(axes[axis_idx].address, TMC2208_REG_GCONF, &gconf);
}

static void mark_axis_response(uint8_t axis_idx, bool success)
{
    if (axis_idx >= TMC2208_MAX_AXES) return;
    tmc2208_axis_t* a = &axes[axis_idx];

    if (success) {
        a->last_response_ms = PLATFORM_MILLIS();
        if (a->consecutive_misses >= 3) {
            saint_log_publish("info",
                "TMC2208 axis %u (slave %u) recovered after %u misses",
                (unsigned)axis_idx, (unsigned)a->address,
                (unsigned)a->consecutive_misses);
        }
        a->consecutive_misses = 0;
        if (!a->connected) {
            a->connected = true;
            saint_log_publish("info",
                "TMC2208 axis %u (slave %u) connected",
                (unsigned)axis_idx, (unsigned)a->address);
        }
    } else {
        if (a->consecutive_misses < 255) a->consecutive_misses++;
        if (a->consecutive_misses == 3) {
            if (a->connected) {
                a->connected = false;
                saint_log_publish("warn",
                    "TMC2208 axis %u (slave %u) dropped — %u missed reads",
                    (unsigned)axis_idx, (unsigned)a->address,
                    (unsigned)a->consecutive_misses);
            } else {
                saint_log_publish("warn",
                    "TMC2208 axis %u (slave %u) — no UART reply after %u "
                    "attempts (check MS1/MS2 strapping, PDN_UART wiring, "
                    "baud)",
                    (unsigned)axis_idx, (unsigned)a->address,
                    (unsigned)a->consecutive_misses);
            }
        }
    }
}

/* ── Step-completion callback (called from ISR/timer per pulse) ── */

bool tmc2208_step_done(uint8_t axis)
{
    if (axis >= TMC2208_MAX_AXES) return false;
    tmc2208_axis_t* a = &axes[axis];

    /* Increment current_position toward target, decrement remaining.
     * Use the latched active_direction so we don't race a target
     * change happening on the main thread. */
    if (a->active_direction > 0) {
        a->current_position++;
    } else if (a->active_direction < 0) {
        a->current_position--;
    }
    if (a->steps_remaining > 0) {
        a->steps_remaining--;
        if (a->steps_remaining == 0) {
            /* Caller (transport timer) should stop scheduling. */
            return false;
        }
        return true;
    }
    /* Velocity mode or unbounded — keep running until the operator
     * stops us via tmc2208_halt() or a new target_velocity=0. */
    return true;
}

/* ── Public API ─────────────────────────────────────────────────── */

void tmc2208_init(void)
{
    /* Seed sensible defaults for never-touched axes so the JSON sync
     * path doesn't have to explicitly set everything. */
    for (uint8_t i = 0; i < TMC2208_MAX_AXES; i++) {
        if (axes[i].rsense_milliohm == 0) axes[i].rsense_milliohm = TMC2208_DEFAULT_RSENSE_MOHM;
        if (axes[i].microsteps      == 0) axes[i].microsteps      = TMC2208_DEFAULT_MICROSTEPS;
        if (axes[i].max_position    == 0) axes[i].max_position    = TMC2208_DEFAULT_MAX_POSITION;
        if (axes[i].max_speed_pps   == 0) axes[i].max_speed_pps   = TMC2208_DEFAULT_MAX_SPEED;
    }

    /* Validate pin pair. */
    uint8_t resolved_inst;
    uint8_t req_tx = tmc2208_tx_pin;
    uint8_t req_rx = tmc2208_rx_pin;
    if (uart_pin_pair_lookup(tmc2208_tx_pin, tmc2208_rx_pin, &resolved_inst)) {
        configured_serial_port = resolved_inst;
    } else {
        saint_log_publish("warn",
            "TMC2208: requested pair TX=%u RX=%u isn't a valid UART pair "
            "— falling back to defaults",
            (unsigned)req_tx, (unsigned)req_rx);
        tmc2208_tx_pin = TMC2208_DEFAULT_TX_PIN;
        tmc2208_rx_pin = TMC2208_DEFAULT_RX_PIN;
        configured_serial_port = 0;
    }

    const tmc2208_transport_ops_t* t = transport();
    if (!t) {
        /* SIMULATION / test path. */
        active_tx_pin = tmc2208_tx_pin;
        active_rx_pin = tmc2208_rx_pin;
        active_uart   = configured_serial_port;
        port_initialized = true;
        return;
    }

    /* Idempotent fast-path. */
    if (active_tx_pin == tmc2208_tx_pin
        && active_rx_pin == tmc2208_rx_pin
        && active_uart   == configured_serial_port
        && port_initialized
        && t->is_open()) {
        return;
    }

    if (!t->open(tmc2208_tx_pin, tmc2208_rx_pin, configured_baud)) {
        saint_log_publish("error",
            "TMC2208: %s transport open failed (tx=%u rx=%u baud=%u)",
            t->name, (unsigned)tmc2208_tx_pin, (unsigned)tmc2208_rx_pin,
            (unsigned)configured_baud);
        return;
    }
    configured_serial_port = t->resolved_instance();
    active_tx_pin = tmc2208_tx_pin;
    active_rx_pin = tmc2208_rx_pin;
    active_uart   = configured_serial_port;

    PLATFORM_SLEEP_MS(20);

    saint_log_publish("info",
        "TMC2208: bound UART%u TX=%u RX=%u @ %u baud — configuring %u axis%s",
        (unsigned)configured_serial_port,
        (unsigned)tmc2208_tx_pin, (unsigned)tmc2208_rx_pin,
        (unsigned)configured_baud,
        (unsigned)axis_count, axis_count == 1 ? "" : "es");

    /* Attach step generation + push initial config for each known axis. */
    for (uint8_t i = 0; i < axis_count; i++) {
        tmc2208_axis_t* a = &axes[i];
        a->consecutive_misses = 0;
        a->last_reprobe_ms = PLATFORM_MILLIS();
        if (!a->attached && a->step_pin != 0xFF && a->dir_pin != 0xFF) {
            if (t->axis_attach(i, a->step_pin, a->dir_pin)) {
                a->attached = true;
            } else {
                saint_log_publish("warn",
                    "TMC2208 axis %u: axis_attach failed (step=%u dir=%u) "
                    "— step generation unavailable for this axis",
                    (unsigned)i, (unsigned)a->step_pin, (unsigned)a->dir_pin);
            }
        }
        if (probe_axis(i)) {
            mark_axis_response(i, true);
            configure_axis(i);
        }
    }
    port_initialized = true;
}

void tmc2208_update(void)
{
    if (!port_initialized || axis_count == 0) return;

#ifndef SIMULATION
    /* Round-robin DRV_STATUS read for the connected axes — one per
     * tick to keep the bus quiet. */
    uint8_t a = poll_axis;
    if (a < axis_count) {
        if (axes[a].connected) {
            uint32_t status;
            if (reg_read(axes[a].address, TMC2208_REG_DRV_STATUS, &status)) {
                axes[a].error_flags = status;
                mark_axis_response(a, true);
            } else {
                mark_axis_response(a, false);
            }
        } else {
            uint32_t now = PLATFORM_MILLIS();
            if (now - axes[a].last_reprobe_ms >= TMC2208_PROBE_INTERVAL_MS) {
                axes[a].last_reprobe_ms = now;
                if (probe_axis(a)) {
                    mark_axis_response(a, true);
                    configure_axis(a);
                }
            }
        }
    }
    poll_axis = (uint8_t)((poll_axis + 1) % axis_count);
#endif
}

bool tmc2208_is_connected(void)
{
    if (!port_initialized) return false;
    for (uint8_t i = 0; i < axis_count; i++) {
        if (axes[i].connected) return true;
    }
    return false;
}

/* Recompute steps_remaining + active_direction for the axis from
 * current target_position / current_position / target_velocity, and
 * tell the transport to (re)start step output at the right rate. */
static void apply_motion_command(uint8_t axis)
{
    if (axis >= TMC2208_MAX_AXES) return;
    tmc2208_axis_t* a = &axes[axis];
    if (!a->attached) return;
    const tmc2208_transport_ops_t* t = transport();
    if (!t) return;

    /* Use the absolute value of target_velocity for the rate; sign is
     * an alternate way to express direction. If target_position is
     * set, that takes precedence and determines direction. */
    uint32_t rate_pps = (uint32_t)(a->target_velocity < 0
                                    ? -a->target_velocity
                                    :  a->target_velocity);
    bool forward = (a->target_velocity >= 0);

    int32_t delta = a->target_position - a->current_position;
    if (delta != 0) {
        forward = (delta > 0);
        int32_t abs_delta = delta > 0 ? delta : -delta;
        a->steps_remaining = abs_delta;
        a->active_direction = forward ? 1 : -1;
    } else if (rate_pps > 0) {
        /* Pure velocity command (no position target). Run forever
         * until target_velocity returns to 0 or the operator halts. */
        a->steps_remaining = 0;  /* sentinel: unbounded */
        a->active_direction = forward ? 1 : -1;
    } else {
        /* Stopped. */
        a->steps_remaining = 0;
        a->active_direction = 0;
        rate_pps = 0;
    }

    t->axis_set_rate(axis, rate_pps, forward);
}

bool tmc2208_set_target_position(uint8_t axis, int32_t position)
{
    if (axis >= TMC2208_MAX_AXES) return false;
    if (!port_initialized) {
        saint_log_publish("warn",
            "TMC2208: set_target_position(axis=%u, pos=%ld) — driver not ready",
            (unsigned)axis, (long)position);
        return false;
    }
    axes[axis].target_position = position;

#ifndef SIMULATION
    if (!axes[axis].first_cmd_logged) {
        axes[axis].first_cmd_logged = true;
        saint_log_publish("info",
            "TMC2208 axis %u: first move command → target_position=%ld "
            "(current=%ld)",
            (unsigned)axis, (long)position, (long)axes[axis].current_position);
    }
    apply_motion_command(axis);
#endif
    return true;
}

bool tmc2208_set_target_velocity(uint8_t axis, int32_t pulses_per_sec)
{
    if (axis >= TMC2208_MAX_AXES) return false;
    if (!port_initialized) {
        saint_log_publish("warn",
            "TMC2208: set_target_velocity(axis=%u, pps=%ld) — driver not ready",
            (unsigned)axis, (long)pulses_per_sec);
        return false;
    }
    axes[axis].target_velocity = pulses_per_sec;

#ifndef SIMULATION
    apply_motion_command(axis);
#endif
    return true;
}

bool tmc2208_halt(uint8_t axis)
{
    if (axis >= TMC2208_MAX_AXES) return false;
    axes[axis].target_velocity = 0;
    /* Lock the target to current so steps_remaining stays 0. */
    axes[axis].target_position = axes[axis].current_position;
#ifndef SIMULATION
    apply_motion_command(axis);
#endif
    return true;
}

void tmc2208_halt_all(void)
{
    for (uint8_t i = 0; i < TMC2208_MAX_AXES; i++) {
        (void)tmc2208_halt(i);
    }
}

/* ── peripheral_driver_t glue ───────────────────────────────────── */

static bool drv_init(void)
{
    /* No-op — same pattern as RoboClaw/Tic. Real init runs from drv_load
     * or drv_apply_config when we know which axes exist + pin assignments. */
    return true;
}

static bool drv_set_value(uint8_t channel, float value)
{
    uint8_t axis = channel / TMC2208_CHANNELS_PER_AXIS;
    uint8_t sub  = channel % TMC2208_CHANNELS_PER_AXIS;
    if (axis >= TMC2208_MAX_AXES) return false;
    if (value < -1.0f) value = -1.0f;
    if (value >  1.0f) value =  1.0f;

    switch (sub) {
    case TMC2208_SUB_TARGET_POSITION: {
        int32_t pos = (int32_t)(value * (float)axes[axis].max_position);
        return tmc2208_set_target_position(axis, pos);
    }
    case TMC2208_SUB_TARGET_VELOCITY: {
        int32_t pps = (int32_t)(value * (float)axes[axis].max_speed_pps);
        return tmc2208_set_target_velocity(axis, pps);
    }
    default:
        return false;
    }
}

static bool drv_get_value(uint8_t channel, float* value)
{
    if (!value) return false;
    uint8_t axis = channel / TMC2208_CHANNELS_PER_AXIS;
    uint8_t sub  = channel % TMC2208_CHANNELS_PER_AXIS;
    if (axis >= TMC2208_MAX_AXES) return false;
    tmc2208_axis_t* a = &axes[axis];

    switch (sub) {
    case TMC2208_SUB_TARGET_POSITION:
        *value = (a->max_position == 0) ? 0.0f
            : (float)a->target_position / (float)a->max_position;
        return true;
    case TMC2208_SUB_TARGET_VELOCITY:
        *value = (a->max_speed_pps == 0) ? 0.0f
            : (float)a->target_velocity / (float)a->max_speed_pps;
        return true;
    case TMC2208_SUB_CURRENT_POSITION:
        *value = (float)a->current_position;
        return true;
    case TMC2208_SUB_ERROR_FLAGS:
        /* Surface the low-byte error bits (otpw, ot, s2ga/b, ola/b)
         * as a small integer cast to float. Operator sees the bitset
         * as a value the dashboard can compare to nonzero. */
        *value = (float)(a->error_flags & 0xFFu);
        return true;
    default:
        return false;
    }
}

static void drv_set_defaults(uint8_t channel, pin_config_t* config)
{
    (void)channel;
    config->params.tmc2208.address          = 0;
    config->params.tmc2208.step_pin         = 0xFF;
    config->params.tmc2208.dir_pin          = 0xFF;
    config->params.tmc2208.rsense_milliohm  = TMC2208_DEFAULT_RSENSE_MOHM;
    config->params.tmc2208.microsteps       = TMC2208_DEFAULT_MICROSTEPS;
    config->params.tmc2208.run_current_ma   = TMC2208_DEFAULT_RUN_MA;
    config->params.tmc2208.hold_current_ma  = TMC2208_DEFAULT_HOLD_MA;
    config->params.tmc2208.stealth_chop     = 1;
    config->params.tmc2208.max_position     = TMC2208_DEFAULT_MAX_POSITION;
    config->params.tmc2208.max_speed_pps    = TMC2208_DEFAULT_MAX_SPEED;
}

static bool drv_apply_config(uint8_t channel, const pin_config_t* config)
{
    uint8_t axis = channel / TMC2208_CHANNELS_PER_AXIS;
    if (axis >= TMC2208_MAX_AXES) return false;

    /* step_pin == 0xFF is the "boot-reload zero-init params" fingerprint;
     * trust drv_load's prior fill in that case. */
    if (config->params.tmc2208.step_pin != 0xFF) {
        axes[axis].address          = config->params.tmc2208.address;
        axes[axis].step_pin         = config->params.tmc2208.step_pin;
        axes[axis].dir_pin          = config->params.tmc2208.dir_pin;
        axes[axis].rsense_milliohm  = config->params.tmc2208.rsense_milliohm;
        axes[axis].microsteps       = config->params.tmc2208.microsteps;
        axes[axis].run_current_ma   = config->params.tmc2208.run_current_ma;
        axes[axis].hold_current_ma  = config->params.tmc2208.hold_current_ma;
        axes[axis].stealth_chop     = config->params.tmc2208.stealth_chop;
        axes[axis].max_position     = config->params.tmc2208.max_position;
        axes[axis].max_speed_pps    = config->params.tmc2208.max_speed_pps;
    }

    if (axis >= axis_count) axis_count = axis + 1;
    tmc2208_init();
    return true;
}

static int parse_int_field(const char* json, const char* end, const char* key)
{
    const char* p = strstr(json, key);
    if (!p || p >= end) return -1;
    p = strchr(p, ':');
    if (!p) return -1;
    p++; while (*p == ' ') p++;
    return atoi(p);
}

static bool drv_parse_json(const char* json_start, const char* json_end,
                            pin_config_t* config)
{
    int v;
    if ((v = parse_int_field(json_start, json_end, "\"address\""))           >= 0)
        config->params.tmc2208.address = (uint8_t)v;
    if ((v = parse_int_field(json_start, json_end, "\"step_pin\""))          >= 0)
        config->params.tmc2208.step_pin = (uint8_t)v;
    if ((v = parse_int_field(json_start, json_end, "\"dir_pin\""))           >= 0)
        config->params.tmc2208.dir_pin = (uint8_t)v;
    if ((v = parse_int_field(json_start, json_end, "\"rsense_milliohm\""))   >= 0)
        config->params.tmc2208.rsense_milliohm = (uint16_t)v;
    if ((v = parse_int_field(json_start, json_end, "\"microsteps\""))        >= 0)
        config->params.tmc2208.microsteps = (uint16_t)v;
    if ((v = parse_int_field(json_start, json_end, "\"run_current_ma\""))    >= 0)
        config->params.tmc2208.run_current_ma = (uint16_t)v;
    if ((v = parse_int_field(json_start, json_end, "\"hold_current_ma\""))   >= 0)
        config->params.tmc2208.hold_current_ma = (uint16_t)v;
    if ((v = parse_int_field(json_start, json_end, "\"max_position\""))      >= 0)
        config->params.tmc2208.max_position = (int32_t)v;
    if ((v = parse_int_field(json_start, json_end, "\"max_speed_pps\""))     >= 0)
        config->params.tmc2208.max_speed_pps = (uint16_t)v;

    /* "stealth_chop" — accept bool or int. */
    const char* p = strstr(json_start, "\"stealth_chop\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) {
            p++; while (*p == ' ') p++;
            config->params.tmc2208.stealth_chop =
                (strncmp(p, "true", 4) == 0 || atoi(p) != 0) ? 1 : 0;
        }
    }

    uint8_t tx, rx, inst;
    bool got_pins = uart_pin_pair_parse_json(json_start, json_end, &tx, &rx, &inst);
    if (got_pins) {
        tmc2208_tx_pin = tx;
        tmc2208_rx_pin = rx;
        configured_serial_port = inst;
    }

    /* Log once per sync — first channel of first axis. Same uint8_t
     * truncation trick as Tic (base 348 → (uint8_t)92). */
    if (config->gpio == (uint8_t)TMC2208_VIRTUAL_GPIO_BASE) {
        saint_log_publish("info",
            "TMC2208 sync: pins TX=%u RX=%u (UART%u), axis 0 slave=%u "
            "step=%u dir=%u %u µstep IRUN=%umA IHOLD=%umA Rsense=%umΩ %s",
            (unsigned)tmc2208_tx_pin, (unsigned)tmc2208_rx_pin,
            (unsigned)configured_serial_port,
            (unsigned)config->params.tmc2208.address,
            (unsigned)config->params.tmc2208.step_pin,
            (unsigned)config->params.tmc2208.dir_pin,
            (unsigned)config->params.tmc2208.microsteps,
            (unsigned)config->params.tmc2208.run_current_ma,
            (unsigned)config->params.tmc2208.hold_current_ma,
            (unsigned)config->params.tmc2208.rsense_milliohm,
            config->params.tmc2208.stealth_chop ? "StealthChop" : "SpreadCycle");
    }
    return true;
}

static void drv_estop(void)
{
    /* Stop pulse output + set IRUN to 0 (release motor torque) on
     * every configured axis. Defense in depth — pulse output stops
     * immediately; current cut takes effect within ~one UART frame
     * (~700 µs at 115200 baud). */
    const tmc2208_transport_ops_t* t = transport();
    for (uint8_t i = 0; i < axis_count; i++) {
        axes[i].target_velocity = 0;
        axes[i].target_position = axes[i].current_position;
        axes[i].steps_remaining = 0;
        axes[i].active_direction = 0;
        if (t) t->axis_set_rate(i, 0, true);
#ifndef SIMULATION
        if (t && t->is_open()) {
            uint8_t ihold_cs = tmc2208_current_to_cs(axes[i].hold_current_ma,
                                                      axes[i].rsense_milliohm);
            uint32_t reg = tmc2208_pack_ihold_irun(ihold_cs, /*irun*/ 0, /*delay*/ 0);
            (void)reg_write(axes[i].address, TMC2208_REG_IHOLD_IRUN, reg);
        }
#endif
    }
    saint_log_publish("warn",
        "TMC2208: ESTOP — pulse output halted + IRUN=0 on %u axis%s",
        (unsigned)axis_count, axis_count == 1 ? "" : "es");
}

static bool drv_save(void* storage_ptr)
{
    flash_storage_data_t* storage = (flash_storage_data_t*)storage_ptr;

    memset(&storage->tmc2208_config, 0, sizeof(storage->tmc2208_config));
    storage->tmc2208_config.axis_count = axis_count;
    storage->tmc2208_config.serial_port = configured_serial_port;
    storage->tmc2208_config.baud_rate = configured_baud;

    storage->uart_pins.tmc2208_tx_pin = tmc2208_tx_pin;
    storage->uart_pins.tmc2208_rx_pin = tmc2208_rx_pin;

    for (uint8_t i = 0; i < TMC2208_MAX_AXES; i++) {
        storage->tmc2208_config.axes[i].address          = axes[i].address;
        storage->tmc2208_config.axes[i].step_pin         = axes[i].step_pin;
        storage->tmc2208_config.axes[i].dir_pin          = axes[i].dir_pin;
        storage->tmc2208_config.axes[i].stealth_chop     = axes[i].stealth_chop;
        storage->tmc2208_config.axes[i].microsteps       = axes[i].microsteps;
        storage->tmc2208_config.axes[i].run_current_ma   = axes[i].run_current_ma;
        storage->tmc2208_config.axes[i].hold_current_ma  = axes[i].hold_current_ma;
        storage->tmc2208_config.axes[i].rsense_milliohm  = axes[i].rsense_milliohm;
        storage->tmc2208_config.axes[i].max_speed_pps    = axes[i].max_speed_pps;
        storage->tmc2208_config.axes[i].max_position     = axes[i].max_position;
    }
    return true;
}

static bool drv_load(const void* storage_ptr)
{
    const flash_storage_data_t* storage = (const flash_storage_data_t*)storage_ptr;

    uint8_t stored_tx = storage->uart_pins.tmc2208_tx_pin;
    uint8_t stored_rx = storage->uart_pins.tmc2208_rx_pin;
    if (stored_tx != 0 || stored_rx != 0) {
        uint8_t inst;
        if (uart_pin_pair_lookup(stored_tx, stored_rx, &inst)) {
            tmc2208_tx_pin = stored_tx;
            tmc2208_rx_pin = stored_rx;
            configured_serial_port = inst;
        } else {
            saint_log_publish("warn",
                "TMC2208: stored pin pair TX=%u RX=%u invalid — using defaults",
                (unsigned)stored_tx, (unsigned)stored_rx);
        }
    }

    if (storage->tmc2208_config.axis_count == 0) {
        saint_log_publish("info",
            "TMC2208: no saved peripheral in flash — driver dormant");
        return true;
    }

    uint8_t count = storage->tmc2208_config.axis_count;
    if (count > TMC2208_MAX_AXES) count = TMC2208_MAX_AXES;
    axis_count = count;

    if (storage->tmc2208_config.baud_rate > 0) {
        configured_baud = storage->tmc2208_config.baud_rate;
    }
    configured_serial_port = storage->tmc2208_config.serial_port;

    for (uint8_t i = 0; i < count; i++) {
        axes[i].address          = storage->tmc2208_config.axes[i].address;
        axes[i].step_pin         = storage->tmc2208_config.axes[i].step_pin;
        axes[i].dir_pin          = storage->tmc2208_config.axes[i].dir_pin;
        axes[i].stealth_chop     = storage->tmc2208_config.axes[i].stealth_chop;
        axes[i].microsteps       = storage->tmc2208_config.axes[i].microsteps;
        axes[i].run_current_ma   = storage->tmc2208_config.axes[i].run_current_ma;
        axes[i].hold_current_ma  = storage->tmc2208_config.axes[i].hold_current_ma;
        axes[i].rsense_milliohm  = storage->tmc2208_config.axes[i].rsense_milliohm;
        axes[i].max_speed_pps    = storage->tmc2208_config.axes[i].max_speed_pps;
        axes[i].max_position     = storage->tmc2208_config.axes[i].max_position;
    }

    saint_log_publish("info",
        "TMC2208: restored %u axis config(s) from flash",
        (unsigned)count);

    tmc2208_init();
    return true;
}

static const peripheral_driver_t tmc2208_peripheral = {
    .name              = "tmc2208",
    .mode_string       = "tmc2208_stepper",
    .pin_mode          = PIN_MODE_TMC2208_STEPPER,
    .capability_flag   = PIN_CAP_TMC2208_STEPPER,
    .virtual_gpio_base = TMC2208_VIRTUAL_GPIO_BASE,
    .channel_count          = TMC2208_MAX_CHANNELS,
    .channels_per_instance  = TMC2208_CHANNELS_PER_AXIS,
    .init              = drv_init,
    .update            = tmc2208_update,
    .is_connected      = tmc2208_is_connected,
    .set_value         = drv_set_value,
    .get_value         = drv_get_value,
    .set_defaults      = drv_set_defaults,
    .apply_config      = drv_apply_config,
    .parse_json_params = drv_parse_json,
    .estop             = drv_estop,
    .save_config       = drv_save,
    .load_config       = drv_load,
};

const peripheral_driver_t* tmc2208_get_peripheral_driver(void)
{
    return &tmc2208_peripheral;
}
