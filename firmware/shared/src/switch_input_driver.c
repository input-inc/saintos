/**
 * SAINT.OS Firmware - Generic switch / sensor input driver (shared)
 *
 * See switch_input_driver.h and docs/SENSOR_INPUTS.md.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "switch_input_driver.h"
#include "peripheral_driver.h"
#include "platform.h"
#include "saint_log.h"

/* Poll cadence. The main loop runs at ~2 ms, but there is no reason to
 * hammer an ADC that fast — and the debounce window is far longer than
 * this anyway. A magnet passing a reed at actuator speeds is tens of
 * milliseconds, so 5 ms sampling has ample margin. */
#define SWITCH_POLL_INTERVAL_MS   5

/* Default assert threshold for analog sense, in millivolts. Sits well
 * clear of both the ~1.9 V a conducting 2-wire sensor presents and the
 * 3.3 V of an open circuit. */
#define SWITCH_DEFAULT_THRESHOLD_MV   2500
#define SWITCH_DEFAULT_HYSTERESIS_MV  200
#define SWITCH_DEFAULT_DEBOUNCE_MS    5

/* ── Per-Unit State ─────────────────────────────────────────────── */

typedef struct {
    bool     configured;
    char     peripheral_id[SWITCH_INPUT_TARGET_ID_LEN];

    uint8_t  pin;
    uint8_t  sense;
    bool     active_low;
    bool     pull_up;
    bool     latch;
    uint16_t debounce_ms;
    uint16_t threshold_mv;
    uint16_t hysteresis_mv;
    uint8_t  on_trip;

    char     targets[SWITCH_INPUT_MAX_TARGETS][SWITCH_INPUT_TARGET_ID_LEN];
    /* Which direction of travel this switch blocks on each target —
     * SWITCH_BLOCK_*. Per-target because it describes where the switch
     * sits relative to that axis, which the switch can't know itself. */
    uint8_t  target_block[SWITCH_INPUT_MAX_TARGETS];
    uint8_t  target_count;

    /* Debounce: `raw` is the instantaneous read, `state` only follows it
     * once it has been stable for debounce_ms. */
    bool     raw;
    bool     state;
    uint32_t raw_since_ms;

    bool     latched;
    uint16_t last_mv;
    uint32_t trip_count;
} switch_unit_t;

static switch_unit_t units[SWITCH_INPUT_MAX_UNITS];
static uint8_t  unit_count = 0;
static uint32_t last_poll_ms = 0;

/* Targets arrive in parse_json_params, which has no instance index; the
 * sync then calls apply_config for that instance immediately after. Same
 * parse-then-apply staging the other drivers use for out-of-band fields. */
static char    pending_targets[SWITCH_INPUT_MAX_TARGETS][SWITCH_INPUT_TARGET_ID_LEN];
static uint8_t pending_target_block[SWITCH_INPUT_MAX_TARGETS];
static uint8_t pending_target_count = 0;

/* ── Assert evaluation ──────────────────────────────────────────── */

/* Instantaneous "is it asserted", before debounce.
 *
 * Analog uses hysteresis around the threshold: a sensor sitting near the
 * crossing point would otherwise chatter, and each chatter edge is a
 * trip. The band is applied against the CURRENT state, so it takes a
 * genuine excursion to flip. */
static bool read_raw(switch_unit_t* u)
{
    bool above;

    if (u->sense == SWITCH_SENSE_ANALOG) {
        uint16_t mv = 0;
        if (!switch_input_read_analog_mv(u->pin, &mv)) return u->raw;
        u->last_mv = mv;

        uint16_t hi = u->threshold_mv + u->hysteresis_mv;
        uint16_t lo = (u->threshold_mv > u->hysteresis_mv)
                        ? (uint16_t)(u->threshold_mv - u->hysteresis_mv) : 0;
        /* Only the band edge that would CHANGE the state is tested. */
        above = u->raw ? (mv > lo) : (mv >= hi);
    } else {
        above = switch_input_read_digital_pin(u->pin, u->pull_up);
    }

    /* active_low means the asserted condition is "low"/"below". For a
     * normally-closed sensor the asserted condition is the contact
     * OPENING — which also means a cut cable reads as asserted, the
     * correct fail-safe direction. */
    return u->active_low ? !above : above;
}

/* Fire the local interlock. Uses the peripheral_command routing rather
 * than the estop() vtable entry because estop() is driver-WIDE: it stops
 * every unit that driver owns, so a limit guarding one axis would take
 * out every Kangaroo on the node. Routing by peripheral_id gives
 * per-instance granularity. See docs/SENSOR_INPUTS.md. */
static void fire_trip(switch_unit_t* u)
{
    if (u->on_trip == SWITCH_TRIP_ESTOP_NODE) {
        saint_log_publish("warn",
            "switch '%s' TRIPPED — e-stopping every peripheral on this node",
            u->peripheral_id);
        peripheral_estop_all();
        return;
    }
    if (u->on_trip != SWITCH_TRIP_STOP_TARGETS) return;

    if (u->target_count == 0) {
        saint_log_publish("warn",
            "switch '%s' tripped with stop-targets selected but no targets "
            "configured — nothing was stopped", u->peripheral_id);
        return;
    }
    for (uint8_t i = 0; i < u->target_count; i++) {
        /* Hand the blocked direction to the target so it can refuse
         * motion INTO the switch while still allowing a retreat. Numeric
         * so the receiving driver's parse stays trivial — there is no
         * JSON parser on the MCU. */
        char args[24];
        int n = snprintf(args, sizeof(args), "{\"block\":%u}",
                         (unsigned)u->target_block[i]);
        const char* a_end = (n > 0 && (size_t)n < sizeof(args))
                              ? args + n : NULL;
        bool ok = peripheral_dispatch_command(u->targets[i], "estop",
                                              a_end ? args : NULL, a_end);
        const char* dirtext =
            u->target_block[i] == SWITCH_BLOCK_POSITIVE ? "extend/forward" :
            u->target_block[i] == SWITCH_BLOCK_NEGATIVE ? "retract/reverse" :
                                                          "both directions";
        saint_log_publish(ok ? "warn" : "error",
            ok ? "switch '%s' TRIPPED — blocked %s on '%s'"
               : "switch '%s' TRIPPED — could NOT stop '%s' (no driver "
                 "claimed it, or it has no estop command) [wanted %s]",
            u->peripheral_id,
            ok ? dirtext : u->targets[i],
            ok ? u->targets[i] : dirtext);
    }
}

static void poll_unit(uint8_t idx, uint32_t now)
{
    switch_unit_t* u = &units[idx];
    if (!u->configured) return;

    bool raw = read_raw(u);
    if (raw != u->raw) {
        u->raw = raw;
        u->raw_since_ms = now;
        return;                     /* start the debounce window */
    }
    if ((uint32_t)(now - u->raw_since_ms) < u->debounce_ms) return;
    if (raw == u->state) return;    /* stable, and already reflected */

    u->state = raw;

    if (!raw) return;               /* releasing is not a trip */

    u->trip_count++;
    if (u->latch) u->latched = true;
    fire_trip(u);
}

/* ── Public API ─────────────────────────────────────────────────── */

void switch_input_init(void)
{
    last_poll_ms = PLATFORM_MILLIS();
    /* Seed each input from its current level so a switch that is ALREADY
     * asserted at boot is seen as asserted, rather than waiting for an
     * edge that may never come. Seeded through the debounce window
     * directly — this is a known-good starting state, not a transition. */
    for (uint8_t i = 0; i < unit_count; i++) {
        if (!units[i].configured) continue;
        bool raw = read_raw(&units[i]);
        units[i].raw = raw;
        units[i].state = raw;
        units[i].raw_since_ms = last_poll_ms;
        if (raw) {
            if (units[i].latch) units[i].latched = true;
            saint_log_publish("warn",
                "switch '%s' is ALREADY asserted at startup",
                units[i].peripheral_id);
        }
    }
}

void switch_input_update(void)
{
    if (unit_count == 0) return;
    uint32_t now = PLATFORM_MILLIS();
    if ((uint32_t)(now - last_poll_ms) < SWITCH_POLL_INTERVAL_MS) return;
    last_poll_ms = now;

    for (uint8_t i = 0; i < unit_count; i++) poll_unit(i, now);
}

bool switch_input_clear_latch(uint8_t unit)
{
    if (unit >= SWITCH_INPUT_MAX_UNITS || !units[unit].configured) return false;
    if (!units[unit].latched) return true;

    /* Refuse while still physically asserted. Clearing here would
     * re-latch on the very next poll, which looks like a dead button —
     * far better to say why. */
    if (units[unit].state) {
        saint_log_publish("warn",
            "switch '%s' clear ignored — still asserted. Move the mechanism "
            "off the switch first.", units[unit].peripheral_id);
        return false;
    }

    units[unit].latched = false;
    saint_log_publish("info", "switch '%s' latch cleared",
        units[unit].peripheral_id);
    return true;
}

bool switch_input_any_latched(void)
{
    for (uint8_t i = 0; i < unit_count; i++) {
        if (units[i].configured && units[i].latched) return true;
    }
    return false;
}

/* ── peripheral_driver_t glue ───────────────────────────────────── */

static bool drv_init(void)
{
    switch_input_init();
    return true;
}

static bool drv_get_value(uint8_t channel, float* value)
{
    if (!value) return false;
    uint8_t unit = channel / SWITCH_INPUT_CHANNELS_PER_UNIT;
    uint8_t sub  = channel % SWITCH_INPUT_CHANNELS_PER_UNIT;
    if (unit >= SWITCH_INPUT_MAX_UNITS) return false;

    switch (sub) {
    case SWITCH_SUB_STATE:      *value = units[unit].state ? 1.0f : 0.0f;   return true;
    case SWITCH_SUB_LATCHED:    *value = units[unit].latched ? 1.0f : 0.0f; return true;
    case SWITCH_SUB_VOLTAGE:    *value = (float)units[unit].last_mv / 1000.0f; return true;
    case SWITCH_SUB_TRIP_COUNT: *value = (float)units[unit].trip_count;     return true;
    default: return false;
    }
}

/* Every channel is read-only — a sensor input has nothing to command. */
static bool drv_set_value(uint8_t channel, float value)
{
    (void)channel; (void)value;
    return false;
}

static void drv_set_defaults(uint8_t channel, pin_config_t* config)
{
    (void)channel;
    config->params.switch_input.sense         = SWITCH_SENSE_DIGITAL;
    config->params.switch_input.active_low    = 1;
    config->params.switch_input.pull_up       = 1;
    config->params.switch_input.latch         = 1;
    config->params.switch_input.debounce_ms   = SWITCH_DEFAULT_DEBOUNCE_MS;
    config->params.switch_input.threshold_mv  = SWITCH_DEFAULT_THRESHOLD_MV;
    config->params.switch_input.hysteresis_mv = SWITCH_DEFAULT_HYSTERESIS_MV;
    config->params.switch_input.on_trip       = SWITCH_TRIP_NONE;
}

static bool drv_apply_config(uint8_t channel, const pin_config_t* config)
{
    uint8_t unit = channel / SWITCH_INPUT_CHANNELS_PER_UNIT;
    if (unit >= SWITCH_INPUT_MAX_UNITS) return false;
    switch_unit_t* u = &units[unit];

    u->pin           = config->gpio;
    u->sense         = config->params.switch_input.sense;
    u->active_low    = config->params.switch_input.active_low != 0;
    u->pull_up       = config->params.switch_input.pull_up != 0;
    u->latch         = config->params.switch_input.latch != 0;
    u->debounce_ms   = config->params.switch_input.debounce_ms;
    u->threshold_mv  = config->params.switch_input.threshold_mv
                        ? config->params.switch_input.threshold_mv
                        : SWITCH_DEFAULT_THRESHOLD_MV;
    u->hysteresis_mv = config->params.switch_input.hysteresis_mv;
    u->on_trip       = config->params.switch_input.on_trip;

    if (config->logical_name[0]) {
        strncpy(u->peripheral_id, config->logical_name,
                sizeof(u->peripheral_id) - 1);
        u->peripheral_id[sizeof(u->peripheral_id) - 1] = '\0';
    }

    /* Targets staged by the preceding parse_json_params for this entry. */
    u->target_count = pending_target_count;
    for (uint8_t i = 0; i < pending_target_count; i++) {
        memcpy(u->targets[i], pending_targets[i], SWITCH_INPUT_TARGET_ID_LEN);
        u->target_block[i] = pending_target_block[i];
    }
    pending_target_count = 0;

    u->configured = true;
    if (unit >= unit_count) unit_count = unit + 1;

    /* Re-seed: the pin or polarity may have changed under us. */
    u->raw = u->state = read_raw(u);
    u->raw_since_ms = PLATFORM_MILLIS();

    saint_log_publish("info",
        "switch '%s' on pin %u — %s sense, %s, debounce %u ms, on-trip %s "
        "(%u target%s)",
        u->peripheral_id, (unsigned)u->pin,
        u->sense == SWITCH_SENSE_ANALOG ? "analog" : "digital",
        u->active_low ? "active low" : "active high",
        (unsigned)u->debounce_ms,
        u->on_trip == SWITCH_TRIP_ESTOP_NODE   ? "estop-node" :
        u->on_trip == SWITCH_TRIP_STOP_TARGETS ? "stop-targets" : "none",
        (unsigned)u->target_count, u->target_count == 1 ? "" : "s");
    return true;
}

/* Pull a numeric field out of the params object. Hand-rolled for the
 * same reason every other driver here does it: no JSON parser on the
 * MCU. */
static bool json_int(const char* start, const char* end,
                     const char* key, long* out)
{
    char pat[40];
    int n = snprintf(pat, sizeof(pat), "\"%s\"", key);
    if (n <= 0 || (size_t)n >= sizeof(pat)) return false;
    const char* p = strstr(start, pat);
    if (!p || p >= end) return false;
    p = strchr(p + n, ':');
    if (!p || p >= end) return false;
    p++;
    while (*p == ' ' || *p == '"') p++;
    if (*p == 't' || *p == 'T') { *out = 1; return true; }   /* true  */
    if (*p == 'f' || *p == 'F') { *out = 0; return true; }   /* false */
    *out = atol(p);
    return true;
}

static bool drv_parse_json(const char* json_start, const char* json_end,
                           pin_config_t* config)
{
    long v;

    if (json_int(json_start, json_end, "sense_analog", &v))
        config->params.switch_input.sense =
            v ? SWITCH_SENSE_ANALOG : SWITCH_SENSE_DIGITAL;
    if (json_int(json_start, json_end, "active_low", &v))
        config->params.switch_input.active_low = v ? 1 : 0;
    if (json_int(json_start, json_end, "pull_up", &v))
        config->params.switch_input.pull_up = v ? 1 : 0;
    if (json_int(json_start, json_end, "latch", &v))
        config->params.switch_input.latch = v ? 1 : 0;
    if (json_int(json_start, json_end, "debounce_ms", &v)) {
        if (v < 0)    v = 0;
        if (v > 5000) v = 5000;
        config->params.switch_input.debounce_ms = (uint16_t)v;
    }
    if (json_int(json_start, json_end, "threshold_mv", &v)) {
        if (v < 0)     v = 0;
        if (v > 65535) v = 65535;
        config->params.switch_input.threshold_mv = (uint16_t)v;
    }
    if (json_int(json_start, json_end, "hysteresis_mv", &v)) {
        if (v < 0)     v = 0;
        if (v > 65535) v = 65535;
        config->params.switch_input.hysteresis_mv = (uint16_t)v;
    }
    if (json_int(json_start, json_end, "on_trip", &v)) {
        if (v < 0 || v > SWITCH_TRIP_ESTOP_NODE) v = SWITCH_TRIP_NONE;
        config->params.switch_input.on_trip = (uint8_t)v;
    }

    /* targets: ["kangaroo-1:+","roboclaw-2:-"] — staged for the
     * apply_config that follows this parse for the same entry.
     *
     * The ":<dir>" suffix is which direction of travel this switch
     * blocks on that target: "+" extend/forward, "-" retract/reverse,
     * anything else (including no suffix) means both. Both is the
     * conservative decode — over-blocking is recoverable, a wrong
     * direction drives further into the switch. */
    pending_target_count = 0;
    const char* p = strstr(json_start, "\"targets\"");
    if (p && p < json_end) {
        p = strchr(p, '[');
        if (p && p < json_end) {
            const char* q = p;
            while (pending_target_count < SWITCH_INPUT_MAX_TARGETS) {
                q = strchr(q + 1, '"');
                if (!q || q >= json_end) break;
                const char* s = q + 1;
                const char* e = strchr(s, '"');
                if (!e || e >= json_end) break;
                size_t len = (size_t)(e - s);
                /* Split the ":<dir>" suffix off the id. */
                uint8_t block = SWITCH_BLOCK_BOTH;
                const char* colon = NULL;
                for (const char* c = s; c < e; c++) {
                    if (*c == ':') { colon = c; break; }
                }
                if (colon) {
                    if (colon + 1 < e && *(colon + 1) == '+')
                        block = SWITCH_BLOCK_POSITIVE;
                    else if (colon + 1 < e && *(colon + 1) == '-')
                        block = SWITCH_BLOCK_NEGATIVE;
                    len = (size_t)(colon - s);
                }
                if (len >= SWITCH_INPUT_TARGET_ID_LEN)
                    len = SWITCH_INPUT_TARGET_ID_LEN - 1;
                memcpy(pending_targets[pending_target_count], s, len);
                pending_targets[pending_target_count][len] = '\0';
                pending_target_block[pending_target_count] = block;
                pending_target_count++;
                q = e;
                /* Stop at the closing bracket rather than running on
                 * into the next param's strings. */
                const char* close = strchr(e, ']');
                const char* nextq = strchr(e + 1, '"');
                if (!nextq || (close && close < nextq)) break;
            }
        }
    }
    return true;
}

static bool drv_command(const char* peripheral_id, const char* command,
                        const char* args_json, const char* args_json_end)
{
    (void)args_json; (void)args_json_end;
    if (!peripheral_id || !command) return false;

    for (uint8_t i = 0; i < unit_count; i++) {
        if (!units[i].configured) continue;
        if (strcmp(units[i].peripheral_id, peripheral_id) != 0) continue;

        if (strcmp(command, "clear_latch") == 0) {
            (void)switch_input_clear_latch(i);
            return true;
        }
        saint_log_publish("warn",
            "switch '%s' ignoring unknown command '%s'",
            peripheral_id, command);
        return true;
    }
    return false;
}

/* A sensor has no outputs to stop, and clearing its latch on e-stop
 * would destroy the very evidence of why the node stopped. */
static void drv_estop(void) { }

static int drv_state_emit(char* buf, size_t cap, bool* first)
{
    int total = 0;
    for (uint8_t i = 0; i < unit_count; i++) {
        if (!units[i].configured || !units[i].peripheral_id[0]) continue;
        struct { const char* id; float v; } recs[] = {
            { "state",      units[i].state   ? 1.0f : 0.0f },
            { "latched",    units[i].latched ? 1.0f : 0.0f },
            { "trip_count", (float)units[i].trip_count },
        };
        for (size_t r = 0; r < sizeof(recs) / sizeof(recs[0]); r++) {
            int n = peripheral_state_append_channel(
                buf + total, cap - (size_t)total, first,
                units[i].peripheral_id, recs[r].id, recs[r].v);
            if (n < 0) return -1;
            total += n;
        }
        if (units[i].sense == SWITCH_SENSE_ANALOG) {
            int n = peripheral_state_append_channel(
                buf + total, cap - (size_t)total, first,
                units[i].peripheral_id, "voltage",
                (float)units[i].last_mv / 1000.0f);
            if (n < 0) return -1;
            total += n;
        }
    }
    return total;
}

static const peripheral_driver_t switch_input_peripheral = {
    .name              = "switch_input",
    .mode_string       = "switch_input",
    .pin_mode          = PIN_MODE_SWITCH_INPUT,
    .capability_flag   = PIN_CAP_SWITCH_INPUT,
    .virtual_gpio_base = SWITCH_INPUT_VIRTUAL_GPIO_BASE,
    .channel_count         = SWITCH_INPUT_MAX_CHANNELS,
    .channels_per_instance = SWITCH_INPUT_CHANNELS_PER_UNIT,
    .init              = drv_init,
    .update            = switch_input_update,
    .is_connected      = NULL,
    .set_value         = drv_set_value,
    .get_value         = drv_get_value,
    .set_defaults      = drv_set_defaults,
    .apply_config      = drv_apply_config,
    .parse_json_params = drv_parse_json,
    .command           = drv_command,
    .estop             = drv_estop,
    .save_config       = NULL,
    .load_config       = NULL,
    .state_emit_channels = drv_state_emit,
};

const peripheral_driver_t* switch_input_get_peripheral_driver(void)
{
    return &switch_input_peripheral;
}
