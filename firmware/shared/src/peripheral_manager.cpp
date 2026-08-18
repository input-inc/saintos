/**
 * SAINT.OS Firmware - Peripheral Manager
 *
 * Registration and dispatch for modular peripheral drivers.
 */

#include "peripheral_driver.h"
#include "platform.h"
#include "saint_log.h"
#include <stdio.h>
#include <string.h>

// =============================================================================
// State
// =============================================================================

static const peripheral_driver_t* drivers[PERIPHERAL_MAX_DRIVERS];
static uint8_t driver_count = 0;

// =============================================================================
// Registration
// =============================================================================

bool peripheral_register(const peripheral_driver_t* driver)
{
    if (!driver || driver_count >= PERIPHERAL_MAX_DRIVERS) {
        return false;
    }

    drivers[driver_count++] = driver;
    PLATFORM_PRINTF("Peripheral: registered '%s' (GPIO %d-%d, %d channels)\n",
                    driver->name,
                    driver->virtual_gpio_base,
                    driver->virtual_gpio_base + driver->channel_count - 1,
                    driver->channel_count);
    return true;
}

// =============================================================================
// Lifecycle
// =============================================================================

void peripheral_init_all(void)
{
    for (uint8_t i = 0; i < driver_count; i++) {
        if (drivers[i]->init) {
            drivers[i]->init();
        }
    }
}

void peripheral_update_all(void)
{
    for (uint8_t i = 0; i < driver_count; i++) {
        if (drivers[i]->update) {
            drivers[i]->update();
        }
    }
}

void peripheral_estop_all(void)
{
    for (uint8_t i = 0; i < driver_count; i++) {
        if (drivers[i]->estop) {
            drivers[i]->estop();
        }
    }
}

void peripheral_clear_estop_all(void)
{
    for (uint8_t i = 0; i < driver_count; i++) {
        if (drivers[i]->clear_estop) {
            drivers[i]->clear_estop();
        }
    }
}

bool peripheral_dispatch_command(const char* peripheral_id,
                                 const char* command,
                                 const char* args_json,
                                 const char* args_json_end)
{
    if (!peripheral_id || !command) return false;

    for (uint8_t i = 0; i < driver_count; i++) {
        if (!drivers[i]->command) continue;
        if (drivers[i]->command(peripheral_id, command,
                                args_json, args_json_end)) {
            return true;
        }
    }
    // Not a silent drop: an unroutable command means the operator's UI
    // and the node disagree about what is configured, and that is worth
    // seeing in the log rather than debugging as "the button does
    // nothing."
    saint_log_publish("warn",
        "peripheral_command: no driver claimed peripheral '%s' for "
        "command '%s'", peripheral_id, command);
    return false;
}

// Extract a JSON string value for `key` into `out`. Hand-rolled: there
// is no JSON parser on the MCU and every driver here already parses its
// own params the same way.
static bool json_get_string(const char* json, const char* key,
                            char* out, size_t out_cap)
{
    char pat[32];
    int n = snprintf(pat, sizeof(pat), "\"%s\"", key);
    if (n <= 0 || (size_t)n >= sizeof(pat)) return false;

    const char* p = strstr(json, pat);
    if (!p) return false;
    p = strchr(p + n, ':');
    if (!p) return false;
    p++;
    while (*p == ' ' || *p == '\t') p++;
    if (*p != '"') return false;
    p++;

    size_t i = 0;
    while (*p && *p != '"' && i < out_cap - 1) out[i++] = *p++;
    if (*p != '"') return false;      // truncated or unterminated
    out[i] = '\0';
    return i > 0;
}

// Locate the {...} object following "args". Returns false if absent —
// commands without arguments are legitimate.
static bool json_find_object(const char* json, const char* key,
                             const char** start, const char** end)
{
    char pat[32];
    int n = snprintf(pat, sizeof(pat), "\"%s\"", key);
    if (n <= 0 || (size_t)n >= sizeof(pat)) return false;

    const char* p = strstr(json, pat);
    if (!p) return false;
    p = strchr(p + n, ':');
    if (!p) return false;
    p++;
    while (*p == ' ' || *p == '\t') p++;
    if (*p != '{') return false;

    // Depth-count to the matching brace, skipping braces inside strings
    // so a string value can't truncate the object early.
    int depth = 0;
    bool in_string = false, escaped = false;
    const char* q = p;
    for (; *q; q++) {
        if (in_string) {
            if (escaped)        escaped = false;
            else if (*q == '\\') escaped = true;
            else if (*q == '"')  in_string = false;
            continue;
        }
        if (*q == '"') { in_string = true; continue; }
        if (*q == '{') depth++;
        else if (*q == '}') {
            if (--depth == 0) { *start = p; *end = q + 1; return true; }
        }
    }
    return false;
}

bool peripheral_command_handle_json(const char* json)
{
    if (!json) return false;

    char peripheral_id[32];
    char command[32];
    if (!json_get_string(json, "peripheral", peripheral_id, sizeof(peripheral_id))) {
        saint_log_publish("warn",
            "peripheral_command: missing or malformed 'peripheral' field");
        return false;
    }
    if (!json_get_string(json, "command", command, sizeof(command))) {
        saint_log_publish("warn",
            "peripheral_command: '%s' sent no 'command' field", peripheral_id);
        return false;
    }

    const char* args = NULL;
    const char* args_end = NULL;
    (void)json_find_object(json, "args", &args, &args_end);

    return peripheral_dispatch_command(peripheral_id, command, args, args_end);
}

// =============================================================================
// Lookup
// =============================================================================

const peripheral_driver_t* peripheral_find_by_gpio(uint16_t gpio)
{
    for (uint8_t i = 0; i < driver_count; i++) {
        uint16_t base = drivers[i]->virtual_gpio_base;
        uint16_t end = base + drivers[i]->channel_count;
        if (gpio >= base && gpio < end) {
            return drivers[i];
        }
    }
    return NULL;
}

const peripheral_driver_t* peripheral_find_by_mode(pin_mode_t mode)
{
    for (uint8_t i = 0; i < driver_count; i++) {
        if (drivers[i]->pin_mode == mode) {
            return drivers[i];
        }
    }
    return NULL;
}

const peripheral_driver_t* peripheral_find_by_mode_string(const char* mode_str)
{
    if (!mode_str) return NULL;
    for (uint8_t i = 0; i < driver_count; i++) {
        if (drivers[i]->mode_string && strcmp(drivers[i]->mode_string, mode_str) == 0) {
            return drivers[i];
        }
    }
    return NULL;
}

uint8_t peripheral_gpio_to_channel(const peripheral_driver_t* drv, uint16_t gpio)
{
    if (!drv) return 0xFF;
    return (uint8_t)(gpio - drv->virtual_gpio_base);
}

bool peripheral_is_virtual_gpio(uint16_t gpio)
{
    return peripheral_find_by_gpio(gpio) != NULL;
}

// =============================================================================
// Iteration
// =============================================================================

uint8_t peripheral_get_count(void)
{
    return driver_count;
}

// =============================================================================
// Channel-addressed state emission (peripheral-first migration; Phase 1)
// =============================================================================
//
// Each migrated driver implements `state_emit_channels(buf, cap, first)`
// to append zero-or-more `{"peripheral_id":..,"channel_id":..,"value":..}`
// records to a shared `channels[]` array in the outbound state JSON.
// The helpers below let drivers stay decoupled from the array bookkeeping:
// they just call peripheral_state_append_channel() per record, and the
// `first` flag handed in (shared across every driver this tick) tracks
// whether to prefix a comma.
//
// See docs/PERIPHERAL_FIRST_MIGRATION.md for the broader plan; this is
// the entry point Phase 1 (per-driver migration) hooks into.

int peripheral_state_append_channel(char* buf, size_t cap, bool* first,
                                    const char* peripheral_id,
                                    const char* channel_id,
                                    float value)
{
    if (!buf || !first || !peripheral_id || !channel_id) return -1;
    int n = snprintf(buf, cap,
                     "%s{\"peripheral_id\":\"%s\",\"channel_id\":\"%s\",\"value\":%.4f}",
                     *first ? "" : ",",
                     peripheral_id, channel_id, (double)value);
    if (n < 0 || (size_t)n >= cap) return -1;
    *first = false;
    return n;
}

int peripheral_state_emit_all_channels(char* buf, size_t cap)
{
    if (!buf) return -1;
    bool first = true;
    int total = 0;
    for (uint8_t i = 0; i < driver_count; i++) {
        if (!drivers[i] || !drivers[i]->state_emit_channels) continue;
        int n = drivers[i]->state_emit_channels(buf + total, cap - (size_t)total, &first);
        if (n < 0) return -1;
        total += n;
    }
    return total;
}

const peripheral_driver_t* peripheral_get(uint8_t index)
{
    if (index >= driver_count) return NULL;
    return drivers[index];
}
