/**
 * SAINT.OS Node Firmware - Pin Configuration Implementation
 *
 * Manages GPIO pin capabilities, runtime configuration, and persistence.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

#include "pin_config.h"
#include "pin_control_types.h"
#include "flash_storage.h"
#include "saint_node.h"
#include "peripheral_driver.h"
#include "saint_log.h"

// =============================================================================
// Static Variables
// =============================================================================
//
// Available + reserved pin tables used to live here as static arrays
// hardcoded for the Feather RP2040 + Ethernet FeatherWing. They now
// live in server/config/boards/rp2040/feather_rp2040_w5500.yaml on
// the server, and the server validates peripheral assignments against
// that file before pushing the configure message. The firmware just
// trusts the incoming peripheral list and wires the hardware.

// Current pin configurations (runtime state)
static pin_config_t pin_configs[PIN_CONFIG_MAX_PINS];
static uint8_t pin_config_count = 0;

static bool initialized = false;

// =============================================================================
// Mode String Mapping
// =============================================================================

static const char* mode_strings[] = {
    [PIN_MODE_UNCONFIGURED] = "unconfigured",
    [PIN_MODE_DIGITAL_IN]   = "digital_in",
    [PIN_MODE_DIGITAL_OUT]  = "digital_out",
    [PIN_MODE_PWM]          = "pwm",
    [PIN_MODE_SERVO]        = "servo",
    [PIN_MODE_ADC]          = "adc",
    [PIN_MODE_I2C_SDA]      = "i2c_sda",
    [PIN_MODE_I2C_SCL]      = "i2c_scl",
    [PIN_MODE_UART_TX]      = "uart_tx",
    [PIN_MODE_UART_RX]      = "uart_rx",
    [PIN_MODE_RESERVED]     = "reserved",
    [PIN_MODE_MAESTRO_SERVO] = "maestro_servo",
    [PIN_MODE_SYREN_MOTOR]   = "syren_motor",
};

#define MODE_STRING_COUNT (sizeof(mode_strings) / sizeof(mode_strings[0]))

// =============================================================================
// Helper Functions
// =============================================================================

/**
 * Find or create config entry for GPIO.
 */
static pin_config_t* find_or_create_config(uint8_t gpio)
{
    // Check if already exists
    for (uint8_t i = 0; i < pin_config_count; i++) {
        if (pin_configs[i].gpio == gpio) {
            return &pin_configs[i];
        }
    }

    // Create new entry if space available
    if (pin_config_count < PIN_CONFIG_MAX_PINS) {
        pin_config_t* cfg = &pin_configs[pin_config_count++];
        memset(cfg, 0, sizeof(pin_config_t));
        cfg->gpio = gpio;
        cfg->mode = PIN_MODE_UNCONFIGURED;
        return cfg;
    }

    return NULL;
}

// =============================================================================
// Public API Implementation
// =============================================================================

void pin_config_init(void)
{
    if (initialized) return;

    memset(pin_configs, 0, sizeof(pin_configs));
    pin_config_count = 0;

    // Initialize ADC hardware
    adc_init();

    printf("Pin config: initialized\n");

    initialized = true;
}

const pin_config_t* pin_config_get(uint8_t gpio)
{
    for (uint8_t i = 0; i < pin_config_count; i++) {
        if (pin_configs[i].gpio == gpio) {
            return &pin_configs[i];
        }
    }
    return NULL;
}

const pin_config_t* pin_config_get_all(uint8_t* count)
{
    if (count) {
        *count = pin_config_count;
    }
    return pin_configs;
}

// =============================================================================
// Type-id → driver-mode lookup
// =============================================================================
//
// The server now emits peripheral `type` ids (e.g. "fas100"); each one
// maps to an existing driver registered by mode_string (e.g.
// "fas100_sensor"). New peripheral types added to the server catalog
// must also be listed here with the matching firmware driver.

static const struct {
    const char* type_id;
    const char* mode_string;
} k_peripheral_type_map[] = {
    {"fas100",         "fas100_sensor"},
    {"syren",          "syren_motor"},
    {"roboclaw",       "roboclaw_motor"},
    {"pathfinder_bms", "pathfinder_bms_sensor"},
    {"tic",            "tic_stepper"},
    {"tmc2208",        "tmc2208_stepper"},
};

static const peripheral_driver_t* driver_for_type_id(const char* type_id)
{
    if (!type_id || !*type_id) return NULL;
    for (size_t i = 0; i < sizeof(k_peripheral_type_map)/sizeof(k_peripheral_type_map[0]); i++) {
        if (strcmp(k_peripheral_type_map[i].type_id, type_id) == 0) {
            return peripheral_find_by_mode_string(k_peripheral_type_map[i].mode_string);
        }
    }
    return NULL;
}

// Pull a quoted-string field value out of a JSON range. Returns true
// on success; out is null-terminated and at most out_size-1 chars long.
static bool extract_string_field(const char* start, const char* end,
                                 const char* key, char* out, size_t out_size)
{
    const char* p = strstr(start, key);
    if (!p || p >= end) return false;
    p = strchr(p, ':');
    if (!p || p >= end) return false;
    p++;
    while (p < end && (*p == ' ' || *p == '\t' || *p == '"')) p++;
    size_t n = 0;
    while (p < end && *p != '"' && n < out_size - 1) {
        out[n++] = *p++;
    }
    out[n] = '\0';
    return n > 0;
}

// Apply a single peripheral entry: instantiate the firmware driver's
// virtual GPIO channels and let the driver pull its pins + params out
// of `obj_start..obj_end`.
//
// Each peripheral instance gets exactly `drv->channels_per_instance`
// channels — not the driver's full slab. Multiple instances of the
// same driver stack consecutively within the slab; the instance slot
// is derived from how many entries with this mode already exist in
// pin_config (which `apply_peripherals_json` resets before calling us).
// This way the firmware no longer pre-claims a fixed grid of "up to N
// units" — each node carries exactly what it has.
static bool apply_one_peripheral(const char* obj_start, const char* obj_end)
{
    char type_id[32];
    if (!extract_string_field(obj_start, obj_end, "\"type\"", type_id, sizeof(type_id))) {
        saint_log_publish("warn", "Peripheral missing 'type' — skipping");
        return false;
    }

    const peripheral_driver_t* drv = driver_for_type_id(type_id);
    if (!drv) {
        saint_log_publish("warn", "No driver for type '%s' — skipping", type_id);
        return false;
    }
    if (drv->channels_per_instance == 0) {
        saint_log_publish("error", "Driver '%s' has channels_per_instance=0", type_id);
        return false;
    }

    // Optional friendly id used as logical_name on every channel.
    char inst_id[32];
    inst_id[0] = '\0';
    extract_string_field(obj_start, obj_end, "\"id\"", inst_id, sizeof(inst_id));

    // Count existing pin_config entries with this driver's mode to
    // determine which instance slot we're filling. Stateless across
    // apply runs because apply_peripherals_json calls pin_config_reset
    // first.
    uint8_t existing = 0;
    const pin_config_t* existing_configs = pin_config_get_all(&existing);
    uint8_t used_channels = 0;
    for (uint8_t i = 0; i < existing; i++) {
        if (existing_configs[i].mode == drv->pin_mode) used_channels++;
    }
    uint8_t inst_idx = used_channels / drv->channels_per_instance;
    uint8_t base_channel = (uint8_t)(inst_idx * drv->channels_per_instance);

    if ((uint16_t)base_channel + drv->channels_per_instance > drv->channel_count) {
        saint_log_publish("warn",
            "Driver '%s' instance %u exceeds slab (%u channels) — skipping",
            type_id, (unsigned)inst_idx, (unsigned)drv->channel_count);
        return false;
    }
    // pin_config_t.gpio is uint8_t. When virtual_gpio_base + ch
    // exceeds 255, vgpio wraps into the low-numbered range and can
    // alias a real-GPIO config entry. We don't refuse the allocation
    // (Pathfinder BMS at base 276 has always wrapped — relocating its
    // base needs a coordinated server-side change and is tracked
    // separately) but we surface the danger.
    if ((uint16_t)drv->virtual_gpio_base + base_channel
            + drv->channels_per_instance > 256) {
        saint_log_publish("warn",
            "Driver '%s' instance %u wraps GPIO range "
            "(base=%u + ch=%u + %u > 256) — may alias real GPIOs",
            type_id, (unsigned)inst_idx,
            (unsigned)drv->virtual_gpio_base,
            (unsigned)base_channel,
            (unsigned)drv->channels_per_instance);
    }

    for (uint8_t sub = 0; sub < drv->channels_per_instance; sub++) {
        uint8_t ch = (uint8_t)(base_channel + sub);
        uint8_t vgpio = (uint8_t)(drv->virtual_gpio_base + ch);
        if (!pin_config_set(vgpio, drv->pin_mode, inst_id)) {
            // pin_config_set already logged the reason
            continue;
        }
        pin_config_t* pcfg = find_or_create_config(vgpio);
        if (!pcfg) continue;
        if (drv->set_defaults) drv->set_defaults(ch, pcfg);
        if (drv->parse_json_params) drv->parse_json_params(obj_start, obj_end, pcfg);
    }
    return true;
}

// Walk a "peripherals":[ ... ] array and apply each entry.
static bool apply_peripherals_json(const char* arr_start, const char* json_end)
{
    if (!arr_start || arr_start >= json_end) return false;

    pin_config_reset();

    const char* p = arr_start;
    while (p < json_end && *p != ']') {
        // Skip whitespace + commas
        while (p < json_end && (*p == ' ' || *p == ',' || *p == '\n' ||
                                 *p == '\r' || *p == '\t')) p++;
        if (p >= json_end || *p == ']' || *p == '\0') break;
        if (*p != '{') break;

        // Find this peripheral's matching '}'
        const char* obj_start = p;
        int depth = 1;
        const char* q = obj_start + 1;
        while (q < json_end && depth > 0) {
            if (*q == '{') depth++;
            else if (*q == '}') depth--;
            q++;
        }
        if (depth != 0) break;

        apply_one_peripheral(obj_start, q);
        p = q;
    }

    pin_config_apply_hardware();
    return true;
}

bool pin_config_apply_json(const char* json, size_t json_len)
{
    if (!json || json_len == 0) return false;

    // SAFETY: Validate string is null-terminated within json_len
    bool found_null = false;
    for (size_t i = 0; i <= json_len && i < 4096; i++) {
        if (json[i] == '\0') {
            found_null = true;
            break;
        }
    }
    if (!found_null) {
        printf("Pin config: JSON not null-terminated within length\n");
        return false;
    }

    // Check for "configure" action
    const char* action = strstr(json, "\"action\"");
    if (!action || !strstr(action, "\"configure\"")) {
        printf("Pin config: invalid or missing action\n");
        return false;
    }

    // Preferred format (peripheral-first):
    //   {"action":"configure","version":N,
    //    "peripherals":[{"id":"fas100-1","type":"fas100","pins":{...},"params":{...}},...]}
    const char* peripherals_key = strstr(json, "\"peripherals\"");
    if (peripherals_key) {
        const char* arr = strchr(peripherals_key, '[');
        if (!arr) {
            printf("Pin config: 'peripherals' is not an array\n");
            return false;
        }
        arr++;
        return apply_peripherals_json(arr, json + json_len);
    }

    // Fallback legacy format: {"pins":{"GPIO":{...}}} — left in place
    // so older config snapshots stored as JSON can still re-apply.

    // Find pins object
    const char* pins_start = strstr(json, "\"pins\"");
    if (!pins_start) {
        printf("Pin config: missing pins object\n");
        return false;
    }

    // Find opening brace of pins object
    pins_start = strchr(pins_start, '{');
    if (!pins_start) return false;
    pins_start++; // Skip opening brace

    // Reset current configuration
    pin_config_reset();

    // Parse each pin entry
    // Format: "GPIO":{"mode":"pwm","logical_name":"xxx","pwm_frequency":1000}
    while (*pins_start && *pins_start != '}') {
        // Skip whitespace and commas
        while (*pins_start == ' ' || *pins_start == ',' || *pins_start == '\n' || *pins_start == '\r') {
            pins_start++;
        }

        if (*pins_start == '}') break;

        // Parse GPIO number (key)
        if (*pins_start != '"') break;
        pins_start++;

        char gpio_str[8];
        int gpio_len = 0;
        while (*pins_start && *pins_start != '"' && gpio_len < 7) {
            gpio_str[gpio_len++] = *pins_start++;
        }
        gpio_str[gpio_len] = '\0';

        if (*pins_start != '"') break;
        pins_start++;

        uint8_t gpio = (uint8_t)atoi(gpio_str);

        // Skip to value object
        const char* value_start = strchr(pins_start, '{');
        if (!value_start) break;
        value_start++;

        // Find end of this object
        int brace_count = 1;
        const char* value_end = value_start;
        while (*value_end && brace_count > 0) {
            if (*value_end == '{') brace_count++;
            else if (*value_end == '}') brace_count--;
            value_end++;
        }

        // Parse mode
        pin_mode_t mode = PIN_MODE_UNCONFIGURED;
        const char* mode_str = strstr(value_start, "\"mode\"");
        if (mode_str && mode_str < value_end) {
            mode_str = strchr(mode_str, ':');
            if (mode_str) {
                mode_str++;
                while (*mode_str == ' ' || *mode_str == '"') mode_str++;

                char mode_buf[32];
                int mode_len = 0;
                while (*mode_str && *mode_str != '"' && *mode_str != ',' && mode_len < 31) {
                    mode_buf[mode_len++] = *mode_str++;
                }
                mode_buf[mode_len] = '\0';
                mode = pin_mode_from_string(mode_buf);
            }
        }

        // Parse logical_name
        char logical_name[PIN_CONFIG_MAX_NAME_LEN] = "";
        const char* name_str = strstr(value_start, "\"logical_name\"");
        if (name_str && name_str < value_end) {
            name_str = strchr(name_str, ':');
            if (name_str) {
                name_str++;
                while (*name_str == ' ' || *name_str == '"') name_str++;

                int name_len = 0;
                while (*name_str && *name_str != '"' && name_len < PIN_CONFIG_MAX_NAME_LEN - 1) {
                    logical_name[name_len++] = *name_str++;
                }
                logical_name[name_len] = '\0';
            }
        }

        // Apply configuration
        if (mode != PIN_MODE_UNCONFIGURED) {
            if (pin_config_set(gpio, mode, logical_name)) {
                printf("Pin config: GPIO %d -> %s (%s)\n",
                       gpio, pin_mode_to_string(mode), logical_name);

                // Parse mode-specific parameters
                if (mode == PIN_MODE_PWM) {
                    // Parse pwm_frequency
                    uint32_t freq = PIN_CONFIG_DEFAULT_PWM_FREQ;

                    const char* freq_str = strstr(value_start, "\"pwm_frequency\"");
                    if (freq_str && freq_str < value_end) {
                        freq_str = strchr(freq_str, ':');
                        if (freq_str) {
                            freq_str++;
                            while (*freq_str == ' ') freq_str++;
                            freq = (uint32_t)atoi(freq_str);
                        }
                    }

                    pin_config_set_pwm_params(gpio, freq, 0);
                } else if (mode == PIN_MODE_SERVO) {
                    // Parse servo extents — start/end/center/home are
                    // each a uint16 µs pulse width. Missing keys fall
                    // back to compile-time defaults.
                    uint16_t start_us  = PIN_CONFIG_SERVO_START_US;
                    uint16_t end_us    = PIN_CONFIG_SERVO_END_US;
                    uint16_t center_us = PIN_CONFIG_SERVO_CENTER_US;
                    uint16_t home_us   = PIN_CONFIG_SERVO_HOME_US;

                    struct { const char* key; uint16_t* out; } fields[] = {
                        { "\"start_us\"",  &start_us  },
                        { "\"end_us\"",    &end_us    },
                        { "\"center_us\"", &center_us },
                        { "\"home_us\"",   &home_us   },
                    };
                    for (size_t f = 0; f < sizeof(fields)/sizeof(fields[0]); f++) {
                        const char* k = strstr(value_start, fields[f].key);
                        if (!k || k >= value_end) continue;
                        k = strchr(k, ':');
                        if (!k) continue;
                        k++;
                        while (*k == ' ') k++;
                        *fields[f].out = (uint16_t)atoi(k);
                    }

                    pin_config_set_servo_params(gpio, start_us, end_us, center_us, home_us);
                } else {
                    // Delegate to peripheral driver for mode-specific params
                    const peripheral_driver_t* drv = peripheral_find_by_mode(mode);
                    if (drv && drv->parse_json_params) {
                        pin_config_t* pcfg = find_or_create_config(gpio);
                        if (pcfg) {
                            drv->parse_json_params(value_start, value_end, pcfg);
                        }
                    }
                }
            }
        }

        pins_start = value_end;
    }

    // Apply hardware configuration
    pin_config_apply_hardware();

    return true;
}

bool pin_config_save(void)
{
    flash_storage_data_t storage;

    // Load existing storage data
    if (!flash_storage_load(&storage)) {
        // Initialize new storage
        memset(&storage, 0, sizeof(storage));
        storage.magic = FLASH_STORAGE_MAGIC;
        storage.version = FLASH_STORAGE_VERSION;

        // Copy current node config
        flash_storage_from_node(&storage, &g_node);
    }

    // Copy pin configurations to storage
    storage.pin_config.version = PIN_CONFIG_VERSION;
    storage.pin_config.pin_count = pin_config_count;

    for (uint8_t i = 0; i < pin_config_count && i < PIN_CONFIG_MAX_PINS; i++) {
        storage.pin_config.pins[i].gpio = pin_configs[i].gpio;
        storage.pin_config.pins[i].mode = (uint8_t)pin_configs[i].mode;
        strncpy(storage.pin_config.pins[i].logical_name,
                pin_configs[i].logical_name,
                PIN_CONFIG_MAX_NAME_LEN - 1);

        // Store mode-specific params
        if (pin_configs[i].mode == PIN_MODE_PWM) {
            storage.pin_config.pins[i].param1 = pin_configs[i].params.pwm.frequency;
            storage.pin_config.pins[i].param2 = pin_configs[i].params.pwm.duty_cycle;
        } else if (pin_configs[i].mode == PIN_MODE_SERVO) {
            // Pack the 4×uint16 servo extents into the storage slot's
            // 8 bytes: param1 holds (start_us in low 16, end_us in
            // high 16), param2 holds center_us, reserved_pin[0..1]
            // holds home_us. Frequency is fixed at PIN_CONFIG_SERVO_PWM_FREQ
            // (50 Hz) so it doesn't need flash bytes.
            uint32_t start_us  = pin_configs[i].params.servo.start_us;
            uint32_t end_us    = pin_configs[i].params.servo.end_us;
            storage.pin_config.pins[i].param1 = (start_us & 0xFFFF) | ((end_us & 0xFFFF) << 16);
            storage.pin_config.pins[i].param2 = pin_configs[i].params.servo.center_us;
            uint16_t home_us = pin_configs[i].params.servo.home_us;
            storage.pin_config.pins[i].reserved_pin[0] = (uint8_t)(home_us & 0xFF);
            storage.pin_config.pins[i].reserved_pin[1] = (uint8_t)((home_us >> 8) & 0xFF);
        }
    }

    // Save peripheral driver configs (SyRen, etc.)
    for (uint8_t d = 0; d < peripheral_get_count(); d++) {
        const peripheral_driver_t* drv = peripheral_get(d);
        if (drv && drv->save_config) {
            drv->save_config(&storage);
        }
    }

    // Save to flash
    if (!flash_storage_save(&storage)) {
        printf("Pin config: failed to save to flash\n");
        return false;
    }

    printf("Pin config: saved %d pins to flash\n", pin_config_count);
    return true;
}

bool pin_config_load(void)
{
    flash_storage_data_t storage;

    if (!flash_storage_load(&storage)) {
        printf("Pin config: no stored configuration\n");
        return false;
    }

    // Check pin config version
    if (storage.pin_config.version != PIN_CONFIG_VERSION) {
        printf("Pin config: version mismatch (%d vs %d)\n",
               storage.pin_config.version, PIN_CONFIG_VERSION);
        return false;
    }

    // Reset current config
    pin_config_reset();

    // Load configurations
    for (uint8_t i = 0; i < storage.pin_config.pin_count && i < PIN_CONFIG_MAX_PINS; i++) {
        uint8_t gpio = storage.pin_config.pins[i].gpio;
        pin_mode_t mode = (pin_mode_t)storage.pin_config.pins[i].mode;
        const char* name = storage.pin_config.pins[i].logical_name;

        if (pin_config_set(gpio, mode, name)) {
            // Restore mode-specific params
            if (mode == PIN_MODE_PWM) {
                pin_config_set_pwm_params(gpio,
                    storage.pin_config.pins[i].param1,
                    storage.pin_config.pins[i].param2);
            } else if (mode == PIN_MODE_SERVO) {
                // Inverse of the pack in pin_config_save (see comment
                // there for the byte layout). Zero values fall back to
                // compile-time defaults so a fresh-magic blob doesn't
                // produce a 0-µs pulse.
                uint32_t packed   = storage.pin_config.pins[i].param1;
                uint16_t start_us = (uint16_t)(packed & 0xFFFF);
                uint16_t end_us   = (uint16_t)((packed >> 16) & 0xFFFF);
                uint16_t center_us = storage.pin_config.pins[i].param2;
                uint16_t home_us  = (uint16_t)storage.pin_config.pins[i].reserved_pin[0] |
                                    ((uint16_t)storage.pin_config.pins[i].reserved_pin[1] << 8);
                if (start_us  == 0) start_us  = PIN_CONFIG_SERVO_START_US;
                if (end_us    == 0) end_us    = PIN_CONFIG_SERVO_END_US;
                if (center_us == 0) center_us = PIN_CONFIG_SERVO_CENTER_US;
                if (home_us   == 0) home_us   = PIN_CONFIG_SERVO_HOME_US;
                pin_config_set_servo_params(gpio, start_us, end_us, center_us, home_us);
            }
        }
    }

    // Restore peripheral driver configs from flash.
    //
    // Design rule: a driver only grabs hardware (UART, SPI, GPIO,
    // timer, etc.) when the operator has actually configured pins of
    // its type on this node. Without this gate, a stale per-driver
    // config block in flash (e.g. a Maestro config that the operator
    // removed from the pin map but left enabled in the driver-local
    // store) would silently re-open a UART at boot, leaving floating
    // pads claimed across the board. That sometimes wire-OR-ed onto
    // another peripheral's RX input — concretely, this is what was
    // killing FAS100 readings after the driver consolidation.
    // Skipping load_config for drivers whose pin_mode has no
    // matching pin in pin_config keeps the hardware-ownership
    // contract intact.
    for (uint8_t d = 0; d < peripheral_get_count(); d++) {
        const peripheral_driver_t* drv = peripheral_get(d);
        if (!drv || !drv->load_config) continue;
        if (!pin_config_has_mode((uint8_t)drv->pin_mode)) {
            printf("Peripheral '%s' has no pins in pin_config — skipping load_config\n",
                   drv->name ? drv->name : "?");
            continue;
        }
        drv->load_config(&storage);
    }

    // Apply hardware configuration
    pin_config_apply_hardware();

    printf("Pin config: loaded %d pins from flash\n", pin_config_count);
    return true;
}

void pin_config_reset(void)
{
    // De-initialize any configured pins (skip peripheral virtual pins)
    for (uint8_t i = 0; i < pin_config_count; i++) {
        if (peripheral_is_virtual_gpio(pin_configs[i].gpio)) continue;
        gpio_deinit(pin_configs[i].gpio);
    }

    memset(pin_configs, 0, sizeof(pin_configs));
    pin_config_count = 0;

    printf("Pin config: reset to defaults\n");
}

bool pin_config_has_configured_pins(void)
{
    for (uint8_t i = 0; i < pin_config_count; i++) {
        if (pin_configs[i].mode != PIN_MODE_UNCONFIGURED) {
            return true;
        }
    }
    return false;
}

bool pin_config_has_mode(uint8_t mode)
{
    for (uint8_t i = 0; i < pin_config_count; i++) {
        if ((uint8_t)pin_configs[i].mode == mode) {
            return true;
        }
    }
    return false;
}

bool pin_config_set(uint8_t gpio, pin_mode_t mode, const char* logical_name)
{
    // Validation against pin tables used to live here. The server now
    // owns that — it checks against config/boards/<chip>/<board>.yaml
    // before sending the configure message, so we just trust whatever
    // (gpio, mode) pair arrives.

    // Find or create config entry
    pin_config_t* cfg = find_or_create_config(gpio);
    if (!cfg) {
        printf("Pin config: no space for GPIO %d\n", gpio);
        return false;
    }

    cfg->mode = mode;
    if (logical_name) {
        strncpy(cfg->logical_name, logical_name, PIN_CONFIG_MAX_NAME_LEN - 1);
        cfg->logical_name[PIN_CONFIG_MAX_NAME_LEN - 1] = '\0';
    } else {
        cfg->logical_name[0] = '\0';
    }

    // Set default parameters for built-in modes whose params live in
    // this struct. Peripheral driver modes intentionally DO NOT get a
    // set_defaults call here — set_defaults populates fields like
    // address/deadband to non-zero "default" values, which then look
    // identical to a fresh JSON-sync payload to apply_config. That
    // confused the boot-reload path (pin_config_load → pin_config_set
    // → set_defaults → apply_hardware → apply_config) into thinking it
    // had real per-channel params and clobbering whatever drv_load just
    // restored from flash — manifesting as "RoboClaw doesn't talk
    // until I sync from the dashboard" after every reboot. The JSON-
    // sync code path (apply_one_peripheral) calls drv->set_defaults
    // explicitly right before parse_json_params, so removing it here
    // doesn't change that path's behavior.
    switch (mode) {
        case PIN_MODE_PWM:
            cfg->params.pwm.frequency = PIN_CONFIG_DEFAULT_PWM_FREQ;
            cfg->params.pwm.duty_cycle = 0;
            break;
        case PIN_MODE_SERVO:
            cfg->params.servo.frequency = PIN_CONFIG_SERVO_PWM_FREQ;
            cfg->params.servo.start_us  = PIN_CONFIG_SERVO_START_US;
            cfg->params.servo.end_us    = PIN_CONFIG_SERVO_END_US;
            cfg->params.servo.center_us = PIN_CONFIG_SERVO_CENTER_US;
            cfg->params.servo.home_us   = PIN_CONFIG_SERVO_HOME_US;
            break;
        case PIN_MODE_DIGITAL_IN:
            cfg->params.digital_in.pull_up = false;
            cfg->params.digital_in.pull_down = false;
            break;
        case PIN_MODE_DIGITAL_OUT:
            cfg->params.digital_out.initial_state = false;
            break;
        default:
            // Peripheral driver modes leave params zero-initialized;
            // either drv_load (boot reload) or apply_one_peripheral's
            // explicit set_defaults + parse_json_params (JSON sync)
            // fills them.
            break;
    }

    return true;
}

bool pin_config_set_pwm_params(uint8_t gpio, uint32_t frequency, uint16_t duty_cycle)
{
    pin_config_t* cfg = find_or_create_config(gpio);
    if (!cfg || cfg->mode != PIN_MODE_PWM) {
        return false;
    }

    cfg->params.pwm.frequency = frequency;
    cfg->params.pwm.duty_cycle = duty_cycle;

    return true;
}

bool pin_config_set_servo_params(uint8_t gpio,
                                 uint16_t start_us, uint16_t end_us,
                                 uint16_t center_us, uint16_t home_us)
{
    pin_config_t* cfg = find_or_create_config(gpio);
    if (!cfg || cfg->mode != PIN_MODE_SERVO) {
        return false;
    }

    cfg->params.servo.start_us  = start_us;
    cfg->params.servo.end_us    = end_us;
    cfg->params.servo.center_us = center_us;
    cfg->params.servo.home_us   = home_us;

    return true;
}

bool pin_config_set_digital_in_params(uint8_t gpio, bool pull_up, bool pull_down)
{
    pin_config_t* cfg = find_or_create_config(gpio);
    if (!cfg || cfg->mode != PIN_MODE_DIGITAL_IN) {
        return false;
    }

    cfg->params.digital_in.pull_up = pull_up;
    cfg->params.digital_in.pull_down = pull_down;

    return true;
}

void pin_config_apply_hardware(void)
{
    for (uint8_t i = 0; i < pin_config_count; i++) {
        pin_config_t* cfg = &pin_configs[i];
        uint8_t gpio = cfg->gpio;

        switch (cfg->mode) {
            case PIN_MODE_DIGITAL_IN:
                gpio_init(gpio);
                gpio_set_dir(gpio, GPIO_IN);
                if (cfg->params.digital_in.pull_up) {
                    gpio_pull_up(gpio);
                } else if (cfg->params.digital_in.pull_down) {
                    gpio_pull_down(gpio);
                }
                break;

            case PIN_MODE_DIGITAL_OUT:
                gpio_init(gpio);
                gpio_set_dir(gpio, GPIO_OUT);
                gpio_put(gpio, cfg->params.digital_out.initial_state);
                break;

            case PIN_MODE_PWM:
            case PIN_MODE_SERVO:
                gpio_set_function(gpio, GPIO_FUNC_PWM);
                {
                    uint slice = pwm_gpio_to_slice_num(gpio);
                    uint channel = pwm_gpio_to_channel(gpio);

                    // Calculate wrap and divider for desired frequency
                    // PWM frequency = 125MHz / (divider * wrap)
                    uint32_t freq = (cfg->mode == PIN_MODE_SERVO)
                        ? cfg->params.servo.frequency
                        : cfg->params.pwm.frequency;
                    if (freq == 0) freq = 1000; // Default 1kHz

                    uint32_t clock = 125000000; // 125 MHz
                    uint32_t divider = 1;
                    uint32_t wrap = clock / freq;

                    // Adjust divider if wrap is too large
                    while (wrap > 65535 && divider < 256) {
                        divider++;
                        wrap = clock / (freq * divider);
                    }

                    pwm_set_wrap(slice, wrap);
                    pwm_set_clkdiv(slice, (float)divider);
                    pwm_set_chan_level(slice, channel, 0);
                    pwm_set_enabled(slice, true);
                }
                // For SERVO pins, drive straight to the operator-
                // configured home_us right after enabling the slice —
                // leaving the channel at 0 % until the first command
                // arrives lets the servo flop unloaded.
                if (cfg->mode == PIN_MODE_SERVO) {
                    pin_control_drive_servo_pulse(gpio, cfg->params.servo.home_us);
                }
                break;

            case PIN_MODE_ADC:
                adc_gpio_init(gpio);
                break;

            case PIN_MODE_I2C_SDA:
            case PIN_MODE_I2C_SCL:
                gpio_set_function(gpio, GPIO_FUNC_I2C);
                break;

            case PIN_MODE_UART_TX:
            case PIN_MODE_UART_RX:
                // Don't clobber a PIO peripheral that's already bound
                // to this pin (e.g. RoboClaw's PIO UART when the
                // operator picked the swapped TX/RX layout). The
                // peripheral driver got there first via
                // drv_load → roboclaw_init; trampling it back to
                // GPIO_FUNC_UART would silently break the link until
                // the next sync.
                //
                // How GPIO 0/1 entries end up in pin_configs at all is
                // a legacy flash artifact — the peripheral-first JSON
                // sync stores its UART pin pair in flash_uart_pins_t,
                // not as standalone pin_configs. Older config formats
                // saved them directly though, so the entries survive
                // forever on devices that were ever provisioned with
                // the old format.
#ifndef SIMULATION
                {
                    uint fn = gpio_get_function(gpio);
                    if (fn != GPIO_FUNC_PIO0 && fn != GPIO_FUNC_PIO1) {
                        gpio_set_function(gpio, GPIO_FUNC_UART);
                    }
                }
#endif
                break;

            default:
            {
                const peripheral_driver_t* drv = peripheral_find_by_mode(cfg->mode);
                if (drv && drv->apply_config) {
                    uint8_t ch = peripheral_gpio_to_channel(drv, gpio);
                    drv->apply_config(ch, cfg);
                }
                break;
            }
        }

        printf("Pin config: applied hardware config for GPIO %d (%s)\n",
               gpio, pin_mode_to_string(cfg->mode));
    }
}

const char* pin_mode_to_string(pin_mode_t mode)
{
    if (mode < MODE_STRING_COUNT && mode_strings[mode]) {
        return mode_strings[mode];
    }
    const peripheral_driver_t* drv = peripheral_find_by_mode(mode);
    if (drv) return drv->mode_string;
    return "unknown";
}

pin_mode_t pin_mode_from_string(const char* str)
{
    if (!str) return PIN_MODE_UNCONFIGURED;

    for (size_t i = 0; i < MODE_STRING_COUNT; i++) {
        if (mode_strings[i] && strcmp(str, mode_strings[i]) == 0) {
            return (pin_mode_t)i;
        }
    }

    const peripheral_driver_t* drv = peripheral_find_by_mode_string(str);
    if (drv) return drv->pin_mode;

    return PIN_MODE_UNCONFIGURED;
}
