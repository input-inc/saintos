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
#include "flash_storage.h"
#include "saint_node.h"
#include "peripheral_driver.h"

// =============================================================================
// Available Pins Definition (Feather RP2040 + Ethernet FeatherWing)
// =============================================================================

// Pins available for user configuration
static const pin_definition_t available_pins[] = {
    // Digital pins with PWM capability
    { 5,  "D5",  PIN_CAP_GPIO_PWM },
    { 6,  "D6",  PIN_CAP_GPIO_PWM },
    { 9,  "D9",  PIN_CAP_GPIO_PWM },
    { 12, "D12", PIN_CAP_GPIO_PWM },
    { 13, "D13", PIN_CAP_GPIO_PWM },     // Also onboard LED
    { 24, "D24", PIN_CAP_GPIO_PWM },
    { 25, "D25", PIN_CAP_GPIO_PWM },

    // Analog pins (ADC + digital + PWM)
    { 26, "A0",  PIN_CAP_GPIO_PWM | PIN_CAP_ADC },
    { 27, "A1",  PIN_CAP_GPIO_PWM | PIN_CAP_ADC },
    { 28, "A2",  PIN_CAP_GPIO_PWM | PIN_CAP_ADC },
    { 29, "A3",  PIN_CAP_GPIO_PWM | PIN_CAP_ADC },

    // I2C pins
    { 2,  "SDA", PIN_CAP_GPIO_PWM | PIN_CAP_I2C_SDA },
    { 3,  "SCL", PIN_CAP_GPIO_PWM | PIN_CAP_I2C_SCL },

    // UART pins
    { 0,  "TX",  PIN_CAP_GPIO_PWM | PIN_CAP_UART_TX },
    { 1,  "RX",  PIN_CAP_GPIO_PWM | PIN_CAP_UART_RX },
};

#define AVAILABLE_PIN_COUNT (sizeof(available_pins) / sizeof(available_pins[0]))

// Reserved pins (used by system - ethernet, NeoPixel, etc.)
static const uint8_t reserved_pins[] = {
    10,     // ETH_PIN_CS
    11,     // ETH_PIN_RST
    16,     // NEOPIXEL_PIN
    18,     // ETH_PIN_SCK
    19,     // ETH_PIN_MOSI
    20,     // ETH_PIN_MISO
};

#define RESERVED_PIN_COUNT (sizeof(reserved_pins) / sizeof(reserved_pins[0]))

// =============================================================================
// Static Variables
// =============================================================================

static node_pin_capabilities_t node_capabilities = {
    .available_pins = available_pins,
    .available_count = AVAILABLE_PIN_COUNT,
    .reserved_pins = reserved_pins,
    .reserved_count = RESERVED_PIN_COUNT
};

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
 * Scratch space for peripheral virtual pin definitions.
 */
static pin_definition_t virtual_pin_scratch;

/**
 * Find pin definition by GPIO number.
 * Searches physical pins first, then registered peripheral virtual pins.
 */
static const pin_definition_t* find_pin_definition(uint8_t gpio)
{
    for (size_t i = 0; i < AVAILABLE_PIN_COUNT; i++) {
        if (available_pins[i].gpio == gpio) {
            return &available_pins[i];
        }
    }

    // Check peripheral virtual pins
    const peripheral_driver_t* drv = peripheral_find_by_gpio(gpio);
    if (drv) {
        uint8_t ch = peripheral_gpio_to_channel(drv, gpio);
        virtual_pin_scratch.gpio = gpio;
        virtual_pin_scratch.capabilities = (uint16_t)drv->capability_flag;
        snprintf(virtual_pin_scratch.name, sizeof(virtual_pin_scratch.name),
                 "%s%d",
                 (drv->pin_mode == PIN_MODE_MAESTRO_SERVO) ? "M" :
                 (drv->pin_mode == PIN_MODE_SYREN_MOTOR) ? "SY" : "P",
                 ch);
        return &virtual_pin_scratch;
    }

    return NULL;
}

/**
 * Check if GPIO is reserved.
 */
static bool is_pin_reserved(uint8_t gpio)
{
    for (size_t i = 0; i < RESERVED_PIN_COUNT; i++) {
        if (reserved_pins[i] == gpio) {
            return true;
        }
    }
    return false;
}

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

/**
 * Check if mode is compatible with pin capabilities.
 */
static bool is_mode_compatible(uint16_t capabilities, pin_mode_t mode)
{
    switch (mode) {
        case PIN_MODE_UNCONFIGURED:
            return true;
        case PIN_MODE_DIGITAL_IN:
            return (capabilities & PIN_CAP_DIGITAL_IN) != 0;
        case PIN_MODE_DIGITAL_OUT:
            return (capabilities & PIN_CAP_DIGITAL_OUT) != 0;
        case PIN_MODE_PWM:
        case PIN_MODE_SERVO:
            return (capabilities & PIN_CAP_PWM) != 0;
        case PIN_MODE_ADC:
            return (capabilities & PIN_CAP_ADC) != 0;
        case PIN_MODE_I2C_SDA:
            return (capabilities & PIN_CAP_I2C_SDA) != 0;
        case PIN_MODE_I2C_SCL:
            return (capabilities & PIN_CAP_I2C_SCL) != 0;
        case PIN_MODE_UART_TX:
            return (capabilities & PIN_CAP_UART_TX) != 0;
        case PIN_MODE_UART_RX:
            return (capabilities & PIN_CAP_UART_RX) != 0;
        default:
        {
            const peripheral_driver_t* drv = peripheral_find_by_mode(mode);
            if (drv) {
                return (capabilities & drv->capability_flag) != 0;
            }
            return false;
        }
    }
}

/**
 * Simple JSON string escape (escapes quotes and backslashes).
 */
static void json_escape_string(char* dest, size_t dest_size, const char* src)
{
    size_t di = 0;
    for (size_t si = 0; src[si] && di < dest_size - 1; si++) {
        if (src[si] == '"' || src[si] == '\\') {
            if (di < dest_size - 2) {
                dest[di++] = '\\';
            }
        }
        dest[di++] = src[si];
    }
    dest[di] = '\0';
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

    printf("Pin config: initialized (%d available, %d reserved)\n",
           AVAILABLE_PIN_COUNT, RESERVED_PIN_COUNT);

    initialized = true;
}

const node_pin_capabilities_t* pin_config_get_capabilities(void)
{
    return &node_capabilities;
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

/**
 * Helper: Build capability string list for a pin's capabilities bitmask.
 */
static int build_caps_string(uint16_t capabilities, char* caps_str, size_t caps_size)
{
    int caps_len = 0;

    struct { uint16_t flag; const char* name; } cap_map[] = {
        { PIN_CAP_DIGITAL_IN,  "digital_in" },
        { PIN_CAP_DIGITAL_OUT, "digital_out" },
        { PIN_CAP_PWM,         "pwm" },
        { PIN_CAP_ADC,         "adc" },
        { PIN_CAP_I2C_SDA,     "i2c_sda" },
        { PIN_CAP_I2C_SCL,     "i2c_scl" },
        { PIN_CAP_UART_TX,     "uart_tx" },
        { PIN_CAP_UART_RX,     "uart_rx" },
    };

    for (size_t j = 0; j < sizeof(cap_map) / sizeof(cap_map[0]); j++) {
        if (capabilities & cap_map[j].flag) {
            caps_len += snprintf(caps_str + caps_len, caps_size - caps_len,
                "%s\"%s\"", caps_len > 0 ? "," : "", cap_map[j].name);
        }
    }

    // Peripheral capability flags
    for (uint8_t d = 0; d < peripheral_get_count(); d++) {
        const peripheral_driver_t* drv = peripheral_get(d);
        if (drv && (capabilities & drv->capability_flag)) {
            caps_len += snprintf(caps_str + caps_len, caps_size - caps_len,
                "%s\"%s\"", caps_len > 0 ? "," : "", drv->mode_string);
        }
    }

    return caps_len;
}

int pin_config_capabilities_to_json(char* buffer, size_t buffer_size, const char* node_id)
{
    if (!buffer || buffer_size < 256) return -1;

    int written = 0;
    int ret;
    bool first_pin = true;

    // Start JSON object
    ret = snprintf(buffer + written, buffer_size - written,
        "{\"node_id\":\"%s\",\"pins\":[", node_id);
    if (ret < 0 || (size_t)ret >= buffer_size - written) return -1;
    written += ret;

    // Write physical pins
    for (size_t i = 0; i < AVAILABLE_PIN_COUNT; i++) {
        const pin_definition_t* pin = &available_pins[i];

        char caps_str[256] = "";
        build_caps_string(pin->capabilities, caps_str, sizeof(caps_str));

        ret = snprintf(buffer + written, buffer_size - written,
            "%s{\"gpio\":%d,\"name\":\"%s\",\"capabilities\":[%s]}",
            first_pin ? "" : ",", pin->gpio, pin->name, caps_str);
        if (ret < 0 || (size_t)ret >= buffer_size - written) return -1;
        written += ret;
        first_pin = false;
    }

    // Write virtual pins from registered peripheral drivers
    for (uint8_t d = 0; d < peripheral_get_count(); d++) {
        const peripheral_driver_t* drv = peripheral_get(d);
        if (!drv) continue;

        for (uint8_t ch = 0; ch < drv->channel_count; ch++) {
            if (drv->capabilities_to_json) {
                ret = snprintf(buffer + written, buffer_size - written,
                    "%s", first_pin ? "" : ",");
                if (ret < 0 || (size_t)ret >= buffer_size - written) return -1;
                written += ret;

                int frag = drv->capabilities_to_json(ch, buffer + written,
                                                      buffer_size - written);
                if (frag < 0) return -1;
                written += frag;
            } else {
                uint16_t gpio = drv->virtual_gpio_base + ch;
                char pin_name[8];
                snprintf(pin_name, sizeof(pin_name), "P%d", ch);

                ret = snprintf(buffer + written, buffer_size - written,
                    "%s{\"gpio\":%d,\"name\":\"%s\",\"capabilities\":[\"%s\"]}",
                    first_pin ? "" : ",", gpio, pin_name, drv->mode_string);
                if (ret < 0 || (size_t)ret >= buffer_size - written) return -1;
                written += ret;
            }
            first_pin = false;
        }
    }

    // Write reserved pins array
    ret = snprintf(buffer + written, buffer_size - written, "],\"reserved_pins\":[");
    if (ret < 0 || (size_t)ret >= buffer_size - written) return -1;
    written += ret;

    for (size_t i = 0; i < RESERVED_PIN_COUNT; i++) {
        ret = snprintf(buffer + written, buffer_size - written,
            "%s%d", i > 0 ? "," : "", reserved_pins[i]);
        if (ret < 0 || (size_t)ret >= buffer_size - written) return -1;
        written += ret;
    }

    // Close JSON
    ret = snprintf(buffer + written, buffer_size - written, "]}");
    if (ret < 0 || (size_t)ret >= buffer_size - written) return -1;
    written += ret;

    return written;
}

bool pin_config_apply_json(const char* json, size_t json_len)
{
    if (!json || json_len == 0) return false;

    // SAFETY: Validate string is null-terminated within json_len
    // This prevents buffer over-reads if caller provides wrong length
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

    // Simple JSON parser for pin configuration
    // Expected format:
    // {"action":"configure","version":N,"pins":{"GPIO":{"mode":"pwm","logical_name":"xxx",...}}}

    // Check for "configure" action
    const char* action = strstr(json, "\"action\"");
    if (!action || !strstr(action, "\"configure\"")) {
        printf("Pin config: invalid or missing action\n");
        return false;
    }

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
                    // Parse servo pulse range
                    uint16_t min_us = PIN_CONFIG_SERVO_MIN_PULSE_US;
                    uint16_t max_us = PIN_CONFIG_SERVO_MAX_PULSE_US;

                    const char* min_str = strstr(value_start, "\"min_pulse_us\"");
                    if (min_str && min_str < value_end) {
                        min_str = strchr(min_str, ':');
                        if (min_str) {
                            min_str++;
                            while (*min_str == ' ') min_str++;
                            min_us = (uint16_t)atoi(min_str);
                        }
                    }

                    const char* max_str = strstr(value_start, "\"max_pulse_us\"");
                    if (max_str && max_str < value_end) {
                        max_str = strchr(max_str, ':');
                        if (max_str) {
                            max_str++;
                            while (*max_str == ' ') max_str++;
                            max_us = (uint16_t)atoi(max_str);
                        }
                    }

                    pin_config_set_servo_params(gpio, min_us, max_us);
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
            storage.pin_config.pins[i].param1 = pin_configs[i].params.servo.frequency;
            storage.pin_config.pins[i].param2 = pin_configs[i].params.servo.min_pulse_us;
            // Pack max_pulse_us into reserved bytes (little-endian)
            storage.pin_config.pins[i].reserved_pin[0] = (uint8_t)(pin_configs[i].params.servo.max_pulse_us & 0xFF);
            storage.pin_config.pins[i].reserved_pin[1] = (uint8_t)((pin_configs[i].params.servo.max_pulse_us >> 8) & 0xFF);
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
                uint16_t min_us = storage.pin_config.pins[i].param2;
                if (min_us == 0) {
                    // Old format: param2 was duty_cycle (always 0 for servo)
                    // Use defaults for backward compatibility
                    min_us = PIN_CONFIG_SERVO_MIN_PULSE_US;
                }
                uint16_t max_us = (uint16_t)storage.pin_config.pins[i].reserved_pin[0] |
                                  ((uint16_t)storage.pin_config.pins[i].reserved_pin[1] << 8);
                if (max_us == 0) {
                    max_us = PIN_CONFIG_SERVO_MAX_PULSE_US;
                }
                pin_config_set_servo_params(gpio, min_us, max_us);
            }
        }
    }

    // Restore peripheral driver configs from flash
    for (uint8_t d = 0; d < peripheral_get_count(); d++) {
        const peripheral_driver_t* drv = peripheral_get(d);
        if (drv && drv->load_config) {
            drv->load_config(&storage);
        }
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

bool pin_config_set(uint8_t gpio, pin_mode_t mode, const char* logical_name)
{
    // Verify GPIO is available
    const pin_definition_t* def = find_pin_definition(gpio);
    if (!def) {
        printf("Pin config: GPIO %d not available\n", gpio);
        return false;
    }

    // Verify pin is not reserved
    if (is_pin_reserved(gpio)) {
        printf("Pin config: GPIO %d is reserved\n", gpio);
        return false;
    }

    // Verify mode is compatible
    if (!is_mode_compatible(def->capabilities, mode)) {
        printf("Pin config: mode %s not compatible with GPIO %d\n",
               pin_mode_to_string(mode), gpio);
        return false;
    }

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

    // Set default parameters based on mode
    switch (mode) {
        case PIN_MODE_PWM:
            cfg->params.pwm.frequency = PIN_CONFIG_DEFAULT_PWM_FREQ;
            cfg->params.pwm.duty_cycle = 0;
            break;
        case PIN_MODE_SERVO:
            cfg->params.servo.frequency = PIN_CONFIG_SERVO_PWM_FREQ;
            cfg->params.servo.min_pulse_us = PIN_CONFIG_SERVO_MIN_PULSE_US;
            cfg->params.servo.max_pulse_us = PIN_CONFIG_SERVO_MAX_PULSE_US;
            break;
        case PIN_MODE_DIGITAL_IN:
            cfg->params.digital_in.pull_up = false;
            cfg->params.digital_in.pull_down = false;
            break;
        case PIN_MODE_DIGITAL_OUT:
            cfg->params.digital_out.initial_state = false;
            break;
        default:
        {
            const peripheral_driver_t* drv = peripheral_find_by_mode(mode);
            if (drv && drv->set_defaults) {
                uint8_t ch = peripheral_gpio_to_channel(drv, gpio);
                drv->set_defaults(ch, cfg);
            }
            break;
        }
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

bool pin_config_set_servo_params(uint8_t gpio, uint16_t min_pulse_us, uint16_t max_pulse_us)
{
    pin_config_t* cfg = find_or_create_config(gpio);
    if (!cfg || cfg->mode != PIN_MODE_SERVO) {
        return false;
    }

    cfg->params.servo.min_pulse_us = min_pulse_us;
    cfg->params.servo.max_pulse_us = max_pulse_us;

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
                gpio_set_function(gpio, GPIO_FUNC_UART);
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
