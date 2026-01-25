/**
 * SAINT.OS Node Firmware - Pin Control Implementation
 *
 * Runtime control of pin values for PWM, servo, digital outputs, and ADC reading.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

#include "pin_control.h"
#include "pin_config.h"

// =============================================================================
// Constants
// =============================================================================

#define MAX_RUNTIME_VALUES 16

// Servo timing constants (standard servo)
#define SERVO_MIN_PULSE_US  500     // 0.5ms for 0 degrees
#define SERVO_MAX_PULSE_US  2500    // 2.5ms for 180 degrees
#define SERVO_PERIOD_US     20000   // 20ms (50Hz)

// ADC constants
#define ADC_VREF            3.3f
#define ADC_RESOLUTION      4096    // 12-bit

// =============================================================================
// Static Variables
// =============================================================================

static pin_runtime_value_t runtime_values[MAX_RUNTIME_VALUES];
static uint8_t runtime_value_count = 0;
static bool initialized = false;

// =============================================================================
// Helper Functions
// =============================================================================

/**
 * Find or create runtime value entry for GPIO.
 */
static pin_runtime_value_t* find_or_create_runtime(uint8_t gpio)
{
    // Check if exists
    for (uint8_t i = 0; i < runtime_value_count; i++) {
        if (runtime_values[i].gpio == gpio) {
            return &runtime_values[i];
        }
    }

    // Create new if space available
    if (runtime_value_count < MAX_RUNTIME_VALUES) {
        pin_runtime_value_t* rv = &runtime_values[runtime_value_count++];
        rv->gpio = gpio;
        rv->value = 0.0f;
        rv->last_updated = 0;
        return rv;
    }

    return NULL;
}

/**
 * Get runtime value entry for GPIO.
 */
static pin_runtime_value_t* find_runtime(uint8_t gpio)
{
    for (uint8_t i = 0; i < runtime_value_count; i++) {
        if (runtime_values[i].gpio == gpio) {
            return &runtime_values[i];
        }
    }
    return NULL;
}

/**
 * Calculate PWM wrap value for a slice at a given frequency.
 */
static uint32_t get_pwm_wrap(uint8_t gpio)
{
    const pin_config_t* cfg = pin_config_get(gpio);
    if (!cfg) return 65535;

    uint32_t freq = cfg->params.pwm.frequency;
    if (freq == 0) freq = 1000;

    uint32_t clock = 125000000; // 125 MHz
    uint32_t divider = 1;
    uint32_t wrap = clock / freq;

    while (wrap > 65535 && divider < 256) {
        divider++;
        wrap = clock / (freq * divider);
    }

    return wrap;
}

// =============================================================================
// Public API Implementation
// =============================================================================

void pin_control_init(void)
{
    if (initialized) return;

    memset(runtime_values, 0, sizeof(runtime_values));
    runtime_value_count = 0;

    // Initialize runtime values for all configured pins
    uint8_t count;
    const pin_config_t* configs = pin_config_get_all(&count);

    for (uint8_t i = 0; i < count; i++) {
        const pin_config_t* cfg = &configs[i];
        if (cfg->mode != PIN_MODE_UNCONFIGURED) {
            pin_runtime_value_t* rv = find_or_create_runtime(cfg->gpio);
            if (rv) {
                rv->value = 0.0f;
                rv->last_updated = to_ms_since_boot(get_absolute_time());
            }
        }
    }

    printf("Pin control: initialized with %d pins\n", runtime_value_count);
    initialized = true;
}

bool pin_control_set_value(uint8_t gpio, float value)
{
    const pin_config_t* cfg = pin_config_get(gpio);
    if (!cfg) {
        printf("Pin control: GPIO %d not configured\n", gpio);
        return false;
    }

    switch (cfg->mode) {
        case PIN_MODE_PWM:
            return pin_control_set_pwm(gpio, value);

        case PIN_MODE_SERVO:
            return pin_control_set_servo(gpio, value);

        case PIN_MODE_DIGITAL_OUT:
            return pin_control_set_digital(gpio, value >= 0.5f);

        default:
            printf("Pin control: GPIO %d mode %s not controllable\n",
                   gpio, pin_mode_to_string(cfg->mode));
            return false;
    }
}

bool pin_control_set_pwm(uint8_t gpio, float percent)
{
    const pin_config_t* cfg = pin_config_get(gpio);
    if (!cfg || cfg->mode != PIN_MODE_PWM) {
        printf("Pin control: GPIO %d not configured as PWM\n", gpio);
        return false;
    }

    // Clamp value
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;

    // Calculate PWM level
    uint slice = pwm_gpio_to_slice_num(gpio);
    uint channel = pwm_gpio_to_channel(gpio);
    uint32_t wrap = get_pwm_wrap(gpio);
    uint32_t level = (uint32_t)((percent / 100.0f) * wrap);

    pwm_set_chan_level(slice, channel, level);

    // Store runtime value
    pin_runtime_value_t* rv = find_or_create_runtime(gpio);
    if (rv) {
        rv->value = percent;
        rv->last_updated = to_ms_since_boot(get_absolute_time());
    }

    return true;
}

bool pin_control_set_servo(uint8_t gpio, float angle)
{
    const pin_config_t* cfg = pin_config_get(gpio);
    if (!cfg || cfg->mode != PIN_MODE_SERVO) {
        printf("Pin control: GPIO %d not configured as servo\n", gpio);
        return false;
    }

    // Clamp angle
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    // Calculate pulse width
    // Servo expects 50Hz PWM with 0.5ms-2.5ms pulse
    float pulse_us = SERVO_MIN_PULSE_US +
                     (angle / 180.0f) * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
    float duty_percent = (pulse_us / SERVO_PERIOD_US) * 100.0f;

    // Calculate PWM level
    uint slice = pwm_gpio_to_slice_num(gpio);
    uint channel = pwm_gpio_to_channel(gpio);
    uint32_t wrap = get_pwm_wrap(gpio);
    uint32_t level = (uint32_t)((duty_percent / 100.0f) * wrap);

    pwm_set_chan_level(slice, channel, level);

    // Store runtime value (store angle, not duty cycle)
    pin_runtime_value_t* rv = find_or_create_runtime(gpio);
    if (rv) {
        rv->value = angle;
        rv->last_updated = to_ms_since_boot(get_absolute_time());
    }

    return true;
}

bool pin_control_set_digital(uint8_t gpio, bool state)
{
    const pin_config_t* cfg = pin_config_get(gpio);
    if (!cfg || cfg->mode != PIN_MODE_DIGITAL_OUT) {
        printf("Pin control: GPIO %d not configured as digital_out\n", gpio);
        return false;
    }

    gpio_put(gpio, state);

    // Store runtime value
    pin_runtime_value_t* rv = find_or_create_runtime(gpio);
    if (rv) {
        rv->value = state ? 1.0f : 0.0f;
        rv->last_updated = to_ms_since_boot(get_absolute_time());
    }

    return true;
}

bool pin_control_read_adc(uint8_t gpio, uint16_t* raw_value, float* voltage)
{
    const pin_config_t* cfg = pin_config_get(gpio);
    if (!cfg || cfg->mode != PIN_MODE_ADC) {
        return false;
    }

    // ADC channels: GPIO 26-29 map to ADC 0-3
    if (gpio < 26 || gpio > 29) {
        return false;
    }

    uint8_t adc_channel = gpio - 26;
    adc_select_input(adc_channel);
    uint16_t raw = adc_read();

    if (raw_value) {
        *raw_value = raw;
    }
    if (voltage) {
        *voltage = (raw / (float)ADC_RESOLUTION) * ADC_VREF;
    }

    // Store runtime value (store voltage)
    pin_runtime_value_t* rv = find_or_create_runtime(gpio);
    if (rv) {
        rv->value = (raw / (float)ADC_RESOLUTION) * ADC_VREF;
        rv->last_updated = to_ms_since_boot(get_absolute_time());
    }

    return true;
}

bool pin_control_read_digital(uint8_t gpio)
{
    const pin_config_t* cfg = pin_config_get(gpio);
    if (!cfg || cfg->mode != PIN_MODE_DIGITAL_IN) {
        return false;
    }

    bool state = gpio_get(gpio);

    // Store runtime value
    pin_runtime_value_t* rv = find_or_create_runtime(gpio);
    if (rv) {
        rv->value = state ? 1.0f : 0.0f;
        rv->last_updated = to_ms_since_boot(get_absolute_time());
    }

    return state;
}

bool pin_control_get_value(uint8_t gpio, float* value)
{
    pin_runtime_value_t* rv = find_runtime(gpio);
    if (!rv) {
        return false;
    }

    if (value) {
        *value = rv->value;
    }
    return true;
}

void pin_control_update_state(void)
{
    // Read all input pins to update their values
    uint8_t count;
    const pin_config_t* configs = pin_config_get_all(&count);

    for (uint8_t i = 0; i < count; i++) {
        const pin_config_t* cfg = &configs[i];

        switch (cfg->mode) {
            case PIN_MODE_ADC:
                pin_control_read_adc(cfg->gpio, NULL, NULL);
                break;

            case PIN_MODE_DIGITAL_IN:
                pin_control_read_digital(cfg->gpio);
                break;

            default:
                break;
        }
    }
}

bool pin_control_apply_json(const char* json, size_t json_len)
{
    if (!json || json_len == 0) return false;

    // Check for set_pin action
    if (!strstr(json, "\"action\":\"set_pin\"") &&
        !strstr(json, "\"action\": \"set_pin\"")) {
        return false;
    }

    // Parse GPIO
    const char* gpio_str = strstr(json, "\"gpio\"");
    if (!gpio_str) return false;

    gpio_str = strchr(gpio_str, ':');
    if (!gpio_str) return false;
    gpio_str++;
    while (*gpio_str == ' ') gpio_str++;

    uint8_t gpio = (uint8_t)atoi(gpio_str);

    // Parse value
    const char* value_str = strstr(json, "\"value\"");
    if (!value_str) return false;

    value_str = strchr(value_str, ':');
    if (!value_str) return false;
    value_str++;
    while (*value_str == ' ') value_str++;

    float value = (float)atof(value_str);

    printf("Pin control: set GPIO %d = %.2f\n", gpio, value);

    return pin_control_set_value(gpio, value);
}

int pin_control_state_to_json(char* buffer, size_t buffer_size, const char* node_id)
{
    if (!buffer || buffer_size < 128) return -1;

    int written = 0;
    int ret;

    // Start JSON object
    uint32_t timestamp = to_ms_since_boot(get_absolute_time());
    ret = snprintf(buffer + written, buffer_size - written,
        "{\"node_id\":\"%s\",\"timestamp\":%lu,\"pins\":[",
        node_id, (unsigned long)timestamp);
    if (ret < 0 || (size_t)ret >= buffer_size - written) return -1;
    written += ret;

    // Write configured pins
    uint8_t count;
    const pin_config_t* configs = pin_config_get_all(&count);
    bool first = true;

    for (uint8_t i = 0; i < count; i++) {
        const pin_config_t* cfg = &configs[i];
        if (cfg->mode == PIN_MODE_UNCONFIGURED) continue;

        pin_runtime_value_t* rv = find_runtime(cfg->gpio);
        float value = rv ? rv->value : 0.0f;

        const char* mode_str = pin_mode_to_string(cfg->mode);
        char escaped_name[PIN_CONFIG_MAX_NAME_LEN * 2];

        // Simple escape for logical name
        size_t j = 0;
        for (size_t k = 0; cfg->logical_name[k] && j < sizeof(escaped_name) - 1; k++) {
            if (cfg->logical_name[k] == '"' || cfg->logical_name[k] == '\\') {
                if (j < sizeof(escaped_name) - 2) {
                    escaped_name[j++] = '\\';
                }
            }
            escaped_name[j++] = cfg->logical_name[k];
        }
        escaped_name[j] = '\0';

        ret = snprintf(buffer + written, buffer_size - written,
            "%s{\"gpio\":%d,\"mode\":\"%s\",\"value\":%.2f,\"name\":\"%s\"",
            first ? "" : ",",
            cfg->gpio, mode_str, value, escaped_name);
        if (ret < 0 || (size_t)ret >= buffer_size - written) return -1;
        written += ret;

        // Add voltage for ADC pins
        if (cfg->mode == PIN_MODE_ADC) {
            float voltage = value;  // Already stored as voltage
            ret = snprintf(buffer + written, buffer_size - written,
                ",\"voltage\":%.3f", voltage);
            if (ret < 0 || (size_t)ret >= buffer_size - written) return -1;
            written += ret;
        }

        ret = snprintf(buffer + written, buffer_size - written, "}");
        if (ret < 0 || (size_t)ret >= buffer_size - written) return -1;
        written += ret;

        first = false;
    }

    // Close JSON
    ret = snprintf(buffer + written, buffer_size - written, "]}");
    if (ret < 0 || (size_t)ret >= buffer_size - written) return -1;
    written += ret;

    return written;
}
