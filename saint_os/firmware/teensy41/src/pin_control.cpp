/**
 * SAINT.OS Node Firmware - Teensy 4.1 Pin Control Implementation
 *
 * Runtime control of pin values for PWM, servo, digital outputs, and ADC reading.
 * Uses Arduino/Teensy APIs: analogWrite(), analogRead(), digitalWriteFast(), etc.
 */

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern "C" {
#include "pin_control.h"
#include "pin_config.h"
#include "peripheral_driver.h"
}

// =============================================================================
// Constants
// =============================================================================

#define MAX_RUNTIME_VALUES 48

// Servo timing constant
#define SERVO_PERIOD_US     20000   // 20ms (50Hz)

// ADC constants
#define ADC_VREF            3.3f
#define ADC_RESOLUTION_BITS 12
#define ADC_MAX_VALUE       4096    // 12-bit

// PWM constants
#define PWM_RESOLUTION_BITS 12
#define PWM_MAX_VALUE       4095    // 12-bit (0-4095)

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

// =============================================================================
// Public API Implementation (extern "C" linkage for shared headers)
// =============================================================================

extern "C" {

void pin_control_init(void)
{
    if (initialized) return;

    memset(runtime_values, 0, sizeof(runtime_values));
    runtime_value_count = 0;

    // Set Teensy PWM and ADC resolution to 12-bit
    analogWriteResolution(PWM_RESOLUTION_BITS);
    analogReadResolution(ADC_RESOLUTION_BITS);

    // Initialize runtime values for all configured pins
    uint8_t count;
    const pin_config_t* configs = pin_config_get_all(&count);

    for (uint8_t i = 0; i < count; i++) {
        const pin_config_t* cfg = &configs[i];
        if (cfg->mode != PIN_MODE_UNCONFIGURED) {
            pin_runtime_value_t* rv = find_or_create_runtime(cfg->gpio);
            if (rv) {
                rv->value = 0.0f;
                rv->last_updated = millis();
            }
        }
    }

    Serial.printf("Pin control: initialized with %d pins\n", runtime_value_count);
    initialized = true;
}

bool pin_control_set_value(uint8_t gpio, float value)
{
    const pin_config_t* cfg = pin_config_get(gpio);
    if (!cfg) {
        Serial.printf("Pin control: GPIO %d not configured\n", gpio);
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
        {
            // Dispatch to peripheral driver
            const peripheral_driver_t* drv = peripheral_find_by_mode(cfg->mode);
            if (drv && drv->set_value) {
                uint8_t ch = peripheral_gpio_to_channel(drv, gpio);
                bool ok = drv->set_value(ch, value);

                // Store runtime value
                pin_runtime_value_t* rv = find_or_create_runtime(gpio);
                if (rv) {
                    rv->value = value;
                    rv->last_updated = millis();
                }
                return ok;
            }

            Serial.printf("Pin control: GPIO %d mode %s not controllable\n",
                   gpio, pin_mode_to_string(cfg->mode));
            return false;
        }
    }
}

bool pin_control_set_pwm(uint8_t gpio, float percent)
{
    const pin_config_t* cfg = pin_config_get(gpio);
    if (!cfg || cfg->mode != PIN_MODE_PWM) {
        Serial.printf("Pin control: GPIO %d not configured as PWM\n", gpio);
        return false;
    }

    // Clamp value
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;

    // Calculate PWM level (12-bit: 0-4095)
    uint32_t level = (uint32_t)((percent / 100.0f) * PWM_MAX_VALUE);
    analogWrite(gpio, level);

    // Store runtime value
    pin_runtime_value_t* rv = find_or_create_runtime(gpio);
    if (rv) {
        rv->value = percent;
        rv->last_updated = millis();
    }

    return true;
}

bool pin_control_set_servo(uint8_t gpio, float angle)
{
    const pin_config_t* cfg = pin_config_get(gpio);
    if (!cfg || cfg->mode != PIN_MODE_SERVO) {
        Serial.printf("Pin control: GPIO %d not configured as servo\n", gpio);
        return false;
    }

    // Clamp angle
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    // Calculate pulse width using per-pin configurable range
    uint16_t min_us = cfg->params.servo.min_pulse_us;
    uint16_t max_us = cfg->params.servo.max_pulse_us;
    float pulse_us = min_us + (angle / 180.0f) * (max_us - min_us);
    float duty_fraction = pulse_us / (float)SERVO_PERIOD_US;

    // Teensy analogWrite with 12-bit resolution at 50Hz
    // Ensure frequency is set to 50Hz (should already be set by pin_config_apply_hardware)
    analogWriteFrequency(gpio, PIN_CONFIG_SERVO_PWM_FREQ);
    uint32_t level = (uint32_t)(duty_fraction * PWM_MAX_VALUE);
    analogWrite(gpio, level);

    // Store runtime value (store angle, not duty cycle)
    pin_runtime_value_t* rv = find_or_create_runtime(gpio);
    if (rv) {
        rv->value = angle;
        rv->last_updated = millis();
    }

    return true;
}

bool pin_control_set_digital(uint8_t gpio, bool state)
{
    const pin_config_t* cfg = pin_config_get(gpio);
    if (!cfg || cfg->mode != PIN_MODE_DIGITAL_OUT) {
        Serial.printf("Pin control: GPIO %d not configured as digital_out\n", gpio);
        return false;
    }

    digitalWriteFast(gpio, state ? HIGH : LOW);

    // Store runtime value
    pin_runtime_value_t* rv = find_or_create_runtime(gpio);
    if (rv) {
        rv->value = state ? 1.0f : 0.0f;
        rv->last_updated = millis();
    }

    return true;
}

bool pin_control_read_adc(uint8_t gpio, uint16_t* raw_value, float* voltage)
{
    const pin_config_t* cfg = pin_config_get(gpio);
    if (!cfg || cfg->mode != PIN_MODE_ADC) {
        return false;
    }

    // Teensy 4.1 analogRead works with pin numbers directly
    uint16_t raw = analogRead(gpio);

    if (raw_value) {
        *raw_value = raw;
    }
    if (voltage) {
        *voltage = (raw / (float)ADC_MAX_VALUE) * ADC_VREF;
    }

    // Store runtime value (store voltage)
    pin_runtime_value_t* rv = find_or_create_runtime(gpio);
    if (rv) {
        rv->value = (raw / (float)ADC_MAX_VALUE) * ADC_VREF;
        rv->last_updated = millis();
    }

    return true;
}

bool pin_control_read_digital(uint8_t gpio)
{
    const pin_config_t* cfg = pin_config_get(gpio);
    if (!cfg || cfg->mode != PIN_MODE_DIGITAL_IN) {
        return false;
    }

    bool state = digitalReadFast(gpio);

    // Store runtime value
    pin_runtime_value_t* rv = find_or_create_runtime(gpio);
    if (rv) {
        rv->value = state ? 1.0f : 0.0f;
        rv->last_updated = millis();
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
            {
                // Dispatch to peripheral driver for value readback
                const peripheral_driver_t* drv = peripheral_find_by_mode(cfg->mode);
                if (drv && drv->get_value && drv->is_connected && drv->is_connected()) {
                    uint8_t ch = peripheral_gpio_to_channel(drv, cfg->gpio);
                    float val;
                    if (drv->get_value(ch, &val)) {
                        pin_runtime_value_t* rv = find_or_create_runtime(cfg->gpio);
                        if (rv) {
                            rv->value = val;
                            rv->last_updated = millis();
                        }
                    }
                }
                break;
            }
        }
    }
}

void pin_control_estop(void)
{
    Serial.printf("ESTOP: Setting all outputs to safe values\n");

    // Get all configured pins
    uint8_t count;
    const pin_config_t* configs = pin_config_get_all(&count);

    for (uint8_t i = 0; i < count; i++) {
        const pin_config_t* cfg = &configs[i];

        switch (cfg->mode) {
            case PIN_MODE_PWM:
                // Set PWM to 0%
                pin_control_set_pwm(cfg->gpio, 0.0f);
                Serial.printf("ESTOP: GPIO %d (PWM) -> 0%%\n", cfg->gpio);
                break;

            case PIN_MODE_SERVO:
                // Set servo to center position (90 degrees) for safety
                pin_control_set_servo(cfg->gpio, 90.0f);
                Serial.printf("ESTOP: GPIO %d (Servo) -> 90 deg (center)\n", cfg->gpio);
                break;

            case PIN_MODE_DIGITAL_OUT:
                // Set digital outputs low
                pin_control_set_digital(cfg->gpio, false);
                Serial.printf("ESTOP: GPIO %d (Digital Out) -> LOW\n", cfg->gpio);
                break;

            default:
                // Input pins don't need ESTOP handling
                break;
        }
    }

    // Emergency stop all peripheral drivers (Maestro, SyRen, etc.)
    peripheral_estop_all();
    Serial.printf("ESTOP: peripherals stopped\n");

    Serial.printf("ESTOP: Complete\n");
}

bool pin_control_apply_json(const char* json, size_t json_len)
{
    if (!json || json_len == 0) return false;

    // SAFETY: Validate string is null-terminated within json_len
    bool found_null = false;
    for (size_t i = 0; i <= json_len && i < 1024; i++) {
        if (json[i] == '\0') {
            found_null = true;
            break;
        }
    }
    if (!found_null) {
        Serial.printf("Pin control: JSON not null-terminated within length\n");
        return false;
    }

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

    Serial.printf("Pin control: set GPIO %d = %.2f\n", gpio, value);

    return pin_control_set_value(gpio, value);
}

int pin_control_state_to_json(char* buffer, size_t buffer_size, const char* node_id)
{
    if (!buffer || buffer_size < 128) return -1;

    int written = 0;
    int ret;

    // Start JSON object
    uint32_t timestamp = millis();
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

} // extern "C"
