/**
 * SAINT.OS Node Firmware - Teensy 4.1 Pin Control Implementation
 *
 * Runtime control of pin values for PWM, servo, digital outputs, and ADC reading.
 * Uses Arduino/Teensy APIs: analogWrite(), analogRead(), digitalWriteFast(), etc.
 */

#include <Arduino.h>
#include "platform.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern "C" {
#include "pin_control.h"
#include "pin_config.h"
#include "peripheral_driver.h"
#include "maestro_driver.h"   // maestro_set_target_preview (live extent-dial jog)
#include "saint_types.h"   // led_set_override_color / led_set_override_brightness / led_clear_override
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
            Serial.printf("pin_control_set_value: GPIO %d mode %s -> drv=%s set_value=%p\n",
                          gpio, pin_mode_to_string(cfg->mode),
                          drv ? drv->name : "<null>",
                          drv ? (void*)drv->set_value : NULL);
            if (drv && drv->set_value) {
                uint8_t ch = peripheral_gpio_to_channel(drv, gpio);
                Serial.printf("pin_control_set_value: drv=%s ch=%u value=%.3f\n",
                              drv->name, (unsigned)ch, value);
                bool ok = drv->set_value(ch, value);
                Serial.printf("pin_control_set_value: drv->set_value returned %s\n",
                              ok ? "true" : "false");

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

bool pin_control_set_servo(uint8_t gpio, float value)
{
    const pin_config_t* cfg = pin_config_get(gpio);
    if (!cfg || cfg->mode != PIN_MODE_SERVO) {
        Serial.printf("Pin control: GPIO %d not configured as servo\n", gpio);
        return false;
    }

    // Normalized input −1..+1, piecewise-linear through center_us so
    // asymmetric mechanical ranges (center not at midpoint of
    // start/end) map sensibly.
    if (value < -1.0f) value = -1.0f;
    if (value >  1.0f) value =  1.0f;

    float pulse_us;
    if (value <= 0.0f) {
        // [-1, 0]: lerp start → center
        pulse_us = (float)cfg->params.servo.center_us
                 + value * ((float)cfg->params.servo.center_us
                          - (float)cfg->params.servo.start_us);
    } else {
        // [0, +1]: lerp center → end
        pulse_us = (float)cfg->params.servo.center_us
                 + value * ((float)cfg->params.servo.end_us
                          - (float)cfg->params.servo.center_us);
    }

    // Mechanical-envelope clamp so a botched extent can't drive past
    // typical hobby-servo safe limits.
    if (pulse_us < (float)PIN_CONFIG_SERVO_MIN_PULSE_US) pulse_us = PIN_CONFIG_SERVO_MIN_PULSE_US;
    if (pulse_us > (float)PIN_CONFIG_SERVO_MAX_PULSE_US) pulse_us = PIN_CONFIG_SERVO_MAX_PULSE_US;

    float duty_fraction = pulse_us / (float)SERVO_PERIOD_US;
    analogWriteFrequency(gpio, PIN_CONFIG_SERVO_PWM_FREQ);
    uint32_t level = (uint32_t)(duty_fraction * PWM_MAX_VALUE);
    analogWrite(gpio, level);

    // Store the operator-facing normalized value, not the pulse —
    // matches how the dashboard slider expresses commanded position.
    pin_runtime_value_t* rv = find_or_create_runtime(gpio);
    if (rv) {
        rv->value = value;
        rv->last_updated = millis();
    }

    return true;
}

// Drive the servo straight to a pulse-µs value, bypassing the
// −1..+1 → pulse mapping. Used at boot (apply_hardware → home_us)
// and on safe-reset. The runtime "value" stored after this is the
// normalized -1..+1 estimate so the dashboard's last-known position
// matches what's actually on the wire.
bool pin_control_drive_servo_pulse(uint8_t gpio, uint16_t pulse_us)
{
    const pin_config_t* cfg = pin_config_get(gpio);
    if (!cfg || cfg->mode != PIN_MODE_SERVO) return false;

    if (pulse_us < PIN_CONFIG_SERVO_MIN_PULSE_US) pulse_us = PIN_CONFIG_SERVO_MIN_PULSE_US;
    if (pulse_us > PIN_CONFIG_SERVO_MAX_PULSE_US) pulse_us = PIN_CONFIG_SERVO_MAX_PULSE_US;

    float duty_fraction = (float)pulse_us / (float)SERVO_PERIOD_US;
    analogWriteFrequency(gpio, PIN_CONFIG_SERVO_PWM_FREQ);
    uint32_t level = (uint32_t)(duty_fraction * PWM_MAX_VALUE);
    analogWrite(gpio, level);

    // Best-effort inverse map to record where -1..+1 the pulse sits.
    float v;
    uint16_t c = cfg->params.servo.center_us;
    if (pulse_us == c) {
        v = 0.0f;
    } else if (pulse_us < c) {
        uint16_t s = cfg->params.servo.start_us;
        v = (s == c) ? 0.0f : ((float)pulse_us - (float)c) / ((float)c - (float)s);
    } else {
        uint16_t e = cfg->params.servo.end_us;
        v = (e == c) ? 0.0f : ((float)pulse_us - (float)c) / ((float)e - (float)c);
    }
    pin_runtime_value_t* rv = find_or_create_runtime(gpio);
    if (rv) {
        rv->value = v;
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
                // Safe-reset drives to the operator-configured home_us
                // (defaults to center_us if unset). Boot-time apply
                // uses the same helper.
                pin_control_drive_servo_pulse(cfg->gpio, cfg->params.servo.home_us);
                Serial.printf("ESTOP: GPIO %d (Servo) -> %u µs (home)\n",
                              cfg->gpio, (unsigned)cfg->params.servo.home_us);
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

/* Extract a JSON string field's value into `out`. Pointer-only parser
 * matching the style used by dispatch_action_buffer in main.cpp —
 * cheap and reliable enough for the small set_channel envelope. */
static bool extract_str_field(const char* json, const char* key,
                              char* out, size_t out_size)
{
    const char* p = strstr(json, key);
    if (!p) return false;
    p = strchr(p, ':');
    if (!p) return false;
    p++;
    while (*p == ' ' || *p == '\t') p++;
    if (*p != '"') return false;
    p++;
    size_t n = 0;
    while (*p && *p != '"' && n + 1 < out_size) {
        out[n++] = *p++;
    }
    out[n] = '\0';
    return n > 0;
}

/* Route a set_channel write for the onboard LED (peripheral type
 * "neopixel" — same id used for the Feather's WS2812 so the dashboard
 * widget targets either platform identically). RGB and brightness
 * are tracked independently in led_status.cpp so the color picker
 * and brightness slider can move on their own. The actual pin write
 * still collapses to on/off — that's done inside led_update(). */
static bool apply_neopixel_channel(const char* channel_id, float value)
{
    if (strcmp(channel_id, "color") == 0) {
        uint32_t packed = (uint32_t)value;
        uint8_t r = (uint8_t)((packed >> 16) & 0xFF);
        uint8_t g = (uint8_t)((packed >>  8) & 0xFF);
        uint8_t b = (uint8_t)( packed        & 0xFF);
        led_set_override_color(r, g, b, 255);
        Serial.printf("LED: override color via set_channel = (%u,%u,%u) "
                       "(collapsed to %s)\n",
                       (unsigned)r, (unsigned)g, (unsigned)b,
                       ((r | g | b) != 0) ? "ON" : "OFF");
        return true;
    }
    if (strcmp(channel_id, "brightness") == 0) {
        if (value < 0.0f) value = 0.0f;
        if (value > 1.0f) value = 1.0f;
        uint8_t b8 = (uint8_t)(value * 255.0f + 0.5f);
        led_set_override_brightness(b8);
        Serial.printf("LED: override brightness via set_channel = %u (%.3f)\n",
                       (unsigned)b8, value);
        return true;
    }
    /* "state" — single binary channel that the State tab uses for
     * mono LEDs that don't have a color picker. value > 0.5 → ON. */
    if (strcmp(channel_id, "state") == 0) {
        if (value > 0.5f) {
            led_set_override_color(255, 255, 255, 255);
        } else {
            led_set_override_color(255, 255, 255, 0);
        }
        Serial.printf("LED: override state via set_channel = %s\n",
                       (value > 0.5f) ? "ON" : "OFF");
        return true;
    }
    Serial.printf("LED: unknown channel '%s' on set_channel\n", channel_id);
    return false;
}

/* Dispatch a `set_channel` write. Mirrors RP2040's apply_set_channel
 * shape: parse {peripheral, type, channel, value}, route to the
 * NeoPixel override path for type=="neopixel", otherwise (eventually)
 * walk pin_config to find the peripheral's slot. Today only the
 * NeoPixel route is wired up on Teensy — other peripherals use
 * apply_*_channel paths through pin_config_apply_json's set_pin
 * branch or via their own driver-specific actions. */
static bool apply_set_channel(const char* json)
{
    char peripheral_id[PIN_CONFIG_MAX_NAME_LEN];
    char channel_id[PIN_CONFIG_MAX_NAME_LEN];
    char peripheral_type[PIN_CONFIG_MAX_NAME_LEN];

    if (!extract_str_field(json, "\"peripheral\"", peripheral_id, sizeof(peripheral_id))) {
        Serial.printf("set_channel: missing 'peripheral' field\n");
        return false;
    }
    if (!extract_str_field(json, "\"channel\"", channel_id, sizeof(channel_id))) {
        Serial.printf("set_channel: missing 'channel' field\n");
        return false;
    }
    peripheral_type[0] = '\0';
    extract_str_field(json, "\"type\"", peripheral_type, sizeof(peripheral_type));

    /* Optional absolute-microseconds jog ("us"): the dashboard's live
     * extent-dial preview drives the servo to a raw pulse so the
     * operator can dial in start/end/center/home visually. It bypasses
     * the normalized value→pulse mapping and the per-channel software
     * clamp (Maestro only — routed to maestro_set_target_preview below).
     * When "us" is present, "value" is optional. */
    long preview_us = -1;
    const char* us_str = strstr(json, "\"us\"");
    if (us_str) {
        us_str = strchr(us_str, ':');
        if (us_str) {
            us_str++;
            while (*us_str == ' ') us_str++;
            preview_us = atol(us_str);
        }
    }

    float value = 0.0f;
    bool has_value = false;
    const char* value_str = strstr(json, "\"value\"");
    if (value_str) {
        value_str = strchr(value_str, ':');
        if (value_str) {
            value_str++;
            while (*value_str == ' ') value_str++;
            value = (float)atof(value_str);
            has_value = true;
        }
    }
    if (!has_value && preview_us < 0) {
        Serial.printf("set_channel: missing both 'value' and 'us'\n");
        return false;
    }

    /* On-board LED routes — both `neopixel` (RGB) and `mono_led`
     * (single-color, channels = state + brightness) end up in the
     * same led_status override path. apply_neopixel_channel handles
     * all three channel ids ("color", "brightness", "state"); the
     * difference between the two peripheral types is whether the
     * dashboard exposes a color picker, not how the firmware writes
     * the pin. Id-substring fallback covers older servers and the
     * builtin id our own seed uses. */
    if (strcmp(peripheral_type, "mono_led") == 0
        || strcmp(peripheral_type, "neopixel") == 0
        || strstr(peripheral_id, "neopixel") != NULL
        || strstr(peripheral_id, "onboard_led") != NULL) {
        return apply_neopixel_channel(channel_id, value);
    }

    /* Pin-config walk: find the smallest GPIO whose logical_name
     * matches peripheral_id — that's the base channel slot for the
     * peripheral. Multi-channel peripherals (Maestro's 24 servos,
     * RoboClaw's motor/encoder/voltage/etc) live as one pin_config
     * entry PER channel on Teensy, with the same logical_name; the
     * channel_id suffix tells us which slot to write. Matches the
     * RP2040 implementation. */
    uint8_t count = 0;
    const pin_config_t* configs = pin_config_get_all(&count);
    int base_gpio = -1;
    pin_mode_t mode = PIN_MODE_UNCONFIGURED;
    for (uint8_t i = 0; i < count; i++) {
        if (configs[i].mode == PIN_MODE_UNCONFIGURED) continue;
        if (strcmp(configs[i].logical_name, peripheral_id) != 0) continue;
        if (base_gpio < 0 || configs[i].gpio < base_gpio) {
            base_gpio = configs[i].gpio;
            mode = configs[i].mode;
        }
    }
    if (base_gpio < 0) {
        Serial.printf("set_channel: peripheral '%s' (type '%s') not in pin_config\n",
                       peripheral_id, peripheral_type);
        return false;
    }
    Serial.printf("set_channel: %s/%s = %.3f -> base_gpio=%d mode=%d\n",
                  peripheral_id, channel_id, value, base_gpio, (int)mode);

    /* channel_id → offset. For Maestro (24 channels) and other
     * multi-channel peripherals the server uses "chN" — strip the
     * prefix and parse the integer. Single-channel peripherals (PWM,
     * SERVO, DIGITAL_OUT) ignore the channel id and use offset 0. */
    int offset = 0;
    if (mode == PIN_MODE_MAESTRO_SERVO) {
        const char* digits = channel_id;
        if (digits[0] == 'c' && digits[1] == 'h') digits += 2;
        if (*digits < '0' || *digits > '9') {
            Serial.printf("set_channel: Maestro channel '%s' is not 'chN'\n",
                           channel_id);
            return false;
        }
        offset = atoi(digits);
    } else if (mode == PIN_MODE_PWM || mode == PIN_MODE_SERVO
            || mode == PIN_MODE_DIGITAL_OUT) {
        offset = 0;
    } else {
        Serial.printf("set_channel: mode %d not yet routable on Teensy "
                       "(peripheral=%s channel=%s)\n",
                       (int)mode, peripheral_id, channel_id);
        return false;
    }

    /* Live extent-dial jog: drive the Maestro channel to an absolute
     * pulse, bypassing the normalized mapping + per-channel clamp. */
    if (preview_us >= 0 && mode == PIN_MODE_MAESTRO_SERVO) {
        Serial.printf("set_channel: Maestro ch %d live jog -> %ld us (preview)\n",
                      offset, preview_us);
        bool ok = maestro_set_target_preview((uint8_t)offset, (uint16_t)preview_us);
        Serial.printf("set_channel: preview jog -> %s\n", ok ? "OK" : "FAIL");
        return ok;
    }

    uint8_t target_gpio = (uint8_t)(base_gpio + offset);
    Serial.printf("set_channel: dispatching to GPIO %u (offset=%d)\n",
                  (unsigned)target_gpio, offset);
    bool ok = pin_control_set_value(target_gpio, value);
    Serial.printf("set_channel: pin_control_set_value -> %s\n", ok ? "OK" : "FAIL");
    return ok;
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

    /* set_channel — peripheral-first wire shape used by the State
     * tab's per-peripheral widgets (color picker / brightness slider
     * / on-off toggle). Handled BEFORE set_pin so a set_channel
     * envelope doesn't get mis-routed when both an action field and
     * a peripheral field are present. */
    if (strstr(json, "\"action\":\"set_channel\"") ||
        strstr(json, "\"action\": \"set_channel\"")) {
        return apply_set_channel(json);
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

    // Close legacy GPIO-indexed pins[] array.
    ret = snprintf(buffer + written, buffer_size - written, "]");
    if (ret < 0 || (size_t)ret >= buffer_size - written) return -1;
    written += ret;

    // Peripheral-first channel-addressed records (Phase 1 of
    // docs/PERIPHERAL_FIRST_MIGRATION.md). Drivers that have
    // migrated populate a parallel "channels":[...] array of
    // {peripheral_id, channel_id, value} objects, dispatched
    // through peripheral_state_emit_all_channels. Drivers still
    // on the GPIO-keyed path emit nothing here. The array is
    // unconditionally rendered so the JSON shape stays stable
    // even when no driver has migrated yet.
    ret = snprintf(buffer + written, buffer_size - written, ",\"channels\":[");
    if (ret < 0 || (size_t)ret >= buffer_size - written) return -1;
    written += ret;
    int ch_n = peripheral_state_emit_all_channels(
        buffer + written, buffer_size - (size_t)written);
    if (ch_n < 0) return -1;
    written += ch_n;
    ret = snprintf(buffer + written, buffer_size - written, "]}");
    if (ret < 0 || (size_t)ret >= buffer_size - written) return -1;
    written += ret;

    return written;
}

} // extern "C"
