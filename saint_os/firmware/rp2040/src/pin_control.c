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
#include "peripheral_driver.h"
#include "saint_types.h"   // led_set_override_color / led_clear_override
#include "saint_log.h"     // saint_log_publish (dashboard Logs tab)

// =============================================================================
// Constants
// =============================================================================

#define MAX_RUNTIME_VALUES 16

// Servo timing constant
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

    uint32_t freq = (cfg->mode == PIN_MODE_SERVO)
        ? cfg->params.servo.frequency
        : cfg->params.pwm.frequency;
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
        {
            const peripheral_driver_t* drv = peripheral_find_by_mode(cfg->mode);
            if (drv && drv->set_value) {
                uint8_t ch = peripheral_gpio_to_channel(drv, gpio);
                bool ok = drv->set_value(ch, value);

                pin_runtime_value_t* rv = find_or_create_runtime(gpio);
                if (rv) {
                    rv->value = value;
                    rv->last_updated = to_ms_since_boot(get_absolute_time());
                }
                return ok;
            }

            printf("Pin control: GPIO %d mode %s not controllable\n",
                   gpio, pin_mode_to_string(cfg->mode));
            return false;
        }
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

    // Calculate pulse width using per-pin configurable range
    uint16_t min_us = cfg->params.servo.min_pulse_us;
    uint16_t max_us = cfg->params.servo.max_pulse_us;
    float pulse_us = min_us + (angle / 180.0f) * (max_us - min_us);
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
            {
                const peripheral_driver_t* drv = peripheral_find_by_mode(cfg->mode);
                if (drv && drv->get_value && drv->is_connected && drv->is_connected()) {
                    uint8_t ch = peripheral_gpio_to_channel(drv, cfg->gpio);
                    float val;
                    if (drv->get_value(ch, &val)) {
                        pin_runtime_value_t* rv = find_or_create_runtime(cfg->gpio);
                        if (rv) {
                            rv->value = val;
                            rv->last_updated = to_ms_since_boot(get_absolute_time());
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
    printf("ESTOP: Setting all outputs to safe values\n");

    // Get all configured pins
    uint8_t count;
    const pin_config_t* configs = pin_config_get_all(&count);

    for (uint8_t i = 0; i < count; i++) {
        const pin_config_t* cfg = &configs[i];

        switch (cfg->mode) {
            case PIN_MODE_PWM:
                // Set PWM to 0%
                pin_control_set_pwm(cfg->gpio, 0.0f);
                printf("ESTOP: GPIO %d (PWM) -> 0%%\n", cfg->gpio);
                break;

            case PIN_MODE_SERVO:
                // Set servo to center position (90 degrees) for safety
                // Some servos may be safer at 0, but center is generally safe
                pin_control_set_servo(cfg->gpio, 90.0f);
                printf("ESTOP: GPIO %d (Servo) -> 90 deg (center)\n", cfg->gpio);
                break;

            case PIN_MODE_DIGITAL_OUT:
                // Set digital outputs low
                pin_control_set_digital(cfg->gpio, false);
                printf("ESTOP: GPIO %d (Digital Out) -> LOW\n", cfg->gpio);
                break;

            default:
                // Input pins don't need ESTOP handling
                break;
        }
    }

    // Emergency stop all peripheral drivers
    peripheral_estop_all();
    printf("ESTOP: peripherals stopped\n");

    printf("ESTOP: Complete\n");
}

void pin_control_clear_estop(void)
{
    // Release the latch. Pins controlled directly via PWM/servo/digital
    // are NOT automatically restored to their pre-stop values — once
    // we've forced them to safe defaults we don't know what the
    // operator wanted them at, so the next set_value from the host
    // will bring them back. The peripheral drivers' clear_estop hooks
    // are the important part: they release hardware latches (e.g.
    // RoboClaw's S3 estop_pin returns to LOW so motor commands work
    // again) so motor controllers can be brought out of safety mode
    // without a power cycle.
    printf("ESTOP CLEAR: releasing peripheral latches\n");
    peripheral_clear_estop_all();
    printf("ESTOP CLEAR: Complete\n");
}

// Pull a quoted-string field value out of a JSON buffer. Returns true on
// success; `out` is null-terminated. Mirrors extract_string_field() in
// pin_config.c — duplicated rather than exposed because it's a small
// helper and giving it a public home would mean reshuffling headers.
static bool extract_str_field(const char* json, const char* key,
                              char* out, size_t out_size)
{
    const char* p = strstr(json, key);
    if (!p) return false;
    p = strchr(p, ':');
    if (!p) return false;
    p++;
    while (*p == ' ' || *p == '\t' || *p == '"') p++;
    size_t n = 0;
    while (*p && *p != '"' && n < out_size - 1) {
        out[n++] = *p++;
    }
    out[n] = '\0';
    return n > 0;
}

// Resolve a (mode, channel_name) into the firmware channel offset within
// the peripheral's virtual-GPIO slot. For mode-handled peripherals
// (PWM, servo, digital_out) there's only one channel — return 0
// regardless of name. For peripheral_driver_t-backed peripherals, the
// offset must match the driver's internal layout.
static int channel_offset_for(pin_mode_t mode, const char* channel)
{
    switch (mode) {
        case PIN_MODE_ROBOCLAW_MOTOR:
            if (strcmp(channel, "motor")   == 0) return 0;
            if (strcmp(channel, "encoder") == 0) return 1;
            if (strcmp(channel, "voltage") == 0) return 2;
            if (strcmp(channel, "current") == 0) return 3;
            if (strcmp(channel, "temp")    == 0) return 4;
            return -1;
        case PIN_MODE_SYREN_MOTOR:
            if (strcmp(channel, "motor") == 0) return 0;
            return -1;
        case PIN_MODE_FAS100_SENSOR:
            if (strcmp(channel, "amps")  == 0) return 0;
            if (strcmp(channel, "volts") == 0) return 1;
            if (strcmp(channel, "temp1") == 0) return 2;
            if (strcmp(channel, "temp2") == 0) return 3;
            return -1;
        case PIN_MODE_PATHFINDER_BMS:
            if (strcmp(channel, "pack_voltage") == 0) return 0;
            if (strcmp(channel, "current")      == 0) return 1;
            if (strcmp(channel, "soc")          == 0) return 2;
            if (strcmp(channel, "temp_1")       == 0) return 4;
            if (strcmp(channel, "temp_2")       == 0) return 5;
            return -1;
        case PIN_MODE_PWM:
        case PIN_MODE_SERVO:
        case PIN_MODE_DIGITAL_OUT:
        case PIN_MODE_DIGITAL_IN:
        case PIN_MODE_ADC:
            // Single-channel peripherals — channel name is informational.
            return 0;
        default:
            return -1;
    }
}

// The status NeoPixel isn't a pin_config entry — it's owned by
// led_status.c. Route set_channel writes for peripheral="neopixel"
// directly to the LED override API instead of trying to look it up
// via pin_config (which would fail and silently drop the write).
//
// JSON value semantics:
//   channel="color"      → packed uint24 RGB (0xRRGGBB), from the
//                          State tab color picker
//   channel="brightness" → 0..1 float; scaled to 0..255 and the
//                          current override color is kept
static bool apply_neopixel_channel(const char* channel_id, float value)
{
    if (strcmp(channel_id, "color") == 0) {
        // The State tab packs #RRGGBB into a uint24 carried in a float.
        // atof() round-trips integers up to 2^24 exactly, so no loss.
        uint32_t packed = (uint32_t)value;
        uint8_t r = (uint8_t)((packed >> 16) & 0xFF);
        uint8_t g = (uint8_t)((packed >>  8) & 0xFF);
        uint8_t b = (uint8_t)( packed        & 0xFF);
        led_set_override_color(r, g, b, 255);
        saint_log_publish("info",
            "NeoPixel: override RGB=(%u,%u,%u) via set_channel (server UI)",
            (unsigned)r, (unsigned)g, (unsigned)b);
        return true;
    }
    if (strcmp(channel_id, "brightness") == 0) {
        // Without a known active color, drawing a "brightness 0.5"
        // override would just paint dim-black. Refuse rather than
        // silently look wrong; the operator should set color first.
        // (When we later track override color alongside brightness
        // separately we can do better here.)
        saint_log_publish("warn",
            "NeoPixel: brightness-only override not implemented yet "
            "(value=%.3f). Set color first via the color picker.", value);
        return false;
    }
    saint_log_publish("warn",
        "NeoPixel: unknown channel '%s' on set_channel", channel_id);
    return false;
}

// Dispatch a `set_channel` write by walking pin_config to find the
// peripheral's slot, then computing the target GPIO from its mode +
// channel offset. The wire format is operator-visible names; the
// (peripheral_id, channel) → GPIO translation lives here in the firmware.
static bool apply_set_channel(const char* json)
{
    char peripheral_id[PIN_CONFIG_MAX_NAME_LEN];
    char channel_id[PIN_CONFIG_MAX_NAME_LEN];

    if (!extract_str_field(json, "\"peripheral\"", peripheral_id, sizeof(peripheral_id))) {
        saint_log_publish("warn", "set_channel: missing 'peripheral' field");
        return false;
    }
    if (!extract_str_field(json, "\"channel\"", channel_id, sizeof(channel_id))) {
        saint_log_publish("warn", "set_channel: missing 'channel' field");
        return false;
    }

    const char* value_str = strstr(json, "\"value\"");
    if (!value_str) return false;
    value_str = strchr(value_str, ':');
    if (!value_str) return false;
    value_str++;
    while (*value_str == ' ') value_str++;
    float value = (float)atof(value_str);

    // Optional peripheral type from server. Used to route writes for
    // peripherals that don't live in pin_config — the built-in status
    // NeoPixel is the canonical case (operator-chosen instance id
    // like "onboard_neopixel" but firmware only knows it as
    // type="neopixel"). Older servers don't send "type" — we fall
    // back to id-substring matching below.
    char peripheral_type[PIN_CONFIG_MAX_NAME_LEN];
    peripheral_type[0] = '\0';
    extract_str_field(json, "\"type\"", peripheral_type, sizeof(peripheral_type));

    // NeoPixel isn't a pin_config entry — special-case before the
    // logical_name walk below would unconditionally fail for it.
    // Prefer explicit type when present; otherwise match by id
    // substring so "neopixel", "onboard_neopixel", "neopixel_strip_1",
    // etc. route to the same status-LED override path.
    if (strcmp(peripheral_type, "neopixel") == 0
        || strstr(peripheral_id, "neopixel") != NULL) {
        return apply_neopixel_channel(channel_id, value);
    }

    // Find the smallest GPIO whose logical_name matches the peripheral.
    // For multi-channel peripherals that's the base slot — offsets are
    // applied on top of it.
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
        // Surface the miss in the Logs tab — previously a printf-only
        // diagnostic, which only operators with serial console access
        // ever saw. This was the silent failure mode behind "I'm
        // moving the slider and nothing happens."
        saint_log_publish("warn",
            "set_channel: peripheral '%s' is not in pin_config — "
            "has it been synced from the Peripherals tab?",
            peripheral_id);
        return false;
    }

    int offset = channel_offset_for(mode, channel_id);
    if (offset < 0) {
        saint_log_publish("warn",
            "set_channel: peripheral '%s' has no channel '%s' for mode %s",
            peripheral_id, channel_id, pin_mode_to_string(mode));
        return false;
    }

    // INTERNAL: The wire format from the server is already
    // (peripheral_id, channel_id) end-to-end. The remaining
    // virtual-GPIO arithmetic below (base_gpio + offset →
    // pin_control_set_value) is purely an internal dispatch detail —
    // pin_control_set_value also serves PWM/servo/digital_out modes
    // which have real GPIOs, so unifying the dispatch saves a parallel
    // path. The log line stays peripheral-first; no virtual GPIO
    // number leaks to the operator.
    //
    // TODO(peripheral-first): the per-driver set_value() still takes
    // a channel index computed from `gpio - drv->virtual_gpio_base`,
    // which is the last vestige of virtual-GPIO addressing inside the
    // firmware. Eliminating it requires drivers to expose a per-
    // instance lookup keyed by peripheral_id (instance name) instead
    // of by GPIO offset — bigger refactor, separate change.
    uint8_t target_gpio = (uint8_t)(base_gpio + offset);
    saint_log_publish("info",
        "set_channel %s/%s = %.3f",
        peripheral_id, channel_id, value);
    return pin_control_set_value(target_gpio, value);
}

bool pin_control_apply_json(const char* json, size_t json_len)
{
    if (!json || json_len == 0) return false;

    // SAFETY: Validate string is null-terminated within json_len
    // This prevents buffer over-reads if caller provides wrong length
    bool found_null = false;
    for (size_t i = 0; i <= json_len && i < 1024; i++) {
        if (json[i] == '\0') {
            found_null = true;
            break;
        }
    }
    if (!found_null) {
        printf("Pin control: JSON not null-terminated within length\n");
        return false;
    }

    // Channel-addressed write: operator names go through verbatim from
    // the server; firmware handles the (peripheral, channel) lookup.
    if (strstr(json, "\"action\":\"set_channel\"") ||
        strstr(json, "\"action\": \"set_channel\"")) {
        return apply_set_channel(json);
    }

    // Legacy GPIO-addressed write. Kept while existing call sites (eg.
    // the GPIO-keyed `set_pin_value` WS action) still use it.
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

        // If there's no runtime entry for this pin, the driver
        // either hasn't sampled yet (peripheral disconnected, init
        // problem, sensor silent) or this is an output pin no one
        // has read. Emit value:null so the UI can show "—" instead
        // of a misleading 0.000. find_or_create_runtime() is called
        // by pin_control_update_state() only when get_value()
        // succeeds, so missing rv == "no real reading."
        pin_runtime_value_t* rv = find_runtime(cfg->gpio);

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

        if (rv) {
            ret = snprintf(buffer + written, buffer_size - written,
                "%s{\"gpio\":%d,\"mode\":\"%s\",\"value\":%.2f,\"name\":\"%s\"",
                first ? "" : ",",
                cfg->gpio, mode_str, rv->value, escaped_name);
        } else {
            ret = snprintf(buffer + written, buffer_size - written,
                "%s{\"gpio\":%d,\"mode\":\"%s\",\"value\":null,\"name\":\"%s\"",
                first ? "" : ",",
                cfg->gpio, mode_str, escaped_name);
        }
        if (ret < 0 || (size_t)ret >= buffer_size - written) return -1;
        written += ret;

        // Add voltage for ADC pins (only when we actually have a reading).
        if (cfg->mode == PIN_MODE_ADC && rv) {
            ret = snprintf(buffer + written, buffer_size - written,
                ",\"voltage\":%.3f", rv->value);
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
