/**
 * SAINT.OS Node Firmware - Pololu Maestro-24 USB Host Driver
 *
 * Uses USBHost_t36 to communicate with a Pololu Maestro-24 connected
 * to the Teensy 4.1 USB host port via the Compact Protocol.
 */

#include <Arduino.h>

#ifndef SIMULATION
#include <USBHost_t36.h>
#endif

extern "C" {
#include "maestro_driver.h"
#include "peripheral_driver.h"
}

// =============================================================================
// USB Host Objects (hardware only)
// =============================================================================

#ifndef SIMULATION
static USBHost myusb;
static USBHub hub1(myusb);
static USBSerial_BigBuffer maestroSerial(myusb);
#endif

// =============================================================================
// State
// =============================================================================

static bool maestro_initialized = false;
static bool maestro_connected = false;
static uint32_t last_connect_check = 0;

static maestro_channel_config_t channel_configs[MAESTRO_MAX_CHANNELS];

// =============================================================================
// Helpers
// =============================================================================

/**
 * Write bytes to the Maestro serial port.
 * Returns true if all bytes were written.
 */
static bool maestro_write(const uint8_t* data, size_t len)
{
#ifndef SIMULATION
    if (!maestro_connected) return false;
    size_t written = maestroSerial.write(data, len);
    return written == len;
#else
    (void)data;
    (void)len;
    return false;
#endif
}

/**
 * Read bytes from the Maestro serial port.
 * Returns number of bytes read.
 */
static size_t maestro_read(uint8_t* data, size_t len, uint32_t timeout_ms)
{
#ifndef SIMULATION
    if (!maestro_connected) return 0;

    size_t count = 0;
    uint32_t start = millis();
    while (count < len && (millis() - start) < timeout_ms) {
        if (maestroSerial.available()) {
            data[count++] = maestroSerial.read();
        }
    }
    return count;
#else
    (void)data;
    (void)len;
    (void)timeout_ms;
    return 0;
#endif
}

// =============================================================================
// Public API
// =============================================================================

extern "C" {

void maestro_init(void)
{
    if (maestro_initialized) return;

    // Initialize channel configs with defaults
    for (int i = 0; i < MAESTRO_MAX_CHANNELS; i++) {
        channel_configs[i].min_pulse_us = MAESTRO_DEFAULT_MIN_PULSE;
        channel_configs[i].max_pulse_us = MAESTRO_DEFAULT_MAX_PULSE;
        channel_configs[i].neutral_us   = MAESTRO_DEFAULT_NEUTRAL;
        channel_configs[i].speed        = 0;
        channel_configs[i].acceleration = 0;
        channel_configs[i].home_us      = 0;
    }

#ifndef SIMULATION
    myusb.begin();
    Serial.printf("Maestro: USB host initialized, waiting for device...\n");
#else
    Serial.printf("Maestro: simulation mode (no USB host)\n");
#endif

    maestro_initialized = true;
}

void maestro_update(void)
{
    if (!maestro_initialized) return;

#ifndef SIMULATION
    myusb.Task();

    // Periodic connection check
    uint32_t now = millis();
    if (now - last_connect_check >= 500) {
        last_connect_check = now;

        bool was_connected = maestro_connected;
        maestro_connected = (bool)maestroSerial;

        if (maestro_connected && !was_connected) {
            Serial.printf("Maestro: device connected\n");
            // Clear any pending errors
            maestro_get_errors();
        } else if (!maestro_connected && was_connected) {
            Serial.printf("Maestro: device disconnected\n");
        }
    }
#endif
}

bool maestro_is_connected(void)
{
    return maestro_connected;
}

bool maestro_set_target(uint8_t channel, uint16_t quarter_us)
{
    if (channel >= MAESTRO_MAX_CHANNELS) return false;

    uint8_t cmd[4];
    cmd[0] = 0x84;                          // Set Target command
    cmd[1] = channel;
    cmd[2] = quarter_us & 0x7F;             // Low 7 bits
    cmd[3] = (quarter_us >> 7) & 0x7F;      // High 7 bits

    return maestro_write(cmd, sizeof(cmd));
}

bool maestro_set_speed(uint8_t channel, uint16_t speed)
{
    if (channel >= MAESTRO_MAX_CHANNELS) return false;

    uint8_t cmd[4];
    cmd[0] = 0x87;                          // Set Speed command
    cmd[1] = channel;
    cmd[2] = speed & 0x7F;
    cmd[3] = (speed >> 7) & 0x7F;

    return maestro_write(cmd, sizeof(cmd));
}

bool maestro_set_acceleration(uint8_t channel, uint16_t accel)
{
    if (channel >= MAESTRO_MAX_CHANNELS) return false;

    uint8_t cmd[4];
    cmd[0] = 0x89;                          // Set Acceleration command
    cmd[1] = channel;
    cmd[2] = accel & 0x7F;
    cmd[3] = (accel >> 7) & 0x7F;

    return maestro_write(cmd, sizeof(cmd));
}

uint16_t maestro_get_position(uint8_t channel)
{
    if (channel >= MAESTRO_MAX_CHANNELS) return 0;

    uint8_t cmd[2];
    cmd[0] = 0x90;                          // Get Position command
    cmd[1] = channel;

    if (!maestro_write(cmd, sizeof(cmd))) return 0;

    uint8_t response[2];
    if (maestro_read(response, 2, 10) != 2) return 0;

    return response[0] | ((uint16_t)response[1] << 8);
}

uint16_t maestro_get_errors(void)
{
    uint8_t cmd = 0xA1;                     // Get Errors command

    if (!maestro_write(&cmd, 1)) return 0xFFFF;

    uint8_t response[2];
    if (maestro_read(response, 2, 10) != 2) return 0xFFFF;

    return response[0] | ((uint16_t)response[1] << 8);
}

void maestro_go_home(void)
{
    uint8_t cmd = 0xA2;                     // Go Home command
    maestro_write(&cmd, 1);
}

uint8_t maestro_get_channel_count(void)
{
    return MAESTRO_MAX_CHANNELS;
}

void maestro_set_channel_config(uint8_t channel, const maestro_channel_config_t* config)
{
    if (channel >= MAESTRO_MAX_CHANNELS || !config) return;
    channel_configs[channel] = *config;

    // Apply speed and acceleration to the Maestro hardware if connected
    if (maestro_connected) {
        if (config->speed > 0) {
            maestro_set_speed(channel, config->speed);
        }
        if (config->acceleration > 0) {
            maestro_set_acceleration(channel, config->acceleration);
        }
    }
}

const maestro_channel_config_t* maestro_get_channel_config(uint8_t channel)
{
    if (channel >= MAESTRO_MAX_CHANNELS) return NULL;
    return &channel_configs[channel];
}

uint16_t maestro_angle_to_target(float angle, const maestro_channel_config_t* config)
{
    if (!config) return 0;

    // Clamp angle to 0-180
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    // Linear map: 0 deg -> min_pulse, 180 deg -> max_pulse
    float pulse_us = config->min_pulse_us +
                     (angle / 180.0f) * (config->max_pulse_us - config->min_pulse_us);

    // Convert microseconds to quarter-microseconds
    return (uint16_t)(pulse_us * 4.0f);
}

// =============================================================================
// Peripheral Driver Interface
// =============================================================================

static bool maestro_drv_init(void)
{
    maestro_init();
    return true;
}

static bool maestro_drv_set_value(uint8_t channel, float value)
{
    if (channel >= MAESTRO_MAX_CHANNELS) return false;

    // value is angle 0-180
    float angle = value;
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    const maestro_channel_config_t* mcfg = maestro_get_channel_config(channel);
    if (!mcfg) return false;

    uint16_t target = maestro_angle_to_target(angle, mcfg);
    return maestro_set_target(channel, target);
}

static bool maestro_drv_get_value(uint8_t channel, float* value)
{
    if (channel >= MAESTRO_MAX_CHANNELS || !value) return false;

    if (!maestro_is_connected()) return false;

    uint16_t pos_qus = maestro_get_position(channel);
    if (pos_qus == 0) return false;

    const maestro_channel_config_t* mcfg = maestro_get_channel_config(channel);
    if (!mcfg || mcfg->max_pulse_us <= mcfg->min_pulse_us) return false;

    float pulse_us = pos_qus / 4.0f;
    float angle = (pulse_us - mcfg->min_pulse_us) /
                  (float)(mcfg->max_pulse_us - mcfg->min_pulse_us) * 180.0f;
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;
    *value = angle;
    return true;
}

static void maestro_drv_set_defaults(uint8_t channel, pin_config_t* config)
{
    (void)channel;
    if (!config) return;
    config->params.maestro.min_pulse_us  = MAESTRO_DEFAULT_MIN_PULSE;
    config->params.maestro.max_pulse_us  = MAESTRO_DEFAULT_MAX_PULSE;
    config->params.maestro.neutral_us    = MAESTRO_DEFAULT_NEUTRAL;
    config->params.maestro.speed         = 0;
    config->params.maestro.acceleration  = 0;
    config->params.maestro.home_us       = 0;
}

static bool maestro_drv_apply_config(uint8_t channel, const pin_config_t* config)
{
    if (channel >= MAESTRO_MAX_CHANNELS || !config) return false;

    maestro_channel_config_t mcfg;
    mcfg.min_pulse_us  = config->params.maestro.min_pulse_us;
    mcfg.max_pulse_us  = config->params.maestro.max_pulse_us;
    mcfg.neutral_us    = config->params.maestro.neutral_us;
    mcfg.speed         = config->params.maestro.speed;
    mcfg.acceleration  = config->params.maestro.acceleration;
    mcfg.home_us       = config->params.maestro.home_us;
    maestro_set_channel_config(channel, &mcfg);
    return true;
}

static bool maestro_drv_parse_json(const char* json_start, const char* json_end,
                                    pin_config_t* config)
{
    if (!json_start || !json_end || !config) return false;

    uint16_t min_p = MAESTRO_DEFAULT_MIN_PULSE;
    uint16_t max_p = MAESTRO_DEFAULT_MAX_PULSE;
    uint16_t neut  = MAESTRO_DEFAULT_NEUTRAL;
    uint16_t spd   = 0;
    uint16_t acc   = 0;
    uint16_t home  = 0;

    const char* p;
    p = strstr(json_start, "\"min_pulse_us\"");
    if (p && p < json_end) { p = strchr(p, ':'); if (p) { p++; while (*p == ' ') p++; min_p = (uint16_t)atoi(p); } }
    p = strstr(json_start, "\"max_pulse_us\"");
    if (p && p < json_end) { p = strchr(p, ':'); if (p) { p++; while (*p == ' ') p++; max_p = (uint16_t)atoi(p); } }
    p = strstr(json_start, "\"neutral_us\"");
    if (p && p < json_end) { p = strchr(p, ':'); if (p) { p++; while (*p == ' ') p++; neut = (uint16_t)atoi(p); } }
    p = strstr(json_start, "\"speed\"");
    if (p && p < json_end) { p = strchr(p, ':'); if (p) { p++; while (*p == ' ') p++; spd = (uint16_t)atoi(p); } }
    p = strstr(json_start, "\"acceleration\"");
    if (p && p < json_end) { p = strchr(p, ':'); if (p) { p++; while (*p == ' ') p++; acc = (uint16_t)atoi(p); } }
    p = strstr(json_start, "\"home_us\"");
    if (p && p < json_end) { p = strchr(p, ':'); if (p) { p++; while (*p == ' ') p++; home = (uint16_t)atoi(p); } }

    config->params.maestro.min_pulse_us  = min_p;
    config->params.maestro.max_pulse_us  = max_p;
    config->params.maestro.neutral_us    = neut;
    config->params.maestro.speed         = spd;
    config->params.maestro.acceleration  = acc;
    config->params.maestro.home_us       = home;
    return true;
}

static void maestro_drv_estop(void)
{
    if (maestro_is_connected()) {
        maestro_go_home();
    }
}

static int maestro_drv_caps_json(uint8_t channel, char* buf, size_t remaining)
{
    if (channel >= MAESTRO_MAX_CHANNELS || !buf) return -1;

    char name[8];
    int gpio = MAESTRO_VIRTUAL_GPIO_BASE + channel;
    snprintf(name, sizeof(name), "M%d", channel);

    return snprintf(buf, remaining,
        "{\"gpio\":%d,\"name\":\"%s\",\"capabilities\":[\"maestro_servo\"]}",
        gpio, name);
}

static bool maestro_drv_save(void* storage)
{
    flash_storage_data_t* s = (flash_storage_data_t*)storage;
    if (!s) return false;

    memset(&s->maestro_config, 0, sizeof(s->maestro_config));
    s->maestro_config.channel_count = MAESTRO_MAX_CHANNELS;
    for (uint8_t ch = 0; ch < MAESTRO_MAX_CHANNELS; ch++) {
        const maestro_channel_config_t* mcfg = maestro_get_channel_config(ch);
        if (mcfg) {
            s->maestro_config.channels[ch].min_pulse_us  = mcfg->min_pulse_us;
            s->maestro_config.channels[ch].max_pulse_us  = mcfg->max_pulse_us;
            s->maestro_config.channels[ch].neutral_us    = mcfg->neutral_us;
            s->maestro_config.channels[ch].speed         = mcfg->speed;
            s->maestro_config.channels[ch].acceleration  = mcfg->acceleration;
            s->maestro_config.channels[ch].home_us       = mcfg->home_us;
        }
    }
    return true;
}

static bool maestro_drv_load(const void* storage)
{
    const flash_storage_data_t* s = (const flash_storage_data_t*)storage;
    if (!s) return false;

    if (s->maestro_config.channel_count > 0) {
        uint8_t count_m = s->maestro_config.channel_count;
        if (count_m > MAESTRO_MAX_CHANNELS) count_m = MAESTRO_MAX_CHANNELS;
        for (uint8_t ch = 0; ch < count_m; ch++) {
            maestro_channel_config_t mcfg;
            mcfg.min_pulse_us  = s->maestro_config.channels[ch].min_pulse_us;
            mcfg.max_pulse_us  = s->maestro_config.channels[ch].max_pulse_us;
            mcfg.neutral_us    = s->maestro_config.channels[ch].neutral_us;
            mcfg.speed         = s->maestro_config.channels[ch].speed;
            mcfg.acceleration  = s->maestro_config.channels[ch].acceleration;
            mcfg.home_us       = s->maestro_config.channels[ch].home_us;
            if (mcfg.min_pulse_us > 0 || mcfg.max_pulse_us > 0) {
                maestro_set_channel_config(ch, &mcfg);
            }
        }
        Serial.printf("Maestro: restored %d channel configs from flash\n", count_m);
    }
    return true;
}

static const peripheral_driver_t maestro_peripheral = {
    .name             = "maestro",
    .mode_string      = "maestro_servo",
    .pin_mode         = PIN_MODE_MAESTRO_SERVO,
    .capability_flag  = PIN_CAP_MAESTRO_SERVO,
    .virtual_gpio_base = MAESTRO_VIRTUAL_GPIO_BASE,
    .channel_count    = MAESTRO_MAX_CHANNELS,
    .init             = maestro_drv_init,
    .update           = maestro_update,
    .is_connected     = maestro_is_connected,
    .set_value        = maestro_drv_set_value,
    .get_value        = maestro_drv_get_value,
    .set_defaults     = maestro_drv_set_defaults,
    .apply_config     = maestro_drv_apply_config,
    .parse_json_params = maestro_drv_parse_json,
    .estop            = maestro_drv_estop,
    .capabilities_to_json = maestro_drv_caps_json,
    .save_config      = maestro_drv_save,
    .load_config      = maestro_drv_load,
};

const peripheral_driver_t* maestro_get_peripheral_driver(void)
{
    return &maestro_peripheral;
}

} // extern "C"
