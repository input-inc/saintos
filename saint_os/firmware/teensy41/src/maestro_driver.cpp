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

} // extern "C"
