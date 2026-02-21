/**
 * SAINT.OS Node Firmware - SyRen 50 Motor Controller Driver (RP2040)
 *
 * Uses Pico SDK UART for packetized serial communication with up to 8
 * SyRen 50 motor controllers on addresses 128-135.
 */

#include "syren_driver.h"
#include "peripheral_driver.h"
#include "flash_types.h"
#include "platform.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#ifndef SIMULATION
#include "hardware/uart.h"
#include "hardware/gpio.h"
#endif

// =============================================================================
// Configuration
// =============================================================================

// Default UART instance and pins for RP2040
#define SYREN_UART          uart1
#define SYREN_UART_TX_PIN   4
#define SYREN_SERIAL_PORT   1

// =============================================================================
// State
// =============================================================================

static syren_channel_config_t channel_configs[SYREN_MAX_CHANNELS];
static bool port_initialized = false;
static uint16_t configured_baud = SYREN_DEFAULT_BAUD;

// =============================================================================
// Internal Helpers
// =============================================================================

static void send_packet(uint8_t address, uint8_t command, uint8_t value)
{
    uint8_t packet[4];
    syren_build_packet(address, command, value, packet);

#ifndef SIMULATION
    uart_write_blocking(SYREN_UART, packet, 4);
#endif
}

// =============================================================================
// Public API
// =============================================================================

void syren_init(void)
{
    // Set default channel configs
    for (uint8_t i = 0; i < SYREN_MAX_CHANNELS; i++) {
        channel_configs[i].address = SYREN_ADDRESS_MIN + i;
        channel_configs[i].deadband = 0;
        channel_configs[i].ramping = 0;
        channel_configs[i].timeout_ms = 0;
    }

#ifndef SIMULATION
    uart_init(SYREN_UART, configured_baud);
    gpio_set_function(SYREN_UART_TX_PIN, GPIO_FUNC_UART);

    PLATFORM_SLEEP_MS(100);

    // Send autobaud byte
    uint8_t autobaud = SYREN_AUTOBAUD_BYTE;
    uart_write_blocking(SYREN_UART, &autobaud, 1);
    PLATFORM_SLEEP_MS(50);
#endif

    port_initialized = true;
    PLATFORM_PRINTF("SyRen: initialized on UART%d at %d baud\n",
                    SYREN_SERIAL_PORT, configured_baud);
}

void syren_update(void)
{
    // No async polling needed for SyRen
}

bool syren_is_connected(void)
{
    return port_initialized;
}

bool syren_set_power(uint8_t channel, int16_t power)
{
    if (channel >= SYREN_MAX_CHANNELS || !port_initialized) return false;

    // Clamp power
    if (power > 127) power = 127;
    if (power < -127) power = -127;

    uint8_t address = channel_configs[channel].address;
    uint8_t command;
    uint8_t value;

    if (power >= 0) {
        command = SYREN_CMD_MOTOR_FWD;
        value = (uint8_t)power;
    } else {
        command = SYREN_CMD_MOTOR_REV;
        value = (uint8_t)(-power);
    }

#ifndef SIMULATION
    send_packet(address, command, value);
#endif

    return true;
}

bool syren_stop(uint8_t channel)
{
    return syren_set_power(channel, 0);
}

void syren_stop_all(void)
{
    for (uint8_t i = 0; i < SYREN_MAX_CHANNELS; i++) {
        syren_set_power(i, 0);
    }
}

void syren_set_channel_config(uint8_t channel, const syren_channel_config_t* config)
{
    if (channel >= SYREN_MAX_CHANNELS || !config) return;
    channel_configs[channel] = *config;

    if (port_initialized) {
#ifndef SIMULATION
        uint8_t addr = config->address;
        if (config->deadband > 0) {
            send_packet(addr, SYREN_CMD_DEADBAND, config->deadband);
        }
        if (config->ramping > 0) {
            send_packet(addr, SYREN_CMD_RAMPING, config->ramping);
        }
        if (config->timeout_ms > 0) {
            uint8_t timeout_val = (uint8_t)(config->timeout_ms / 100);
            if (timeout_val > 127) timeout_val = 127;
            send_packet(addr, SYREN_CMD_SERIAL_TIMEOUT, timeout_val);
        }
#endif
    }
}

const syren_channel_config_t* syren_get_channel_config(uint8_t channel)
{
    if (channel >= SYREN_MAX_CHANNELS) return NULL;
    return &channel_configs[channel];
}

// =============================================================================
// Peripheral Driver Interface
// =============================================================================

static bool syren_drv_init(void)
{
    syren_init();
    return true;
}

static bool syren_drv_set_value(uint8_t channel, float value)
{
    if (value < -1.0f) value = -1.0f;
    if (value > 1.0f) value = 1.0f;
    int16_t power = (int16_t)(value * 127.0f);
    return syren_set_power(channel, power);
}

static bool syren_drv_get_value(uint8_t channel, float* value)
{
    (void)channel;
    (void)value;
    return false;
}

static void syren_drv_set_defaults(uint8_t channel, pin_config_t* config)
{
    config->params.syren.address = SYREN_ADDRESS_MIN + channel;
    config->params.syren.deadband = 0;
    config->params.syren.ramping = 0;
    config->params.syren.reserved = 0;
    config->params.syren.timeout_ms = 0;
}

static bool syren_drv_apply_config(uint8_t channel, const pin_config_t* config)
{
    if (channel >= SYREN_MAX_CHANNELS) return false;

    syren_channel_config_t scfg;
    scfg.address = config->params.syren.address;
    scfg.deadband = config->params.syren.deadband;
    scfg.ramping = config->params.syren.ramping;
    scfg.timeout_ms = config->params.syren.timeout_ms;
    syren_set_channel_config(channel, &scfg);
    return true;
}

static bool syren_drv_parse_json(const char* json_start, const char* json_end,
                                  pin_config_t* config)
{
    const char* p;

    p = strstr(json_start, "\"address\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) { p++; while (*p == ' ') p++; config->params.syren.address = (uint8_t)atoi(p); }
    }

    p = strstr(json_start, "\"deadband\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) { p++; while (*p == ' ') p++; config->params.syren.deadband = (uint8_t)atoi(p); }
    }

    p = strstr(json_start, "\"ramping\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) { p++; while (*p == ' ') p++; config->params.syren.ramping = (uint8_t)atoi(p); }
    }

    p = strstr(json_start, "\"timeout_ms\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) { p++; while (*p == ' ') p++; config->params.syren.timeout_ms = (uint16_t)atoi(p); }
    }

    return true;
}

static void syren_drv_estop(void)
{
    syren_stop_all();
    PLATFORM_PRINTF("SyRen: ESTOP - all channels stopped\n");
}

static int syren_drv_caps_json(uint8_t channel, char* buf, size_t remaining)
{
    uint8_t gpio = SYREN_VIRTUAL_GPIO_BASE + channel;
    return snprintf(buf, remaining,
        "{\"gpio\":%d,\"name\":\"SY%d\",\"capabilities\":[\"syren_motor\"]}",
        gpio, channel);
}

static bool syren_drv_save(void* storage_ptr)
{
    flash_storage_data_t* storage = (flash_storage_data_t*)storage_ptr;

    memset(&storage->syren_config, 0, sizeof(storage->syren_config));
    storage->syren_config.channel_count = SYREN_MAX_CHANNELS;
    storage->syren_config.serial_port = SYREN_SERIAL_PORT;
    storage->syren_config.baud_rate = configured_baud;

    for (uint8_t ch = 0; ch < SYREN_MAX_CHANNELS; ch++) {
        storage->syren_config.channels[ch].address    = channel_configs[ch].address;
        storage->syren_config.channels[ch].deadband   = channel_configs[ch].deadband;
        storage->syren_config.channels[ch].ramping    = channel_configs[ch].ramping;
        storage->syren_config.channels[ch].reserved_s = 0;
        storage->syren_config.channels[ch].timeout_ms = channel_configs[ch].timeout_ms;
    }

    return true;
}

static bool syren_drv_load(const void* storage_ptr)
{
    const flash_storage_data_t* storage = (const flash_storage_data_t*)storage_ptr;

    if (storage->syren_config.channel_count == 0) return true;

    uint8_t count = storage->syren_config.channel_count;
    if (count > SYREN_MAX_CHANNELS) count = SYREN_MAX_CHANNELS;

    if (storage->syren_config.baud_rate > 0) {
        configured_baud = storage->syren_config.baud_rate;
    }

    for (uint8_t ch = 0; ch < count; ch++) {
        channel_configs[ch].address    = storage->syren_config.channels[ch].address;
        channel_configs[ch].deadband   = storage->syren_config.channels[ch].deadband;
        channel_configs[ch].ramping    = storage->syren_config.channels[ch].ramping;
        channel_configs[ch].timeout_ms = storage->syren_config.channels[ch].timeout_ms;
    }

    PLATFORM_PRINTF("SyRen: restored %d channel configs from flash\n", count);
    return true;
}

// Static driver struct
static const peripheral_driver_t syren_peripheral = {
    .name              = "syren",
    .mode_string       = "syren_motor",
    .pin_mode          = PIN_MODE_SYREN_MOTOR,
    .capability_flag   = PIN_CAP_SYREN_MOTOR,
    .virtual_gpio_base = SYREN_VIRTUAL_GPIO_BASE,
    .channel_count     = SYREN_MAX_CHANNELS,
    .init              = syren_drv_init,
    .update            = syren_update,
    .is_connected      = syren_is_connected,
    .set_value         = syren_drv_set_value,
    .get_value         = syren_drv_get_value,
    .set_defaults      = syren_drv_set_defaults,
    .apply_config      = syren_drv_apply_config,
    .parse_json_params = syren_drv_parse_json,
    .estop             = syren_drv_estop,
    .capabilities_to_json = syren_drv_caps_json,
    .save_config       = syren_drv_save,
    .load_config       = syren_drv_load,
};

const peripheral_driver_t* syren_get_peripheral_driver(void)
{
    return &syren_peripheral;
}
