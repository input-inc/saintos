/**
 * SAINT.OS Node Firmware - RoboClaw Solo 60A Motor Controller Driver (Teensy 4.1)
 *
 * Uses HardwareSerial for CRC16 packet serial communication with up to 8
 * RoboClaw Solo 60A motor controllers on addresses 0x80-0x87.
 *
 * Virtual GPIO layout: 5 channels per unit (motor, encoder, voltage, current, temp).
 * Telemetry is polled round-robin to avoid blocking.
 */

#include <Arduino.h>

extern "C" {
#include "roboclaw_driver.h"
#include "peripheral_driver.h"
#include "flash_types.h"
}

// =============================================================================
// Configuration
// =============================================================================

// Default to Serial3 (pins 14/15 on Teensy 4.1)
#define ROBOCLAW_DEFAULT_SERIAL_PORT 3

// =============================================================================
// Per-Unit State
// =============================================================================

typedef struct {
    uint8_t address;
    uint8_t deadband;
    uint16_t max_current_ma;
    int16_t duty;
    int32_t encoder;
    uint16_t voltage_mv;
    int16_t current_ma;
    int16_t temp_tenths;
    bool connected;
} roboclaw_unit_t;

// =============================================================================
// Driver State
// =============================================================================

static roboclaw_unit_t units[ROBOCLAW_MAX_UNITS];
static uint8_t unit_count = 0;
static bool port_initialized = false;
static uint16_t configured_baud = ROBOCLAW_DEFAULT_BAUD;
static uint8_t configured_serial_port = ROBOCLAW_DEFAULT_SERIAL_PORT;

// Round-robin telemetry polling state
static uint8_t poll_unit = 0;
static uint8_t poll_register = 0;

// Pointer to the HardwareSerial instance in use
static HardwareSerial* hw_serial = nullptr;

// =============================================================================
// Serial Port Selection
// =============================================================================

static HardwareSerial* get_serial_port(uint8_t port)
{
    switch (port) {
        case 1: return &Serial1;
        case 2: return &Serial2;
        case 3: return &Serial3;
        case 4: return &Serial4;
        default: return &Serial3;
    }
}

// =============================================================================
// Internal Helpers
// =============================================================================

static void send_command(uint8_t address, uint8_t command,
                         const uint8_t* data, uint8_t data_len)
{
    if (!hw_serial) return;

    uint8_t packet[64];
    uint8_t idx = 0;

    packet[idx++] = address;
    packet[idx++] = command;
    for (uint8_t i = 0; i < data_len; i++) {
        packet[idx++] = data[i];
    }

    uint16_t crc = roboclaw_crc16_calculate(packet, idx);
    packet[idx++] = (uint8_t)(crc >> 8);
    packet[idx++] = (uint8_t)(crc & 0xFF);

    hw_serial->write(packet, idx);
}

static bool read_response(uint8_t* buffer, uint8_t expected_len,
                           uint8_t address, uint8_t command)
{
    if (!hw_serial) return false;

    uint8_t total = expected_len + 2;  // data + 2 CRC bytes
    uint32_t start = millis();

    for (uint8_t i = 0; i < total; i++) {
        while (!hw_serial->available()) {
            if (millis() - start > ROBOCLAW_RESPONSE_TIMEOUT_MS) {
                return false;
            }
        }
        buffer[i] = hw_serial->read();
        start = millis();
    }

    // Verify CRC: covers address + command + data
    uint16_t crc = 0;
    crc = roboclaw_crc16_update(crc, address);
    crc = roboclaw_crc16_update(crc, command);
    for (uint8_t i = 0; i < expected_len; i++) {
        crc = roboclaw_crc16_update(crc, buffer[i]);
    }

    uint16_t received_crc = ((uint16_t)buffer[expected_len] << 8) |
                             buffer[expected_len + 1];
    return crc == received_crc;
}

static bool read_ack(void)
{
    if (!hw_serial) return false;

    uint32_t start = millis();
    while (!hw_serial->available()) {
        if (millis() - start > ROBOCLAW_RESPONSE_TIMEOUT_MS) {
            return false;
        }
    }
    return hw_serial->read() == ROBOCLAW_ACK_BYTE;
}

// =============================================================================
// Public API
// =============================================================================

extern "C" {

void roboclaw_init(void)
{
    for (uint8_t i = 0; i < ROBOCLAW_MAX_UNITS; i++) {
        units[i].address = ROBOCLAW_ADDRESS_MIN + i;
        units[i].deadband = 0;
        units[i].max_current_ma = 0;
        units[i].duty = 0;
        units[i].encoder = 0;
        units[i].voltage_mv = 0;
        units[i].current_ma = 0;
        units[i].temp_tenths = 0;
        units[i].connected = false;
    }

#ifndef SIMULATION
    hw_serial = get_serial_port(configured_serial_port);
    hw_serial->begin(configured_baud);
    delay(100);

    // Probe each unit with GETVERSION
    for (uint8_t i = 0; i < ROBOCLAW_MAX_UNITS; i++) {
        send_command(units[i].address, ROBOCLAW_CMD_GETVERSION, NULL, 0);

        uint8_t resp[ROBOCLAW_VERSION_MAX_LEN + 2];
        uint32_t start = millis();
        uint8_t len = 0;
        bool got_null = false;
        while (len < ROBOCLAW_VERSION_MAX_LEN) {
            while (!hw_serial->available()) {
                if (millis() - start > ROBOCLAW_RESPONSE_TIMEOUT_MS) {
                    goto version_done;
                }
            }
            resp[len] = hw_serial->read();
            if (resp[len] == 0) {
                got_null = true;
                len++;
                break;
            }
            len++;
            start = millis();
        }

        if (got_null) {
            for (uint8_t j = 0; j < 2; j++) {
                while (!hw_serial->available()) {
                    if (millis() - start > ROBOCLAW_BYTE_TIMEOUT_MS) {
                        goto version_done;
                    }
                }
                resp[len++] = hw_serial->read();
                start = millis();
            }
            units[i].connected = true;
            if (i >= unit_count) unit_count = i + 1;
            Serial.printf("RoboClaw: unit %d (0x%02X) connected\n",
                          i, units[i].address);
        }
version_done:
        ;
    }
#endif

    port_initialized = true;
    Serial.printf("RoboClaw: initialized on Serial%d at %d baud, %d units\n",
                  configured_serial_port, configured_baud, unit_count);
}

void roboclaw_update(void)
{
    if (!port_initialized || unit_count == 0) return;

#ifndef SIMULATION
    uint8_t u = poll_unit;
    if (!units[u].connected) goto next_poll;

    {
        uint8_t addr = units[u].address;
        uint8_t resp[8];

        switch (poll_register) {
        case 0: // Encoder
            send_command(addr, ROBOCLAW_CMD_GETM1ENC, NULL, 0);
            if (read_response(resp, 5, addr, ROBOCLAW_CMD_GETM1ENC)) {
                units[u].encoder = ((int32_t)resp[0] << 24) |
                                   ((int32_t)resp[1] << 16) |
                                   ((int32_t)resp[2] << 8) |
                                   resp[3];
            }
            break;

        case 1: // Battery voltage
            send_command(addr, ROBOCLAW_CMD_GETMBATT, NULL, 0);
            if (read_response(resp, 2, addr, ROBOCLAW_CMD_GETMBATT)) {
                uint16_t raw = ((uint16_t)resp[0] << 8) | resp[1];
                units[u].voltage_mv = raw * 100;
            }
            break;

        case 2: // Motor current
            send_command(addr, ROBOCLAW_CMD_GETCURRENTS, NULL, 0);
            if (read_response(resp, 4, addr, ROBOCLAW_CMD_GETCURRENTS)) {
                int16_t raw = (int16_t)(((uint16_t)resp[0] << 8) | resp[1]);
                units[u].current_ma = raw * 10;
            }
            break;

        case 3: // Temperature
            send_command(addr, ROBOCLAW_CMD_GETTEMP, NULL, 0);
            if (read_response(resp, 2, addr, ROBOCLAW_CMD_GETTEMP)) {
                units[u].temp_tenths = (int16_t)(((uint16_t)resp[0] << 8) | resp[1]);
            }
            break;
        }
    }

next_poll:
    poll_register++;
    if (poll_register > 3) {
        poll_register = 0;
        poll_unit++;
        if (poll_unit >= unit_count) {
            poll_unit = 0;
        }
    }
#endif
}

bool roboclaw_is_connected(void)
{
    if (!port_initialized) return false;
    for (uint8_t i = 0; i < unit_count; i++) {
        if (units[i].connected) return true;
    }
    return false;
}

bool roboclaw_set_duty(uint8_t unit, int16_t duty)
{
    if (unit >= ROBOCLAW_MAX_UNITS || !port_initialized) return false;

    if (duty > ROBOCLAW_DUTY_MAX) duty = ROBOCLAW_DUTY_MAX;
    if (duty < ROBOCLAW_DUTY_MIN) duty = ROBOCLAW_DUTY_MIN;

    units[unit].duty = duty;

#ifndef SIMULATION
    uint8_t data[2];
    data[0] = (uint8_t)((uint16_t)duty >> 8);
    data[1] = (uint8_t)((uint16_t)duty & 0xFF);
    send_command(units[unit].address, ROBOCLAW_CMD_M1DUTY, data, 2);
    read_ack();
#endif

    return true;
}

bool roboclaw_stop(uint8_t unit)
{
    return roboclaw_set_duty(unit, 0);
}

void roboclaw_stop_all(void)
{
    for (uint8_t i = 0; i < ROBOCLAW_MAX_UNITS; i++) {
        roboclaw_set_duty(i, 0);
    }
}

// =============================================================================
// Peripheral Driver Interface
// =============================================================================

static bool roboclaw_drv_init(void)
{
    roboclaw_init();
    return true;
}

static bool roboclaw_drv_set_value(uint8_t channel, float value)
{
    uint8_t unit = channel / ROBOCLAW_CHANNELS_PER_UNIT;
    uint8_t sub = channel % ROBOCLAW_CHANNELS_PER_UNIT;

    if (sub != ROBOCLAW_SUB_MOTOR) return false;

    if (value < -1.0f) value = -1.0f;
    if (value > 1.0f) value = 1.0f;
    int16_t duty = (int16_t)(value * ROBOCLAW_DUTY_MAX);
    return roboclaw_set_duty(unit, duty);
}

static bool roboclaw_drv_get_value(uint8_t channel, float* value)
{
    uint8_t unit = channel / ROBOCLAW_CHANNELS_PER_UNIT;
    uint8_t sub = channel % ROBOCLAW_CHANNELS_PER_UNIT;

    if (unit >= ROBOCLAW_MAX_UNITS) return false;

    switch (sub) {
    case ROBOCLAW_SUB_MOTOR:
        *value = (float)units[unit].duty / (float)ROBOCLAW_DUTY_MAX;
        return true;
    case ROBOCLAW_SUB_ENCODER:
        *value = (float)units[unit].encoder;
        return true;
    case ROBOCLAW_SUB_VOLTAGE:
        *value = (float)units[unit].voltage_mv / 1000.0f;
        return true;
    case ROBOCLAW_SUB_CURRENT:
        *value = (float)units[unit].current_ma / 1000.0f;
        return true;
    case ROBOCLAW_SUB_TEMP:
        *value = (float)units[unit].temp_tenths / 10.0f;
        return true;
    default:
        return false;
    }
}

static void roboclaw_drv_set_defaults(uint8_t channel, pin_config_t* config)
{
    uint8_t unit = channel / ROBOCLAW_CHANNELS_PER_UNIT;
    config->params.roboclaw.address = ROBOCLAW_ADDRESS_MIN + unit;
    config->params.roboclaw.deadband = 0;
    config->params.roboclaw.max_current_ma = 0;
}

static bool roboclaw_drv_apply_config(uint8_t channel, const pin_config_t* config)
{
    uint8_t unit = channel / ROBOCLAW_CHANNELS_PER_UNIT;
    if (unit >= ROBOCLAW_MAX_UNITS) return false;

    units[unit].address = config->params.roboclaw.address;
    units[unit].deadband = config->params.roboclaw.deadband;
    units[unit].max_current_ma = config->params.roboclaw.max_current_ma;

    if (unit >= unit_count) {
        unit_count = unit + 1;
    }

    return true;
}

static bool roboclaw_drv_parse_json(const char* json_start, const char* json_end,
                                     pin_config_t* config)
{
    const char* p;

    p = strstr(json_start, "\"address\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) { p++; while (*p == ' ') p++; config->params.roboclaw.address = (uint8_t)atoi(p); }
    }

    p = strstr(json_start, "\"deadband\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) { p++; while (*p == ' ') p++; config->params.roboclaw.deadband = (uint8_t)atoi(p); }
    }

    p = strstr(json_start, "\"max_current_ma\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) { p++; while (*p == ' ') p++; config->params.roboclaw.max_current_ma = (uint16_t)atoi(p); }
    }

    return true;
}

static void roboclaw_drv_estop(void)
{
    roboclaw_stop_all();
    Serial.printf("RoboClaw: ESTOP - all units stopped\n");
}

static int roboclaw_drv_caps_json(uint8_t channel, char* buf, size_t remaining)
{
    uint16_t gpio = ROBOCLAW_VIRTUAL_GPIO_BASE + channel;
    uint8_t unit = channel / ROBOCLAW_CHANNELS_PER_UNIT;
    uint8_t sub = channel % ROBOCLAW_CHANNELS_PER_UNIT;

    static const char* sub_names[] = {"Motor", "Encoder", "Voltage", "Current", "Temp"};
    const char* sub_name = (sub < 5) ? sub_names[sub] : "?";

    return snprintf(buf, remaining,
        "{\"gpio\":%d,\"name\":\"RC%d_%s\",\"capabilities\":[\"roboclaw_motor\"]}",
        gpio, unit, sub_name);
}

static bool roboclaw_drv_save(void* storage_ptr)
{
    flash_storage_data_t* storage = (flash_storage_data_t*)storage_ptr;

    memset(&storage->roboclaw_config, 0, sizeof(storage->roboclaw_config));
    storage->roboclaw_config.unit_count = unit_count;
    storage->roboclaw_config.serial_port = configured_serial_port;
    storage->roboclaw_config.baud_rate = configured_baud;

    for (uint8_t i = 0; i < ROBOCLAW_MAX_UNITS; i++) {
        storage->roboclaw_config.units[i].address = units[i].address;
        storage->roboclaw_config.units[i].deadband = units[i].deadband;
        storage->roboclaw_config.units[i].max_current_ma = units[i].max_current_ma;
    }

    return true;
}

static bool roboclaw_drv_load(const void* storage_ptr)
{
    const flash_storage_data_t* storage = (const flash_storage_data_t*)storage_ptr;

    if (storage->roboclaw_config.unit_count == 0) return true;

    uint8_t count = storage->roboclaw_config.unit_count;
    if (count > ROBOCLAW_MAX_UNITS) count = ROBOCLAW_MAX_UNITS;
    unit_count = count;

    if (storage->roboclaw_config.baud_rate > 0) {
        configured_baud = storage->roboclaw_config.baud_rate;
    }
    configured_serial_port = storage->roboclaw_config.serial_port;

    for (uint8_t i = 0; i < count; i++) {
        units[i].address = storage->roboclaw_config.units[i].address;
        units[i].deadband = storage->roboclaw_config.units[i].deadband;
        units[i].max_current_ma = storage->roboclaw_config.units[i].max_current_ma;
    }

    Serial.printf("RoboClaw: restored %d unit configs from flash\n", count);
    return true;
}

// Static driver struct
static const peripheral_driver_t roboclaw_peripheral = {
    .name              = "roboclaw",
    .mode_string       = "roboclaw_motor",
    .pin_mode          = PIN_MODE_ROBOCLAW_MOTOR,
    .capability_flag   = PIN_CAP_ROBOCLAW_MOTOR,
    .virtual_gpio_base = ROBOCLAW_VIRTUAL_GPIO_BASE,
    .channel_count     = ROBOCLAW_MAX_CHANNELS,
    .init              = roboclaw_drv_init,
    .update            = roboclaw_update,
    .is_connected      = roboclaw_is_connected,
    .set_value         = roboclaw_drv_set_value,
    .get_value         = roboclaw_drv_get_value,
    .set_defaults      = roboclaw_drv_set_defaults,
    .apply_config      = roboclaw_drv_apply_config,
    .parse_json_params = roboclaw_drv_parse_json,
    .estop             = roboclaw_drv_estop,
    .capabilities_to_json = roboclaw_drv_caps_json,
    .save_config       = roboclaw_drv_save,
    .load_config       = roboclaw_drv_load,
};

const peripheral_driver_t* roboclaw_get_peripheral_driver(void)
{
    return &roboclaw_peripheral;
}

} // extern "C"
