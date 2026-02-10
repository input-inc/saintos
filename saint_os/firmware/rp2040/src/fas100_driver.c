/**
 * SAINT.OS Node Firmware - FrSky FAS100 ADV Sensor Driver (RP2040)
 *
 * Uses Pico SDK UART with GPIO inversion for S.Port half-duplex communication.
 * Polls the FAS100 at a configurable interval and parses current, voltage,
 * and temperature telemetry responses.
 */

#include "fas100_driver.h"
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

#define FAS100_UART          uart0
#define FAS100_UART_TX_PIN   0
#define FAS100_UART_RX_PIN   1
#define FAS100_SERIAL_PORT   0

// Response buffer (enough for one de-stuffed frame)
#define FAS100_RX_BUF_SIZE   16

// =============================================================================
// State
// =============================================================================

static bool port_initialized = false;
static uint8_t poll_interval_ms = FAS100_DEFAULT_POLL_INTERVAL_MS;
static uint32_t last_poll_time = 0;
static bool sensor_responded = false;

// Latest sensor values
static float current_amps = 0.0f;
static float voltage_volts = 0.0f;
static float temp1_celsius = 0.0f;
static float temp2_celsius = 0.0f;

// Receive buffer for accumulating response bytes
static uint8_t rx_buf[FAS100_RX_BUF_SIZE];
static uint8_t rx_pos = 0;

// =============================================================================
// Internal Helpers
// =============================================================================

/**
 * Send a poll frame: [0x7E, physical_id]
 */
static void send_poll(void)
{
#ifndef SIMULATION
    uint8_t frame[SPORT_POLL_FRAME_SIZE] = {
        SPORT_POLL_HEADER,
        SPORT_FAS100_PHYSICAL_ID
    };
    uart_write_blocking(FAS100_UART, frame, SPORT_POLL_FRAME_SIZE);
#endif
}

/**
 * De-stuff a byte from the S.Port response stream.
 * Returns true if a valid byte was produced, false if waiting for more data.
 */
static bool destuff_byte(uint8_t raw, uint8_t* out, bool* in_stuff)
{
    if (*in_stuff) {
        *out = raw ^ SPORT_STUFF_MASK;
        *in_stuff = false;
        return true;
    }
    if (raw == SPORT_STUFF_MARKER) {
        *in_stuff = true;
        return false;
    }
    *out = raw;
    return true;
}

/**
 * Parse a complete de-stuffed response frame.
 * Frame format: [header(1)] [data_id_lo(1)] [data_id_hi(1)] [val0..val3(4)] [crc(1)]
 */
static void parse_response(const uint8_t* frame, uint8_t len)
{
    if (len < FAS100_RESPONSE_FRAME_SIZE) return;

    // Verify header byte
    if (frame[0] != SPORT_DATA_HEADER) return;

    // Verify CRC over bytes 0..6 (header + data_id + value)
    uint8_t expected_crc = sport_crc_calculate(frame, 7);
    if (frame[7] != expected_crc) return;

    // Extract data ID (16-bit little-endian)
    uint16_t data_id = (uint16_t)frame[1] | ((uint16_t)frame[2] << 8);

    // Extract value (32-bit little-endian)
    uint32_t value = (uint32_t)frame[3]
                   | ((uint32_t)frame[4] << 8)
                   | ((uint32_t)frame[5] << 16)
                   | ((uint32_t)frame[6] << 24);

    sensor_responded = true;

    switch (data_id) {
        case SPORT_DATA_ID_CURRENT:
            current_amps = (float)value / 10.0f;
            break;
        case SPORT_DATA_ID_VOLTAGE:
            voltage_volts = (float)value / 100.0f;
            break;
        case SPORT_DATA_ID_TEMP1:
            temp1_celsius = (float)(int32_t)value;
            break;
        case SPORT_DATA_ID_TEMP2:
            temp2_celsius = (float)(int32_t)value;
            break;
        default:
            break;
    }
}

// =============================================================================
// Public API
// =============================================================================

void fas100_init(void)
{
#ifndef SIMULATION
    uart_init(FAS100_UART, SPORT_BAUD_RATE);

    gpio_set_function(FAS100_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(FAS100_UART_RX_PIN, GPIO_FUNC_UART);

    // S.Port uses inverted signaling
    gpio_set_outover(FAS100_UART_TX_PIN, GPIO_OVERRIDE_INVERT);
    gpio_set_inover(FAS100_UART_RX_PIN, GPIO_OVERRIDE_INVERT);
#endif

    rx_pos = 0;
    sensor_responded = false;
    port_initialized = true;

    PLATFORM_PRINTF("FAS100: initialized on UART%d at %d baud (poll %dms)\n",
                    FAS100_SERIAL_PORT, SPORT_BAUD_RATE, poll_interval_ms);
}

void fas100_update(void)
{
    if (!port_initialized) return;

    uint32_t now = PLATFORM_MILLIS();

    // Non-blocking read of any available bytes
#ifndef SIMULATION
    while (uart_is_readable(FAS100_UART)) {
        uint8_t raw = uart_getc(FAS100_UART);
        static bool in_stuff = false;
        uint8_t byte;

        if (destuff_byte(raw, &byte, &in_stuff)) {
            if (rx_pos < FAS100_RX_BUF_SIZE) {
                rx_buf[rx_pos++] = byte;
            }
        }

        // Check for complete frame
        if (rx_pos >= FAS100_RESPONSE_FRAME_SIZE) {
            parse_response(rx_buf, rx_pos);
            rx_pos = 0;
            in_stuff = false;
        }
    }
#endif

    // Send next poll if interval has elapsed
    if (now - last_poll_time >= poll_interval_ms) {
        last_poll_time = now;
        rx_pos = 0;  // Reset receive buffer for new response
        send_poll();
    }
}

bool fas100_is_connected(void)
{
    return port_initialized && sensor_responded;
}

float fas100_get_current(void)  { return current_amps; }
float fas100_get_voltage(void)  { return voltage_volts; }
float fas100_get_temp1(void)    { return temp1_celsius; }
float fas100_get_temp2(void)    { return temp2_celsius; }

// =============================================================================
// Peripheral Driver Interface
// =============================================================================

static bool fas100_drv_init(void)
{
    fas100_init();
    return true;
}

static bool fas100_drv_set_value(uint8_t channel, float value)
{
    // Read-only sensor — cannot set values
    (void)channel;
    (void)value;
    return false;
}

static bool fas100_drv_get_value(uint8_t channel, float* value)
{
    if (!value) return false;

    switch (channel) {
        case FAS100_CH_CURRENT: *value = current_amps;   return true;
        case FAS100_CH_VOLTAGE: *value = voltage_volts;   return true;
        case FAS100_CH_TEMP1:   *value = temp1_celsius;   return true;
        case FAS100_CH_TEMP2:   *value = temp2_celsius;   return true;
        default: return false;
    }
}

static void fas100_drv_set_defaults(uint8_t channel, pin_config_t* config)
{
    (void)channel;
    config->params.fas100.poll_interval_ms = FAS100_DEFAULT_POLL_INTERVAL_MS;
}

static bool fas100_drv_apply_config(uint8_t channel, const pin_config_t* config)
{
    (void)channel;
    uint8_t interval = config->params.fas100.poll_interval_ms;
    if (interval >= 20) {
        poll_interval_ms = interval;
    }
    return true;
}

static bool fas100_drv_parse_json(const char* json_start, const char* json_end,
                                   pin_config_t* config)
{
    const char* p = strstr(json_start, "\"poll_interval_ms\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) {
            p++;
            while (*p == ' ') p++;
            config->params.fas100.poll_interval_ms = (uint8_t)atoi(p);
        }
    }
    return true;
}

static void fas100_drv_estop(void)
{
    // Read-only sensor — nothing to stop
    PLATFORM_PRINTF("FAS100: ESTOP (no action needed for read-only sensor)\n");
}

static int fas100_drv_caps_json(uint8_t channel, char* buf, size_t remaining)
{
    uint8_t gpio = FAS100_VIRTUAL_GPIO_BASE + channel;
    const char* names[] = { "AMPS", "VOLTS", "TMP1", "TMP2" };
    const char* name = (channel < 4) ? names[channel] : "???";

    return snprintf(buf, remaining,
        "{\"gpio\":%d,\"name\":\"%s\",\"capabilities\":[\"fas100_sensor\"]}",
        gpio, name);
}

static bool fas100_drv_save(void* storage_ptr)
{
    flash_storage_data_t* storage = (flash_storage_data_t*)storage_ptr;

    memset(&storage->fas100_config, 0, sizeof(storage->fas100_config));
    storage->fas100_config.enabled = port_initialized ? 1 : 0;
    storage->fas100_config.serial_port = FAS100_SERIAL_PORT;
    storage->fas100_config.poll_interval_ms = poll_interval_ms;

    return true;
}

static bool fas100_drv_load(const void* storage_ptr)
{
    const flash_storage_data_t* storage = (const flash_storage_data_t*)storage_ptr;

    if (!storage->fas100_config.enabled) return true;

    if (storage->fas100_config.poll_interval_ms >= 20) {
        poll_interval_ms = storage->fas100_config.poll_interval_ms;
    }

    PLATFORM_PRINTF("FAS100: restored config from flash (poll %dms)\n",
                    poll_interval_ms);
    return true;
}

// Static driver struct
static const peripheral_driver_t fas100_peripheral = {
    .name              = "fas100",
    .mode_string       = "fas100_sensor",
    .pin_mode          = PIN_MODE_FAS100_SENSOR,
    .capability_flag   = PIN_CAP_FAS100_SENSOR,
    .virtual_gpio_base = FAS100_VIRTUAL_GPIO_BASE,
    .channel_count     = FAS100_CHANNEL_COUNT,
    .init              = fas100_drv_init,
    .update            = fas100_update,
    .is_connected      = fas100_is_connected,
    .set_value         = fas100_drv_set_value,
    .get_value         = fas100_drv_get_value,
    .set_defaults      = fas100_drv_set_defaults,
    .apply_config      = fas100_drv_apply_config,
    .parse_json_params = fas100_drv_parse_json,
    .estop             = fas100_drv_estop,
    .capabilities_to_json = fas100_drv_caps_json,
    .save_config       = fas100_drv_save,
    .load_config       = fas100_drv_load,
};

const peripheral_driver_t* fas100_get_peripheral_driver(void)
{
    return &fas100_peripheral;
}
