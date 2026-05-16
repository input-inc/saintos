/**
 * SAINT.OS Node Firmware - Pathfinder BMS Driver (RP2040)
 *
 * Uses Pico SDK UART for JBD protocol communication at 9600 baud.
 * Non-blocking RX state machine alternates between reading register 0x03
 * (basic info) and 0x04 (cell voltages) each poll cycle.
 */

#include "pathfinder_bms_driver.h"
#include "peripheral_driver.h"
#include "flash_types.h"
#include "platform.h"
#include "uart_pin_pairs.h"

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

#define BMS_DEFAULT_TX_PIN     0
#define BMS_DEFAULT_RX_PIN     1

#ifndef SIMULATION
static uart_inst_t* bms_uart = NULL;
#endif
static uint8_t bms_tx_pin = BMS_DEFAULT_TX_PIN;
static uint8_t bms_rx_pin = BMS_DEFAULT_RX_PIN;
static uint8_t bms_uart_instance = 0;

// =============================================================================
// RX State Machine
// =============================================================================

typedef enum {
    RX_IDLE,
    RX_WAIT_ACTION,
    RX_WAIT_REGISTER,
    RX_WAIT_LENGTH,
    RX_READING_DATA,
    RX_WAIT_CRC_HI,
    RX_WAIT_CRC_LO,
    RX_WAIT_END
} rx_state_t;

// =============================================================================
// State
// =============================================================================

static bool port_initialized = false;
static uint16_t poll_interval_ms = JBD_BMS_DEFAULT_POLL_INTERVAL_MS;
static uint32_t last_poll_time = 0;
static bool bms_responded = false;

// Alternating register polling
static uint8_t next_register = JBD_REG_BASIC_INFO;

// RX state machine
static rx_state_t rx_state = RX_IDLE;
static uint8_t rx_action = 0;
static uint8_t rx_register = 0;
static uint8_t rx_length = 0;
static uint8_t rx_data[JBD_RESPONSE_MAX_DATA];
static uint8_t rx_data_pos = 0;
static uint8_t rx_crc_hi = 0;

// Latest sensor values
static float pack_voltage = 0.0f;
static float current_amps = 0.0f;
static float soc_percent = 0.0f;
static float remain_cap_ah = 0.0f;
static float temp1_celsius = 0.0f;
static float temp2_celsius = 0.0f;
static uint16_t cycle_count = 0;
static uint16_t protection_status = 0;
static float cell_voltages[JBD_MAX_CELLS];

// =============================================================================
// Internal Helpers
// =============================================================================

static void send_read_request(uint8_t reg)
{
#ifndef SIMULATION
    if (!bms_uart) return;
    uint8_t frame[JBD_READ_REQUEST_SIZE];
    jbd_build_read_request(reg, frame);
    uart_write_blocking(bms_uart, frame, JBD_READ_REQUEST_SIZE);
#else
    (void)reg;
#endif
}

static void parse_basic_info(const uint8_t* data, uint8_t len)
{
    if (len < JBD_BASIC_MIN_DATA_LEN) return;

    pack_voltage = (float)((uint16_t)(data[JBD_BASIC_PACK_VOLTAGE_OFF] << 8 |
                   data[JBD_BASIC_PACK_VOLTAGE_OFF + 1])) * 0.01f;

    int16_t raw_current = (int16_t)(data[JBD_BASIC_CURRENT_OFF] << 8 |
                          data[JBD_BASIC_CURRENT_OFF + 1]);
    current_amps = (float)raw_current * 0.01f;

    remain_cap_ah = (float)((uint16_t)(data[JBD_BASIC_REMAIN_CAP_OFF] << 8 |
                    data[JBD_BASIC_REMAIN_CAP_OFF + 1])) * 0.01f;

    cycle_count = (uint16_t)(data[JBD_BASIC_CYCLES_OFF] << 8 |
                  data[JBD_BASIC_CYCLES_OFF + 1]);

    protection_status = (uint16_t)(data[JBD_BASIC_PROTECTION_OFF] << 8 |
                        data[JBD_BASIC_PROTECTION_OFF + 1]);

    soc_percent = (float)data[JBD_BASIC_SOC_OFF];

    // Parse NTC temperatures
    uint8_t ntc_count = data[JBD_BASIC_NTC_COUNT_OFF];
    if (ntc_count >= 1 && len >= JBD_BASIC_NTC_START_OFF + 2) {
        uint16_t raw_t1 = (uint16_t)(data[JBD_BASIC_NTC_START_OFF] << 8 |
                          data[JBD_BASIC_NTC_START_OFF + 1]);
        temp1_celsius = ((float)raw_t1 - 2731.0f) * 0.1f;
    }
    if (ntc_count >= 2 && len >= JBD_BASIC_NTC_START_OFF + 4) {
        uint16_t raw_t2 = (uint16_t)(data[JBD_BASIC_NTC_START_OFF + 2] << 8 |
                          data[JBD_BASIC_NTC_START_OFF + 3]);
        temp2_celsius = ((float)raw_t2 - 2731.0f) * 0.1f;
    }
}

static void parse_cell_voltages(const uint8_t* data, uint8_t len)
{
    uint8_t num_cells = len / 2;
    if (num_cells > JBD_MAX_CELLS) num_cells = JBD_MAX_CELLS;

    for (uint8_t i = 0; i < num_cells; i++) {
        uint16_t mv = (uint16_t)(data[i * 2] << 8 | data[i * 2 + 1]);
        cell_voltages[i] = (float)mv * 0.001f;
    }
}

static void process_rx_byte(uint8_t byte)
{
    switch (rx_state) {
        case RX_IDLE:
            if (byte == JBD_FRAME_START) {
                rx_state = RX_WAIT_ACTION;
            }
            break;

        case RX_WAIT_ACTION:
            rx_action = byte;
            rx_state = RX_WAIT_REGISTER;
            break;

        case RX_WAIT_REGISTER:
            rx_register = byte;
            rx_state = RX_WAIT_LENGTH;
            break;

        case RX_WAIT_LENGTH:
            rx_length = byte;
            rx_data_pos = 0;
            if (rx_length == 0) {
                rx_state = RX_WAIT_CRC_HI;
            } else if (rx_length > JBD_RESPONSE_MAX_DATA) {
                rx_state = RX_IDLE;
            } else {
                rx_state = RX_READING_DATA;
            }
            break;

        case RX_READING_DATA:
            rx_data[rx_data_pos++] = byte;
            if (rx_data_pos >= rx_length) {
                rx_state = RX_WAIT_CRC_HI;
            }
            break;

        case RX_WAIT_CRC_HI:
            rx_crc_hi = byte;
            rx_state = RX_WAIT_CRC_LO;
            break;

        case RX_WAIT_CRC_LO: {
            uint8_t crc_lo = byte;
            uint16_t received_crc = ((uint16_t)rx_crc_hi << 8) | crc_lo;

            // Verify checksum over register + length + data
            uint8_t crc_input[2 + JBD_RESPONSE_MAX_DATA];
            crc_input[0] = rx_register;
            crc_input[1] = rx_length;
            memcpy(&crc_input[2], rx_data, rx_length);
            uint16_t expected_crc = jbd_checksum_calculate(crc_input, 2 + rx_length);

            if (received_crc == expected_crc) {
                rx_state = RX_WAIT_END;
            } else {
                rx_state = RX_IDLE;
            }
            break;
        }

        case RX_WAIT_END:
            if (byte == JBD_FRAME_END && rx_action == JBD_ACTION_OK) {
                bms_responded = true;

                if (rx_register == JBD_REG_BASIC_INFO) {
                    parse_basic_info(rx_data, rx_length);
                } else if (rx_register == JBD_REG_CELL_VOLTAGES) {
                    parse_cell_voltages(rx_data, rx_length);
                }
            }
            rx_state = RX_IDLE;
            break;
    }
}

// =============================================================================
// Public API
// =============================================================================

void pathfinder_bms_init(void)
{
#ifndef SIMULATION
    if (!uart_pin_pair_lookup(bms_tx_pin, bms_rx_pin, &bms_uart_instance)) {
        bms_tx_pin = BMS_DEFAULT_TX_PIN;
        bms_rx_pin = BMS_DEFAULT_RX_PIN;
        bms_uart_instance = 0;
    }
    bms_uart = (bms_uart_instance == 0) ? uart0 : uart1;

    uart_init(bms_uart, JBD_BAUD_RATE);
    gpio_set_function(bms_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(bms_rx_pin, GPIO_FUNC_UART);
#endif

    memset(cell_voltages, 0, sizeof(cell_voltages));
    rx_state = RX_IDLE;
    bms_responded = false;
    next_register = JBD_REG_BASIC_INFO;
    port_initialized = true;

    PLATFORM_PRINTF("Pathfinder BMS: initialized on UART%d TX=%d RX=%d at %d baud (poll %dms)\n",
                    bms_uart_instance, bms_tx_pin, bms_rx_pin,
                    JBD_BAUD_RATE, poll_interval_ms);
}

void pathfinder_bms_update(void)
{
    if (!port_initialized) return;

    uint32_t now = PLATFORM_MILLIS();

    // Non-blocking read of available bytes
#ifndef SIMULATION
    while (bms_uart && uart_is_readable(bms_uart)) {
        uint8_t byte = uart_getc(bms_uart);
        process_rx_byte(byte);
    }
#endif

    // Send next poll if interval has elapsed
    if (now - last_poll_time >= poll_interval_ms) {
        last_poll_time = now;
        rx_state = RX_IDLE;
        send_read_request(next_register);

        // Alternate between basic info and cell voltages
        next_register = (next_register == JBD_REG_BASIC_INFO)
                        ? JBD_REG_CELL_VOLTAGES
                        : JBD_REG_BASIC_INFO;
    }
}

bool pathfinder_bms_is_connected(void)
{
    return port_initialized && bms_responded;
}

float pathfinder_bms_get_pack_voltage(void)     { return pack_voltage; }
float pathfinder_bms_get_current(void)           { return current_amps; }
float pathfinder_bms_get_soc(void)               { return soc_percent; }
float pathfinder_bms_get_remain_cap(void)        { return remain_cap_ah; }
float pathfinder_bms_get_temp1(void)             { return temp1_celsius; }
float pathfinder_bms_get_temp2(void)             { return temp2_celsius; }
uint16_t pathfinder_bms_get_cycles(void)         { return cycle_count; }
uint16_t pathfinder_bms_get_protection_status(void) { return protection_status; }

float pathfinder_bms_get_cell_voltage(uint8_t cell)
{
    if (cell >= JBD_MAX_CELLS) return 0.0f;
    return cell_voltages[cell];
}

// =============================================================================
// Peripheral Driver Interface
// =============================================================================

static bool bms_drv_init(void)
{
    pathfinder_bms_init();
    return true;
}

static bool bms_drv_set_value(uint8_t channel, float value)
{
    (void)channel;
    (void)value;
    return false;
}

static bool bms_drv_get_value(uint8_t channel, float* value)
{
    if (!value) return false;

    switch (channel) {
        case JBD_BMS_CH_PACK_VOLTAGE: *value = pack_voltage;                  return true;
        case JBD_BMS_CH_CURRENT:      *value = current_amps;                  return true;
        case JBD_BMS_CH_SOC:          *value = soc_percent;                   return true;
        case JBD_BMS_CH_REMAIN_CAP:   *value = remain_cap_ah;                return true;
        case JBD_BMS_CH_TEMP1:        *value = temp1_celsius;                 return true;
        case JBD_BMS_CH_TEMP2:        *value = temp2_celsius;                 return true;
        case JBD_BMS_CH_CYCLES:       *value = (float)cycle_count;            return true;
        case JBD_BMS_CH_PROTECTION:   *value = (float)protection_status;      return true;
        default:
            if (channel >= JBD_BMS_CH_CELL_BASE &&
                channel < JBD_BMS_CH_CELL_BASE + JBD_MAX_CELLS) {
                *value = cell_voltages[channel - JBD_BMS_CH_CELL_BASE];
                return true;
            }
            return false;
    }
}

static void bms_drv_set_defaults(uint8_t channel, pin_config_t* config)
{
    (void)channel;
    config->params.pathfinder_bms.poll_interval_ms = JBD_BMS_DEFAULT_POLL_INTERVAL_MS;
}

static bool bms_drv_apply_config(uint8_t channel, const pin_config_t* config)
{
    (void)channel;
    uint16_t interval = config->params.pathfinder_bms.poll_interval_ms;
    if (interval >= 100) {
        poll_interval_ms = interval;
    }
    return true;
}

static bool bms_drv_parse_json(const char* json_start, const char* json_end,
                                pin_config_t* config)
{
    const char* p = strstr(json_start, "\"poll_interval_ms\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) {
            p++;
            while (*p == ' ') p++;
            config->params.pathfinder_bms.poll_interval_ms = (uint16_t)atoi(p);
        }
    }

    uint8_t tx, rx, inst;
    if (uart_pin_pair_parse_json(json_start, json_end, &tx, &rx, &inst)) {
        bms_tx_pin = tx;
        bms_rx_pin = rx;
        bms_uart_instance = inst;
    }

    return true;
}

static void bms_drv_estop(void)
{
    PLATFORM_PRINTF("Pathfinder BMS: ESTOP (no action needed for read-only sensor)\n");
}

static int bms_drv_caps_json(uint8_t channel, char* buf, size_t remaining)
{
    uint16_t gpio = JBD_BMS_VIRTUAL_GPIO_BASE + channel;

    const char* names[] = {
        "PackV", "Amps", "SOC", "RemCap", "Tmp1", "Tmp2", "Cyc", "Prot",
        "Cel01", "Cel02", "Cel03", "Cel04", "Cel05", "Cel06", "Cel07", "Cel08",
        "Cel09", "Cel10", "Cel11", "Cel12", "Cel13", "Cel14", "Cel15", "Cel16"
    };
    const char* name = (channel < JBD_BMS_CHANNEL_COUNT) ? names[channel] : "???";

    return snprintf(buf, remaining,
        "{\"gpio\":%d,\"name\":\"%s\",\"capabilities\":[\"pathfinder_bms_sensor\"]}",
        gpio, name);
}

static bool bms_drv_save(void* storage_ptr)
{
    flash_storage_data_t* storage = (flash_storage_data_t*)storage_ptr;

    memset(&storage->pathfinder_bms_config, 0, sizeof(storage->pathfinder_bms_config));
    storage->pathfinder_bms_config.enabled = port_initialized ? 1 : 0;
    storage->pathfinder_bms_config.serial_port = bms_uart_instance;
    storage->pathfinder_bms_config.poll_interval_ms = poll_interval_ms;

    storage->uart_pins.pathfinder_bms_tx_pin = bms_tx_pin;
    storage->uart_pins.pathfinder_bms_rx_pin = bms_rx_pin;

    return true;
}

static bool bms_drv_load(const void* storage_ptr)
{
    const flash_storage_data_t* storage = (const flash_storage_data_t*)storage_ptr;

    uint8_t stored_tx = storage->uart_pins.pathfinder_bms_tx_pin;
    uint8_t stored_rx = storage->uart_pins.pathfinder_bms_rx_pin;
    if (stored_tx != 0 || stored_rx != 0) {
        uint8_t inst;
        if (uart_pin_pair_lookup(stored_tx, stored_rx, &inst)) {
            bms_tx_pin = stored_tx;
            bms_rx_pin = stored_rx;
            bms_uart_instance = inst;
        } else {
            PLATFORM_PRINTF("Pathfinder BMS: invalid stored pin pair tx=%d rx=%d, using defaults\n",
                            stored_tx, stored_rx);
        }
    }

    if (!storage->pathfinder_bms_config.enabled) return true;

    if (storage->pathfinder_bms_config.poll_interval_ms >= 100) {
        poll_interval_ms = storage->pathfinder_bms_config.poll_interval_ms;
    }

    PLATFORM_PRINTF("Pathfinder BMS: restored config from flash (poll %dms)\n",
                    poll_interval_ms);
    return true;
}

// Static driver struct
static const peripheral_driver_t pathfinder_bms_peripheral = {
    .name              = "pathfinder_bms",
    .mode_string       = "pathfinder_bms_sensor",
    .pin_mode          = PIN_MODE_PATHFINDER_BMS,
    .capability_flag   = PIN_CAP_PATHFINDER_BMS,
    .virtual_gpio_base = JBD_BMS_VIRTUAL_GPIO_BASE,
    .channel_count     = JBD_BMS_CHANNEL_COUNT,
    .init              = bms_drv_init,
    .update            = pathfinder_bms_update,
    .is_connected      = pathfinder_bms_is_connected,
    .set_value         = bms_drv_set_value,
    .get_value         = bms_drv_get_value,
    .set_defaults      = bms_drv_set_defaults,
    .apply_config      = bms_drv_apply_config,
    .parse_json_params = bms_drv_parse_json,
    .estop             = bms_drv_estop,
    .capabilities_to_json = bms_drv_caps_json,
    .save_config       = bms_drv_save,
    .load_config       = bms_drv_load,
};

const peripheral_driver_t* pathfinder_bms_get_peripheral_driver(void)
{
    return &pathfinder_bms_peripheral;
}
