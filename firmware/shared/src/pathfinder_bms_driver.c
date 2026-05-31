/**
 * SAINT.OS Firmware - Pathfinder BMS driver core (shared)
 *
 * Owns: JBD-protocol RX state machine, frame parsers (basic info /
 * cell voltages), poll scheduler, sensor readout cache, and the
 * peripheral_driver_t glue. UART I/O dispatches through the per-
 * platform transport ops (shared/include/pathfinder_bms_transport.h).
 *
 * Was previously duplicated between firmware/rp2040/src/pathfinder_bms_driver.c
 * and firmware/teensy41/src/pathfinder_bms_driver.cpp — same shape,
 * just different UART APIs. One source of truth eliminates that.
 */

#include "pathfinder_bms_driver.h"
#include "pathfinder_bms_transport.h"

#include "flash_types.h"
#include "peripheral_driver.h"
#include "pin_types.h"
#include "platform.h"
#include "saint_log.h"
#include "uart_pin_pairs.h"

#include <stdlib.h>
#include <string.h>

/* ── RX state machine ───────────────────────────────────────────── */

typedef enum {
    RX_IDLE,
    RX_WAIT_ACTION,
    RX_WAIT_REGISTER,
    RX_WAIT_LENGTH,
    RX_READING_DATA,
    RX_WAIT_CRC_HI,
    RX_WAIT_CRC_LO,
    RX_WAIT_END,
} rx_state_t;

/* ── Module state ───────────────────────────────────────────────── */

static bool     g_port_initialized = false;
static uint16_t g_poll_interval_ms = JBD_BMS_DEFAULT_POLL_INTERVAL_MS;
static uint32_t g_last_poll_time   = 0;
static bool     g_bms_responded    = false;
static uint8_t  g_next_register    = JBD_REG_BASIC_INFO;

/* RX parser state */
static rx_state_t g_rx_state    = RX_IDLE;
static uint8_t    g_rx_action   = 0;
static uint8_t    g_rx_register = 0;
static uint8_t    g_rx_length   = 0;
static uint8_t    g_rx_data[JBD_RESPONSE_MAX_DATA];
static uint8_t    g_rx_data_pos = 0;
static uint8_t    g_rx_crc_hi   = 0;

/* Pin / baud overrides — passed to transport at open time. */
static uint8_t  g_tx_pin = 0;
static uint8_t  g_rx_pin = 0;
static uint16_t g_baud   = JBD_BAUD_RATE;

/* Latest decoded sensor values. */
static float    g_pack_voltage  = 0.0f;
static float    g_current_amps  = 0.0f;
static float    g_soc_percent   = 0.0f;
static float    g_remain_cap_ah = 0.0f;
static float    g_temp1_celsius = 0.0f;
static float    g_temp2_celsius = 0.0f;
static uint16_t g_cycle_count   = 0;
static uint16_t g_protection    = 0;
static float    g_cell_voltages[JBD_MAX_CELLS];

static const pathfinder_bms_transport_ops_t* transport(void)
{
    return pathfinder_bms_get_transport();
}

/* ── Internal helpers ───────────────────────────────────────────── */

static void send_read_request(uint8_t reg)
{
    const pathfinder_bms_transport_ops_t* t = transport();
    if (!t || !t->is_open()) return;
    uint8_t frame[JBD_READ_REQUEST_SIZE];
    jbd_build_read_request(reg, frame);
    (void)t->write(frame, JBD_READ_REQUEST_SIZE);
}

static void parse_basic_info(const uint8_t* data, uint8_t len)
{
    if (len < JBD_BASIC_MIN_DATA_LEN) return;

    g_pack_voltage = (float)((uint16_t)(data[JBD_BASIC_PACK_VOLTAGE_OFF] << 8 |
                     data[JBD_BASIC_PACK_VOLTAGE_OFF + 1])) * 0.01f;

    int16_t raw_current = (int16_t)(data[JBD_BASIC_CURRENT_OFF] << 8 |
                          data[JBD_BASIC_CURRENT_OFF + 1]);
    g_current_amps = (float)raw_current * 0.01f;

    g_remain_cap_ah = (float)((uint16_t)(data[JBD_BASIC_REMAIN_CAP_OFF] << 8 |
                      data[JBD_BASIC_REMAIN_CAP_OFF + 1])) * 0.01f;

    g_cycle_count = (uint16_t)(data[JBD_BASIC_CYCLES_OFF] << 8 |
                    data[JBD_BASIC_CYCLES_OFF + 1]);

    g_protection = (uint16_t)(data[JBD_BASIC_PROTECTION_OFF] << 8 |
                   data[JBD_BASIC_PROTECTION_OFF + 1]);

    g_soc_percent = (float)data[JBD_BASIC_SOC_OFF];

    uint8_t ntc_count = data[JBD_BASIC_NTC_COUNT_OFF];
    if (ntc_count >= 1 && len >= JBD_BASIC_NTC_START_OFF + 2) {
        uint16_t raw_t1 = (uint16_t)(data[JBD_BASIC_NTC_START_OFF] << 8 |
                          data[JBD_BASIC_NTC_START_OFF + 1]);
        g_temp1_celsius = ((float)raw_t1 - 2731.0f) * 0.1f;
    }
    if (ntc_count >= 2 && len >= JBD_BASIC_NTC_START_OFF + 4) {
        uint16_t raw_t2 = (uint16_t)(data[JBD_BASIC_NTC_START_OFF + 2] << 8 |
                          data[JBD_BASIC_NTC_START_OFF + 3]);
        g_temp2_celsius = ((float)raw_t2 - 2731.0f) * 0.1f;
    }
}

static void parse_cell_voltages(const uint8_t* data, uint8_t len)
{
    uint8_t num_cells = len / 2;
    if (num_cells > JBD_MAX_CELLS) num_cells = JBD_MAX_CELLS;
    for (uint8_t i = 0; i < num_cells; i++) {
        uint16_t mv = (uint16_t)(data[i * 2] << 8 | data[i * 2 + 1]);
        g_cell_voltages[i] = (float)mv * 0.001f;
    }
}

static void process_rx_byte(uint8_t byte)
{
    switch (g_rx_state) {
    case RX_IDLE:
        if (byte == JBD_FRAME_START) g_rx_state = RX_WAIT_ACTION;
        break;
    case RX_WAIT_ACTION:
        g_rx_action = byte;
        g_rx_state  = RX_WAIT_REGISTER;
        break;
    case RX_WAIT_REGISTER:
        g_rx_register = byte;
        g_rx_state    = RX_WAIT_LENGTH;
        break;
    case RX_WAIT_LENGTH:
        g_rx_length   = byte;
        g_rx_data_pos = 0;
        if (g_rx_length == 0)                        g_rx_state = RX_WAIT_CRC_HI;
        else if (g_rx_length > JBD_RESPONSE_MAX_DATA) g_rx_state = RX_IDLE;
        else                                          g_rx_state = RX_READING_DATA;
        break;
    case RX_READING_DATA:
        g_rx_data[g_rx_data_pos++] = byte;
        if (g_rx_data_pos >= g_rx_length) g_rx_state = RX_WAIT_CRC_HI;
        break;
    case RX_WAIT_CRC_HI:
        g_rx_crc_hi = byte;
        g_rx_state  = RX_WAIT_CRC_LO;
        break;
    case RX_WAIT_CRC_LO: {
        uint16_t received = ((uint16_t)g_rx_crc_hi << 8) | byte;
        uint8_t crc_input[2 + JBD_RESPONSE_MAX_DATA];
        crc_input[0] = g_rx_register;
        crc_input[1] = g_rx_length;
        memcpy(&crc_input[2], g_rx_data, g_rx_length);
        uint16_t expected = jbd_checksum_calculate(crc_input, 2 + g_rx_length);
        g_rx_state = (received == expected) ? RX_WAIT_END : RX_IDLE;
        break;
    }
    case RX_WAIT_END:
        if (byte == JBD_FRAME_END && g_rx_action == JBD_ACTION_OK) {
            g_bms_responded = true;
            if      (g_rx_register == JBD_REG_BASIC_INFO)    parse_basic_info(g_rx_data, g_rx_length);
            else if (g_rx_register == JBD_REG_CELL_VOLTAGES) parse_cell_voltages(g_rx_data, g_rx_length);
        }
        g_rx_state = RX_IDLE;
        break;
    }
}

/* ── Public API ─────────────────────────────────────────────────── */

void pathfinder_bms_init(void)
{
    /* Data-only init. Idempotent. Hardware acquisition is deferred to
     * the transport via drv_load / drv_apply_config, so the chip
     * doesn't grab pins at boot unless configured. */
    static bool initialized = false;
    if (initialized) return;
    memset(g_cell_voltages, 0, sizeof(g_cell_voltages));
    g_rx_state       = RX_IDLE;
    g_bms_responded  = false;
    g_next_register  = JBD_REG_BASIC_INFO;
    initialized      = true;
}

static bool bms_open(void)
{
    const pathfinder_bms_transport_ops_t* t = transport();
    if (!t) return false;
    if (t->is_open()) {
        g_port_initialized = true;
        return true;
    }
    if (!t->open(g_tx_pin, g_rx_pin, g_baud)) {
        saint_log_publish("error",
            "Pathfinder BMS: %s transport open failed (tx=%d rx=%d)",
            t->name, g_tx_pin, g_rx_pin);
        return false;
    }
    g_port_initialized = true;
    saint_log_publish("info",
        "Pathfinder BMS: opened on %s (tx=%d rx=%d baud=%u, poll %ums)",
        t->name, g_tx_pin, g_rx_pin, (unsigned)g_baud,
        (unsigned)g_poll_interval_ms);
    return true;
}

void pathfinder_bms_update(void)
{
    if (!g_port_initialized) return;

    /* Drain whatever the transport has buffered. */
    const pathfinder_bms_transport_ops_t* t = transport();
    if (t && t->read) {
        uint8_t buf[64];
        size_t n;
        while ((n = t->read(buf, sizeof(buf))) > 0) {
            for (size_t i = 0; i < n; i++) process_rx_byte(buf[i]);
        }
    }

    uint32_t now = PLATFORM_MILLIS();
    if (now - g_last_poll_time >= g_poll_interval_ms) {
        g_last_poll_time = now;
        g_rx_state = RX_IDLE;  /* drop any half-parsed response */
        send_read_request(g_next_register);
        g_next_register = (g_next_register == JBD_REG_BASIC_INFO)
                          ? JBD_REG_CELL_VOLTAGES
                          : JBD_REG_BASIC_INFO;
    }
}

bool pathfinder_bms_is_connected(void)
{
    return g_port_initialized && g_bms_responded;
}

float    pathfinder_bms_get_pack_voltage(void)     { return g_pack_voltage; }
float    pathfinder_bms_get_current(void)           { return g_current_amps; }
float    pathfinder_bms_get_soc(void)               { return g_soc_percent; }
float    pathfinder_bms_get_remain_cap(void)        { return g_remain_cap_ah; }
float    pathfinder_bms_get_temp1(void)             { return g_temp1_celsius; }
float    pathfinder_bms_get_temp2(void)             { return g_temp2_celsius; }
uint16_t pathfinder_bms_get_cycles(void)            { return g_cycle_count; }
uint16_t pathfinder_bms_get_protection_status(void) { return g_protection; }
float    pathfinder_bms_get_cell_voltage(uint8_t cell)
{
    if (cell >= JBD_MAX_CELLS) return 0.0f;
    return g_cell_voltages[cell];
}

/* ── peripheral_driver_t glue ───────────────────────────────────── */

static bool drv_init(void)
{
    /* Intentional no-op (README item 3 pattern). Real init runs from
     * drv_load (when flash has enabled=1) or drv_apply_config. */
    return true;
}

static bool drv_set_value(uint8_t channel, float value)
{
    (void)channel; (void)value;
    return false;  /* read-only sensor */
}

static bool drv_get_value(uint8_t channel, float* value)
{
    if (!value) return false;
    switch (channel) {
    case JBD_BMS_CH_PACK_VOLTAGE: *value = g_pack_voltage;        return true;
    case JBD_BMS_CH_CURRENT:      *value = g_current_amps;        return true;
    case JBD_BMS_CH_SOC:          *value = g_soc_percent;         return true;
    case JBD_BMS_CH_REMAIN_CAP:   *value = g_remain_cap_ah;       return true;
    case JBD_BMS_CH_TEMP1:        *value = g_temp1_celsius;       return true;
    case JBD_BMS_CH_TEMP2:        *value = g_temp2_celsius;       return true;
    case JBD_BMS_CH_CYCLES:       *value = (float)g_cycle_count;  return true;
    case JBD_BMS_CH_PROTECTION:   *value = (float)g_protection;   return true;
    default:
        if (channel >= JBD_BMS_CH_CELL_BASE &&
            channel < JBD_BMS_CH_CELL_BASE + JBD_MAX_CELLS) {
            *value = g_cell_voltages[channel - JBD_BMS_CH_CELL_BASE];
            return true;
        }
        return false;
    }
}

static void drv_set_defaults(uint8_t channel, pin_config_t* config)
{
    (void)channel;
    config->params.pathfinder_bms.poll_interval_ms = JBD_BMS_DEFAULT_POLL_INTERVAL_MS;
}

static bool drv_apply_config(uint8_t channel, const pin_config_t* config)
{
    (void)channel;
    uint16_t interval = config->params.pathfinder_bms.poll_interval_ms;
    if (interval >= 100) g_poll_interval_ms = interval;
    pathfinder_bms_init();
    bms_open();
    return true;
}

static bool drv_parse_json(const char* json_start, const char* json_end,
                            pin_config_t* config)
{
    const char* p = strstr(json_start, "\"poll_interval_ms\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) { p++; while (*p == ' ') p++;
                 config->params.pathfinder_bms.poll_interval_ms = (uint16_t)atoi(p); }
    }
    uint8_t tx, rx, inst;
    if (uart_pin_pair_parse_json(json_start, json_end, &tx, &rx, &inst)) {
        g_tx_pin = tx;
        g_rx_pin = rx;
    }
    return true;
}

static void drv_estop(void)
{
    saint_log_publish("info",
        "Pathfinder BMS: ESTOP (no action — read-only sensor)");
}

static bool drv_save(void* storage_ptr)
{
    flash_storage_data_t* storage = (flash_storage_data_t*)storage_ptr;
    memset(&storage->pathfinder_bms_config, 0, sizeof(storage->pathfinder_bms_config));
    storage->pathfinder_bms_config.enabled          = g_port_initialized ? 1 : 0;
    storage->pathfinder_bms_config.poll_interval_ms = g_poll_interval_ms;
    const pathfinder_bms_transport_ops_t* t = transport();
    storage->pathfinder_bms_config.serial_port =
        (t && t->resolved_instance) ? t->resolved_instance() : 0xFF;

    storage->uart_pins.pathfinder_bms_tx_pin = g_tx_pin;
    storage->uart_pins.pathfinder_bms_rx_pin = g_rx_pin;
    return true;
}

static bool drv_load(const void* storage_ptr)
{
    const flash_storage_data_t* storage = (const flash_storage_data_t*)storage_ptr;

    pathfinder_bms_init();   /* seed state defaults */

    if (storage->uart_pins.pathfinder_bms_tx_pin != 0 ||
        storage->uart_pins.pathfinder_bms_rx_pin != 0) {
        g_tx_pin = storage->uart_pins.pathfinder_bms_tx_pin;
        g_rx_pin = storage->uart_pins.pathfinder_bms_rx_pin;
    }

    if (!storage->pathfinder_bms_config.enabled) return true;

    if (storage->pathfinder_bms_config.poll_interval_ms >= 100) {
        g_poll_interval_ms = storage->pathfinder_bms_config.poll_interval_ms;
    }

    bms_open();
    saint_log_publish("info",
        "Pathfinder BMS: restored config from flash (poll %ums)",
        (unsigned)g_poll_interval_ms);
    return true;
}

static const peripheral_driver_t pathfinder_bms_peripheral = {
    .name              = "pathfinder_bms",
    .mode_string       = "pathfinder_bms_sensor",
    .pin_mode          = PIN_MODE_PATHFINDER_BMS,
    .capability_flag   = PIN_CAP_PATHFINDER_BMS,
    .virtual_gpio_base = JBD_BMS_VIRTUAL_GPIO_BASE,
    .channel_count          = JBD_BMS_CHANNEL_COUNT,
    .channels_per_instance  = JBD_BMS_CHANNEL_COUNT,
    .init              = drv_init,
    .update            = pathfinder_bms_update,
    .is_connected      = pathfinder_bms_is_connected,
    .set_value         = drv_set_value,
    .get_value         = drv_get_value,
    .set_defaults      = drv_set_defaults,
    .apply_config      = drv_apply_config,
    .parse_json_params = drv_parse_json,
    .estop             = drv_estop,
    .save_config       = drv_save,
    .load_config       = drv_load,
};

const peripheral_driver_t* pathfinder_bms_get_peripheral_driver(void)
{
    return &pathfinder_bms_peripheral;
}
