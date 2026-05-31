/**
 * SAINT.OS Firmware - SyRen 50 driver core (shared)
 *
 * Owns: per-channel config state, protocol byte assembly (via the
 * existing shared syren_build_packet inline), and the
 * peripheral_driver_t glue (set_value / get_value / save / load /
 * parse_json / estop). All UART I/O dispatches through the per-
 * platform transport ops table (shared/include/syren_transport.h).
 *
 * Was previously duplicated between firmware/rp2040/src/syren_driver.c
 * and firmware/teensy41/src/syren_driver.cpp — those copies drifted
 * (README item 4: the RP2040 copy kept losing channel configs across
 * reboots because drv_init clobbered them; the Teensy copy had the
 * same latent bug). One implementation can't drift from itself.
 */

#include "syren_driver.h"
#include "syren_transport.h"

#include "flash_types.h"
#include "peripheral_driver.h"
#include "pin_types.h"
#include "saint_log.h"
#include "uart_pin_pairs.h"

#include <stdlib.h>
#include <string.h>

/* ── Module state ───────────────────────────────────────────────── */

static syren_channel_config_t g_channel_configs[SYREN_MAX_CHANNELS];
static uint8_t  g_tx_pin    = 0;          /* 0 = transport's default   */
static uint8_t  g_rx_pin    = 0;
static uint16_t g_baud      = SYREN_DEFAULT_BAUD;
static bool     g_initialized = false;

static const syren_transport_ops_t* g_transport(void)
{
    return syren_get_transport();
}

/* ── Public API ─────────────────────────────────────────────────── */

void syren_init(void)
{
    /* Data-only init. Idempotency guard ensures defaults are set
     * only ONCE per boot — drv_load / drv_apply_config may call this
     * after they've already written stored values, and re-running the
     * defaults loop would clobber them.
     *
     * Hardware (UART open + autobaud byte) is deferred to
     * drv_load / drv_apply_config via the transport ops, so the chip
     * doesn't grab pins at boot unless something is configured.
     * Mirrors the pattern documented in firmware/rp2040/README.md item 4. */
    if (g_initialized) return;
    for (uint8_t i = 0; i < SYREN_MAX_CHANNELS; i++) {
        g_channel_configs[i].address    = SYREN_ADDRESS_MIN + i;
        g_channel_configs[i].deadband   = 0;
        g_channel_configs[i].ramping    = 0;
        g_channel_configs[i].timeout_ms = 0;
    }
    g_initialized = true;
}

/* Open UART through transport, then send autobaud byte. Idempotent. */
static bool syren_open(void)
{
    const syren_transport_ops_t* t = g_transport();
    if (!t) return false;
    if (t->is_open()) return true;

    if (!t->open(g_tx_pin, g_rx_pin, g_baud)) {
        saint_log_publish("error",
            "SyRen: %s transport open failed (tx=%d rx=%d baud=%u)",
            t->name, g_tx_pin, g_rx_pin, (unsigned)g_baud);
        return false;
    }

    /* SyRen autobaud: the controller waits to see a 0xAA byte at boot
     * and locks to whatever baud rate that byte arrives at. Without
     * it the controller stays in its default-baud listen mode and
     * subsequent packets at our configured baud are ignored. */
    uint8_t autobaud = SYREN_AUTOBAUD_BYTE;
    (void)t->write(&autobaud, 1);

    saint_log_publish("info",
        "SyRen: opened on %s (tx=%d rx=%d baud=%u)",
        t->name, g_tx_pin, g_rx_pin, (unsigned)g_baud);
    return true;
}

void syren_update(void) {
    /* No async polling — SyRen is write-only. Kept for symmetry with
     * the peripheral_driver_t.update slot. */
}

bool syren_is_connected(void)
{
    const syren_transport_ops_t* t = g_transport();
    return t && t->is_open && t->is_open();
}

bool syren_set_power(uint8_t channel, int16_t power)
{
    if (channel >= SYREN_MAX_CHANNELS || !syren_is_connected()) return false;

    if (power > 127)  power = 127;
    if (power < -127) power = -127;

    uint8_t address = g_channel_configs[channel].address;
    uint8_t command;
    uint8_t value;
    if (power >= 0) {
        command = SYREN_CMD_MOTOR_FWD;
        value   = (uint8_t)power;
    } else {
        command = SYREN_CMD_MOTOR_REV;
        value   = (uint8_t)(-power);
    }

    uint8_t packet[4];
    syren_build_packet(address, command, value, packet);
    return g_transport()->write(packet, 4);
}

bool syren_stop(uint8_t channel)
{
    return syren_set_power(channel, 0);
}

void syren_stop_all(void)
{
    for (uint8_t i = 0; i < SYREN_MAX_CHANNELS; i++) {
        syren_stop(i);
    }
}

void syren_set_channel_config(uint8_t channel,
                               const syren_channel_config_t* config)
{
    if (channel >= SYREN_MAX_CHANNELS || !config) return;
    g_channel_configs[channel] = *config;

    /* If the link is up, push deadband/ramping/timeout to the device
     * now. These are persistent settings on the SyRen; without an
     * explicit write the unit stays at whatever the prior session
     * left it at. */
    if (!syren_is_connected()) return;
    const syren_transport_ops_t* t = g_transport();
    uint8_t packet[4];
    syren_build_packet(config->address, SYREN_CMD_DEADBAND,
                       config->deadband, packet);
    (void)t->write(packet, 4);
    syren_build_packet(config->address, SYREN_CMD_RAMPING,
                       config->ramping, packet);
    (void)t->write(packet, 4);
    /* timeout register accepts a single byte (100ms units), so clamp. */
    uint8_t timeout_byte = (config->timeout_ms / 100u) & 0x7F;
    syren_build_packet(config->address, SYREN_CMD_SERIAL_TIMEOUT,
                       timeout_byte, packet);
    (void)t->write(packet, 4);
}

const syren_channel_config_t* syren_get_channel_config(uint8_t channel)
{
    if (channel >= SYREN_MAX_CHANNELS) return NULL;
    return &g_channel_configs[channel];
}

/* ── peripheral_driver_t glue ───────────────────────────────────── */

static bool drv_init(void)
{
    /* Intentional no-op. peripheral_init_all runs AFTER pin_config_load,
     * so doing anything here would clobber configs that drv_load just
     * restored from flash. Real init runs from drv_load (flash had
     * channels) or drv_apply_config (operator sync). README item 3. */
    return true;
}

static bool drv_set_value(uint8_t channel, float value)
{
    if (value < -1.0f) value = -1.0f;
    if (value > 1.0f)  value = 1.0f;
    return syren_set_power(channel, (int16_t)(value * 127.0f));
}

static bool drv_get_value(uint8_t channel, float* value)
{
    (void)channel; (void)value;
    /* SyRen has no position / status readback. */
    return false;
}

static void drv_set_defaults(uint8_t channel, pin_config_t* config)
{
    config->params.syren.address    = SYREN_ADDRESS_MIN + channel;
    config->params.syren.deadband   = 0;
    config->params.syren.ramping    = 0;
    config->params.syren.reserved   = 0;
    config->params.syren.timeout_ms = 0;
}

static bool drv_apply_config(uint8_t channel, const pin_config_t* config)
{
    if (channel >= SYREN_MAX_CHANNELS) return false;
    syren_init();    /* idempotent — seeds defaults if not already */
    syren_open();    /* idempotent — opens UART if not already      */

    syren_channel_config_t scfg = {
        .address    = config->params.syren.address,
        .deadband   = config->params.syren.deadband,
        .ramping    = config->params.syren.ramping,
        .timeout_ms = config->params.syren.timeout_ms,
    };
    syren_set_channel_config(channel, &scfg);
    return true;
}

static bool drv_parse_json(const char* json_start, const char* json_end,
                            pin_config_t* config)
{
    const char* p;
    #define PARSE_U(key, into, cast) do {                                       \
        p = strstr(json_start, "\"" key "\"");                                  \
        if (p && p < json_end) {                                                \
            p = strchr(p, ':');                                                 \
            if (p) { p++; while (*p == ' ') p++;                                \
                     into = (cast)atoi(p); }                                    \
        }                                                                       \
    } while (0)
    PARSE_U("address",    config->params.syren.address,    uint8_t);
    PARSE_U("deadband",   config->params.syren.deadband,   uint8_t);
    PARSE_U("ramping",    config->params.syren.ramping,    uint8_t);
    PARSE_U("timeout_ms", config->params.syren.timeout_ms, uint16_t);
    #undef PARSE_U

    /* Pin overrides go to module state so a subsequent drv_apply_config
     * (which calls syren_open) hands them to the transport. */
    uint8_t tx, rx, inst;
    if (uart_pin_pair_parse_json(json_start, json_end, &tx, &rx, &inst)) {
        g_tx_pin = tx;
        g_rx_pin = rx;
    }
    return true;
}

static void drv_estop(void)
{
    syren_stop_all();
    saint_log_publish("warn", "SyRen: ESTOP — all channels stopped");
}

static bool drv_save(void* storage_ptr)
{
    flash_storage_data_t* storage = (flash_storage_data_t*)storage_ptr;

    memset(&storage->syren_config, 0, sizeof(storage->syren_config));
    storage->syren_config.channel_count = SYREN_MAX_CHANNELS;
    storage->syren_config.baud_rate     = g_baud;
    const syren_transport_ops_t* t = g_transport();
    storage->syren_config.serial_port   = t ? t->resolved_instance() : 0xFF;

    storage->uart_pins.syren_tx_pin = g_tx_pin;
    storage->uart_pins.syren_rx_pin = g_rx_pin;

    for (uint8_t ch = 0; ch < SYREN_MAX_CHANNELS; ch++) {
        storage->syren_config.channels[ch].address    = g_channel_configs[ch].address;
        storage->syren_config.channels[ch].deadband   = g_channel_configs[ch].deadband;
        storage->syren_config.channels[ch].ramping    = g_channel_configs[ch].ramping;
        storage->syren_config.channels[ch].timeout_ms = g_channel_configs[ch].timeout_ms;
    }
    return true;
}

static bool drv_load(const void* storage_ptr)
{
    const flash_storage_data_t* storage = (const flash_storage_data_t*)storage_ptr;

    /* Seed defaults FIRST so any unconfigured channel slots start at
     * the conventional 128+ch addresses rather than the BSS-zero we'd
     * otherwise see (would confuse the dashboard's channel picker). */
    syren_init();

    /* Validate pin overrides from flash against the platform's known
     * UART pin pairs before honoring them. 0xFF/0xFF (or any other
     * combo the platform doesn't actually wire to a UART) means the
     * flash slot was never properly written — typically a pre-refactor
     * flash layout being reinterpreted by post-refactor code. Treat
     * the record as unconfigured rather than opening the UART with
     * bogus pin numbers. Mirrors roboclaw_drv_load. */
    uint8_t stored_tx = storage->uart_pins.syren_tx_pin;
    uint8_t stored_rx = storage->uart_pins.syren_rx_pin;
    bool pins_valid = false;
    if (stored_tx != 0 || stored_rx != 0) {
        uint8_t inst;
        if (uart_pin_pair_lookup(stored_tx, stored_rx, &inst)) {
            g_tx_pin = stored_tx;
            g_rx_pin = stored_rx;
            pins_valid = true;
        } else {
            saint_log_publish("warn",
                "SyRen: stored pin pair TX=%u RX=%u isn't a valid UART pair "
                "— driver dormant. Re-sync a SyRen config from the dashboard.",
                (unsigned)stored_tx, (unsigned)stored_rx);
        }
    }

    if (storage->syren_config.channel_count == 0) {
        saint_log_publish("info",
            "SyRen: no saved peripheral in flash — driver dormant. "
            "Sync a SyRen config from the dashboard to bind the UART.");
        return true;
    }

    /* channel_count says configured but the pin pair is invalid — the
     * flash record is inconsistent. Don't open the UART on garbage;
     * wait for a fresh sync. */
    if (!pins_valid) return true;

    uint8_t count = storage->syren_config.channel_count;
    if (count > SYREN_MAX_CHANNELS) count = SYREN_MAX_CHANNELS;

    if (storage->syren_config.baud_rate > 0) {
        g_baud = storage->syren_config.baud_rate;
    }

    for (uint8_t ch = 0; ch < count; ch++) {
        g_channel_configs[ch].address    = storage->syren_config.channels[ch].address;
        g_channel_configs[ch].deadband   = storage->syren_config.channels[ch].deadband;
        g_channel_configs[ch].ramping    = storage->syren_config.channels[ch].ramping;
        g_channel_configs[ch].timeout_ms = storage->syren_config.channels[ch].timeout_ms;
    }

    /* Flash had configured channels — open the UART now using the
     * restored pin/baud values. */
    syren_open();

    saint_log_publish("info",
        "SyRen: restored %u channel configs from flash", (unsigned)count);
    return true;
}

static const peripheral_driver_t syren_peripheral = {
    .name              = "syren",
    .mode_string       = "syren_motor",
    .pin_mode          = PIN_MODE_SYREN_MOTOR,
    .capability_flag   = PIN_CAP_SYREN_MOTOR,
    .virtual_gpio_base = SYREN_VIRTUAL_GPIO_BASE,
    .channel_count          = SYREN_MAX_CHANNELS,
    .channels_per_instance  = 1,
    .init              = drv_init,
    .update            = syren_update,
    .is_connected      = syren_is_connected,
    .set_value         = drv_set_value,
    .get_value         = drv_get_value,
    .set_defaults      = drv_set_defaults,
    .apply_config      = drv_apply_config,
    .parse_json_params = drv_parse_json,
    .estop             = drv_estop,
    .save_config       = drv_save,
    .load_config       = drv_load,
};

const peripheral_driver_t* syren_get_peripheral_driver(void)
{
    return &syren_peripheral;
}
