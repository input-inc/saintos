/**
 * SAINT.OS Firmware - Pololu Maestro-24 driver core (shared)
 *
 * Owns: channel-config state, compact-protocol byte assembly, the
 * peripheral_driver_t glue (set_value/get_value/save/load/parse_json/...).
 *
 * Doesn't own: the physical link to the Maestro — that's a per-platform
 * transport ops table picked at maestro_drv_init based on the saved
 * config's transport_mode. See shared/include/maestro_transport.h.
 *
 * Hot-plug-aware features (connect/disconnect announce, descriptor
 * probe) are gated on transport->supports_hotplug; on UART links the
 * driver behaves as if the device is always present once the port
 * opens.
 */

#include "maestro_driver.h"
#include "maestro_transport.h"

#include "flash_types.h"
#include "peripheral_driver.h"
#include "pin_types.h"
#include "saint_log.h"

#include <stdlib.h>
#include <string.h>

/* ── Module state ────────────────────────────────────────────────── */

static const maestro_transport_ops_t* g_transport = NULL;
static bool     g_initialized   = false;
static bool     g_was_connected = false;
static uint32_t g_transport_mode = FLASH_MAESTRO_TRANSPORT_USB_HOST;

static maestro_channel_config_t g_channel_configs[MAESTRO_MAX_CHANNELS];

/* Resolve a transport ops table for the given flash mode. Returns NULL
 * if the platform doesn't provide that transport — caller logs and
 * leaves the driver inert. */
static const maestro_transport_ops_t* pick_transport(uint8_t mode)
{
    switch (mode) {
        case FLASH_MAESTRO_TRANSPORT_USB_HOST:
            return maestro_get_transport_usb_host();
        case FLASH_MAESTRO_TRANSPORT_UART:
            return maestro_get_transport_uart();
        default:
            return NULL;
    }
}

/* Pick + open a transport for the given mode. Idempotent: if the
 * requested transport is already bound, returns true without re-
 * opening. Used by both drv_load (boot-time flash restore, with
 * storage) and drv_parse_json (fresh dashboard sync, no storage yet)
 * so a freshly-configured Maestro starts pumping USB / UART within
 * the same tick the config arrived — without this the operator had
 * to power-cycle before the transport was ever bound. */
static bool bind_transport(uint8_t mode, const flash_storage_data_t* storage)
{
    const maestro_transport_ops_t* picked = pick_transport(mode);
    if (!picked) {
        saint_log_publish("error",
            "Maestro: no transport for mode=%u on this platform",
            (unsigned)mode);
        g_transport = NULL;
        return false;
    }
    if (picked == g_transport) {
        return true;   /* already bound */
    }
    g_transport = picked;
    if (g_transport->open && !g_transport->open(storage)) {
        saint_log_publish("error",
            "Maestro: %s transport open failed",
            g_transport->name);
        g_transport = NULL;
        return false;
    }
    saint_log_publish("info",
        "Maestro: bound %s transport", g_transport->name);
    return true;
}

/* Defaults setup, idempotent. Called from both drv_init and drv_load —
 * the peripheral framework orders these as register → load_config →
 * init, so without the idempotent guard maestro_init would clobber
 * freshly-loaded channel configs back to defaults. */
static void ensure_state_init(void)
{
    if (g_initialized) return;
    for (int i = 0; i < MAESTRO_MAX_CHANNELS; i++) {
        g_channel_configs[i].min_pulse_us = MAESTRO_DEFAULT_MIN_PULSE;
        g_channel_configs[i].max_pulse_us = MAESTRO_DEFAULT_MAX_PULSE;
        g_channel_configs[i].neutral_us   = MAESTRO_DEFAULT_NEUTRAL;
        g_channel_configs[i].speed        = 0;
        g_channel_configs[i].acceleration = 0;
        g_channel_configs[i].home_us      = 0;
    }
    /* Pick a sensible default transport for *this* platform: USB host
     * if the platform provides it (Teensy), otherwise UART (RP2040).
     * Without this a fresh RP2040 device would save transport_mode=0
     * (USB host) and then refuse to talk on next boot — get_transport
     * returns NULL on platforms that can't host USB. */
    if (maestro_get_transport_usb_host() != NULL) {
        g_transport_mode = FLASH_MAESTRO_TRANSPORT_USB_HOST;
    } else if (maestro_get_transport_uart() != NULL) {
        g_transport_mode = FLASH_MAESTRO_TRANSPORT_UART;
    }
    g_initialized = true;
}

/* ── Public lifecycle (called from peripheral framework + main) ──── */

void maestro_init(void)
{
    bool first = !g_initialized;
    ensure_state_init();
    if (first) {
        /* Transport stays NULL until load_config picks it from flash —
         * the peripheral_register call in main.c runs before flash
         * read on some platforms, so we can't bind transport here. */
        saint_log_publish("info",
            "Maestro: driver registered (transport bound on first config)");
    }
}

void maestro_update(void)
{
    if (!g_initialized || !g_transport) return;
    if (g_transport->update) g_transport->update();

    if (g_transport->supports_hotplug && g_transport->supports_hotplug()) {
        bool now = g_transport->is_connected();
        if (now && !g_was_connected) {
            saint_log_publish("info",
                "Maestro: device connected via %s", g_transport->name);
            (void)maestro_get_errors();  /* clear stale error flags */
        } else if (!now && g_was_connected) {
            saint_log_publish("warn",
                "Maestro: device disconnected (%s)", g_transport->name);
        }
        g_was_connected = now;
    }
}

bool maestro_is_connected(void)
{
    return g_transport && g_transport->is_connected
           ? g_transport->is_connected()
           : false;
}

uint8_t maestro_get_channel_count(void)
{
    return MAESTRO_MAX_CHANNELS;
}

/* ── Compact-protocol command formatting ─────────────────────────── */

static bool tx_bytes(const uint8_t* buf, size_t len)
{
    if (!g_transport || !g_transport->write) return false;
    return g_transport->write(buf, len);
}

bool maestro_set_target(uint8_t channel, uint16_t quarter_us)
{
    if (channel >= MAESTRO_MAX_CHANNELS) return false;
    uint8_t cmd[4] = {
        0x84,
        channel,
        (uint8_t)(quarter_us & 0x7F),
        (uint8_t)((quarter_us >> 7) & 0x7F),
    };
    return tx_bytes(cmd, sizeof(cmd));
}

bool maestro_set_speed(uint8_t channel, uint16_t speed)
{
    if (channel >= MAESTRO_MAX_CHANNELS) return false;
    uint8_t cmd[4] = {
        0x87,
        channel,
        (uint8_t)(speed & 0x7F),
        (uint8_t)((speed >> 7) & 0x7F),
    };
    return tx_bytes(cmd, sizeof(cmd));
}

bool maestro_set_acceleration(uint8_t channel, uint16_t accel)
{
    if (channel >= MAESTRO_MAX_CHANNELS) return false;
    uint8_t cmd[4] = {
        0x89,
        channel,
        (uint8_t)(accel & 0x7F),
        (uint8_t)((accel >> 7) & 0x7F),
    };
    return tx_bytes(cmd, sizeof(cmd));
}

uint16_t maestro_get_position(uint8_t channel)
{
    if (channel >= MAESTRO_MAX_CHANNELS) return 0;
    uint8_t cmd[2] = { 0x90, channel };
    if (!tx_bytes(cmd, sizeof(cmd))) return 0;

    uint8_t r[2];
    if (!g_transport || !g_transport->read) return 0;
    if (g_transport->read(r, sizeof(r), 10) != 2) return 0;
    return (uint16_t)r[0] | ((uint16_t)r[1] << 8);
}

uint16_t maestro_get_errors(void)
{
    uint8_t cmd = 0xA1;
    if (!tx_bytes(&cmd, 1)) return 0xFFFF;

    uint8_t r[2];
    if (!g_transport || !g_transport->read) return 0xFFFF;
    if (g_transport->read(r, sizeof(r), 10) != 2) return 0xFFFF;
    return (uint16_t)r[0] | ((uint16_t)r[1] << 8);
}

void maestro_go_home(void)
{
    uint8_t cmd = 0xA2;
    (void)tx_bytes(&cmd, 1);
}

/* ── Channel config ──────────────────────────────────────────────── */

void maestro_set_channel_config(uint8_t channel,
                                 const maestro_channel_config_t* config)
{
    if (channel >= MAESTRO_MAX_CHANNELS || !config) return;
    g_channel_configs[channel] = *config;

    /* Push speed/accel limits to the device whenever they're non-zero
     * AND the link is up. Pulse-range fields are software-only (we
     * only multiply them into set_target on output). */
    if (maestro_is_connected()) {
        if (config->speed > 0)        maestro_set_speed(channel, config->speed);
        if (config->acceleration > 0) maestro_set_acceleration(channel, config->acceleration);
    }
}

const maestro_channel_config_t* maestro_get_channel_config(uint8_t channel)
{
    if (channel >= MAESTRO_MAX_CHANNELS) return NULL;
    return &g_channel_configs[channel];
}

uint16_t maestro_angle_to_target(float angle,
                                  const maestro_channel_config_t* config)
{
    if (!config) return 0;
    if (angle < 0.0f)   angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;
    float pulse_us = (float)config->min_pulse_us +
                     (angle / 180.0f) *
                     (float)(config->max_pulse_us - config->min_pulse_us);
    return (uint16_t)(pulse_us * 4.0f);
}

/* ── peripheral_driver_t glue ────────────────────────────────────── */

static bool drv_init(void)
{
    maestro_init();
    return true;
}

static bool drv_set_value(uint8_t channel, float value)
{
    if (channel >= MAESTRO_MAX_CHANNELS) return false;
    const maestro_channel_config_t* cfg = maestro_get_channel_config(channel);
    if (!cfg) return false;

    /* Normalized input −1..+1, piecewise-linear through neutral so
     * asymmetric mechanical ranges (neutral not at midpoint of
     * min/max) map sensibly. Matches the wire shape pin_control's
     * native-servo path uses (firmware/teensy41/src/pin_control.cpp
     * :pin_control_set_servo) so the dashboard slider sends the
     * same scale regardless of whether the channel is a Teensy PWM
     * pin or a Maestro USB-host channel.
     *
     * Previous version treated `value` as an angle in degrees
     * (0..180) — at a slider value of 0.5 that pegs to ~0.5° which
     * maps to ~min_pulse_us; the servo always sat at min regardless
     * of slider position, which we mistook for working motion. */
    if (value < -1.0f) value = -1.0f;
    if (value >  1.0f) value =  1.0f;

    float center = (float)(cfg->neutral_us ? cfg->neutral_us
                          : ((cfg->min_pulse_us + cfg->max_pulse_us) / 2));
    float pulse_us;
    if (value <= 0.0f) {
        // [-1, 0]: lerp min → center
        pulse_us = center + value * (center - (float)cfg->min_pulse_us);
    } else {
        // [0, +1]: lerp center → max
        pulse_us = center + value * ((float)cfg->max_pulse_us - center);
    }

    /* Clamp to the channel's calibrated mechanical envelope so a
     * miscomputed value can't drive past the configured limits. */
    if (pulse_us < (float)cfg->min_pulse_us) pulse_us = (float)cfg->min_pulse_us;
    if (pulse_us > (float)cfg->max_pulse_us) pulse_us = (float)cfg->max_pulse_us;

    return maestro_set_target(channel, (uint16_t)(pulse_us * 4.0f));
}

static bool drv_get_value(uint8_t channel, float* value)
{
    if (channel >= MAESTRO_MAX_CHANNELS || !value) return false;
    if (!maestro_is_connected()) return false;

    uint16_t pos_qus = maestro_get_position(channel);
    if (pos_qus == 0) return false;
    const maestro_channel_config_t* cfg = maestro_get_channel_config(channel);
    if (!cfg || cfg->max_pulse_us <= cfg->min_pulse_us) return false;

    float pulse_us = (float)pos_qus / 4.0f;
    float angle = (pulse_us - (float)cfg->min_pulse_us) /
                  (float)(cfg->max_pulse_us - cfg->min_pulse_us) * 180.0f;
    if (angle < 0.0f)   angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;
    *value = angle;
    return true;
}

static void drv_set_defaults(uint8_t channel, pin_config_t* config)
{
    (void)channel;
    if (!config) return;
    config->params.maestro.min_pulse_us = MAESTRO_DEFAULT_MIN_PULSE;
    config->params.maestro.max_pulse_us = MAESTRO_DEFAULT_MAX_PULSE;
    config->params.maestro.neutral_us   = MAESTRO_DEFAULT_NEUTRAL;
    config->params.maestro.speed        = 0;
    config->params.maestro.acceleration = 0;
    config->params.maestro.home_us      = 0;
}

static bool drv_apply_config(uint8_t channel, const pin_config_t* config)
{
    if (channel >= MAESTRO_MAX_CHANNELS || !config) return false;
    maestro_channel_config_t mcfg = {
        .min_pulse_us  = config->params.maestro.min_pulse_us,
        .max_pulse_us  = config->params.maestro.max_pulse_us,
        .neutral_us    = config->params.maestro.neutral_us,
        .speed         = config->params.maestro.speed,
        .acceleration  = config->params.maestro.acceleration,
        .home_us       = config->params.maestro.home_us,
    };
    maestro_set_channel_config(channel, &mcfg);
    return true;
}

static bool drv_parse_json(const char* json_start, const char* json_end,
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
    #define MAESTRO_PARSE_U16(key, into)                                          \
        do {                                                                       \
            p = strstr(json_start, "\"" key "\"");                                 \
            if (p && p < json_end) {                                               \
                p = strchr(p, ':');                                                \
                if (p) { p++; while (*p == ' ') p++; into = (uint16_t)atoi(p); }   \
            }                                                                      \
        } while (0)
    MAESTRO_PARSE_U16("min_pulse_us", min_p);
    MAESTRO_PARSE_U16("max_pulse_us", max_p);
    MAESTRO_PARSE_U16("neutral_us",   neut);
    MAESTRO_PARSE_U16("speed",        spd);
    MAESTRO_PARSE_U16("acceleration", acc);
    MAESTRO_PARSE_U16("home_us",      home);
    #undef MAESTRO_PARSE_U16

    /* Optional "transport":"usb_host"|"uart" — lets the dashboard pick
     * the mode at config time. Falls back to whatever ensure_state_init
     * defaulted to for this platform. */
    p = strstr(json_start, "\"transport\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) {
            p++;
            while (*p == ' ' || *p == '"') p++;
            if (strncmp(p, "uart", 4) == 0) {
                g_transport_mode = FLASH_MAESTRO_TRANSPORT_UART;
            } else if (strncmp(p, "usb_host", 8) == 0 ||
                       strncmp(p, "usb", 3)      == 0) {
                g_transport_mode = FLASH_MAESTRO_TRANSPORT_USB_HOST;
            }
        }
    }
    /* Bind + open the transport right now. Previously this only ran
     * inside drv_load (boot-time flash restore), so a fresh sync from
     * the dashboard left g_transport NULL until the next reboot —
     * maestro_update() would no-op and on USB host platforms
     * myusb.Task() would never tick, so the Maestro never enumerated
     * (no LEDs, no movement, no live polling). Storage is NULL here
     * because the parse path runs before drv_save; UART transports
     * fall back to compile-time default pins until the next reboot
     * picks up the just-saved pair. */
    (void)bind_transport((uint8_t)g_transport_mode, NULL);

    config->params.maestro.min_pulse_us  = min_p;
    config->params.maestro.max_pulse_us  = max_p;
    config->params.maestro.neutral_us    = neut;
    config->params.maestro.speed         = spd;
    config->params.maestro.acceleration  = acc;
    config->params.maestro.home_us       = home;
    return true;
}

static void drv_estop(void)
{
    if (maestro_is_connected()) maestro_go_home();
}

static bool drv_save(void* storage)
{
    flash_storage_data_t* s = (flash_storage_data_t*)storage;
    if (!s) return false;
    s->maestro_config.channel_count  = MAESTRO_MAX_CHANNELS;
    s->maestro_config.transport_mode = (uint8_t)g_transport_mode;
    /* serial_port + uart_pins are written by per-platform transport
     * (mirrors how syren/roboclaw save their port assignment). */
    for (uint8_t ch = 0; ch < MAESTRO_MAX_CHANNELS; ch++) {
        const maestro_channel_config_t* mcfg = maestro_get_channel_config(ch);
        if (!mcfg) continue;
        s->maestro_config.channels[ch].min_pulse_us  = mcfg->min_pulse_us;
        s->maestro_config.channels[ch].max_pulse_us  = mcfg->max_pulse_us;
        s->maestro_config.channels[ch].neutral_us    = mcfg->neutral_us;
        s->maestro_config.channels[ch].speed         = mcfg->speed;
        s->maestro_config.channels[ch].acceleration  = mcfg->acceleration;
        s->maestro_config.channels[ch].home_us       = mcfg->home_us;
    }
    return true;
}

static bool drv_load(const void* storage)
{
    const flash_storage_data_t* s = (const flash_storage_data_t*)storage;
    if (!s) return false;

    /* Make sure defaults are in place BEFORE we overlay flash values —
     * the peripheral framework's order is register → load_config →
     * init, so without this drv_init would later clobber whatever we
     * loaded with defaults. */
    ensure_state_init();

    /* Detect "no Maestro ever saved on this node." drv_save always
     * writes channel_count = MAESTRO_MAX_CHANNELS and transport_mode in
     * [0, 1]; a saved record never has channel_count == 0 or 0xFF and
     * never has transport_mode > FLASH_MAESTRO_TRANSPORT_UART. Reading
     * any of those means the maestro_config bytes are either zero-
     * initialized (never written) or erased flash (0xFF) — typical when
     * the operator has no Maestro hardware on this node. Bail before
     * we try to bind a transport, which would otherwise log a spurious
     * "no transport for mode=255" error on every boot. */
    const uint8_t saved_cc = s->maestro_config.channel_count;
    const uint8_t saved_tm = s->maestro_config.transport_mode;
    if (saved_cc == 0 || saved_cc == 0xFF
        || saved_tm > FLASH_MAESTRO_TRANSPORT_UART) {
        return true;
    }

    /* Restore channel configs. */
    if (s->maestro_config.channel_count > 0) {
        uint8_t count_m = s->maestro_config.channel_count;
        if (count_m > MAESTRO_MAX_CHANNELS) count_m = MAESTRO_MAX_CHANNELS;
        for (uint8_t ch = 0; ch < count_m; ch++) {
            maestro_channel_config_t mcfg = {
                .min_pulse_us = s->maestro_config.channels[ch].min_pulse_us,
                .max_pulse_us = s->maestro_config.channels[ch].max_pulse_us,
                .neutral_us   = s->maestro_config.channels[ch].neutral_us,
                .speed        = s->maestro_config.channels[ch].speed,
                .acceleration = s->maestro_config.channels[ch].acceleration,
                .home_us      = s->maestro_config.channels[ch].home_us,
            };
            if (mcfg.min_pulse_us > 0 || mcfg.max_pulse_us > 0) {
                g_channel_configs[ch] = mcfg;
            }
        }
    }

    /* Bind transport based on the saved mode. transport_mode is zero-
     * default → USB host, which preserves existing Teensy behavior on
     * upgrade. Platforms that can't supply the requested transport
     * leave the driver inert (bind_transport logs once). */
    g_transport_mode = s->maestro_config.transport_mode;
    (void)bind_transport((uint8_t)g_transport_mode, s);
    return true;  /* failures already logged; don't abort flash load */
}

static const peripheral_driver_t maestro_peripheral = {
    .name              = "maestro",
    .mode_string       = "maestro_servo",
    .pin_mode          = PIN_MODE_MAESTRO_SERVO,
    .capability_flag   = PIN_CAP_MAESTRO_SERVO,
    .virtual_gpio_base = MAESTRO_VIRTUAL_GPIO_BASE,
    .channel_count          = MAESTRO_MAX_CHANNELS,
    .channels_per_instance  = MAESTRO_MAX_CHANNELS,
    .init              = drv_init,
    .update            = maestro_update,
    .is_connected      = maestro_is_connected,
    .set_value         = drv_set_value,
    .get_value         = drv_get_value,
    .set_defaults      = drv_set_defaults,
    .apply_config      = drv_apply_config,
    .parse_json_params = drv_parse_json,
    .estop             = drv_estop,
    .save_config       = drv_save,
    .load_config       = drv_load,
};

const peripheral_driver_t* maestro_get_peripheral_driver(void)
{
    return &maestro_peripheral;
}
