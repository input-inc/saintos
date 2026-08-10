/**
 * SAINT.OS Firmware - Pimoroni Servo 2040 driver core (shared)
 *
 * Host-side (I2C master) driver for the Pimoroni Servo 2040 servo
 * controller. Owns per-channel servo extents + home, onboard-LED
 * color/brightness state, the current-telemetry cache, and the
 * peripheral_driver_t glue. Doesn't own the physical link — that's a
 * per-platform I2C transport ops table
 * (shared/include/pimoroni_servo2040_transport.h).
 *
 * Wire contract: the I2C register map in
 * shared/include/pimoroni_servo2040_protocol.h, spoken to the board's
 * fixed Pimoroni-SDK firmware (firmware/pimoroni_servo2040/), which is an
 * I2C target on the board's Qwiic/STEMMA-QT connector.
 *
 * Maestro parity:
 *   - EXTENTS are host-side: drv_set_value maps normalized −1..+1 → pulse
 *     and clamps, then writes SERVO_TARGET (mirrors the Maestro driver's
 *     per-channel min/max clamp).
 *   - HOME mirrors the Maestro's EEPROM HomeMode=Goto: on a config change
 *     we write SERVO_HOME[ch] + COMMIT so the board persists home pulses
 *     to its own flash and drives them on power-on (before the host even
 *     connects). We also re-home on connect.
 */

#include "pimoroni_servo2040_driver.h"
#include "pimoroni_servo2040_transport.h"

#include "flash_types.h"
#include "peripheral_driver.h"
#include "pin_types.h"
#include "platform.h"
#include "saint_log.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ── Module state ────────────────────────────────────────────────── */

static const pimoroni_servo2040_transport_ops_t* g_transport = NULL;
static bool     g_initialized   = false;
static bool     g_board_present = false;  /* last WHOAMI poll succeeded */
static bool     g_homed         = false;  /* runtime home sent this connection */

/* Set when a config sync changes per-channel home/extents; drives a
 * board-flash home re-provision on the next update() tick (Maestro
 * g_config_dirty analogue). */
static bool     g_config_dirty  = false;

/* Operator-configured peripheral id (for telemetry labelling). */
static char     g_peripheral_id[48] = "pimoroni_servo2040";

/* Per-channel servo extents. */
static pimoroni_servo2040_channel_config_t
    g_channel_configs[PIMORONI_SERVO2040_NUM_SERVOS];

/* Onboard-LED state (packed 0xRRGGBB per pixel) + global brightness. */
static uint32_t g_led_rgb[PIMORONI_SERVO2040_NUM_LEDS];
static uint8_t  g_led_brightness = 255;

/* I2C SDA/SCL pins captured from the config push (Qwiic pins), persisted
 * by drv_save into flash_uart_pins_t; the transport's open() reads them
 * back. 0 = use the platform default pair. */
static uint8_t  g_sda_pin = 0;
static uint8_t  g_scl_pin = 0;

/* Telemetry cache, refreshed in update() at POLL_MS cadence. */
static float    g_current_amps = 0.0f;
static uint8_t  g_flags        = 0;

/* Cadence bookkeeping. */
static uint32_t g_last_ping_ms = 0;
static uint32_t g_last_poll_ms = 0;

/* ── Defaults ────────────────────────────────────────────────────── */

static void ensure_state_init(void)
{
    if (g_initialized) return;
    for (int i = 0; i < PIMORONI_SERVO2040_NUM_SERVOS; i++) {
        g_channel_configs[i].start_us  = PIMORONI_SERVO2040_MIN_PULSE_US;
        g_channel_configs[i].end_us    = PIMORONI_SERVO2040_MAX_PULSE_US;
        g_channel_configs[i].center_us = PIMORONI_SERVO2040_MID_PULSE_US;
        g_channel_configs[i].home_us   = 0;   /* relaxed until commanded */
    }
    for (int i = 0; i < PIMORONI_SERVO2040_NUM_LEDS; i++) g_led_rgb[i] = 0;
    g_led_brightness = 255;
    g_initialized = true;
}

/* Bind + open the I2C transport. Idempotent. */
static bool bind_transport(const flash_storage_data_t* storage)
{
    const pimoroni_servo2040_transport_ops_t* picked =
        pimoroni_servo2040_get_transport_i2c();
    if (!picked) {
        if (g_transport != NULL) {
            saint_log_publish("error",
                "Servo2040: no I2C transport on this platform");
        }
        g_transport = NULL;
        return false;
    }
    if (picked == g_transport) return true;   /* already bound */

    g_transport = picked;
    if (g_transport->open && !g_transport->open(storage)) {
        saint_log_publish("error", "Servo2040: I2C transport open failed");
        g_transport = NULL;
        return false;
    }
    saint_log_publish("info", "Servo2040: bound %s transport",
                      g_transport->name ? g_transport->name : "i2c");
    return true;
}

/* ── Register helpers ────────────────────────────────────────────── */

static bool write_reg(uint8_t reg, const uint8_t* data, size_t len)
{
    if (!g_transport || !g_transport->write_reg) return false;
    return g_transport->write_reg(reg, data, len);
}

static bool write_reg_u8(uint8_t reg, uint8_t v)
{
    return write_reg(reg, &v, 1);
}

static bool write_reg_u16(uint8_t reg, uint16_t v)
{
    uint8_t b[2] = { (uint8_t)(v & 0xFF), (uint8_t)((v >> 8) & 0xFF) };  /* LE */
    return write_reg(reg, b, 2);
}

static bool read_reg(uint8_t reg, uint8_t* data, size_t len)
{
    if (!g_transport || !g_transport->read_reg) return false;
    return g_transport->read_reg(reg, data, len);
}

static bool send_servo_target(uint8_t ch, uint16_t pulse_us)
{
    return write_reg_u16(
        (uint8_t)(PIMORONI_SERVO2040_REG_SERVO_TARGET_BASE + ch * 2), pulse_us);
}

static bool send_led(uint8_t idx, uint32_t rgb)
{
    uint8_t b[3] = {
        (uint8_t)((rgb >> 16) & 0xFF),
        (uint8_t)((rgb >> 8) & 0xFF),
        (uint8_t)(rgb & 0xFF),
    };
    return write_reg((uint8_t)(PIMORONI_SERVO2040_REG_LED_BASE + idx * 3), b, 3);
}

/* Normalized −1..+1 → pulse µs, piecewise-linear through center. Mirrors
 * the Maestro / native-servo mapping. */
static uint16_t normalized_to_pulse(uint8_t channel, float value)
{
    const pimoroni_servo2040_channel_config_t* c = &g_channel_configs[channel];
    if (value < -1.0f) value = -1.0f;
    if (value >  1.0f) value =  1.0f;
    float center = (float)c->center_us;
    float pulse;
    if (value <= 0.0f) {
        pulse = center + value * (center - (float)c->start_us);
    } else {
        pulse = center + value * ((float)c->end_us - center);
    }
    if (pulse < (float)PIMORONI_SERVO2040_HARD_MIN_US) pulse = PIMORONI_SERVO2040_HARD_MIN_US;
    if (pulse > (float)PIMORONI_SERVO2040_HARD_MAX_US) pulse = PIMORONI_SERVO2040_HARD_MAX_US;
    return (uint16_t)(pulse + 0.5f);
}

/* Persist every channel's home pulse to the board's flash (Maestro
 * EEPROM HomeMode=Goto parity) so the board comes up homed on power-on
 * before the host connects. The board diff-checks before wearing its
 * flash on COMMIT. */
static void provision_home(void)
{
    if (!g_board_present) return;
    for (uint8_t ch = 0; ch < PIMORONI_SERVO2040_NUM_SERVOS; ch++) {
        (void)write_reg_u16(
            (uint8_t)(PIMORONI_SERVO2040_REG_SERVO_HOME_BASE + ch * 2),
            g_channel_configs[ch].home_us);
    }
    (void)write_reg_u8(PIMORONI_SERVO2040_REG_COMMIT, 1);
    g_config_dirty = false;
    saint_log_publish("info", "Servo2040: home positions provisioned to board flash");
}

/* Drive every configured servo to its home pulse now + push current LED
 * state. Called once on first observed-connected tick. */
static void apply_connect_state(void)
{
    uint8_t homed = 0;
    for (uint8_t ch = 0; ch < PIMORONI_SERVO2040_NUM_SERVOS; ch++) {
        uint16_t home_us = g_channel_configs[ch].home_us;
        if (home_us == 0) continue;
        (void)send_servo_target(ch, home_us);
        homed++;
    }
    (void)write_reg_u8(PIMORONI_SERVO2040_REG_BRIGHTNESS, g_led_brightness);
    for (uint8_t i = 0; i < PIMORONI_SERVO2040_NUM_LEDS; i++) {
        (void)send_led(i, g_led_rgb[i]);
    }
    /* Bring the board's persisted home in line with the current config. */
    provision_home();
    saint_log_publish("info",
        "Servo2040: connected — homed %u servo%s + pushed LED state",
        (unsigned)homed, homed == 1 ? "" : "s");
}

/* ── Public lifecycle ────────────────────────────────────────────── */

void pimoroni_servo2040_init(void)
{
    bool first = !g_initialized;
    ensure_state_init();
    if (first) {
        saint_log_publish("info",
            "Servo2040: driver registered (transport bound on first config)");
    }
}

void pimoroni_servo2040_update(void)
{
    if (!g_initialized || !g_transport) return;
    if (g_transport->update) g_transport->update();
    if (!(g_transport->is_connected && g_transport->is_connected())) return;

    uint32_t now = PLATFORM_MILLIS();

    /* Poll WHOAMI + telemetry at POLL_MS. WHOAMI presence is our
     * connected signal (bus-open != device-present). */
    if (now - g_last_poll_ms >= PIMORONI_SERVO2040_POLL_MS) {
        g_last_poll_ms = now;
        uint8_t who = 0;
        bool present = read_reg(PIMORONI_SERVO2040_REG_WHOAMI, &who, 1)
                       && who == PIMORONI_SERVO2040_WHOAMI_MAGIC;
        if (present && !g_board_present) {
            g_board_present = true;
            saint_log_publish("info", "Servo2040: board detected on I2C");
            g_homed = false;
        } else if (!present && g_board_present) {
            g_board_present = false;
            saint_log_publish("warn", "Servo2040: board lost on I2C");
            g_homed = false;
        }

        if (g_board_present) {
            uint8_t cur[2] = {0, 0};
            if (read_reg(PIMORONI_SERVO2040_REG_CURRENT_MA, cur, 2)) {
                uint16_t ma = (uint16_t)cur[0] | ((uint16_t)cur[1] << 8);
                g_current_amps = (float)ma / 1000.0f;
            }
            uint8_t st = 0;
            if (read_reg(PIMORONI_SERVO2040_REG_STATUS, &st, 1)) g_flags = st;
        }
    }

    if (g_board_present && !g_homed) {
        apply_connect_state();
        g_homed = true;
    }
    /* Live config re-provision (operator hit Sync while connected). */
    if (g_board_present && g_config_dirty) {
        provision_home();
    }

    /* Keepalive so the board's failsafe doesn't relax the servos. */
    if (g_board_present && now - g_last_ping_ms >= PIMORONI_SERVO2040_PING_MS) {
        g_last_ping_ms = now;
        (void)write_reg_u8(PIMORONI_SERVO2040_REG_HEARTBEAT, 1);
    }
}

bool pimoroni_servo2040_is_connected(void)
{
    return g_board_present;
}

float   pimoroni_servo2040_get_current_amps(void) { return g_current_amps; }
uint8_t pimoroni_servo2040_get_flags(void)        { return g_flags; }

/* ── peripheral_driver_t glue ────────────────────────────────────── */

static bool drv_init(void)
{
    pimoroni_servo2040_init();
    return true;
}

static bool drv_set_value(uint8_t channel, float value)
{
    if (channel < PIMORONI_SERVO2040_NUM_SERVOS) {
        uint16_t pulse = normalized_to_pulse(channel, value);
        return send_servo_target(channel, pulse);
    }
    /* LED channels: value carries a packed uint24 RGB (0xRRGGBB), same
     * convention as the NeoPixel color channel. */
    uint8_t idx = (uint8_t)(channel - PIMORONI_SERVO2040_LED_CHANNEL_BASE);
    if (idx >= PIMORONI_SERVO2040_NUM_LEDS) return false;
    uint32_t rgb = (uint32_t)value & 0xFFFFFFu;
    g_led_rgb[idx] = rgb;
    return send_led(idx, rgb);
}

static bool drv_get_value(uint8_t channel, float* value)
{
    /* Servos are output-only from the board; LED channels echo their
     * last-set color. Telemetry rides state_emit_channels, not here. */
    if (!value || channel < PIMORONI_SERVO2040_NUM_SERVOS) return false;
    uint8_t idx = (uint8_t)(channel - PIMORONI_SERVO2040_LED_CHANNEL_BASE);
    if (idx >= PIMORONI_SERVO2040_NUM_LEDS) return false;
    *value = (float)g_led_rgb[idx];
    return true;
}

static void drv_set_defaults(uint8_t channel, pin_config_t* config)
{
    (void)channel;
    if (!config) return;
    config->params.pimoroni_servo2040.start_us  = PIMORONI_SERVO2040_MIN_PULSE_US;
    config->params.pimoroni_servo2040.end_us    = PIMORONI_SERVO2040_MAX_PULSE_US;
    config->params.pimoroni_servo2040.center_us = PIMORONI_SERVO2040_MID_PULSE_US;
    config->params.pimoroni_servo2040.home_us   = 0;
}

static bool drv_apply_config(uint8_t channel, const pin_config_t* config)
{
    if (!config) return false;
    if (channel >= PIMORONI_SERVO2040_NUM_SERVOS) return true;  /* LED channels carry no extents */
    pimoroni_servo2040_channel_config_t nc = {
        .start_us  = config->params.pimoroni_servo2040.start_us,
        .end_us    = config->params.pimoroni_servo2040.end_us,
        .center_us = config->params.pimoroni_servo2040.center_us,
        .home_us   = config->params.pimoroni_servo2040.home_us,
    };
    /* Flag a board-flash home re-provision only when home actually
     * changed (avoid needless COMMIT / board flash wear). */
    if (nc.home_us != g_channel_configs[channel].home_us) g_config_dirty = true;
    g_channel_configs[channel] = nc;
    return true;
}

/* Extract an unsigned integer field value out of [start, end). */
static uint32_t parse_u32_field(const char* start, const char* end,
                                const char* key, uint32_t fallback)
{
    const char* p = strstr(start, key);
    if (!p || p >= end) return fallback;
    p = strchr(p, ':');
    if (!p || p >= end) return fallback;
    p++;
    while (p < end && (*p == ' ' || *p == '\t')) p++;
    if (p >= end || *p < '0' || *p > '9') return fallback;
    return (uint32_t)strtoul(p, NULL, 10);
}

static bool drv_parse_json(const char* json_start, const char* json_end,
                            pin_config_t* config)
{
    if (!json_start || !json_end || !config) return false;

    /* Capture peripheral id for telemetry labelling. */
    {
        const char* p = strstr(json_start, "\"id\"");
        if (p && p < json_end) {
            p = strchr(p, ':');
            if (p) {
                p++;
                while (*p == ' ' || *p == '\t') p++;
                if (*p == '"') {
                    p++;
                    const char* e = strchr(p, '"');
                    if (e && e < json_end) {
                        size_t n = (size_t)(e - p);
                        if (n >= sizeof(g_peripheral_id)) n = sizeof(g_peripheral_id) - 1;
                        memcpy(g_peripheral_id, p, n);
                        g_peripheral_id[n] = '\0';
                    }
                }
            }
        }
    }

    /* I2C SDA/SCL pins from the pins object (0 = platform default). */
    g_sda_pin = (uint8_t)parse_u32_field(json_start, json_end, "\"sda_pin\"", g_sda_pin);
    g_scl_pin = (uint8_t)parse_u32_field(json_start, json_end, "\"scl_pin\"", g_scl_pin);

    /* Global LED brightness (0..255). */
    g_led_brightness = (uint8_t)parse_u32_field(json_start, json_end,
                                                "\"led_brightness\"", g_led_brightness);

    /* Peripheral-level extent defaults. */
    uint16_t start_us  = PIMORONI_SERVO2040_MIN_PULSE_US;
    uint16_t end_us    = PIMORONI_SERVO2040_MAX_PULSE_US;
    uint16_t center_us = PIMORONI_SERVO2040_MID_PULSE_US;
    uint16_t home_us   = 0;
    start_us  = (uint16_t)parse_u32_field(json_start, json_end, "\"start_us\"",  start_us);
    end_us    = (uint16_t)parse_u32_field(json_start, json_end, "\"end_us\"",    end_us);
    center_us = (uint16_t)parse_u32_field(json_start, json_end, "\"center_us\"", center_us);
    home_us   = (uint16_t)parse_u32_field(json_start, json_end, "\"home_us\"",   home_us);

    /* Per-channel override from the "channels" array (same walk shape as
     * the Maestro driver). parse_json_params is called once per slab
     * channel with config->gpio = base+channel; recover the servo index.
     * LED channels (>= NUM_SERVOS) carry no extents. */
    if (config->gpio >= PIMORONI_SERVO2040_VIRTUAL_GPIO_BASE) {
        uint8_t ch = (uint8_t)(config->gpio - PIMORONI_SERVO2040_VIRTUAL_GPIO_BASE);
        if (ch < PIMORONI_SERVO2040_NUM_SERVOS) {
            const char* channels_key = strstr(json_start, "\"channels\"");
            if (channels_key && channels_key < json_end) {
                const char* arr_open = strchr(channels_key, '[');
                if (arr_open && arr_open < json_end) {
                    int idx = 0, depth = 0;
                    const char* obj_s = NULL;
                    const char* obj_e = NULL;
                    for (const char* q = arr_open + 1; q < json_end; q++) {
                        if (*q == '{') {
                            if (depth == 0 && idx == ch) obj_s = q;
                            depth++;
                        } else if (*q == '}') {
                            depth--;
                            if (depth == 0) {
                                if (idx == ch) { obj_e = q + 1; break; }
                                idx++;
                            }
                        } else if (*q == ']' && depth == 0) {
                            break;
                        }
                    }
                    if (obj_s && obj_e) {
                        start_us  = (uint16_t)parse_u32_field(obj_s, obj_e, "\"start_us\"",  start_us);
                        end_us    = (uint16_t)parse_u32_field(obj_s, obj_e, "\"end_us\"",    end_us);
                        center_us = (uint16_t)parse_u32_field(obj_s, obj_e, "\"center_us\"", center_us);
                        home_us   = (uint16_t)parse_u32_field(obj_s, obj_e, "\"home_us\"",   home_us);
                    }
                }
            }
        }
    }

    config->params.pimoroni_servo2040.start_us  = start_us;
    config->params.pimoroni_servo2040.end_us    = end_us;
    config->params.pimoroni_servo2040.center_us = center_us;
    config->params.pimoroni_servo2040.home_us   = home_us;

    /* Bind the transport now so a fresh sync starts pumping without a
     * reboot (storage is NULL here — pins fall back to defaults until the
     * next boot loads the just-saved pair). */
    (void)bind_transport(NULL);
    return true;
}

static void drv_estop(void)
{
    (void)write_reg_u8(PIMORONI_SERVO2040_REG_ESTOP, 1);
}

static bool drv_save(void* storage)
{
    flash_storage_data_t* s = (flash_storage_data_t*)storage;
    if (!s) return false;
    s->pimoroni_servo2040_config.channel_count = PIMORONI_SERVO2040_NUM_SERVOS;
    s->pimoroni_servo2040_config.led_brightness = g_led_brightness;
    for (uint8_t ch = 0; ch < PIMORONI_SERVO2040_NUM_SERVOS; ch++) {
        s->pimoroni_servo2040_config.channels[ch].start_us  = g_channel_configs[ch].start_us;
        s->pimoroni_servo2040_config.channels[ch].end_us    = g_channel_configs[ch].end_us;
        s->pimoroni_servo2040_config.channels[ch].center_us = g_channel_configs[ch].center_us;
        s->pimoroni_servo2040_config.channels[ch].home_us   = g_channel_configs[ch].home_us;
    }
    if (g_sda_pin) s->uart_pins.pimoroni_tx_pin = g_sda_pin;   /* SDA reuses tx slot */
    if (g_scl_pin) s->uart_pins.pimoroni_rx_pin = g_scl_pin;   /* SCL reuses rx slot */
    return true;
}

static bool drv_load(const void* storage)
{
    const flash_storage_data_t* s = (const flash_storage_data_t*)storage;
    if (!s) return false;
    ensure_state_init();

    const uint8_t saved_cc = s->pimoroni_servo2040_config.channel_count;
    if (saved_cc == 0 || saved_cc == 0xFF) return true;   /* nothing saved */

    uint8_t count = saved_cc;
    if (count > PIMORONI_SERVO2040_NUM_SERVOS) count = PIMORONI_SERVO2040_NUM_SERVOS;
    for (uint8_t ch = 0; ch < count; ch++) {
        pimoroni_servo2040_channel_config_t c = {
            .start_us  = s->pimoroni_servo2040_config.channels[ch].start_us,
            .end_us    = s->pimoroni_servo2040_config.channels[ch].end_us,
            .center_us = s->pimoroni_servo2040_config.channels[ch].center_us,
            .home_us   = s->pimoroni_servo2040_config.channels[ch].home_us,
        };
        if (c.start_us > 0 || c.end_us > 0) g_channel_configs[ch] = c;
    }
    g_led_brightness = s->pimoroni_servo2040_config.led_brightness;
    if (g_led_brightness == 0) g_led_brightness = 255;
    g_sda_pin = s->uart_pins.pimoroni_tx_pin;
    g_scl_pin = s->uart_pins.pimoroni_rx_pin;

    (void)bind_transport(s);
    return true;
}

/* Emit "connected", "current_a", "error_flags" for the dashboard's Live
 * card. connected reflects the WHOAMI poll (bus open != board present). */
static int drv_state_emit_channels(char* buf, size_t cap, bool* first)
{
    if (!g_initialized) return 0;
    bool connected = g_board_present;

    int total = 0, n;
    n = peripheral_state_append_channel(buf + total, cap - (size_t)total, first,
        g_peripheral_id, "connected", connected ? 1.0f : 0.0f);
    if (n < 0) return -1;
    total += n;
    n = peripheral_state_append_channel(buf + total, cap - (size_t)total, first,
        g_peripheral_id, "current_a", connected ? g_current_amps : 0.0f);
    if (n < 0) return -1;
    total += n;
    n = peripheral_state_append_channel(buf + total, cap - (size_t)total, first,
        g_peripheral_id, "error_flags", connected ? (float)g_flags : 0.0f);
    if (n < 0) return -1;
    total += n;
    return total;
}

static const peripheral_driver_t pimoroni_servo2040_peripheral = {
    .name              = "pimoroni_servo2040",
    .mode_string       = "pimoroni_servo",
    .pin_mode          = PIN_MODE_PIMORONI_SERVO,
    .capability_flag   = PIN_CAP_PIMORONI_SERVO,
    .virtual_gpio_base = PIMORONI_SERVO2040_VIRTUAL_GPIO_BASE,
    .channel_count          = PIMORONI_SERVO2040_CHANNELS_PER_INSTANCE,
    .channels_per_instance  = PIMORONI_SERVO2040_CHANNELS_PER_INSTANCE,
    .init              = drv_init,
    .update            = pimoroni_servo2040_update,
    .is_connected      = pimoroni_servo2040_is_connected,
    .set_value         = drv_set_value,
    .get_value         = drv_get_value,
    .set_defaults      = drv_set_defaults,
    .apply_config      = drv_apply_config,
    .parse_json_params = drv_parse_json,
    .estop             = drv_estop,
    .clear_estop       = NULL,
    .save_config       = drv_save,
    .load_config       = drv_load,
    .state_emit_channels = drv_state_emit_channels,
};

const peripheral_driver_t* pimoroni_servo2040_get_peripheral_driver(void)
{
    return &pimoroni_servo2040_peripheral;
}
