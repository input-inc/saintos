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

/* Set when a live config sync changes channel config (maestro_set_channel_config).
 * maestro_update() re-provisions the chip's EEPROM + REINITIALIZEs on the
 * next tick if the device is already connected, so a Sync from the
 * dashboard takes full effect (e.g. a widened MIN/MAX range) without the
 * operator power-cycling the Maestro. Cleared by the provisioning paths. */
static bool     g_config_dirty  = false;

static maestro_channel_config_t g_channel_configs[MAESTRO_MAX_CHANNELS];

/* Resolve a transport ops table for the given flash mode. Returns NULL
 * if the platform doesn't provide that transport — caller logs and
 * leaves the driver inert. */
static const maestro_transport_ops_t* pick_transport(uint8_t mode)
{
    switch (mode) {
        case FLASH_MAESTRO_TRANSPORT_USB_CDC:
            /* Was USB_HOST in older flash saves — same numeric value. */
            return maestro_get_transport_usb_cdc();
        case FLASH_MAESTRO_TRANSPORT_UART:
            return maestro_get_transport_uart();
        case FLASH_MAESTRO_TRANSPORT_USB_VENDOR:
            return maestro_get_transport_usb_vendor();
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
 * to power-cycle before the transport was ever bound.
 *
 * If the requested mode isn't implemented on this platform (e.g.
 * `usb_vendor` on the Teensy until task #17 lands), fall back to
 * usb_cdc → uart in order. Operator's saved preference stays
 * untouched so a future firmware that implements the requested mode
 * picks it up automatically. The fallback log fires only on state
 * change — drv_parse_json calls this once per channel and would
 * otherwise spam 24 log lines per config push. */
static bool bind_transport(uint8_t mode, const flash_storage_data_t* storage)
{
    const maestro_transport_ops_t* picked = pick_transport(mode);
    uint8_t effective_mode = mode;
    bool fell_back = false;

    if (!picked) {
        /* Fallback ladder: try the next-best transports this platform
         * actually provides. usb_cdc first (most natural mapping for a
         * Maestro on a USB host port); uart as a last resort. */
        static const uint8_t fallbacks[] = {
            FLASH_MAESTRO_TRANSPORT_USB_CDC,
            FLASH_MAESTRO_TRANSPORT_UART,
        };
        for (size_t i = 0; i < sizeof(fallbacks)/sizeof(fallbacks[0]); i++) {
            if (fallbacks[i] == mode) continue;       /* already failed */
            picked = pick_transport(fallbacks[i]);
            if (picked) {
                effective_mode = fallbacks[i];
                fell_back = true;
                break;
            }
        }
    }

    if (!picked) {
        /* Only log on transition into the "no transport" state —
         * subsequent calls (per-channel parse) shouldn't re-spam. */
        if (g_transport != NULL || g_transport_mode != mode) {
            saint_log_publish("error",
                "Maestro: no transport for mode=%u on this platform",
                (unsigned)mode);
        }
        g_transport = NULL;
        return false;
    }

    if (picked == g_transport) {
        return true;   /* already bound to this transport (incl. fallback) */
    }

    g_transport = picked;
    if (g_transport->open && !g_transport->open(storage)) {
        saint_log_publish("error",
            "Maestro: %s transport open failed",
            g_transport->name);
        g_transport = NULL;
        return false;
    }
    if (fell_back) {
        saint_log_publish("info",
            "Maestro: requested mode=%u unavailable on this platform; "
            "bound %s as fallback (saved preference preserved)",
            (unsigned)mode, g_transport->name);
    } else {
        saint_log_publish("info",
            "Maestro: bound %s transport", g_transport->name);
    }
    (void)effective_mode;
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
    /* Pick a sensible default transport for *this* platform:
     *   - USB vendor first if available (full feature set incl. EEPROM
     *     readback) — Teensy with usb_vendor implementation
     *   - else USB CDC if available — Teensy fallback
     *   - else UART — RP2040 and any UART-only platform
     * Without this a fresh RP2040 device would save transport_mode=0
     * (USB CDC) and then refuse to talk on next boot — get_transport
     * returns NULL on platforms that can't host USB. */
    if (maestro_get_transport_usb_vendor() != NULL) {
        g_transport_mode = FLASH_MAESTRO_TRANSPORT_USB_VENDOR;
    } else if (maestro_get_transport_usb_cdc() != NULL) {
        g_transport_mode = FLASH_MAESTRO_TRANSPORT_USB_CDC;
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

/* Forward declaration — definition is below maestro_set_channel_config
 * because it depends on g_channel_configs which is configured via
 * set_channel_config. The connect/reconnect hot-plug branches below
 * call this, so it needs a prototype here. */
static void maestro_apply_home_positions(void);

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
            /* Send each channel to its configured home position so a
             * freshly-connected (or re-connected) Maestro lands at
             * known positions instead of holding whatever it had
             * last. SaintOS-side home, not the Maestro EEPROM HOME —
             * see comment on maestro_apply_home_positions. */
            maestro_apply_home_positions();
        } else if (!now && g_was_connected) {
            saint_log_publish("warn",
                "Maestro: device disconnected (%s)", g_transport->name);
        }
        g_was_connected = now;
    } else if (g_transport->is_connected && g_transport->is_connected() && !g_was_connected) {
        /* UART transport: no hot-plug signal. Apply home once on the
         * first observed-connected tick after init. */
        maestro_apply_home_positions();
        g_was_connected = true;
    }

    /* Live config sync re-provision: a dashboard Sync that changed the
     * channel config (e.g. a wider MIN/MAX) updated g_channel_configs,
     * but the chip's EEPROM still holds the old values until we rewrite
     * them. When already connected (so the connect-hook provisioning
     * above won't fire), push the new params to EEPROM and REINITIALIZE
     * here so everything is set up the moment the operator hits Sync —
     * no power-cycle. Diff-checked + vendor-only, so it's a no-op on
     * CDC/UART or when nothing actually changed. */
    if (g_config_dirty && g_transport->ctrl_xfer
        && g_transport->is_connected && g_transport->is_connected()) {
        (void)maestro_provision_all_channels();
        g_config_dirty = false;
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

    /* Software-side range clamp: the Maestro firmware also enforces
     * the EEPROM MIN/MAX for the channel, but we keep our own
     * per-channel min_pulse_us / max_pulse_us as the source of truth.
     * Operators set these in the channel-edit modal; they may be
     * tighter than the Maestro's EEPROM range (e.g. to keep a servo
     * away from a mechanical end stop). Clamp here so a stale
     * animation input or a runaway operator slider can't drive the
     * channel past its safe range. quarter_us is in 0.25-us units;
     * configs are in µs, so multiply by 4 to compare. */
    const maestro_channel_config_t* cfg = &g_channel_configs[channel];
    uint16_t min_qus = (uint16_t)(cfg->min_pulse_us * 4u);
    uint16_t max_qus = (uint16_t)(cfg->max_pulse_us * 4u);
    if (min_qus > max_qus) { uint16_t t = min_qus; min_qus = max_qus; max_qus = t; }
    if (quarter_us < min_qus) quarter_us = min_qus;
    if (quarter_us > max_qus) quarter_us = max_qus;

    uint8_t cmd[4] = {
        0x84,
        channel,
        (uint8_t)(quarter_us & 0x7F),
        (uint8_t)((quarter_us >> 7) & 0x7F),
    };
    return tx_bytes(cmd, sizeof(cmd));
}

bool maestro_set_target_preview(uint8_t channel, uint16_t us)
{
    if (channel >= MAESTRO_MAX_CHANNELS) return false;

    /* Hard absolute safety clamp only — intentionally NOT the
     * per-channel min/max (see header). The dashboard sends pulses the
     * operator is dialing toward, which may be outside the saved
     * envelope. */
    if (us < MAESTRO_PREVIEW_MIN_US) us = MAESTRO_PREVIEW_MIN_US;
    if (us > MAESTRO_PREVIEW_MAX_US) us = MAESTRO_PREVIEW_MAX_US;

    uint16_t quarter_us = (uint16_t)(us * 4u);
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

bool maestro_stop_script(void)
{
    /* Compact Protocol Stop Script. Pololu Maestro User's Guide
     * documents this as a single 0xA4 byte over the active byte-
     * stream transport (CDC or UART). Stops execution of any user
     * script that might be auto-running and overriding our channel
     * targets — the symptom is the amber LED's 2-blink heartbeat
     * pattern, which Pololu documents as "device operational, script
     * running." We call this from the connect hook unconditionally
     * because we never WANT a Maestro-side script running (SaintOS
     * drives channels directly from the dashboard / animations). */
    uint8_t cmd = 0xA4;
    return tx_bytes(&cmd, 1);
}

/* ── Channel config ──────────────────────────────────────────────── */

void maestro_set_channel_config(uint8_t channel,
                                 const maestro_channel_config_t* config)
{
    if (channel >= MAESTRO_MAX_CHANNELS || !config) return;
    g_channel_configs[channel] = *config;

    /* Mark for EEPROM re-provision on the next update tick (live config
     * sync — see g_config_dirty). Only this live-apply path sets it;
     * drv_load (boot flash restore) assigns g_channel_configs directly,
     * so boot leaves provisioning to the connect hook. */
    g_config_dirty = true;

    /* Speed/accel are stored as the channel's *default* (used on Pose
     * transitions) — they're NOT pushed to the device on config save.
     * Animation/control input snaps to the target value at full speed,
     * so we want the device's runtime limits to stay at 0 (unlimited)
     * unless a Pose deliberately sets them.
     *
     * TODO(Pose-editor): when the Pose system is built, a Pose-play
     * path should call maestro_set_speed(channel, config->speed) and
     * maestro_set_acceleration(channel, config->acceleration) BEFORE
     * its maestro_set_target, then reset them to 0 after the transition
     * settles (or before the next animation input frame). See task #14
     * in docs/MAESTRO_BRINGUP.md. */
}

/* On connect:
 *   1. Probe the byte-stream path with Get Errors. If the Maestro
 *      answers (any value, even 0x0000), Compact Protocol commands
 *      are being parsed by the command interpreter — Serial Mode is
 *      correct (USB Dual Port or USB Chained). If we get no
 *      response, the bytes are going somewhere else (most commonly
 *      Serial Mode = UART, where CDC bytes route straight to the
 *      TTL pins instead of the interpreter). Log either way.
 *   2. Stop any auto-running Maestro-side user script (otherwise the
 *      script overrides our SET_TARGET commands and channels stay
 *      stuck at script-controlled positions — see
 *      maestro_stop_script docstring). The 2-blink amber Status LED
 *      is the symptom of this.
 *   3. Send each configured channel to its home_us position via Set
 *      Target so the freshly-connected Maestro lands at known
 *      positions instead of wherever it last was. home_us is the
 *      SaintOS-side home (not the Maestro EEPROM HomeMode/HOME) —
 *      the EEPROM HomeMode is still required to be `Goto` with a
 *      non-zero HOME for PWM to come up at all (see
 *      docs/MAESTRO_BRINGUP.md root cause). */
static void maestro_apply_home_positions(void)
{
    /* Probe: round-trip Get Errors. 0xFFFF is the maestro_get_errors
     * sentinel for "no bytes back within 10 ms" — that's the
     * Serial-Mode-is-UART symptom, not a real Maestro error register. */
    uint16_t probe = maestro_get_errors();
    if (probe == 0xFFFF) {
        saint_log_publish("warn",
            "Maestro: connect probe — no response to Get Errors. "
            "CDC bytes aren't reaching the command interpreter. "
            "Check Maestro Serial Mode = USB Dual Port (must NOT be UART).");
    } else {
        saint_log_publish("info",
            "Maestro: connect probe ok — Get Errors returned 0x%04x; "
            "Compact Protocol path is live.",
            (unsigned)probe);
    }

    saint_log_publish("info",
        "Maestro: sending Stop Script (Compact 0xA4) to clear any "
        "running user script that would override SET_TARGET");
    (void)maestro_stop_script();

    /* If the active transport can issue vendor parameter writes
     * (usb_vendor), bring each configured channel's EEPROM in line with
     * its SaintOS config — Mode = Servo, MIN/MAX, NEUTRAL, and
     * HomeMode = Goto — so the channel comes up enabled at its home
     * position on every power-up without any Maestro Control Center
     * step. Diff-checked, so steady-state connects do reads only and
     * don't wear the EEPROM. No-ops on CDC/UART (ctrl_xfer == NULL):
     * those carriers physically can't write parameters, so provisioning
     * there still requires MCC. */
    if (g_transport->ctrl_xfer) {
        (void)maestro_provision_all_channels();
    }
    /* Connect-hook provisioning just brought EEPROM in line with the
     * current config, so a pending live-sync re-provision would be
     * redundant — clear the flag. */
    g_config_dirty = false;

    uint8_t homed = 0;
    for (uint8_t ch = 0; ch < MAESTRO_MAX_CHANNELS; ch++) {
        uint16_t home_us = g_channel_configs[ch].home_us;
        if (home_us == 0) continue;  /* 0 = "unconfigured", skip */
        (void)maestro_set_target(ch, (uint16_t)(home_us * 4u));
        homed++;
    }
    saint_log_publish("info",
        "Maestro: applied home positions to %u channel%s",
        (unsigned)homed, homed == 1 ? "" : "s");
}

/* Pololu uscParameter IDs from
 * https://github.com/pololu/pololu-usb-sdk Maestro/Usc/Usc_protocol.cs.
 * SERVO0_HOME = 30; each channel adds 9 to the base param ID.
 *   offset 0..1 = HOME       (2 bytes, qus)
 *   offset 2    = MIN        (1 byte; stored ÷64, so multiply by 64 for qus)
 *   offset 3    = MAX        (1 byte; same scaling as MIN)
 *   offset 4..5 = NEUTRAL    (2 bytes, qus)
 *   offset 6    = RANGE      (1 byte; stored ÷127)
 *   offset 7    = SPEED      (1 byte exponential — see Pololu's
 *                             normalSpeedToExponentialSpeed; for EEPROM
 *                             readback we keep the raw byte and trust
 *                             the operator to interpret)
 *   offset 8    = ACCELERATION (1 byte) */
#define MAESTRO_PARAM_SERVO0_BASE   30
#define MAESTRO_PARAM_PER_CHANNEL    9

/* Per-servo-block offsets (added to the channel base). */
#define MAESTRO_PARAM_OFF_HOME       0   /* 2 bytes, qus (0=Off,1=Ignore,else Goto) */
#define MAESTRO_PARAM_OFF_MIN        2   /* 1 byte, stored ÷64 qus (= µs/16)        */
#define MAESTRO_PARAM_OFF_MAX        3   /* 1 byte, same scaling as MIN             */
#define MAESTRO_PARAM_OFF_NEUTRAL    4   /* 2 bytes, qus                            */

/* Channel modes are packed 2 bits/channel into CHANNEL_MODES_0_3 (12)
 * .. CHANNEL_MODES_20_23 (17): param = 12 + channel/4, shift =
 * (channel%4)*2 (Pololu Usc.cs: (byte[ch>>2] >> ((ch&3)<<1)) & 3).
 * ChannelMode.Servo == 0. */
#define MAESTRO_PARAM_CHANNEL_MODES_BASE  12
#define MAESTRO_CHANNEL_MODE_SERVO         0

/* Pololu vendor (EP0) request codes — protocol.h `enum uscRequest`. */
#define MAESTRO_REQ_GET_PARAMETER   0x81
#define MAESTRO_REQ_SET_PARAMETER   0x82
#define MAESTRO_REQ_REINITIALIZE    0x90

bool maestro_read_channel_config_from_device(uint8_t channel,
                                              maestro_channel_config_t* out)
{
    if (channel >= MAESTRO_MAX_CHANNELS || !out) return false;
    if (!g_transport || !g_transport->ctrl_xfer) {
        /* Transport doesn't support vendor control transfers (UART /
         * USB CDC). Caller falls back to RAM defaults. */
        return false;
    }
    if (!g_transport->is_connected || !g_transport->is_connected()) {
        return false;
    }

    const uint16_t base = MAESTRO_PARAM_SERVO0_BASE + channel * MAESTRO_PARAM_PER_CHANNEL;
    uint8_t buf[2];
    int n;

    /* HOME — 2-byte read at offset 0/1 */
    n = g_transport->ctrl_xfer(0xC0, 0x81, 0, base + 0, buf, 2, 500);
    if (n != 2) return false;
    uint16_t home_qus = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);

    /* MIN — 1-byte, stored ÷64 (so byte × 64 = qus, × 16 = µs) */
    n = g_transport->ctrl_xfer(0xC0, 0x81, 0, base + 2, buf, 1, 500);
    if (n != 1) return false;
    uint8_t min_byte = buf[0];

    /* MAX */
    n = g_transport->ctrl_xfer(0xC0, 0x81, 0, base + 3, buf, 1, 500);
    if (n != 1) return false;
    uint8_t max_byte = buf[0];

    /* NEUTRAL — 2 bytes at offset 4/5 */
    n = g_transport->ctrl_xfer(0xC0, 0x81, 0, base + 4, buf, 2, 500);
    if (n != 2) return false;
    uint16_t neutral_qus = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);

    /* SPEED — 1 byte at offset 7. EEPROM stores in Pololu exponential
     * format; we keep the raw value, callers convert as needed. */
    n = g_transport->ctrl_xfer(0xC0, 0x81, 0, base + 7, buf, 1, 500);
    if (n != 1) return false;
    uint8_t speed_byte = buf[0];

    /* ACCELERATION — 1 byte at offset 8 */
    n = g_transport->ctrl_xfer(0xC0, 0x81, 0, base + 8, buf, 1, 500);
    if (n != 1) return false;
    uint8_t accel_byte = buf[0];

    /* qus → µs: divide by 4. MIN/MAX bytes × 64 = qus → × 16 = µs. */
    out->home_us      = (uint16_t)(home_qus / 4);
    out->min_pulse_us = (uint16_t)(min_byte * 16);
    out->max_pulse_us = (uint16_t)(max_byte * 16);
    out->neutral_us   = (uint16_t)(neutral_qus / 4);
    out->speed        = (uint16_t)speed_byte;
    out->acceleration = (uint16_t)accel_byte;
    return true;
}

/* ── EEPROM provisioning via vendor SET_PARAMETER ─────────────────────
 *
 * Raw GET/SET of a single uscParameter. SET encodes the byte count in
 * the high byte of wIndex with the value in wValue and a zero-length
 * data stage — the exact wire format Pololu's MCC uses
 * (Usc.cs setRawParameterNoChecks). There is no separate "commit to
 * EEPROM" step in the Maestro protocol: SET_PARAMETER *is* the
 * persistence. */
static bool maestro_get_param(uint16_t param, uint8_t nbytes, uint16_t* out)
{
    if (!g_transport || !g_transport->ctrl_xfer || !out) return false;
    if (nbytes != 1 && nbytes != 2) return false;
    uint8_t buf[2] = { 0, 0 };
    int n = g_transport->ctrl_xfer(0xC0, MAESTRO_REQ_GET_PARAMETER,
                                   0, param, buf, nbytes, 500);
    if (n != (int)nbytes) return false;
    *out = (nbytes == 2) ? (uint16_t)(buf[0] | (buf[1] << 8))
                         : (uint16_t)buf[0];
    return true;
}

static bool maestro_set_param(uint16_t param, uint16_t value, uint8_t nbytes)
{
    if (!g_transport || !g_transport->ctrl_xfer) return false;
    uint16_t windex = (uint16_t)(((uint16_t)nbytes << 8) | param);
    /* Zero-length data stage: value rides in wValue, byte count in the
     * wIndex high byte (Usc.cs setRawParameterNoChecks). */
    int n = g_transport->ctrl_xfer(0x40, MAESTRO_REQ_SET_PARAMETER,
                                   value, windex, NULL, 0, 500);
    return n >= 0;
}

/* Write one parameter only if the chip's current value differs, so we
 * never burn an EEPROM write cycle re-storing a value that's already
 * correct. Counts the write into *writes. A failed read is treated as
 * "can't confirm" and skips the write (conservative — avoids blind
 * writes when the transport is flaky). */
static void maestro_provision_param(uint16_t param, uint16_t desired,
                                     uint8_t nbytes, int* writes)
{
    uint16_t cur;
    if (!maestro_get_param(param, nbytes, &cur)) return;
    if (cur == desired) return;
    if (maestro_set_param(param, desired, nbytes)) (*writes)++;
}

int maestro_provision_channel(uint8_t channel)
{
    if (channel >= MAESTRO_MAX_CHANNELS) return -1;
    if (!g_transport || !g_transport->ctrl_xfer) return -1;
    if (!g_transport->is_connected || !g_transport->is_connected()) return -1;

    const maestro_channel_config_t* cfg = &g_channel_configs[channel];
    const uint16_t base = MAESTRO_PARAM_SERVO0_BASE
                        + channel * MAESTRO_PARAM_PER_CHANNEL;
    int writes = 0;

    /* 1. Channel Mode = Servo. Modes are 2 bits/channel packed into a
     * shared byte, so read-modify-write to avoid disturbing the other
     * three channels in the group. Servo == 0 → clear the channel's
     * two bits. */
    {
        uint16_t mode_param = (uint16_t)(MAESTRO_PARAM_CHANNEL_MODES_BASE
                                         + (channel / 4));
        uint8_t  shift = (uint8_t)((channel % 4) * 2);
        uint16_t cur;
        if (maestro_get_param(mode_param, 1, &cur)) {
            uint16_t want = (uint16_t)(cur & ~(0x03u << shift))
                          | ((uint16_t)MAESTRO_CHANNEL_MODE_SERVO << shift);
            if (want != cur && maestro_set_param(mode_param, want, 1)) writes++;
        }
    }

    /* 2. MIN / MAX — 1 byte each, stored as qus÷64 (= µs/16). */
    uint16_t min_byte = (uint16_t)(cfg->min_pulse_us / 16);
    uint16_t max_byte = (uint16_t)(cfg->max_pulse_us / 16);
    if (min_byte > 255) min_byte = 255;
    if (max_byte > 255) max_byte = 255;
    maestro_provision_param(base + MAESTRO_PARAM_OFF_MIN, min_byte, 1, &writes);
    maestro_provision_param(base + MAESTRO_PARAM_OFF_MAX, max_byte, 1, &writes);

    /* 3. NEUTRAL — 2 bytes, quarter-µs. */
    maestro_provision_param(base + MAESTRO_PARAM_OFF_NEUTRAL,
                            (uint16_t)(cfg->neutral_us * 4), 2, &writes);

    /* 4. HOME = HomeMode.Goto at home_us (in qus). HOME doubles as the
     * mode selector: 0 = Off (channel disabled at startup — the exact
     * state that left us chasing "no PWM"), 1 = Ignore, anything else
     * = Goto <position>. Callers only provision channels with
     * home_us > 0, so this always lands as a Goto. */
    maestro_provision_param(base + MAESTRO_PARAM_OFF_HOME,
                            (uint16_t)(cfg->home_us * 4), 2, &writes);

    return writes;
}

int maestro_provision_all_channels(void)
{
    if (!g_transport || !g_transport->ctrl_xfer) return -1;
    if (!g_transport->is_connected || !g_transport->is_connected()) return -1;

    int total = 0;
    uint8_t channels = 0;
    for (uint8_t ch = 0; ch < MAESTRO_MAX_CHANNELS; ch++) {
        /* home_us == 0 means "unconfigured / don't auto-enable" — leave
         * that channel's EEPROM exactly as the operator left it (it may
         * be an Input/Output channel we must not clobber). */
        if (g_channel_configs[ch].home_us == 0) continue;
        int w = maestro_provision_channel(ch);
        if (w > 0) { total += w; channels++; }
    }

    if (total > 0) {
        /* Channel Mode and HOME are init parameters; REINITIALIZE makes
         * the freshly-written EEPROM take effect (and re-homes each
         * channel to its new Goto position) without a power cycle. */
        (void)g_transport->ctrl_xfer(0x40, MAESTRO_REQ_REINITIALIZE,
                                     0, 0, NULL, 0, 500);
        saint_log_publish("info",
            "Maestro: provisioned %u channel%s (%d EEPROM param%s written), "
            "REINITIALIZE sent",
            (unsigned)channels, channels == 1 ? "" : "s",
            total, total == 1 ? "" : "s");
    }
    return total;
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

    /* Per-channel override: walk the "channels" array (if present)
     * and find the entry matching THIS channel's index. Each channel
     * has its own min/max/neutral/home/speed/acceleration that
     * overrides the peripheral-level defaults parsed above.
     *
     * parse_json_params is called once per channel by
     * apply_one_peripheral with the same peripheral-object JSON range
     * but a different `config` (pin_config_t.gpio is set to
     * virtual_gpio_base + channel). We recover the channel index from
     * the gpio so we know which array entry to look at.
     *
     * Schema mirrors maestro_channel_config_t — see
     * server/saint_server/peripheral_model.py
     * _maestro_default_channel. */
    if (config->gpio >= MAESTRO_VIRTUAL_GPIO_BASE
        && config->gpio < (MAESTRO_VIRTUAL_GPIO_BASE + MAESTRO_MAX_CHANNELS)) {
        uint8_t channel = (uint8_t)(config->gpio - MAESTRO_VIRTUAL_GPIO_BASE);
        const char* channels_key = strstr(json_start, "\"channels\"");
        if (channels_key && channels_key < json_end) {
            const char* arr_open = strchr(channels_key, '[');
            if (arr_open && arr_open < json_end) {
                /* Scan for the channel-th sibling object inside this
                 * array. Track brace depth so nested objects don't
                 * inflate the index. depth==1 means we're directly
                 * inside the channels array; depth>1 means we're inside
                 * one of its entry objects. */
                int idx = 0;
                int depth = 0;
                const char* obj_s = NULL;
                const char* obj_e = NULL;
                for (const char* q = arr_open + 1; q < json_end; q++) {
                    if (*q == '{') {
                        if (depth == 0 && idx == channel) obj_s = q;
                        depth++;
                    } else if (*q == '}') {
                        depth--;
                        if (depth == 0) {
                            if (idx == channel) { obj_e = q + 1; break; }
                            idx++;
                        }
                    } else if (*q == ']' && depth == 0) {
                        break;  /* end of channels array */
                    }
                }
                if (obj_s && obj_e) {
                    /* Same lookups as the peripheral-level pass but
                     * scoped to this channel's object. Each found
                     * key wins over the peripheral default. */
                    #define MAESTRO_PARSE_CH_U16(key, into)                       \
                        do {                                                       \
                            const char* k = strstr(obj_s, "\"" key "\"");          \
                            if (k && k < obj_e) {                                  \
                                k = strchr(k, ':');                                \
                                if (k && k < obj_e) {                              \
                                    k++; while (*k == ' ') k++;                    \
                                    into = (uint16_t)atoi(k);                      \
                                }                                                  \
                            }                                                      \
                        } while (0)
                    MAESTRO_PARSE_CH_U16("min_pulse_us", min_p);
                    MAESTRO_PARSE_CH_U16("max_pulse_us", max_p);
                    MAESTRO_PARSE_CH_U16("neutral_us",   neut);
                    MAESTRO_PARSE_CH_U16("speed",        spd);
                    MAESTRO_PARSE_CH_U16("acceleration", acc);
                    MAESTRO_PARSE_CH_U16("home_us",      home);
                    #undef MAESTRO_PARSE_CH_U16
                }
            }
        }
    }

    /* Optional "transport":"uart"|"usb_cdc"|"usb_vendor"|"usb_host" —
     * lets the dashboard pick the mode at config time. Falls back to
     * whatever ensure_state_init defaulted to for this platform.
     * "usb_host" is the legacy string (= "usb_cdc"); bare "usb" also
     * maps to CDC for forward compat with older dashboards that
     * shipped only the umbrella name. Match longest prefix first so
     * "usb_vendor" doesn't get caught by the "usb" suffix. */
    p = strstr(json_start, "\"transport\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) {
            p++;
            while (*p == ' ' || *p == '"') p++;
            if (strncmp(p, "uart", 4) == 0) {
                g_transport_mode = FLASH_MAESTRO_TRANSPORT_UART;
            } else if (strncmp(p, "usb_vendor", 10) == 0) {
                g_transport_mode = FLASH_MAESTRO_TRANSPORT_USB_VENDOR;
            } else if (strncmp(p, "usb_cdc",  7) == 0 ||
                       strncmp(p, "usb_host", 8) == 0 ||
                       strncmp(p, "usb",      3) == 0) {
                g_transport_mode = FLASH_MAESTRO_TRANSPORT_USB_CDC;
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
     * [0, 2]; a saved record never has channel_count == 0 or 0xFF and
     * never has transport_mode > FLASH_MAESTRO_TRANSPORT_USB_VENDOR.
     * Reading any of those means the maestro_config bytes are either
     * zero-initialized (never written) or erased flash (0xFF) —
     * typical when the operator has no Maestro hardware on this
     * node. Bail before we try to bind a transport, which would
     * otherwise log a spurious "no transport for mode=255" error on
     * every boot. */
    const uint8_t saved_cc = s->maestro_config.channel_count;
    const uint8_t saved_tm = s->maestro_config.transport_mode;
    if (saved_cc == 0 || saved_cc == 0xFF
        || saved_tm > FLASH_MAESTRO_TRANSPORT_USB_VENDOR) {
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
