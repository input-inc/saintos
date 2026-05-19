/**
 * SAINT.OS Node Firmware - FrSky FAS100 ADV Sensor Driver (RP2040)
 *
 * The FAS100 ADV speaks two protocols and auto-selects one at power-on
 * based on what it sees on the bus: S.Port (57600 baud, slow LED blink)
 * or FBUS (460800 baud, fast LED blink). Operators can't pick a mode —
 * the sensor decides. So this driver supports BOTH and probes for the
 * active one at startup, alternating ~2 s between each baud rate until
 * one of them responds, then locking to that protocol.
 *
 * If responses stop for a while after locking we drop back to probing,
 * so a hot-unplug/replug of the sensor (which may come back in the
 * other mode) recovers without a reboot.
 *
 * Both protocols are inverted half-duplex UART. Frame parsing differs:
 *   - S.Port uses 0x7E as start-of-poll and 0x7D byte-stuffing on the
 *     response payload.
 *   - FBUS has an explicit length byte and no stuffing; frames are a
 *     fixed 10 bytes [0x08, sensor_id, 0x10, data_id(2), value(4), crc].
 * The sensor_id, data IDs, units, and CRC algorithm are the same in
 * both protocols, so the channel-value storage is shared.
 */

#include "fas100_driver.h"
#include "peripheral_driver.h"
#include "flash_types.h"
#include "platform.h"
#include "saint_log.h"
#include "uart_pin_pairs.h"
#include "fbus_protocol.h"

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

#define FAS100_DEFAULT_TX_PIN   0
#define FAS100_DEFAULT_RX_PIN   1

#ifndef SIMULATION
static uart_inst_t* fas100_uart = NULL;
#endif
static uint8_t fas100_tx_pin = FAS100_DEFAULT_TX_PIN;
static uint8_t fas100_rx_pin = FAS100_DEFAULT_RX_PIN;
static uint8_t fas100_uart_instance = 0;

// Active pins are what the UART hardware is currently bound to. If the
// operator picks a different pair (e.g. GPIO 28/29 instead of 0/1) we
// need to deinit the old pins before re-binding — otherwise the old
// pair keeps chattering at 57600 baud onto whatever's wired there.
static uint8_t fas100_active_tx_pin = 0xFF;     // 0xFF = "no UART bound yet"
static uint8_t fas100_active_rx_pin = 0xFF;
static uint8_t fas100_active_uart   = 0xFF;

// Response buffer: must hold the larger of an FBUS frame (10 bytes) or
// a de-stuffed S.Port frame (8 bytes). Round up.
#define FAS100_RX_BUF_SIZE   16

// =============================================================================
// Protocol state
// =============================================================================

typedef enum {
    PROTO_SPORT = 0,
    PROTO_FBUS  = 1,
} fas100_protocol_t;

typedef enum {
    PHASE_PROBE_SPORT,  // listening at 57600, sending S.Port polls
    PHASE_PROBE_FBUS,   // listening at 460800, sending FBUS polls
    PHASE_LOCKED,       // a protocol responded, stay with it
} fas100_phase_t;

// Time spent on each baud rate during probing before switching. The
// FAS100's response latency to a poll is well under 10 ms; 1500 ms is
// generous enough that intermittent wiring still gets at least a
// couple of poll attempts per probe cycle.
#define FAS100_PROBE_MS         1500

// If we lose responses for this long while locked, drop back to
// probing. Lets a hot-replug come up cleanly even if the new sensor
// powers up in the other protocol mode.
#define FAS100_LOST_LINK_MS     4000

// =============================================================================
// State
// =============================================================================

static bool port_initialized = false;
static uint8_t poll_interval_ms = FAS100_DEFAULT_POLL_INTERVAL_MS;
static uint32_t last_poll_time = 0;
static bool sensor_responded = false;

static fas100_protocol_t active_proto = PROTO_SPORT;
static fas100_phase_t    phase            = PHASE_PROBE_SPORT;
static uint32_t          phase_started_ms = 0;
static uint32_t          last_response_ms = 0;

// Latest sensor values (shared between protocols — same data IDs/units)
static float current_amps = 0.0f;
static float voltage_volts = 0.0f;
static float temp1_celsius = 0.0f;
static float temp2_celsius = 0.0f;

// Receive buffer for accumulating response bytes
static uint8_t rx_buf[FAS100_RX_BUF_SIZE];
static uint8_t rx_pos = 0;
static bool    rx_in_stuff = false;  // S.Port byte-stuffing state

// We share a single wire for TX and RX (TX → 1 kΩ → RX, with the
// sensor's signal pin tied to RX directly). Every byte we transmit
// loops right back into our own RX FIFO. If we feed that echo into
// the parser, rx_buf[0] ends up holding our poll's first byte (0x7E
// for S.Port, 0x08 for FBUS) instead of the sensor's response header,
// and the parser silently misaligns frame after frame.
//
// Track how many echo bytes to swallow next time we read. send_*_poll
// adds the poll length here, and the read loop in fas100_update drops
// that many bytes before handing anything to the parser.
static uint8_t echo_to_discard = 0;

// =============================================================================
// Internal Helpers
// =============================================================================

/* Apply the baud rate that matches active_proto to the bound UART.
 * Called when init() runs or when the auto-detect state machine
 * switches between probe phases. Also resets the receive accumulator
 * and drains any stale bytes from the RX FIFO so a half-decoded frame
 * (or in-flight echo) from the old baud rate doesn't poison the
 * parser at the new rate. */
static void apply_active_baud_rate(void)
{
#ifndef SIMULATION
    if (!fas100_uart) return;
    uint32_t baud = (active_proto == PROTO_FBUS) ? FBUS_BAUD_RATE
                                                 : SPORT_BAUD_RATE;
    uart_set_baudrate(fas100_uart, baud);
    while (uart_is_readable(fas100_uart)) (void)uart_getc(fas100_uart);
#endif
    rx_pos = 0;
    rx_in_stuff = false;
    echo_to_discard = 0;
}

/* Move to a new phase. Centralized so logging and rx-reset live in
 * one place — the state machine in fas100_update() just calls this. */
static void enter_phase(fas100_phase_t new_phase, fas100_protocol_t new_proto)
{
    if (new_phase != PHASE_LOCKED) {
        sensor_responded = false;
    }
    if (new_proto != active_proto) {
        active_proto = new_proto;
        apply_active_baud_rate();
    } else {
        rx_pos = 0;
        rx_in_stuff = false;
    }
    phase = new_phase;
    phase_started_ms = PLATFORM_MILLIS();

    if (new_phase == PHASE_PROBE_SPORT) {
        saint_log_publish("info",
            "FAS100: probing S.Port @ %d baud", SPORT_BAUD_RATE);
    } else if (new_phase == PHASE_PROBE_FBUS) {
        saint_log_publish("info",
            "FAS100: probing FBUS @ %d baud", FBUS_BAUD_RATE);
    }
}

static void send_sport_poll(void)
{
#ifndef SIMULATION
    if (!fas100_uart) return;
    uint8_t frame[FAS100_POLL_FRAME_SIZE] = {
        SPORT_POLL_HEADER,
        SPORT_FAS100_PHYSICAL_ID
    };
    uart_write_blocking(fas100_uart, frame, FAS100_POLL_FRAME_SIZE);
    echo_to_discard += FAS100_POLL_FRAME_SIZE;
#endif
}

static void send_fbus_poll(void)
{
#ifndef SIMULATION
    if (!fas100_uart) return;
    uint8_t frame[FBUS_POLL_FRAME_SIZE] = {
        FBUS_LEN_BYTE,
        FBUS_FAS100_SENSOR_ID,
        FBUS_FRAME_ID_DATA,
    };
    uart_write_blocking(fas100_uart, frame, FBUS_POLL_FRAME_SIZE);
    echo_to_discard += FBUS_POLL_FRAME_SIZE;
#endif
}

static void send_poll_for_active_proto(void)
{
    if (active_proto == PROTO_FBUS) {
        send_fbus_poll();
    } else {
        send_sport_poll();
    }
}

/* Update channel storage from a (data_id, raw value) pair. Both
 * S.Port and FBUS use the same IDs and scaling — only the framing
 * differs — so the dispatch table here is shared. */
static void store_telemetry(uint16_t data_id, uint32_t value)
{
    bool was_disconnected = !sensor_responded;
    sensor_responded = true;
    last_response_ms = PLATFORM_MILLIS();
    if (was_disconnected) {
        saint_log_publish("info",
            "FAS100: first %s response (data_id=0x%04x)",
            active_proto == PROTO_FBUS ? "FBUS" : "S.Port",
            data_id);
    }

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

/* S.Port byte-stuffing de-escape. Returns true if `raw` produced a
 * decoded byte in *out, false if it consumed an escape marker and
 * is waiting for the next raw byte. */
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

/* Feed one raw UART byte through the S.Port parser. Accumulates a
 * de-stuffed response frame and dispatches it once complete. */
static void sport_feed_byte(uint8_t raw)
{
    uint8_t byte;
    if (!destuff_byte(raw, &byte, &rx_in_stuff)) return;

    if (rx_pos < FAS100_RX_BUF_SIZE) {
        rx_buf[rx_pos++] = byte;
    }
    if (rx_pos < FAS100_RESPONSE_FRAME_SIZE) return;

    // Have a full frame's worth of de-stuffed bytes.
    if (rx_buf[0] == SPORT_DATA_HEADER) {
        uint8_t crc = sport_crc_calculate(rx_buf, 7);
        if (rx_buf[7] == crc) {
            uint16_t data_id = (uint16_t)rx_buf[1] | ((uint16_t)rx_buf[2] << 8);
            uint32_t value   = (uint32_t)rx_buf[3]
                             | ((uint32_t)rx_buf[4] << 8)
                             | ((uint32_t)rx_buf[5] << 16)
                             | ((uint32_t)rx_buf[6] << 24);
            store_telemetry(data_id, value);
        }
    }
    rx_pos = 0;
    rx_in_stuff = false;
}

/* FBUS parser. FBUS frames have no byte stuffing but DO have a fixed
 * 3-byte signature [0x08, sensor_id, 0x10] at the start. Walk through
 * the byte stream with a per-byte state machine: rx_pos tracks how
 * many of the expected bytes we've accumulated. If a header byte
 * doesn't match, drop back — but if the mismatched byte itself looks
 * like the start of a new frame (0x08), restart from there rather
 * than dropping it. */
static void fbus_feed_byte(uint8_t raw)
{
    if (rx_pos == 0) {
        if (raw == FBUS_LEN_BYTE) rx_buf[rx_pos++] = raw;
        return;
    }
    if (rx_pos == 1) {
        if (raw == FBUS_FAS100_SENSOR_ID) {
            rx_buf[rx_pos++] = raw;
        } else if (raw == FBUS_LEN_BYTE) {
            // mis-aligned: treat the mismatched byte as a new start
            rx_buf[0] = raw;
            rx_pos = 1;
        } else {
            rx_pos = 0;
        }
        return;
    }
    if (rx_pos == 2) {
        if (raw == FBUS_FRAME_ID_DATA) {
            rx_buf[rx_pos++] = raw;
        } else if (raw == FBUS_LEN_BYTE) {
            rx_buf[0] = raw;
            rx_pos = 1;
        } else {
            rx_pos = 0;
        }
        return;
    }
    // rx_pos in [3..9]: body bytes (data_id, value, crc)
    if (rx_pos < FAS100_RX_BUF_SIZE) {
        rx_buf[rx_pos++] = raw;
    }
    if (rx_pos < FBUS_RESPONSE_FRAME_SIZE) return;

    // Complete 10-byte FBUS frame. CRC covers the first 9 bytes
    // (len, sensor_id, frame_id, data_id, value).
    uint8_t crc = sport_crc_calculate(rx_buf, 9);
    if (rx_buf[9] == crc) {
        uint16_t data_id = (uint16_t)rx_buf[3] | ((uint16_t)rx_buf[4] << 8);
        uint32_t value   = (uint32_t)rx_buf[5]
                         | ((uint32_t)rx_buf[6] << 8)
                         | ((uint32_t)rx_buf[7] << 16)
                         | ((uint32_t)rx_buf[8] << 24);
        store_telemetry(data_id, value);
    }
    rx_pos = 0;
}

// =============================================================================
// Public API
// =============================================================================

void fas100_init(void)
{
#ifndef SIMULATION
    // Resolve UART instance from configured pin pair, with fallback to defaults.
    // The fallback used to be silent — if the operator-picked pair wasn't in
    // the rp2040_uart_pairs table, the driver would silently switch to 0/1
    // and the operator's wiring to (say) 28/29 would never be touched. Log
    // the fallback so it's not invisible.
    uint8_t req_tx = fas100_tx_pin;
    uint8_t req_rx = fas100_rx_pin;
    if (!uart_pin_pair_lookup(fas100_tx_pin, fas100_rx_pin, &fas100_uart_instance)) {
        saint_log_publish("warn",
            "FAS100: requested pair TX=%d RX=%d isn't a valid RP2040 UART pair "
            "— falling back to TX=%d RX=%d (UART0). Check board YAML uart_pairs.",
            req_tx, req_rx, FAS100_DEFAULT_TX_PIN, FAS100_DEFAULT_RX_PIN);
        fas100_tx_pin = FAS100_DEFAULT_TX_PIN;
        fas100_rx_pin = FAS100_DEFAULT_RX_PIN;
        fas100_uart_instance = 0;
    }

    // If the UART is already bound to these exact pins, nothing to do.
    // Re-binding the same pins would just toggle them briefly and lose
    // any S.Port byte mid-flight on the bus.
    if (fas100_active_tx_pin == fas100_tx_pin &&
        fas100_active_rx_pin == fas100_rx_pin &&
        fas100_active_uart   == fas100_uart_instance) {
        return;
    }

    // Detach the previously-bound pins so they stop driving the bus
    // when we move to a different pair (or to a different UART).
    if (fas100_active_tx_pin != 0xFF) {
        gpio_set_outover(fas100_active_tx_pin, GPIO_OVERRIDE_NORMAL);
        gpio_set_function(fas100_active_tx_pin, GPIO_FUNC_SIO);
        gpio_set_dir(fas100_active_tx_pin, GPIO_IN);
    }
    if (fas100_active_rx_pin != 0xFF) {
        gpio_set_inover(fas100_active_rx_pin, GPIO_OVERRIDE_NORMAL);
        gpio_set_function(fas100_active_rx_pin, GPIO_FUNC_SIO);
        gpio_set_dir(fas100_active_rx_pin, GPIO_IN);
    }
    if (fas100_active_uart != 0xFF && fas100_active_uart != fas100_uart_instance) {
        uart_deinit(fas100_active_uart == 0 ? uart0 : uart1);
    }

    fas100_uart = (fas100_uart_instance == 0) ? uart0 : uart1;

    // Start by probing S.Port (the user-reported "slow blink" mode);
    // if the sensor came up in FBUS the state machine will switch us
    // over after the probe window expires.
    active_proto = PROTO_SPORT;
    uart_init(fas100_uart, SPORT_BAUD_RATE);

    gpio_set_function(fas100_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(fas100_rx_pin, GPIO_FUNC_UART);

    // S.Port uses inverted signaling
    gpio_set_outover(fas100_tx_pin, GPIO_OVERRIDE_INVERT);
    gpio_set_inover(fas100_rx_pin, GPIO_OVERRIDE_INVERT);

    // No internal pull-up. With inversion enabled, the UART block's
    // high-idle becomes a physical LOW on the wire — i.e. the bus
    // idles low. An internal pull-up to 3.3 V would fight that idle
    // state, and worse, it appears to be enough to make the FAS100
    // ADV decide at power-on that an FBUS master is talking to it
    // (sensor LED switches from slow blink = S.Port to fast blink =
    // FBUS). Leaving the pad floating from the SDK's perspective and
    // relying on the MCU's active drive (via the 1 kΩ from TX to RX,
    // plus the sensor's own driver) keeps the idle state correct for
    // inverted S.Port.
    gpio_disable_pulls(fas100_tx_pin);
    gpio_disable_pulls(fas100_rx_pin);

    fas100_active_tx_pin = fas100_tx_pin;
    fas100_active_rx_pin = fas100_rx_pin;
    fas100_active_uart   = fas100_uart_instance;
#endif

    rx_pos = 0;
    rx_in_stuff = false;
    sensor_responded = false;
    port_initialized = true;

    // Boot the state machine into the S.Port probe phase. enter_phase()
    // logs the transition so the operator can see probing on the bus.
    phase = PHASE_PROBE_SPORT;
    phase_started_ms = PLATFORM_MILLIS();
    last_response_ms = 0;
    saint_log_publish("info",
        "FAS100: UART%d on TX=%d RX=%d, auto-detecting protocol "
        "(S.Port %d / FBUS %d, %d ms probe)",
        fas100_uart_instance, fas100_tx_pin, fas100_rx_pin,
        SPORT_BAUD_RATE, FBUS_BAUD_RATE, FAS100_PROBE_MS);
}

void fas100_update(void)
{
    if (!port_initialized) return;

    uint32_t now = PLATFORM_MILLIS();

    // --- 1) Pump received bytes through the active protocol parser. ---
    //
    // Drop echo bytes (our own outgoing polls looped back via the
    // shared TX/RX wire) before they reach the parser. echo_to_discard
    // is bumped by send_*_poll. Cap it defensively so a single weird
    // event (sensor not echoing back, RX wire loose) can't make us
    // permanently swallow incoming data.
#ifndef SIMULATION
    while (fas100_uart && uart_is_readable(fas100_uart)) {
        uint8_t raw = uart_getc(fas100_uart);
        if (echo_to_discard > 0) {
            echo_to_discard--;
            continue;
        }
        if (active_proto == PROTO_FBUS) {
            fbus_feed_byte(raw);
        } else {
            sport_feed_byte(raw);
        }
    }
    if (echo_to_discard > FBUS_POLL_FRAME_SIZE * 4) {
        // Almost certainly we're not seeing our own echo anymore (RX
        // wire problem?). Clear the counter so genuine incoming bytes
        // can flow again — the parser will just hunt for valid frames.
        echo_to_discard = 0;
    }
#endif

    // --- 2) Drive the auto-detect state machine. ---
    //
    // Probe phases hold for FAS100_PROBE_MS. If a valid frame arrives
    // during a probe (sensor_responded flips true via store_telemetry)
    // we jump straight to PHASE_LOCKED on whichever protocol just
    // answered. If the probe window expires without a response, we
    // alternate to the other protocol and probe again.
    //
    // Once locked, we monitor for lost responses; FAS100_LOST_LINK_MS
    // of silence drops us back into probing. The probe starts on the
    // OTHER protocol on the theory that the sensor likely rebooted
    // into a different mode (which is the failure case that motivated
    // the auto-detect in the first place).
    if (phase == PHASE_PROBE_SPORT && sensor_responded) {
        phase = PHASE_LOCKED;
        saint_log_publish("info",
            "FAS100: locked to S.Port @ %d baud", SPORT_BAUD_RATE);
    } else if (phase == PHASE_PROBE_FBUS && sensor_responded) {
        phase = PHASE_LOCKED;
        saint_log_publish("info",
            "FAS100: locked to FBUS @ %d baud", FBUS_BAUD_RATE);
    } else if (phase == PHASE_PROBE_SPORT
               && (now - phase_started_ms) >= FAS100_PROBE_MS) {
        enter_phase(PHASE_PROBE_FBUS, PROTO_FBUS);
    } else if (phase == PHASE_PROBE_FBUS
               && (now - phase_started_ms) >= FAS100_PROBE_MS) {
        enter_phase(PHASE_PROBE_SPORT, PROTO_SPORT);
    } else if (phase == PHASE_LOCKED
               && last_response_ms != 0
               && (now - last_response_ms) >= FAS100_LOST_LINK_MS) {
        // Switch to the OTHER protocol — most common cause of a lost
        // link is the sensor rebooted into a different mode.
        fas100_protocol_t other = (active_proto == PROTO_SPORT) ? PROTO_FBUS
                                                                : PROTO_SPORT;
        fas100_phase_t target = (other == PROTO_SPORT) ? PHASE_PROBE_SPORT
                                                       : PHASE_PROBE_FBUS;
        saint_log_publish("warn",
            "FAS100: no responses for %d ms — re-probing",
            FAS100_LOST_LINK_MS);
        enter_phase(target, other);
    }

    // --- 3) Send next poll if interval has elapsed. ---
    if (now - last_poll_time >= poll_interval_ms) {
        last_poll_time = now;
        send_poll_for_active_proto();
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
    // Intentionally a no-op: the peripheral driver registry calls this
    // for every driver at boot, but the FAS100 has no meaningful
    // default configuration — it doesn't know which UART pins to use
    // or whether the node is even adopted yet. Starting the state
    // machine here would dump auto-detect probe polls onto pins 0/1
    // before the operator has wired anything up.
    //
    // Polling is started by drv_load() (when a previously-saved
    // enabled config is restored from flash) or by drv_apply_config()
    // (when a fresh config arrives from the server). Until one of
    // those runs, fas100_update() bails on !port_initialized.
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

    // The operator may have picked a different UART pin pair (e.g.
    // GPIO 28/29 instead of the default 0/1). parse_json_params already
    // updated fas100_tx_pin / fas100_rx_pin / fas100_uart_instance from
    // the JSON. fas100_init() is idempotent — it no-ops when the active
    // pins already match — so call it here to bind the UART hardware
    // to the configured pair. Without this, the driver keeps reading
    // from whatever pins it was initialized on at boot (defaults).
    fas100_init();
    return true;
}

static bool fas100_drv_parse_json(const char* json_start, const char* json_end,
                                   pin_config_t* config)
{
    // Track which pieces of the config we found in the JSON so the
    // log line below tells the operator exactly what arrived. A silent
    // "we didn't find uart_tx/uart_rx" is one of the failure modes
    // that's hard to spot from the Logs tab otherwise.
    bool got_poll = false;
    bool got_pins = false;

    const char* p = strstr(json_start, "\"poll_interval_ms\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) {
            p++;
            while (*p == ' ') p++;
            config->params.fas100.poll_interval_ms = (uint8_t)atoi(p);
            got_poll = true;
        }
    }

    uint8_t tx, rx, inst;
    if (uart_pin_pair_parse_json(json_start, json_end, &tx, &rx, &inst)) {
        fas100_tx_pin = tx;
        fas100_rx_pin = rx;
        fas100_uart_instance = inst;
        got_pins = true;
    }

    // Per-channel parse_json fires 4 times for a FAS100 (one per
    // channel); the JSON contents are identical each call, so only
    // log on channel 0 to avoid 4 identical lines per sync.
    if (config->gpio == FAS100_VIRTUAL_GPIO_BASE + FAS100_CH_CURRENT) {
        if (got_pins) {
            saint_log_publish("info",
                "FAS100 sync: pins TX=%d RX=%d (UART%d), poll=%d ms",
                fas100_tx_pin, fas100_rx_pin, fas100_uart_instance,
                got_poll ? config->params.fas100.poll_interval_ms
                         : poll_interval_ms);
        } else {
            saint_log_publish("warn",
                "FAS100 sync: didn't find uart_tx/uart_rx in JSON — "
                "driver will keep using TX=%d RX=%d",
                fas100_tx_pin, fas100_rx_pin);
        }
    }
    return true;
}

static void fas100_drv_estop(void)
{
    // Read-only sensor — nothing to stop
    PLATFORM_PRINTF("FAS100: ESTOP (no action needed for read-only sensor)\n");
}

static bool fas100_drv_save(void* storage_ptr)
{
    flash_storage_data_t* storage = (flash_storage_data_t*)storage_ptr;

    memset(&storage->fas100_config, 0, sizeof(storage->fas100_config));
    storage->fas100_config.enabled = port_initialized ? 1 : 0;
    storage->fas100_config.serial_port = fas100_uart_instance;
    storage->fas100_config.poll_interval_ms = poll_interval_ms;

    storage->uart_pins.fas100_tx_pin = fas100_tx_pin;
    storage->uart_pins.fas100_rx_pin = fas100_rx_pin;

    return true;
}

static bool fas100_drv_load(const void* storage_ptr)
{
    const flash_storage_data_t* storage = (const flash_storage_data_t*)storage_ptr;

    // Pin assignment applies before init even if disabled, so init reads it.
    uint8_t stored_tx = storage->uart_pins.fas100_tx_pin;
    uint8_t stored_rx = storage->uart_pins.fas100_rx_pin;
    if (stored_tx != 0 || stored_rx != 0) {
        uint8_t inst;
        if (uart_pin_pair_lookup(stored_tx, stored_rx, &inst)) {
            fas100_tx_pin = stored_tx;
            fas100_rx_pin = stored_rx;
            fas100_uart_instance = inst;
        } else {
            PLATFORM_PRINTF("FAS100: invalid stored pin pair tx=%d rx=%d, using defaults\n",
                            stored_tx, stored_rx);
        }
    }

    if (!storage->fas100_config.enabled) return true;

    if (storage->fas100_config.poll_interval_ms >= 20) {
        poll_interval_ms = storage->fas100_config.poll_interval_ms;
    }

    PLATFORM_PRINTF("FAS100: restored config from flash (UART%d TX=%d RX=%d, poll %dms)\n",
                    fas100_uart_instance, fas100_tx_pin, fas100_rx_pin, poll_interval_ms);

    // The peripheral was enabled in the prior session and we have
    // valid stored pins. Auto-start polling so the operator doesn't
    // have to re-sync from the dashboard after every reboot.
    fas100_init();
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
    .save_config       = fas100_drv_save,
    .load_config       = fas100_drv_load,
};

const peripheral_driver_t* fas100_get_peripheral_driver(void)
{
    return &fas100_peripheral;
}
