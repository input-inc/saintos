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
 * If the wire goes truly silent after locking we drop back to probing,
 * so a hot-unplug/replug of the sensor (which may come back in the
 * other mode) recovers without a reboot. The probe is deliberately
 * conservative — only switch protocols when no bytes have arrived at
 * all for FAS100_RX_QUIET_MS. A stream of CRC-failing bytes means the
 * sensor is alive and the parser just got out of sync; that case is
 * handled with a soft-resync (FIFO flush + parser reset) WITHOUT
 * leaving the locked protocol or flapping is_connected. Reason: in
 * the field we saw the old "no valid frame in 4 s → switch protocol"
 * rule cycle once every ~7 s — re-probing burns ~1.5 s of FBUS-baud
 * garbage on the wire which, on a shared half-duplex line, "could
 * cause false responses" from a sensor that's actually in S.Port mode.
 *
 * Both protocols are inverted half-duplex UART. Frame parsing differs:
 *   - S.Port uses 0x7E as start-of-poll and 0x7D byte-stuffing on the
 *     response payload.
 *   - FBUS has an explicit length byte and no stuffing; frames are a
 *     fixed 10 bytes [0x08, sensor_id, 0x10, data_id(2), value(4), crc].
 * The sensor_id, data IDs, units, and CRC algorithm are the same in
 * both protocols, so the channel-value storage is shared.
 *
 * Wiring (single bidirectional wire to the sensor):
 *
 *       MCU TX ──[ 1 kΩ ]──┬──── MCU RX
 *                          │
 *                          └──── FAS100 signal pin (S.Port / FBUS)
 *
 * Both protocols are half-duplex on one wire, so MCU TX is tied to MCU
 * RX through a 1 kΩ series resistor — that lets the sensor's
 * low-impedance driver win the bus during its response slot while still
 * letting our active-drive idle hold the line low (inverted-UART idle =
 * physical LOW). Every byte we transmit echoes straight back into our
 * own RX FIFO; drain_echo_bytes() below removes those echo bytes
 * synchronously after each TX so the parser only ever sees genuine
 * sensor response data. Do NOT enable the RP2040's internal pull-up on
 * the RX pin — it fights the inverted idle level AND is enough bias to
 * make the FAS100 decide at power-on that it's talking FBUS instead of
 * S.Port (see gpio_disable_pulls() in fas100_init()).
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
#include "pico/time.h"
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

// If we lose VALID frames for this long while locked, soft-resync the
// parser (flush UART FIFO + reset frame accumulator). Doesn't change
// protocol or flip is_connected — the sensor is presumed alive because
// bytes are still flowing, we just got desynced from some earlier
// corruption.
#define FAS100_LOST_LINK_MS     4000

// If we get NO BYTES AT ALL for this long while locked, the wire is
// truly dead — sensor unplugged, rebooted, or switched protocol modes.
// Drop back to probing the OTHER protocol. Set longer than
// FAS100_LOST_LINK_MS so a soft-resync gets a few cycles to try before
// we burn the bus on a protocol switch.
#define FAS100_RX_QUIET_MS      8000

// is_connected() keeps reporting `true` this long after the last valid
// frame even when sensor_responded has been cleared (e.g. during a
// transient re-probe). Prevents the dashboard from flapping
// connect/disconnect every time we lose then re-acquire the sensor.
// Must be ≥ FAS100_RX_QUIET_MS + FAS100_PROBE_MS so a worst-case
// silent-then-probe cycle is covered.
#define FAS100_STICKY_CONNECTED_MS  12000

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
// Updated on every byte the parser pumps from the UART FIFO (echo
// bytes are drained separately by drain_echo_bytes() and don't touch
// this). Used by the LOCKED-phase health check to distinguish "wire
// went silent" (→ switch protocols) from "bytes are flowing but
// CRC-failing" (→ soft-resync only).
static uint32_t          last_byte_ms       = 0;
// Last time we performed a soft-resync. Throttles repeated flushes so
// we don't churn the parser every iteration when CRC errors are
// persistent — one resync per FAS100_LOST_LINK_MS window is enough.
static uint32_t          last_soft_resync_ms = 0;
// Diagnostic count of soft-resyncs since the last stats dump.
static uint32_t          stat_soft_resyncs   = 0;

// Latest sensor values (shared between protocols — same data IDs/units)
static float current_amps = 0.0f;
static float voltage_volts = 0.0f;
static float temp1_celsius = 0.0f;
static float temp2_celsius = 0.0f;

// Receive buffer for accumulating response bytes
static uint8_t rx_buf[FAS100_RX_BUF_SIZE];
static uint8_t rx_pos = 0;
static bool    rx_in_stuff = false;  // S.Port byte-stuffing state

// S.Port hard-sync: when sport_feed_byte sees 0x7E (the unstuffed poll
// header — sensors byte-stuff any 0x7E values in their payload, so it
// never appears unescaped in a response), the parser resets and the
// NEXT byte is the physical-ID of the poll (which we discard). After
// that one byte, normal response accumulation resumes. Belt-and-
// suspenders alongside drain_echo_bytes: even if a tail byte from a
// late-arriving previous response leaks past the drain and desyncs the
// frame accumulator, the next poll cycle's 0x7E pulls us back in.
static bool    sport_skip_addr = false;

// Diagnostic counters. Dumped every FAS100_STATS_DUMP_MS while locked
// so we can see, in the dashboard Logs tab, what's actually happening
// on the bus — polls fired, valid frames, CRC failures, mid-frame
// resyncs. Reset on each phase transition so the numbers are scoped
// to the current lock attempt.
static uint32_t stat_polls_sent     = 0;
static uint32_t stat_echo_bytes     = 0;  // bytes drain_echo_bytes consumed
static uint32_t stat_echo_missed    = 0;  // bytes drain wanted but timed out on
static uint32_t stat_frames_ok      = 0;
static uint32_t stat_frames_crc_bad = 0;
static uint32_t stat_resyncs        = 0;  // 0x7E mid-frame
static uint32_t stat_unknown_id     = 0;  // CRC-valid but data_id not ours
static uint32_t stat_heartbeats     = 0;  // CRC-valid 0x00-header (no-data) frames
static uint32_t stats_last_dump_ms  = 0;
#define FAS100_STATS_DUMP_MS  5000

// Diagnostic raw-byte capture. The parser only reports pass/fail
// counts; this dumps the actual bytes between two consecutive polls
// so we can see (a) whether the sensor's first byte is 0x10 like the
// spec says or something else (e.g. a sensor-id echo), and (b)
// whether byte-stuffing is present in real frames. Armed by the next
// poll once every FAS100_RAW_DUMP_MS while locked; emits one log
// line and disarms.
#define FAS100_RAW_DUMP_MS    10000
#define FAS100_RAW_DUMP_SIZE  24
static uint8_t  raw_dump_buf[FAS100_RAW_DUMP_SIZE];
static uint8_t  raw_dump_pos        = 0;
static bool     raw_dump_armed      = false;
static uint32_t raw_dump_last_ms    = 0;

// We share a single wire for TX and RX (TX → 1 kΩ → RX, with the
// sensor's signal pin tied to RX directly). Every byte we transmit
// loops right back into our own RX FIFO. We drain that echo
// synchronously inside send_*_poll() — see drain_echo_bytes() — so
// fas100_update()'s read loop only ever sees genuine sensor response
// bytes. An earlier version of this driver tracked an asynchronous
// "echo to discard" counter, but when the main loop ran slow enough
// for two polls to fire between reads, the FIFO ended up holding
// [echo_N, response_N, echo_N+1, response_N+1, ...] and the counter
// happily skipped past response bytes thinking they were echo. The
// link would survive ~1 s before a single slow main-loop iteration
// permanently desynced the parser.

// =============================================================================
// Internal Helpers
// =============================================================================

/* Apply the baud rate that matches active_proto to the bound UART.
 * Called when init() runs or when the auto-detect state machine
 * switches between probe phases. Also resets the receive accumulator
 * and drains any stale bytes from the RX FIFO so a half-decoded frame
 * from the old baud rate doesn't poison the parser at the new rate. */
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
    sport_skip_addr = false;
}

static void reset_stats(void)
{
    stat_polls_sent = 0;
    stat_echo_bytes = 0;
    stat_echo_missed = 0;
    stat_frames_ok = 0;
    stat_frames_crc_bad = 0;
    stat_resyncs = 0;
    stat_unknown_id = 0;
    stat_heartbeats = 0;
    stat_soft_resyncs = 0;
    stats_last_dump_ms = PLATFORM_MILLIS();
    raw_dump_armed = false;
    raw_dump_pos = 0;
    raw_dump_last_ms = 0;
}

/* Drain exactly `n` echo bytes from the RX FIFO with a per-byte
 * timeout, so a missing/disconnected echo can't hang us forever.
 * Called immediately after uart_tx_wait_blocking() in send_*_poll(),
 * when we know the FIFO will hold our just-transmitted bytes. */
static void drain_echo_bytes(int n)
{
#ifndef SIMULATION
    if (!fas100_uart) return;
    uint32_t baud = (active_proto == PROTO_FBUS) ? FBUS_BAUD_RATE
                                                 : SPORT_BAUD_RATE;
    // Two byte-times of slack per byte (~350 us at 57600, ~45 us at
    // 460800). Generous enough to absorb sampling jitter on the
    // stop bit; tight enough that a genuinely-missing echo doesn't
    // stall the poll loop for long.
    uint32_t per_byte_timeout_us = (20 * 1000000UL) / baud + 100;
    for (int i = 0; i < n; i++) {
        absolute_time_t deadline = make_timeout_time_us(per_byte_timeout_us);
        while (!uart_is_readable(fas100_uart) && !time_reached(deadline)) {
            tight_loop_contents();
        }
        if (uart_is_readable(fas100_uart)) {
            (void)uart_getc(fas100_uart);
            stat_echo_bytes++;
        } else {
            // Echo byte didn't arrive in time. Stop draining — better
            // to let the parser hunt for a valid frame than to keep
            // consuming bytes that might be genuine response data.
            stat_echo_missed += (uint32_t)(n - i);
            break;
        }
    }
#endif
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
    reset_stats();

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
    uart_tx_wait_blocking(fas100_uart);
    drain_echo_bytes(FAS100_POLL_FRAME_SIZE);
    stat_polls_sent++;

    // Arm raw-byte capture once every FAS100_RAW_DUMP_MS so the Logs
    // tab gets a hex view of what's actually coming back between two
    // polls. Useful when the parser is reporting CRC failures and we
    // need to see whether the sensor's first byte is 0x10 (the spec)
    // or something else (e.g. a sensor-ID echo), and whether 0x7D
    // stuffing is present in the payload.
    uint32_t now = PLATFORM_MILLIS();
    if (phase == PHASE_LOCKED
        && (raw_dump_last_ms == 0 || (now - raw_dump_last_ms) >= FAS100_RAW_DUMP_MS)
        && !raw_dump_armed) {
        raw_dump_pos = 0;
        raw_dump_armed = true;
    }
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
    uart_tx_wait_blocking(fas100_uart);
    drain_echo_bytes(FBUS_POLL_FRAME_SIZE);
    stat_polls_sent++;
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
    // Hard-sync on the poll header. 0x7E only appears unescaped at the
    // start of a poll (sensors stuff 0x7E values in their payload), so
    // when we see one we know the parser is at a known position. The
    // BYTE immediately after 0x7E is the addressed sensor's physical
    // ID — we discard it and start collecting the response on the
    // byte after that.
    if (raw == SPORT_POLL_HEADER) {
        if (rx_pos > 0 || rx_in_stuff) stat_resyncs++;
        rx_pos = 0;
        rx_in_stuff = false;
        sport_skip_addr = true;
        return;
    }
    if (sport_skip_addr) {
        sport_skip_addr = false;
        return;
    }

    uint8_t byte;
    if (!destuff_byte(raw, &byte, &rx_in_stuff)) return;

    if (rx_pos < FAS100_RX_BUF_SIZE) {
        rx_buf[rx_pos++] = byte;
    }
    if (rx_pos < FAS100_RESPONSE_FRAME_SIZE) return;

    // Have a full frame's worth of de-stuffed bytes.
    if (rx_buf[0] == SPORT_DATA_HEADER || rx_buf[0] == SPORT_EMPTY_HEADER) {
        uint8_t crc = sport_crc_calculate(rx_buf, 7);
        if (rx_buf[7] == crc) {
            if (rx_buf[0] == SPORT_DATA_HEADER) {
                uint16_t data_id = (uint16_t)rx_buf[1] | ((uint16_t)rx_buf[2] << 8);
                uint32_t value   = (uint32_t)rx_buf[3]
                                 | ((uint32_t)rx_buf[4] << 8)
                                 | ((uint32_t)rx_buf[5] << 16)
                                 | ((uint32_t)rx_buf[6] << 24);
                stat_frames_ok++;
                if (data_id != SPORT_DATA_ID_CURRENT
                    && data_id != SPORT_DATA_ID_VOLTAGE
                    && data_id != SPORT_DATA_ID_TEMP1
                    && data_id != SPORT_DATA_ID_TEMP2) {
                    stat_unknown_id++;
                }
                store_telemetry(data_id, value);
            } else {
                // Empty-header frame: sensor is alive but had nothing
                // new for this poll. Don't update telemetry, but DO
                // refresh sensor_responded / last_response_ms so the
                // LOCKED-phase liveness check doesn't trip soft-resync.
                stat_heartbeats++;
                sensor_responded = true;
                last_response_ms = PLATFORM_MILLIS();
            }
        } else {
            stat_frames_crc_bad++;
        }
    } else {
        stat_frames_crc_bad++;
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
        stat_frames_ok++;
        if (data_id != SPORT_DATA_ID_CURRENT
            && data_id != SPORT_DATA_ID_VOLTAGE
            && data_id != SPORT_DATA_ID_TEMP1
            && data_id != SPORT_DATA_ID_TEMP2) {
            stat_unknown_id++;
        }
        store_telemetry(data_id, value);
    } else {
        stat_frames_crc_bad++;
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
    // Echo is already drained synchronously inside send_*_poll(), so
    // anything in the FIFO here is genuine sensor response data.
#ifndef SIMULATION
    while (fas100_uart && uart_is_readable(fas100_uart)) {
        uint8_t raw = uart_getc(fas100_uart);
        // Any byte the parser sees here is genuine sensor data — echo
        // was drained synchronously in send_*_poll(). Tracking the
        // timestamp lets the LOCKED-phase health check distinguish a
        // dead bus from a busy-but-CRC-failing one.
        last_byte_ms = now;
        // Tee into the raw-byte capture buffer if armed. The next
        // poll's send_*_poll arm cycle resets dump_pos; we just emit
        // once the buffer fills.
        if (raw_dump_armed && raw_dump_pos < FAS100_RAW_DUMP_SIZE) {
            raw_dump_buf[raw_dump_pos++] = raw;
            if (raw_dump_pos >= FAS100_RAW_DUMP_SIZE) {
                char hex[FAS100_RAW_DUMP_SIZE * 3 + 1];
                int n = 0;
                for (uint8_t i = 0; i < FAS100_RAW_DUMP_SIZE; i++) {
                    n += snprintf(hex + n, sizeof(hex) - n, "%02X ",
                                  raw_dump_buf[i]);
                }
                if (n > 0 && hex[n - 1] == ' ') hex[n - 1] = '\0';
                saint_log_publish("info",
                    "FAS100 raw RX (%s, %u bytes): %s",
                    active_proto == PROTO_FBUS ? "FBUS" : "S.Port",
                    (unsigned)FAS100_RAW_DUMP_SIZE, hex);
                raw_dump_armed = false;
                raw_dump_last_ms = now;
            }
        }
        if (active_proto == PROTO_FBUS) {
            fbus_feed_byte(raw);
        } else {
            sport_feed_byte(raw);
        }
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
    } else if (phase == PHASE_LOCKED && last_response_ms != 0) {
        // Refresh `now` here. The read loop above runs the parser,
        // which on a valid frame sets last_response_ms =
        // PLATFORM_MILLIS() — that PLATFORM_MILLIS() call can return a
        // tick AFTER the `now` we captured at the top of the function,
        // so `now - last_response_ms` would underflow (uint32 wraps to
        // UINT32_MAX) and trivially trip the >= LOST_LINK_MS check
        // every iteration. Re-reading now closes the window.
        now = PLATFORM_MILLIS();
        // Both silences are still clamped below to guard against any
        // remaining ordering surprise (e.g. last_byte_ms set in the
        // read loop using the stale now).
        uint32_t valid_silence = (now >= last_response_ms)
            ? (now - last_response_ms) : 0;
        // last_byte_ms==0 means we've literally never seen a byte since
        // entering this phase — treat that as worst-case silence so
        // the RX_QUIET_MS check fires correctly.
        uint32_t byte_silence;
        if (last_byte_ms == 0) {
            byte_silence = valid_silence;
        } else if (now >= last_byte_ms) {
            byte_silence = now - last_byte_ms;
        } else {
            byte_silence = 0;
        }
        if (byte_silence >= FAS100_RX_QUIET_MS) {
            // Bus is truly dead: no bytes at all for RX_QUIET_MS. Most
            // common cause is the sensor rebooted into the OTHER mode
            // (FBUS↔S.Port). Switch protocols and resume probing.
            fas100_protocol_t other = (active_proto == PROTO_SPORT) ? PROTO_FBUS
                                                                    : PROTO_SPORT;
            fas100_phase_t target = (other == PROTO_SPORT) ? PHASE_PROBE_SPORT
                                                           : PHASE_PROBE_FBUS;
            saint_log_publish("warn",
                "FAS100: bus silent for %lu ms — re-probing %s",
                (unsigned long)byte_silence,
                other == PROTO_FBUS ? "FBUS" : "S.Port");
            enter_phase(target, other);
        } else if (valid_silence >= FAS100_LOST_LINK_MS
                   && (now - last_soft_resync_ms) >= FAS100_LOST_LINK_MS) {
            // Bytes are arriving (link is alive) but none have CRC-
            // validated in a while — parser is desynced or the wire is
            // briefly noisy. Flush the UART FIFO and reset the frame
            // accumulator so we start fresh on the next 0x7E poll
            // echo (S.Port) or 0x08 header (FBUS). Stay in PHASE_LOCKED;
            // sensor_responded stays true so is_connected doesn't flap.
#ifndef SIMULATION
            if (fas100_uart) {
                while (uart_is_readable(fas100_uart)) (void)uart_getc(fas100_uart);
            }
#endif
            rx_pos = 0;
            rx_in_stuff = false;
            sport_skip_addr = false;
            last_soft_resync_ms = now;
            stat_soft_resyncs++;
            saint_log_publish("info",
                "FAS100: soft-resync (%s, no valid frame for %lu ms, "
                "bus still active)",
                active_proto == PROTO_FBUS ? "FBUS" : "S.Port",
                (unsigned long)valid_silence);
        }
    }

    // --- 3) Send next poll if interval has elapsed. ---
    if (now - last_poll_time >= poll_interval_ms) {
        last_poll_time = now;
        send_poll_for_active_proto();
    }

    // --- 4) Periodic diagnostic stats dump while locked. ---
    //
    // Surfaces what's actually happening on the bus: polls fired,
    // echo bytes drained vs missed, frames OK vs CRC-bad, mid-frame
    // resyncs (a healthy locked link should show roughly polls_sent
    // frames_ok per dump and near-zero crc_bad / resyncs / missed).
    if (phase == PHASE_LOCKED
        && (now - stats_last_dump_ms) >= FAS100_STATS_DUMP_MS) {
        saint_log_publish("info",
            "FAS100 stats (%s, %lu ms): polls=%lu echo_ok=%lu echo_miss=%lu "
            "frames_ok=%lu heartbeat=%lu crc_bad=%lu resync=%lu "
            "soft_resync=%lu unknown_id=%lu",
            active_proto == PROTO_FBUS ? "FBUS" : "S.Port",
            (unsigned long)(now - stats_last_dump_ms),
            (unsigned long)stat_polls_sent,
            (unsigned long)stat_echo_bytes,
            (unsigned long)stat_echo_missed,
            (unsigned long)stat_frames_ok,
            (unsigned long)stat_heartbeats,
            (unsigned long)stat_frames_crc_bad,
            (unsigned long)stat_resyncs,
            (unsigned long)stat_soft_resyncs,
            (unsigned long)stat_unknown_id);
        stat_polls_sent = 0;
        stat_echo_bytes = 0;
        stat_echo_missed = 0;
        stat_frames_ok = 0;
        stat_heartbeats = 0;
        stat_frames_crc_bad = 0;
        stat_resyncs = 0;
        stat_soft_resyncs = 0;
        stat_unknown_id = 0;
        stats_last_dump_ms = now;
    }
}

bool fas100_is_connected(void)
{
    if (!port_initialized) return false;
    if (sensor_responded)  return true;
    // Sticky: if we got a valid frame recently, keep reporting connected
    // through a brief re-probe so the dashboard doesn't flap. After
    // FAS100_STICKY_CONNECTED_MS of no valid frames we give up and
    // report disconnected — covers the actual sensor-unplugged case.
    if (last_response_ms != 0) {
        uint32_t age = PLATFORM_MILLIS() - last_response_ms;
        if (age < FAS100_STICKY_CONNECTED_MS) return true;
    }
    return false;
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
    .channel_count          = FAS100_CHANNEL_COUNT,
    .channels_per_instance  = FAS100_CHANNEL_COUNT,  /* One FAS100 owns all 4 channels */
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
