/**
 * SAINT.OS Firmware - FrSky FAS100 ADV Sensor Driver (shared core)
 *
 * Auto-detects S.Port (57600 baud, slow LED) vs FBUS (460800 baud,
 * fast LED) — operators can't pick a mode; the sensor decides at
 * power-on. The driver probes both protocols at startup and locks to
 * whichever responds. If a locked link goes silent, we drop back to
 * probing the OTHER protocol.
 *
 * The probe is deliberately conservative — only switch protocols when
 * NO BYTES at all arrive for FAS100_RX_QUIET_MS. A stream of CRC-
 * failing bytes means the sensor is alive and the parser just
 * desynced; that case is handled with a soft-resync (FIFO flush +
 * parser reset) WITHOUT leaving the locked protocol or flipping
 * is_connected. In field testing the old "no valid frame in 4 s →
 * switch protocol" rule cycled once every ~7 s — re-probing burned
 * ~1.5 s of FBUS-baud garbage on the wire which, on a shared half-
 * duplex line, could cause false responses from a sensor that was
 * actually in S.Port mode.
 *
 * Both protocols are inverted half-duplex UART:
 *   - S.Port uses 0x7E start-of-poll and 0x7D byte-stuffing on the response
 *   - FBUS has an explicit length byte and no stuffing; fixed 10-byte frames
 * Same sensor_id, data IDs, units, and CRC algorithm in both modes, so
 * the channel-value storage is shared between paths.
 *
 * Wiring (single bidirectional wire to the sensor):
 *
 *       MCU TX ──[ 1 kΩ ]──┬──── MCU RX
 *                          │
 *                          └──── FAS100 signal pin (S.Port / FBUS)
 *
 * Every byte we transmit echoes straight back into our own RX FIFO;
 * drain_echo_bytes() removes those echo bytes synchronously after each
 * TX so the parser only ever sees genuine sensor response data.
 *
 * Platform-specific UART setup (inverted signaling on the right pins,
 * baud switching during probe, byte read/write) lives behind the
 * fas100_transport_ops contract — see shared/include/fas100_transport.h.
 */

#include "fas100_driver.h"
#include "fas100_transport.h"

#include "fbus_protocol.h"
#include "flash_types.h"
#include "peripheral_driver.h"
#include "pin_types.h"
#include "platform.h"
#include "saint_log.h"
#include "sport_protocol.h"
#include "uart_pin_pairs.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ── Configuration ──────────────────────────────────────────────── */

#define FAS100_DEFAULT_TX_PIN   0
#define FAS100_DEFAULT_RX_PIN   1

static uint8_t fas100_tx_pin = FAS100_DEFAULT_TX_PIN;
static uint8_t fas100_rx_pin = FAS100_DEFAULT_RX_PIN;
static uint8_t fas100_uart_instance = 0;

/* Active pins are what the transport is currently bound to. Tracked so
 * fas100_init() can no-op when pins haven't changed (avoids briefly
 * dropping the wire mid-stream during runtime sync). */
static uint8_t fas100_active_tx_pin = 0xFF;
static uint8_t fas100_active_rx_pin = 0xFF;
static uint8_t fas100_active_uart   = 0xFF;

/* Response buffer: must hold the larger of an FBUS frame (10 bytes) or
 * a de-stuffed S.Port frame (8 bytes). Round up. */
#define FAS100_RX_BUF_SIZE   16

/* ── Protocol state ─────────────────────────────────────────────── */

typedef enum {
    PROTO_SPORT = 0,
    PROTO_FBUS  = 1,
} fas100_protocol_t;

typedef enum {
    PHASE_PROBE_SPORT,
    PHASE_PROBE_FBUS,
    PHASE_LOCKED,
} fas100_phase_t;

#define FAS100_PROBE_MS             1500
#define FAS100_LOST_LINK_MS         4000
#define FAS100_RX_QUIET_MS          8000
#define FAS100_STICKY_CONNECTED_MS  12000
#define FAS100_STATS_DUMP_MS        5000
#define FAS100_CONSEC_BAD_THRESH    16
#define FAS100_RAW_DUMP_MS          10000
#define FAS100_RAW_DUMP_SIZE        24

/* ── State (kept under original names — test_fas100_parser.c pokes
 *    them directly via reset_state() in the same translation unit) ─ */

static bool port_initialized = false;
static uint8_t poll_interval_ms = FAS100_DEFAULT_POLL_INTERVAL_MS;
static uint32_t last_poll_time = 0;
static bool sensor_responded = false;

static fas100_protocol_t active_proto = PROTO_SPORT;
static fas100_phase_t    phase            = PHASE_PROBE_SPORT;
static uint32_t          phase_started_ms = 0;
static uint32_t          last_response_ms = 0;
static uint32_t          last_byte_ms       = 0;
static uint32_t          last_soft_resync_ms = 0;
static uint32_t          stat_soft_resyncs   = 0;

static float current_amps = 0.0f;
static float voltage_volts = 0.0f;
static float temp1_celsius = 0.0f;
static float temp2_celsius = 0.0f;

static uint8_t rx_buf[FAS100_RX_BUF_SIZE];
static uint8_t rx_pos = 0;
static bool    rx_in_stuff = false;
static bool    sport_skip_addr = false;

static uint32_t stat_polls_sent     = 0;
static uint32_t stat_echo_bytes     = 0;
static uint32_t stat_echo_missed    = 0;
static uint32_t stat_frames_ok      = 0;
static uint32_t stat_frames_crc_bad = 0;
static uint32_t stat_resyncs        = 0;
static uint32_t stat_unknown_id     = 0;
static uint32_t stat_heartbeats     = 0;
static uint32_t stats_last_dump_ms  = 0;
static uint32_t stat_consecutive_bad = 0;

static uint8_t  raw_dump_buf[FAS100_RAW_DUMP_SIZE];
static uint8_t  raw_dump_pos        = 0;
static bool     raw_dump_armed      = false;
static uint32_t raw_dump_last_ms    = 0;

static const fas100_transport_ops_t* transport(void)
{
    return fas100_get_transport();
}

/* ── Internal helpers ───────────────────────────────────────────── */

/* Apply the baud rate that matches active_proto to the bound UART.
 * Also resets the receive accumulator and drains any stale bytes from
 * the RX FIFO so a half-decoded frame from the old rate doesn't poison
 * the parser at the new rate. */
static void apply_active_baud_rate(void)
{
    const fas100_transport_ops_t* t = transport();
    if (t && t->is_open()) {
        uint32_t baud = (active_proto == PROTO_FBUS) ? FBUS_BAUD_RATE
                                                     : SPORT_BAUD_RATE;
        if (t->set_baud) t->set_baud(baud);
        /* set_baud drains the RX FIFO internally; nothing to do here. */
    }
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
    stat_consecutive_bad = 0;
    stats_last_dump_ms = PLATFORM_MILLIS();
    raw_dump_armed = false;
    raw_dump_pos = 0;
    raw_dump_last_ms = 0;
}

/* Drain exactly `n` echo bytes from the RX FIFO with a per-byte
 * timeout. Called right after transport->flush() in send_*_poll(),
 * once we know the FIFO will hold our just-transmitted bytes. */
static void drain_echo_bytes(int n)
{
    const fas100_transport_ops_t* t = transport();
    if (!t || !t->is_open()) return;

    /* Two byte-times of slack per byte (~350 us at 57600, ~45 us at
     * 460800). Generous enough to absorb sampling jitter on the stop
     * bit; tight enough that a genuinely-missing echo doesn't stall
     * the poll loop for long. */
    uint32_t baud = (active_proto == PROTO_FBUS) ? FBUS_BAUD_RATE
                                                 : SPORT_BAUD_RATE;
    uint32_t per_byte_timeout_us = (20u * 1000000UL) / baud + 100u;
    for (int i = 0; i < n; i++) {
        uint32_t start_us = PLATFORM_MICROS();
        uint8_t byte;
        bool got = false;
        while ((PLATFORM_MICROS() - start_us) < per_byte_timeout_us) {
            if (t->read(&byte, 1) == 1) { got = true; break; }
        }
        if (got) {
            stat_echo_bytes++;
        } else {
            stat_echo_missed += (uint32_t)(n - i);
            break;
        }
    }
}

static void enter_phase(fas100_phase_t new_phase, fas100_protocol_t new_proto)
{
    if (new_phase != PHASE_LOCKED) sensor_responded = false;
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
    const fas100_transport_ops_t* t = transport();
    if (!t || !t->is_open()) return;

    uint8_t frame[FAS100_POLL_FRAME_SIZE] = {
        SPORT_POLL_HEADER,
        SPORT_FAS100_PHYSICAL_ID,
    };
    (void)t->write(frame, FAS100_POLL_FRAME_SIZE);
    if (t->flush) t->flush();
    drain_echo_bytes(FAS100_POLL_FRAME_SIZE);
    stat_polls_sent++;

    uint32_t now = PLATFORM_MILLIS();
    if (phase == PHASE_LOCKED
        && (raw_dump_last_ms == 0 || (now - raw_dump_last_ms) >= FAS100_RAW_DUMP_MS)
        && !raw_dump_armed) {
        raw_dump_pos = 0;
        raw_dump_armed = true;
    }
}

static void send_fbus_poll(void)
{
    const fas100_transport_ops_t* t = transport();
    if (!t || !t->is_open()) return;

    uint8_t frame[FBUS_POLL_FRAME_SIZE] = {
        FBUS_LEN_BYTE,
        FBUS_FAS100_SENSOR_ID,
        FBUS_FRAME_ID_DATA,
    };
    (void)t->write(frame, FBUS_POLL_FRAME_SIZE);
    if (t->flush) t->flush();
    drain_echo_bytes(FBUS_POLL_FRAME_SIZE);
    stat_polls_sent++;
}

static void send_poll_for_active_proto(void)
{
    if (active_proto == PROTO_FBUS) send_fbus_poll();
    else                            send_sport_poll();
}

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
    case SPORT_DATA_ID_CURRENT:  current_amps  = (float)value / 10.0f;        break;
    case SPORT_DATA_ID_VOLTAGE:  voltage_volts = (float)value / 100.0f;       break;
    case SPORT_DATA_ID_TEMP1:    temp1_celsius = (float)(int32_t)value;       break;
    case SPORT_DATA_ID_TEMP2:    temp2_celsius = (float)(int32_t)value;       break;
    default:                                                                  break;
    }
}

static bool destuff_byte(uint8_t raw, uint8_t* out, bool* in_stuff)
{
    if (*in_stuff) {
        *out = raw ^ SPORT_STUFF_MASK;
        *in_stuff = false;
        return true;
    }
    if (raw == SPORT_STUFF_MARKER) { *in_stuff = true; return false; }
    *out = raw;
    return true;
}

static void sport_feed_byte(uint8_t raw)
{
    if (raw == SPORT_POLL_HEADER) {
        if (rx_pos > 0 || rx_in_stuff) stat_resyncs++;
        rx_pos = 0;
        rx_in_stuff = false;
        sport_skip_addr = true;
        return;
    }
    if (sport_skip_addr) { sport_skip_addr = false; return; }

    uint8_t byte;
    if (!destuff_byte(raw, &byte, &rx_in_stuff)) return;

    if (rx_pos < FAS100_RX_BUF_SIZE) rx_buf[rx_pos++] = byte;
    if (rx_pos < FAS100_RESPONSE_FRAME_SIZE) return;

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
                    stat_consecutive_bad++;
                } else {
                    stat_consecutive_bad = 0;
                }
                store_telemetry(data_id, value);
            } else {
                stat_heartbeats++;
                sensor_responded = true;
                last_response_ms = PLATFORM_MILLIS();
            }
            rx_pos = 0;
            rx_in_stuff = false;
        } else {
            stat_frames_crc_bad++;
            stat_consecutive_bad++;
            memmove(rx_buf, rx_buf + 1, FAS100_RESPONSE_FRAME_SIZE - 1);
            rx_pos = FAS100_RESPONSE_FRAME_SIZE - 1;
        }
    } else {
        stat_frames_crc_bad++;
        stat_consecutive_bad++;
        memmove(rx_buf, rx_buf + 1, FAS100_RESPONSE_FRAME_SIZE - 1);
        rx_pos = FAS100_RESPONSE_FRAME_SIZE - 1;
    }
}

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
            rx_buf[0] = raw; rx_pos = 1;
        } else { rx_pos = 0; }
        return;
    }
    if (rx_pos == 2) {
        if (raw == FBUS_FRAME_ID_DATA) {
            rx_buf[rx_pos++] = raw;
        } else if (raw == FBUS_LEN_BYTE) {
            rx_buf[0] = raw; rx_pos = 1;
        } else { rx_pos = 0; }
        return;
    }
    if (rx_pos < FAS100_RX_BUF_SIZE) rx_buf[rx_pos++] = raw;
    if (rx_pos < FBUS_RESPONSE_FRAME_SIZE) return;

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
            stat_consecutive_bad++;
        } else {
            stat_consecutive_bad = 0;
        }
        store_telemetry(data_id, value);
    } else {
        stat_frames_crc_bad++;
        stat_consecutive_bad++;
    }
    rx_pos = 0;
}

/* ── Public API ─────────────────────────────────────────────────── */

void fas100_init(void)
{
    /* Validate the requested pin pair against the platform's UART pair
     * table. A silent fallback was a known footgun: an operator-picked
     * pair that doesn't map to anything on the platform used to switch
     * the driver to defaults (0/1) without telling anyone, and the
     * actual wiring stayed cold. Log loudly. */
    uint8_t req_tx = fas100_tx_pin;
    uint8_t req_rx = fas100_rx_pin;
    if (!uart_pin_pair_lookup(fas100_tx_pin, fas100_rx_pin, &fas100_uart_instance)) {
        saint_log_publish("warn",
            "FAS100: requested pair TX=%d RX=%d isn't a valid UART pair "
            "— falling back to TX=%d RX=%d (UART0). Check board YAML uart_pairs.",
            req_tx, req_rx, FAS100_DEFAULT_TX_PIN, FAS100_DEFAULT_RX_PIN);
        fas100_tx_pin = FAS100_DEFAULT_TX_PIN;
        fas100_rx_pin = FAS100_DEFAULT_RX_PIN;
        fas100_uart_instance = 0;
    }

    /* Idempotent: if pins haven't changed, no-op (avoids briefly
     * dropping the wire during runtime sync). */
    if (fas100_active_tx_pin == fas100_tx_pin &&
        fas100_active_rx_pin == fas100_rx_pin &&
        fas100_active_uart   == fas100_uart_instance) {
        port_initialized = true;
        return;
    }

    /* Open via transport with inverted signaling — RP2040 path uses
     * gpio_set_outover/inover INVERT + disable pulls; Teensy path uses
     * SERIAL_8N1_RXINV_TXINV. Probe always starts at S.Port baud; the
     * SM may switch to FBUS via apply_active_baud_rate() later. */
    active_proto = PROTO_SPORT;
    const fas100_transport_ops_t* t = transport();
    if (t) {
        if (!t->open(fas100_tx_pin, fas100_rx_pin, SPORT_BAUD_RATE, /*invert*/ true)) {
            saint_log_publish("error",
                "FAS100: %s transport open failed (tx=%d rx=%d)",
                t->name, fas100_tx_pin, fas100_rx_pin);
            return;
        }
    }

    fas100_active_tx_pin = fas100_tx_pin;
    fas100_active_rx_pin = fas100_rx_pin;
    fas100_active_uart   = fas100_uart_instance;

    rx_pos = 0;
    rx_in_stuff = false;
    sensor_responded = false;
    port_initialized = true;

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
    const fas100_transport_ops_t* t = transport();

    /* --- 1) Pump received bytes through the active protocol parser. --- */
    if (t && t->is_open()) {
        uint8_t buf[64];
        size_t n;
        while ((n = t->read(buf, sizeof(buf))) > 0) {
            for (size_t i = 0; i < n; i++) {
                uint8_t raw = buf[i];
                last_byte_ms = now;
                if (raw_dump_armed && raw_dump_pos < FAS100_RAW_DUMP_SIZE) {
                    raw_dump_buf[raw_dump_pos++] = raw;
                    if (raw_dump_pos >= FAS100_RAW_DUMP_SIZE) {
                        char hex[FAS100_RAW_DUMP_SIZE * 3 + 1];
                        int hn = 0;
                        for (uint8_t j = 0; j < FAS100_RAW_DUMP_SIZE; j++) {
                            hn += snprintf(hex + hn, sizeof(hex) - hn,
                                           "%02X ", raw_dump_buf[j]);
                        }
                        if (hn > 0 && hex[hn - 1] == ' ') hex[hn - 1] = '\0';
                        saint_log_publish("info",
                            "FAS100 raw RX (%s, %u bytes): %s",
                            active_proto == PROTO_FBUS ? "FBUS" : "S.Port",
                            (unsigned)FAS100_RAW_DUMP_SIZE, hex);
                        raw_dump_armed = false;
                        raw_dump_last_ms = now;
                    }
                }
                if (active_proto == PROTO_FBUS) fbus_feed_byte(raw);
                else                            sport_feed_byte(raw);
            }
        }
    }

    /* --- 2) Drive the auto-detect state machine. --- */
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
        now = PLATFORM_MILLIS();
        uint32_t valid_silence = (now >= last_response_ms)
            ? (now - last_response_ms) : 0;
        uint32_t byte_silence;
        if (last_byte_ms == 0)         byte_silence = valid_silence;
        else if (now >= last_byte_ms)  byte_silence = now - last_byte_ms;
        else                           byte_silence = 0;

        if (byte_silence >= FAS100_RX_QUIET_MS) {
            fas100_protocol_t other = (active_proto == PROTO_SPORT) ? PROTO_FBUS
                                                                    : PROTO_SPORT;
            fas100_phase_t target = (other == PROTO_SPORT) ? PHASE_PROBE_SPORT
                                                           : PHASE_PROBE_FBUS;
            saint_log_publish("warn",
                "FAS100: bus silent for %lu ms — re-probing %s",
                (unsigned long)byte_silence,
                other == PROTO_FBUS ? "FBUS" : "S.Port");
            enter_phase(target, other);
        } else if ((valid_silence >= FAS100_LOST_LINK_MS
                    && (now - last_soft_resync_ms) >= FAS100_LOST_LINK_MS)
                   || stat_consecutive_bad >= FAS100_CONSEC_BAD_THRESH) {
            bool fast = stat_consecutive_bad >= FAS100_CONSEC_BAD_THRESH;
            /* Drain whatever's buffered in the transport so we restart
             * parsing on a fresh byte boundary. */
            if (t && t->is_open()) {
                uint8_t drain[64];
                while (t->read(drain, sizeof(drain)) > 0) { /* spin */ }
            }
            rx_pos = 0;
            rx_in_stuff = false;
            sport_skip_addr = false;
            uint32_t bad_at_trigger = stat_consecutive_bad;
            stat_consecutive_bad = 0;
            last_soft_resync_ms = now;
            stat_soft_resyncs++;
            if (fast) {
                saint_log_publish("info",
                    "FAS100: fast soft-resync (%s, %lu consecutive bad frames)",
                    active_proto == PROTO_FBUS ? "FBUS" : "S.Port",
                    (unsigned long)bad_at_trigger);
            } else {
                saint_log_publish("info",
                    "FAS100: soft-resync (%s, no valid frame for %lu ms, "
                    "bus still active)",
                    active_proto == PROTO_FBUS ? "FBUS" : "S.Port",
                    (unsigned long)valid_silence);
            }
        }
    }

    /* --- 3) Send next poll if interval has elapsed. --- */
    if (now - last_poll_time >= poll_interval_ms) {
        last_poll_time = now;
        send_poll_for_active_proto();
    }

    /* --- 4) Periodic diagnostic stats dump while locked. --- */
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

void fas100_get_diag(fas100_diag_t* out)
{
    if (!out) return;
    out->phase            = (uint8_t)phase;
    out->proto            = (uint8_t)active_proto;
    out->connected        = fas100_is_connected();
    out->port_initialized = port_initialized;
    out->polls_sent       = stat_polls_sent;
    out->echo_bytes       = stat_echo_bytes;
    out->frames_ok        = stat_frames_ok;
    out->frames_crc_bad   = stat_frames_crc_bad;
    uint32_t now = PLATFORM_MILLIS();
    out->last_byte_ms_ago     = (last_byte_ms == 0)     ? 0xFFFFFFFFu : (now - last_byte_ms);
    out->last_response_ms_ago = (last_response_ms == 0) ? 0xFFFFFFFFu : (now - last_response_ms);
}

/* ── peripheral_driver_t glue ───────────────────────────────────── */

static bool fas100_drv_init(void)
{
    /* No-op (README item 3 pattern). Real init runs from drv_load
     * (flash had enabled=1) or drv_apply_config. */
    return true;
}

static bool fas100_drv_set_value(uint8_t channel, float value)
{
    (void)channel; (void)value;
    return false;  /* read-only sensor */
}

static bool fas100_drv_get_value(uint8_t channel, float* value)
{
    if (!value) return false;
    switch (channel) {
    case FAS100_CH_CURRENT: *value = current_amps;  return true;
    case FAS100_CH_VOLTAGE: *value = voltage_volts; return true;
    case FAS100_CH_TEMP1:   *value = temp1_celsius; return true;
    case FAS100_CH_TEMP2:   *value = temp2_celsius; return true;
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
    if (interval >= 20) poll_interval_ms = interval;
    fas100_init();
    return true;
}

static bool fas100_drv_parse_json(const char* json_start, const char* json_end,
                                   pin_config_t* config)
{
    bool got_poll = false;
    bool got_pins = false;

    const char* p = strstr(json_start, "\"poll_interval_ms\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) { p++; while (*p == ' ') p++;
                 config->params.fas100.poll_interval_ms = (uint8_t)atoi(p);
                 got_poll = true; }
    }

    uint8_t tx, rx, inst;
    if (uart_pin_pair_parse_json(json_start, json_end, &tx, &rx, &inst)) {
        fas100_tx_pin = tx;
        fas100_rx_pin = rx;
        fas100_uart_instance = inst;
        got_pins = true;
    }

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
    saint_log_publish("info",
        "FAS100: ESTOP (no action — read-only sensor)");
}

static bool fas100_drv_save(void* storage_ptr)
{
    flash_storage_data_t* storage = (flash_storage_data_t*)storage_ptr;
    memset(&storage->fas100_config, 0, sizeof(storage->fas100_config));
    storage->fas100_config.enabled         = port_initialized ? 1 : 0;
    storage->fas100_config.serial_port     = fas100_uart_instance;
    storage->fas100_config.poll_interval_ms = poll_interval_ms;

    storage->uart_pins.fas100_tx_pin = fas100_tx_pin;
    storage->uart_pins.fas100_rx_pin = fas100_rx_pin;
    return true;
}

static bool fas100_drv_load(const void* storage_ptr)
{
    const flash_storage_data_t* storage = (const flash_storage_data_t*)storage_ptr;

    uint8_t stored_tx = storage->uart_pins.fas100_tx_pin;
    uint8_t stored_rx = storage->uart_pins.fas100_rx_pin;
    if (stored_tx != 0 || stored_rx != 0) {
        uint8_t inst;
        if (uart_pin_pair_lookup(stored_tx, stored_rx, &inst)) {
            fas100_tx_pin = stored_tx;
            fas100_rx_pin = stored_rx;
            fas100_uart_instance = inst;
        } else {
            saint_log_publish("warn",
                "FAS100: invalid stored pin pair tx=%d rx=%d, using defaults",
                stored_tx, stored_rx);
        }
    }

    if (!storage->fas100_config.enabled) return true;

    if (storage->fas100_config.poll_interval_ms >= 20) {
        poll_interval_ms = storage->fas100_config.poll_interval_ms;
    }

    saint_log_publish("info",
        "FAS100: restored config from flash (UART%d TX=%d RX=%d, poll %dms)",
        fas100_uart_instance, fas100_tx_pin, fas100_rx_pin,
        poll_interval_ms);

    fas100_init();
    return true;
}

static const peripheral_driver_t fas100_peripheral = {
    .name              = "fas100",
    .mode_string       = "fas100_sensor",
    .pin_mode          = PIN_MODE_FAS100_SENSOR,
    .capability_flag   = PIN_CAP_FAS100_SENSOR,
    .virtual_gpio_base = FAS100_VIRTUAL_GPIO_BASE,
    .channel_count          = FAS100_CHANNEL_COUNT,
    .channels_per_instance  = FAS100_CHANNEL_COUNT,
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
