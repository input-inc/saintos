/**
 * SAINT.OS Node Firmware - RoboClaw Solo 60A Motor Controller Driver (RP2040)
 *
 * Uses Pico SDK UART for CRC16 packet serial communication with up to 8
 * RoboClaw Solo 60A motor controllers on addresses 0x80-0x87.
 *
 * Virtual GPIO layout: 5 channels per unit (motor, encoder, voltage, current, temp).
 * Telemetry is polled round-robin to avoid blocking.
 */

#include "roboclaw_driver.h"
#include "peripheral_driver.h"
#include "flash_types.h"
#include "platform.h"
#include "saint_log.h"
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

#define ROBOCLAW_DEFAULT_TX_PIN     0
#define ROBOCLAW_DEFAULT_RX_PIN     1
#define ROBOCLAW_DEFAULT_SERIAL_PORT 0

// How many consecutive missed polls before we flip a unit's connected
// flag back to false. One miss can happen for any number of harmless
// reasons (bus contention, a single dropped byte) — we don't want a
// transient blip to drop the unit from the dashboard. Three in a row
// is a controller that genuinely went away.
#define ROBOCLAW_DROP_AFTER_MISSES  3

// Re-probe a disconnected unit at most this often. The probe is a
// GETVERSION packet (4 bytes out, 50 ms timeout in) — cheap, but if
// every disconnected unit re-probed on every update() tick we'd spend
// most of the bus on probes nobody asked for. Per-unit pacing keeps
// the bus quiet while still letting a freshly-plugged controller
// recover within a couple of seconds.
#define ROBOCLAW_REPROBE_INTERVAL_MS 2000

// Pins requested by the operator (set via parse_json_params from the
// peripheral config sync). Don't read these as "the UART is bound
// here" — see active_tx_pin/active_rx_pin/active_uart below for that.
static uint8_t roboclaw_tx_pin = ROBOCLAW_DEFAULT_TX_PIN;
static uint8_t roboclaw_rx_pin = ROBOCLAW_DEFAULT_RX_PIN;

// What the UART hardware is currently bound to. 0xFF means "no UART
// bound yet". If a fresh config comes in with different pins, we
// deinit the old pair before re-binding — otherwise the previous
// pins keep driving traffic onto whatever's wired there.
static uint8_t active_tx_pin = 0xFF;
static uint8_t active_rx_pin = 0xFF;
static uint8_t active_uart   = 0xFF;

// =============================================================================
// Per-Unit State
// =============================================================================

typedef struct {
    uint8_t  address;
    uint8_t  deadband;
    uint16_t max_current_ma;
    int16_t  duty;
    int32_t  encoder;
    uint16_t voltage_mv;
    int16_t  current_ma;
    int16_t  temp_tenths;
    bool     connected;
    // Connection-state tracking. last_response_ms is the timestamp of
    // the most recent successful read; consecutive_misses counts
    // failed polls since the last good response (resets on success).
    // last_reprobe_ms paces re-probing of disconnected units so the
    // bus doesn't fill with retries when nothing's there. Set to 0
    // at boot and after a re-bind so the first probe fires immediately.
    uint32_t last_response_ms;
    uint8_t  consecutive_misses;
    uint32_t last_reprobe_ms;
    // Per-unit E-stop output pin. 0 (or any value outside 1..29) = no
    // pin assigned — the PCB doesn't route an E-stop wire from this
    // MCU to this controller, OR the operator hasn't filled in the
    // param yet. We use 0 as the unset sentinel (rather than 0xFF) so
    // a zero-initialized struct and a never-written flash slot both
    // map to "no E-stop wiring" without a separate validity flag.
    // GPIO 0 is the default UART0 TX on this platform and is never
    // a valid choice for an E-stop output, so this overload is safe.
    //
    // When set to a valid GPIO (1..29), the driver configures that
    // pin as a digital output and drives it LOW (deasserted) the
    // moment the unit's config is applied or loaded from flash, so
    // a floating-pin glitch can't trip the RoboClaw's latching S3
    // input before any motor command goes out. Polarity is fixed
    // (LOW = deasserted, HIGH = tripped) — see pin_types.h for why.
    uint8_t  estop_pin;
} roboclaw_unit_t;

// =============================================================================
// Driver State
// =============================================================================

static roboclaw_unit_t units[ROBOCLAW_MAX_UNITS];
static uint8_t unit_count = 0;
static bool port_initialized = false;
static uint16_t configured_baud = ROBOCLAW_DEFAULT_BAUD;
static uint8_t configured_serial_port = ROBOCLAW_DEFAULT_SERIAL_PORT;

// Round-robin telemetry polling state
static uint8_t poll_unit = 0;
static uint8_t poll_register = 0;  // 0=encoder, 1=voltage, 2=current, 3=temp

#ifndef SIMULATION
static uart_inst_t* hw_uart = NULL;
#endif

// =============================================================================
// Internal Helpers
// =============================================================================

#ifndef SIMULATION
static void send_command(uint8_t address, uint8_t command,
                         const uint8_t* data, uint8_t data_len)
{
    if (!hw_uart) return;

    uint8_t packet[64];
    uint8_t idx = 0;

    packet[idx++] = address;
    packet[idx++] = command;
    for (uint8_t i = 0; i < data_len; i++) {
        packet[idx++] = data[i];
    }

    uint16_t crc = roboclaw_crc16_calculate(packet, idx);
    packet[idx++] = (uint8_t)(crc >> 8);
    packet[idx++] = (uint8_t)(crc & 0xFF);

    uart_write_blocking(hw_uart, packet, idx);
}

static bool read_response(uint8_t* buffer, uint8_t expected_len,
                           uint8_t address, uint8_t command)
{
    if (!hw_uart) return false;

    // Total bytes: expected_len data + 2 CRC bytes
    uint8_t total = expected_len + 2;
    uint32_t start = PLATFORM_MILLIS();

    for (uint8_t i = 0; i < total; i++) {
        while (!uart_is_readable(hw_uart)) {
            if (PLATFORM_MILLIS() - start > ROBOCLAW_RESPONSE_TIMEOUT_MS) {
                return false;
            }
        }
        buffer[i] = uart_getc(hw_uart);
        start = PLATFORM_MILLIS();  // Reset per-byte timeout
    }

    // Verify CRC: CRC covers address + command + data bytes
    uint16_t crc = 0;
    crc = roboclaw_crc16_update(crc, address);
    crc = roboclaw_crc16_update(crc, command);
    for (uint8_t i = 0; i < expected_len; i++) {
        crc = roboclaw_crc16_update(crc, buffer[i]);
    }

    uint16_t received_crc = ((uint16_t)buffer[expected_len] << 8) |
                             buffer[expected_len + 1];
    return crc == received_crc;
}

static bool read_ack(void)
{
    if (!hw_uart) return false;

    uint32_t start = PLATFORM_MILLIS();
    while (!uart_is_readable(hw_uart)) {
        if (PLATFORM_MILLIS() - start > ROBOCLAW_RESPONSE_TIMEOUT_MS) {
            return false;
        }
    }
    return uart_getc(hw_uart) == ROBOCLAW_ACK_BYTE;
}
#endif

// =============================================================================
// Probe / connection tracking
// =============================================================================

#ifndef SIMULATION
/* Issue GETVERSION on `unit_idx` and read the null-terminated version
 * string + 2 CRC bytes. Returns true on a valid response. Used both
 * from the initial probe pass (after a UART re-bind) and from
 * roboclaw_update() to opportunistically re-probe disconnected units.
 * Doesn't itself touch connected/last_response_ms — callers do that
 * so the same logic applies whether the success came from probe or
 * telemetry. */
static bool probe_unit(uint8_t unit_idx, char* version_out, size_t version_out_size)
{
    if (!hw_uart || unit_idx >= ROBOCLAW_MAX_UNITS) return false;

    // Drain any stale bytes from a previous failed exchange so the
    // version parser doesn't latch onto leftover garbage.
    while (uart_is_readable(hw_uart)) (void)uart_getc(hw_uart);

    send_command(units[unit_idx].address, ROBOCLAW_CMD_GETVERSION, NULL, 0);

    uint8_t resp[ROBOCLAW_VERSION_MAX_LEN + 2];
    uint32_t start = PLATFORM_MILLIS();
    uint8_t len = 0;
    bool got_null = false;
    while (len < ROBOCLAW_VERSION_MAX_LEN) {
        while (!uart_is_readable(hw_uart)) {
            if (PLATFORM_MILLIS() - start > ROBOCLAW_RESPONSE_TIMEOUT_MS) {
                return false;
            }
        }
        resp[len] = uart_getc(hw_uart);
        if (resp[len] == 0) {
            got_null = true;
            len++;
            break;
        }
        len++;
        start = PLATFORM_MILLIS();
    }
    if (!got_null) return false;

    // Slurp the 2 CRC bytes that follow the null terminator.
    for (uint8_t j = 0; j < 2; j++) {
        while (!uart_is_readable(hw_uart)) {
            if (PLATFORM_MILLIS() - start > ROBOCLAW_BYTE_TIMEOUT_MS) {
                return false;
            }
        }
        resp[len++] = uart_getc(hw_uart);
        start = PLATFORM_MILLIS();
    }

    // Copy the version string out for the log line. resp[0..len-3] is
    // the null-terminated version; truncate safely into the caller's
    // buffer regardless of its size.
    if (version_out && version_out_size > 0) {
        size_t copy_len = (len >= 2) ? (size_t)(len - 2) : 0;
        if (copy_len >= version_out_size) copy_len = version_out_size - 1;
        memcpy(version_out, resp, copy_len);
        version_out[copy_len] = '\0';
    }
    return true;
}
#endif

/* Configure the unit's E-stop output pin (if assigned) as a digital
 * output driven LOW. Called from drv_apply_config and drv_load so
 * the deassert happens as soon as we know which pin the PCB wired
 * to this controller's S3 input. Without this, the GPIO floats at
 * reset (RP2040 datasheet § 2.19.6.1: no pull, input mode) and a
 * single glitch high is enough to latch the RoboClaw's S3 ("Default
 * → E-Stop (Latching)" in Packet Serial mode) into emergency-stop
 * — at which point the controller still validates packets and ACKs
 * 0xFF but silently drops every motor command, with no path back
 * except a power-cycle.
 *
 * Safe to call any number of times — gpio_init/gpio_set_dir/gpio_put
 * are idempotent. If the operator changes which pin is assigned in
 * a fresh sync, the OLD pin remains driven LOW (no observable
 * effect) and the NEW pin starts being driven LOW too. */
static void apply_estop_pin(uint8_t unit_idx)
{
#ifndef SIMULATION
    if (unit_idx >= ROBOCLAW_MAX_UNITS) return;
    uint8_t pin = units[unit_idx].estop_pin;
    // 0 = unset sentinel (GPIO 0 is UART0 TX and never a valid
    // E-stop wire on this platform). Anything outside 1..29 also
    // skipped — RP2040 has 30 GPIOs (0..29).
    if (pin == 0 || pin > 29) return;

    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);   // LOW = deasserted = motor enabled
    saint_log_publish("info",
        "RoboClaw: unit %u E-stop pin GPIO %u driven LOW (deasserted) "
        "— S3 should now be releasable on the controller",
        (unsigned)unit_idx, (unsigned)pin);
#else
    (void)unit_idx;
#endif
}

/* Update a unit's connected state after a poll attempt, logging
 * connection transitions exactly once. Called from probe paths and
 * from the telemetry round-robin so all link-state transitions show
 * up in the Logs tab. */
static void mark_unit_response(uint8_t unit_idx, bool success)
{
    if (unit_idx >= ROBOCLAW_MAX_UNITS) return;
    roboclaw_unit_t* u = &units[unit_idx];

    if (success) {
        u->last_response_ms = PLATFORM_MILLIS();
        u->consecutive_misses = 0;
        if (!u->connected) {
            u->connected = true;
            saint_log_publish("info",
                "RoboClaw: unit %u (0x%02X) connected",
                (unsigned)unit_idx, (unsigned)u->address);
        }
    } else {
        if (u->consecutive_misses < 255) u->consecutive_misses++;
        if (u->connected
            && u->consecutive_misses >= ROBOCLAW_DROP_AFTER_MISSES) {
            u->connected = false;
            saint_log_publish("warn",
                "RoboClaw: unit %u (0x%02X) dropped — %u consecutive missed polls",
                (unsigned)unit_idx, (unsigned)u->address,
                (unsigned)u->consecutive_misses);
        }
    }
}

// =============================================================================
// Public API
// =============================================================================

/*
 * Bind the UART hardware to the configured pin pair and run an
 * initial GETVERSION probe against every populated unit. Idempotent:
 * if the active pins/UART already match the configured ones, this
 * returns immediately without disturbing the bus. Called both at
 * config load (drv_load) and on every config sync (drv_apply_config).
 *
 * Why this is split out from drv_init: drv_init runs at peripheral
 * registration during boot, before any config has arrived. Probing
 * with default pins at that point is worse than useless — if the
 * operator wired RoboClaw to GPIO 4/5 we'd be banging bytes onto
 * 0/1 and never learn anything. Roboclaw_init() instead runs *after*
 * either a stored config restored from flash or a fresh config sync
 * has set roboclaw_tx_pin/roboclaw_rx_pin to where the wires
 * actually are.
 */
void roboclaw_init(void)
{
    // Seed the default address for any unit slot the operator hasn't
    // explicitly touched. Don't touch deadband/max_current_ma/etc —
    // those are operator config that drv_apply_config and drv_load
    // populate BEFORE calling us, and the previous version of this
    // block clobbered them on the first init() right after the
    // populate, leaving the dashboard's deadband setting silently
    // ignored. A "never touched" slot is identified by address==0
    // (the static zero-init value); a real RoboClaw address is
    // 0x80..0x87, so this is unambiguous.
    for (uint8_t i = 0; i < ROBOCLAW_MAX_UNITS; i++) {
        if (units[i].address == 0) {
            units[i].address = ROBOCLAW_ADDRESS_MIN + i;
        }
    }

    // Resolve the operator-picked TX/RX into a UART instance. The
    // table lives in board YAML and is loaded into the firmware via
    // uart_pin_pairs.c — see board configs under saint_os/config/boards.
    // If the pair the operator picked isn't valid we log AND fall back,
    // because a silent fallback is exactly how this driver gets into
    // the "configured pair was 4/5 but I'm running on 0/1" state where
    // the bus looks dead from the dashboard's point of view.
    //
    // This block is outside #ifndef SIMULATION on purpose: it's pure C
    // and the host-side test runner relies on the warn log firing so
    // the silent-fallback regression has a non-hardware test gate.
    uint8_t resolved_inst;
    uint8_t req_tx = roboclaw_tx_pin;
    uint8_t req_rx = roboclaw_rx_pin;
    if (uart_pin_pair_lookup(roboclaw_tx_pin, roboclaw_rx_pin, &resolved_inst)) {
        configured_serial_port = resolved_inst;
    } else {
        saint_log_publish("warn",
            "RoboClaw: requested pair TX=%u RX=%u isn't a valid RP2040 UART pair "
            "— falling back to TX=%u RX=%u (UART0). Check board YAML uart_pairs.",
            (unsigned)req_tx, (unsigned)req_rx,
            (unsigned)ROBOCLAW_DEFAULT_TX_PIN, (unsigned)ROBOCLAW_DEFAULT_RX_PIN);
        roboclaw_tx_pin = ROBOCLAW_DEFAULT_TX_PIN;
        roboclaw_rx_pin = ROBOCLAW_DEFAULT_RX_PIN;
        configured_serial_port = ROBOCLAW_DEFAULT_SERIAL_PORT;
    }

#ifndef SIMULATION
    // Idempotent fast-path: same pair already bound, nothing to do.
    // Without this, every drv_apply_config call (which fires per
    // channel — up to ROBOCLAW_MAX_CHANNELS times per sync) would
    // tear down and reinit the UART, dropping any bytes in flight.
    if (active_tx_pin == roboclaw_tx_pin
        && active_rx_pin == roboclaw_rx_pin
        && active_uart   == configured_serial_port
        && port_initialized) {
        return;
    }

    // Detach the previously-bound pins so they stop driving traffic
    // onto stale wires when the operator moves the cable to a new pair.
    if (active_tx_pin != 0xFF) {
        gpio_set_function(active_tx_pin, GPIO_FUNC_SIO);
        gpio_set_dir(active_tx_pin, GPIO_IN);
    }
    if (active_rx_pin != 0xFF) {
        gpio_set_function(active_rx_pin, GPIO_FUNC_SIO);
        gpio_set_dir(active_rx_pin, GPIO_IN);
    }
    if (active_uart != 0xFF && active_uart != configured_serial_port) {
        uart_deinit(active_uart == 0 ? uart0 : uart1);
    }

    hw_uart = (configured_serial_port == 0) ? uart0 : uart1;
    uart_init(hw_uart, configured_baud);
    gpio_set_function(roboclaw_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(roboclaw_rx_pin, GPIO_FUNC_UART);

    active_tx_pin = roboclaw_tx_pin;
    active_rx_pin = roboclaw_rx_pin;
    active_uart   = configured_serial_port;

    PLATFORM_SLEEP_MS(100);

    // Probe every unit slot. Unit_count grows in apply_config based on
    // configured channels — but at first probe we don't know which
    // addresses the operator has wired, so we sweep the full address
    // range. Any unit that responds gets flipped connected (with a log
    // line) via mark_unit_response. Reset miss counters first so a
    // re-bind doesn't carry stale state into the new probe.
    saint_log_publish("info",
        "RoboClaw: bound UART%u TX=%u RX=%u @ %u baud — probing 0x%02X..0x%02X",
        (unsigned)configured_serial_port,
        (unsigned)roboclaw_tx_pin, (unsigned)roboclaw_rx_pin,
        (unsigned)configured_baud,
        (unsigned)ROBOCLAW_ADDRESS_MIN,
        (unsigned)(ROBOCLAW_ADDRESS_MIN + ROBOCLAW_MAX_UNITS - 1));

    uint8_t connected_count = 0;
    for (uint8_t i = 0; i < ROBOCLAW_MAX_UNITS; i++) {
        units[i].consecutive_misses = 0;
        units[i].last_reprobe_ms    = PLATFORM_MILLIS();
        char version[ROBOCLAW_VERSION_MAX_LEN] = {0};
        if (probe_unit(i, version, sizeof(version))) {
            // mark_unit_response handles the "first connected" log;
            // emit a more detailed line here with the version string
            // since we only get it on a successful GETVERSION.
            mark_unit_response(i, true);
            saint_log_publish("info",
                "RoboClaw: unit %u (0x%02X) version: %s",
                (unsigned)i, (unsigned)units[i].address, version);
            if (i >= unit_count) unit_count = i + 1;
            connected_count++;
        }
    }
    saint_log_publish(connected_count > 0 ? "info" : "warn",
        "RoboClaw: probe complete — %u of %u addresses responded",
        (unsigned)connected_count, (unsigned)ROBOCLAW_MAX_UNITS);
#endif

    port_initialized = true;
}

void roboclaw_update(void)
{
    if (!port_initialized || unit_count == 0) return;

#ifndef SIMULATION
    uint8_t u = poll_unit;
    uint8_t addr = units[u].address;
    uint8_t resp[8];
    bool success = false;

    if (!units[u].connected) {
        // Unit isn't currently connected — try to re-probe rather
        // than spinning on telemetry that's guaranteed to fail. Pace
        // these probes so a chassis with three offline units doesn't
        // fill the bus with GETVERSION traffic.
        uint32_t now = PLATFORM_MILLIS();
        if (now - units[u].last_reprobe_ms >= ROBOCLAW_REPROBE_INTERVAL_MS) {
            units[u].last_reprobe_ms = now;
            char version[ROBOCLAW_VERSION_MAX_LEN] = {0};
            if (probe_unit(u, version, sizeof(version))) {
                mark_unit_response(u, true);
                saint_log_publish("info",
                    "RoboClaw: unit %u (0x%02X) recovered — version: %s",
                    (unsigned)u, (unsigned)addr, version);
            }
            // Don't bump consecutive_misses on a re-probe miss; the
            // unit is already marked disconnected, no state change.
        }
        goto next_poll;
    }

    // Connected unit: poll one telemetry register, advance the cursor.
    // read_response returns false on timeout or CRC mismatch — either
    // way it's a missed poll, which feeds mark_unit_response so we
    // notice when a previously-good controller goes quiet.
    switch (poll_register) {
    case 0: // Encoder
        send_command(addr, ROBOCLAW_CMD_GETM1ENC, NULL, 0);
        if (read_response(resp, 5, addr, ROBOCLAW_CMD_GETM1ENC)) {
            units[u].encoder = ((int32_t)resp[0] << 24) |
                               ((int32_t)resp[1] << 16) |
                               ((int32_t)resp[2] << 8) |
                               resp[3];
            success = true;
        }
        break;

    case 1: // Battery voltage
        send_command(addr, ROBOCLAW_CMD_GETMBATT, NULL, 0);
        if (read_response(resp, 2, addr, ROBOCLAW_CMD_GETMBATT)) {
            uint16_t raw = ((uint16_t)resp[0] << 8) | resp[1];
            units[u].voltage_mv = raw * 100;  // raw/10 = volts -> raw*100 = mV
            success = true;
        }
        break;

    case 2: // Motor current
        send_command(addr, ROBOCLAW_CMD_GETCURRENTS, NULL, 0);
        if (read_response(resp, 4, addr, ROBOCLAW_CMD_GETCURRENTS)) {
            // M1 current is first 2 bytes (value/100 = amps)
            int16_t raw = (int16_t)(((uint16_t)resp[0] << 8) | resp[1]);
            units[u].current_ma = raw * 10;  // raw/100 amps = raw*10 mA
            success = true;
        }
        break;

    case 3: // Temperature
        send_command(addr, ROBOCLAW_CMD_GETTEMP, NULL, 0);
        if (read_response(resp, 2, addr, ROBOCLAW_CMD_GETTEMP)) {
            units[u].temp_tenths = (int16_t)(((uint16_t)resp[0] << 8) | resp[1]);
            success = true;
        }
        break;
    }
    mark_unit_response(u, success);

next_poll:
    // Advance to next register, wrapping to next unit. Iterate over
    // every populated unit (not just connected ones) so the re-probe
    // path above gets a turn — that's how a freshly-plugged controller
    // comes back online without operator intervention.
    poll_register++;
    if (poll_register > 3) {
        poll_register = 0;
        poll_unit++;
        if (poll_unit >= unit_count) {
            poll_unit = 0;
        }
    }
#endif
}

bool roboclaw_is_connected(void)
{
    if (!port_initialized) return false;
    for (uint8_t i = 0; i < unit_count; i++) {
        if (units[i].connected) return true;
    }
    return false;
}

bool roboclaw_set_duty(uint8_t unit, int16_t duty)
{
    if (unit >= ROBOCLAW_MAX_UNITS) return false;
    if (!port_initialized) {
        // The slider arrived but the UART has never been bound. This
        // is the silent-failure mode users hit when no RoboClaw
        // peripheral was synced — duty value never reaches the wire.
        // Surface it in the dashboard Logs tab so it's diagnosable
        // without a serial console.
        saint_log_publish("warn",
            "RoboClaw: set_duty(unit=%u, duty=%d) ignored — driver not "
            "initialized (no peripheral synced; UART not bound).",
            (unsigned)unit, (int)duty);
        return false;
    }

    if (duty > ROBOCLAW_DUTY_MAX) duty = ROBOCLAW_DUTY_MAX;
    if (duty < ROBOCLAW_DUTY_MIN) duty = ROBOCLAW_DUTY_MIN;

    units[unit].duty = duty;

#ifndef SIMULATION
    if (!hw_uart) {
        saint_log_publish("warn",
            "RoboClaw: set_duty(unit=%u, duty=%d) — UART handle null "
            "(internal bind state inconsistent).",
            (unsigned)unit, (int)duty);
        return false;
    }
    uint8_t data[2];
    data[0] = (uint8_t)((uint16_t)duty >> 8);
    data[1] = (uint8_t)((uint16_t)duty & 0xFF);
    send_command(units[unit].address, ROBOCLAW_CMD_M1DUTY, data, 2);
    // Capture the ACK result. Previously thrown away, which made
    // "I'm sliding the control and the motor isn't moving" undetectable
    // from the logs — bytes go out, no signal back. Feeding mark_unit_
    // response gives the same transition-only logging the telemetry
    // poller produces: first ACK success → "unit N (0xNN) connected",
    // ROBOCLAW_DROP_AFTER_MISSES consecutive ACK failures → "dropped —
    // N consecutive missed polls". On a wired RoboClaw in Packet Serial
    // mode, the first duty write will ACK (0xFF) within ~5 ms; if the
    // operator slides and never sees a "connected" line, the issue is
    // wiring/baud/address, not the firmware send path.
    bool ack_ok = read_ack();
    mark_unit_response(unit, ack_ok);
#endif

    return true;
}

bool roboclaw_stop(uint8_t unit)
{
    return roboclaw_set_duty(unit, 0);
}

void roboclaw_stop_all(void)
{
    for (uint8_t i = 0; i < ROBOCLAW_MAX_UNITS; i++) {
        roboclaw_set_duty(i, 0);
    }
}

// =============================================================================
// Peripheral Driver Interface
// =============================================================================

static bool roboclaw_drv_init(void)
{
    // Intentional no-op. peripheral_init_all() fires this during boot,
    // BEFORE either the saved-flash config has loaded or a fresh sync
    // has arrived — so we don't yet know which UART pins the operator
    // wired the RoboClaw to. Probing on defaults here would either
    // succeed by accident (driving traffic on the right wires) or
    // silently fail and leave the dashboard reporting no units even
    // when there are some. Either way the operator can't tell which
    // happened. Real init now runs from drv_load (restored from flash)
    // or drv_apply_config (fresh sync), once we know the pins.
    return true;
}

static bool roboclaw_drv_set_value(uint8_t channel, float value)
{
    uint8_t unit = channel / ROBOCLAW_CHANNELS_PER_UNIT;
    uint8_t sub = channel % ROBOCLAW_CHANNELS_PER_UNIT;

    // Only motor sub-channel is writable
    if (sub != ROBOCLAW_SUB_MOTOR) return false;

    if (value < -1.0f) value = -1.0f;
    if (value > 1.0f) value = 1.0f;
    int16_t duty = (int16_t)(value * ROBOCLAW_DUTY_MAX);
    return roboclaw_set_duty(unit, duty);
}

static bool roboclaw_drv_get_value(uint8_t channel, float* value)
{
    uint8_t unit = channel / ROBOCLAW_CHANNELS_PER_UNIT;
    uint8_t sub = channel % ROBOCLAW_CHANNELS_PER_UNIT;

    if (unit >= ROBOCLAW_MAX_UNITS) return false;

    switch (sub) {
    case ROBOCLAW_SUB_MOTOR:
        *value = (float)units[unit].duty / (float)ROBOCLAW_DUTY_MAX;
        return true;
    case ROBOCLAW_SUB_ENCODER:
        *value = (float)units[unit].encoder;
        return true;
    case ROBOCLAW_SUB_VOLTAGE:
        *value = (float)units[unit].voltage_mv / 1000.0f;  // volts
        return true;
    case ROBOCLAW_SUB_CURRENT:
        *value = (float)units[unit].current_ma / 1000.0f;  // amps
        return true;
    case ROBOCLAW_SUB_TEMP:
        *value = (float)units[unit].temp_tenths / 10.0f;   // celsius
        return true;
    default:
        return false;
    }
}

static void roboclaw_drv_set_defaults(uint8_t channel, pin_config_t* config)
{
    uint8_t unit = channel / ROBOCLAW_CHANNELS_PER_UNIT;
    config->params.roboclaw.address = ROBOCLAW_ADDRESS_MIN + unit;
    config->params.roboclaw.deadband = 0;
    config->params.roboclaw.max_current_ma = 0;
}

static bool roboclaw_drv_apply_config(uint8_t channel, const pin_config_t* config)
{
    uint8_t unit = channel / ROBOCLAW_CHANNELS_PER_UNIT;
    if (unit >= ROBOCLAW_MAX_UNITS) return false;

    units[unit].address = config->params.roboclaw.address;
    units[unit].deadband = config->params.roboclaw.deadband;
    units[unit].max_current_ma = config->params.roboclaw.max_current_ma;
    units[unit].estop_pin = config->params.roboclaw.estop_pin;

    if (unit >= unit_count) {
        unit_count = unit + 1;
    }

    // Drive the E-stop pin LOW (deasserted) IMMEDIATELY on config
    // apply — before roboclaw_init() runs the probe sweep below. The
    // probe sends GETVERSION packets that the controller will only
    // respond to if its S3 latch isn't engaged; if we did the probe
    // before the deassert, a freshly-booted controller could still
    // be latched from boot-time floating S3 and the probe would show
    // zero units responding.
    apply_estop_pin(unit);

    // The operator may have picked a different UART pin pair via the
    // sync. parse_json_params already updated roboclaw_tx_pin /
    // roboclaw_rx_pin from the JSON. roboclaw_init() is idempotent —
    // it no-ops when the active pins already match — so calling it
    // here is the cheapest way to (a) actually bind the new hardware
    // when the pins changed and (b) trigger a probe pass so the
    // operator sees their unit come online in the Logs tab. Without
    // this, the driver would keep talking on whatever pins drv_init
    // bound (which is nothing now, since drv_init is a no-op), and the
    // dashboard would show "no units" forever.
    roboclaw_init();
    return true;
}

static bool roboclaw_drv_parse_json(const char* json_start, const char* json_end,
                                     pin_config_t* config)
{
    const char* p;
    bool got_pins = false;

    p = strstr(json_start, "\"address\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) { p++; while (*p == ' ') p++; config->params.roboclaw.address = (uint8_t)atoi(p); }
    }

    p = strstr(json_start, "\"deadband\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) { p++; while (*p == ' ') p++; config->params.roboclaw.deadband = (uint8_t)atoi(p); }
    }

    p = strstr(json_start, "\"max_current_ma\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) { p++; while (*p == ' ') p++; config->params.roboclaw.max_current_ma = (uint16_t)atoi(p); }
    }

    // Optional: GPIO this unit's S3 (E-stop input) is wired to. Absent
    // from the JSON → field stays at whatever the caller initialized
    // it to (typically 0 = no E-stop wire), which apply_estop_pin
    // treats as a no-op. The server's Peripherals tab modal exposes
    // this as "E-stop pin" on the RoboClaw type; operators on PCBs
    // that don't route an E-stop wire just leave it blank.
    p = strstr(json_start, "\"estop_pin\"");
    if (p && p < json_end) {
        p = strchr(p, ':');
        if (p) {
            p++;
            while (*p == ' ') p++;
            int v = atoi(p);
            if (v >= 1 && v <= 29) {
                config->params.roboclaw.estop_pin = (uint8_t)v;
            } else {
                config->params.roboclaw.estop_pin = 0;  // unset sentinel
            }
        }
    }

    uint8_t tx, rx, inst;
    if (uart_pin_pair_parse_json(json_start, json_end, &tx, &rx, &inst)) {
        roboclaw_tx_pin = tx;
        roboclaw_rx_pin = rx;
        configured_serial_port = inst;
        got_pins = true;
    }

    // Per-channel parse_json fires up to ROBOCLAW_MAX_CHANNELS times
    // (5 channels × up to 8 units) on a sync, all with the same pin
    // payload. Only log once — on the first channel of the first unit
    // — so the Logs tab doesn't drown in 40 identical lines.
    if (config->gpio == ROBOCLAW_VIRTUAL_GPIO_BASE) {
        char estop_buf[16];
        if (config->params.roboclaw.estop_pin >= 1
            && config->params.roboclaw.estop_pin <= 29) {
            snprintf(estop_buf, sizeof(estop_buf), "GPIO %u",
                     (unsigned)config->params.roboclaw.estop_pin);
        } else {
            snprintf(estop_buf, sizeof(estop_buf), "none");
        }
        if (got_pins) {
            saint_log_publish("info",
                "RoboClaw sync: pins TX=%u RX=%u (UART%u), unit 0 addr=0x%02X "
                "deadband=%u max_current=%u mA, E-stop pin=%s",
                (unsigned)roboclaw_tx_pin, (unsigned)roboclaw_rx_pin,
                (unsigned)configured_serial_port,
                (unsigned)config->params.roboclaw.address,
                (unsigned)config->params.roboclaw.deadband,
                (unsigned)config->params.roboclaw.max_current_ma,
                estop_buf);
        } else {
            saint_log_publish("warn",
                "RoboClaw sync: didn't find uart_tx/uart_rx in JSON — "
                "driver will keep using TX=%u RX=%u (E-stop pin=%s)",
                (unsigned)roboclaw_tx_pin, (unsigned)roboclaw_rx_pin,
                estop_buf);
        }
    }

    return true;
}

static void roboclaw_drv_estop(void)
{
    roboclaw_stop_all();
    saint_log_publish("warn", "RoboClaw: ESTOP — all units commanded to duty=0");
}

static bool roboclaw_drv_save(void* storage_ptr)
{
    flash_storage_data_t* storage = (flash_storage_data_t*)storage_ptr;

    memset(&storage->roboclaw_config, 0, sizeof(storage->roboclaw_config));
    storage->roboclaw_config.unit_count = unit_count;
    storage->roboclaw_config.serial_port = configured_serial_port;
    storage->roboclaw_config.baud_rate = configured_baud;

    storage->uart_pins.roboclaw_tx_pin = roboclaw_tx_pin;
    storage->uart_pins.roboclaw_rx_pin = roboclaw_rx_pin;

    for (uint8_t i = 0; i < ROBOCLAW_MAX_UNITS; i++) {
        storage->roboclaw_config.units[i].address = units[i].address;
        storage->roboclaw_config.units[i].deadband = units[i].deadband;
        storage->roboclaw_config.units[i].max_current_ma = units[i].max_current_ma;
        storage->roboclaw_config.units[i].estop_pin = units[i].estop_pin;
    }

    return true;
}

static bool roboclaw_drv_load(const void* storage_ptr)
{
    const flash_storage_data_t* storage = (const flash_storage_data_t*)storage_ptr;

    uint8_t stored_tx = storage->uart_pins.roboclaw_tx_pin;
    uint8_t stored_rx = storage->uart_pins.roboclaw_rx_pin;
    if (stored_tx != 0 || stored_rx != 0) {
        uint8_t inst;
        if (uart_pin_pair_lookup(stored_tx, stored_rx, &inst)) {
            roboclaw_tx_pin = stored_tx;
            roboclaw_rx_pin = stored_rx;
            configured_serial_port = inst;
        } else {
            saint_log_publish("warn",
                "RoboClaw: stored pin pair TX=%u RX=%u isn't a valid RP2040 "
                "UART pair — using defaults. Re-sync from the dashboard.",
                (unsigned)stored_tx, (unsigned)stored_rx);
        }
    }

    // No saved peripheral → stay dormant. Surface this in the log so
    // the operator doesn't burn time wondering why the UART is quiet:
    // it's quiet on purpose because no RoboClaw config has been synced
    // (or saved) yet. A sync from the dashboard fires
    // roboclaw_drv_apply_config → roboclaw_init() and starts traffic.
    if (storage->roboclaw_config.unit_count == 0) {
        saint_log_publish("info",
            "RoboClaw: no saved peripheral in flash — driver dormant "
            "(no UART traffic). Sync a RoboClaw config from the dashboard "
            "to bind the UART and start polling.");
        return true;
    }

    uint8_t count = storage->roboclaw_config.unit_count;
    if (count > ROBOCLAW_MAX_UNITS) count = ROBOCLAW_MAX_UNITS;
    unit_count = count;

    if (storage->roboclaw_config.baud_rate > 0) {
        configured_baud = storage->roboclaw_config.baud_rate;
    }
    configured_serial_port = storage->roboclaw_config.serial_port;

    for (uint8_t i = 0; i < count; i++) {
        units[i].address = storage->roboclaw_config.units[i].address;
        units[i].deadband = storage->roboclaw_config.units[i].deadband;
        units[i].max_current_ma = storage->roboclaw_config.units[i].max_current_ma;
        units[i].estop_pin = storage->roboclaw_config.units[i].estop_pin;
        // Drive the E-stop pin LOW now — BEFORE roboclaw_init() runs
        // the probe sweep at the end of drv_load. If the saved config
        // assigned an E-stop pin, the controller may have latched
        // E-stop from a floating boot-time signal; deasserting first
        // means the probe and any pending duty commands see a
        // clean S3.
        apply_estop_pin(i);
    }

    saint_log_publish("info",
        "RoboClaw: restored %u unit configs from flash (UART%u TX=%u RX=%u @ %u baud)",
        (unsigned)count, (unsigned)configured_serial_port,
        (unsigned)roboclaw_tx_pin, (unsigned)roboclaw_rx_pin,
        (unsigned)configured_baud);

    // Restored config means the operator had this peripheral configured
    // before the last reboot — start probing right away so the dashboard
    // doesn't show units as offline until the next sync.
    roboclaw_init();
    return true;
}

// Static driver struct
static const peripheral_driver_t roboclaw_peripheral = {
    .name              = "roboclaw",
    .mode_string       = "roboclaw_motor",
    .pin_mode          = PIN_MODE_ROBOCLAW_MOTOR,
    .capability_flag   = PIN_CAP_ROBOCLAW_MOTOR,
    .virtual_gpio_base = ROBOCLAW_VIRTUAL_GPIO_BASE,
    .channel_count          = ROBOCLAW_MAX_CHANNELS,
    .channels_per_instance  = ROBOCLAW_CHANNELS_PER_UNIT,
    .init              = roboclaw_drv_init,
    .update            = roboclaw_update,
    .is_connected      = roboclaw_is_connected,
    .set_value         = roboclaw_drv_set_value,
    .get_value         = roboclaw_drv_get_value,
    .set_defaults      = roboclaw_drv_set_defaults,
    .apply_config      = roboclaw_drv_apply_config,
    .parse_json_params = roboclaw_drv_parse_json,
    .estop             = roboclaw_drv_estop,
    .save_config       = roboclaw_drv_save,
    .load_config       = roboclaw_drv_load,
};

const peripheral_driver_t* roboclaw_get_peripheral_driver(void)
{
    return &roboclaw_peripheral;
}
