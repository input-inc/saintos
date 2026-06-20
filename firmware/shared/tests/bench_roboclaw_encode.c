/**
 * Benchmark: RoboClaw command encode (shared C).
 *
 * Times the per-/control-tick work to form ONE M1DUTY motor command on
 * the wire — send_command(address, ROBOCLAW_CMD_M1DUTY, data, 2):
 * packet assembly [addr, cmd, duty_hi, duty_lo] → CRC16 (XMODEM 0x1021,
 * per-byte 8-iteration loop) → append CRC → hand to transport. This is
 * the part WE own; the RoboClaw controller past the UART is a black box,
 * so the bench stops at the transport handoff (write stubbed to a sink).
 *
 * Also breaks out the raw CRC16 cost (the dominant term) for comparison.
 * See bench_harness.h for the timing model + "relative, not MCU
 * wall-clock" caveat.
 */
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "bench_harness.h"

/* ── saint_log stub ───────────────────────────────────────────────── */
#define SAINT_LOG_H
static void saint_log_publish(const char* l, const char* f, ...) { (void)l; (void)f; }
static void saint_log_boot_queue(const char* l, const char* f, ...) { (void)l; (void)f; }
static void saint_log_set_ros_ready(bool r) { (void)r; }
static void saint_log_drain_boot_queue(void) {}
static void saint_log_emit_local(const char* l, const char* t) { (void)l; (void)t; }
static bool saint_log_emit_ros(const char* j, size_t n) { (void)j; (void)n; return true; }
static uint32_t saint_log_uptime_ms(void) { return 0; }

/* ── estop pin externs the driver references (not on encode path) ─── */
void roboclaw_estop_apply_pin(uint8_t p)  { (void)p; }
void roboclaw_estop_assert_pin(uint8_t p) { (void)p; }

/* ── uart_pin_pairs helpers — used by init/drv_load, NOT the encode
 *    path. Stub them so the linker is satisfied; the bench never calls
 *    init/drv_load, so these bodies never run. ─────────────────────── */
#include "uart_pin_pairs.h"
bool uart_pin_pair_lookup(uint8_t tx, uint8_t rx, uint8_t* out_instance)
{ (void)tx; (void)rx; if (out_instance) *out_instance = 0; return true; }
bool uart_pin_pair_parse_json(const char* a, const char* b,
                              uint8_t* tx, uint8_t* rx, uint8_t* inst)
{ (void)a; (void)b; (void)tx; (void)rx; (void)inst; return false; }

/* ── transport fake: sink the assembled packet, no real UART ──────── */
#include "roboclaw_transport.h"
static size_t g_bytes = 0, g_packets = 0;
static bool   fc_open(uint8_t tx, uint8_t rx, uint32_t b, bool s) { (void)tx;(void)rx;(void)b;(void)s; return true; }
static bool   fc_verify(void) { return true; }
static bool   fc_is_open(void) { return true; }
static bool   fc_write(const uint8_t* d, size_t n) {
    /* Fold the actual written bytes (incl. the appended CRC16) into the
     * volatile sink. Without an observable dependency on the encoded
     * output, -O2 collapses the whole assemble+crc16 path to closed-form
     * counter bumps and the timed loop measures ~0 ns. */
    if (n) BENCH_SINK(d[0] + d[n - 1]);
    g_bytes += n; g_packets++; return true;
}
static size_t fc_read(uint8_t* d, size_t n) { (void)d; (void)n; return 0; }
static bool   fc_pio_active(void) { return false; }
static uint8_t fc_instance(void) { return 0; }
static const roboclaw_transport_ops_t fake_ops = {
    .name = "fake", .open = fc_open, .verify_binding = fc_verify,
    .is_open = fc_is_open, .write = fc_write, .read = fc_read,
    .pio_swap_active = fc_pio_active, .resolved_instance = fc_instance,
};
const roboclaw_transport_ops_t* roboclaw_get_transport(void) { return &fake_ops; }

/* Host platform shim (skipped when run_benchmarks.sh force-includes it). */
#ifndef PLATFORM_H
#define PLATFORM_H 1
#define PLATFORM_MILLIS()      (0u)
#define PLATFORM_SLEEP_MS(ms)  ((void)(ms))
#define PLATFORM_PRINTF        printf
#endif

#include "../src/roboclaw_driver.c"
#include "roboclaw_protocol.h"   /* CRC16 helpers (public, not SIM-gated) */

int main(void)
{
    const uint8_t ADDR = 0x80;   /* RoboClaw unit 0 */

    printf("\nbench_roboclaw_encode — M1DUTY command encode (our side of the UART)\n");

    /* Full per-tick encode: assemble [addr, M1DUTY, hi, lo] + CRC16 +
     * hand to transport. send_command is the real driver path; the
     * fake transport's is_open() satisfies wire_ready(). Vary the duty
     * so the encoded bytes (and CRC) change each call. */
    BENCH("send_command M1DUTY (assemble+crc16+tx)", 2000000, {
        int16_t duty = (int16_t)((_i % 65535) - 32767);
        uint8_t data[2] = { (uint8_t)((uint16_t)duty >> 8),
                            (uint8_t)((uint16_t)duty & 0xFF) };
        send_command(ADDR, ROBOCLAW_CMD_M1DUTY, data, 2);
    });

    /* Isolate the CRC16 cost over the 4-byte M1DUTY frame — the
     * dominant term in the encode (per-byte 8-iteration shift loop). */
    BENCH("crc16 over 4-byte M1DUTY frame", 2000000, {
        uint8_t frame[4] = { ADDR, ROBOCLAW_CMD_M1DUTY,
                             (uint8_t)(_i >> 8), (uint8_t)_i };
        BENCH_SINK(roboclaw_crc16_calculate(frame, 4));
    });

    if (g_packets == 0) { fprintf(stderr, "FAIL: no packets encoded\n"); return 1; }
    printf("  sanity: %zu packets, avg %.1f bytes/packet (expect 6: addr+cmd+2 data+2 crc)\n",
           g_packets, (double)g_bytes / g_packets);
    return 0;
}
