/**
 * Benchmark: FAS100 S.Port frame parser (shared C).
 *
 * Times the inbound decode we own — sport_feed_byte() fed one byte at a
 * time: poll-header reset → physical-id skip → 0x7D/0x20 destuffing →
 * 8-byte frame accumulation → S.Port CRC check → data-id dispatch →
 * value decode → store_telemetry. This is the read-path twin of the
 * RoboClaw/Maestro encode benches: the FAS100 sensor past the UART is a
 * black box; everything from the received byte to a stored telemetry
 * value is ours.
 *
 * Measures cost per COMPLETE frame (the unit that arrives per poll). The
 * decoded reading is folded into BENCH_SINK so -O2 can't elide the
 * parse + store.
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

/* ── transport + uart_pin_pairs: used by init/update, NOT the parser.
 *    Stub so the driver links; the bench drives sport_feed_byte directly
 *    and never opens a transport. ──────────────────────────────────── */
#include "fas100_transport.h"
const fas100_transport_ops_t* fas100_get_transport(void) { return NULL; }
#include "uart_pin_pairs.h"
bool uart_pin_pair_lookup(uint8_t tx, uint8_t rx, uint8_t* out)
{ (void)tx; (void)rx; if (out) *out = 0; return true; }
bool uart_pin_pair_parse_json(const char* a, const char* b,
                              uint8_t* tx, uint8_t* rx, uint8_t* inst)
{ (void)a; (void)b; (void)tx; (void)rx; (void)inst; return false; }

/* Host platform shim (skipped when run_benchmarks.sh force-includes it). */
#ifndef PLATFORM_H
#define PLATFORM_H 1
#define PLATFORM_MILLIS()      (0u)
#define PLATFORM_SLEEP_MS(ms)  ((void)(ms))
#define PLATFORM_PRINTF        printf
#endif

#include "../src/fas100_driver.c"

/* Build the 8-byte S.Port response payload for a data frame:
 * [0x10, id_lo, id_hi, v0..v3, crc]. Returns via `out` (8 bytes). */
static void build_frame(uint16_t data_id, uint32_t value, uint8_t out[8])
{
    out[0] = SPORT_DATA_HEADER;
    out[1] = (uint8_t)(data_id & 0xFF);
    out[2] = (uint8_t)(data_id >> 8);
    out[3] = (uint8_t)(value & 0xFF);
    out[4] = (uint8_t)(value >> 8);
    out[5] = (uint8_t)(value >> 16);
    out[6] = (uint8_t)(value >> 24);
    out[7] = sport_crc_calculate(out, 7);
}

int main(void)
{
    fas100_init();

    /* A valid VOLTAGE frame (value 1234 → 12.34 V). Built once; the
     * loop only measures the per-byte parse + decode + store. */
    uint8_t payload[8];
    build_frame(SPORT_DATA_ID_VOLTAGE, 1234, payload);

    printf("\nbench_fas100_parse — S.Port frame parser (our side of the UART)\n");

    /* Per complete frame: poll header (reset) + physical id (skipped) +
     * 8 payload bytes through the destuff/accumulate/CRC/decode path.
     * Sink the decoded voltage so the parse + store can't be optimized
     * out. */
    BENCH("sport_feed_byte: full frame parse", 2000000, {
        sport_feed_byte(SPORT_POLL_HEADER);
        sport_feed_byte(SPORT_FAS100_PHYSICAL_ID);
        for (int _k = 0; _k < 8; _k++) sport_feed_byte(payload[_k]);
        BENCH_SINK((uint64_t)(fas100_get_voltage() * 1000.0f));
    });

    /* Isolate the S.Port CRC over the 7 payload bytes (the per-frame
     * integrity check). Vary a byte per iteration — a pure function of a
     * CONSTANT buffer is loop-invariant and -O2 hoists it out (BENCH_SINK
     * doesn't help; the input must change). */
    BENCH("sport_crc_calculate over 7 bytes", 2000000, {
        payload[3] = (uint8_t)_i;
        BENCH_SINK(sport_crc_calculate(payload, 7));
    });

    fas100_diag_t d;
    fas100_get_diag(&d);
    printf("  sanity: frames_ok=%lu crc_bad=%lu  voltage=%.2f V\n",
           (unsigned long)d.frames_ok, (unsigned long)d.frames_crc_bad,
           (double)fas100_get_voltage());
    if (d.frames_ok == 0) { fprintf(stderr, "FAIL: no frames parsed\n"); return 1; }
    return 0;
}
