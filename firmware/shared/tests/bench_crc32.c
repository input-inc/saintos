/**
 * Benchmark: shared CRC-32 (saint_crc32_update).
 *
 * Runs on the OTA verify path — every node CRCs the whole downloaded
 * firmware image to confirm bit-exact equality with the server's value
 * before swapping it in. Throughput here bounds how long that verify
 * takes (a ~1 MB image / MB-per-sec). Pure platform-agnostic C, so the
 * host number is a clean relative measure of the algorithm's cost.
 *
 * Example of benching a function whose result must be kept live so -O2
 * doesn't delete the call: we fold the running CRC into BENCH_SINK.
 */
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "bench_harness.h"
#include "../src/crc32.c"

int main(void)
{
    /* A 4 KiB block — the unit OTA streams/verifies in chunks. */
    enum { BLK = 4096 };
    static uint8_t buf[BLK];
    for (int i = 0; i < BLK; i++) buf[i] = (uint8_t)(i * 31 + 7);

    printf("\nbench_crc32 — saint_crc32_update (OTA image verify)\n");

    /* Per-4KiB-block throughput. Keep the CRC live via BENCH_SINK. */
    BENCH("crc32 4KiB block", 200000,
          BENCH_SINK(saint_crc32_update(0, buf, BLK)));

    /* Report MB/s explicitly — the operator-meaningful number for "how
     * long does verifying a firmware image take". */
    long n = 200000;
    long long t0 = bench_now_ns();
    uint32_t crc = 0;
    for (long i = 0; i < n; i++) crc = saint_crc32_update(crc, buf, BLK);
    long long el = bench_now_ns() - t0;
    BENCH_SINK(crc);
    double mb = (double)n * BLK / (1024.0 * 1024.0);
    printf("  throughput: %.0f MB/s  (%.1f MB in %.1f ms)\n",
           mb / ((double)el / 1e9), mb, (double)el / 1e6);
    return 0;
}
