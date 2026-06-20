/**
 * Benchmark: the firmware Maestro control hot path (shared C).
 *
 * Times the per-/control-tick work to drive one Maestro channel —
 * drv_set_value(channel, normalized): normalize −1..+1 → piecewise
 * pulse map through neutral → quarter-µs → maestro_set_target
 * (per-channel min/max clamp) → compact-protocol byte assembly →
 * transport write (stubbed memcpy). Isolates SHARED control compute
 * from the parts only measurable on real hardware (USB/UART I/O, the
 * platform-specific JSON parse). See bench_harness.h for the timing
 * model and the "relative, not MCU wall-clock" caveat.
 */
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

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

/* ── transport fake: sink assembled bytes, no real I/O ────────────── */
#include "flash_types.h"
#include "maestro_transport.h"
static size_t g_bytes = 0, g_writes = 0;
static bool fake_open(const flash_storage_data_t* s) { (void)s; return true; }
static void fake_update(void) {}
static bool fake_is_connected(void) { return true; }
static bool fake_write(const uint8_t* d, size_t n) { (void)d; g_bytes += n; g_writes++; return true; }
static size_t fake_read(uint8_t* d, size_t n, uint32_t t) { (void)d; (void)n; (void)t; return 0; }
static const maestro_transport_ops_t uart_ops = {
    .name = "uart", .open = fake_open, .update = fake_update,
    .is_connected = fake_is_connected, .write = fake_write,
    .read = fake_read, .supports_hotplug = NULL,
};
const maestro_transport_ops_t* maestro_get_transport_usb_cdc(void)    { return NULL; }
const maestro_transport_ops_t* maestro_get_transport_usb_vendor(void) { return NULL; }
const maestro_transport_ops_t* maestro_get_transport_usb_host(void)   { return NULL; }
const maestro_transport_ops_t* maestro_get_transport_uart(void)       { return &uart_ops; }

/* peripheral-state JSON helper — not on the control path; stub it. */
int peripheral_state_append_channel(char* b, size_t c, bool* f,
                                    const char* p, const char* ch, float v) {
    (void)b; (void)c; (void)f; (void)p; (void)ch; (void)v; return 0;
}

/* Host platform shim (skipped when run_benchmarks.sh force-includes
 * host_platform.h, which defines PLATFORM_H first). */
#ifndef PLATFORM_H
#define PLATFORM_H 1
#define PLATFORM_MILLIS()      (0u)
#define PLATFORM_SLEEP_MS(ms)  ((void)(ms))
#define PLATFORM_PRINTF        printf
#endif

#include "../src/maestro_driver.c"

int main(void)
{
    enum { CHANNELS = 18 };
    maestro_init();
    g_transport = &uart_ops;   /* bind without a full flash load */

    printf("\nbench_maestro_control — Maestro control hot path (%d channels)\n", CHANNELS);
    BENCH("drv_set_value (normalize+map+clamp+tx)", 2000000,
          drv_set_value((uint8_t)(_i % CHANNELS), sinf((float)_i * 0.05f)));
    BENCH_DIST("drv_set_value (per-call dist)", 500000,
          drv_set_value((uint8_t)(_i % CHANNELS), sinf((float)_i * 0.05f)));

    if (g_writes == 0) { fprintf(stderr, "FAIL: control chain wrote nothing\n"); return 1; }
    printf("  sanity: %zu writes, avg %.1f bytes/call\n", g_writes, (double)g_bytes / g_writes);
    return 0;
}
