/**
 * Host build shim for the shared platform abstraction.
 *
 * Shared sources (maestro_driver.c, saint_log.c, …) include the shared
 * platform.h, which is a GUARD stub: it #errors unless a platform has
 * already defined PLATFORM_H plus the PLATFORM_* macros. On RP2040 /
 * Teensy the per-target platform.h supplies these; for host-compiled
 * unit tests + benchmarks there is no target platform, so this shim
 * stands in.
 *
 * run_tests.sh force-includes this (`-include host_platform.h`) ahead of
 * every test translation unit, so by the time the source pulls in
 * platform.h the guard is satisfied and the macros resolve.
 *
 * PLATFORM_MILLIS() returns a monotonic millisecond counter backed by
 * the host clock (NOT a constant) so tests that exercise time-based
 * logic — idle-disengage timers, log rate-limits — see time advance.
 */
#ifndef SAINT_HOST_PLATFORM_SHIM_H
#define SAINT_HOST_PLATFORM_SHIM_H

#define PLATFORM_H 1   /* satisfy the shared platform.h guard */

#include <stdio.h>
#include <stdint.h>
#include <time.h>

static inline uint32_t _host_millis(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

static inline uint32_t _host_micros(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000000u + ts.tv_nsec / 1000u);
}

/* The full set of PLATFORM_* macros the shared sources reference (grep
 * `PLATFORM_` in firmware/shared/src). Time sources are host-clock
 * backed; the side-effect hooks (sleep, watchdog, led) are no-ops off
 * target. Keep this in lockstep with what the sources use. */
#define PLATFORM_MILLIS()        (_host_millis())
#define PLATFORM_MICROS()        (_host_micros())
#define PLATFORM_SLEEP_MS(ms)    ((void)(ms))
#define PLATFORM_PRINTF          printf
#define PLATFORM_WATCHDOG_FEED() ((void)0)
#define PLATFORM_LED_TICK()      ((void)0)

#endif /* SAINT_HOST_PLATFORM_SHIM_H */
