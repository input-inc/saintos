/**
 * SAINT.OS Node Firmware - RP2040 Platform Abstraction
 */

#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdio.h>          // for PLATFORM_PRINTF → printf in cpp callers
#include "pico/stdlib.h"
#include "hardware/watchdog.h"

#define PLATFORM_SLEEP_MS(ms)  sleep_ms(ms)
#define PLATFORM_MILLIS()      to_ms_since_boot(get_absolute_time())
#define PLATFORM_MICROS()      ((uint32_t)to_us_since_boot(get_absolute_time()))
#define PLATFORM_PRINTF        printf

// Side-effect hooks that long-running shared helpers (e.g.
// discover_server_retry_forever) call inside their sleep loops so the
// status LED keeps animating and the hardware watchdog doesn't reset
// the chip mid-wait. Per-platform implementations:
//   RP2040 — led_update() lives in src/led_status.c; watchdog_update()
//   is the pico SDK's hardware-watchdog feed.
//
// led_update is wrapped in extern "C" to match the declaration in
// shared/include/saint_types.h (which is inside its own extern "C"
// block). Without the wrapper, a C++ TU including both saint_types.h
// AND platform.h sees one C-linkage declaration and one C++-linkage
// declaration of the same symbol → "conflicting declaration" error.
#ifdef __cplusplus
extern "C" {
#endif
void led_update(void);
#ifdef __cplusplus
}
#endif
#define PLATFORM_LED_TICK()       led_update()
#define PLATFORM_WATCHDOG_FEED()  watchdog_update()

#endif // PLATFORM_H
