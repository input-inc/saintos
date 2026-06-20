/**
 * SAINT.OS Node Firmware - Teensy 4.1 Platform Abstraction
 */

#ifndef PLATFORM_H
#define PLATFORM_H

#include <Arduino.h>

#define PLATFORM_SLEEP_MS(ms)  delay(ms)
#define PLATFORM_MILLIS()      ((uint32_t)millis())
#define PLATFORM_MICROS()      ((uint32_t)micros())

#ifdef SIMULATION
// In simulation, USB Serial has no host to flush to — its output disappears.
// Renode CAN model hardware UARTs (NXP_LPUART), so route every `Serial.foo()`
// call to `Serial1` (LPUART6 on Teensy 4.1, pins 0/1). The simulation .repl
// hooks LPUART6's analyzer/file-backend to capture this stream just like the
// RP2040 path captures UART0. `Serial.begin(...)` becomes `Serial1.begin(...)`
// automatically via this redefinition, so the existing setup() call wires up
// the right peripheral with no extra work.
//
// Caveat: this `#define` applies to every translation unit that pulls in
// `platform.h` — user firmware files do, the Arduino framework does not.
// If you ever need real USB Serial in sim (e.g. to test the OTA loop on a
// real host), wrap the relevant code in `#undef Serial ... #define Serial`.
#define Serial Serial1
#endif

#define PLATFORM_PRINTF        Serial.printf

// Side-effect hooks that long-running shared helpers (e.g.
// discover_server_retry_forever) call inside their sleep loops so the
// status LED keeps animating and the hardware watchdog doesn't reset
// the chip mid-wait. Per-platform implementations:
//   Teensy — led_update() lives in src/led_status.cpp; watchdog_feed()
//   is in src/watchdog.cpp (drives WDOG1; see the block comment there
//   for the 30 s timeout rationale).
#ifdef __cplusplus
extern "C" {
#endif
void led_update(void);
void watchdog_feed(void);
#ifdef __cplusplus
}
#endif
#define PLATFORM_LED_TICK()       led_update()
#define PLATFORM_WATCHDOG_FEED()  watchdog_feed()

#endif // PLATFORM_H
