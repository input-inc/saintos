/**
 * SAINT.OS Node Firmware — Teensy 4.1 hardware watchdog.
 *
 * Two-call API:
 *   watchdog_init() — call once from setup(), AFTER Serial.begin so the
 *                     boot-log "previous reset was WDOG1" line is
 *                     visible. Arms WDOG1 with a coarse timeout
 *                     (default 30 s, override via -DSAINT_WATCHDOG_TIMEOUT_S=N).
 *                     Once armed, WDOG1 is write-once-locked and stays
 *                     on until reset.
 *
 *   watchdog_feed() — call from loop() (and from any long-running
 *                     blocking section in setup()). Reloads the
 *                     counter. If feed stops for > timeout seconds,
 *                     WDOG1 issues a full chip reset.
 *
 * Both calls are no-ops under SIMULATION — see watchdog.cpp header
 * for the rationale.
 *
 * See firmware/teensy41/docs/POST_INIT_HANG.md for the failure mode
 * this guards against.
 */
#ifndef SAINT_WATCHDOG_H
#define SAINT_WATCHDOG_H

#ifdef __cplusplus
extern "C" {
#endif

void watchdog_init(void);
void watchdog_feed(void);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_WATCHDOG_H */
