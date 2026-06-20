/**
 * SAINT.OS — external NeoPixel (WS2812) strip interface.
 *
 * The ONBOARD status NeoPixel/LED is NOT managed here — it's owned by
 * led_status.{c,cpp} and driven by node_state plus the operator color
 * override. This interface is for OPERATOR-ADDED NeoPixels on an
 * arbitrary data GPIO (catalog type "neopixel", pin slot "data",
 * `pixel_count` LEDs), pushed in the firmware config's peripherals
 * array.
 *
 * The interface is platform-agnostic; each platform provides its own
 * WS2812 implementation under the hood:
 *   • Teensy 4.1 — Adafruit_NeoPixel (bit-bang, any pin)
 *   • RP2040     — PIO state machine (TODO)
 * In SIMULATION builds the implementation is a logging stub (no WS2812
 * hardware in Renode), mirroring led_status's PIO guard.
 */
#ifndef SAINT_NEOPIXEL_STRIP_H
#define SAINT_NEOPIXEL_STRIP_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Clear every configured external strip. Called at the start of each
 * config apply so the strip set is rebuilt from the new peripheral
 * list (mirrors pin_config_reset). Frees per-strip resources. */
void neopixel_strip_reset(void);

/* Register (or re-point) an external WS2812 strip with `count` pixels
 * on data `pin`, keyed by the operator's peripheral instance id.
 * Idempotent per id — re-adding the same id reuses the slot, and a
 * changed pin/count re-initializes it. Returns false if the strip
 * table is full or the arguments are invalid. */
bool neopixel_strip_add(const char* id, uint8_t pin, uint16_t count);

/* True iff an external strip with this exact instance id is
 * registered. The set_channel router uses this to tell an operator
 * strip apart from the onboard status LED (which is never a strip). */
bool neopixel_strip_exists(const char* id);

/* Paint the whole strip `id` a packed 0xRRGGBB color. Returns false if
 * `id` isn't a registered external strip. */
bool neopixel_strip_set_color(const char* id, uint32_t rgb);

/* Set the whole-strip brightness (0..255) for strip `id`, keeping its
 * current color. Returns false if `id` isn't registered. */
bool neopixel_strip_set_brightness(const char* id, uint8_t brightness);

/* Drive every configured strip dark — called from the ESTOP path. */
void neopixel_strip_all_off(void);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_NEOPIXEL_STRIP_H */
