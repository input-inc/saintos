/**
 * SAINT.OS Teensy 4.1 — external NeoPixel (WS2812) strips.
 *
 * Implements neopixel_strip.h for operator-added NeoPixels on an
 * arbitrary data pin (catalog type "neopixel"). Uses Adafruit_NeoPixel
 * for the timing-critical WS2812 bit-bang, which works on any GPIO on
 * the Teensy 4.1 (i.MX RT1062). The onboard status LED is unrelated —
 * see led_status.cpp.
 *
 * SIMULATION builds stub the body: Renode has no WS2812 hardware (same
 * reason led_status guards its PIO code), and Adafruit_NeoPixel isn't a
 * sim dependency. The stubs keep the config-apply / set_channel call
 * sites identical across envs.
 *
 * Lifecycle: neopixel_strip_reset() runs at the head of every config
 * apply, then neopixel_strip_add() re-registers each strip in the new
 * peripheral list. Config pushes are infrequent (adopt / config edit /
 * reconnect), so re-creating the Adafruit_NeoPixel objects then is
 * cheap and keeps the table a faithful mirror of the operator config.
 */

#include <Arduino.h>
#include "platform.h"
#include <string.h>

extern "C" {
#include "neopixel_strip.h"
}

// Each strip carries a 3-byte/pixel buffer; cap the count so a typo in
// the dashboard ("3000") can't exhaust RAM. 300 matches the catalog's
// pixel_count max. Strip table is small — a handful of external strips
// per node is plenty; extra registrations are rejected with a warning.
#define NEOPIXEL_MAX_STRIPS  4
#define NEOPIXEL_MAX_PIXELS  300
#define NEOPIXEL_ID_LEN      32

#ifndef SIMULATION

#include <Adafruit_NeoPixel.h>

namespace {

struct Strip {
    char              id[NEOPIXEL_ID_LEN];
    Adafruit_NeoPixel *px;       // nullptr when the slot is free
    uint8_t           pin;
    uint16_t          count;
    uint8_t           r, g, b;   // last commanded color (pre-brightness)
    uint8_t           brightness;
    bool              active;
};

Strip g_strips[NEOPIXEL_MAX_STRIPS];

Strip *find(const char *id)
{
    if (!id || !*id) return nullptr;
    for (int i = 0; i < NEOPIXEL_MAX_STRIPS; i++) {
        if (g_strips[i].active && strcmp(g_strips[i].id, id) == 0) {
            return &g_strips[i];
        }
    }
    return nullptr;
}

// Re-push the whole strip at the current color scaled by brightness.
// Adafruit applies setBrightness() during show(), so we set the raw
// color on every pixel and let the library scale.
void render(Strip *s)
{
    if (!s || !s->px) return;
    s->px->setBrightness(s->brightness);
    uint32_t c = Adafruit_NeoPixel::Color(s->r, s->g, s->b);
    for (uint16_t i = 0; i < s->count; i++) s->px->setPixelColor(i, c);
    s->px->show();
}

} // namespace

extern "C" {

void neopixel_strip_reset(void)
{
    for (int i = 0; i < NEOPIXEL_MAX_STRIPS; i++) {
        if (g_strips[i].px) {
            // Blank before tearing down so a removed strip doesn't
            // latch its last frame on the LEDs.
            g_strips[i].px->clear();
            g_strips[i].px->show();
            delete g_strips[i].px;
            g_strips[i].px = nullptr;
        }
        g_strips[i].active = false;
        g_strips[i].id[0]  = '\0';
    }
}

bool neopixel_strip_add(const char *id, uint8_t pin, uint16_t count)
{
    if (!id || !*id) {
        Serial.printf("NeoPixel: strip add ignored — empty id\n");
        return false;
    }
    if (count == 0) count = 1;
    if (count > NEOPIXEL_MAX_PIXELS) {
        Serial.printf("NeoPixel: '%s' count %u clamped to %u\n",
                      id, (unsigned)count, NEOPIXEL_MAX_PIXELS);
        count = NEOPIXEL_MAX_PIXELS;
    }

    Strip *s = find(id);
    if (!s) {
        for (int i = 0; i < NEOPIXEL_MAX_STRIPS; i++) {
            if (!g_strips[i].active) { s = &g_strips[i]; break; }
        }
    }
    if (!s) {
        Serial.printf("NeoPixel: strip table full — '%s' on pin %u dropped\n",
                      id, (unsigned)pin);
        return false;
    }

    // (Re)create the underlying object when new or when pin/count moved.
    if (!s->px || s->pin != pin || s->count != count) {
        if (s->px) delete s->px;
        s->px = new Adafruit_NeoPixel(count, pin, NEO_GRB + NEO_KHZ800);
        if (!s->px) {
            Serial.printf("NeoPixel: alloc failed for '%s'\n", id);
            s->active = false;
            return false;
        }
        s->px->begin();
    }

    strncpy(s->id, id, NEOPIXEL_ID_LEN - 1);
    s->id[NEOPIXEL_ID_LEN - 1] = '\0';
    s->pin        = pin;
    s->count      = count;
    s->r = s->g = s->b = 0;
    s->brightness = 255;
    s->active     = true;
    render(s);   // start dark (color 0,0,0) — operator routes it on

    Serial.printf("NeoPixel: strip '%s' = %u px on pin %u\n",
                  id, (unsigned)count, (unsigned)pin);
    return true;
}

bool neopixel_strip_exists(const char *id)
{
    return find(id) != nullptr;
}

bool neopixel_strip_set_color(const char *id, uint32_t rgb)
{
    Strip *s = find(id);
    if (!s) return false;
    s->r = (uint8_t)((rgb >> 16) & 0xFF);
    s->g = (uint8_t)((rgb >>  8) & 0xFF);
    s->b = (uint8_t)( rgb        & 0xFF);
    render(s);
    return true;
}

bool neopixel_strip_set_brightness(const char *id, uint8_t brightness)
{
    Strip *s = find(id);
    if (!s) return false;
    s->brightness = brightness;
    render(s);
    return true;
}

void neopixel_strip_all_off(void)
{
    for (int i = 0; i < NEOPIXEL_MAX_STRIPS; i++) {
        if (g_strips[i].active && g_strips[i].px) {
            g_strips[i].px->clear();
            g_strips[i].px->show();
        }
    }
}

} // extern "C"

#else  // SIMULATION — no WS2812 hardware in Renode; log-only stubs.

extern "C" {

void neopixel_strip_reset(void) {}

bool neopixel_strip_add(const char *id, uint8_t pin, uint16_t count)
{
    Serial.printf("NeoPixel [SIM]: would add '%s' = %u px on pin %u\n",
                  id ? id : "?", (unsigned)count, (unsigned)pin);
    return true;
}

bool neopixel_strip_exists(const char *id) { (void)id; return false; }

bool neopixel_strip_set_color(const char *id, uint32_t rgb)
{
    Serial.printf("NeoPixel [SIM]: '%s' color=0x%06lX\n",
                  id ? id : "?", (unsigned long)(rgb & 0xFFFFFF));
    return true;
}

bool neopixel_strip_set_brightness(const char *id, uint8_t brightness)
{
    Serial.printf("NeoPixel [SIM]: '%s' brightness=%u\n",
                  id ? id : "?", (unsigned)brightness);
    return true;
}

void neopixel_strip_all_off(void) {}

} // extern "C"

#endif // SIMULATION
