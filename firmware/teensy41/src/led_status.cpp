/**
 * SAINT.OS Node Firmware - Teensy 4.1 LED Status
 *
 * Onboard LED on pin 13 (LED_BUILTIN). Used for two layered purposes:
 *
 *   1) Status indication — boot / connecting / adopting / active / error
 *      patterns from led_update() driven by node_state_t.
 *   2) Operator override — when the dashboard sends a set_neopixel
 *      action or set_channel write to the onboard LED peripheral,
 *      led_set_override_color() takes the LED over and led_update()
 *      stops driving the status pattern until led_clear_override().
 *
 * The override API matches the shared saint_types.h signature used by
 * the RP2040 Feather's WS2812 NeoPixel so the dashboard can speak the
 * same wire protocol to either platform. RGB collapses to on/off here:
 * any non-black color at brightness > 0 turns the pin HIGH, brightness
 * 0 (or all-black) turns it LOW. The operator still picks a color in
 * the UI; the Teensy just doesn't have hardware to honor it.
 */

#include <Arduino.h>
#include "platform.h"

extern "C" {
#include "saint_node.h"
}

static node_state_t current_state = NODE_STATE_BOOT;

/* Operator-set override. While active, led_update() drives the LED
 * to brightness-scaled-by-color rather than the state-driven pattern.
 *
 * Color is tracked (even though the LED is single-channel) so the
 * dashboard's color picker and brightness slider can move
 * independently: setting color first then brightness later should
 * still produce the right output, and vice-versa. The actual pin
 * write collapses (r,g,b) at brightness > 0 to ON, brightness 0 (or
 * all-zero color) to OFF. */
static bool    g_override_active     = false;
static uint8_t g_override_r          = 0;
static uint8_t g_override_g          = 0;
static uint8_t g_override_b          = 0;
static uint8_t g_override_brightness = 0;

extern "C" {

void led_init(void)
{
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    Serial.printf("LED initialized\n");
}

void led_set_state(node_state_t state)
{
    current_state = state;
}

void led_identify(uint8_t flash_count)
{
    if (flash_count == 0) flash_count = 5;

    Serial.printf("LED identify: flashing %d times\n", flash_count);

    node_state_t saved_state = current_state;

    for (uint8_t i = 0; i < flash_count; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
        // Quick double-flash
        digitalWrite(LED_PIN, HIGH);
        delay(50);
        digitalWrite(LED_PIN, LOW);
        delay(50);
    }

    current_state = saved_state;
}

void led_update(void)
{
    /* Operator override beats state indication. The pin is repainted
     * every tick so a stray state-driven write between override toggles
     * doesn't peek through.
     *
     * Brightness uses analogWrite — Teensy 4.1 pin 13 (LED_BUILTIN)
     * is PWM-capable via FlexPWM at ~4.5 kHz default, well above the
     * eye's flicker threshold, so the brightness slider produces real
     * dimming instead of just on/off. Color collapses to a gate: any
     * non-black color enables the override-driven brightness; a black
     * color forces 0 regardless of the brightness value.
     *
     * (If a future Teensy variant ever lands a non-PWM-capable LED
     * pin, swap analogWrite for digitalWrite(LED_PIN, dim ? HIGH : LOW)
     * — the channel API and stored state stay identical.) */
    if (g_override_active) {
        bool color_on = (g_override_r | g_override_g | g_override_b) != 0;
        uint8_t dim = color_on ? g_override_brightness : 0;
        analogWrite(LED_PIN, dim);
        return;
    }

    uint32_t now = millis();

    switch (current_state) {
        case NODE_STATE_BOOT:
        case NODE_STATE_ACTIVE:
            // Solid on
            digitalWrite(LED_PIN, HIGH);
            break;

        case NODE_STATE_CONNECTING:
        case NODE_STATE_UNADOPTED:
            // Blink at 2Hz
            digitalWrite(LED_PIN, (now / 250) % 2);
            break;

        case NODE_STATE_ADOPTING:
            // Fast blink at 5Hz
            digitalWrite(LED_PIN, (now / 100) % 2);
            break;

        case NODE_STATE_ERROR:
            // Slow blink at 0.5Hz
            digitalWrite(LED_PIN, (now / 1000) % 2);
            break;
    }
}

void led_set_override_color(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
    g_override_r          = r;
    g_override_g          = g;
    g_override_b          = b;
    g_override_brightness = brightness;
    g_override_active     = true;
}

void led_set_override_brightness(uint8_t brightness)
{
    /* Brightness slider moves independently of color picker. If the
     * operator hasn't set a color yet, seed white so a brightness-
     * only write produces a visible result instead of dim-black.
     * Same shape as the RP2040's apply_neopixel_channel brightness
     * path. */
    if (g_override_r == 0 && g_override_g == 0 && g_override_b == 0) {
        g_override_r = 255;
        g_override_g = 255;
        g_override_b = 255;
    }
    g_override_brightness = brightness;
    g_override_active     = true;
}

void led_clear_override(void)
{
    g_override_active     = false;
    g_override_brightness = 0;
    /* Don't reset r/g/b — keep the operator's last-picked color so
     * re-engaging an override picks up where they left off. */
}

bool led_override_active(void)
{
    return g_override_active;
}

} // extern "C"
