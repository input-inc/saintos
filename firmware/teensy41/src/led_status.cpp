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

/* Operator-set override. While active, led_update() honors
 * override_on rather than the state-driven pattern. */
static bool g_override_active = false;
static bool g_override_on     = false;

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
    /* Operator override beats state indication. The pin is set
     * directly each tick so a stale state-driven write between
     * override toggles doesn't peek through. */
    if (g_override_active) {
        digitalWrite(LED_PIN, g_override_on ? HIGH : LOW);
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
    /* Single-color LED can't honor hue. Collapse to on/off: ON when
     * the operator asked for any non-black color at non-zero
     * brightness; OFF otherwise. The color picker in the dashboard
     * is still useful — picking black is a clean "off". */
    g_override_active = true;
    g_override_on     = (brightness > 0) && ((r | g | b) != 0);
}

void led_clear_override(void)
{
    g_override_active = false;
    g_override_on     = false;
}

bool led_override_active(void)
{
    return g_override_active;
}

} // extern "C"
