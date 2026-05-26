/**
 * SAINT.OS Node Firmware - Teensy 4.1 LED Status
 *
 * Simple digital LED on pin 13 (onboard LED).
 * No NeoPixel - uses direct digital output for state indication.
 */

#include <Arduino.h>

extern "C" {
#include "saint_node.h"
}

static node_state_t current_state = NODE_STATE_BOOT;

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

} // extern "C"
