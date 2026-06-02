/**
 * SAINT.OS Node Firmware - Teensy 4.1 Hardware Abstraction
 *
 * Provides unique ID from OCOTP fuses and CPU temperature
 * from the built-in temperature monitor.
 */

#include <Arduino.h>
#include "platform.h"
#include <stdio.h>
#include <string.h>

extern "C" {
#include "saint_node.h"
}

// Teensy 4.1 built-in temperature monitor
extern float tempmonGetTemp(void);

// Teensy 4.1 unique ID (from OCOTP fuse bank)
//extern "C" uint32_t HW_OCOTP_MAC0;
//extern "C" uint32_t HW_OCOTP_MAC1;

extern "C" {

void hardware_init(void)
{
    // Initialize onboard LED
    pinMode(LED_PIN, OUTPUT);

    Serial.printf("Hardware initialized\n");
}

void hardware_update(void)
{
    // Nothing to do currently
}

uint32_t hardware_get_unique_id(char* buffer, size_t len)
{
    // Use Teensy's unique chip ID from OCOTP fuses
    uint32_t uid[2];
    uid[0] = HW_OCOTP_MAC0;
    uid[1] = HW_OCOTP_MAC1;

    int written = snprintf(buffer, len, "%08lx%08lx",
                           (unsigned long)uid[0], (unsigned long)uid[1]);

    return (written > 0) ? written : 0;
}

float hardware_get_cpu_temp(void)
{
#ifdef SIMULATION
    // Renode doesn't model the iMXRT1062 temperature monitor (TEMPMON at
    // 0x400D8000), so the ADC reads back zero and tempmonGetTemp()'s
    // calibration math divides by zero → NaN. `snprintf("%.1f", NaN)`
    // prints the literal "nan", which then sails out to the server as
    // part of the announcement JSON and crashes json.loads at the cpu_temp
    // field. Returning a real (if synthetic) value keeps the JSON valid.
    return 0.0f;
#else
    return tempmonGetTemp();
#endif
}

} // extern "C"
