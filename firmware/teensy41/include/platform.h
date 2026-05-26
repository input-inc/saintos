/**
 * SAINT.OS Node Firmware - Teensy 4.1 Platform Abstraction
 */

#ifndef PLATFORM_H
#define PLATFORM_H

#include <Arduino.h>

#define PLATFORM_SLEEP_MS(ms)  delay(ms)
#define PLATFORM_MILLIS()      ((uint32_t)millis())
#define PLATFORM_PRINTF        Serial.printf

#endif // PLATFORM_H
