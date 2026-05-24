/**
 * SAINT.OS Node Firmware - RP2040 Platform Abstraction
 */

#ifndef PLATFORM_H
#define PLATFORM_H

#include "pico/stdlib.h"

#define PLATFORM_SLEEP_MS(ms)  sleep_ms(ms)
#define PLATFORM_MILLIS()      to_ms_since_boot(get_absolute_time())
#define PLATFORM_PRINTF        printf

#endif // PLATFORM_H
