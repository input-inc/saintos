/**
 * SAINT.OS Node Firmware - SyRen 50 Motor Controller Driver (RP2040)
 *
 * Drives up to 8 Dimension Engineering SyRen 50 motor controllers
 * over packetized serial (addresses 128-135). Uses Pico SDK UART.
 */

#ifndef SYREN_DRIVER_H
#define SYREN_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "syren_protocol.h"

// =============================================================================
// Per-Channel Configuration
// =============================================================================

typedef struct {
    uint8_t address;        // 128-135
    uint8_t deadband;       // 0-127 (0 = use device default of 3)
    uint8_t ramping;        // 0-80 (acceleration profile, 0 = none)
    uint16_t timeout_ms;    // Serial timeout (0 = disabled)
} syren_channel_config_t;

// =============================================================================
// Driver API
// =============================================================================

void syren_init(void);
void syren_update(void);
bool syren_is_connected(void);

bool syren_set_power(uint8_t channel, int16_t power);
bool syren_stop(uint8_t channel);
void syren_stop_all(void);

void syren_set_channel_config(uint8_t channel, const syren_channel_config_t* config);
const syren_channel_config_t* syren_get_channel_config(uint8_t channel);

struct peripheral_driver;
const struct peripheral_driver* syren_get_peripheral_driver(void);

#endif // SYREN_DRIVER_H
