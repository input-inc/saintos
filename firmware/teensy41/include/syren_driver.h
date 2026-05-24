/**
 * SAINT.OS Node Firmware - SyRen 50 Motor Controller Driver (Teensy 4.1)
 *
 * Drives up to 8 Dimension Engineering SyRen 50 motor controllers
 * over packetized serial (addresses 128-135). Uses HardwareSerial.
 */

#ifndef SYREN_DRIVER_H
#define SYREN_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

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

/**
 * Initialize the SyRen driver.
 * Configures the serial port and sends autobaud byte.
 */
void syren_init(void);

/**
 * Poll / update. Call from loop().
 * Currently a no-op for SyRen (no async state to poll).
 */
void syren_update(void);

/**
 * Check if the SyRen serial port is ready.
 * Returns true if the serial port has been initialized.
 */
bool syren_is_connected(void);

/**
 * Set motor power.
 * @param channel  SyRen channel 0-7 (maps to address 128-135)
 * @param power    Motor power: -127 (full reverse) to +127 (full forward)
 * @return true on success
 */
bool syren_set_power(uint8_t channel, int16_t power);

/**
 * Stop a single channel (set power to 0).
 */
bool syren_stop(uint8_t channel);

/**
 * Emergency stop all configured channels.
 */
void syren_stop_all(void);

/**
 * Set per-channel configuration.
 */
void syren_set_channel_config(uint8_t channel, const syren_channel_config_t* config);

/**
 * Get per-channel configuration.
 */
const syren_channel_config_t* syren_get_channel_config(uint8_t channel);

/**
 * Get the peripheral driver interface for the SyRen.
 * Used for registration with the peripheral manager.
 */
struct peripheral_driver;
const struct peripheral_driver* syren_get_peripheral_driver(void);

#ifdef __cplusplus
}
#endif

#endif // SYREN_DRIVER_H
