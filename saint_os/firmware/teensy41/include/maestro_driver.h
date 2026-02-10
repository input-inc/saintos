/**
 * SAINT.OS Node Firmware - Pololu Maestro-24 Servo Controller Driver
 *
 * Communicates with the Maestro over USBHost_t36 serial using the
 * Pololu Compact Protocol. Provides 24 additional servo channels
 * mapped as virtual GPIO pins 200-223.
 */

#ifndef MAESTRO_DRIVER_H
#define MAESTRO_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Constants
// =============================================================================

#define MAESTRO_VIRTUAL_GPIO_BASE   200
#define MAESTRO_MAX_CHANNELS        24

// Default pulse widths (microseconds)
#define MAESTRO_DEFAULT_MIN_PULSE   992
#define MAESTRO_DEFAULT_MAX_PULSE   2000
#define MAESTRO_DEFAULT_NEUTRAL     1500

// =============================================================================
// Per-Channel Configuration
// =============================================================================

typedef struct {
    uint16_t min_pulse_us;      // Minimum pulse width (us)
    uint16_t max_pulse_us;      // Maximum pulse width (us)
    uint16_t neutral_us;        // Neutral/center pulse width (us)
    uint16_t speed;             // Speed limit (0 = unlimited)
    uint16_t acceleration;      // Acceleration limit (0 = none)
    uint16_t home_us;           // Home position pulse width (0 = use neutral)
} maestro_channel_config_t;

// =============================================================================
// Driver API
// =============================================================================

/**
 * Initialize the Maestro USB host driver.
 * Must be called after Arduino setup() begins.
 */
void maestro_init(void);

/**
 * Poll USBHost for device events. Call from loop().
 */
void maestro_update(void);

/**
 * Check if a Maestro is currently connected.
 */
bool maestro_is_connected(void);

/**
 * Set servo target position.
 * @param channel  Maestro channel 0-23
 * @param quarter_us  Target in quarter-microseconds (e.g., 6000 = 1500us)
 * @return true on success
 */
bool maestro_set_target(uint8_t channel, uint16_t quarter_us);

/**
 * Set channel speed limit.
 * @param channel  Maestro channel 0-23
 * @param speed    Speed limit (0 = unlimited)
 * @return true on success
 */
bool maestro_set_speed(uint8_t channel, uint16_t speed);

/**
 * Set channel acceleration limit.
 * @param channel  Maestro channel 0-23
 * @param accel    Acceleration limit (0 = none)
 * @return true on success
 */
bool maestro_set_acceleration(uint8_t channel, uint16_t accel);

/**
 * Read current servo position.
 * @param channel  Maestro channel 0-23
 * @return Position in quarter-microseconds, or 0 on error
 */
uint16_t maestro_get_position(uint8_t channel);

/**
 * Read error flags from the Maestro.
 * @return Error bitmask (see Pololu docs for bit meanings)
 */
uint16_t maestro_get_errors(void);

/**
 * Send all channels to their home positions.
 */
void maestro_go_home(void);

/**
 * Get the number of channels (always 24 for Maestro-24).
 */
uint8_t maestro_get_channel_count(void);

/**
 * Set per-channel configuration.
 */
void maestro_set_channel_config(uint8_t channel, const maestro_channel_config_t* config);

/**
 * Get per-channel configuration.
 */
const maestro_channel_config_t* maestro_get_channel_config(uint8_t channel);

/**
 * Convert angle (0-180 degrees) to quarter-microsecond target
 * using the channel's min/max pulse configuration.
 */
uint16_t maestro_angle_to_target(float angle, const maestro_channel_config_t* config);

#ifdef __cplusplus
}
#endif

#endif // MAESTRO_DRIVER_H
