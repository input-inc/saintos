/**
 * SAINT.OS Firmware - Peripheral Driver Interface
 *
 * Modular driver interface for sub-node peripherals (Maestro, SyRen, etc.)
 * Each peripheral implements this interface and registers with the manager.
 * Pin config/control code dispatches to registered drivers by GPIO range.
 */

#ifndef PERIPHERAL_DRIVER_H
#define PERIPHERAL_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "pin_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Driver Interface
// =============================================================================

typedef struct peripheral_driver {
    const char* name;               // "maestro", "syren"
    const char* mode_string;        // "maestro_servo", "syren_motor"
    pin_mode_t pin_mode;            // PIN_MODE_MAESTRO_SERVO, etc.
    uint32_t capability_flag;       // PIN_CAP_MAESTRO_SERVO, etc.
    uint16_t virtual_gpio_base;     // 200, 224, 276, etc.
    uint8_t channel_count;          // 24, 8, etc.

    // Lifecycle
    bool (*init)(void);
    void (*update)(void);
    bool (*is_connected)(void);

    // Control
    bool (*set_value)(uint8_t channel, float value);
    bool (*get_value)(uint8_t channel, float* value);

    // Configuration
    void (*set_defaults)(uint8_t channel, pin_config_t* config);
    bool (*apply_config)(uint8_t channel, const pin_config_t* config);
    bool (*parse_json_params)(const char* json_start, const char* json_end,
                              pin_config_t* config);

    // Emergency stop
    void (*estop)(void);

    // Capabilities JSON fragment (returns bytes written, -1 on error)
    int (*capabilities_to_json)(uint8_t channel, char* buf, size_t remaining);

    // Flash persistence
    bool (*save_config)(void* storage);
    bool (*load_config)(const void* storage);
} peripheral_driver_t;

// =============================================================================
// Manager API
// =============================================================================

#define PERIPHERAL_MAX_DRIVERS 8

bool peripheral_register(const peripheral_driver_t* driver);
void peripheral_init_all(void);
void peripheral_update_all(void);
void peripheral_estop_all(void);

const peripheral_driver_t* peripheral_find_by_gpio(uint16_t gpio);
const peripheral_driver_t* peripheral_find_by_mode(pin_mode_t mode);
const peripheral_driver_t* peripheral_find_by_mode_string(const char* mode_str);
uint8_t peripheral_gpio_to_channel(const peripheral_driver_t* drv, uint16_t gpio);
bool peripheral_is_virtual_gpio(uint16_t gpio);

uint8_t peripheral_get_count(void);
const peripheral_driver_t* peripheral_get(uint8_t index);

#ifdef __cplusplus
}
#endif

#endif // PERIPHERAL_DRIVER_H
