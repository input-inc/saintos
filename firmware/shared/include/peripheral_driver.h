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
    uint8_t channel_count;          // Slab size: max channels the driver
                                    // can host (e.g. 40 = 8 units * 5
                                    // sub-channels for RoboClaw). Sets
                                    // the upper bound for how many
                                    // peripheral instances can stack.
    uint8_t channels_per_instance;  // Channels claimed by one peripheral
                                    // instance from the configure JSON
                                    // (e.g. 5 for one RoboClaw, 1 for
                                    // one SyRen, 4 for a FAS100). The
                                    // peripheral-first sync allocates
                                    // exactly this many channels per
                                    // entry — drivers no longer pre-
                                    // claim every channel in the slab.

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

    // Emergency stop.
    //   estop:       latch on  — assert e-stop (motors → 0, e-stop pins
    //                            driven into their asserted state, etc.)
    //   clear_estop: latch off — return outputs to their normal control
    //                            path. May be NULL on drivers where the
    //                            engaged action has no persistent side
    //                            effect (e.g. SyRen, FAS100).
    void (*estop)(void);
    void (*clear_estop)(void);

    // Flash persistence
    bool (*save_config)(void* storage);
    bool (*load_config)(const void* storage);

    // Peripheral-first state emission (optional). Append zero or
    // more channel-addressed records to `buf` as JSON objects of
    // the form {"peripheral_id":"...","channel_id":"...","value":<num>}.
    // The framework supplies a `first` flag so multiple drivers can
    // emit into the same shared `channels[]` array without colliding
    // on commas: each call to `peripheral_state_append_channel`
    // either consumes the slot (prepending a comma when *first is
    // false) or sets *first = false on first emit. Returns total
    // bytes written, or -1 on overflow. NULL means "this driver
    // hasn't migrated to peripheral-first state yet — its readings
    // still flow through the legacy GPIO-keyed pins[] array."
    // See docs/PERIPHERAL_FIRST_MIGRATION.md.
    int (*state_emit_channels)(char* buf, size_t cap, bool* first);
} peripheral_driver_t;

// =============================================================================
// Manager API
// =============================================================================

// 8 drivers ship today (maestro, syren, fas100, roboclaw, pathfinder_bms,
// tic, tmc2208, kangaroo) — exactly the old cap of 8. Bumped to 12 so the
// next driver doesn't silently fail peripheral_register(). Only costs a
// few pointers of .bss in the drivers[] table.
#define PERIPHERAL_MAX_DRIVERS 12

bool peripheral_register(const peripheral_driver_t* driver);
void peripheral_init_all(void);
void peripheral_update_all(void);
void peripheral_estop_all(void);
void peripheral_clear_estop_all(void);

const peripheral_driver_t* peripheral_find_by_gpio(uint16_t gpio);
const peripheral_driver_t* peripheral_find_by_mode(pin_mode_t mode);
const peripheral_driver_t* peripheral_find_by_mode_string(const char* mode_str);

// Helper that drivers' state_emit_channels callbacks use to append
// one channel record to the shared buffer. Handles the leading
// comma (skipped on the first record across the whole channels[]
// array, set automatically afterwards). Returns bytes written, or
// -1 on overflow. Implemented in peripheral_manager.cpp.
int peripheral_state_append_channel(char* buf, size_t cap, bool* first,
                                    const char* peripheral_id,
                                    const char* channel_id,
                                    float value);

// Emit every registered driver's channel-addressed state into the
// caller's buffer. Called from per-platform pin_control_state_to_json
// after the legacy pins[] array. Returns bytes written, or -1 on
// overflow. The caller is responsible for the surrounding
// "channels":[ ... ] brackets.
int peripheral_state_emit_all_channels(char* buf, size_t cap);
uint8_t peripheral_gpio_to_channel(const peripheral_driver_t* drv, uint16_t gpio);
bool peripheral_is_virtual_gpio(uint16_t gpio);

uint8_t peripheral_get_count(void);
const peripheral_driver_t* peripheral_get(uint8_t index);

#ifdef __cplusplus
}
#endif

#endif // PERIPHERAL_DRIVER_H
