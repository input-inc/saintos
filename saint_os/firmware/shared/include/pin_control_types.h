/**
 * SAINT.OS Firmware - Shared Pin Control Types
 *
 * Runtime pin value structures and control API declarations.
 */

#ifndef PIN_CONTROL_TYPES_H
#define PIN_CONTROL_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// =============================================================================
// Runtime Value Structures
// =============================================================================

typedef struct {
    uint8_t gpio;
    float value;
    uint32_t last_updated;
} pin_runtime_value_t;

// =============================================================================
// Control Functions
// =============================================================================

void pin_control_init(void);
bool pin_control_set_value(uint8_t gpio, float value);
bool pin_control_set_pwm(uint8_t gpio, float percent);
bool pin_control_set_servo(uint8_t gpio, float angle);
bool pin_control_set_digital(uint8_t gpio, bool state);
bool pin_control_read_adc(uint8_t gpio, uint16_t* raw_value, float* voltage);
bool pin_control_read_digital(uint8_t gpio);
bool pin_control_get_value(uint8_t gpio, float* value);
bool pin_control_apply_json(const char* json, size_t json_len);
int pin_control_state_to_json(char* buffer, size_t buffer_size, const char* node_id);
void pin_control_update_state(void);
void pin_control_estop(void);

#endif // PIN_CONTROL_TYPES_H
