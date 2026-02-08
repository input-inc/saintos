/**
 * SAINT.OS Node Firmware - Pin Control
 *
 * Runtime control of pin values (PWM duty cycle, servo angle, digital output).
 * Separate from pin_config which handles pin mode assignment.
 */

#ifndef PIN_CONTROL_H
#define PIN_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// =============================================================================
// Runtime Value Structures
// =============================================================================

/**
 * Runtime value for a single pin.
 * Stores the current commanded value and actual output.
 */
typedef struct {
    uint8_t gpio;           // GPIO number
    float value;            // Current value (0-100 for PWM, 0-180 for servo, 0/1 for digital)
    uint32_t last_updated;  // Timestamp of last update (ms)
} pin_runtime_value_t;

// =============================================================================
// Control Functions
// =============================================================================

/**
 * Initialize pin control subsystem.
 * Should be called after pin_config_init() and pin_config_load().
 */
void pin_control_init(void);

/**
 * Set value for a pin (dispatcher by mode).
 *
 * @param gpio GPIO number
 * @param value Value to set:
 *              - PWM: 0-100 (percent duty cycle)
 *              - Servo: 0-180 (degrees)
 *              - Digital Out: 0 or 1
 * @return true if value was set successfully
 */
bool pin_control_set_value(uint8_t gpio, float value);

/**
 * Set PWM duty cycle.
 *
 * @param gpio GPIO number (must be configured as PWM)
 * @param percent Duty cycle percentage (0-100)
 * @return true if successful
 */
bool pin_control_set_pwm(uint8_t gpio, float percent);

/**
 * Set servo position.
 *
 * @param gpio GPIO number (must be configured as servo)
 * @param angle Angle in degrees (0-180)
 * @return true if successful
 */
bool pin_control_set_servo(uint8_t gpio, float angle);

/**
 * Set digital output state.
 *
 * @param gpio GPIO number (must be configured as digital_out)
 * @param state Output state (true = high, false = low)
 * @return true if successful
 */
bool pin_control_set_digital(uint8_t gpio, bool state);

/**
 * Read ADC value.
 *
 * @param gpio GPIO number (must be configured as ADC, GPIO 26-29)
 * @param[out] raw_value Raw ADC value (0-4095)
 * @param[out] voltage Voltage value (0-3.3V)
 * @return true if successful
 */
bool pin_control_read_adc(uint8_t gpio, uint16_t* raw_value, float* voltage);

/**
 * Read digital input state.
 *
 * @param gpio GPIO number (must be configured as digital_in)
 * @return true if high, false if low
 */
bool pin_control_read_digital(uint8_t gpio);

/**
 * Get current value for a pin.
 *
 * @param gpio GPIO number
 * @param[out] value Current value (meaning depends on mode)
 * @return true if pin has a stored value
 */
bool pin_control_get_value(uint8_t gpio, float* value);

// =============================================================================
// JSON Interface
// =============================================================================

/**
 * Apply control command from JSON.
 *
 * Expected format:
 * {"action":"set_pin","gpio":5,"value":75.0}
 *
 * @param json JSON string
 * @param json_len Length of JSON string
 * @return true if command was applied
 */
bool pin_control_apply_json(const char* json, size_t json_len);

/**
 * Serialize current pin state to JSON.
 *
 * Output format:
 * {"node_id":"xxx","timestamp":123456,"pins":[{"gpio":5,"mode":"pwm","value":75.0,"name":"motor1"},...]}"
 *
 * @param buffer Output buffer
 * @param buffer_size Buffer size
 * @param node_id Node ID to include
 * @return Number of bytes written, or -1 on error
 */
int pin_control_state_to_json(char* buffer, size_t buffer_size, const char* node_id);

/**
 * Update all pin states (read ADC values, etc.).
 * Should be called periodically before publishing state.
 */
void pin_control_update_state(void);

/**
 * Emergency stop - set all output pins to safe values.
 * Sets all PWM and servo outputs to 0%, digital outputs to low.
 */
void pin_control_estop(void);

#endif // PIN_CONTROL_H
