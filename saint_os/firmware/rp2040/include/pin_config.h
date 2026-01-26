/**
 * SAINT.OS Node Firmware - Pin Configuration
 *
 * Defines pin capabilities, configuration structures, and management functions
 * for runtime GPIO configuration.
 */

#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// =============================================================================
// Pin Capability Flags
// =============================================================================

#define PIN_CAP_NONE            0x00
#define PIN_CAP_DIGITAL_IN      0x01    // Can be used as digital input
#define PIN_CAP_DIGITAL_OUT     0x02    // Can be used as digital output
#define PIN_CAP_PWM             0x04    // Can output PWM signal
#define PIN_CAP_ADC             0x08    // Has ADC capability
#define PIN_CAP_I2C_SDA         0x10    // Can be I2C data line
#define PIN_CAP_I2C_SCL         0x20    // Can be I2C clock line
#define PIN_CAP_UART_TX         0x40    // Can be UART transmit
#define PIN_CAP_UART_RX         0x80    // Can be UART receive
#define PIN_CAP_SPI             0x100   // Can be used for SPI

// Convenience combinations
#define PIN_CAP_GPIO            (PIN_CAP_DIGITAL_IN | PIN_CAP_DIGITAL_OUT)
#define PIN_CAP_GPIO_PWM        (PIN_CAP_GPIO | PIN_CAP_PWM)
#define PIN_CAP_GPIO_ADC        (PIN_CAP_GPIO | PIN_CAP_ADC)

// =============================================================================
// Pin Mode Enumeration
// =============================================================================

typedef enum {
    PIN_MODE_UNCONFIGURED = 0,  // Pin not configured
    PIN_MODE_DIGITAL_IN,        // Digital input
    PIN_MODE_DIGITAL_OUT,       // Digital output
    PIN_MODE_PWM,               // PWM output
    PIN_MODE_SERVO,             // Servo output (50Hz PWM)
    PIN_MODE_ADC,               // Analog input
    PIN_MODE_I2C_SDA,           // I2C data
    PIN_MODE_I2C_SCL,           // I2C clock
    PIN_MODE_UART_TX,           // UART transmit
    PIN_MODE_UART_RX,           // UART receive
    PIN_MODE_RESERVED           // Reserved by system (ethernet, LED, etc.)
} pin_mode_t;

// =============================================================================
// Configuration Constants
// =============================================================================

#define PIN_CONFIG_MAX_PINS     16      // Maximum configurable pins
#define PIN_CONFIG_MAX_NAME_LEN 32      // Maximum logical name length
#define PIN_CONFIG_VERSION      1       // Configuration format version

// Default PWM settings
#define PIN_CONFIG_DEFAULT_PWM_FREQ     1000    // 1 kHz
#define PIN_CONFIG_SERVO_PWM_FREQ       50      // 50 Hz for servos

// =============================================================================
// Pin Definition Structure
// =============================================================================

/**
 * Defines a single physical pin's capabilities.
 */
typedef struct {
    uint8_t gpio;               // GPIO number
    char name[8];               // Short name (e.g., "D5", "A0")
    uint16_t capabilities;      // Bitmask of PIN_CAP_* flags
} pin_definition_t;

// =============================================================================
// Pin Configuration Structure
// =============================================================================

/**
 * Runtime configuration for a single pin.
 */
typedef struct {
    uint8_t gpio;                               // GPIO number
    pin_mode_t mode;                            // Current mode
    char logical_name[PIN_CONFIG_MAX_NAME_LEN]; // Logical function name
    union {
        struct {
            uint32_t frequency;     // PWM frequency in Hz
            uint16_t duty_cycle;    // Duty cycle (0-65535)
        } pwm;
        struct {
            bool pull_up;           // Enable pull-up resistor
            bool pull_down;         // Enable pull-down resistor
        } digital_in;
        struct {
            bool initial_state;     // Initial output state
        } digital_out;
    } params;
} pin_config_t;

// =============================================================================
// Pin Configuration Storage (for flash persistence)
// =============================================================================

/**
 * Flash storage format for pin configurations.
 * This is stored alongside the main flash_storage_data_t.
 */
typedef struct __attribute__((packed)) {
    uint8_t version;                                // Configuration version
    uint8_t pin_count;                              // Number of configured pins
    uint8_t reserved[2];                            // Padding for alignment
    struct __attribute__((packed)) {
        uint8_t gpio;                               // GPIO number
        uint8_t mode;                               // pin_mode_t value
        char logical_name[PIN_CONFIG_MAX_NAME_LEN]; // Logical function name
        uint32_t param1;                            // Mode-specific param (e.g., PWM freq)
        uint16_t param2;                            // Mode-specific param (e.g., duty cycle)
        uint8_t reserved[2];                        // Padding
    } pins[PIN_CONFIG_MAX_PINS];
} pin_config_storage_t;

// =============================================================================
// Node Capabilities Structure (for reporting to server)
// =============================================================================

/**
 * Complete pin capabilities for the node.
 */
typedef struct {
    const pin_definition_t* available_pins;     // Array of available pins
    uint8_t available_count;                    // Number of available pins
    const uint8_t* reserved_pins;               // Array of reserved GPIO numbers
    uint8_t reserved_count;                     // Number of reserved pins
} node_pin_capabilities_t;

// =============================================================================
// Function Declarations
// =============================================================================

/**
 * Initialize pin configuration subsystem.
 * Sets up available pins based on hardware.
 */
void pin_config_init(void);

/**
 * Get the node's pin capabilities.
 * @return Pointer to capabilities structure
 */
const node_pin_capabilities_t* pin_config_get_capabilities(void);

/**
 * Get the current pin configuration.
 * @param gpio GPIO number to query
 * @return Pointer to pin config, or NULL if not configured
 */
const pin_config_t* pin_config_get(uint8_t gpio);

/**
 * Get all current pin configurations.
 * @param count Output parameter for number of configurations
 * @return Array of pin configurations
 */
const pin_config_t* pin_config_get_all(uint8_t* count);

/**
 * Serialize capabilities to JSON string.
 * @param buffer Output buffer
 * @param buffer_size Buffer size
 * @param node_id Node ID string to include
 * @return Number of bytes written, or -1 on error
 */
int pin_config_capabilities_to_json(char* buffer, size_t buffer_size, const char* node_id);

/**
 * Apply pin configuration from JSON string.
 * @param json JSON configuration string
 * @param json_len Length of JSON string
 * @return true if configuration applied successfully
 */
bool pin_config_apply_json(const char* json, size_t json_len);

/**
 * Save current pin configuration to flash.
 * @return true if save successful
 */
bool pin_config_save(void);

/**
 * Load pin configuration from flash.
 * @return true if valid configuration was loaded
 */
bool pin_config_load(void);

/**
 * Reset pin configuration to defaults (all unconfigured).
 */
void pin_config_reset(void);

/**
 * Check if any pins are configured (non-UNCONFIGURED mode).
 * Used to determine if node was previously adopted.
 * @return true if at least one pin has a mode other than UNCONFIGURED
 */
bool pin_config_has_configured_pins(void);

/**
 * Configure a single pin.
 * @param gpio GPIO number
 * @param mode Desired mode
 * @param logical_name Logical function name (can be NULL)
 * @return true if configuration successful
 */
bool pin_config_set(uint8_t gpio, pin_mode_t mode, const char* logical_name);

/**
 * Set PWM parameters for a configured PWM pin.
 * @param gpio GPIO number (must be configured as PWM)
 * @param frequency PWM frequency in Hz
 * @param duty_cycle Duty cycle (0-65535)
 * @return true if parameters set successfully
 */
bool pin_config_set_pwm_params(uint8_t gpio, uint32_t frequency, uint16_t duty_cycle);

/**
 * Set digital input parameters.
 * @param gpio GPIO number (must be configured as digital input)
 * @param pull_up Enable pull-up resistor
 * @param pull_down Enable pull-down resistor
 * @return true if parameters set successfully
 */
bool pin_config_set_digital_in_params(uint8_t gpio, bool pull_up, bool pull_down);

/**
 * Apply hardware configuration for all configured pins.
 * Called after loading config or applying changes.
 */
void pin_config_apply_hardware(void);

/**
 * Get string representation of pin mode.
 * @param mode Pin mode
 * @return Mode name string
 */
const char* pin_mode_to_string(pin_mode_t mode);

/**
 * Parse pin mode from string.
 * @param str Mode string
 * @return Pin mode, or PIN_MODE_UNCONFIGURED if unknown
 */
pin_mode_t pin_mode_from_string(const char* str);

#endif // PIN_CONFIG_H
