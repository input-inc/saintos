/**
 * SAINT.OS Firmware - Shared Pin Types
 *
 * Platform-agnostic pin capability flags, mode enums, and configuration structures.
 */

#ifndef PIN_TYPES_H
#define PIN_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// =============================================================================
// Pin Capability Flags
// =============================================================================

#define PIN_CAP_NONE            0x00
#define PIN_CAP_DIGITAL_IN      0x01
#define PIN_CAP_DIGITAL_OUT     0x02
#define PIN_CAP_PWM             0x04
#define PIN_CAP_ADC             0x08
#define PIN_CAP_I2C_SDA         0x10
#define PIN_CAP_I2C_SCL         0x20
#define PIN_CAP_UART_TX         0x40
#define PIN_CAP_UART_RX         0x80
#define PIN_CAP_SPI             0x100
#define PIN_CAP_MAESTRO_SERVO   0x200
#define PIN_CAP_SYREN_MOTOR     0x400
#define PIN_CAP_FAS100_SENSOR   0x800

// Convenience combinations
#define PIN_CAP_GPIO            (PIN_CAP_DIGITAL_IN | PIN_CAP_DIGITAL_OUT)
#define PIN_CAP_GPIO_PWM        (PIN_CAP_GPIO | PIN_CAP_PWM)
#define PIN_CAP_GPIO_ADC        (PIN_CAP_GPIO | PIN_CAP_ADC)

// =============================================================================
// Pin Mode Enumeration
// =============================================================================

typedef enum {
    PIN_MODE_UNCONFIGURED = 0,
    PIN_MODE_DIGITAL_IN,
    PIN_MODE_DIGITAL_OUT,
    PIN_MODE_PWM,
    PIN_MODE_SERVO,
    PIN_MODE_ADC,
    PIN_MODE_I2C_SDA,
    PIN_MODE_I2C_SCL,
    PIN_MODE_UART_TX,
    PIN_MODE_UART_RX,
    PIN_MODE_RESERVED,
    PIN_MODE_MAESTRO_SERVO,
    PIN_MODE_SYREN_MOTOR,
    PIN_MODE_FAS100_SENSOR
} pin_mode_t;

// =============================================================================
// Configuration Constants
// =============================================================================

#define PIN_CONFIG_MAX_NAME_LEN 32
#define PIN_CONFIG_VERSION      1

// Default PWM settings
#define PIN_CONFIG_DEFAULT_PWM_FREQ     1000    // 1 kHz
#define PIN_CONFIG_SERVO_PWM_FREQ       50      // 50 Hz for servos

// Default servo pulse range
#define PIN_CONFIG_SERVO_MIN_PULSE_US   500     // 0.5ms
#define PIN_CONFIG_SERVO_MAX_PULSE_US   2500    // 2.5ms

// =============================================================================
// Pin Definition Structure
// =============================================================================

typedef struct {
    uint8_t gpio;
    char name[8];
    uint16_t capabilities;
} pin_definition_t;

// =============================================================================
// Pin Configuration Structure
// =============================================================================

typedef struct {
    uint8_t gpio;
    pin_mode_t mode;
    char logical_name[PIN_CONFIG_MAX_NAME_LEN];
    union {
        struct {
            uint32_t frequency;
            uint16_t duty_cycle;
        } pwm;
        struct {
            uint32_t frequency;     // Always 50Hz for servo
            uint16_t min_pulse_us;  // Default 500 (0.5ms)
            uint16_t max_pulse_us;  // Default 2500 (2.5ms)
        } servo;
        struct {
            bool pull_up;
            bool pull_down;
        } digital_in;
        struct {
            bool initial_state;
        } digital_out;
        struct {
            uint16_t min_pulse_us;
            uint16_t max_pulse_us;
            uint16_t neutral_us;
            uint16_t speed;
            uint16_t acceleration;
            uint16_t home_us;
        } maestro;
        struct {
            uint8_t address;
            uint8_t deadband;
            uint8_t ramping;
            uint8_t reserved;
            uint16_t timeout_ms;
        } syren;
        struct {
            uint8_t poll_interval_ms;
        } fas100;
    } params;
} pin_config_t;

// =============================================================================
// Pin Configuration Storage (for flash persistence)
// =============================================================================

typedef struct __attribute__((packed)) {
    uint8_t version;
    uint8_t pin_count;
    uint8_t reserved[2];
    struct __attribute__((packed)) {
        uint8_t gpio;
        uint8_t mode;
        char logical_name[PIN_CONFIG_MAX_NAME_LEN];
        uint32_t param1;
        uint16_t param2;
        uint8_t reserved[2];
    } pins[16];     // Uses inline constant to avoid circular dependency with per-platform MAX_PINS
} pin_config_storage_t;

// =============================================================================
// Node Capabilities Structure
// =============================================================================

typedef struct {
    const pin_definition_t* available_pins;
    uint8_t available_count;
    const uint8_t* reserved_pins;
    uint8_t reserved_count;
} node_pin_capabilities_t;

// =============================================================================
// Function Declarations (per-platform implementations)
// =============================================================================

void pin_config_init(void);
const node_pin_capabilities_t* pin_config_get_capabilities(void);
const pin_config_t* pin_config_get(uint8_t gpio);
const pin_config_t* pin_config_get_all(uint8_t* count);
int pin_config_capabilities_to_json(char* buffer, size_t buffer_size, const char* node_id);
bool pin_config_apply_json(const char* json, size_t json_len);
bool pin_config_save(void);
bool pin_config_load(void);
void pin_config_reset(void);
bool pin_config_has_configured_pins(void);
bool pin_config_set(uint8_t gpio, pin_mode_t mode, const char* logical_name);
bool pin_config_set_pwm_params(uint8_t gpio, uint32_t frequency, uint16_t duty_cycle);
bool pin_config_set_digital_in_params(uint8_t gpio, bool pull_up, bool pull_down);
bool pin_config_set_servo_params(uint8_t gpio, uint16_t min_pulse_us, uint16_t max_pulse_us);
bool pin_config_set_maestro_params(uint8_t gpio, uint16_t min_pulse_us, uint16_t max_pulse_us,
                                    uint16_t neutral_us, uint16_t speed, uint16_t acceleration,
                                    uint16_t home_us);
bool pin_config_set_fas100_params(uint8_t gpio, uint8_t poll_interval_ms);
void pin_config_apply_hardware(void);
const char* pin_mode_to_string(pin_mode_t mode);
pin_mode_t pin_mode_from_string(const char* str);

#endif // PIN_TYPES_H
