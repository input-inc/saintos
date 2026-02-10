/**
 * SAINT.OS Firmware - Shared Flash Storage Types
 *
 * Platform-agnostic storage structures, magic numbers, and helper function declarations.
 */

#ifndef FLASH_TYPES_H
#define FLASH_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "saint_types.h"

// =============================================================================
// Storage Configuration
// =============================================================================

#define FLASH_STORAGE_MAGIC     0x53414E54  // "SANT"
#define FLASH_STORAGE_VERSION   5

#define FLASH_PIN_CONFIG_MAX_PINS     16
#define FLASH_PIN_CONFIG_MAX_NAME_LEN 32
#define FLASH_PIN_CONFIG_VERSION      1

// =============================================================================
// Pin Configuration Storage Structure
// =============================================================================

typedef struct __attribute__((packed)) {
    uint8_t version;
    uint8_t pin_count;
    uint8_t reserved_hdr[2];
    struct __attribute__((packed)) {
        uint8_t gpio;
        uint8_t mode;
        char logical_name[FLASH_PIN_CONFIG_MAX_NAME_LEN];
        uint32_t param1;
        uint16_t param2;
        uint8_t reserved_pin[2];
    } pins[FLASH_PIN_CONFIG_MAX_PINS];
} flash_pin_config_t;

// =============================================================================
// Maestro Servo Controller Configuration
// =============================================================================

#define FLASH_MAESTRO_MAX_CHANNELS 24

typedef struct __attribute__((packed)) {
    uint8_t channel_count;
    uint8_t reserved_m[3];
    struct __attribute__((packed)) {
        uint16_t min_pulse_us;
        uint16_t max_pulse_us;
        uint16_t neutral_us;
        uint16_t speed;
        uint16_t acceleration;
        uint16_t home_us;
    } channels[FLASH_MAESTRO_MAX_CHANNELS];
} flash_maestro_config_t;

// =============================================================================
// SyRen Motor Controller Configuration
// =============================================================================

#define FLASH_SYREN_MAX_CHANNELS 8

typedef struct __attribute__((packed)) {
    uint8_t channel_count;
    uint8_t serial_port;    // Which UART (1-8 Teensy, 0-1 RP2040)
    uint16_t baud_rate;     // Default 9600
    struct __attribute__((packed)) {
        uint8_t address;       // 128-135
        uint8_t deadband;
        uint8_t ramping;
        uint8_t reserved_s;
        uint16_t timeout_ms;
    } channels[FLASH_SYREN_MAX_CHANNELS];
} flash_syren_config_t;

// =============================================================================
// FAS100 ADV Current/Voltage Sensor Configuration
// =============================================================================

typedef struct __attribute__((packed)) {
    uint8_t enabled;
    uint8_t serial_port;       // Which UART (0-1 RP2040, 1-8 Teensy)
    uint8_t poll_interval_ms;  // Default 50ms
    uint8_t reserved_f;
} flash_fas100_config_t;

// =============================================================================
// Main Storage Structure
// =============================================================================

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t crc;

    char node_id[32];
    node_role_t role;
    char display_name[64];

    bool use_dhcp;
    uint8_t static_ip[4];
    uint8_t subnet_mask[4];
    uint8_t gateway[4];
    uint8_t server_ip[4];
    uint16_t server_port;

    flash_pin_config_t pin_config;

    flash_maestro_config_t maestro_config;

    flash_syren_config_t syren_config;

    flash_fas100_config_t fas100_config;

    uint8_t reserved[32];

} flash_storage_data_t;

// =============================================================================
// Function Declarations
// =============================================================================

// Per-platform implementations
bool flash_storage_init(void);
bool flash_storage_load(flash_storage_data_t* data);
bool flash_storage_save(const flash_storage_data_t* data);
bool flash_storage_erase(void);
bool flash_storage_has_config(void);

// Shared helpers (implemented in shared/src/flash_helpers.c)
void flash_storage_from_node(flash_storage_data_t* data, const saint_node_config_t* node);
void flash_storage_to_node(const flash_storage_data_t* data, saint_node_config_t* node);

#endif // FLASH_TYPES_H
