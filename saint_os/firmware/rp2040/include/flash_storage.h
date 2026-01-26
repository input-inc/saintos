/**
 * SAINT.OS Node Firmware - Flash Storage
 *
 * Persistent storage for node configuration in RP2040 flash memory.
 * For simulation, uses a memory-mapped storage peripheral.
 */

#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include <stdint.h>
#include <stdbool.h>
#include "saint_node.h"

// =============================================================================
// Storage Configuration
// =============================================================================

// Magic number to identify valid configuration
#define FLASH_STORAGE_MAGIC     0x53414E54  // "SANT"
#define FLASH_STORAGE_VERSION   2           // Bumped for pin config support

// Pin configuration constants (duplicated here to avoid circular dependency)
#define FLASH_PIN_CONFIG_MAX_PINS     16
#define FLASH_PIN_CONFIG_MAX_NAME_LEN 32
#define FLASH_PIN_CONFIG_VERSION      1

// =============================================================================
// Stored Configuration Structure
// =============================================================================

// =============================================================================
// Pin Configuration Storage Structure
// =============================================================================

typedef struct __attribute__((packed)) {
    uint8_t version;                                // Configuration version
    uint8_t pin_count;                              // Number of configured pins
    uint8_t reserved_hdr[2];                        // Padding for alignment
    struct __attribute__((packed)) {
        uint8_t gpio;                               // GPIO number
        uint8_t mode;                               // pin_mode_t value
        char logical_name[FLASH_PIN_CONFIG_MAX_NAME_LEN]; // Logical function name
        uint32_t param1;                            // Mode-specific param (e.g., PWM freq)
        uint16_t param2;                            // Mode-specific param (e.g., duty cycle)
        uint8_t reserved_pin[2];                    // Padding
    } pins[FLASH_PIN_CONFIG_MAX_PINS];
} flash_pin_config_t;

// =============================================================================
// Main Storage Structure
// =============================================================================

typedef struct __attribute__((packed)) {
    uint32_t magic;             // Must be FLASH_STORAGE_MAGIC
    uint16_t version;           // Storage format version
    uint16_t crc;               // CRC16 of data (excluding this header)

    // Persisted node configuration
    char node_id[32];           // Assigned node ID (if any)
    node_role_t role;           // Assigned role
    char display_name[64];      // Display name

    // Network configuration
    bool use_dhcp;
    uint8_t static_ip[4];
    uint8_t subnet_mask[4];
    uint8_t gateway[4];
    uint8_t server_ip[4];
    uint16_t server_port;

    // Pin configuration
    flash_pin_config_t pin_config;

    // Reserved for future use
    uint8_t reserved[32];

} flash_storage_data_t;

// =============================================================================
// Function Declarations
// =============================================================================

/**
 * Initialize flash storage subsystem.
 * @return true if initialization successful
 */
bool flash_storage_init(void);

/**
 * Load configuration from flash.
 * @param data Pointer to storage structure to fill
 * @return true if valid configuration was loaded
 */
bool flash_storage_load(flash_storage_data_t* data);

/**
 * Save configuration to flash.
 * @param data Pointer to storage structure to save
 * @return true if save successful
 */
bool flash_storage_save(const flash_storage_data_t* data);

/**
 * Erase stored configuration (factory reset).
 * @return true if erase successful
 */
bool flash_storage_erase(void);

/**
 * Check if valid configuration exists.
 * @return true if flash contains valid configuration
 */
bool flash_storage_has_config(void);

/**
 * Helper: Copy node config to storage structure.
 */
void flash_storage_from_node(flash_storage_data_t* data, const saint_node_config_t* node);

/**
 * Helper: Copy storage structure to node config.
 */
void flash_storage_to_node(const flash_storage_data_t* data, saint_node_config_t* node);

#endif // FLASH_STORAGE_H
