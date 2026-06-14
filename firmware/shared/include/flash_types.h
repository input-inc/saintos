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

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Storage Configuration
// =============================================================================

#define FLASH_STORAGE_MAGIC     0x53414E54  // "SANT"
#define FLASH_STORAGE_VERSION   11
// Bump history:
//   v8: added uart_pins block.
//   v9: added estop_pin + uart_swap to flash_roboclaw_config_t units.
//       Each unit grew 2 bytes (×FLASH_ROBOCLAW_MAX_UNITS = +16 bytes),
//       which shifts every field AFTER roboclaw_config in
//       flash_storage_data_t. Without a migration, loading an old v8
//       blob into the new struct mis-aligns pathfinder_bms_config and
//       uart_pins — concretely, FAS100 TX/RX pins read as 0 and the
//       driver auto-starts probing on default pins, never locking.
//       The v8→v9 migration zeros roboclaw_config, pathfinder_bms_
//       config, and uart_pins; operators re-sync from the dashboard
//       once, then everything persists correctly.
//   v10: added flash_tic_config_t for the Pololu Tic stepper driver,
//        AND added tic_tx_pin/tic_rx_pin to flash_uart_pins_t (stolen
//        from reserved_u). The Tic block lives AFTER pathfinder_bms_
//        config in flash_storage_data_t so existing fields keep their
//        offsets. uart_pins shifts by 0 bytes (the new pins replace
//        reserved bytes), but the FLASH_UART_PINS layout is part of
//        flash_storage_data_t and an unmigrated v9 load would still
//        zero-fill the new tic-specific bytes. The v9→v10 migration
//        zeros tic_config + the new uart_pins fields; pre-existing
//        peripheral configs survive untouched.
//   v11: added flash_tmc2208_config_t for the TMC2208 stepper driver,
//        AND added tmc2208_tx_pin/tmc2208_rx_pin to flash_uart_pins_t
//        (more reserved_u bytes). The TMC2208 block lives AFTER
//        tic_config so blocks BEFORE it (maestro/syren/fas100/roboclaw/
//        pathfinder/tic) keep their offsets. The v10→v11 migration
//        zeros tmc2208_config + uart_pins; non-Tic peripheral configs
//        survive untouched but operators must re-sync UART pin pairs.

#define FLASH_PIN_CONFIG_MAX_PINS     16
#define FLASH_PIN_CONFIG_MAX_NAME_LEN 32
// v2 (2026-06): servo mode's param1/param2/reserved_pin slot now
//     stores (start_us<<0, end_us<<16) in param1, center_us in
//     param2, and home_us in reserved_pin[0..1]. Old v1 saves had
//     (frequency, min_pulse_us, max_pulse_us) and are rejected on
//     load — operator re-syncs from the Peripherals tab.
#define FLASH_PIN_CONFIG_VERSION      2

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

/* transport_mode values: 0 = USB host (zero-default for backward
 * compat with older Teensy saves), 1 = UART. serial_port is only
 * meaningful in UART mode and indexes the platform's UART instance
 * (0-1 on RP2040, 1-8 on Teensy hardware Serial). */
#define FLASH_MAESTRO_TRANSPORT_USB_HOST 0
#define FLASH_MAESTRO_TRANSPORT_UART     1

typedef struct __attribute__((packed)) {
    uint8_t channel_count;
    uint8_t transport_mode;     /* FLASH_MAESTRO_TRANSPORT_*           */
    uint8_t serial_port;        /* UART instance index (UART mode)     */
    uint8_t reserved_m;
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
// RoboClaw Solo 60A Motor Controller Configuration
// =============================================================================

#define FLASH_ROBOCLAW_MAX_UNITS 8

typedef struct __attribute__((packed)) {
    uint8_t unit_count;
    uint8_t serial_port;    // Which UART (0-1 RP2040, 1-8 Teensy)
    uint16_t baud_rate;     // Default 38400
    struct __attribute__((packed)) {
        uint8_t address;       // 0x80-0x87
        uint8_t deadband;
        uint16_t max_current_ma;
        // GPIO wired to this unit's S3 (E-stop) input. 0xFF = none
        // (skip the boot-time deassert). See pin_types.h's roboclaw
        // struct for full rationale.
        uint8_t estop_pin;
        // 1 = use PIO UART (TX/RX swappable to any GPIO); 0 = use
        // hardware UART with silicon-fixed pin map. See pin_types.h.
        uint8_t uart_swap;
    } units[FLASH_ROBOCLAW_MAX_UNITS];
} flash_roboclaw_config_t;

// =============================================================================
// Pathfinder BMS Configuration
// =============================================================================

typedef struct __attribute__((packed)) {
    uint8_t enabled;
    uint8_t serial_port;       // Which UART (0-1 RP2040, 1-8 Teensy)
    uint16_t poll_interval_ms; // Default 1000ms
} flash_pathfinder_bms_config_t;

// =============================================================================
// Pololu Tic Stepper Controller Configuration
// =============================================================================

#define FLASH_TIC_MAX_UNITS 8

typedef struct __attribute__((packed)) {
    uint8_t unit_count;
    uint8_t serial_port;    // Which UART (0-1 RP2040, 1-8 Teensy)
    uint16_t baud_rate;     // Default 9600
    struct __attribute__((packed)) {
        uint8_t address;          // Tic device ID, 1..127
        uint8_t reserved_t;
        uint16_t max_speed_pps;   // operator scaling for target_velocity
        int32_t  max_position;    // operator scaling for target_position
    } units[FLASH_TIC_MAX_UNITS];
} flash_tic_config_t;

// =============================================================================
// TMC2208 Stepper Driver Configuration
// =============================================================================

#define FLASH_TMC2208_MAX_AXES 4

typedef struct __attribute__((packed)) {
    uint8_t axis_count;
    uint8_t serial_port;     // Which UART
    uint16_t reserved_t208;  // padding to keep baud_rate aligned
    uint32_t baud_rate;      // Default 115200; full 32 bits because the
                             // common TMC2208 rate 115200 exceeds uint16.
    struct __attribute__((packed)) {
        uint8_t address;          // 0..3 slave addr (PCB MS1/MS2 strap)
        uint8_t step_pin;
        uint8_t dir_pin;
        uint8_t stealth_chop;     // 0/1
        uint16_t microsteps;
        uint16_t run_current_ma;
        uint16_t hold_current_ma;
        uint16_t rsense_milliohm;
        uint16_t max_speed_pps;
        int32_t  max_position;
    } axes[FLASH_TMC2208_MAX_AXES];
} flash_tmc2208_config_t;

// =============================================================================
// UART Pin Assignments (added v8)
// =============================================================================
//
// Per-peripheral TX/RX pin selection. A value of 0 for tx_pin means
// "use platform default" — drivers fall back to their hardcoded pair.
// On v7 -> v8 migration this whole block is zero-initialized, so existing
// nodes silently keep their old behavior.

typedef struct __attribute__((packed)) {
    uint8_t fas100_tx_pin;
    uint8_t fas100_rx_pin;
    uint8_t syren_tx_pin;
    uint8_t syren_rx_pin;
    uint8_t roboclaw_tx_pin;
    uint8_t roboclaw_rx_pin;
    uint8_t pathfinder_bms_tx_pin;
    uint8_t pathfinder_bms_rx_pin;
    uint8_t maestro_tx_pin;
    uint8_t maestro_rx_pin;
    uint8_t tic_tx_pin;
    uint8_t tic_rx_pin;
    uint8_t tmc2208_tx_pin;
    uint8_t tmc2208_rx_pin;
    uint8_t reserved_u[2];
} flash_uart_pins_t;

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

    flash_roboclaw_config_t roboclaw_config;

    flash_pathfinder_bms_config_t pathfinder_bms_config;

    flash_tic_config_t tic_config;

    flash_tmc2208_config_t tmc2208_config;

    flash_uart_pins_t uart_pins;

    uint8_t reserved[16];

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

#ifdef __cplusplus
}
#endif

#endif // FLASH_TYPES_H
