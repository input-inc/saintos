/**
 * SAINT.OS Firmware - Shared Node Types
 *
 * Platform-agnostic node state, role, configuration structures, and timing constants.
 * Shared between RP2040 and Teensy 4.1 firmware.
 */

#ifndef SAINT_TYPES_H
#define SAINT_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// =============================================================================
// Node States
// =============================================================================

typedef enum {
    NODE_STATE_BOOT,        // Initial boot
    NODE_STATE_CONNECTING,  // Connecting to network
    NODE_STATE_UNADOPTED,   // Connected but not adopted
    NODE_STATE_ADOPTING,    // Being adopted (downloading config)
    NODE_STATE_ACTIVE,      // Adopted and running
    NODE_STATE_ERROR        // Error state
} node_state_t;

// =============================================================================
// Node Roles
// =============================================================================

typedef enum {
    NODE_ROLE_NONE = 0,
    NODE_ROLE_HEAD,
    NODE_ROLE_ARMS_LEFT,
    NODE_ROLE_ARMS_RIGHT,
    NODE_ROLE_TRACKS,
    NODE_ROLE_CONSOLE
} node_role_t;

// =============================================================================
// Node Configuration
// =============================================================================

typedef struct {
    // Identity
    char node_id[32];           // Unique node ID (from flash or MAC)
    uint8_t mac_address[6];     // Ethernet MAC address

    // Network
    bool use_dhcp;
    uint8_t static_ip[4];
    uint8_t subnet_mask[4];
    uint8_t gateway[4];
    uint8_t server_ip[4];
    uint16_t server_port;       // micro-ROS agent UDP port

    // Role
    node_role_t role;
    char display_name[64];

    // State
    node_state_t state;
    uint32_t uptime_ms;
    uint32_t last_announce_ms;

} saint_node_config_t;

// =============================================================================
// Timing Constants
// =============================================================================

#define ANNOUNCE_INTERVAL_MS    1000    // Announce every 1 second when unadopted
#define HEARTBEAT_INTERVAL_MS   5000    // Heartbeat every 5 seconds
#define STATE_UPDATE_INTERVAL_MS 100    // State updates at 10Hz when active
#define RECONNECT_DELAY_MS      5000    // Wait 5 seconds between reconnect attempts

// =============================================================================
// Global Node Instance
// =============================================================================

extern saint_node_config_t g_node;

// =============================================================================
// Function Declarations (implemented in shared/src/node_state.c)
// =============================================================================

void node_state_init(void);
void node_state_update(void);
void node_set_state(node_state_t new_state);
const char* node_state_to_string(node_state_t state);
const char* node_role_to_string(node_role_t role);
bool node_adopt(node_role_t role, const char* display_name);
bool node_reset(bool factory_reset);
bool node_save_config(void);

// hardware.c (per-platform)
void hardware_init(void);
void hardware_update(void);
uint32_t hardware_get_unique_id(char* buffer, size_t len);
float hardware_get_cpu_temp(void);

// led_status.c (per-platform)
void led_init(void);
void led_set_state(node_state_t state);
void led_update(void);
void led_identify(uint8_t flash_count);

#endif // SAINT_TYPES_H
