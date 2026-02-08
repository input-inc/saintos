/**
 * SAINT.OS Node Firmware - State Machine
 *
 * Manages node state transitions and role-specific behavior.
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "saint_node.h"
#include "flash_storage.h"

// =============================================================================
// State Strings
// =============================================================================

static const char* state_strings[] = {
    "BOOT",
    "CONNECTING",
    "UNADOPTED",
    "ADOPTING",
    "ACTIVE",
    "ERROR"
};

static const char* role_strings[] = {
    "none",
    "head",
    "arms_left",
    "arms_right",
    "tracks",
    "console"
};

// =============================================================================
// Public Functions
// =============================================================================

/**
 * Initialize node state.
 */
void node_state_init(void)
{
    memset(&g_node, 0, sizeof(g_node));

    g_node.state = NODE_STATE_BOOT;
    g_node.role = NODE_ROLE_NONE;
    g_node.use_dhcp = true;

    // Default server configuration (micro-ROS agent)
#ifdef SIMULATION
    // Simulation: agent runs on localhost (no discovery in simulation)
    g_node.server_ip[0] = 127;
    g_node.server_ip[1] = 0;
    g_node.server_ip[2] = 0;
    g_node.server_ip[3] = 1;
    g_node.server_port = 8888;
#else
    // Hardware: server IP will be discovered via UDP broadcast
    // Initialize to zeros - will be set by discover_server()
    g_node.server_ip[0] = 0;
    g_node.server_ip[1] = 0;
    g_node.server_ip[2] = 0;
    g_node.server_ip[3] = 0;
    g_node.server_port = 0;
#endif

    // Initialize flash storage
    if (!flash_storage_init()) {
        printf("Warning: Flash storage init failed\n");
    }

    // Try to load saved configuration
    flash_storage_data_t saved_config;
    if (flash_storage_load(&saved_config)) {
        printf("Loaded saved configuration\n");
        flash_storage_to_node(&saved_config, &g_node);

        // If we have a saved role, we're previously adopted
        if (g_node.role != NODE_ROLE_NONE) {
            printf("Previously adopted as: %s (%s)\n",
                   node_role_to_string(g_node.role), g_node.display_name);
        }
    } else {
        printf("No saved configuration found\n");
    }

    printf("Node state initialized\n");
}

/**
 * Update node state machine.
 * Called from main loop.
 */
void node_state_update(void)
{
    // State-specific updates
    switch (g_node.state) {
        case NODE_STATE_BOOT:
            // Transition to connecting handled in main
            break;

        case NODE_STATE_CONNECTING:
            // Waiting for network connection
            break;

        case NODE_STATE_UNADOPTED:
            // Waiting for adoption
            // Announcements handled in main loop
            break;

        case NODE_STATE_ADOPTING:
            // Downloading configuration, initializing role
            break;

        case NODE_STATE_ACTIVE:
            // Running role-specific logic
            // TODO: Call role-specific update functions
            break;

        case NODE_STATE_ERROR:
            // Error state - wait for reset
            break;
    }
}

/**
 * Set node state.
 */
void node_set_state(node_state_t new_state)
{
    if (g_node.state != new_state) {
        printf("State: %s -> %s\n",
               state_strings[g_node.state],
               state_strings[new_state]);
        g_node.state = new_state;
    }
}

/**
 * Get string representation of state.
 */
const char* node_state_to_string(node_state_t state)
{
    if (state < sizeof(state_strings) / sizeof(state_strings[0])) {
        return state_strings[state];
    }
    return "UNKNOWN";
}

/**
 * Get string representation of role.
 */
const char* node_role_to_string(node_role_t role)
{
    if (role < sizeof(role_strings) / sizeof(role_strings[0])) {
        return role_strings[role];
    }
    return "unknown";
}

/**
 * Adopt node with specified role.
 */
bool node_adopt(node_role_t role, const char* display_name)
{
    if (g_node.state != NODE_STATE_UNADOPTED) {
        printf("Cannot adopt: not in UNADOPTED state\n");
        return false;
    }

    printf("Adopting node as %s (%s)\n",
           node_role_to_string(role), display_name);

    node_set_state(NODE_STATE_ADOPTING);

    g_node.role = role;
    strncpy(g_node.display_name, display_name, sizeof(g_node.display_name) - 1);

    // Save configuration to flash
    flash_storage_data_t config;
    flash_storage_from_node(&config, &g_node);
    if (flash_storage_save(&config)) {
        printf("Configuration saved to flash\n");
    } else {
        printf("Warning: Failed to save configuration\n");
    }

    // TODO: Initialize role-specific publishers/subscribers
    // TODO: Load role-specific configuration

    // Transition to active
    node_set_state(NODE_STATE_ACTIVE);

    printf("Node adopted successfully as %s\n", node_role_to_string(role));
    return true;
}

/**
 * Reset node to unadopted state.
 */
bool node_reset(bool factory_reset)
{
    printf("Resetting node (factory: %s)\n", factory_reset ? "yes" : "no");

    // Stop role-specific activities
    g_node.role = NODE_ROLE_NONE;
    memset(g_node.display_name, 0, sizeof(g_node.display_name));

    if (factory_reset) {
        // Clear stored configuration
        if (flash_storage_erase()) {
            printf("Flash storage erased\n");
        } else {
            printf("Warning: Failed to erase flash storage\n");
        }
    }

    // Return to unadopted state
    node_set_state(NODE_STATE_UNADOPTED);

    return true;
}

/**
 * Save current node configuration to flash.
 * Call this after any configuration changes.
 */
bool node_save_config(void)
{
    flash_storage_data_t config;
    flash_storage_from_node(&config, &g_node);
    return flash_storage_save(&config);
}
