/**
 * SAINT.OS Node Firmware - State Machine
 *
 * Manages node state transitions and role-specific behavior.
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "saint_node.h"

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

    // Default server configuration
    g_node.server_ip[0] = 192;
    g_node.server_ip[1] = 168;
    g_node.server_ip[2] = 1;
    g_node.server_ip[3] = 10;
    g_node.server_port = 8888;  // micro-ROS agent port

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
        // TODO: Clear flash storage
    }

    // Return to unadopted state
    node_set_state(NODE_STATE_UNADOPTED);

    return true;
}
