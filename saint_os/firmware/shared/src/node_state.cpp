/**
 * SAINT.OS Firmware - State Machine (Shared)
 *
 * Platform-agnostic node state management.
 * Uses PLATFORM_PRINTF macro for output.
 */

#include <stdio.h>
#include <string.h>

#include "platform.h"
#include "saint_types.h"
#include "flash_types.h"

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

void node_state_init(void)
{
    memset(&g_node, 0, sizeof(g_node));

    g_node.state = NODE_STATE_BOOT;
    g_node.role = NODE_ROLE_NONE;
    g_node.use_dhcp = true;

#ifdef SIMULATION
    g_node.server_ip[0] = 127;
    g_node.server_ip[1] = 0;
    g_node.server_ip[2] = 0;
    g_node.server_ip[3] = 1;
    g_node.server_port = 8888;
#else
    g_node.server_ip[0] = 0;
    g_node.server_ip[1] = 0;
    g_node.server_ip[2] = 0;
    g_node.server_ip[3] = 0;
    g_node.server_port = 0;
#endif

    if (!flash_storage_init()) {
        PLATFORM_PRINTF("Warning: Flash storage init failed\n");
    }

    flash_storage_data_t saved_config;
    if (flash_storage_load(&saved_config)) {
        PLATFORM_PRINTF("Loaded saved configuration\n");
        flash_storage_to_node(&saved_config, &g_node);

        if (g_node.role != NODE_ROLE_NONE) {
            PLATFORM_PRINTF("Previously adopted as: %s (%s)\n",
                   node_role_to_string(g_node.role), g_node.display_name);
        }
    } else {
        PLATFORM_PRINTF("No saved configuration found\n");
    }

    PLATFORM_PRINTF("Node state initialized\n");
}

void node_state_update(void)
{
    switch (g_node.state) {
        case NODE_STATE_BOOT:
            break;
        case NODE_STATE_CONNECTING:
            break;
        case NODE_STATE_UNADOPTED:
            break;
        case NODE_STATE_ADOPTING:
            break;
        case NODE_STATE_ACTIVE:
            break;
        case NODE_STATE_ERROR:
            break;
    }
}

void node_set_state(node_state_t new_state)
{
    if (g_node.state != new_state) {
        PLATFORM_PRINTF("State: %s -> %s\n",
               state_strings[g_node.state],
               state_strings[new_state]);
        g_node.state = new_state;
    }
}

const char* node_state_to_string(node_state_t state)
{
    if (state < sizeof(state_strings) / sizeof(state_strings[0])) {
        return state_strings[state];
    }
    return "UNKNOWN";
}

const char* node_role_to_string(node_role_t role)
{
    if (role < sizeof(role_strings) / sizeof(role_strings[0])) {
        return role_strings[role];
    }
    return "unknown";
}

bool node_adopt(node_role_t role, const char* display_name)
{
    if (g_node.state != NODE_STATE_UNADOPTED) {
        PLATFORM_PRINTF("Cannot adopt: not in UNADOPTED state\n");
        return false;
    }

    PLATFORM_PRINTF("Adopting node as %s (%s)\n",
           node_role_to_string(role), display_name);

    node_set_state(NODE_STATE_ADOPTING);

    g_node.role = role;
    strncpy(g_node.display_name, display_name, sizeof(g_node.display_name) - 1);

    flash_storage_data_t config;
    flash_storage_from_node(&config, &g_node);
    if (flash_storage_save(&config)) {
        PLATFORM_PRINTF("Configuration saved to flash\n");
    } else {
        PLATFORM_PRINTF("Warning: Failed to save configuration\n");
    }

    node_set_state(NODE_STATE_ACTIVE);

    PLATFORM_PRINTF("Node adopted successfully as %s\n", node_role_to_string(role));
    return true;
}

bool node_reset(bool factory_reset)
{
    PLATFORM_PRINTF("Resetting node (factory: %s)\n", factory_reset ? "yes" : "no");

    g_node.role = NODE_ROLE_NONE;
    memset(g_node.display_name, 0, sizeof(g_node.display_name));

    if (factory_reset) {
        if (flash_storage_erase()) {
            PLATFORM_PRINTF("Flash storage erased\n");
        } else {
            PLATFORM_PRINTF("Warning: Failed to erase flash storage\n");
        }
    }

    node_set_state(NODE_STATE_UNADOPTED);

    return true;
}

bool node_save_config(void)
{
    flash_storage_data_t config;
    flash_storage_from_node(&config, &g_node);
    return flash_storage_save(&config);
}
