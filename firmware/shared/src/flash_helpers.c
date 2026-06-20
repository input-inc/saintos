/**
 * SAINT.OS Firmware - Flash Storage Helpers (Shared)
 *
 * Platform-agnostic data conversion between node config and flash storage.
 * Only uses memset/memcpy/strncpy - no platform-specific APIs.
 */

#include <stdio.h>      /* snprintf */
#include <string.h>

#include "flash_types.h"

void flash_storage_from_node(flash_storage_data_t* data, const saint_node_config_t* node)
{
    if (!data || !node) return;

    memset(data, 0, sizeof(flash_storage_data_t));

    data->magic = FLASH_STORAGE_MAGIC;
    data->version = FLASH_STORAGE_VERSION;
    data->crc = 0; // TODO: Calculate CRC

    /* snprintf instead of strncpy(dst, src, sizeof(dst)-1): the latter
     * idiom doesn't guarantee a NUL terminator when src is exactly
     * the buffer length, and -Wstringop-truncation rightly flags
     * it. snprintf always NUL-terminates and is clear-intent. */
    snprintf(data->node_id, sizeof(data->node_id), "%s", node->node_id);
    data->role = node->role;
    snprintf(data->display_name, sizeof(data->display_name), "%s", node->display_name);

    data->use_dhcp = node->use_dhcp;
    memcpy(data->static_ip, node->static_ip, 4);
    memcpy(data->subnet_mask, node->subnet_mask, 4);
    memcpy(data->gateway, node->gateway, 4);
    memcpy(data->server_ip, node->server_ip, 4);
    data->server_port = node->server_port;
}

void flash_storage_to_node(const flash_storage_data_t* data, saint_node_config_t* node)
{
    if (!data || !node) return;

    snprintf(node->node_id, sizeof(node->node_id), "%s", data->node_id);
    node->role = data->role;
    snprintf(node->display_name, sizeof(node->display_name), "%s", data->display_name);

    node->use_dhcp = data->use_dhcp;
    memcpy(node->static_ip, data->static_ip, 4);
    memcpy(node->subnet_mask, data->subnet_mask, 4);
    memcpy(node->gateway, data->gateway, 4);
    memcpy(node->server_ip, data->server_ip, 4);
    node->server_port = data->server_port;
}
