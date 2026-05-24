/**
 * SAINT.OS Firmware - Flash Storage Helpers (Shared)
 *
 * Platform-agnostic data conversion between node config and flash storage.
 * Only uses memset/memcpy/strncpy - no platform-specific APIs.
 */

#include <string.h>

#include "flash_types.h"

void flash_storage_from_node(flash_storage_data_t* data, const saint_node_config_t* node)
{
    if (!data || !node) return;

    memset(data, 0, sizeof(flash_storage_data_t));

    data->magic = FLASH_STORAGE_MAGIC;
    data->version = FLASH_STORAGE_VERSION;
    data->crc = 0; // TODO: Calculate CRC

    strncpy(data->node_id, node->node_id, sizeof(data->node_id) - 1);
    data->role = node->role;
    strncpy(data->display_name, node->display_name, sizeof(data->display_name) - 1);

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

    strncpy(node->node_id, data->node_id, sizeof(node->node_id) - 1);
    node->role = data->role;
    strncpy(node->display_name, data->display_name, sizeof(node->display_name) - 1);

    node->use_dhcp = data->use_dhcp;
    memcpy(node->static_ip, data->static_ip, 4);
    memcpy(node->subnet_mask, data->subnet_mask, 4);
    memcpy(node->gateway, data->gateway, 4);
    memcpy(node->server_ip, data->server_ip, 4);
    node->server_port = data->server_port;
}
