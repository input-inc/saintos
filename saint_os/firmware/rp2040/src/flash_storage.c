/**
 * SAINT.OS Node Firmware - Flash Storage Implementation
 *
 * Persistent storage for node configuration.
 * - Hardware: Uses RP2040 flash memory (last sector)
 * - Simulation: Uses memory-mapped storage peripheral in Renode
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "flash_storage.h"

#ifdef SIMULATION
// =============================================================================
// Simulation Storage (Memory-Mapped Peripheral)
// =============================================================================

// Storage peripheral base address (configured in Renode)
#define STORAGE_BASE        0x50400000
#define STORAGE_SIZE        4096

// Register offsets
#define STORAGE_REG_CONTROL (STORAGE_BASE + 0x00)
#define STORAGE_REG_STATUS  (STORAGE_BASE + 0x04)
#define STORAGE_DATA_BASE   (STORAGE_BASE + 0x100)

// Control bits
#define STORAGE_CTRL_SAVE   (1 << 0)
#define STORAGE_CTRL_LOAD   (1 << 1)
#define STORAGE_CTRL_ERASE  (1 << 2)

// Status bits
#define STORAGE_STATUS_VALID (1 << 0)
#define STORAGE_STATUS_BUSY  (1 << 1)

#define STORAGE_WRITE32(addr, val) (*(volatile uint32_t*)(addr) = (val))
#define STORAGE_READ32(addr)       (*(volatile uint32_t*)(addr))

static bool storage_initialized = false;

bool flash_storage_init(void)
{
    printf("Flash storage: simulation mode (peripheral at 0x%08X)\n", STORAGE_BASE);
    storage_initialized = true;
    return true;
}

bool flash_storage_load(flash_storage_data_t* data)
{
    if (!storage_initialized || !data) return false;

    // Trigger load from file
    STORAGE_WRITE32(STORAGE_REG_CONTROL, STORAGE_CTRL_LOAD);
    sleep_ms(1);

    // Check if valid data exists
    uint32_t status = STORAGE_READ32(STORAGE_REG_STATUS);
    if (!(status & STORAGE_STATUS_VALID)) {
        printf("Flash storage: no valid data\n");
        return false;
    }

    // Copy data from peripheral memory
    volatile uint8_t* src = (volatile uint8_t*)STORAGE_DATA_BASE;
    uint8_t* dst = (uint8_t*)data;
    for (size_t i = 0; i < sizeof(flash_storage_data_t); i++) {
        dst[i] = src[i];
    }

    // Verify magic number
    if (data->magic != FLASH_STORAGE_MAGIC) {
        printf("Flash storage: invalid magic (got 0x%08X)\n", data->magic);
        return false;
    }

    // Handle version migration
    if (data->version < FLASH_STORAGE_VERSION) {
        flash_storage_data_t* mutable_data = (flash_storage_data_t*)data;
        printf("Flash storage: migrating from version %d to %d\n",
               mutable_data->version, FLASH_STORAGE_VERSION);

        // Version 1 -> 2: Added pin_config
        if (mutable_data->version == 1) {
            // Initialize pin_config to empty
            memset(&mutable_data->pin_config, 0, sizeof(mutable_data->pin_config));
            mutable_data->pin_config.version = FLASH_PIN_CONFIG_VERSION;
            mutable_data->pin_config.pin_count = 0;
        }

        mutable_data->version = FLASH_STORAGE_VERSION;
    }

    printf("Flash storage: loaded config (version %d)\n", data->version);
    return true;
}

bool flash_storage_save(const flash_storage_data_t* data)
{
    if (!storage_initialized || !data) return false;

    // Copy data to peripheral memory
    volatile uint8_t* dst = (volatile uint8_t*)STORAGE_DATA_BASE;
    const uint8_t* src = (const uint8_t*)data;
    for (size_t i = 0; i < sizeof(flash_storage_data_t); i++) {
        dst[i] = src[i];
    }

    // Trigger save to file
    STORAGE_WRITE32(STORAGE_REG_CONTROL, STORAGE_CTRL_SAVE);
    sleep_ms(1);

    printf("Flash storage: saved config\n");
    return true;
}

bool flash_storage_erase(void)
{
    if (!storage_initialized) return false;

    STORAGE_WRITE32(STORAGE_REG_CONTROL, STORAGE_CTRL_ERASE);
    sleep_ms(1);

    printf("Flash storage: erased\n");
    return true;
}

bool flash_storage_has_config(void)
{
    if (!storage_initialized) return false;

    STORAGE_WRITE32(STORAGE_REG_CONTROL, STORAGE_CTRL_LOAD);
    sleep_ms(1);

    uint32_t status = STORAGE_READ32(STORAGE_REG_STATUS);
    return (status & STORAGE_STATUS_VALID) != 0;
}

#else
// =============================================================================
// Hardware Flash Storage (RP2040)
// =============================================================================

#include "hardware/flash.h"
#include "hardware/sync.h"

// Use last sector of flash for storage
// RP2040 has 2MB flash, sector size is 4KB
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define FLASH_TARGET_ADDR   (XIP_BASE + FLASH_TARGET_OFFSET)

static bool storage_initialized = false;

bool flash_storage_init(void)
{
    printf("Flash storage: hardware mode (offset 0x%X)\n", FLASH_TARGET_OFFSET);
    storage_initialized = true;
    return true;
}

bool flash_storage_load(flash_storage_data_t* data)
{
    if (!storage_initialized || !data) return false;

    // Read directly from flash (memory-mapped)
    const flash_storage_data_t* flash_data = (const flash_storage_data_t*)FLASH_TARGET_ADDR;

    // Verify magic number
    if (flash_data->magic != FLASH_STORAGE_MAGIC) {
        printf("Flash storage: no valid data (magic 0x%08X)\n", flash_data->magic);
        return false;
    }

    // TODO: Verify CRC

    // Copy to output
    memcpy(data, flash_data, sizeof(flash_storage_data_t));

    // Handle version migration
    if (data->version < FLASH_STORAGE_VERSION) {
        printf("Flash storage: migrating from version %d to %d\n",
               data->version, FLASH_STORAGE_VERSION);

        // Version 1 -> 2: Added pin_config
        if (data->version == 1) {
            // Initialize pin_config to empty
            memset(&data->pin_config, 0, sizeof(data->pin_config));
            data->pin_config.version = FLASH_PIN_CONFIG_VERSION;
            data->pin_config.pin_count = 0;
        }

        data->version = FLASH_STORAGE_VERSION;
    }

    printf("Flash storage: loaded config (version %d)\n", data->version);
    return true;
}

// Calculate how many flash pages we need for the storage structure
// Flash pages are 256 bytes, sector is 4KB
#define STORAGE_SIZE_PAGES  ((sizeof(flash_storage_data_t) + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE)
#define STORAGE_SIZE_BYTES  (STORAGE_SIZE_PAGES * FLASH_PAGE_SIZE)

bool flash_storage_save(const flash_storage_data_t* data)
{
    if (!storage_initialized || !data) return false;

    printf("Flash storage: preparing to save (%zu bytes, %d pages)...\n",
           sizeof(flash_storage_data_t), STORAGE_SIZE_PAGES);

    // Prepare data with proper alignment
    // Use static buffer to avoid stack overflow (struct is ~835 bytes, needs 4 pages)
    static uint8_t buffer[STORAGE_SIZE_BYTES] __attribute__((aligned(4)));
    memset(buffer, 0xFF, sizeof(buffer));
    memcpy(buffer, data, sizeof(flash_storage_data_t));

    // Flash operations disable interrupts for ~100-400ms which can disrupt
    // network communication. We need to handle this carefully.

    // Give any pending network operations time to complete
    sleep_ms(50);

    // Disable interrupts during flash operations
    uint32_t interrupts = save_and_disable_interrupts();

    // Erase sector (this is the slow part - can take 50-400ms)
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);

    // Write all pages needed for the data structure
    flash_range_program(FLASH_TARGET_OFFSET, buffer, STORAGE_SIZE_BYTES);

    restore_interrupts(interrupts);

    // Allow network stack to recover after long interrupt-disabled period
    // The W5500 may have received packets that need processing
    sleep_ms(100);

    printf("Flash storage: saved config\n");
    return true;
}

bool flash_storage_erase(void)
{
    if (!storage_initialized) return false;

    uint32_t interrupts = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    restore_interrupts(interrupts);

    printf("Flash storage: erased\n");
    return true;
}

bool flash_storage_has_config(void)
{
    if (!storage_initialized) return false;

    const flash_storage_data_t* flash_data = (const flash_storage_data_t*)FLASH_TARGET_ADDR;
    return flash_data->magic == FLASH_STORAGE_MAGIC;
}

#endif // SIMULATION

// =============================================================================
// Common Helper Functions
// =============================================================================

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
