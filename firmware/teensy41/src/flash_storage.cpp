/**
 * SAINT.OS Node Firmware - Teensy 4.1 Flash Storage
 *
 * Hardware mode: EEPROM backend (Teensy 4.1 has 4284 bytes of emulated EEPROM)
 * Simulation mode: Memory-mapped storage peripheral (same addresses as RP2040 sim)
 */

#include <Arduino.h>
#include "platform.h"
#include <string.h>

extern "C" {
#include "flash_storage.h"
}

#ifdef SIMULATION
// =============================================================================
// Simulation: memory-mapped storage peripheral (same addresses as RP2040 sim)
// =============================================================================

#define STORAGE_BASE        0x50400000
#define STORAGE_SIZE        4096
#define STORAGE_REG_CONTROL (STORAGE_BASE + 0x00)
#define STORAGE_REG_STATUS  (STORAGE_BASE + 0x04)
#define STORAGE_DATA_BASE   (STORAGE_BASE + 0x100)

#define STORAGE_CTRL_SAVE   (1 << 0)
#define STORAGE_CTRL_LOAD   (1 << 1)
#define STORAGE_CTRL_ERASE  (1 << 2)

#define STORAGE_STATUS_VALID (1 << 0)
#define STORAGE_STATUS_BUSY  (1 << 1)

#define STORAGE_WRITE32(addr, val) (*(volatile uint32_t*)(addr) = (val))
#define STORAGE_READ32(addr)       (*(volatile uint32_t*)(addr))

static bool storage_initialized = false;

extern "C" {

bool flash_storage_init(void)
{
    Serial.printf("Flash storage: simulation mode (peripheral at 0x%08X)\n", STORAGE_BASE);
    storage_initialized = true;
    return true;
}

bool flash_storage_load(flash_storage_data_t* data)
{
    if (!storage_initialized || !data) return false;

    STORAGE_WRITE32(STORAGE_REG_CONTROL, STORAGE_CTRL_LOAD);
    delay(1);

    uint32_t status = STORAGE_READ32(STORAGE_REG_STATUS);
    if (!(status & STORAGE_STATUS_VALID)) {
        Serial.printf("Flash storage: no valid data\n");
        return false;
    }

    volatile uint8_t* src = (volatile uint8_t*)STORAGE_DATA_BASE;
    uint8_t* dst = (uint8_t*)data;
    for (size_t i = 0; i < sizeof(flash_storage_data_t); i++) {
        dst[i] = src[i];
    }

    if (data->magic != FLASH_STORAGE_MAGIC) {
        Serial.printf("Flash storage: invalid magic (got 0x%08lX)\n", (unsigned long)data->magic);
        return false;
    }

    if (data->version < FLASH_STORAGE_VERSION) {
        flash_storage_data_t* mutable_data = (flash_storage_data_t*)data;
        Serial.printf("Flash storage: migrating from version %d to %d\n",
                       mutable_data->version, FLASH_STORAGE_VERSION);
        if (mutable_data->version == 1) {
            memset(&mutable_data->pin_config, 0, sizeof(mutable_data->pin_config));
            mutable_data->pin_config.version = FLASH_PIN_CONFIG_VERSION;
            mutable_data->pin_config.pin_count = 0;
        }
        if (mutable_data->version <= 2) {
            memset(&mutable_data->maestro_config, 0, sizeof(mutable_data->maestro_config));
        }
        if (mutable_data->version <= 3) {
            memset(&mutable_data->syren_config, 0, sizeof(mutable_data->syren_config));
        }
        if (mutable_data->version <= 7) {
            memset(&mutable_data->uart_pins, 0, sizeof(mutable_data->uart_pins));
        }
        // v8 -> v9: flash_roboclaw_config_t units grew by 2 bytes each
        // (estop_pin + uart_swap). Same fix as RP2040: zero everything
        // after roboclaw_config so misaligned bytes don't get
        // interpreted as live config. Operators re-sync once.
        if (mutable_data->version <= 8) {
            memset(&mutable_data->roboclaw_config, 0,
                   sizeof(mutable_data->roboclaw_config));
            memset(&mutable_data->pathfinder_bms_config, 0,
                   sizeof(mutable_data->pathfinder_bms_config));
            memset(&mutable_data->uart_pins, 0,
                   sizeof(mutable_data->uart_pins));
            mutable_data->fas100_config.enabled = 0;
        }
        // v9 -> v10: inserted flash_tic_config_t between
        // pathfinder_bms_config and uart_pins. Zero tic_config +
        // uart_pins (only Tic gets fresh defaults; earlier peripherals
        // sit before the inserted block and keep their offsets).
        if (mutable_data->version <= 9) {
            memset(&mutable_data->tic_config, 0,
                   sizeof(mutable_data->tic_config));
            memset(&mutable_data->uart_pins, 0,
                   sizeof(mutable_data->uart_pins));
        }
        // v10 -> v11: inserted flash_tmc2208_config_t between tic_config
        // and uart_pins. Same fix pattern.
        if (mutable_data->version <= 10) {
            memset(&mutable_data->tmc2208_config, 0,
                   sizeof(mutable_data->tmc2208_config));
            memset(&mutable_data->uart_pins, 0,
                   sizeof(mutable_data->uart_pins));
        }
        mutable_data->version = FLASH_STORAGE_VERSION;
    }

    Serial.printf("Flash storage: loaded config (version %d)\n", data->version);
    return true;
}

bool flash_storage_save(const flash_storage_data_t* data)
{
    if (!storage_initialized || !data) return false;

    volatile uint8_t* dst = (volatile uint8_t*)STORAGE_DATA_BASE;
    const uint8_t* src = (const uint8_t*)data;
    for (size_t i = 0; i < sizeof(flash_storage_data_t); i++) {
        dst[i] = src[i];
    }

    STORAGE_WRITE32(STORAGE_REG_CONTROL, STORAGE_CTRL_SAVE);
    delay(1);

    Serial.printf("Flash storage: saved config\n");
    return true;
}

bool flash_storage_erase(void)
{
    if (!storage_initialized) return false;

    STORAGE_WRITE32(STORAGE_REG_CONTROL, STORAGE_CTRL_ERASE);
    delay(1);

    Serial.printf("Flash storage: erased\n");
    return true;
}

bool flash_storage_has_config(void)
{
    if (!storage_initialized) return false;

    STORAGE_WRITE32(STORAGE_REG_CONTROL, STORAGE_CTRL_LOAD);
    delay(1);

    uint32_t status = STORAGE_READ32(STORAGE_REG_STATUS);
    return (status & STORAGE_STATUS_VALID) != 0;
}

} // extern "C"

#else
// =============================================================================
// Hardware: EEPROM storage
// =============================================================================

#include <EEPROM.h>

#define EEPROM_OFFSET 0

static bool storage_initialized = false;

extern "C" {

bool flash_storage_init(void)
{
    Serial.printf("Flash storage: EEPROM mode\n");
    storage_initialized = true;
    return true;
}

bool flash_storage_load(flash_storage_data_t* data)
{
    if (!storage_initialized || !data) return false;

    uint8_t* dst = (uint8_t*)data;
    for (size_t i = 0; i < sizeof(flash_storage_data_t); i++) {
        dst[i] = EEPROM.read(EEPROM_OFFSET + i);
    }

    if (data->magic != FLASH_STORAGE_MAGIC) {
        Serial.printf("Flash storage: no valid data (magic 0x%08lX)\n", (unsigned long)data->magic);
        return false;
    }

    if (data->version < FLASH_STORAGE_VERSION) {
        Serial.printf("Flash storage: migrating from version %d to %d\n",
                       data->version, FLASH_STORAGE_VERSION);
        if (data->version == 1) {
            memset(&data->pin_config, 0, sizeof(data->pin_config));
            data->pin_config.version = FLASH_PIN_CONFIG_VERSION;
            data->pin_config.pin_count = 0;
        }
        if (data->version <= 2) {
            memset(&data->maestro_config, 0, sizeof(data->maestro_config));
        }
        if (data->version <= 3) {
            memset(&data->syren_config, 0, sizeof(data->syren_config));
        }
        if (data->version <= 7) {
            memset(&data->uart_pins, 0, sizeof(data->uart_pins));
        }
        if (data->version <= 8) {
            memset(&data->roboclaw_config, 0, sizeof(data->roboclaw_config));
            memset(&data->pathfinder_bms_config, 0,
                   sizeof(data->pathfinder_bms_config));
            memset(&data->uart_pins, 0, sizeof(data->uart_pins));
            data->fas100_config.enabled = 0;
        }
        if (data->version <= 9) {
            memset(&data->tic_config, 0, sizeof(data->tic_config));
            memset(&data->uart_pins, 0, sizeof(data->uart_pins));
        }
        if (data->version <= 10) {
            memset(&data->tmc2208_config, 0, sizeof(data->tmc2208_config));
            memset(&data->uart_pins, 0, sizeof(data->uart_pins));
        }
        data->version = FLASH_STORAGE_VERSION;
    }

    Serial.printf("Flash storage: loaded config (version %d)\n", data->version);
    return true;
}

bool flash_storage_save(const flash_storage_data_t* data)
{
    if (!storage_initialized || !data) return false;

    const uint8_t* src = (const uint8_t*)data;
    for (size_t i = 0; i < sizeof(flash_storage_data_t); i++) {
        EEPROM.write(EEPROM_OFFSET + i, src[i]);
    }

    Serial.printf("Flash storage: saved config\n");
    return true;
}

bool flash_storage_erase(void)
{
    if (!storage_initialized) return false;

    for (size_t i = 0; i < sizeof(flash_storage_data_t); i++) {
        EEPROM.write(EEPROM_OFFSET + i, 0xFF);
    }

    Serial.printf("Flash storage: erased\n");
    return true;
}

bool flash_storage_has_config(void)
{
    if (!storage_initialized) return false;

    uint32_t magic;
    uint8_t* dst = (uint8_t*)&magic;
    for (size_t i = 0; i < sizeof(magic); i++) {
        dst[i] = EEPROM.read(EEPROM_OFFSET + i);
    }
    return magic == FLASH_STORAGE_MAGIC;
}

} // extern "C"

#endif // SIMULATION
