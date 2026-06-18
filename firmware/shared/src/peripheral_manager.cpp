/**
 * SAINT.OS Firmware - Peripheral Manager
 *
 * Registration and dispatch for modular peripheral drivers.
 */

#include "peripheral_driver.h"
#include "platform.h"
#include <stdio.h>
#include <string.h>

// =============================================================================
// State
// =============================================================================

static const peripheral_driver_t* drivers[PERIPHERAL_MAX_DRIVERS];
static uint8_t driver_count = 0;

// =============================================================================
// Registration
// =============================================================================

bool peripheral_register(const peripheral_driver_t* driver)
{
    if (!driver || driver_count >= PERIPHERAL_MAX_DRIVERS) {
        return false;
    }

    drivers[driver_count++] = driver;
    PLATFORM_PRINTF("Peripheral: registered '%s' (GPIO %d-%d, %d channels)\n",
                    driver->name,
                    driver->virtual_gpio_base,
                    driver->virtual_gpio_base + driver->channel_count - 1,
                    driver->channel_count);
    return true;
}

// =============================================================================
// Lifecycle
// =============================================================================

void peripheral_init_all(void)
{
    for (uint8_t i = 0; i < driver_count; i++) {
        if (drivers[i]->init) {
            drivers[i]->init();
        }
    }
}

void peripheral_update_all(void)
{
    for (uint8_t i = 0; i < driver_count; i++) {
        if (drivers[i]->update) {
            drivers[i]->update();
        }
    }
}

void peripheral_estop_all(void)
{
    for (uint8_t i = 0; i < driver_count; i++) {
        if (drivers[i]->estop) {
            drivers[i]->estop();
        }
    }
}

void peripheral_clear_estop_all(void)
{
    for (uint8_t i = 0; i < driver_count; i++) {
        if (drivers[i]->clear_estop) {
            drivers[i]->clear_estop();
        }
    }
}

// =============================================================================
// Lookup
// =============================================================================

const peripheral_driver_t* peripheral_find_by_gpio(uint16_t gpio)
{
    for (uint8_t i = 0; i < driver_count; i++) {
        uint16_t base = drivers[i]->virtual_gpio_base;
        uint16_t end = base + drivers[i]->channel_count;
        if (gpio >= base && gpio < end) {
            return drivers[i];
        }
    }
    return NULL;
}

const peripheral_driver_t* peripheral_find_by_mode(pin_mode_t mode)
{
    for (uint8_t i = 0; i < driver_count; i++) {
        if (drivers[i]->pin_mode == mode) {
            return drivers[i];
        }
    }
    return NULL;
}

const peripheral_driver_t* peripheral_find_by_mode_string(const char* mode_str)
{
    if (!mode_str) return NULL;
    for (uint8_t i = 0; i < driver_count; i++) {
        if (drivers[i]->mode_string && strcmp(drivers[i]->mode_string, mode_str) == 0) {
            return drivers[i];
        }
    }
    return NULL;
}

uint8_t peripheral_gpio_to_channel(const peripheral_driver_t* drv, uint16_t gpio)
{
    if (!drv) return 0xFF;
    return (uint8_t)(gpio - drv->virtual_gpio_base);
}

bool peripheral_is_virtual_gpio(uint16_t gpio)
{
    return peripheral_find_by_gpio(gpio) != NULL;
}

// =============================================================================
// Iteration
// =============================================================================

uint8_t peripheral_get_count(void)
{
    return driver_count;
}

// =============================================================================
// Channel-addressed state emission (peripheral-first migration; Phase 1)
// =============================================================================
//
// Each migrated driver implements `state_emit_channels(buf, cap, first)`
// to append zero-or-more `{"peripheral_id":..,"channel_id":..,"value":..}`
// records to a shared `channels[]` array in the outbound state JSON.
// The helpers below let drivers stay decoupled from the array bookkeeping:
// they just call peripheral_state_append_channel() per record, and the
// `first` flag handed in (shared across every driver this tick) tracks
// whether to prefix a comma.
//
// See docs/PERIPHERAL_FIRST_MIGRATION.md for the broader plan; this is
// the entry point Phase 1 (per-driver migration) hooks into.

int peripheral_state_append_channel(char* buf, size_t cap, bool* first,
                                    const char* peripheral_id,
                                    const char* channel_id,
                                    float value)
{
    if (!buf || !first || !peripheral_id || !channel_id) return -1;
    int n = snprintf(buf, cap,
                     "%s{\"peripheral_id\":\"%s\",\"channel_id\":\"%s\",\"value\":%.4f}",
                     *first ? "" : ",",
                     peripheral_id, channel_id, (double)value);
    if (n < 0 || (size_t)n >= cap) return -1;
    *first = false;
    return n;
}

int peripheral_state_emit_all_channels(char* buf, size_t cap)
{
    if (!buf) return -1;
    bool first = true;
    int total = 0;
    for (uint8_t i = 0; i < driver_count; i++) {
        if (!drivers[i] || !drivers[i]->state_emit_channels) continue;
        int n = drivers[i]->state_emit_channels(buf + total, cap - (size_t)total, &first);
        if (n < 0) return -1;
        total += n;
    }
    return total;
}

const peripheral_driver_t* peripheral_get(uint8_t index)
{
    if (index >= driver_count) return NULL;
    return drivers[index];
}
