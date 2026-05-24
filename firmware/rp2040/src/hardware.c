/**
 * SAINT.OS Node Firmware - Hardware Abstraction
 *
 * Handles RP2040 hardware initialization and access.
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

#include "saint_node.h"

// =============================================================================
// Constants
// =============================================================================

// ADC channel for internal temperature sensor
#define ADC_TEMP_CHANNEL    4

// Conversion factor for temperature (from RP2040 datasheet)
#define ADC_TEMP_FACTOR     (3.3f / (1 << 12))

// =============================================================================
// Public Functions
// =============================================================================

/**
 * Initialize hardware peripherals.
 */
void hardware_init(void)
{
    // Initialize ADC for temperature reading
    adc_init();
    adc_set_temp_sensor_enabled(true);

    // Initialize onboard LED (GPIO13 on Feather RP2040)
    gpio_init(GPIO_D13);
    gpio_set_dir(GPIO_D13, GPIO_OUT);

    printf("Hardware initialized\n");
}

/**
 * Update hardware state.
 * Called from main loop.
 */
void hardware_update(void)
{
    // Nothing to do currently
    // Role-specific hardware updates would go here
}

/**
 * Get unique board ID.
 *
 * Returns the RP2040's unique flash ID as a hex string.
 *
 * @param buffer Output buffer for ID string
 * @param len Buffer length
 * @return Length of ID string
 */
uint32_t hardware_get_unique_id(char* buffer, size_t len)
{
    pico_unique_board_id_t board_id;
    pico_get_unique_board_id(&board_id);

    // Format as hex string (last 6 bytes for shorter ID)
    int written = snprintf(buffer, len, "%02x%02x%02x%02x%02x%02x",
                           board_id.id[2], board_id.id[3],
                           board_id.id[4], board_id.id[5],
                           board_id.id[6], board_id.id[7]);

    return (written > 0) ? written : 0;
}

/**
 * Get CPU temperature in Celsius.
 */
float hardware_get_cpu_temp(void)
{
    // Select ADC temperature channel
    adc_select_input(ADC_TEMP_CHANNEL);

    // Read ADC
    uint16_t raw = adc_read();

    // Convert to voltage
    float voltage = raw * ADC_TEMP_FACTOR;

    // Convert to temperature (from RP2040 datasheet)
    // T = 27 - (V - 0.706) / 0.001721
    float temperature = 27.0f - (voltage - 0.706f) / 0.001721f;

    return temperature;
}
