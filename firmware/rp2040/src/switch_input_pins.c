/**
 * SAINT.OS Firmware - switch_input pin access (RP2040)
 *
 * Per-platform half of the generic switch/sensor input driver. See
 * firmware/shared/src/switch_input_driver.c and docs/SENSOR_INPUTS.md.
 *
 * Deliberately not routed through pin_control_read_digital/_read_adc:
 * those require the pin to be registered under PIN_MODE_DIGITAL_IN /
 * PIN_MODE_ADC, but a switch_input pin is claimed under
 * PIN_MODE_SWITCH_INPUT and would be rejected.
 */

#include <stdbool.h>
#include <stdint.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include "switch_input_driver.h"

/* RP2040 exposes ADC only on GP26-29 (channels 0-3). GP29 is commonly
 * tied to board functions, but the map is the silicon's, so accept the
 * full range and let pin assignment be the operator's problem. */
#define RP2040_ADC_FIRST_GPIO   26
#define RP2040_ADC_LAST_GPIO    29
#define RP2040_ADC_VREF_MV      3300
#define RP2040_ADC_FULL_SCALE   4095    /* 12-bit */

static bool adc_ready = false;

bool switch_input_read_digital_pin(uint8_t pin, bool pull_up)
{
    if (pin > 29) return false;
    /* gpio_init is idempotent and cheap; calling it per read keeps this
     * correct if the pin was reassigned by a config sync. */
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    if (pull_up) gpio_pull_up(pin);
    else         gpio_pull_down(pin);
    return gpio_get(pin);
}

bool switch_input_read_analog_mv(uint8_t pin, uint16_t* out_mv)
{
    if (!out_mv) return false;
    if (pin < RP2040_ADC_FIRST_GPIO || pin > RP2040_ADC_LAST_GPIO) return false;

    if (!adc_ready) {
        adc_init();
        adc_ready = true;
    }
    adc_gpio_init(pin);
    adc_select_input((uint)(pin - RP2040_ADC_FIRST_GPIO));

    uint32_t raw = adc_read();
    *out_mv = (uint16_t)((raw * RP2040_ADC_VREF_MV) / RP2040_ADC_FULL_SCALE);
    return true;
}
