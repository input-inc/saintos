/**
 * SAINT.OS Firmware - switch_input pin access (Teensy 4.1)
 *
 * Per-platform half of the generic switch/sensor input driver. See
 * firmware/shared/src/switch_input_driver.c and docs/SENSOR_INPUTS.md.
 *
 * Deliberately not routed through pin_control_read_digital/_read_adc:
 * those require the pin to be registered under PIN_MODE_DIGITAL_IN /
 * PIN_MODE_ADC, but a switch_input pin is claimed under
 * PIN_MODE_SWITCH_INPUT and would be rejected.
 */

#include <Arduino.h>

extern "C" {
#include "switch_input_driver.h"
}

// Matches pin_control.cpp so a voltage read here and one from the
// generic ADC path agree.
#define TEENSY_ADC_VREF_MV     3300
#define TEENSY_ADC_MAX_VALUE   4096    // 12-bit

// Teensy 4.1 analog-capable pins are A0-A17 == digital 14-27 and 38-41.
static inline bool teensy_pin_is_analog(uint8_t pin)
{
    return (pin >= 14 && pin <= 27) || (pin >= 38 && pin <= 41);
}

extern "C" bool switch_input_read_digital_pin(uint8_t pin, bool pull_up)
{
    if (pin > 41) return false;
    // Re-applying the mode per read keeps this correct if a config sync
    // reassigned the pin; pinMode on Teensy is cheap.
    pinMode(pin, pull_up ? INPUT_PULLUP : INPUT_PULLDOWN);
    return digitalReadFast(pin) != 0;
}

extern "C" bool switch_input_read_analog_mv(uint8_t pin, uint16_t* out_mv)
{
    if (!out_mv) return false;
    if (!teensy_pin_is_analog(pin)) return false;

    // analogReadResolution is set to 12 bits in pin_control.cpp's init;
    // set it here too so this works even if the switch is the only
    // analog consumer on the node.
    analogReadResolution(12);
    uint32_t raw = (uint32_t)analogRead(pin);
    *out_mv = (uint16_t)((raw * TEENSY_ADC_VREF_MV) / TEENSY_ADC_MAX_VALUE);
    return true;
}
