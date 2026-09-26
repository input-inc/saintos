/* Host-test stub for hardware/adc.h. */
#ifndef SAINT_TEST_STUB_HARDWARE_ADC_H
#define SAINT_TEST_STUB_HARDWARE_ADC_H
#include "pico/stdlib.h"
static inline void adc_init(void)         { }
static inline void adc_gpio_init(uint g)  { (void)g; }
#endif
