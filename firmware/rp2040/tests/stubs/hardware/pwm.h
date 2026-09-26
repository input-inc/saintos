/* Host-test stub for hardware/pwm.h. */
#ifndef SAINT_TEST_STUB_HARDWARE_PWM_H
#define SAINT_TEST_STUB_HARDWARE_PWM_H
#include "pico/stdlib.h"
static inline uint pwm_gpio_to_slice_num(uint g) { return g / 2u; }
static inline uint pwm_gpio_to_channel(uint g)   { return g % 2u; }
static inline void pwm_set_wrap(uint s, uint16_t w)            { (void)s; (void)w; }
static inline void pwm_set_clkdiv(uint s, float d)             { (void)s; (void)d; }
static inline void pwm_set_chan_level(uint s, uint c, uint16_t l) { (void)s; (void)c; (void)l; }
static inline void pwm_set_enabled(uint s, bool e)             { (void)s; (void)e; }
#endif
