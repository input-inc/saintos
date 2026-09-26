/* Host-test stub for hardware/gpio.h. Every call is recorded-or-ignored;
 * the boot-reload test only cares about the peripheral-driver
 * delegation, not the pad muxing. */
#ifndef SAINT_TEST_STUB_HARDWARE_GPIO_H
#define SAINT_TEST_STUB_HARDWARE_GPIO_H
#include "pico/stdlib.h"
#define GPIO_IN  0
#define GPIO_OUT 1
enum gpio_function {
    GPIO_FUNC_XIP = 0, GPIO_FUNC_SPI = 1, GPIO_FUNC_UART = 2,
    GPIO_FUNC_I2C = 3, GPIO_FUNC_PWM = 4, GPIO_FUNC_SIO = 5,
    GPIO_FUNC_PIO0 = 6, GPIO_FUNC_PIO1 = 7, GPIO_FUNC_NULL = 0x1f,
};
static inline void gpio_init(uint g)                 { (void)g; }
static inline void gpio_deinit(uint g)               { (void)g; }
static inline void gpio_set_dir(uint g, bool out)    { (void)g; (void)out; }
static inline void gpio_put(uint g, bool v)          { (void)g; (void)v; }
static inline void gpio_pull_up(uint g)              { (void)g; }
static inline void gpio_pull_down(uint g)            { (void)g; }
static inline void gpio_set_function(uint g, enum gpio_function f) { (void)g; (void)f; }
static inline enum gpio_function gpio_get_function(uint g) { (void)g; return GPIO_FUNC_NULL; }
#endif
