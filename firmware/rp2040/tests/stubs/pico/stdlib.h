/* Host-test stub for pico/stdlib.h — see tests/run_tests.sh.
 * Only what firmware/rp2040/src/pin_config.c actually reaches for. */
#ifndef SAINT_TEST_STUB_PICO_STDLIB_H
#define SAINT_TEST_STUB_PICO_STDLIB_H
#include <stdint.h>
#include <stdbool.h>
typedef unsigned int uint;
static inline void sleep_ms(uint32_t ms) { (void)ms; }
#endif
