/**
 * SAINT.OS Firmware - UART Pin Pair Table
 *
 * Each platform exposes a list of legal (uart_instance, tx_pin, rx_pin) tuples
 * that drivers can validate against and the capabilities JSON can advertise to
 * the web UI.
 *
 * Numbering:
 *   - RP2040:    uart_instance 0 = UART0,    1 = UART1
 *   - Teensy 41: uart_instance 1 = Serial1,  ..., 8 = Serial8
 *
 * Pin numbers are platform-native GPIO numbers (RP2040 GPIO# / Teensy pin#).
 */

#ifndef UART_PIN_PAIRS_H
#define UART_PIN_PAIRS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t uart_instance;
    uint8_t tx_pin;
    uint8_t rx_pin;
} uart_pin_pair_t;

/**
 * Returns the platform's UART pin pair table.
 * Sets *count to the number of entries.
 */
const uart_pin_pair_t* uart_pin_pairs_table(size_t* count);

/**
 * Look up the UART instance for a given (tx, rx) pin pair.
 * Returns true and sets *out_instance if the pair is valid.
 */
bool uart_pin_pair_lookup(uint8_t tx_pin, uint8_t rx_pin, uint8_t* out_instance);

/**
 * Get the platform-default (tx, rx) pair for a UART instance.
 * Returns the first table entry matching uart_instance, or false if none.
 */
bool uart_pin_pair_default(uint8_t uart_instance, uint8_t* out_tx, uint8_t* out_rx);

/**
 * Parse "tx_pin": N and "rx_pin": N fields out of a JSON snippet between
 * json_start and json_end. Both pins must be present and form a legal UART
 * pin pair (verified via uart_pin_pair_lookup) for the call to succeed.
 *
 * On success: sets *out_tx, *out_rx, *out_instance and returns true.
 * On any failure (missing field, invalid pair): returns false; output
 * parameters are left untouched.
 */
bool uart_pin_pair_parse_json(const char* json_start, const char* json_end,
                               uint8_t* out_tx, uint8_t* out_rx,
                               uint8_t* out_instance);

#ifdef __cplusplus
}
#endif

#endif // UART_PIN_PAIRS_H
