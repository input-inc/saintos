/**
 * SAINT.OS Node Firmware - Teensy 4.1 HardwareSerial Lookup
 *
 * Maps a UART instance number (1..8) to the corresponding Teensy
 * HardwareSerial reference. C++ only; pairs with uart_pin_pairs.h.
 */

#ifndef UART_SERIAL_LOOKUP_H
#define UART_SERIAL_LOOKUP_H

#include <Arduino.h>
#include <stdint.h>

HardwareSerial* teensy_serial_from_instance(uint8_t inst);

#endif // UART_SERIAL_LOOKUP_H
