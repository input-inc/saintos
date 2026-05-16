/**
 * SAINT.OS Node Firmware - Teensy 4.1 UART Pin Pair Table
 *
 * Teensy 4.1 has 8 hardware serial ports (Serial1..Serial8), each on a fixed
 * pair of GPIO pins. Per the Teensy 4.1 reference pinout:
 *
 *   Serial1: TX=0,  RX=1
 *   Serial2: TX=8,  RX=7    (note: TX=8, RX=7 — flipped from RP2040 convention)
 *   Serial3: TX=14, RX=15
 *   Serial4: TX=17, RX=16
 *   Serial5: TX=20, RX=21
 *   Serial6: TX=24, RX=25
 *   Serial7: TX=29, RX=28
 *   Serial8: TX=35, RX=34
 */

#include <Arduino.h>

extern "C" {
#include "uart_pin_pairs.h"
}

// Map uart_instance (1..8) to the corresponding Teensy HardwareSerial object.
// Returns nullptr if instance is out of range.
HardwareSerial* teensy_serial_from_instance(uint8_t inst)
{
    switch (inst) {
        case 1: return &Serial1;
        case 2: return &Serial2;
        case 3: return &Serial3;
        case 4: return &Serial4;
        case 5: return &Serial5;
        case 6: return &Serial6;
        case 7: return &Serial7;
        case 8: return &Serial8;
        default: return nullptr;
    }
}

static const uart_pin_pair_t teensy_uart_pairs[] = {
    { 1,  0,  1 },
    { 2,  8,  7 },
    { 3, 14, 15 },
    { 4, 17, 16 },
    { 5, 20, 21 },
    { 6, 24, 25 },
    { 7, 29, 28 },
    { 8, 35, 34 },
};

#define TEENSY_UART_PAIR_COUNT (sizeof(teensy_uart_pairs) / sizeof(teensy_uart_pairs[0]))

extern "C" const uart_pin_pair_t* uart_pin_pairs_table(size_t* count)
{
    if (count) *count = TEENSY_UART_PAIR_COUNT;
    return teensy_uart_pairs;
}

extern "C" bool uart_pin_pair_lookup(uint8_t tx_pin, uint8_t rx_pin, uint8_t* out_instance)
{
    for (size_t i = 0; i < TEENSY_UART_PAIR_COUNT; i++) {
        if (teensy_uart_pairs[i].tx_pin == tx_pin &&
            teensy_uart_pairs[i].rx_pin == rx_pin) {
            if (out_instance) *out_instance = teensy_uart_pairs[i].uart_instance;
            return true;
        }
    }
    return false;
}

extern "C" bool uart_pin_pair_default(uint8_t uart_instance, uint8_t* out_tx, uint8_t* out_rx)
{
    for (size_t i = 0; i < TEENSY_UART_PAIR_COUNT; i++) {
        if (teensy_uart_pairs[i].uart_instance == uart_instance) {
            if (out_tx) *out_tx = teensy_uart_pairs[i].tx_pin;
            if (out_rx) *out_rx = teensy_uart_pairs[i].rx_pin;
            return true;
        }
    }
    return false;
}
