/**
 * SAINT.OS Node Firmware - RP2040 UART Pin Pair Table
 *
 * Per the RP2040 datasheet, each UART can be routed to one of four GPIO
 * pin pairs (function 2 in the IOMUX). The first entry per UART instance
 * is the platform default.
 */

#include "uart_pin_pairs.h"

static const uart_pin_pair_t rp2040_uart_pairs[] = {
    // UART0
    { 0,  0,  1 },   // default: TX=GP0,  RX=GP1   (Feather TX/RX pads)
    { 0, 12, 13 },   // alt:     TX=GP12, RX=GP13  (Feather D12/D13)
    { 0, 16, 17 },   // alt:     TX=GP16, RX=GP17  (GP16 is NeoPixel on Feather)
    { 0, 28, 29 },   // alt:     TX=GP28, RX=GP29  (Feather A2/A3)

    // UART1
    { 1,  4,  5 },   // default: TX=GP4,  RX=GP5   (Feather D4/D5 — D4 not broken out)
    { 1,  8,  9 },   // alt:     TX=GP8,  RX=GP9   (GP8 not broken out on Feather)
    { 1, 20, 21 },   // alt:     TX=GP20, RX=GP21  (GP20 is ETH MISO on Feather)
    { 1, 24, 25 },   // alt:     TX=GP24, RX=GP25  (Feather D24/D25)
};

#define RP2040_UART_PAIR_COUNT (sizeof(rp2040_uart_pairs) / sizeof(rp2040_uart_pairs[0]))

const uart_pin_pair_t* uart_pin_pairs_table(size_t* count)
{
    if (count) *count = RP2040_UART_PAIR_COUNT;
    return rp2040_uart_pairs;
}

bool uart_pin_pair_lookup(uint8_t tx_pin, uint8_t rx_pin, uint8_t* out_instance)
{
    for (size_t i = 0; i < RP2040_UART_PAIR_COUNT; i++) {
        if (rp2040_uart_pairs[i].tx_pin == tx_pin &&
            rp2040_uart_pairs[i].rx_pin == rx_pin) {
            if (out_instance) *out_instance = rp2040_uart_pairs[i].uart_instance;
            return true;
        }
    }
    return false;
}

bool uart_pin_pair_default(uint8_t uart_instance, uint8_t* out_tx, uint8_t* out_rx)
{
    for (size_t i = 0; i < RP2040_UART_PAIR_COUNT; i++) {
        if (rp2040_uart_pairs[i].uart_instance == uart_instance) {
            if (out_tx) *out_tx = rp2040_uart_pairs[i].tx_pin;
            if (out_rx) *out_rx = rp2040_uart_pairs[i].rx_pin;
            return true;
        }
    }
    return false;
}
