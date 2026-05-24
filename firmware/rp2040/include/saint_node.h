/**
 * SAINT.OS Node Firmware - RP2040 Main Header
 *
 * Includes shared types and adds RP2040-specific pin definitions.
 */

#ifndef SAINT_NODE_H
#define SAINT_NODE_H

#include "saint_types.h"

// =============================================================================
// Hardware Pin Definitions (Feather RP2040 + Ethernet FeatherWing)
// =============================================================================

// SPI pins for W5500 Ethernet
#define ETH_SPI_PORT    spi0
#define ETH_PIN_SCK     18
#define ETH_PIN_MOSI    19
#define ETH_PIN_MISO    20
#define ETH_PIN_CS      10
#define ETH_PIN_RST     11

// Onboard NeoPixel (WS2812)
#define NEOPIXEL_PIN    16
#define NEOPIXEL_COUNT  1

// Available GPIO for peripherals (after ethernet)
#define GPIO_A0         26
#define GPIO_A1         27
#define GPIO_A2         28
#define GPIO_A3         29
#define GPIO_D5         5
#define GPIO_D6         6
#define GPIO_D9         9
#define GPIO_D12        12
#define GPIO_D13        13
#define GPIO_D24        24
#define GPIO_D25        25

// I2C pins
#define I2C_SDA         2
#define I2C_SCL         3

// UART pins
#define UART_TX         0
#define UART_RX         1

#endif // SAINT_NODE_H
