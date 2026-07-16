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
#define ETH_PIN_CS      10   // default chip-select (standard Feather + Ethernet FeatherWing)
#define ETH_PIN_CS_ALT  24   // alternate CS (silk D24) for boards where pin 10 is repurposed
// NOTE: no hardware RST pin. The W5500 is reset in software over SPI during
// init (ioLibrary CW_INIT_WIZCHIP -> wizchip_sw_reset -> MR_RST), so a
// dedicated reset GPIO is unnecessary. The active CS is auto-detected at
// boot from {ETH_PIN_CS_ALT, ETH_PIN_CS} — see transport/wizchip_port.c.

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
