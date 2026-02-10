/**
 * SAINT.OS Node Firmware - Teensy 4.1 Main Header
 *
 * Includes shared types and adds Teensy 4.1-specific pin definitions.
 */

#ifndef SAINT_NODE_H
#define SAINT_NODE_H

#include "saint_types.h"

// =============================================================================
// Teensy 4.1 Hardware Pin Definitions
// =============================================================================

// Built-in Ethernet (uses internal PHY, no GPIO consumed)

// Onboard LED
#define LED_PIN         13

// Available GPIO for peripherals
// Digital pins (most are PWM capable)
#define GPIO_D0         0
#define GPIO_D1         1
#define GPIO_D2         2
#define GPIO_D3         3
#define GPIO_D4         4
#define GPIO_D5         5
#define GPIO_D6         6
#define GPIO_D7         7
#define GPIO_D8         8
#define GPIO_D9         9
#define GPIO_D10        10
#define GPIO_D11        11
#define GPIO_D12        12
#define GPIO_D13        13
#define GPIO_D14        14
#define GPIO_D15        15
#define GPIO_D22        22
#define GPIO_D23        23
#define GPIO_D28        28
#define GPIO_D29        29
#define GPIO_D30        30
#define GPIO_D31        31
#define GPIO_D32        32
#define GPIO_D33        33

// Analog pins (A0-A13 = pins 14-27, A14-A17 = pins 38-41)
#define GPIO_A0         14
#define GPIO_A1         15
#define GPIO_A2         16
#define GPIO_A3         17
#define GPIO_A4         18
#define GPIO_A5         19
#define GPIO_A6         20
#define GPIO_A7         21
#define GPIO_A8         22
#define GPIO_A9         23
#define GPIO_A10        24
#define GPIO_A11        25
#define GPIO_A12        26
#define GPIO_A13        27
#define GPIO_A14        38
#define GPIO_A15        39
#define GPIO_A16        40
#define GPIO_A17        41

// I2C pins (Wire)
#define I2C_SDA         18
#define I2C_SCL         19

// UART pins (Serial1)
#define UART_TX         1
#define UART_RX         0

#endif // SAINT_NODE_H
