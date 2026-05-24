/**
 * SAINT.OS Node Firmware - WIZnet ioLibrary Platform Port
 *
 * Header for WIZnet ioLibrary platform callbacks on RP2040.
 */

#ifndef WIZCHIP_PORT_H
#define WIZCHIP_PORT_H

#include <stdbool.h>
#include <stdint.h>

/**
 * Initialize the WIZnet chip platform layer.
 * Sets up SPI, GPIO, and registers callbacks with ioLibrary.
 *
 * @return true if successful
 */
bool wizchip_port_init(void);

/**
 * Check physical link status.
 *
 * @return true if link is up
 */
bool wizchip_port_link_up(void);

#endif // WIZCHIP_PORT_H
