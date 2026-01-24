/**
 * SAINT.OS Node Firmware - W5500 Driver Header
 *
 * Low-level driver for Wiznet W5500 Ethernet chip.
 */

#ifndef W5500_DRIVER_H
#define W5500_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// =============================================================================
// W5500 Constants
// =============================================================================

// Socket types
#define W5500_SOCK_CLOSED   0x00
#define W5500_SOCK_TCP      0x01
#define W5500_SOCK_UDP      0x02
#define W5500_SOCK_MACRAW   0x04

// Socket states
#define W5500_SOCK_ESTABLISHED  0x17
#define W5500_SOCK_CLOSE_WAIT   0x1C
#define W5500_SOCK_UDP_STATE    0x22

// =============================================================================
// Initialization
// =============================================================================

/**
 * Initialize W5500 chip.
 * SPI must already be configured.
 */
bool w5500_init(void);

/**
 * Software reset W5500.
 */
void w5500_reset(void);

// =============================================================================
// Configuration
// =============================================================================

/**
 * Set MAC address.
 */
void w5500_set_mac(const uint8_t* mac);

/**
 * Get MAC address.
 */
void w5500_get_mac(uint8_t* mac);

/**
 * Set IP address.
 */
void w5500_set_ip(const uint8_t* ip);

/**
 * Get IP address.
 */
void w5500_get_ip(uint8_t* ip);

/**
 * Set subnet mask.
 */
void w5500_set_subnet(const uint8_t* subnet);

/**
 * Set gateway IP.
 */
void w5500_set_gateway(const uint8_t* gateway);

// =============================================================================
// Network Status
// =============================================================================

/**
 * Check physical link status.
 */
bool w5500_get_link_status(void);

/**
 * Get link speed (10/100 Mbps).
 */
uint8_t w5500_get_link_speed(void);

// =============================================================================
// DHCP
// =============================================================================

/**
 * Request IP address via DHCP.
 *
 * @param ip_out Output buffer for received IP (4 bytes)
 * @param timeout_ms Timeout in milliseconds
 * @return true if successful
 */
bool w5500_dhcp_request(uint8_t* ip_out, uint32_t timeout_ms);

// =============================================================================
// Socket Operations
// =============================================================================

/**
 * Open a socket.
 *
 * @param socket Socket number (0-7)
 * @param type Socket type (W5500_SOCK_*)
 * @param port Local port number
 * @return true if successful
 */
bool w5500_socket_open(uint8_t socket, uint8_t type, uint16_t port);

/**
 * Close a socket.
 */
void w5500_socket_close(uint8_t socket);

/**
 * Get socket state.
 */
uint8_t w5500_socket_status(uint8_t socket);

/**
 * Get available bytes to read.
 */
int w5500_socket_available(uint8_t socket);

/**
 * Send UDP datagram.
 *
 * @param socket Socket number
 * @param data Data to send
 * @param len Length of data
 * @param dest_ip Destination IP address
 * @param dest_port Destination port
 * @return Bytes sent, or -1 on error
 */
int w5500_socket_sendto(
    uint8_t socket,
    const uint8_t* data,
    size_t len,
    const uint8_t* dest_ip,
    uint16_t dest_port
);

/**
 * Receive UDP datagram.
 *
 * @param socket Socket number
 * @param buf Buffer to receive data
 * @param len Maximum bytes to receive
 * @param src_ip Source IP address output
 * @param src_port Source port output
 * @return Bytes received, or -1 on error
 */
int w5500_socket_recvfrom(
    uint8_t socket,
    uint8_t* buf,
    size_t len,
    uint8_t* src_ip,
    uint16_t* src_port
);

// =============================================================================
// Low-level SPI Interface
// =============================================================================

/**
 * Write to W5500 register.
 */
void w5500_write_byte(uint16_t addr, uint8_t block, uint8_t data);

/**
 * Read from W5500 register.
 */
uint8_t w5500_read_byte(uint16_t addr, uint8_t block);

/**
 * Write multiple bytes.
 */
void w5500_write_buf(uint16_t addr, uint8_t block, const uint8_t* data, size_t len);

/**
 * Read multiple bytes.
 */
void w5500_read_buf(uint16_t addr, uint8_t block, uint8_t* data, size_t len);

#endif // W5500_DRIVER_H
