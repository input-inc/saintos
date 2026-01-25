/**
 * SAINT.OS Node Firmware - UDP Bridge Transport Header
 *
 * micro-ROS transport using Renode's UDP bridge peripheral.
 */

#ifndef TRANSPORT_UDP_BRIDGE_H
#define TRANSPORT_UDP_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <uxr/client/transport.h>

/**
 * Initialize the UDP bridge transport.
 * Opens a UDP socket on the configured local port.
 *
 * @return true if initialization succeeded
 */
bool transport_udp_bridge_init(void);

/**
 * Connect the UDP bridge transport.
 * For UDP, this succeeds if the socket is open.
 *
 * @return true if connected
 */
bool transport_udp_bridge_connect(void);

/**
 * Get the local IP address.
 *
 * @param ip Buffer to store IP (4 bytes)
 */
void transport_udp_bridge_get_ip(uint8_t* ip);

/**
 * Set the local IP address.
 *
 * @param ip IP address (4 bytes)
 */
void transport_udp_bridge_set_ip(const uint8_t* ip);

/**
 * Check if transport is connected.
 *
 * @return true if connected
 */
bool transport_udp_bridge_is_connected(void);

/**
 * Set the micro-ROS agent address.
 *
 * @param ip Agent IP address (4 bytes)
 * @param port Agent UDP port
 */
void transport_udp_bridge_set_agent(const uint8_t* ip, uint16_t port);

/**
 * micro-ROS transport open callback.
 */
bool transport_udp_bridge_open(struct uxrCustomTransport* transport);

/**
 * micro-ROS transport close callback.
 */
bool transport_udp_bridge_close(struct uxrCustomTransport* transport);

/**
 * micro-ROS transport write callback.
 */
size_t transport_udp_bridge_write(
    struct uxrCustomTransport* transport,
    const uint8_t* buf,
    size_t len,
    uint8_t* err);

/**
 * micro-ROS transport read callback.
 */
size_t transport_udp_bridge_read(
    struct uxrCustomTransport* transport,
    uint8_t* buf,
    size_t len,
    int timeout_ms,
    uint8_t* err);

#endif // TRANSPORT_UDP_BRIDGE_H
