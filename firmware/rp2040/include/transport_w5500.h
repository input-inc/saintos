/**
 * SAINT.OS Node Firmware - W5500 Transport Header
 *
 * micro-ROS custom transport implementation for W5500 ethernet chip.
 */

#ifndef TRANSPORT_W5500_H
#define TRANSPORT_W5500_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

#include <uxr/client/transport.h>

// =============================================================================
// Transport Functions (for micro-ROS rmw_microros)
// =============================================================================

/**
 * Open transport connection.
 * Called by micro-ROS when initializing transport.
 */
bool transport_w5500_open(struct uxrCustomTransport* transport);

/**
 * Close transport connection.
 * Called by micro-ROS when shutting down.
 */
bool transport_w5500_close(struct uxrCustomTransport* transport);

/**
 * Write data to transport.
 * Called by micro-ROS to send data to agent.
 *
 * @param transport Transport handle
 * @param buf Data buffer to send
 * @param len Length of data
 * @param err Error code output
 * @return Number of bytes written
 */
size_t transport_w5500_write(
    struct uxrCustomTransport* transport,
    const uint8_t* buf,
    size_t len,
    uint8_t* err
);

/**
 * Read data from transport.
 * Called by micro-ROS to receive data from agent.
 *
 * @param transport Transport handle
 * @param buf Buffer to receive data
 * @param len Maximum length to read
 * @param timeout_ms Read timeout in milliseconds
 * @param err Error code output
 * @return Number of bytes read
 */
size_t transport_w5500_read(
    struct uxrCustomTransport* transport,
    uint8_t* buf,
    size_t len,
    int timeout_ms,
    uint8_t* err
);

// =============================================================================
// Initialization Functions
// =============================================================================

/**
 * Initialize W5500 ethernet hardware.
 *
 * @return true if successful
 */
bool transport_w5500_init(void);

/**
 * Connect to network (DHCP or static).
 *
 * @return true if connected
 */
bool transport_w5500_connect(void);

/**
 * Get MAC address.
 *
 * @param mac Output buffer (6 bytes)
 */
void transport_w5500_get_mac(uint8_t* mac);

/**
 * Get IP address.
 *
 * @param ip Output buffer (4 bytes)
 */
void transport_w5500_get_ip(uint8_t* ip);

/**
 * Check if network is connected.
 *
 * @return true if connected
 */
bool transport_w5500_is_connected(void);

/**
 * Set micro-ROS agent address.
 *
 * @param ip Agent IP address (4 bytes)
 * @param port Agent UDP port
 */
void transport_w5500_set_agent(const uint8_t* ip, uint16_t port);

/**
 * Pump the DHCP state machine.
 *
 * Must be called from the main loop after a successful connect — without
 * it, DHCP_time_handler() never advances dhcp_tick_1s and the ioLibrary
 * never sees the lease half-time threshold that triggers REREQUEST. The
 * function is cheap to call frequently: it only does work once per
 * second (internal throttle) and falls through quickly otherwise.
 */
void transport_w5500_tick(void);

#endif // TRANSPORT_W5500_H
