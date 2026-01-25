/**
 * SAINT.OS Node Firmware - UDP Bridge Transport Implementation
 *
 * micro-ROS transport using Renode's UDP bridge peripheral.
 * This provides network connectivity in simulation without requiring
 * full W5500 ethernet emulation.
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "saint_node.h"
#include "transport_udp_bridge.h"

#include <uxr/client/transport.h>

// =============================================================================
// UDP Bridge Register Definitions
// =============================================================================

// Base address of the UDP bridge peripheral in Renode
#define UDP_BRIDGE_BASE     0x50300000

// Register offsets
#define UDP_REG_CONTROL     (UDP_BRIDGE_BASE + 0x00)
#define UDP_REG_TX_LEN      (UDP_BRIDGE_BASE + 0x04)
#define UDP_REG_RX_LEN      (UDP_BRIDGE_BASE + 0x08)
#define UDP_REG_REMOTE_IP   (UDP_BRIDGE_BASE + 0x0C)
#define UDP_REG_REMOTE_PORT (UDP_BRIDGE_BASE + 0x10)
#define UDP_REG_LOCAL_PORT  (UDP_BRIDGE_BASE + 0x14)
#define UDP_TX_BUFFER       (UDP_BRIDGE_BASE + 0x100)
#define UDP_RX_BUFFER       (UDP_BRIDGE_BASE + 0x600)

// Control register bits
#define UDP_CTRL_SEND       (1 << 0)
#define UDP_CTRL_OPEN       (1 << 1)
#define UDP_CTRL_CLOSE      (1 << 2)
#define UDP_CTRL_RECV       (1 << 3)
#define UDP_CTRL_SOCKET_OPEN (1 << 8)
#define UDP_CTRL_RX_AVAIL   (1 << 9)

// Helper macros for register access
#define UDP_WRITE32(addr, val) (*(volatile uint32_t*)(addr) = (val))
#define UDP_READ32(addr)       (*(volatile uint32_t*)(addr))

// =============================================================================
// State Variables
// =============================================================================

static bool initialized = false;
static bool socket_open = false;

static uint8_t local_ip[4] = {192, 168, 1, 100};  // Simulated node IP
static uint8_t agent_ip[4] = {127, 0, 0, 1};      // Agent on localhost (simulation)
static uint16_t agent_port = 8888;                // Default agent port
static uint16_t local_port = 9999;                // Local UDP port

// =============================================================================
// Helper Functions
// =============================================================================

static uint32_t ip_to_uint32(const uint8_t* ip)
{
    return (uint32_t)ip[0] | ((uint32_t)ip[1] << 8) |
           ((uint32_t)ip[2] << 16) | ((uint32_t)ip[3] << 24);
}

static void uint32_to_ip(uint32_t val, uint8_t* ip)
{
    ip[0] = val & 0xFF;
    ip[1] = (val >> 8) & 0xFF;
    ip[2] = (val >> 16) & 0xFF;
    ip[3] = (val >> 24) & 0xFF;
}

// =============================================================================
// Public Functions - Initialization
// =============================================================================

bool transport_udp_bridge_init(void)
{
    printf("Initializing UDP bridge transport...\n");

    // Set local port
    UDP_WRITE32(UDP_REG_LOCAL_PORT, local_port);

    // Open the socket
    UDP_WRITE32(UDP_REG_CONTROL, UDP_CTRL_OPEN);

    // Check if socket opened
    sleep_ms(10);
    uint32_t status = UDP_READ32(UDP_REG_CONTROL);
    if (!(status & UDP_CTRL_SOCKET_OPEN)) {
        printf("UDP bridge: Failed to open socket\n");
        return false;
    }

    printf("UDP bridge initialized (port %d)\n", local_port);
    initialized = true;
    socket_open = true;

    return true;
}

bool transport_udp_bridge_connect(void)
{
    if (!initialized) {
        return false;
    }

    // For UDP, we're always "connected" once the socket is open
    printf("UDP bridge ready\n");
    return socket_open;
}

void transport_udp_bridge_get_ip(uint8_t* ip)
{
    memcpy(ip, local_ip, 4);
}

void transport_udp_bridge_set_ip(const uint8_t* ip)
{
    memcpy(local_ip, ip, 4);
}

bool transport_udp_bridge_is_connected(void)
{
    if (!initialized) return false;
    uint32_t status = UDP_READ32(UDP_REG_CONTROL);
    return (status & UDP_CTRL_SOCKET_OPEN) != 0;
}

void transport_udp_bridge_set_agent(const uint8_t* ip, uint16_t port)
{
    memcpy(agent_ip, ip, 4);
    agent_port = port;

    // Update registers
    UDP_WRITE32(UDP_REG_REMOTE_IP, ip_to_uint32(agent_ip));
    UDP_WRITE32(UDP_REG_REMOTE_PORT, agent_port);
}

// =============================================================================
// micro-ROS Transport Interface
// =============================================================================

bool transport_udp_bridge_open(struct uxrCustomTransport* transport)
{
    (void)transport;

    if (!socket_open) {
        printf("Transport open failed: socket not open\n");
        return false;
    }

    // Set agent address
    UDP_WRITE32(UDP_REG_REMOTE_IP, ip_to_uint32(agent_ip));
    UDP_WRITE32(UDP_REG_REMOTE_PORT, agent_port);

    printf("micro-ROS transport opened (UDP %d.%d.%d.%d:%d)\n",
           agent_ip[0], agent_ip[1], agent_ip[2], agent_ip[3], agent_port);

    return true;
}

bool transport_udp_bridge_close(struct uxrCustomTransport* transport)
{
    (void)transport;
    printf("micro-ROS transport closed\n");
    return true;
}

size_t transport_udp_bridge_write(
    struct uxrCustomTransport* transport,
    const uint8_t* buf,
    size_t len,
    uint8_t* err)
{
    (void)transport;

    if (!socket_open || len == 0) {
        *err = 1;
        return 0;
    }

    // Limit to buffer size
    if (len > 1280) len = 1280;

    // Copy data to TX buffer (word-aligned writes)
    volatile uint32_t* tx_buf = (volatile uint32_t*)UDP_TX_BUFFER;
    const uint32_t* src = (const uint32_t*)buf;
    size_t words = (len + 3) / 4;

    for (size_t i = 0; i < words; i++) {
        tx_buf[i] = src[i];
    }

    // Set length and send
    UDP_WRITE32(UDP_REG_TX_LEN, len);
    UDP_WRITE32(UDP_REG_CONTROL, UDP_CTRL_SEND);

    *err = 0;
    return len;
}

size_t transport_udp_bridge_read(
    struct uxrCustomTransport* transport,
    uint8_t* buf,
    size_t len,
    int timeout_ms,
    uint8_t* err)
{
    (void)transport;

    if (!socket_open) {
        *err = 1;
        return 0;
    }

    uint32_t start = to_ms_since_boot(get_absolute_time());

    while (true) {
        // Trigger receive attempt
        UDP_WRITE32(UDP_REG_CONTROL, UDP_CTRL_RECV);

        // Check if data available
        uint32_t status = UDP_READ32(UDP_REG_CONTROL);
        if (status & UDP_CTRL_RX_AVAIL) {
            // Get received length
            uint32_t rx_len = UDP_READ32(UDP_REG_RX_LEN);
            if (rx_len > len) rx_len = len;
            if (rx_len > 1280) rx_len = 1280;

            // Copy from RX buffer
            volatile uint32_t* rx_buf = (volatile uint32_t*)UDP_RX_BUFFER;
            uint32_t* dst = (uint32_t*)buf;
            size_t words = (rx_len + 3) / 4;

            for (size_t i = 0; i < words; i++) {
                dst[i] = rx_buf[i];
            }

            *err = 0;
            return rx_len;
        }

        // Check timeout
        uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - start;
        if (elapsed >= (uint32_t)timeout_ms) {
            break;
        }

        // Small delay before retry
        sleep_ms(1);
    }

    // Timeout - not an error, just no data
    *err = 0;
    return 0;
}
