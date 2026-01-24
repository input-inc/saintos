/**
 * SAINT.OS Node Firmware - W5500 Transport Implementation
 *
 * micro-ROS custom transport over W5500 ethernet using UDP.
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

#include "saint_node.h"
#include "transport_w5500.h"
#include "w5500_driver.h"

#include <uxr/client/transport.h>

// =============================================================================
// Configuration
// =============================================================================

// Socket number to use for micro-ROS (W5500 has 8 sockets: 0-7)
#define UROS_SOCKET         0

// Local UDP port for micro-ROS client
#define UROS_LOCAL_PORT     9999

// =============================================================================
// State Variables
// =============================================================================

static bool initialized = false;
static bool connected = false;

static uint8_t mac_address[6] = {0};
static uint8_t local_ip[4] = {0};
static uint8_t agent_ip[4] = {192, 168, 1, 10};  // Default agent IP
static uint16_t agent_port = 8888;               // Default agent port

// =============================================================================
// W5500 Hardware Interface
// =============================================================================

/**
 * Initialize W5500 SPI and reset chip.
 */
static bool w5500_hw_init(void)
{
    // Initialize SPI at 33MHz (W5500 max is 80MHz, but be conservative)
    spi_init(ETH_SPI_PORT, 33000000);
    spi_set_format(ETH_SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // Configure SPI pins
    gpio_set_function(ETH_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(ETH_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(ETH_PIN_MISO, GPIO_FUNC_SPI);

    // Configure CS pin (directly controlled, not SPI hardware CS)
    gpio_init(ETH_PIN_CS);
    gpio_set_dir(ETH_PIN_CS, GPIO_OUT);
    gpio_put(ETH_PIN_CS, 1);  // Deselect

    // Configure and perform hardware reset
    gpio_init(ETH_PIN_RST);
    gpio_set_dir(ETH_PIN_RST, GPIO_OUT);

    // Reset pulse
    gpio_put(ETH_PIN_RST, 0);
    sleep_ms(10);
    gpio_put(ETH_PIN_RST, 1);
    sleep_ms(50);

    return true;
}

// =============================================================================
// Public Functions - Initialization
// =============================================================================

bool transport_w5500_init(void)
{
    printf("Initializing W5500...\n");

    // Initialize hardware
    if (!w5500_hw_init()) {
        printf("W5500 hardware init failed\n");
        return false;
    }

    // Initialize W5500 driver
    if (!w5500_init()) {
        printf("W5500 driver init failed\n");
        return false;
    }

    // Generate MAC address from unique ID
    // Use locally administered, unicast MAC (02:xx:xx:xx:xx:xx)
    char unique_id[16];
    hardware_get_unique_id(unique_id, sizeof(unique_id));

    mac_address[0] = 0x02;  // Locally administered
    for (int i = 0; i < 5 && unique_id[i*2]; i++) {
        char hex[3] = {unique_id[i*2], unique_id[i*2+1], 0};
        mac_address[i+1] = (uint8_t)strtol(hex, NULL, 16);
    }

    // Set MAC address in W5500
    w5500_set_mac(mac_address);

    printf("W5500 initialized\n");
    initialized = true;

    return true;
}

bool transport_w5500_connect(void)
{
    if (!initialized) {
        return false;
    }

    printf("Connecting to network...\n");

    if (g_node.use_dhcp) {
        // DHCP
        printf("Requesting IP via DHCP...\n");

        if (!w5500_dhcp_request(local_ip, 10000)) {  // 10 second timeout
            printf("DHCP failed\n");
            return false;
        }
    } else {
        // Static IP
        memcpy(local_ip, g_node.static_ip, 4);
        w5500_set_ip(local_ip);
        w5500_set_subnet((uint8_t[]){255, 255, 255, 0});
        w5500_set_gateway(g_node.gateway);
    }

    printf("IP: %d.%d.%d.%d\n",
           local_ip[0], local_ip[1], local_ip[2], local_ip[3]);

    // Check link status
    if (!w5500_get_link_status()) {
        printf("No ethernet link\n");
        return false;
    }

    printf("Ethernet connected\n");
    connected = true;

    return true;
}

void transport_w5500_get_mac(uint8_t* mac)
{
    memcpy(mac, mac_address, 6);
}

void transport_w5500_get_ip(uint8_t* ip)
{
    memcpy(ip, local_ip, 4);
}

bool transport_w5500_is_connected(void)
{
    return connected && w5500_get_link_status();
}

void transport_w5500_set_agent(const uint8_t* ip, uint16_t port)
{
    memcpy(agent_ip, ip, 4);
    agent_port = port;
}

// =============================================================================
// micro-ROS Transport Interface
// =============================================================================

bool transport_w5500_open(struct uxrCustomTransport* transport)
{
    (void)transport;

    if (!connected) {
        printf("Transport open failed: not connected\n");
        return false;
    }

    // Open UDP socket for micro-ROS
    if (!w5500_socket_open(UROS_SOCKET, W5500_SOCK_UDP, UROS_LOCAL_PORT)) {
        printf("Failed to open UDP socket\n");
        return false;
    }

    printf("micro-ROS transport opened (UDP %d.%d.%d.%d:%d)\n",
           agent_ip[0], agent_ip[1], agent_ip[2], agent_ip[3], agent_port);

    return true;
}

bool transport_w5500_close(struct uxrCustomTransport* transport)
{
    (void)transport;

    w5500_socket_close(UROS_SOCKET);
    printf("micro-ROS transport closed\n");

    return true;
}

size_t transport_w5500_write(
    struct uxrCustomTransport* transport,
    const uint8_t* buf,
    size_t len,
    uint8_t* err)
{
    (void)transport;

    if (!connected) {
        *err = 1;
        return 0;
    }

    // Send UDP packet to agent
    int sent = w5500_socket_sendto(
        UROS_SOCKET,
        buf, len,
        agent_ip, agent_port
    );

    if (sent < 0) {
        *err = 1;
        return 0;
    }

    *err = 0;
    return (size_t)sent;
}

size_t transport_w5500_read(
    struct uxrCustomTransport* transport,
    uint8_t* buf,
    size_t len,
    int timeout_ms,
    uint8_t* err)
{
    (void)transport;

    if (!connected) {
        *err = 1;
        return 0;
    }

    // Check for available data with timeout
    uint32_t start = to_ms_since_boot(get_absolute_time());

    while (true) {
        int available = w5500_socket_available(UROS_SOCKET);

        if (available > 0) {
            // Read UDP packet
            uint8_t remote_ip[4];
            uint16_t remote_port;

            int received = w5500_socket_recvfrom(
                UROS_SOCKET,
                buf, len,
                remote_ip, &remote_port
            );

            if (received > 0) {
                *err = 0;
                return (size_t)received;
            }
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
