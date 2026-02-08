/**
 * SAINT.OS Node Firmware - W5500 Transport Implementation
 *
 * micro-ROS custom transport over W5500 ethernet using UDP.
 * Uses WIZnet ioLibrary for DHCP and socket operations.
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

#include "saint_node.h"
#include "transport_w5500.h"
#include "wizchip_port.h"

// WIZnet ioLibrary
#include "wizchip_conf.h"
#include "socket.h"
#include "dhcp.h"

#include <uxr/client/transport.h>

// =============================================================================
// Configuration
// =============================================================================

// Socket allocation (W5500 has 8 sockets: 0-7)
#define DHCP_SOCKET         6       // Socket for DHCP
#define UROS_SOCKET         0       // Socket for micro-ROS

// Local UDP port for micro-ROS client
#define UROS_LOCAL_PORT     9999

// DHCP configuration
#define DHCP_TIMEOUT_MS     10000   // 10 second DHCP timeout
#define DHCP_RETRY_COUNT    5       // Number of DHCP retries

// =============================================================================
// State Variables
// =============================================================================

static bool initialized = false;
static bool connected = false;

static uint8_t mac_address[6] = {0};
static uint8_t local_ip[4] = {0};
static uint8_t subnet_mask[4] = {255, 255, 255, 0};
static uint8_t gateway[4] = {0};
static uint8_t dns_server[4] = {0};
static uint8_t agent_ip[4] = {0};
static uint16_t agent_port = 8888;

// DHCP buffer (must be at least 548 bytes)
static uint8_t dhcp_buffer[548];

// DHCP state
static volatile bool dhcp_got_ip = false;
static volatile uint32_t dhcp_timer_1s = 0;

// =============================================================================
// DHCP Callbacks
// =============================================================================

static void dhcp_ip_assign(void)
{
    getIPfromDHCP(local_ip);
    getGWfromDHCP(gateway);
    getSNfromDHCP(subnet_mask);
    getDNSfromDHCP(dns_server);

    // Apply network settings to W5500
    setSIPR(local_ip);
    setGAR(gateway);
    setSUBR(subnet_mask);

    printf("DHCP assigned IP: %d.%d.%d.%d\n",
           local_ip[0], local_ip[1], local_ip[2], local_ip[3]);
    printf("  Gateway: %d.%d.%d.%d\n",
           gateway[0], gateway[1], gateway[2], gateway[3]);
    printf("  Subnet:  %d.%d.%d.%d\n",
           subnet_mask[0], subnet_mask[1], subnet_mask[2], subnet_mask[3]);

    dhcp_got_ip = true;
}

static void dhcp_ip_update(void)
{
    printf("DHCP IP updated\n");
    dhcp_ip_assign();
}

static void dhcp_ip_conflict(void)
{
    printf("DHCP IP conflict detected!\n");
    dhcp_got_ip = false;
}

// =============================================================================
// DHCP Timer (called from main loop)
// =============================================================================

static uint32_t last_tick_time = 0;

static void dhcp_timer_tick(void)
{
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_tick_time >= 1000) {
        last_tick_time = now;
        DHCP_time_handler();
        dhcp_timer_1s++;
    }
}

// =============================================================================
// DHCP Process
// =============================================================================

static bool run_dhcp(uint32_t timeout_ms)
{
    printf("Starting DHCP...\n");

    // Initialize DHCP
    DHCP_init(DHCP_SOCKET, dhcp_buffer);
    reg_dhcp_cbfunc(dhcp_ip_assign, dhcp_ip_update, dhcp_ip_conflict);

    dhcp_got_ip = false;
    dhcp_timer_1s = 0;
    last_tick_time = to_ms_since_boot(get_absolute_time());

    uint32_t start_time = to_ms_since_boot(get_absolute_time());

    while (!dhcp_got_ip) {
        // Run DHCP state machine
        uint8_t dhcp_status = DHCP_run();

        switch (dhcp_status) {
            case DHCP_IP_ASSIGN:
            case DHCP_IP_CHANGED:
            case DHCP_IP_LEASED:
                // Success!
                printf("DHCP successful\n");
                return true;

            case DHCP_FAILED:
                printf("DHCP failed\n");
                return false;

            case DHCP_STOPPED:
                printf("DHCP stopped\n");
                return false;

            case DHCP_RUNNING:
            default:
                // Still in progress
                break;
        }

        // Update DHCP timer
        dhcp_timer_tick();

        // Check timeout
        uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - start_time;
        if (elapsed >= timeout_ms) {
            printf("DHCP timeout after %lu ms\n", elapsed);
            DHCP_stop();
            return false;
        }

        // Small delay
        sleep_ms(10);
    }

    return dhcp_got_ip;
}

// =============================================================================
// Static IP Fallback
// =============================================================================

static void use_static_ip(void)
{
    printf("Using static IP fallback\n");

    // Use compile-time defaults
    local_ip[0] = DEFAULT_NODE_IP_0;
    local_ip[1] = DEFAULT_NODE_IP_1;
    local_ip[2] = DEFAULT_NODE_IP_2;
    local_ip[3] = DEFAULT_NODE_IP_3;

    gateway[0] = DEFAULT_GATEWAY_IP_0;
    gateway[1] = DEFAULT_GATEWAY_IP_1;
    gateway[2] = DEFAULT_GATEWAY_IP_2;
    gateway[3] = DEFAULT_GATEWAY_IP_3;

    subnet_mask[0] = 255;
    subnet_mask[1] = 255;
    subnet_mask[2] = 255;
    subnet_mask[3] = 0;

    // Apply to W5500
    setSIPR(local_ip);
    setGAR(gateway);
    setSUBR(subnet_mask);

    printf("Static IP: %d.%d.%d.%d\n",
           local_ip[0], local_ip[1], local_ip[2], local_ip[3]);
}

// =============================================================================
// Public Functions - Initialization
// =============================================================================

bool transport_w5500_init(void)
{
    printf("Initializing W5500 (ioLibrary)...\n");

    // Initialize platform (SPI, GPIO, callbacks)
    if (!wizchip_port_init()) {
        printf("W5500 platform init failed\n");
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
    setSHAR(mac_address);

    printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
           mac_address[0], mac_address[1], mac_address[2],
           mac_address[3], mac_address[4], mac_address[5]);

    initialized = true;
    return true;
}

bool transport_w5500_connect(void)
{
    if (!initialized) {
        return false;
    }

    printf("Connecting to network...\n");

    // Wait for link
    int link_wait = 0;
    while (!wizchip_port_link_up()) {
        sleep_ms(100);
        link_wait++;
        if (link_wait > 50) {  // 5 second timeout
            printf("No ethernet link\n");
            return false;
        }
    }
    printf("Ethernet link detected\n");

    // Try DHCP first
    if (g_node.use_dhcp) {
        if (run_dhcp(DHCP_TIMEOUT_MS)) {
            connected = true;
            return true;
        }
        printf("DHCP failed, falling back to static IP\n");
    }

    // Static IP fallback
    use_static_ip();
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
    return connected && wizchip_port_link_up();
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
    int8_t result = socket(UROS_SOCKET, Sn_MR_UDP, UROS_LOCAL_PORT, 0);
    if (result != UROS_SOCKET) {
        printf("Failed to open UDP socket: %d\n", result);
        return false;
    }

    printf("micro-ROS transport opened (UDP %d.%d.%d.%d:%d)\n",
           agent_ip[0], agent_ip[1], agent_ip[2], agent_ip[3], agent_port);

    return true;
}

bool transport_w5500_close(struct uxrCustomTransport* transport)
{
    (void)transport;

    close(UROS_SOCKET);
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
    int32_t sent = sendto(UROS_SOCKET, (uint8_t*)buf, len, agent_ip, agent_port);

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
        int32_t available = getSn_RX_RSR(UROS_SOCKET);

        if (available > 0) {
            // Read UDP packet
            uint8_t remote_ip[4];
            uint16_t remote_port;

            int32_t received = recvfrom(UROS_SOCKET, buf, len, remote_ip, &remote_port);

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
