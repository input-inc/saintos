/**
 * SAINT.OS Node Firmware - Server Discovery
 *
 * UDP broadcast discovery to find the SAINT server on the local network.
 *
 * Protocol:
 *   Request:  "SAINT?" (broadcast to UDP port 8889)
 *   Response: "SAINT!<ip>:<port>" (e.g., "SAINT!192.168.0.104:8888")
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "saint_node.h"
#include "discovery.h"

// WIZnet ioLibrary
#include "wizchip_conf.h"
#include "socket.h"

// Discovery protocol constants
#define DISCOVERY_PORT          8889
#define DISCOVERY_REQUEST       "SAINT?"
#define DISCOVERY_RESPONSE_PREFIX "SAINT!"
#define DISCOVERY_SOCKET        7       // Use socket 7 for discovery (0 is for micro-ROS)
#define DISCOVERY_LOCAL_PORT    8890    // Local port for receiving responses

// Broadcast address
static uint8_t BROADCAST_ADDR[4] = {255, 255, 255, 255};

/**
 * Parse discovery response and extract IP and port.
 *
 * Response format: "SAINT!<ip>:<port>"
 * Example: "SAINT!192.168.0.104:8888"
 *
 * Returns true if parsing succeeded.
 */
static bool parse_discovery_response(const char* response, uint8_t* ip_out, uint16_t* port_out)
{
    // Check prefix
    const char* prefix = DISCOVERY_RESPONSE_PREFIX;
    size_t prefix_len = strlen(prefix);

    if (strncmp(response, prefix, prefix_len) != 0) {
        return false;
    }

    // Parse IP address (after prefix)
    const char* ip_start = response + prefix_len;
    const char* colon = strchr(ip_start, ':');

    if (!colon) {
        return false;
    }

    // Parse IP octets
    int ip[4];
    int parsed = sscanf(ip_start, "%d.%d.%d.%d", &ip[0], &ip[1], &ip[2], &ip[3]);
    if (parsed != 4) {
        return false;
    }

    // Validate octets
    for (int i = 0; i < 4; i++) {
        if (ip[i] < 0 || ip[i] > 255) {
            return false;
        }
        ip_out[i] = (uint8_t)ip[i];
    }

    // Parse port (after colon)
    int port = atoi(colon + 1);
    if (port <= 0 || port > 65535) {
        return false;
    }
    *port_out = (uint16_t)port;

    return true;
}

/**
 * Discover the SAINT server via UDP broadcast.
 *
 * Sends broadcast discovery requests and waits for a response.
 *
 * @param server_ip     Output: discovered server IP address
 * @param server_port   Output: discovered agent port
 * @param timeout_ms    Maximum time to wait for response (per attempt)
 * @param max_attempts  Maximum number of broadcast attempts
 *
 * @return true if server was discovered, false on timeout
 */
bool discover_server(uint8_t* server_ip, uint16_t* server_port,
                     uint32_t timeout_ms, int max_attempts)
{
    printf("Starting server discovery (port %d)...\n", DISCOVERY_PORT);

    // Open UDP socket for discovery
    int8_t sock_result = socket(DISCOVERY_SOCKET, Sn_MR_UDP, DISCOVERY_LOCAL_PORT, 0);
    if (sock_result != DISCOVERY_SOCKET) {
        printf("Failed to open discovery socket: %d\n", sock_result);
        return false;
    }

    const char* request = DISCOVERY_REQUEST;
    size_t request_len = strlen(request);

    for (int attempt = 0; attempt < max_attempts; attempt++) {
        printf("Discovery attempt %d/%d...\n", attempt + 1, max_attempts);

        // Send broadcast
        int32_t sent = sendto(DISCOVERY_SOCKET,
                              (uint8_t*)request, request_len,
                              BROADCAST_ADDR, DISCOVERY_PORT);

        if (sent != (int32_t)request_len) {
            printf("Failed to send discovery broadcast: %ld\n", (long)sent);
            sleep_ms(500);
            continue;
        }

        // Wait for response
        uint32_t start_time = to_ms_since_boot(get_absolute_time());

        while (to_ms_since_boot(get_absolute_time()) - start_time < timeout_ms) {
            // Check if data available
            int32_t available = getSn_RX_RSR(DISCOVERY_SOCKET);

            if (available > 0) {
                uint8_t buffer[128];
                uint8_t src_ip[4];
                uint16_t src_port;

                int32_t received = recvfrom(DISCOVERY_SOCKET,
                                            buffer, sizeof(buffer) - 1,
                                            src_ip, &src_port);

                if (received > 0) {
                    buffer[received] = '\0';  // Null-terminate

                    printf("Discovery response from %d.%d.%d.%d:%d: %s\n",
                           src_ip[0], src_ip[1], src_ip[2], src_ip[3], src_port,
                           (char*)buffer);

                    // Parse response
                    if (parse_discovery_response((char*)buffer, server_ip, server_port)) {
                        printf("Discovered server at %d.%d.%d.%d:%d\n",
                               server_ip[0], server_ip[1], server_ip[2], server_ip[3],
                               *server_port);

                        // Close discovery socket
                        close(DISCOVERY_SOCKET);
                        return true;
                    } else {
                        printf("Invalid discovery response format\n");
                    }
                }
            }

            sleep_ms(10);  // Small delay to prevent busy-waiting
        }

        printf("Discovery timeout, retrying...\n");
    }

    // Close discovery socket
    close(DISCOVERY_SOCKET);

    printf("Server discovery failed after %d attempts\n", max_attempts);
    return false;
}
