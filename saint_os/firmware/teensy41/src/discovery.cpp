/**
 * SAINT.OS Node Firmware - Server Discovery (Teensy 4.1)
 *
 * UDP broadcast discovery to find the SAINT server.
 */

#include <Arduino.h>

#ifndef SIMULATION

#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

extern "C" {
#include "saint_node.h"
#include "discovery.h"
}

static EthernetUDP discovery_udp;

static bool parse_discovery_response(const char* response, uint8_t* ip_out, uint16_t* port_out)
{
    const char* prefix = DISCOVERY_RESPONSE_PREFIX;
    size_t prefix_len = strlen(prefix);

    if (strncmp(response, prefix, prefix_len) != 0) {
        return false;
    }

    const char* ip_start = response + prefix_len;
    const char* colon = strchr(ip_start, ':');
    if (!colon) return false;

    int ip[4];
    int parsed = sscanf(ip_start, "%d.%d.%d.%d", &ip[0], &ip[1], &ip[2], &ip[3]);
    if (parsed != 4) return false;

    for (int i = 0; i < 4; i++) {
        if (ip[i] < 0 || ip[i] > 255) return false;
        ip_out[i] = (uint8_t)ip[i];
    }

    int port = atoi(colon + 1);
    if (port <= 0 || port > 65535) return false;
    *port_out = (uint16_t)port;

    return true;
}

extern "C" {

bool discover_server(uint8_t* server_ip, uint16_t* server_port,
                     uint32_t timeout_ms, int max_attempts)
{
    Serial.printf("Starting server discovery (port %d)...\n", DISCOVERY_PORT);

    discovery_udp.begin(DISCOVERY_LOCAL_PORT);

    const char* request = DISCOVERY_REQUEST;
    size_t request_len = strlen(request);

    for (int attempt = 0; attempt < max_attempts; attempt++) {
        Serial.printf("Discovery attempt %d/%d...\n", attempt + 1, max_attempts);

        // Send broadcast
        IPAddress broadcast(255, 255, 255, 255);
        discovery_udp.beginPacket(broadcast, DISCOVERY_PORT);
        discovery_udp.write((const uint8_t*)request, request_len);
        discovery_udp.endPacket();

        // Wait for response
        uint32_t start_time = millis();

        while (millis() - start_time < timeout_ms) {
            int packet_size = discovery_udp.parsePacket();
            if (packet_size > 0) {
                char buffer[128];
                int received = discovery_udp.read(buffer, sizeof(buffer) - 1);
                if (received > 0) {
                    buffer[received] = '\0';

                    IPAddress remote = discovery_udp.remoteIP();
                    Serial.printf("Discovery response from %d.%d.%d.%d: %s\n",
                                   remote[0], remote[1], remote[2], remote[3],
                                   buffer);

                    if (parse_discovery_response(buffer, server_ip, server_port)) {
                        Serial.printf("Discovered server at %d.%d.%d.%d:%d\n",
                                       server_ip[0], server_ip[1], server_ip[2],
                                       server_ip[3], *server_port);
                        discovery_udp.stop();
                        return true;
                    }
                }
            }
            delay(10);
        }

        Serial.printf("Discovery timeout, retrying...\n");
    }

    discovery_udp.stop();
    Serial.printf("Server discovery failed after %d attempts\n", max_attempts);
    return false;
}

} // extern "C"

#endif // !SIMULATION
