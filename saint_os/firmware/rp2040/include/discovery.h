/**
 * SAINT.OS Node Firmware - Server Discovery
 *
 * UDP broadcast discovery to find the SAINT server on the local network.
 */

#ifndef DISCOVERY_H
#define DISCOVERY_H

#include <stdbool.h>
#include <stdint.h>

/**
 * Discover the SAINT server via UDP broadcast.
 *
 * Sends broadcast discovery requests and waits for a response.
 *
 * @param server_ip     Output: discovered server IP address (4 bytes)
 * @param server_port   Output: discovered agent port
 * @param timeout_ms    Maximum time to wait for response per attempt (default: 2000)
 * @param max_attempts  Maximum number of broadcast attempts (default: 5)
 *
 * @return true if server was discovered, false on timeout
 */
bool discover_server(uint8_t* server_ip, uint16_t* server_port,
                     uint32_t timeout_ms, int max_attempts);

#endif // DISCOVERY_H
