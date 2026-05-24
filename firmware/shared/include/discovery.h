/**
 * SAINT.OS Firmware - Discovery Protocol Constants
 *
 * UDP broadcast discovery protocol for finding the SAINT server.
 * Shared constants; implementation is per-platform.
 */

#ifndef DISCOVERY_H
#define DISCOVERY_H

#include <stdbool.h>
#include <stdint.h>

// Discovery protocol constants
#define DISCOVERY_PORT              8889
#define DISCOVERY_REQUEST           "SAINT?"
#define DISCOVERY_RESPONSE_PREFIX   "SAINT!"
#define DISCOVERY_LOCAL_PORT        8890

bool discover_server(uint8_t* server_ip, uint16_t* server_port,
                     uint32_t timeout_ms, int max_attempts);

#endif // DISCOVERY_H
