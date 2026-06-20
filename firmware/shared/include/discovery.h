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

#ifdef __cplusplus
extern "C" {
#endif

// Discovery protocol constants
#define DISCOVERY_PORT              8889
#define DISCOVERY_REQUEST           "SAINT?"
#define DISCOVERY_RESPONSE_PREFIX   "SAINT!"
#define DISCOVERY_LOCAL_PORT        8890

bool discover_server(uint8_t* server_ip, uint16_t* server_port,
                     uint32_t timeout_ms, int max_attempts);

/**
 * Retry discover_server() forever with backoff until the server is
 * found. Returns only on success. Designed for cold-boot situations
 * where the server and the node powered up together and the server
 * is still booting (Pi takes 30-60s to come up); without retry the
 * node would strand itself in NODE_STATE_ERROR while the operator
 * is still watching the Pi boot.
 *
 * Cadence:
 *   - First 3 batches: 2 s backoff between batches (fast pickup once
 *     the server lands).
 *   - After that: 5 s backoff (sane log volume if the network is
 *     actually down).
 *
 * Each batch tries `attempts_per_batch` discovery requests with
 * `attempt_timeout_ms` per attempt.
 *
 * Drives PLATFORM_LED_TICK() and PLATFORM_WATCHDOG_FEED() inside the
 * backoff sleep so the LED keeps animating and the hardware watchdog
 * doesn't reset the chip while we wait.
 */
void discover_server_retry_forever(uint8_t* server_ip, uint16_t* server_port,
                                    uint32_t attempt_timeout_ms,
                                    int attempts_per_batch);

#ifdef __cplusplus
}
#endif

#endif // DISCOVERY_H
