/**
 * SAINT.OS Firmware - UDP Bridge Transport Header
 *
 * micro-ROS transport using Renode's UDP bridge peripheral.
 * Shared between all platforms for simulation builds.
 */

#ifndef TRANSPORT_UDP_BRIDGE_H
#define TRANSPORT_UDP_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <uxr/client/transport.h>

#ifdef __cplusplus
extern "C" {
#endif

bool transport_udp_bridge_init(void);
bool transport_udp_bridge_connect(void);
void transport_udp_bridge_get_ip(uint8_t* ip);
void transport_udp_bridge_set_ip(const uint8_t* ip);
bool transport_udp_bridge_is_connected(void);
void transport_udp_bridge_set_agent(const uint8_t* ip, uint16_t port);

bool transport_udp_bridge_open(struct uxrCustomTransport* transport);
bool transport_udp_bridge_close(struct uxrCustomTransport* transport);
size_t transport_udp_bridge_write(
    struct uxrCustomTransport* transport,
    const uint8_t* buf,
    size_t len,
    uint8_t* err);
size_t transport_udp_bridge_read(
    struct uxrCustomTransport* transport,
    uint8_t* buf,
    size_t len,
    int timeout_ms,
    uint8_t* err);

#ifdef __cplusplus
}
#endif

#endif // TRANSPORT_UDP_BRIDGE_H
