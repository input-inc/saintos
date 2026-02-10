/**
 * SAINT.OS Node Firmware - Teensy 4.1 NativeEthernet Transport
 *
 * micro-ROS transport using Teensy 4.1's built-in Ethernet PHY
 * via the NativeEthernet library.
 */

#ifndef TRANSPORT_NATIVE_ETH_H
#define TRANSPORT_NATIVE_ETH_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <uxr/client/transport.h>

bool transport_native_eth_open(struct uxrCustomTransport* transport);
bool transport_native_eth_close(struct uxrCustomTransport* transport);
size_t transport_native_eth_write(struct uxrCustomTransport* transport, const uint8_t* buf, size_t len, uint8_t* err);
size_t transport_native_eth_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout_ms, uint8_t* err);

bool transport_native_eth_init(void);
bool transport_native_eth_connect(void);
void transport_native_eth_get_mac(uint8_t* mac);
void transport_native_eth_get_ip(uint8_t* ip);
bool transport_native_eth_is_connected(void);
void transport_native_eth_set_agent(const uint8_t* ip, uint16_t port);

#ifdef __cplusplus
}
#endif

#endif // TRANSPORT_NATIVE_ETH_H
