/**
 * SAINT.OS Firmware - NativeEthernet HTTP transport for http_client (Teensy 4.1)
 *
 * Plug for the shared http_client API, backed by NativeEthernet's
 * EthernetClient. NativeEthernet must already be initialized (DHCP or
 * static) before opening — typically that happens at app startup as
 * part of the micro-ROS transport bring-up.
 */

#ifndef SAINT_HTTP_TRANSPORT_ETHERNET_H
#define SAINT_HTTP_TRANSPORT_ETHERNET_H

#include <stdint.h>
#include "http_client.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Per-instance state. The body is opaque to non-Arduino C — the .cpp
 *  implementation casts it to its own struct. We expose just the
 *  storage size needed. */
typedef struct {
    /* Storage large enough to hold an EthernetClient (which is small —
     * a few pointers + state). 32 words is generous overhead. */
    uint32_t opaque[32];
} http_ethernet_state_t;

/** Initialize an http_transport_t backed by an EthernetClient. After
 *  this returns, pass the transport pointer to http_get(). */
void http_transport_ethernet_init(http_transport_t* out,
                                   http_ethernet_state_t* state);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_HTTP_TRANSPORT_ETHERNET_H */
