/**
 * SAINT.OS Firmware - W5500 TCP transport for http_client (RP2040)
 *
 * Plug for the shared http_client API. Backed by the W5500 ioLibrary's
 * hardware TCP sockets — the W5500 must already be initialized (MAC
 * set, IP assigned via DHCP or static) before calling http_get().
 *
 * Used by the OTA bootloader; the app firmware does not link this.
 */

#ifndef SAINT_HTTP_TRANSPORT_W5500_H
#define SAINT_HTTP_TRANSPORT_W5500_H

#include <stdint.h>
#include "http_client.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Per-instance state for the W5500 HTTP transport. */
typedef struct http_w5500_state {
    uint8_t socket_num;          /* W5500 hardware socket (0-7) */
    uint16_t local_port;         /* Source port for outgoing connection */
    int      opened;             /* 1 once socket() succeeded */
    uint32_t recv_deadline_ms;   /* Set by recv() loop, used for timeout */
} http_w5500_state_t;

/** Initialize an http_transport_t backed by a W5500 hardware socket.
 *  After this returns, pass the transport pointer to http_get(). */
void http_transport_w5500_init(http_transport_t* out,
                                http_w5500_state_t* state,
                                uint8_t socket_num,
                                uint16_t local_port);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_HTTP_TRANSPORT_W5500_H */
