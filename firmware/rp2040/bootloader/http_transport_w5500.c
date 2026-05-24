/**
 * SAINT.OS Firmware - W5500 TCP transport for http_client (RP2040)
 *
 * Drives a single ioLibrary hardware socket as an http_client transport.
 * See http_transport_w5500.h for the contract.
 */

#include "http_transport_w5500.h"

#include <string.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "socket.h"           /* WIZnet ioLibrary */
#include "wizchip_conf.h"

/* Hard timeouts. The bootloader is single-threaded so we just spin. */
#define CONNECT_TIMEOUT_MS  10000
#define RECV_TIMEOUT_MS      5000
#define POLL_DELAY_MS           5

/* Parse a numeric "A.B.C.D" string into a 4-byte array. Returns 1 on
 * success, 0 on malformed input. Hostnames (non-numeric) are not
 * supported — the bootloader gets the server IP from DHCP / the app's
 * handoff, so we don't need a resolver. */
static int parse_ipv4(const char* s, uint8_t out[4])
{
    if (!s) return 0;
    int octets = 0;
    int value = 0;
    int digits = 0;
    while (*s) {
        if (*s >= '0' && *s <= '9') {
            value = value * 10 + (*s - '0');
            if (value > 255) return 0;
            digits++;
        } else if (*s == '.') {
            if (digits == 0 || octets >= 3) return 0;
            out[octets++] = (uint8_t)value;
            value = 0;
            digits = 0;
        } else {
            return 0;
        }
        s++;
    }
    if (octets != 3 || digits == 0) return 0;
    out[3] = (uint8_t)value;
    return 1;
}

/* --- transport callbacks ---------------------------------------------- */

static int w5500_t_open(void* ctx, const char* host, uint16_t port)
{
    http_w5500_state_t* st = (http_w5500_state_t*)ctx;

    uint8_t ip[4];
    if (!parse_ipv4(host, ip)) {
        printf("http-w5500: malformed host '%s'\n", host);
        return -1;
    }

    /* Open the socket as TCP client. */
    int8_t r = socket(st->socket_num, Sn_MR_TCP, st->local_port, 0);
    if (r != st->socket_num) {
        printf("http-w5500: socket() failed (%d)\n", r);
        return -1;
    }
    st->opened = 1;

    /* Block until connected or timeout. ioLibrary's connect() blocks
     * internally already, but it has its own configured timeout that
     * we can't easily change here. */
    int32_t cr = connect(st->socket_num, ip, port);
    if (cr != SOCK_OK) {
        printf("http-w5500: connect() failed (%ld)\n", (long)cr);
        close(st->socket_num);
        st->opened = 0;
        return -1;
    }
    return 0;
}

static int w5500_t_send(void* ctx, const uint8_t* buf, size_t len)
{
    http_w5500_state_t* st = (http_w5500_state_t*)ctx;
    if (!st->opened) return -1;
    int32_t n = send(st->socket_num, (uint8_t*)buf, (uint16_t)len);
    if (n < 0) {
        printf("http-w5500: send() error (%ld)\n", (long)n);
        return -1;
    }
    return (int)n;
}

static int w5500_t_recv(void* ctx, uint8_t* buf, size_t len)
{
    http_w5500_state_t* st = (http_w5500_state_t*)ctx;
    if (!st->opened) return -1;

    uint32_t start = to_ms_since_boot(get_absolute_time());
    for (;;) {
        uint8_t status = getSn_SR(st->socket_num);
        uint16_t avail = getSn_RX_RSR(st->socket_num);

        if (avail > 0) {
            if (avail > len) avail = (uint16_t)len;
            int32_t n = recv(st->socket_num, buf, avail);
            if (n < 0) {
                printf("http-w5500: recv() error (%ld)\n", (long)n);
                return -1;
            }
            return (int)n;
        }

        if (status == SOCK_CLOSE_WAIT || status == SOCK_CLOSED) {
            /* peer closed and buffer is drained */
            return 0;
        }

        if ((to_ms_since_boot(get_absolute_time()) - start) >= RECV_TIMEOUT_MS) {
            printf("http-w5500: recv timeout\n");
            return -1;
        }
        sleep_ms(POLL_DELAY_MS);
    }
}

static void w5500_t_close(void* ctx)
{
    http_w5500_state_t* st = (http_w5500_state_t*)ctx;
    if (st->opened) {
        disconnect(st->socket_num);
        close(st->socket_num);
        st->opened = 0;
    }
}

/* --- public init -------------------------------------------------------- */

void http_transport_w5500_init(http_transport_t* out,
                                http_w5500_state_t* state,
                                uint8_t socket_num,
                                uint16_t local_port)
{
    memset(state, 0, sizeof(*state));
    state->socket_num = socket_num;
    state->local_port = local_port;

    out->open  = w5500_t_open;
    out->send  = w5500_t_send;
    out->recv  = w5500_t_recv;
    out->close = w5500_t_close;
    out->ctx   = state;
}
