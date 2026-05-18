/**
 * SAINT.OS Firmware - NativeEthernet HTTP transport (Teensy 4.1)
 *
 * Backs the shared http_client API with NativeEthernet's EthernetClient.
 * One global EthernetClient stored in the caller-provided opaque state
 * region (placement-new'd so the transport stays C-only externally).
 */

#include "http_transport_ethernet.h"

#include <Arduino.h>
#include <NativeEthernet.h>

#include <stdint.h>
#include <string.h>

#define RECV_TIMEOUT_MS 5000

/* Backing state stored inside http_ethernet_state_t.opaque[]. */
struct ethernet_state {
    EthernetClient client;
    bool opened;
};
static_assert(sizeof(ethernet_state) <= sizeof(((http_ethernet_state_t*)0)->opaque),
              "http_ethernet_state_t opaque storage too small");

static ethernet_state* as_state(void* ctx)
{
    return reinterpret_cast<ethernet_state*>(ctx);
}

/* Parse "A.B.C.D" — same numeric form the bootloader uses. */
static bool parse_ipv4(const char* s, IPAddress& out)
{
    if (!s) return false;
    int parts[4] = {0, 0, 0, 0};
    int idx = 0;
    int value = 0;
    int digits = 0;
    while (*s) {
        if (*s >= '0' && *s <= '9') {
            value = value * 10 + (*s - '0');
            if (value > 255) return false;
            digits++;
        } else if (*s == '.') {
            if (digits == 0 || idx >= 3) return false;
            parts[idx++] = value;
            value = 0;
            digits = 0;
        } else {
            return false;
        }
        s++;
    }
    if (idx != 3 || digits == 0) return false;
    parts[3] = value;
    out = IPAddress((uint8_t)parts[0], (uint8_t)parts[1],
                    (uint8_t)parts[2], (uint8_t)parts[3]);
    return true;
}

/* ── transport callbacks ──────────────────────────────────────────── */

static int eth_t_open(void* ctx, const char* host, uint16_t port)
{
    ethernet_state* st = as_state(ctx);
    IPAddress ip;
    if (!parse_ipv4(host, ip)) {
        Serial.printf("http-eth: malformed host '%s'\n", host);
        return -1;
    }
    if (!st->client.connect(ip, port)) {
        Serial.printf("http-eth: connect to %s:%u failed\n", host, port);
        return -1;
    }
    st->opened = true;
    return 0;
}

static int eth_t_send(void* ctx, const uint8_t* buf, size_t len)
{
    ethernet_state* st = as_state(ctx);
    if (!st->opened) return -1;
    size_t n = st->client.write(buf, len);
    if (n == 0) return -1;
    return (int)n;
}

static int eth_t_recv(void* ctx, uint8_t* buf, size_t len)
{
    ethernet_state* st = as_state(ctx);
    if (!st->opened) return -1;

    uint32_t start = millis();
    while (true) {
        int avail = st->client.available();
        if (avail > 0) {
            size_t take = (size_t)avail < len ? (size_t)avail : len;
            int n = st->client.read(buf, take);
            if (n < 0) return -1;
            return n;
        }
        if (!st->client.connected() && avail == 0) {
            return 0;  /* peer closed and buffer drained */
        }
        if (millis() - start >= RECV_TIMEOUT_MS) {
            Serial.println("http-eth: recv timeout");
            return -1;
        }
        delay(2);
    }
}

static void eth_t_close(void* ctx)
{
    ethernet_state* st = as_state(ctx);
    if (st->opened) {
        st->client.stop();
        st->opened = false;
    }
}

/* ── public init ──────────────────────────────────────────────────── */

extern "C" void http_transport_ethernet_init(http_transport_t* out,
                                              http_ethernet_state_t* state)
{
    memset(state, 0, sizeof(*state));
    ethernet_state* st = as_state(state);
    new (st) ethernet_state();   /* placement-new the EthernetClient */
    st->opened = false;

    out->open  = eth_t_open;
    out->send  = eth_t_send;
    out->recv  = eth_t_recv;
    out->close = eth_t_close;
    out->ctx   = state;
}
