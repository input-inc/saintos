/**
 * SAINT.OS Node Firmware - NativeEthernet Transport
 *
 * micro-ROS custom transport using Teensy 4.1 built-in Ethernet.
 */

#include <Arduino.h>

#ifndef SIMULATION

#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

extern "C" {
#include "transport_native_eth.h"
}

// UDP socket for micro-ROS
static EthernetUDP udp;

// Network configuration
static uint8_t agent_ip[4] = {192, 168, 1, 10};
static uint16_t agent_port = 8888;
static uint16_t local_port = 9999;

static uint8_t mac_addr[6];
static uint8_t local_ip[4];
static bool connected = false;

// DHCP retry pacing — same shape as firmware/rp2040/transport/transport_w5500.c.
// Server (Pi) and node power up together; dnsmasq isn't bound on the gateway
// until NetworkManager finishes activating eth0, so the first ~30s of
// DISCOVERs land in dead air. Short attempts + short backoff burn through
// that window quickly; after ~20 tries we stretch the backoff so log volume
// stays sane if the network is actually broken.
#define DHCP_ATTEMPT_TIMEOUT_MS  3000u
#define DHCP_BACKOFF_FAST_MS     500u
#define DHCP_BACKOFF_SLOW_MS     5000u
#define DHCP_FAST_ATTEMPTS       20u

extern void led_update(void);  // led_status.cpp

extern "C" {

bool transport_native_eth_init(void)
{
    Serial.printf("Transport: NativeEthernet init\n");

    // Generate MAC from Teensy unique ID
    // Use locally administered bit
    //extern uint32_t HW_OCOTP_MAC0;
    //extern uint32_t HW_OCOTP_MAC1;

    mac_addr[0] = 0x02;  // Locally administered
    mac_addr[1] = (HW_OCOTP_MAC0 >> 24) & 0xFF;
    mac_addr[2] = (HW_OCOTP_MAC0 >> 16) & 0xFF;
    mac_addr[3] = (HW_OCOTP_MAC0 >> 8) & 0xFF;
    mac_addr[4] = HW_OCOTP_MAC0 & 0xFF;
    mac_addr[5] = HW_OCOTP_MAC1 & 0xFF;

    return true;
}

/* Pump LED animation while we sleep in 100ms slices. */
static void wait_with_led(uint32_t total_ms)
{
    uint32_t slept = 0;
    while (slept < total_ms) {
        delay(100);
        led_update();
        slept += 100;
    }
}

bool transport_native_eth_connect(void)
{
    // Wait indefinitely for an Ethernet link. The server (Pi) and the node
    // power up together; giving up after a few seconds would strand the
    // node on a useless fallback. Matches RP2040 (transport_w5500.c).
    Serial.printf("Transport: waiting for Ethernet link...\n");
    uint32_t link_wait_ms = 0;
    while (Ethernet.linkStatus() != LinkON) {
        wait_with_led(100);
        link_wait_ms += 100;
        if (link_wait_ms % 5000 == 0) {
            Serial.printf("Still waiting for ethernet link (%lus)...\n",
                          (unsigned long)(link_wait_ms / 1000));
        }
    }
    Serial.printf("Ethernet link detected after %lums\n",
                  (unsigned long)link_wait_ms);

    // Retry DHCP forever. We never fall back to a static IP because the
    // compile-time fallback is on a different subnet than the server: a
    // "successful" fallback would IP-conflict with every other node doing
    // the same thing. The RP2040 OTA path makes the same call for the
    // same reason — see firmware/rp2040/transport/transport_w5500.c:286.
    uint32_t attempt = 0;
    while (true) {
        attempt++;
        Serial.printf("Transport: DHCP attempt %lu...\n",
                      (unsigned long)attempt);
        if (Ethernet.begin(mac_addr, DHCP_ATTEMPT_TIMEOUT_MS) != 0) {
            break;
        }
        uint32_t backoff_ms = (attempt < DHCP_FAST_ATTEMPTS)
                              ? DHCP_BACKOFF_FAST_MS
                              : DHCP_BACKOFF_SLOW_MS;
        Serial.printf("Transport: DHCP attempt %lu failed; retrying in %lums\n",
                      (unsigned long)attempt, (unsigned long)backoff_ms);
        wait_with_led(backoff_ms);
    }

    // Store IP
    IPAddress ip = Ethernet.localIP();
    local_ip[0] = ip[0];
    local_ip[1] = ip[1];
    local_ip[2] = ip[2];
    local_ip[3] = ip[3];

    IPAddress gw = Ethernet.gatewayIP();
    Serial.printf("Transport: DHCP bound IP=%d.%d.%d.%d gw=%d.%d.%d.%d\n",
                   local_ip[0], local_ip[1], local_ip[2], local_ip[3],
                   gw[0], gw[1], gw[2], gw[3]);

    // Start UDP
    udp.begin(local_port);
    connected = true;

    return true;
}

void transport_native_eth_get_mac(uint8_t* mac)
{
    memcpy(mac, mac_addr, 6);
}

void transport_native_eth_get_ip(uint8_t* ip)
{
    memcpy(ip, local_ip, 4);
}

bool transport_native_eth_is_connected(void)
{
    return connected && (Ethernet.linkStatus() == LinkON);
}

void transport_native_eth_set_agent(const uint8_t* ip, uint16_t port)
{
    memcpy(agent_ip, ip, 4);
    agent_port = port;
    Serial.printf("Transport: agent set to %d.%d.%d.%d:%d\n",
                   agent_ip[0], agent_ip[1], agent_ip[2], agent_ip[3], agent_port);
}

bool transport_native_eth_open(struct uxrCustomTransport* transport)
{
    (void)transport;
    return connected;
}

bool transport_native_eth_close(struct uxrCustomTransport* transport)
{
    (void)transport;
    udp.stop();
    connected = false;
    return true;
}

size_t transport_native_eth_write(
    struct uxrCustomTransport* transport,
    const uint8_t* buf,
    size_t len,
    uint8_t* err)
{
    (void)transport;

    IPAddress dest(agent_ip[0], agent_ip[1], agent_ip[2], agent_ip[3]);

    udp.beginPacket(dest, agent_port);
    size_t written = udp.write(buf, len);
    udp.endPacket();

    if (written != len) {
        *err = 1;
        return 0;
    }

    *err = 0;
    return written;
}

size_t transport_native_eth_read(
    struct uxrCustomTransport* transport,
    uint8_t* buf,
    size_t len,
    int timeout_ms,
    uint8_t* err)
{
    (void)transport;

    uint32_t start = millis();

    while ((millis() - start) < (uint32_t)timeout_ms) {
        int packet_size = udp.parsePacket();
        if (packet_size > 0) {
            size_t to_read = (size_t)packet_size < len ? (size_t)packet_size : len;
            size_t read_bytes = udp.read(buf, to_read);
            *err = 0;
            return read_bytes;
        }
        delay(1);
    }

    *err = 0;
    return 0;
}

} // extern "C"

#endif // !SIMULATION
