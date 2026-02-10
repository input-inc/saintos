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

// DHCP or static IP
static uint8_t default_ip[4] = {192, 168, 1, 200};
static uint8_t default_gw[4] = {192, 168, 1, 1};
static uint8_t default_sn[4] = {255, 255, 255, 0};

extern "C" {

bool transport_native_eth_init(void)
{
    Serial.printf("Transport: NativeEthernet init\n");

    // Generate MAC from Teensy unique ID
    // Use locally administered bit
    extern uint32_t HW_OCOTP_MAC0;
    extern uint32_t HW_OCOTP_MAC1;

    mac_addr[0] = 0x02;  // Locally administered
    mac_addr[1] = (HW_OCOTP_MAC0 >> 24) & 0xFF;
    mac_addr[2] = (HW_OCOTP_MAC0 >> 16) & 0xFF;
    mac_addr[3] = (HW_OCOTP_MAC0 >> 8) & 0xFF;
    mac_addr[4] = HW_OCOTP_MAC0 & 0xFF;
    mac_addr[5] = HW_OCOTP_MAC1 & 0xFF;

    return true;
}

bool transport_native_eth_connect(void)
{
    Serial.printf("Transport: connecting via DHCP...\n");

    // Try DHCP first
    if (Ethernet.begin(mac_addr) == 0) {
        Serial.printf("Transport: DHCP failed, using static IP\n");
        IPAddress ip(default_ip[0], default_ip[1], default_ip[2], default_ip[3]);
        IPAddress gw(default_gw[0], default_gw[1], default_gw[2], default_gw[3]);
        IPAddress sn(default_sn[0], default_sn[1], default_sn[2], default_sn[3]);
        Ethernet.begin(mac_addr, ip, IPAddress(0,0,0,0), gw, sn);
    }

    // Store IP
    IPAddress ip = Ethernet.localIP();
    local_ip[0] = ip[0];
    local_ip[1] = ip[1];
    local_ip[2] = ip[2];
    local_ip[3] = ip[3];

    Serial.printf("Transport: connected at %d.%d.%d.%d\n",
                   local_ip[0], local_ip[1], local_ip[2], local_ip[3]);

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
