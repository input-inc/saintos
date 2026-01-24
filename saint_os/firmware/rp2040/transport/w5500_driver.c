/**
 * SAINT.OS Node Firmware - W5500 Driver Implementation
 *
 * Low-level driver for Wiznet W5500 Ethernet chip over SPI.
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

#include "saint_node.h"
#include "w5500_driver.h"

// =============================================================================
// W5500 Register Addresses (Common Registers)
// =============================================================================

#define W5500_MR        0x0000  // Mode Register
#define W5500_GAR       0x0001  // Gateway Address (4 bytes)
#define W5500_SUBR      0x0005  // Subnet Mask (4 bytes)
#define W5500_SHAR      0x0009  // Source Hardware Address (6 bytes)
#define W5500_SIPR      0x000F  // Source IP Address (4 bytes)
#define W5500_PHYCFGR   0x002E  // PHY Configuration

// =============================================================================
// W5500 Socket Register Offsets
// =============================================================================

#define W5500_Sn_MR     0x0000  // Socket n Mode
#define W5500_Sn_CR     0x0001  // Socket n Command
#define W5500_Sn_IR     0x0002  // Socket n Interrupt
#define W5500_Sn_SR     0x0003  // Socket n Status
#define W5500_Sn_PORT   0x0004  // Socket n Source Port (2 bytes)
#define W5500_Sn_DIPR   0x000C  // Socket n Dest IP Address (4 bytes)
#define W5500_Sn_DPORT  0x0010  // Socket n Dest Port (2 bytes)
#define W5500_Sn_RXBUF_SIZE 0x001E  // Socket n RX Buffer Size
#define W5500_Sn_TXBUF_SIZE 0x001F  // Socket n TX Buffer Size
#define W5500_Sn_TX_FSR 0x0020  // Socket n TX Free Size (2 bytes)
#define W5500_Sn_TX_WR  0x0024  // Socket n TX Write Pointer (2 bytes)
#define W5500_Sn_RX_RSR 0x0026  // Socket n RX Received Size (2 bytes)
#define W5500_Sn_RX_RD  0x0028  // Socket n RX Read Pointer (2 bytes)

// =============================================================================
// W5500 Socket Commands
// =============================================================================

#define W5500_CMD_OPEN      0x01
#define W5500_CMD_CLOSE     0x10
#define W5500_CMD_SEND      0x20
#define W5500_CMD_RECV      0x40

// =============================================================================
// W5500 Block Select Bits (for SPI frame)
// =============================================================================

#define W5500_BSB_COMMON    0x00
#define W5500_BSB_SOCKET(n) (((n) << 2) + 0x01)  // Socket n registers
#define W5500_BSB_TX(n)     (((n) << 2) + 0x02)  // Socket n TX buffer
#define W5500_BSB_RX(n)     (((n) << 2) + 0x03)  // Socket n RX buffer

// =============================================================================
// SPI Interface
// =============================================================================

static inline void spi_cs_select(void)
{
    gpio_put(ETH_PIN_CS, 0);
}

static inline void spi_cs_deselect(void)
{
    gpio_put(ETH_PIN_CS, 1);
}

void w5500_write_byte(uint16_t addr, uint8_t block, uint8_t data)
{
    uint8_t frame[4];
    frame[0] = (addr >> 8) & 0xFF;
    frame[1] = addr & 0xFF;
    frame[2] = (block << 3) | 0x04;  // Write mode
    frame[3] = data;

    spi_cs_select();
    spi_write_blocking(ETH_SPI_PORT, frame, 4);
    spi_cs_deselect();
}

uint8_t w5500_read_byte(uint16_t addr, uint8_t block)
{
    uint8_t frame[3];
    frame[0] = (addr >> 8) & 0xFF;
    frame[1] = addr & 0xFF;
    frame[2] = (block << 3) | 0x00;  // Read mode

    uint8_t data;

    spi_cs_select();
    spi_write_blocking(ETH_SPI_PORT, frame, 3);
    spi_read_blocking(ETH_SPI_PORT, 0x00, &data, 1);
    spi_cs_deselect();

    return data;
}

void w5500_write_buf(uint16_t addr, uint8_t block, const uint8_t* data, size_t len)
{
    uint8_t frame[3];
    frame[0] = (addr >> 8) & 0xFF;
    frame[1] = addr & 0xFF;
    frame[2] = (block << 3) | 0x04;  // Write mode

    spi_cs_select();
    spi_write_blocking(ETH_SPI_PORT, frame, 3);
    spi_write_blocking(ETH_SPI_PORT, data, len);
    spi_cs_deselect();
}

void w5500_read_buf(uint16_t addr, uint8_t block, uint8_t* data, size_t len)
{
    uint8_t frame[3];
    frame[0] = (addr >> 8) & 0xFF;
    frame[1] = addr & 0xFF;
    frame[2] = (block << 3) | 0x00;  // Read mode

    spi_cs_select();
    spi_write_blocking(ETH_SPI_PORT, frame, 3);
    spi_read_blocking(ETH_SPI_PORT, 0x00, data, len);
    spi_cs_deselect();
}

// =============================================================================
// Initialization
// =============================================================================

bool w5500_init(void)
{
    // Soft reset
    w5500_write_byte(W5500_MR, W5500_BSB_COMMON, 0x80);
    sleep_ms(10);

    // Wait for reset to complete
    uint32_t timeout = 100;
    while (w5500_read_byte(W5500_MR, W5500_BSB_COMMON) & 0x80) {
        if (--timeout == 0) {
            printf("W5500 reset timeout\n");
            return false;
        }
        sleep_ms(1);
    }

    // Verify chip by reading PHY config
    uint8_t phycfg = w5500_read_byte(W5500_PHYCFGR, W5500_BSB_COMMON);
    if (phycfg == 0x00 || phycfg == 0xFF) {
        printf("W5500 not detected (PHYCFGR=0x%02X)\n", phycfg);
        return false;
    }

    printf("W5500 detected (PHYCFGR=0x%02X)\n", phycfg);

    // Configure socket buffer sizes (2KB each by default)
    for (int i = 0; i < 8; i++) {
        w5500_write_byte(W5500_Sn_RXBUF_SIZE, W5500_BSB_SOCKET(i), 2);
        w5500_write_byte(W5500_Sn_TXBUF_SIZE, W5500_BSB_SOCKET(i), 2);
    }

    return true;
}

void w5500_reset(void)
{
    w5500_write_byte(W5500_MR, W5500_BSB_COMMON, 0x80);
    sleep_ms(10);
}

// =============================================================================
// Configuration
// =============================================================================

void w5500_set_mac(const uint8_t* mac)
{
    w5500_write_buf(W5500_SHAR, W5500_BSB_COMMON, mac, 6);
}

void w5500_get_mac(uint8_t* mac)
{
    w5500_read_buf(W5500_SHAR, W5500_BSB_COMMON, mac, 6);
}

void w5500_set_ip(const uint8_t* ip)
{
    w5500_write_buf(W5500_SIPR, W5500_BSB_COMMON, ip, 4);
}

void w5500_get_ip(uint8_t* ip)
{
    w5500_read_buf(W5500_SIPR, W5500_BSB_COMMON, ip, 4);
}

void w5500_set_subnet(const uint8_t* subnet)
{
    w5500_write_buf(W5500_SUBR, W5500_BSB_COMMON, subnet, 4);
}

void w5500_set_gateway(const uint8_t* gateway)
{
    w5500_write_buf(W5500_GAR, W5500_BSB_COMMON, gateway, 4);
}

// =============================================================================
// Network Status
// =============================================================================

bool w5500_get_link_status(void)
{
    uint8_t phycfg = w5500_read_byte(W5500_PHYCFGR, W5500_BSB_COMMON);
    return (phycfg & 0x01) != 0;  // Link up bit
}

uint8_t w5500_get_link_speed(void)
{
    uint8_t phycfg = w5500_read_byte(W5500_PHYCFGR, W5500_BSB_COMMON);
    return (phycfg & 0x02) ? 100 : 10;  // Speed bit
}

// =============================================================================
// DHCP (Simplified implementation)
// =============================================================================

bool w5500_dhcp_request(uint8_t* ip_out, uint32_t timeout_ms)
{
    // This is a simplified DHCP implementation
    // For production, use a full DHCP library

    // For now, just wait and check if IP was assigned
    // (assumes external DHCP or static config)

    printf("DHCP not fully implemented - using static IP fallback\n");

    // Fallback: use a default IP
    ip_out[0] = 192;
    ip_out[1] = 168;
    ip_out[2] = 1;
    ip_out[3] = 100;

    w5500_set_ip(ip_out);
    w5500_set_subnet((uint8_t[]){255, 255, 255, 0});
    w5500_set_gateway((uint8_t[]){192, 168, 1, 1});

    return true;
}

// =============================================================================
// Socket Operations
// =============================================================================

bool w5500_socket_open(uint8_t socket, uint8_t type, uint16_t port)
{
    if (socket >= 8) return false;

    // Close socket first if open
    w5500_socket_close(socket);

    // Set socket mode
    w5500_write_byte(W5500_Sn_MR, W5500_BSB_SOCKET(socket), type);

    // Set source port
    w5500_write_byte(W5500_Sn_PORT, W5500_BSB_SOCKET(socket), (port >> 8) & 0xFF);
    w5500_write_byte(W5500_Sn_PORT + 1, W5500_BSB_SOCKET(socket), port & 0xFF);

    // Open socket
    w5500_write_byte(W5500_Sn_CR, W5500_BSB_SOCKET(socket), W5500_CMD_OPEN);

    // Wait for command to complete
    while (w5500_read_byte(W5500_Sn_CR, W5500_BSB_SOCKET(socket)) != 0) {
        sleep_ms(1);
    }

    // Verify socket opened
    uint8_t status = w5500_read_byte(W5500_Sn_SR, W5500_BSB_SOCKET(socket));

    if (type == W5500_SOCK_UDP && status == W5500_SOCK_UDP_STATE) {
        return true;
    }

    printf("Socket open failed (status=0x%02X)\n", status);
    return false;
}

void w5500_socket_close(uint8_t socket)
{
    if (socket >= 8) return;

    w5500_write_byte(W5500_Sn_CR, W5500_BSB_SOCKET(socket), W5500_CMD_CLOSE);

    while (w5500_read_byte(W5500_Sn_CR, W5500_BSB_SOCKET(socket)) != 0) {
        sleep_ms(1);
    }
}

uint8_t w5500_socket_status(uint8_t socket)
{
    if (socket >= 8) return 0;
    return w5500_read_byte(W5500_Sn_SR, W5500_BSB_SOCKET(socket));
}

int w5500_socket_available(uint8_t socket)
{
    if (socket >= 8) return -1;

    uint8_t high = w5500_read_byte(W5500_Sn_RX_RSR, W5500_BSB_SOCKET(socket));
    uint8_t low = w5500_read_byte(W5500_Sn_RX_RSR + 1, W5500_BSB_SOCKET(socket));

    return (high << 8) | low;
}

int w5500_socket_sendto(
    uint8_t socket,
    const uint8_t* data,
    size_t len,
    const uint8_t* dest_ip,
    uint16_t dest_port)
{
    if (socket >= 8 || len == 0) return -1;

    // Set destination IP
    w5500_write_buf(W5500_Sn_DIPR, W5500_BSB_SOCKET(socket), dest_ip, 4);

    // Set destination port
    w5500_write_byte(W5500_Sn_DPORT, W5500_BSB_SOCKET(socket), (dest_port >> 8) & 0xFF);
    w5500_write_byte(W5500_Sn_DPORT + 1, W5500_BSB_SOCKET(socket), dest_port & 0xFF);

    // Get TX write pointer
    uint8_t ptr_high = w5500_read_byte(W5500_Sn_TX_WR, W5500_BSB_SOCKET(socket));
    uint8_t ptr_low = w5500_read_byte(W5500_Sn_TX_WR + 1, W5500_BSB_SOCKET(socket));
    uint16_t ptr = (ptr_high << 8) | ptr_low;

    // Write data to TX buffer
    w5500_write_buf(ptr, W5500_BSB_TX(socket), data, len);

    // Update TX write pointer
    ptr += len;
    w5500_write_byte(W5500_Sn_TX_WR, W5500_BSB_SOCKET(socket), (ptr >> 8) & 0xFF);
    w5500_write_byte(W5500_Sn_TX_WR + 1, W5500_BSB_SOCKET(socket), ptr & 0xFF);

    // Send
    w5500_write_byte(W5500_Sn_CR, W5500_BSB_SOCKET(socket), W5500_CMD_SEND);

    // Wait for send to complete
    while (w5500_read_byte(W5500_Sn_CR, W5500_BSB_SOCKET(socket)) != 0) {
        sleep_ms(1);
    }

    return len;
}

int w5500_socket_recvfrom(
    uint8_t socket,
    uint8_t* buf,
    size_t len,
    uint8_t* src_ip,
    uint16_t* src_port)
{
    if (socket >= 8) return -1;

    int available = w5500_socket_available(socket);
    if (available <= 0) return 0;

    // Get RX read pointer
    uint8_t ptr_high = w5500_read_byte(W5500_Sn_RX_RD, W5500_BSB_SOCKET(socket));
    uint8_t ptr_low = w5500_read_byte(W5500_Sn_RX_RD + 1, W5500_BSB_SOCKET(socket));
    uint16_t ptr = (ptr_high << 8) | ptr_low;

    // Read UDP header (8 bytes: 4 IP + 2 port + 2 length)
    uint8_t header[8];
    w5500_read_buf(ptr, W5500_BSB_RX(socket), header, 8);
    ptr += 8;

    // Extract source IP and port
    if (src_ip) {
        memcpy(src_ip, header, 4);
    }
    if (src_port) {
        *src_port = (header[4] << 8) | header[5];
    }

    // Get data length
    uint16_t data_len = (header[6] << 8) | header[7];
    if (data_len > len) {
        data_len = len;
    }

    // Read data
    w5500_read_buf(ptr, W5500_BSB_RX(socket), buf, data_len);
    ptr += data_len;

    // Update RX read pointer
    w5500_write_byte(W5500_Sn_RX_RD, W5500_BSB_SOCKET(socket), (ptr >> 8) & 0xFF);
    w5500_write_byte(W5500_Sn_RX_RD + 1, W5500_BSB_SOCKET(socket), ptr & 0xFF);

    // Complete receive
    w5500_write_byte(W5500_Sn_CR, W5500_BSB_SOCKET(socket), W5500_CMD_RECV);

    while (w5500_read_byte(W5500_Sn_CR, W5500_BSB_SOCKET(socket)) != 0) {
        sleep_ms(1);
    }

    return data_len;
}
