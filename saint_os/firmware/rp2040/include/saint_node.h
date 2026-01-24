/**
 * SAINT.OS Node Firmware - Main Header
 *
 * Core definitions and structures for SAINT.OS nodes running on RP2040.
 */

#ifndef SAINT_NODE_H
#define SAINT_NODE_H

#include <stdint.h>
#include <stdbool.h>

// =============================================================================
// Node States
// =============================================================================

typedef enum {
    NODE_STATE_BOOT,        // Initial boot
    NODE_STATE_CONNECTING,  // Connecting to network
    NODE_STATE_UNADOPTED,   // Connected but not adopted
    NODE_STATE_ADOPTING,    // Being adopted (downloading config)
    NODE_STATE_ACTIVE,      // Adopted and running
    NODE_STATE_ERROR        // Error state
} node_state_t;

// =============================================================================
// Node Roles
// =============================================================================

typedef enum {
    NODE_ROLE_NONE = 0,
    NODE_ROLE_HEAD,
    NODE_ROLE_ARMS_LEFT,
    NODE_ROLE_ARMS_RIGHT,
    NODE_ROLE_TRACKS,
    NODE_ROLE_CONSOLE
} node_role_t;

// =============================================================================
// Node Configuration
// =============================================================================

typedef struct {
    // Identity
    char node_id[32];           // Unique node ID (from flash or MAC)
    uint8_t mac_address[6];     // Ethernet MAC address

    // Network
    bool use_dhcp;
    uint8_t static_ip[4];
    uint8_t subnet_mask[4];
    uint8_t gateway[4];
    uint8_t server_ip[4];
    uint16_t server_port;       // micro-ROS agent UDP port

    // Role
    node_role_t role;
    char display_name[64];

    // State
    node_state_t state;
    uint32_t uptime_ms;
    uint32_t last_announce_ms;

} saint_node_config_t;

// =============================================================================
// Hardware Pin Definitions (Feather RP2040 + Ethernet FeatherWing)
// =============================================================================

// SPI pins for W5500 Ethernet
#define ETH_SPI_PORT    spi0
#define ETH_PIN_SCK     18      // SPI Clock
#define ETH_PIN_MOSI    19      // SPI TX
#define ETH_PIN_MISO    20      // SPI RX
#define ETH_PIN_CS      10      // Chip Select
#define ETH_PIN_RST     11      // Reset

// Onboard NeoPixel (WS2812)
#define NEOPIXEL_PIN    16
#define NEOPIXEL_COUNT  1

// Available GPIO for peripherals (after ethernet)
#define GPIO_A0         26      // ADC0
#define GPIO_A1         27      // ADC1
#define GPIO_A2         28      // ADC2
#define GPIO_A3         29      // ADC3
#define GPIO_D5         5
#define GPIO_D6         6
#define GPIO_D9         9
#define GPIO_D12        12
#define GPIO_D13        13      // Also onboard LED
#define GPIO_D24        24
#define GPIO_D25        25

// I2C pins
#define I2C_SDA         2
#define I2C_SCL         3

// UART pins
#define UART_TX         0
#define UART_RX         1

// =============================================================================
// Timing Constants
// =============================================================================

#define ANNOUNCE_INTERVAL_MS    1000    // Announce every 1 second when unadopted
#define HEARTBEAT_INTERVAL_MS   5000    // Heartbeat every 5 seconds
#define STATE_UPDATE_INTERVAL_MS 100    // State updates at 10Hz when active
#define RECONNECT_DELAY_MS      5000    // Wait 5 seconds between reconnect attempts

// =============================================================================
// Global Node Instance
// =============================================================================

extern saint_node_config_t g_node;

// =============================================================================
// Function Declarations
// =============================================================================

// node_state.c
void node_state_init(void);
void node_state_update(void);
void node_set_state(node_state_t new_state);
const char* node_state_to_string(node_state_t state);
const char* node_role_to_string(node_role_t role);
bool node_adopt(node_role_t role, const char* display_name);
bool node_reset(bool factory_reset);

// hardware.c
void hardware_init(void);
void hardware_update(void);
uint32_t hardware_get_unique_id(char* buffer, size_t len);
float hardware_get_cpu_temp(void);

// led_status.c
void led_init(void);
void led_set_state(node_state_t state);
void led_update(void);

#endif // SAINT_NODE_H
