/**
 * SAINT.OS Node Firmware - Main Entry Point
 *
 * Runs on Adafruit Feather RP2040 with Ethernet FeatherWing.
 * Implements a micro-ROS node that communicates with the SAINT.OS server.
 *
 * NOTE: This version uses standard ROS2 messages (std_msgs/String) for
 * initial testing. Custom SAINT.OS messages require rebuilding libmicroros.
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/unique_id.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microros/rmw_microros.h>

// Standard message types (available in prebuilt libmicroros)
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>

#include "saint_node.h"
#include "version.h"

// Transport selection based on build mode
#ifdef SIMULATION
#include "transport_udp_bridge.h"
#define TRANSPORT_INIT()       transport_udp_bridge_init()
#define TRANSPORT_CONNECT()    transport_udp_bridge_connect()
#define TRANSPORT_CONNECTED()  transport_udp_bridge_is_connected()
#define TRANSPORT_OPEN         transport_udp_bridge_open
#define TRANSPORT_CLOSE        transport_udp_bridge_close
#define TRANSPORT_WRITE        transport_udp_bridge_write
#define TRANSPORT_READ         transport_udp_bridge_read
#define TRANSPORT_FRAMED       false  // UDP doesn't need framing
#define TRANSPORT_NAME         "UDP Bridge (Simulation)"
#define TRANSPORT_SET_AGENT(ip, port) transport_udp_bridge_set_agent(ip, port)
#define TRANSPORT_GET_IP(ip)   transport_udp_bridge_get_ip(ip)
#define TRANSPORT_SET_IP(ip)   transport_udp_bridge_set_ip(ip)
#else
#include "transport_w5500.h"
#define TRANSPORT_INIT()       transport_w5500_init()
#define TRANSPORT_CONNECT()    transport_w5500_connect()
#define TRANSPORT_CONNECTED()  transport_w5500_is_connected()
#define TRANSPORT_OPEN         transport_w5500_open
#define TRANSPORT_CLOSE        transport_w5500_close
#define TRANSPORT_WRITE        transport_w5500_write
#define TRANSPORT_READ         transport_w5500_read
#define TRANSPORT_FRAMED       false  // UDP doesn't need framing
#define TRANSPORT_NAME         "W5500 Ethernet (UDP)"
#define TRANSPORT_SET_AGENT(ip, port) transport_w5500_set_agent(ip, port)
#define TRANSPORT_GET_IP(ip)   transport_w5500_get_ip(ip)
#define TRANSPORT_GET_MAC(mac) transport_w5500_get_mac(mac)
#endif

// =============================================================================
// Global Variables
// =============================================================================

// Node configuration
saint_node_config_t g_node;

// micro-ROS handles
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t ros_node;
static rclc_executor_t executor;

// Publishers
static rcl_publisher_t announcement_pub;

// Timers
static rcl_timer_t announce_timer;

// Message buffers
static std_msgs__msg__String announcement_msg;
static char announcement_buffer[256];

// =============================================================================
// Timer Callbacks
// =============================================================================

/**
 * Timer callback for node announcements.
 * Publishes node info when in unadopted state.
 */
static void announce_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
    (void)last_call_time;

    if (timer == NULL || g_node.state != NODE_STATE_UNADOPTED) {
        return;
    }

    // Build announcement JSON string
    snprintf(announcement_buffer, sizeof(announcement_buffer),
        "{"
        "\"node_id\":\"%s\","
        "\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
        "\"ip\":\"%d.%d.%d.%d\","
        "\"hw\":\"%s\","
        "\"fw\":\"%s\","
        "\"state\":\"%s\","
        "\"uptime\":%lu"
        "}",
        g_node.node_id,
        g_node.mac_address[0], g_node.mac_address[1],
        g_node.mac_address[2], g_node.mac_address[3],
        g_node.mac_address[4], g_node.mac_address[5],
        g_node.static_ip[0], g_node.static_ip[1],
        g_node.static_ip[2], g_node.static_ip[3],
        HARDWARE_MODEL,
        FIRMWARE_VERSION_STRING,
        node_state_to_string(g_node.state),
        g_node.uptime_ms / 1000
    );

    announcement_msg.data.data = announcement_buffer;
    announcement_msg.data.size = strlen(announcement_buffer);
    announcement_msg.data.capacity = sizeof(announcement_buffer);

    rcl_ret_t ret = rcl_publish(&announcement_pub, &announcement_msg, NULL);
    if (ret != RCL_RET_OK) {
        printf("Announce publish failed: %d\n", ret);
    }
}

// =============================================================================
// micro-ROS Initialization
// =============================================================================

/**
 * Initialize micro-ROS node and entities.
 */
static bool init_micro_ros(void)
{
    printf("Initializing micro-ROS...\n");

    allocator = rcl_get_default_allocator();

    // Initialize support with custom transport
    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
        printf("Failed to initialize support: %d\n", ret);
        return false;
    }

    // Create node with name based on node_id
    char node_name[64];
    snprintf(node_name, sizeof(node_name), "saint_node_%s", g_node.node_id);

    // Replace any invalid characters in node name
    for (char* p = node_name; *p; p++) {
        if (*p == '-' || *p == ':') *p = '_';
    }

    ret = rclc_node_init_default(&ros_node, node_name, "saint", &support);
    if (ret != RCL_RET_OK) {
        printf("Failed to create node: %d\n", ret);
        return false;
    }

    printf("Created ROS2 node: %s\n", node_name);

    // Create announcement publisher (using String for initial testing)
    ret = rclc_publisher_init_default(
        &announcement_pub,
        &ros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/saint/nodes/announce"
    );
    if (ret != RCL_RET_OK) {
        printf("Failed to create publisher: %d\n", ret);
        return false;
    }

    // Create announcement timer (1 second interval)
    ret = rclc_timer_init_default(
        &announce_timer,
        &support,
        RCL_MS_TO_NS(ANNOUNCE_INTERVAL_MS),
        announce_timer_callback
    );
    if (ret != RCL_RET_OK) {
        printf("Failed to create timer: %d\n", ret);
        return false;
    }

    // Initialize executor with 1 timer
    ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
    if (ret != RCL_RET_OK) {
        printf("Failed to create executor: %d\n", ret);
        return false;
    }

    // Add timer to executor
    ret = rclc_executor_add_timer(&executor, &announce_timer);
    if (ret != RCL_RET_OK) {
        printf("Failed to add timer: %d\n", ret);
        return false;
    }

    printf("micro-ROS initialized successfully\n");
    return true;
}

/**
 * Clean up micro-ROS resources.
 */
static void cleanup_micro_ros(void)
{
    rcl_publisher_fini(&announcement_pub, &ros_node);
    rcl_timer_fini(&announce_timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&ros_node);
    rclc_support_fini(&support);
}

// =============================================================================
// Main Function
// =============================================================================

int main(void)
{
    // Initialize stdio (USB serial)
    stdio_init_all();

    // Wait for USB serial connection (optional, for debugging)
    sleep_ms(2000);

    printf("\n");
    printf("========================================\n");
    printf("SAINT.OS Node Firmware v%s\n", FIRMWARE_VERSION_STRING);
    printf("Hardware: %s\n", HARDWARE_MODEL);
    printf("========================================\n");

    // Initialize node state
    node_state_init();

    // Initialize hardware
    hardware_init();

    // Initialize status LED
    led_init();
    led_set_state(NODE_STATE_BOOT);

    // Get unique ID for node identification
    char unique_id[32];
    hardware_get_unique_id(unique_id, sizeof(unique_id));
    snprintf(g_node.node_id, sizeof(g_node.node_id), "rp2040_%s", unique_id);
    printf("Node ID: %s\n", g_node.node_id);

    // Initialize transport
    printf("Initializing transport: %s\n", TRANSPORT_NAME);
    led_set_state(NODE_STATE_CONNECTING);

    if (!TRANSPORT_INIT()) {
        printf("ERROR: Transport initialization failed!\n");
        node_set_state(NODE_STATE_ERROR);
        led_set_state(NODE_STATE_ERROR);
        while (1) {
            led_update();
            sleep_ms(100);
        }
    }

#ifdef SIMULATION
    // In simulation mode, generate fake MAC address from unique ID
    char uid[16];
    hardware_get_unique_id(uid, sizeof(uid));
    g_node.mac_address[0] = 0x02;  // Locally administered
    for (int i = 0; i < 5 && uid[i*2]; i++) {
        char hex[3] = {uid[i*2], uid[i*2+1], 0};
        g_node.mac_address[i+1] = (uint8_t)strtol(hex, NULL, 16);
    }
    // Set simulation IP (can be configured via environment or defaults)
    // Each node should have a unique IP in the simulation
    g_node.static_ip[0] = 192;
    g_node.static_ip[1] = 168;
    g_node.static_ip[2] = 1;
    g_node.static_ip[3] = 100;  // Will be unique per node instance
    TRANSPORT_SET_IP(g_node.static_ip);
#else
    // Get MAC address from W5500
    TRANSPORT_GET_MAC(g_node.mac_address);
#endif

    printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
           g_node.mac_address[0], g_node.mac_address[1],
           g_node.mac_address[2], g_node.mac_address[3],
           g_node.mac_address[4], g_node.mac_address[5]);

    // Connect transport
    if (!TRANSPORT_CONNECT()) {
        printf("ERROR: Transport connection failed!\n");
        node_set_state(NODE_STATE_ERROR);
        led_set_state(NODE_STATE_ERROR);
        while (1) {
            led_update();
            sleep_ms(100);
        }
    }

#ifndef SIMULATION
    // Get IP address from W5500 (DHCP or static)
    TRANSPORT_GET_IP(g_node.static_ip);
#endif

    printf("IP: %d.%d.%d.%d\n",
           g_node.static_ip[0], g_node.static_ip[1],
           g_node.static_ip[2], g_node.static_ip[3]);

    // Set agent address from config
    TRANSPORT_SET_AGENT(g_node.server_ip, g_node.server_port);
    printf("Agent: %d.%d.%d.%d:%d\n",
           g_node.server_ip[0], g_node.server_ip[1],
           g_node.server_ip[2], g_node.server_ip[3],
           g_node.server_port);

    // Set micro-ROS custom transport
    rmw_uros_set_custom_transport(
        TRANSPORT_FRAMED,  // Framing depends on transport type
        NULL,              // No transport args
        TRANSPORT_OPEN,
        TRANSPORT_CLOSE,
        TRANSPORT_WRITE,
        TRANSPORT_READ
    );

    // Initialize micro-ROS
    if (!init_micro_ros()) {
        printf("ERROR: micro-ROS initialization failed!\n");
        node_set_state(NODE_STATE_ERROR);
        led_set_state(NODE_STATE_ERROR);
        while (1) {
            led_update();
            sleep_ms(100);
        }
    }

    // Enter unadopted state
    node_set_state(NODE_STATE_UNADOPTED);
    led_set_state(NODE_STATE_UNADOPTED);

    printf("Node ready. Waiting for adoption...\n");
    printf("========================================\n");

    // Main loop
    while (1) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // Update node state
        node_state_update();
        g_node.uptime_ms = now;

        // Update LED
        led_update();

        // Spin micro-ROS executor (process timers and callbacks)
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        // Small delay
        sleep_ms(10);
    }

    // Cleanup (never reached)
    cleanup_micro_ros();

    return 0;
}
