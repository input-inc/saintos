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
#include "hardware/watchdog.h"

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
#include "pin_config.h"
#include "pin_control.h"
#include "flash_storage.h"

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
#include "discovery.h"
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
static rcl_publisher_t capabilities_pub;
static rcl_publisher_t state_pub;          // Pin state publisher

// Subscribers
static rcl_subscription_t config_sub;
static rcl_subscription_t control_sub;     // Pin control subscription

// Timers
static rcl_timer_t announce_timer;
static rcl_timer_t state_timer;            // State publish timer (10Hz)

// Message buffers
static std_msgs__msg__String announcement_msg;
static char announcement_buffer[256];

static std_msgs__msg__String capabilities_msg;
static char capabilities_buffer[2048];  // Larger buffer for capabilities JSON

static std_msgs__msg__String config_msg;
static char config_buffer[2048];        // Buffer for incoming config

static std_msgs__msg__String control_msg;
static char control_buffer[512];        // Buffer for incoming control commands

static std_msgs__msg__String state_msg;
static char state_buffer[2048];         // Buffer for outgoing state

// Flags
static bool capabilities_requested = false;

// State publish interval
#define STATE_PUBLISH_INTERVAL_MS 100   // 10Hz

// Connection monitoring
#define CONNECTION_CHECK_INTERVAL_MS 5000   // Check every 5 seconds
#define CONNECTION_TIMEOUT_MS        15000  // Consider disconnected after 15s
#define MAX_RECONNECT_ATTEMPTS       10     // Max consecutive failures before error state
// Note: RECONNECT_DELAY_MS is defined in saint_node.h

static uint32_t last_successful_comm = 0;   // Timestamp of last successful communication
static uint32_t last_connection_check = 0;  // Timestamp of last connection check
static uint32_t reconnect_attempts = 0;     // Consecutive reconnect attempts
static bool agent_connected = false;        // Agent connection state

// Forward declarations for connection monitoring
static void mark_agent_communication(void);
static bool check_agent_connection(void);
static bool init_micro_ros(void);

// =============================================================================
// Subscription Callbacks
// =============================================================================

/**
 * Subscription callback for configuration messages.
 * Handles pin configuration commands from the server.
 */
static void config_subscription_callback(const void* msgin)
{
    const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;

    if (!msg || !msg->data.data) {
        return;
    }

    // SAFETY: Validate message size before processing
    if (msg->data.size >= sizeof(config_buffer)) {
        printf("Config message too large: %zu >= %zu, rejecting\n",
               msg->data.size, sizeof(config_buffer));
        return;
    }

    // Ensure null termination for safe string operations
    // The buffer should already have the data, but ensure termination
    config_buffer[msg->data.size] = '\0';

    printf("Config received: %.*s\n",
           (int)(msg->data.size < 100 ? msg->data.size : 100),
           msg->data.data);

    // Check for capabilities request
    if (strstr(msg->data.data, "\"action\":\"request_capabilities\"") ||
        strstr(msg->data.data, "\"action\": \"request_capabilities\"")) {
        printf("Capabilities request received\n");
        capabilities_requested = true;
        return;
    }

    // Check for configure action
    if (strstr(msg->data.data, "\"action\":\"configure\"") ||
        strstr(msg->data.data, "\"action\": \"configure\"")) {
        printf("Configuration request received\n");

        if (pin_config_apply_json(msg->data.data, msg->data.size)) {
            printf("Pin configuration applied successfully\n");

            // Save to flash
            if (pin_config_save()) {
                printf("Pin configuration saved to flash\n");
            }

            // Transition to ACTIVE state (node is now adopted)
            if (g_node.state != NODE_STATE_ACTIVE) {
                printf("Node adopted - transitioning to ACTIVE state\n");
                node_set_state(NODE_STATE_ACTIVE);
                led_set_state(NODE_STATE_ACTIVE);
            }

            // Publish updated capabilities/config
            capabilities_requested = true;
        } else {
            printf("Failed to apply pin configuration\n");
        }
    }
}

/**
 * Extract unix timestamp from version string (format: "1.2.0-1738505432")
 * Returns 0 if not found or invalid.
 */
static uint32_t extract_version_timestamp(const char* version)
{
    if (!version) return 0;

    // Find the dash separator
    const char* dash = strchr(version, '-');
    if (!dash) return 0;

    // Parse the number after the dash
    uint32_t timestamp = 0;
    const char* p = dash + 1;
    while (*p >= '0' && *p <= '9') {
        timestamp = timestamp * 10 + (*p - '0');
        p++;
    }

    return timestamp;
}

/**
 * Handle firmware update command.
 * For simulation: triggers a clean exit so Renode can restart with new firmware.
 * For hardware: triggers watchdog reset to enter bootloader mode.
 *
 * If "force" is false, compares version timestamps and only updates if server is newer.
 */
static void handle_firmware_update(const char* json, size_t len)
{
    // SAFETY: Basic validation
    if (!json || len == 0 || len > 512) {
        printf("Firmware update: invalid command\n");
        return;
    }

    // Parse force flag from command (default false)
    bool force_update = false;
    if (strstr(json, "\"force\":true") || strstr(json, "\"force\": true")) {
        force_update = true;
    }

    // Parse version from command
    const char* version_str = strstr(json, "\"version\"");
    char new_version[48] = "unknown";

    if (version_str) {
        version_str = strchr(version_str, ':');
        if (version_str) {
            version_str++;
            // Skip whitespace and opening quote
            while (*version_str == ' ' || *version_str == '"') version_str++;
            // Copy version string
            size_t i = 0;
            while (version_str[i] && version_str[i] != '"' && i < sizeof(new_version) - 1) {
                new_version[i] = version_str[i];
                i++;
            }
            new_version[i] = '\0';
        }
    }

    printf("\n");
    printf("====================================\n");
    printf("FIRMWARE UPDATE REQUESTED\n");
    printf("====================================\n");
    printf("  Current version: %s\n", FIRMWARE_VERSION_FULL);
    printf("  Server version:  %s\n", new_version);
    printf("  Force update:    %s\n", force_update ? "YES" : "NO");

    // If not forced, compare version timestamps
    if (!force_update) {
        uint32_t current_ts = FIRMWARE_BUILD_UNIX;
        uint32_t server_ts = extract_version_timestamp(new_version);

        printf("  Current build:   %lu\n", (unsigned long)current_ts);
        printf("  Server build:    %lu\n", (unsigned long)server_ts);

        if (server_ts == 0) {
            printf("====================================\n");
            printf("Cannot parse server version timestamp.\n");
            printf("Use force update to override.\n");
            printf("====================================\n\n");
            return;
        }

        if (server_ts <= current_ts) {
            printf("====================================\n");
            printf("Firmware is already up to date.\n");
            printf("Use force update to override.\n");
            printf("====================================\n\n");
            return;
        }

        printf("  -> Server has newer build, proceeding with update\n");
    } else {
        printf("  -> Force update enabled, skipping version check\n");
    }

    printf("====================================\n");
    printf("Rebooting for firmware update...\n\n");

    // Small delay to allow message to print
    sleep_ms(500);

#ifdef SIMULATION
    // For simulation: exit cleanly so Renode can restart with new firmware
    // The node manager should restart the simulation with the updated ELF
    printf("Simulation mode: exiting for firmware reload\n");
    // In Renode, we can't really exit, but we can halt the CPU
    // This signals that update is requested
    while (1) {
        // Halt - Renode will need to restart the simulation
        tight_loop_contents();
    }
#else
    // For hardware: use watchdog to reset into bootloader
    // This will cause the RP2040 to reset
    watchdog_enable(1, 1);  // 1ms timeout, pause on debug
    while (1) {
        tight_loop_contents();
    }
#endif
}

/**
 * Subscription callback for control messages.
 * Handles runtime pin value changes from the server.
 */
static void control_subscription_callback(const void* msgin)
{
    const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;

    if (!msg || !msg->data.data) {
        return;
    }

    // SAFETY: Validate message size before processing
    if (msg->data.size >= sizeof(control_buffer)) {
        printf("Control message too large: %zu >= %zu, rejecting\n",
               msg->data.size, sizeof(control_buffer));
        return;
    }

    // Ensure null termination for safe string operations
    control_buffer[msg->data.size] = '\0';

    printf("Control received: %.*s\n",
           (int)(msg->data.size < 80 ? msg->data.size : 80),
           msg->data.data);

    // Check for firmware update command
    if (strstr(msg->data.data, "\"action\":\"firmware_update\"") ||
        strstr(msg->data.data, "\"action\": \"firmware_update\"")) {
        handle_firmware_update(msg->data.data, msg->data.size);
        return;
    }

    // Check for factory reset command
    if (strstr(msg->data.data, "\"action\":\"factory_reset\"") ||
        strstr(msg->data.data, "\"action\": \"factory_reset\"")) {
        printf("\n");
        printf("====================================\n");
        printf("FACTORY RESET REQUESTED\n");
        printf("====================================\n");
        printf("Clearing saved configuration...\n");

        // Clear flash storage (pin configuration)
        flash_storage_erase();

        // Clear in-memory pin configuration
        pin_config_reset();

        // Transition to UNADOPTED state
        node_set_state(NODE_STATE_UNADOPTED);
        led_set_state(NODE_STATE_UNADOPTED);

        printf("Factory reset complete - node is now UNADOPTED\n");
        printf("====================================\n\n");
        return;
    }

    // Check for restart command
    if (strstr(msg->data.data, "\"action\":\"restart\"") ||
        strstr(msg->data.data, "\"action\": \"restart\"")) {
        printf("\n");
        printf("====================================\n");
        printf("RESTART REQUESTED\n");
        printf("====================================\n");
        printf("Rebooting in 500ms...\n");

        sleep_ms(500);

        // Use watchdog to reset the device
        watchdog_enable(1, 1);  // 1ms timeout, pause on debug
        while (1) {
            tight_loop_contents();
        }
        // Never returns
    }

    // Check for identify command
    if (strstr(msg->data.data, "\"action\":\"identify\"") ||
        strstr(msg->data.data, "\"action\": \"identify\"")) {
        printf("\n");
        printf("====================================\n");
        printf("IDENTIFY REQUESTED\n");
        printf("====================================\n");

        // Flash LED to help locate this node
        led_identify(5);  // 5 flash sequences

        printf("Identify complete\n");
        printf("====================================\n\n");
        return;
    }

    // Check for emergency stop command
    if (strstr(msg->data.data, "\"action\":\"estop\"") ||
        strstr(msg->data.data, "\"action\": \"estop\"")) {
        printf("\n");
        printf("====================================\n");
        printf("EMERGENCY STOP ACTIVATED\n");
        printf("====================================\n");

        // Set all outputs to safe values
        pin_control_estop();

        printf("====================================\n\n");
        return;
    }

    // Apply pin control command
    if (pin_control_apply_json(msg->data.data, msg->data.size)) {
        printf("Control command applied\n");
    }
}

/**
 * Publish node capabilities to server.
 */
static void publish_capabilities(void)
{
    int len = pin_config_capabilities_to_json(
        capabilities_buffer, sizeof(capabilities_buffer), g_node.node_id);

    if (len < 0) {
        printf("Failed to generate capabilities JSON\n");
        return;
    }

    capabilities_msg.data.data = capabilities_buffer;
    capabilities_msg.data.size = (size_t)len;
    capabilities_msg.data.capacity = sizeof(capabilities_buffer);

    rcl_ret_t ret = rcl_publish(&capabilities_pub, &capabilities_msg, NULL);
    if (ret == RCL_RET_OK) {
        printf("Published capabilities (%d bytes)\n", len);
    } else {
        printf("Failed to publish capabilities: %d\n", ret);
    }
}

/**
 * Publish current pin state to server.
 */
static void publish_state(void)
{
    // Update all input pins
    pin_control_update_state();

    // Generate state JSON
    int len = pin_control_state_to_json(
        state_buffer, sizeof(state_buffer), g_node.node_id);

    if (len < 0) {
        return;
    }

    state_msg.data.data = state_buffer;
    state_msg.data.size = (size_t)len;
    state_msg.data.capacity = sizeof(state_buffer);

    rcl_ret_t ret = rcl_publish(&state_pub, &state_msg, NULL);
    if (ret != RCL_RET_OK) {
        printf("Failed to publish state: %d\n", ret);
    }
}

/**
 * Timer callback for state publishing (10Hz).
 */
static void state_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
    (void)last_call_time;

    if (timer == NULL) {
        return;
    }

    // Only publish state when active (adopted and has configured pins)
    if (g_node.state == NODE_STATE_ACTIVE) {
        publish_state();
    }
}

// =============================================================================
// Timer Callbacks
// =============================================================================

/**
 * Timer callback for node announcements.
 * Publishes node info to let server know node is online.
 */
static void announce_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
    (void)last_call_time;

    if (timer == NULL) {
        return;
    }

    // Publish capabilities if requested (regardless of state)
    if (capabilities_requested) {
        capabilities_requested = false;
        publish_capabilities();
    }

    // Announce when unadopted or active (for heartbeat/online detection)
    if (g_node.state != NODE_STATE_UNADOPTED && g_node.state != NODE_STATE_ACTIVE) {
        return;
    }

    // Get CPU temperature
    float cpu_temp = hardware_get_cpu_temp();

    // Build announcement JSON string
    snprintf(announcement_buffer, sizeof(announcement_buffer),
        "{"
        "\"node_id\":\"%s\","
        "\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
        "\"ip\":\"%d.%d.%d.%d\","
        "\"hw\":\"%s\","
        "\"fw\":\"%s\","
        "\"fw_build\":\"%s\","
        "\"state\":\"%s\","
        "\"uptime\":%lu,"
        "\"cpu_temp\":%.1f"
        "}",
        g_node.node_id,
        g_node.mac_address[0], g_node.mac_address[1],
        g_node.mac_address[2], g_node.mac_address[3],
        g_node.mac_address[4], g_node.mac_address[5],
        g_node.static_ip[0], g_node.static_ip[1],
        g_node.static_ip[2], g_node.static_ip[3],
        HARDWARE_MODEL,
        FIRMWARE_VERSION_FULL,
        FIRMWARE_BUILD_TIMESTAMP,
        node_state_to_string(g_node.state),
        g_node.uptime_ms / 1000,
        cpu_temp
    );

    announcement_msg.data.data = announcement_buffer;
    announcement_msg.data.size = strlen(announcement_buffer);
    announcement_msg.data.capacity = sizeof(announcement_buffer);

    rcl_ret_t ret = rcl_publish(&announcement_pub, &announcement_msg, NULL);
    if (ret == RCL_RET_OK) {
        mark_agent_communication();
    } else {
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
        printf("Failed to create announcement publisher: %d\n", ret);
        return false;
    }

    // Create capabilities publisher
    char capabilities_topic[64];
    snprintf(capabilities_topic, sizeof(capabilities_topic),
             "/saint/nodes/%s/capabilities", g_node.node_id);
    // Replace invalid chars
    for (char* p = capabilities_topic; *p; p++) {
        if (*p == '-' || *p == ':') *p = '_';
    }

    ret = rclc_publisher_init_default(
        &capabilities_pub,
        &ros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        capabilities_topic
    );
    if (ret != RCL_RET_OK) {
        printf("Failed to create capabilities publisher: %d\n", ret);
        return false;
    }
    printf("Created publisher: %s\n", capabilities_topic);

    // Create config subscriber
    char config_topic[64];
    snprintf(config_topic, sizeof(config_topic),
             "/saint/nodes/%s/config", g_node.node_id);
    // Replace invalid chars
    for (char* p = config_topic; *p; p++) {
        if (*p == '-' || *p == ':') *p = '_';
    }

    printf("Creating subscription for topic: %s\n", config_topic);
    printf("Node ID: %s\n", g_node.node_id);

    ret = rclc_subscription_init_default(
        &config_sub,
        &ros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        config_topic
    );
    if (ret != RCL_RET_OK) {
        printf("Failed to create config subscription: %d\n", ret);
        return false;
    }
    printf("Successfully subscribed to: %s\n", config_topic);

    // Initialize message buffer for subscription
    config_msg.data.data = config_buffer;
    config_msg.data.size = 0;
    config_msg.data.capacity = sizeof(config_buffer);

    // Create control subscriber
    char control_topic[64];
    snprintf(control_topic, sizeof(control_topic),
             "/saint/nodes/%s/control", g_node.node_id);
    // Replace invalid chars
    for (char* p = control_topic; *p; p++) {
        if (*p == '-' || *p == ':') *p = '_';
    }

    ret = rclc_subscription_init_default(
        &control_sub,
        &ros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        control_topic
    );
    if (ret != RCL_RET_OK) {
        printf("Failed to create control subscription: %d\n", ret);
        return false;
    }
    printf("Subscribed to control: %s\n", control_topic);

    // Initialize control message buffer
    control_msg.data.data = control_buffer;
    control_msg.data.size = 0;
    control_msg.data.capacity = sizeof(control_buffer);

    // Create state publisher
    char state_topic[64];
    snprintf(state_topic, sizeof(state_topic),
             "/saint/nodes/%s/state", g_node.node_id);
    // Replace invalid chars
    for (char* p = state_topic; *p; p++) {
        if (*p == '-' || *p == ':') *p = '_';
    }

    ret = rclc_publisher_init_default(
        &state_pub,
        &ros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        state_topic
    );
    if (ret != RCL_RET_OK) {
        printf("Failed to create state publisher: %d\n", ret);
        return false;
    }
    printf("Created state publisher: %s\n", state_topic);

    // Create announcement timer (1 second interval)
    ret = rclc_timer_init_default(
        &announce_timer,
        &support,
        RCL_MS_TO_NS(ANNOUNCE_INTERVAL_MS),
        announce_timer_callback
    );
    if (ret != RCL_RET_OK) {
        printf("Failed to create announcement timer: %d\n", ret);
        return false;
    }

    // Create state timer (100ms = 10Hz)
    ret = rclc_timer_init_default(
        &state_timer,
        &support,
        RCL_MS_TO_NS(STATE_PUBLISH_INTERVAL_MS),
        state_timer_callback
    );
    if (ret != RCL_RET_OK) {
        printf("Failed to create state timer: %d\n", ret);
        return false;
    }

    // Initialize executor with 2 timers + 2 subscriptions
    ret = rclc_executor_init(&executor, &support.context, 4, &allocator);
    if (ret != RCL_RET_OK) {
        printf("Failed to create executor: %d\n", ret);
        return false;
    }

    // Add announce timer to executor
    ret = rclc_executor_add_timer(&executor, &announce_timer);
    if (ret != RCL_RET_OK) {
        printf("Failed to add announce timer: %d\n", ret);
        return false;
    }

    // Add state timer to executor
    ret = rclc_executor_add_timer(&executor, &state_timer);
    if (ret != RCL_RET_OK) {
        printf("Failed to add state timer: %d\n", ret);
        return false;
    }

    // Add config subscription to executor
    ret = rclc_executor_add_subscription(
        &executor,
        &config_sub,
        &config_msg,
        config_subscription_callback,
        ON_NEW_DATA
    );
    if (ret != RCL_RET_OK) {
        printf("Failed to add config subscription: %d\n", ret);
        return false;
    }

    // Add control subscription to executor
    ret = rclc_executor_add_subscription(
        &executor,
        &control_sub,
        &control_msg,
        control_subscription_callback,
        ON_NEW_DATA
    );
    if (ret != RCL_RET_OK) {
        printf("Failed to add control subscription: %d\n", ret);
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
    rcl_subscription_fini(&control_sub, &ros_node);
    rcl_subscription_fini(&config_sub, &ros_node);
    rcl_publisher_fini(&state_pub, &ros_node);
    rcl_publisher_fini(&capabilities_pub, &ros_node);
    rcl_publisher_fini(&announcement_pub, &ros_node);
    rcl_timer_fini(&state_timer);
    rcl_timer_fini(&announce_timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&ros_node);
    rclc_support_fini(&support);
}

/**
 * Mark successful communication with agent.
 * Call this when we successfully send or receive data.
 */
static void mark_agent_communication(void)
{
    last_successful_comm = to_ms_since_boot(get_absolute_time());
    if (!agent_connected) {
        agent_connected = true;
        reconnect_attempts = 0;
        printf("Agent connection established\n");
    }
}

/**
 * Check agent connection health and attempt reconnect if needed.
 * Returns true if connected and healthy, false if reconnecting.
 */
static bool check_agent_connection(void)
{
    uint32_t now = to_ms_since_boot(get_absolute_time());

    // Only check periodically
    if (now - last_connection_check < CONNECTION_CHECK_INTERVAL_MS) {
        return agent_connected;
    }
    last_connection_check = now;

    // Check transport layer connection
    if (!TRANSPORT_CONNECTED()) {
        printf("Transport disconnected\n");
        agent_connected = false;
    }

    // Check for communication timeout
    if (agent_connected && last_successful_comm > 0) {
        if (now - last_successful_comm > CONNECTION_TIMEOUT_MS) {
            printf("Agent communication timeout (no response for %lu ms)\n",
                   now - last_successful_comm);
            agent_connected = false;
        }
    }

    // If disconnected, attempt to reconnect
    if (!agent_connected) {
        reconnect_attempts++;

        if (reconnect_attempts > MAX_RECONNECT_ATTEMPTS) {
            // Too many failures - enter error state
            printf("Max reconnect attempts exceeded (%d), entering ERROR state\n",
                   MAX_RECONNECT_ATTEMPTS);
            node_set_state(NODE_STATE_ERROR);
            led_set_state(NODE_STATE_ERROR);
            return false;
        }

        printf("Reconnect attempt %lu/%d...\n", reconnect_attempts, MAX_RECONNECT_ATTEMPTS);
        led_set_state(NODE_STATE_CONNECTING);

        // Wait before reconnect
        sleep_ms(RECONNECT_DELAY_MS);

        // Try to reconnect transport
        if (!TRANSPORT_CONNECTED()) {
            if (!TRANSPORT_CONNECT()) {
                printf("Transport reconnect failed\n");
                return false;
            }
        }

        // Reinitialize micro-ROS
        printf("Reinitializing micro-ROS...\n");
        cleanup_micro_ros();
        sleep_ms(500);

        if (!init_micro_ros()) {
            printf("micro-ROS reinitialization failed\n");
            return false;
        }

        // Success!
        agent_connected = true;
        last_successful_comm = now;
        reconnect_attempts = 0;

        // Restore LED state
        led_set_state(g_node.state);
        printf("Reconnected successfully!\n");

        // Re-publish capabilities so server knows we're back
        capabilities_requested = true;
    }

    return agent_connected;
}

// =============================================================================
// Main Function
// =============================================================================

int main(void)
{
    // Initialize stdio (USB serial for hardware, UART for simulation)
    stdio_init_all();

    // Brief delay for UART to stabilize
    sleep_ms(100);

    // Print version immediately - this is the first output
    printf("\n\n");
    printf("****************************************\n");
    printf("* SAINT.OS Node Firmware\n");
    printf("* Version: %s\n", FIRMWARE_VERSION_FULL);
    printf("* Built:   %s\n", FIRMWARE_BUILD_TIMESTAMP);
    printf("* Hardware: %s\n", HARDWARE_MODEL);
#ifdef SIMULATION
    printf("* Mode:    SIMULATION (UART/UDP)\n");
#else
    printf("* Mode:    HARDWARE (USB/W5500)\n");
#endif
    printf("****************************************\n");
    printf("\n");

    // Additional startup delay for USB enumeration (hardware only)
#ifndef SIMULATION
    sleep_ms(1900);  // Total 2s with the 100ms above
#endif

    // Initialize node state
    node_state_init();

    // Initialize pin configuration
    pin_config_init();

    // Load saved pin configuration
    if (pin_config_load()) {
        printf("Loaded pin configuration from flash\n");
    }

    // Initialize pin control (after pin_config)
    pin_control_init();

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

#ifndef SIMULATION
    // Discover the SAINT server via UDP broadcast
    printf("Discovering SAINT server...\n");
    if (!discover_server(g_node.server_ip, &g_node.server_port, 2000, 10)) {
        printf("ERROR: Could not discover SAINT server!\n");
        printf("Make sure the server is running on the same network.\n");
        node_set_state(NODE_STATE_ERROR);
        led_set_state(NODE_STATE_ERROR);
        while (1) {
            led_update();
            sleep_ms(100);
        }
    }
#endif

    // Set agent address (discovered or from config for simulation)
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

    // Check if we have a saved configuration (previously adopted)
    if (pin_config_has_configured_pins()) {
        // Node was previously adopted and has saved config
        node_set_state(NODE_STATE_ACTIVE);
        led_set_state(NODE_STATE_ACTIVE);
        printf("Node ready. Restored from saved config (ACTIVE)\n");
    } else {
        // No saved config - enter unadopted state
        node_set_state(NODE_STATE_UNADOPTED);
        led_set_state(NODE_STATE_UNADOPTED);
        printf("Node ready. Waiting for adoption...\n");
    }
    printf("========================================\n");

    // Initialize connection tracking
    last_successful_comm = to_ms_since_boot(get_absolute_time());
    agent_connected = true;

    // Main loop
    static uint32_t last_status_print = 0;
    while (1) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // Update node state
        node_state_update();
        g_node.uptime_ms = now;

        // Update LED
        led_update();

        // Check agent connection and reconnect if needed
        if (g_node.state != NODE_STATE_ERROR) {
            if (!check_agent_connection()) {
                // Reconnecting - skip normal processing
                sleep_ms(100);
                continue;
            }
        }

        // Spin micro-ROS executor (process timers and callbacks)
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        // Periodic status print (every 10 seconds)
        if (now - last_status_print >= 10000) {
            printf("[%lu] Node running, state: %s, agent: %s, caps_requested: %d\n",
                   now / 1000, node_state_to_string(g_node.state),
                   agent_connected ? "connected" : "disconnected",
                   capabilities_requested);
            last_status_print = now;
        }

        // Small delay
        sleep_ms(10);
    }

    // Cleanup (never reached)
    cleanup_micro_ros();

    return 0;
}
