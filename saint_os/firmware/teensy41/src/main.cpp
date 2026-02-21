/**
 * SAINT.OS Node Firmware - Main Entry Point (Teensy 4.1)
 *
 * Uses Arduino framework with micro-ROS for communication.
 */

#include <Arduino.h>
#include <string.h>

extern "C" {
#include "saint_node.h"
#include "version.h"
#include "pin_config.h"
#include "pin_control.h"
#include "flash_storage.h"
#include "peripheral_driver.h"
#include "maestro_driver.h"
#include "syren_driver.h"
#include "fas100_driver.h"
}

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>

// Transport selection
#ifdef SIMULATION
extern "C" {
#include "transport_udp_bridge.h"
}
#define TRANSPORT_INIT()       transport_udp_bridge_init()
#define TRANSPORT_CONNECT()    transport_udp_bridge_connect()
#define TRANSPORT_CONNECTED()  transport_udp_bridge_is_connected()
#define TRANSPORT_OPEN         transport_udp_bridge_open
#define TRANSPORT_CLOSE        transport_udp_bridge_close
#define TRANSPORT_WRITE        transport_udp_bridge_write
#define TRANSPORT_READ         transport_udp_bridge_read
#define TRANSPORT_FRAMED       false
#define TRANSPORT_NAME         "UDP Bridge (Simulation)"
#define TRANSPORT_SET_AGENT(ip, port) transport_udp_bridge_set_agent(ip, port)
#define TRANSPORT_GET_IP(ip)   transport_udp_bridge_get_ip(ip)
#define TRANSPORT_SET_IP(ip)   transport_udp_bridge_set_ip(ip)
#else
extern "C" {
#include "transport_native_eth.h"
#include "discovery.h"
}
#define TRANSPORT_INIT()       transport_native_eth_init()
#define TRANSPORT_CONNECT()    transport_native_eth_connect()
#define TRANSPORT_CONNECTED()  transport_native_eth_is_connected()
#define TRANSPORT_OPEN         transport_native_eth_open
#define TRANSPORT_CLOSE        transport_native_eth_close
#define TRANSPORT_WRITE        transport_native_eth_write
#define TRANSPORT_READ         transport_native_eth_read
#define TRANSPORT_FRAMED       false
#define TRANSPORT_NAME         "NativeEthernet (UDP)"
#define TRANSPORT_SET_AGENT(ip, port) transport_native_eth_set_agent(ip, port)
#define TRANSPORT_GET_IP(ip)   transport_native_eth_get_ip(ip)
#define TRANSPORT_GET_MAC(mac) transport_native_eth_get_mac(mac)
#endif

// Global node configuration
saint_node_config_t g_node;

// micro-ROS handles
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t ros_node;
static rclc_executor_t executor;

// Publishers
static rcl_publisher_t announcement_pub;
static rcl_publisher_t capabilities_pub;
static rcl_publisher_t state_pub;

// Subscribers
static rcl_subscription_t config_sub;
static rcl_subscription_t control_sub;

// Timers
static rcl_timer_t announce_timer;
static rcl_timer_t state_timer;

// Message buffers
static std_msgs__msg__String announcement_msg;
static char announcement_buffer[256];

// NOTE: Current Msg is over 4K in size
static std_msgs__msg__String capabilities_msg;
static char capabilities_buffer[8 *1024];

static std_msgs__msg__String config_msg;
static char config_buffer[2048];

static std_msgs__msg__String control_msg;
static char control_buffer[512];

static std_msgs__msg__String state_msg;
static char state_buffer[2048];

// Flags
static bool capabilities_requested = false;

#define STATE_PUBLISH_INTERVAL_MS 100

// Connection monitoring
#define CONNECTION_CHECK_INTERVAL_MS 5000
#define CONNECTION_TIMEOUT_MS        15000
#define MAX_RECONNECT_ATTEMPTS       10

static uint32_t last_successful_comm = 0;
static uint32_t last_connection_check = 0;
static uint32_t reconnect_attempts = 0;
static bool agent_connected = false;

// Forward declarations
static void mark_agent_communication(void);
static bool check_agent_connection(void);
static bool init_micro_ros(void);
static void cleanup_micro_ros(void);

// ============================================================================
// Subscription Callbacks
// ============================================================================

static void config_subscription_callback(const void* msgin)
{
    const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;
    if (!msg || !msg->data.data) return;

    if (msg->data.size >= sizeof(config_buffer)) {
        Serial.printf("Config message too large: %zu >= %zu, rejecting\n",
                       msg->data.size, sizeof(config_buffer));
        return;
    }

    config_buffer[msg->data.size] = '\0';

    Serial.printf("Config received: %.*s\n",
                   (int)(msg->data.size < 100 ? msg->data.size : 100),
                   msg->data.data);

    if (strstr(msg->data.data, "\"action\":\"request_capabilities\"") ||
        strstr(msg->data.data, "\"action\": \"request_capabilities\"")) {
        capabilities_requested = true;
        return;
    }

    if (strstr(msg->data.data, "\"action\":\"configure\"") ||
        strstr(msg->data.data, "\"action\": \"configure\"")) {
        if (pin_config_apply_json(msg->data.data, msg->data.size)) {
            Serial.printf("Pin configuration applied successfully\n");
            if (pin_config_save()) {
                Serial.printf("Pin configuration saved to flash\n");
            }
            if (g_node.state != NODE_STATE_ACTIVE) {
                node_set_state(NODE_STATE_ACTIVE);
                led_set_state(NODE_STATE_ACTIVE);
            }
            capabilities_requested = true;
        }
    }
}

static uint32_t extract_version_timestamp(const char* version)
{
    if (!version) return 0;
    const char* dash = strchr(version, '-');
    if (!dash) return 0;
    uint32_t timestamp = 0;
    const char* p = dash + 1;
    while (*p >= '0' && *p <= '9') {
        timestamp = timestamp * 10 + (*p - '0');
        p++;
    }
    return timestamp;
}

static void handle_firmware_update(const char* json, size_t len)
{
    if (!json || len == 0 || len > 512) return;

    bool force_update = (strstr(json, "\"force\":true") || strstr(json, "\"force\": true"));

    const char* version_str = strstr(json, "\"version\"");
    char new_version[48] = "unknown";
    if (version_str) {
        version_str = strchr(version_str, ':');
        if (version_str) {
            version_str++;
            while (*version_str == ' ' || *version_str == '"') version_str++;
            size_t i = 0;
            while (version_str[i] && version_str[i] != '"' && i < sizeof(new_version) - 1) {
                new_version[i] = version_str[i];
                i++;
            }
            new_version[i] = '\0';
        }
    }

    Serial.printf("\n====================================\n");
    Serial.printf("FIRMWARE UPDATE REQUESTED\n");
    Serial.printf("  Current: %s\n", FIRMWARE_VERSION_FULL);
    Serial.printf("  Server:  %s\n", new_version);
    Serial.printf("  Force:   %s\n", force_update ? "YES" : "NO");

    if (!force_update) {
        uint32_t current_ts = FIRMWARE_BUILD_UNIX;
        uint32_t server_ts = extract_version_timestamp(new_version);
        if (server_ts == 0 || server_ts <= current_ts) {
            Serial.printf("Firmware is already up to date.\n====================================\n\n");
            return;
        }
    }

    Serial.printf("Rebooting for firmware update...\n====================================\n\n");
    delay(500);

#ifdef SIMULATION
    while (1) { yield(); }
#else
    // Teensy restart
    SCB_AIRCR = 0x05FA0004;
    while (1) { }
#endif
}

static void control_subscription_callback(const void* msgin)
{
    const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;
    if (!msg || !msg->data.data) return;

    if (msg->data.size >= sizeof(control_buffer)) return;

    control_buffer[msg->data.size] = '\0';

    Serial.printf("Control received: %.*s\n",
                   (int)(msg->data.size < 80 ? msg->data.size : 80),
                   msg->data.data);

    if (strstr(msg->data.data, "\"action\":\"firmware_update\"") ||
        strstr(msg->data.data, "\"action\": \"firmware_update\"")) {
        handle_firmware_update(msg->data.data, msg->data.size);
        return;
    }

    if (strstr(msg->data.data, "\"action\":\"factory_reset\"") ||
        strstr(msg->data.data, "\"action\": \"factory_reset\"")) {
        flash_storage_erase();
        pin_config_reset();
        node_set_state(NODE_STATE_UNADOPTED);
        led_set_state(NODE_STATE_UNADOPTED);
        Serial.printf("Factory reset complete\n");
        return;
    }

    if (strstr(msg->data.data, "\"action\":\"restart\"") ||
        strstr(msg->data.data, "\"action\": \"restart\"")) {
        Serial.printf("Restart requested\n");
        delay(500);
#ifdef SIMULATION
        while (1) { yield(); }
#else
        SCB_AIRCR = 0x05FA0004;
        while (1) { }
#endif
    }

    if (strstr(msg->data.data, "\"action\":\"identify\"") ||
        strstr(msg->data.data, "\"action\": \"identify\"")) {
        led_identify(5);
        return;
    }

    if (strstr(msg->data.data, "\"action\":\"estop\"") ||
        strstr(msg->data.data, "\"action\": \"estop\"")) {
        pin_control_estop();
        return;
    }

    pin_control_apply_json(msg->data.data, msg->data.size);
}

// ============================================================================
// Publishing
// ============================================================================

static void publish_capabilities(void)
{
    int len = pin_config_capabilities_to_json(
        capabilities_buffer, sizeof(capabilities_buffer), g_node.node_id);
    if (len < 0) 
    {
        Serial.printf("Failed to generate capabilities JSON\n");
        return;
    }

    capabilities_msg.data.data = capabilities_buffer;
    capabilities_msg.data.size = (size_t)len;
    capabilities_msg.data.capacity = sizeof(capabilities_buffer);

    rcl_publish(&capabilities_pub, &capabilities_msg, NULL);
}

static void publish_state(void)
{
    pin_control_update_state();

    int len = pin_control_state_to_json(
        state_buffer, sizeof(state_buffer), g_node.node_id);
    if (len < 0) return;

    state_msg.data.data = state_buffer;
    state_msg.data.size = (size_t)len;
    state_msg.data.capacity = sizeof(state_buffer);

    rcl_publish(&state_pub, &state_msg, NULL);
}

static void state_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer && g_node.state == NODE_STATE_ACTIVE) {
        publish_state();
    }
}

static void announce_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (!timer) return;

    if (capabilities_requested) {
        capabilities_requested = false;
        publish_capabilities();
    }

    if (g_node.state != NODE_STATE_UNADOPTED && g_node.state != NODE_STATE_ACTIVE) {
        return;
    }

    float cpu_temp = hardware_get_cpu_temp();

    int ann_len = snprintf(announcement_buffer, sizeof(announcement_buffer),
        "{"
        "\"node_id\":\"%s\","
        "\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
        "\"ip\":\"%d.%d.%d.%d\","
        "\"hw\":\"%s\","
        "\"fw\":\"%s\","
        "\"fw_build\":\"%s\","
        "\"state\":\"%s\","
        "\"uptime\":%lu,"
        "\"cpu_temp\":%.1f,"
        "\"peripherals\":{",
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

    // Add peripheral connection status
    for (uint8_t d = 0; d < peripheral_get_count(); d++) {
        const peripheral_driver_t* drv = peripheral_get(d);
        if (!drv) continue;
        bool connected = drv->is_connected ? drv->is_connected() : false;
        ann_len += snprintf(announcement_buffer + ann_len,
            sizeof(announcement_buffer) - ann_len,
            "%s\"%s_connected\":%s",
            d > 0 ? "," : "", drv->name, connected ? "true" : "false");
    }

    snprintf(announcement_buffer + ann_len,
        sizeof(announcement_buffer) - ann_len, "}}");
    ann_len = strlen(announcement_buffer);

    announcement_msg.data.data = announcement_buffer;
    announcement_msg.data.size = strlen(announcement_buffer);
    announcement_msg.data.capacity = sizeof(announcement_buffer);

    rcl_ret_t ret = rcl_publish(&announcement_pub, &announcement_msg, NULL);
    if (ret == RCL_RET_OK) {
        mark_agent_communication();
    }
}

// ============================================================================
// micro-ROS Init/Cleanup
// ============================================================================

static bool init_micro_ros(void)
{
    Serial.printf("Initializing micro-ROS...\n");

    allocator = rcl_get_default_allocator();

    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) return false;

    char node_name[64];
    snprintf(node_name, sizeof(node_name), "saint_node_%s", g_node.node_id);
    for (char* p = node_name; *p; p++) {
        if (*p == '-' || *p == ':') *p = '_';
    }

    ret = rclc_node_init_default(&ros_node, node_name, "saint", &support);
    if (ret != RCL_RET_OK) return false;

    // Announcement publisher
    ret = rclc_publisher_init_default(&announcement_pub, &ros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/saint/nodes/announce");
    if (ret != RCL_RET_OK) return false;

    // Capabilities publisher
    char topic[64];
    snprintf(topic, sizeof(topic), "/saint/nodes/%s/capabilities", g_node.node_id);
    for (char* p = topic; *p; p++) { if (*p == '-' || *p == ':') *p = '_'; }
    ret = rclc_publisher_init_default(&capabilities_pub, &ros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic);
    if (ret != RCL_RET_OK) return false;

    // Config subscriber
    snprintf(topic, sizeof(topic), "/saint/nodes/%s/config", g_node.node_id);
    for (char* p = topic; *p; p++) { if (*p == '-' || *p == ':') *p = '_'; }
    ret = rclc_subscription_init_default(&config_sub, &ros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic);
    if (ret != RCL_RET_OK) return false;
    config_msg.data.data = config_buffer;
    config_msg.data.size = 0;
    config_msg.data.capacity = sizeof(config_buffer);

    // Control subscriber
    snprintf(topic, sizeof(topic), "/saint/nodes/%s/control", g_node.node_id);
    for (char* p = topic; *p; p++) { if (*p == '-' || *p == ':') *p = '_'; }
    ret = rclc_subscription_init_default(&control_sub, &ros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic);
    if (ret != RCL_RET_OK) return false;
    control_msg.data.data = control_buffer;
    control_msg.data.size = 0;
    control_msg.data.capacity = sizeof(control_buffer);

    // State publisher
    snprintf(topic, sizeof(topic), "/saint/nodes/%s/state", g_node.node_id);
    for (char* p = topic; *p; p++) { if (*p == '-' || *p == ':') *p = '_'; }
    ret = rclc_publisher_init_default(&state_pub, &ros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic);
    if (ret != RCL_RET_OK) return false;

    // Timers
    ret = rclc_timer_init_default(&announce_timer, &support,
        RCL_MS_TO_NS(ANNOUNCE_INTERVAL_MS), announce_timer_callback);
    if (ret != RCL_RET_OK) return false;

    ret = rclc_timer_init_default(&state_timer, &support,
        RCL_MS_TO_NS(STATE_PUBLISH_INTERVAL_MS), state_timer_callback);
    if (ret != RCL_RET_OK) return false;

    // Executor (2 timers + 2 subscriptions)
    ret = rclc_executor_init(&executor, &support.context, 4, &allocator);
    if (ret != RCL_RET_OK) return false;

    rclc_executor_add_timer(&executor, &announce_timer);
    rclc_executor_add_timer(&executor, &state_timer);
    rclc_executor_add_subscription(&executor, &config_sub, &config_msg,
        config_subscription_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &control_sub, &control_msg,
        control_subscription_callback, ON_NEW_DATA);

    Serial.printf("micro-ROS initialized successfully\n");
    return true;
}

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

// ============================================================================
// Connection Monitoring
// ============================================================================

static void mark_agent_communication(void)
{
    last_successful_comm = millis();
    if (!agent_connected) {
        agent_connected = true;
        reconnect_attempts = 0;
    }
}

static bool check_agent_connection(void)
{
    uint32_t now = millis();

    if (now - last_connection_check < CONNECTION_CHECK_INTERVAL_MS) {
        return agent_connected;
    }
    last_connection_check = now;

    if (!TRANSPORT_CONNECTED()) {
        agent_connected = false;
    }

    if (agent_connected && last_successful_comm > 0) {
        if (now - last_successful_comm > CONNECTION_TIMEOUT_MS) {
            agent_connected = false;
        }
    }

    if (!agent_connected) {
        reconnect_attempts++;

        if (reconnect_attempts > MAX_RECONNECT_ATTEMPTS) {
            node_set_state(NODE_STATE_ERROR);
            led_set_state(NODE_STATE_ERROR);
            return false;
        }

        led_set_state(NODE_STATE_CONNECTING);
        delay(RECONNECT_DELAY_MS);

        if (!TRANSPORT_CONNECTED()) {
            if (!TRANSPORT_CONNECT()) return false;
        }

        cleanup_micro_ros();
        delay(500);

        if (!init_micro_ros()) return false;

        agent_connected = true;
        last_successful_comm = now;
        reconnect_attempts = 0;
        led_set_state(g_node.state);
        capabilities_requested = true;
    }

    return agent_connected;
}

// ============================================================================
// Arduino Entry Points
// ============================================================================

void setup()
{
    // Initialize serial
    Serial.begin(115200);
    delay(100);

    Serial.printf("\n\n");
    Serial.printf("****************************************\n");
    Serial.printf("* SAINT.OS Node Firmware\n");
    Serial.printf("* Version: %s\n", FIRMWARE_VERSION_FULL);
    Serial.printf("* Built:   %s\n", FIRMWARE_BUILD_TIMESTAMP);
    Serial.printf("* Hardware: %s\n", HARDWARE_MODEL);
#ifdef SIMULATION
    Serial.printf("* Mode:    SIMULATION (UART/UDP)\n");
#else
    Serial.printf("* Mode:    HARDWARE (NativeEthernet)\n");
#endif
    Serial.printf("****************************************\n\n");

    // Initialize node state
    node_state_init();

    // Initialize pin configuration
    pin_config_init();

    // Load saved pin configuration
    if (pin_config_load()) {
        Serial.printf("Loaded pin configuration from flash\n");
    }

    // Initialize pin control
    pin_control_init();

    // Register and initialize peripheral drivers
    peripheral_register(maestro_get_peripheral_driver());
    peripheral_register(syren_get_peripheral_driver());
    peripheral_register(fas100_get_peripheral_driver());
    peripheral_init_all();

    // Initialize hardware
    hardware_init();

    // Initialize LED
    led_init();
    led_set_state(NODE_STATE_BOOT);

    // Get unique ID
    char unique_id[32];
    hardware_get_unique_id(unique_id, sizeof(unique_id));
    snprintf(g_node.node_id, sizeof(g_node.node_id), "teensy41_%s", unique_id);
    Serial.printf("Node ID: %s\n", g_node.node_id);

    // Initialize flash storage
    flash_storage_init();

    // Initialize transport
    Serial.printf("Initializing transport: %s\n", TRANSPORT_NAME);
    led_set_state(NODE_STATE_CONNECTING);

    if (!TRANSPORT_INIT()) {
        Serial.printf("ERROR: Transport init failed!\n");
        node_set_state(NODE_STATE_ERROR);
        led_set_state(NODE_STATE_ERROR);
        return;
    }

#ifdef SIMULATION
    char uid[16];
    hardware_get_unique_id(uid, sizeof(uid));
    g_node.mac_address[0] = 0x02;
    for (int i = 0; i < 5 && uid[i*2]; i++) {
        char hex[3] = {uid[i*2], uid[i*2+1], 0};
        g_node.mac_address[i+1] = (uint8_t)strtol(hex, NULL, 16);
    }
    g_node.static_ip[0] = 192;
    g_node.static_ip[1] = 168;
    g_node.static_ip[2] = 1;
    g_node.static_ip[3] = 100;
    TRANSPORT_SET_IP(g_node.static_ip);
#else
    TRANSPORT_GET_MAC(g_node.mac_address);
#endif

    Serial.printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                   g_node.mac_address[0], g_node.mac_address[1],
                   g_node.mac_address[2], g_node.mac_address[3],
                   g_node.mac_address[4], g_node.mac_address[5]);

    if (!TRANSPORT_CONNECT()) {
        Serial.printf("ERROR: Transport connect failed!\n");
        node_set_state(NODE_STATE_ERROR);
        led_set_state(NODE_STATE_ERROR);
        return;
    }

#ifndef SIMULATION
    TRANSPORT_GET_IP(g_node.static_ip);
#endif

    Serial.printf("IP: %d.%d.%d.%d\n",
                   g_node.static_ip[0], g_node.static_ip[1],
                   g_node.static_ip[2], g_node.static_ip[3]);

#ifndef SIMULATION
    Serial.printf("Discovering SAINT server...\n");
    if (!discover_server(g_node.server_ip, &g_node.server_port, 2000, 10)) {
        Serial.printf("ERROR: Could not discover SAINT server!\n");
        node_set_state(NODE_STATE_ERROR);
        led_set_state(NODE_STATE_ERROR);
        return;
    }
#endif

    TRANSPORT_SET_AGENT(g_node.server_ip, g_node.server_port);

    rmw_uros_set_custom_transport(
        TRANSPORT_FRAMED, NULL,
        TRANSPORT_OPEN, TRANSPORT_CLOSE,
        TRANSPORT_WRITE, TRANSPORT_READ
    );

    if (!init_micro_ros()) {
        Serial.printf("ERROR: micro-ROS init failed!\n");
        node_set_state(NODE_STATE_ERROR);
        led_set_state(NODE_STATE_ERROR);
        return;
    }

    if (pin_config_has_configured_pins()) {
        node_set_state(NODE_STATE_ACTIVE);
        led_set_state(NODE_STATE_ACTIVE);
        Serial.printf("Node ready (ACTIVE - restored from saved config)\n");
    } else {
        node_set_state(NODE_STATE_UNADOPTED);
        led_set_state(NODE_STATE_UNADOPTED);
        Serial.printf("Node ready. Waiting for adoption...\n");
    }
    Serial.printf("========================================\n");

    last_successful_comm = millis();
    agent_connected = true;
}

static uint32_t last_status_print = 0;

void loop()
{
    uint32_t now = millis();

    // Update state
    node_state_update();
    g_node.uptime_ms = now;

    // Update LED
    led_update();

    // Poll peripheral drivers (Maestro USB host, SyRen, etc.)
    peripheral_update_all();

    // Error state - just blink LED
    if (g_node.state == NODE_STATE_ERROR) {
        delay(100);
        return;
    }

    // Check connection
    if (!check_agent_connection()) {
        delay(100);
        return;
    }

    // Spin executor
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    // Periodic status
    if (now - last_status_print >= 10000) {
        Serial.printf("[%lu] state: %s, agent: %s\n",
                       now / 1000, node_state_to_string(g_node.state),
                       agent_connected ? "connected" : "disconnected");
        last_status_print = now;
    }

    delay(10);
}
