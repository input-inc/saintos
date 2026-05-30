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
#include "roboclaw_driver.h"
#include "pathfinder_bms_driver.h"
extern "C" {
#include "saint_log.h"
}
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
static rcl_publisher_t state_pub;
static rcl_publisher_t log_pub;            // Node log lines (best-effort)

// Subscribers
static rcl_subscription_t config_sub;
static rcl_subscription_t control_sub;     // Streaming pin/channel writes (BEST_EFFORT)
static rcl_subscription_t command_sub;     // One-shot operator actions (RELIABLE)

// Timers
static rcl_timer_t announce_timer;
static rcl_timer_t state_timer;

// Message buffers
static std_msgs__msg__String announcement_msg;
static char announcement_buffer[512];

static std_msgs__msg__String config_msg;
static char config_buffer[2048];

static std_msgs__msg__String control_msg;
static char control_buffer[512];

static std_msgs__msg__String command_msg;
static char command_buffer[512];

static std_msgs__msg__String state_msg;
static char state_buffer[2048];

static std_msgs__msg__String log_msg;
static char log_buffer[256];               // One log line at a time

// Announces published since boot — used to gate saint_log boot-queue
// drain (the server creates per-node /log subscriptions lazily, on
// first /announce).
static unsigned g_announce_count = 0;

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

    if (strstr(msg->data.data, "\"action\":\"configure\"") ||
        strstr(msg->data.data, "\"action\": \"configure\"")) {
        bool applied = pin_config_apply_json(msg->data.data, msg->data.size);
        if (applied) {
            Serial.printf("Pin configuration applied successfully\n");
            if (pin_config_save()) {
                Serial.printf("Pin configuration saved to flash\n");
            }
        } else {
            // No pins / peripherals to apply (or apply failed cleanly).
            // That's still a valid "you are adopted" signal — the
            // server only publishes a configure on this node's
            // per-node /config topic when it considers us adopted,
            // and the operator may have adopted without configuring
            // any peripherals yet. Honor the adoption either way.
            Serial.printf("Configure received with no pins to apply (empty/no-op)\n");
        }
        if (g_node.state != NODE_STATE_ACTIVE) {
            // Transition regardless of apply outcome — receipt of the
            // configure is itself the server's "you're adopted" signal.
            // Without this, an adopt-without-peripherals leaves the
            // node permanently in UNADOPTED even though the server
            // has it in its adopted list, and the dashboard's two
            // views of adoption state stay forever out of sync.
            node_set_state(NODE_STATE_ACTIVE);
            led_set_state(NODE_STATE_ACTIVE);
            Serial.printf("State: -> ACTIVE (adopted)\n");
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

#ifndef SIMULATION
#include "teensy_ota.h"
#endif

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

    /* Parse the OTA image metadata. Size + CRC32 are required for the
     * in-app FlashTxx download path; if either is missing we fall back
     * to the legacy bare-restart so old server flows still degrade
     * gracefully (the chip won't actually re-flash, same as before). */
    uint32_t img_size = 0, img_crc = 0;
    const char* p;
    if ((p = strstr(json, "\"size\"")) != NULL) {
        p = strchr(p, ':');
        if (p) { p++; while (*p == ' ') p++; img_size = (uint32_t)strtoul(p, NULL, 10); }
    }
    if ((p = strstr(json, "\"crc32\"")) != NULL) {
        p = strchr(p, ':');
        if (p) {
            p++;
            while (*p == ' ' || *p == '"') p++;
            int base = (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) ? 16 : 10;
            img_crc = (uint32_t)strtoul(p, NULL, base);
        }
    }

#ifdef SIMULATION
    Serial.printf("Simulation: halt — node manager restarts with new ELF\n====================================\n\n");
    delay(500);
    while (1) { yield(); }
#else
    if (img_size > 0 && img_crc != 0) {
        Serial.printf("Performing in-app OTA (size=%lu crc=0x%08lx)\n",
                       (unsigned long)img_size, (unsigned long)img_crc);
        saint_ota_result_t r = saint_ota_perform(img_size, img_crc);
        Serial.printf("OTA returned %d — staying on existing firmware\n", (int)r);
        return;
    }
    Serial.printf("Update message lacks size/crc32 — bare restart (no OTA)\n");
    Serial.printf("====================================\n\n");
    delay(500);
    SCB_AIRCR = 0x05FA0004;
    while (1) { }
#endif
}

/* Dispatch action JSON from either /control (BEST_EFFORT, streaming
 * writes) or /command (RELIABLE, one-shots). Same body for both topics;
 * the split exists for QoS only — see firmware/rp2040/src/main.c and
 * server_node.py for the rationale. */
static void dispatch_action_buffer(const char* data, size_t size)
{
    if (strstr(data, "\"action\":\"firmware_update\"") ||
        strstr(data, "\"action\": \"firmware_update\"")) {
        handle_firmware_update(data, size);
        return;
    }

    if (strstr(data, "\"action\":\"factory_reset\"") ||
        strstr(data, "\"action\": \"factory_reset\"")) {
        flash_storage_erase();
        pin_config_reset();
        node_set_state(NODE_STATE_UNADOPTED);
        led_set_state(NODE_STATE_UNADOPTED);
        Serial.printf("Factory reset complete\n");
        return;
    }

    if (strstr(data, "\"action\":\"restart\"") ||
        strstr(data, "\"action\": \"restart\"")) {
        Serial.printf("Restart requested\n");
        delay(500);
#ifdef SIMULATION
        while (1) { yield(); }
#else
        SCB_AIRCR = 0x05FA0004;
        while (1) { }
#endif
    }

    if (strstr(data, "\"action\":\"identify\"") ||
        strstr(data, "\"action\": \"identify\"")) {
        led_identify(5);
        return;
    }

    if (strstr(data, "\"action\":\"estop\"") ||
        strstr(data, "\"action\": \"estop\"")) {
        pin_control_estop();
        return;
    }

    pin_control_apply_json(data, size);
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
    dispatch_action_buffer(msg->data.data, msg->data.size);
}

static void command_subscription_callback(const void* msgin)
{
    const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;
    if (!msg || !msg->data.data) return;
    if (msg->data.size >= sizeof(command_buffer)) return;
    command_buffer[msg->data.size] = '\0';
    Serial.printf("Command received: %.*s\n",
                   (int)(msg->data.size < 80 ? msg->data.size : 80),
                   msg->data.data);
    dispatch_action_buffer(msg->data.data, msg->data.size);
}

// ============================================================================
// Publishing
// ============================================================================

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

    if (g_node.state != NODE_STATE_UNADOPTED && g_node.state != NODE_STATE_ACTIVE) {
        return;
    }

    float cpu_temp = hardware_get_cpu_temp();

    // chip_family lets the server pick the right board YAML to derive
    // pin layout from. Teensy 4.1 uses the NXP iMXRT1062 — we report
    // "teensy41" as the family so the server matches
    // server/config/boards/teensy41/.
    const char* chip_family = "teensy41";

    int ann_len = snprintf(announcement_buffer, sizeof(announcement_buffer),
        "{"
        "\"node_id\":\"%s\","
        "\"chip_family\":\"%s\","
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
        chip_family,
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
    //Serial.printf("Announcement: %s, len=%d\n", announcement_buffer, ann_len);

    announcement_msg.data.data = announcement_buffer;
    announcement_msg.data.size = strlen(announcement_buffer);
    announcement_msg.data.capacity = sizeof(announcement_buffer);

    rcl_ret_t ret = rcl_publish(&announcement_pub, &announcement_msg, NULL);
    if (ret == RCL_RET_OK) {
        mark_agent_communication();
        // The server creates the per-node /log subscription lazily on
        // first announce, so wait for ≥2 announces before flushing the
        // boot-log buffer — that gives the subscriber time to come up.
        if (g_announce_count < 1000) g_announce_count++;
        if (g_announce_count >= 2) {
            saint_log_drain_boot_queue();
        }
    } else {
        // Surface publish failures instead of swallowing them. Without
        // this, an XRCE-DDS session that's lost its publisher (e.g.
        // because the agent's bookkeeping got out of sync after a
        // disconnect/reconnect cycle) looks identical to a healthy
        // node from the serial console — same boot banner, same
        // periodic status line, just no announces on the wire.
        // Rate-limit to one log line per second-ish so a sustained
        // failure doesn't drown the serial.
        static uint32_t last_fail_log_ms = 0;
        uint32_t now_ms = millis();
        if (now_ms - last_fail_log_ms >= 1000) {
            Serial.printf("Announce publish failed: rcl_ret=%d\n", (int)ret);
            last_fail_log_ms = now_ms;
        }
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

    // Config subscriber
    char topic[64];
    snprintf(topic, sizeof(topic), "/saint/nodes/%s/config", g_node.node_id);
    for (char* p = topic; *p; p++) { if (*p == '-' || *p == ':') *p = '_'; }
    ret = rclc_subscription_init_default(&config_sub, &ros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic);
    if (ret != RCL_RET_OK) return false;
    config_msg.data.data = config_buffer;
    config_msg.data.size = 0;
    config_msg.data.capacity = sizeof(config_buffer);

    // Control subscriber. BEST_EFFORT to match the server's streaming-
    // control publisher — see firmware/rp2040/src/main.c for the
    // rationale (deadstick must not queue behind unacknowledged
    // reliable retransmissions).
    snprintf(topic, sizeof(topic), "/saint/nodes/%s/control", g_node.node_id);
    for (char* p = topic; *p; p++) { if (*p == '-' || *p == ':') *p = '_'; }
    ret = rclc_subscription_init_best_effort(&control_sub, &ros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic);
    if (ret != RCL_RET_OK) return false;
    control_msg.data.data = control_buffer;
    control_msg.data.size = 0;
    control_msg.data.capacity = sizeof(control_buffer);

    // Command subscriber — RELIABLE one-shots (firmware_update, estop,
    // factory_reset, identify, …). See firmware/rp2040/src/main.c and
    // server_node.py CONTROL_QOS/COMMAND_QOS for the rationale.
    snprintf(topic, sizeof(topic), "/saint/nodes/%s/command", g_node.node_id);
    for (char* p = topic; *p; p++) { if (*p == '-' || *p == ':') *p = '_'; }
    ret = rclc_subscription_init_default(&command_sub, &ros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic);
    if (ret != RCL_RET_OK) return false;
    command_msg.data.data = command_buffer;
    command_msg.data.size = 0;
    command_msg.data.capacity = sizeof(command_buffer);

    // State publisher
    snprintf(topic, sizeof(topic), "/saint/nodes/%s/state", g_node.node_id);
    for (char* p = topic; *p; p++) { if (*p == '-' || *p == ':') *p = '_'; }
    ret = rclc_publisher_init_default(&state_pub, &ros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic);
    if (ret != RCL_RET_OK) return false;

    // Log publisher — per-node /log topic. Best-effort; shared/src/saint_log.c
    // owns the publish path, this main just provides the rcl handle.
    snprintf(topic, sizeof(topic), "/saint/nodes/%s/log", g_node.node_id);
    for (char* p = topic; *p; p++) { if (*p == '-' || *p == ':') *p = '_'; }
    ret = rclc_publisher_init_default(&log_pub, &ros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic);
    if (ret != RCL_RET_OK) return false;
    saint_log_set_ros_ready(true);

    // Timers
    ret = rclc_timer_init_default(&announce_timer, &support,
        RCL_MS_TO_NS(ANNOUNCE_INTERVAL_MS), announce_timer_callback);
    if (ret != RCL_RET_OK) return false;

    ret = rclc_timer_init_default(&state_timer, &support,
        RCL_MS_TO_NS(STATE_PUBLISH_INTERVAL_MS), state_timer_callback);
    if (ret != RCL_RET_OK) return false;

    // Executor (2 timers + 3 subscriptions)
    ret = rclc_executor_init(&executor, &support.context, 5, &allocator);
    if (ret != RCL_RET_OK) return false;

    rclc_executor_add_timer(&executor, &announce_timer);
    rclc_executor_add_timer(&executor, &state_timer);
    rclc_executor_add_subscription(&executor, &command_sub, &command_msg,
        command_subscription_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &config_sub, &config_msg,
        config_subscription_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &control_sub, &control_msg,
        control_subscription_callback, ON_NEW_DATA);

    Serial.printf("micro-ROS initialized successfully\n");
    return true;
}

static void cleanup_micro_ros(void)
{
    rcl_subscription_fini(&command_sub, &ros_node);
    rcl_subscription_fini(&control_sub, &ros_node);
    rcl_subscription_fini(&config_sub, &ros_node);
    saint_log_set_ros_ready(false);
    rcl_publisher_fini(&log_pub, &ros_node);
    rcl_publisher_fini(&state_pub, &ros_node);
    rcl_publisher_fini(&announcement_pub, &ros_node);
    rcl_timer_fini(&state_timer);
    rcl_timer_fini(&announce_timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&ros_node);
    rclc_support_fini(&support);
}

// ============================================================================
// saint_log per-platform hooks (shared/src/saint_log.c calls these)
// ============================================================================

extern "C" void saint_log_emit_local(const char* level, const char* text)
{
    Serial.printf("[%s] %s\n", level, text);
}

extern "C" bool saint_log_emit_ros(const char* json, size_t len)
{
    if (len >= sizeof(log_buffer)) return false;
    memcpy(log_buffer, json, len);
    log_msg.data.data     = log_buffer;
    log_msg.data.size     = len;
    log_msg.data.capacity = sizeof(log_buffer);
    return rcl_publish(&log_pub, &log_msg, NULL) == RCL_RET_OK;
}

extern "C" uint32_t saint_log_uptime_ms(void)
{
    return (uint32_t)millis();
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

    // Initialize pin control
    pin_control_init();

    // Register peripheral drivers BEFORE loading config from flash.
    // pin_config_load() walks the registered-drivers list and calls
    // each driver's load_config(&storage) so it can read its stored
    // pins/enabled flag. If we load first and register after, that
    // loop runs over zero drivers and the per-driver flash data is
    // silently discarded — leaving drivers in their default state
    // even after a reboot of a previously-configured node. Matches
    // the order used in firmware/rp2040/src/main.c.
    peripheral_register(maestro_get_peripheral_driver());
    peripheral_register(syren_get_peripheral_driver());
    peripheral_register(fas100_get_peripheral_driver());
    peripheral_register(roboclaw_get_peripheral_driver());
    peripheral_register(pathfinder_bms_get_peripheral_driver());

    // Load saved pin configuration (also fans out to each driver's
    // load_config callback).
    if (pin_config_load()) {
        Serial.printf("Loaded pin configuration from flash\n");
    }

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

    /* Throttle the loop to ~100 Hz, but with the core actually
     * sleeping between SysTick wakes instead of yield-polling at full
     * 600 MHz. Each __WFI halts the CPU until the next interrupt
     * (SysTick fires at 1 kHz, ENET fires on packet arrival), so a
     * burst of inbound traffic wakes us immediately. This is the heat
     * fix: idle iMXRT1062 at 600 MHz with WFI runs ~30 °C cooler than
     * with delay(10), because delay() is itself a yield-busy-loop. */
    uint32_t loop_deadline = now + 10;
    while ((int32_t)(millis() - loop_deadline) < 0) {
        asm volatile ("wfi");
    }
}
