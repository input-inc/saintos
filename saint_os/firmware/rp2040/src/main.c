/**
 * SAINT.OS Node Firmware - Main Entry Point
 *
 * Runs on Adafruit Feather RP2040 with Ethernet FeatherWing.
 * Implements a micro-ROS node that communicates with the SAINT.OS server.
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

#include "saint_node.h"
#include "transport_w5500.h"
#include "version.h"

// SAINT.OS message types
#include <saint_os/msg/node_announcement.h>
#include <saint_os/srv/adopt_node.h>
#include <saint_os/srv/reset_node.h>

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

// Services
static rcl_service_t adopt_service;
static rcl_service_t reset_service;

// Service request/response buffers
static saint_os__srv__AdoptNode_Request adopt_request;
static saint_os__srv__AdoptNode_Response adopt_response;
static saint_os__srv__ResetNode_Request reset_request;
static saint_os__srv__ResetNode_Response reset_response;

// =============================================================================
// Service Callbacks
// =============================================================================

/**
 * Handle adopt node service request.
 */
static void adopt_service_callback(const void* request, void* response)
{
    const saint_os__srv__AdoptNode_Request* req =
        (const saint_os__srv__AdoptNode_Request*)request;
    saint_os__srv__AdoptNode_Response* res =
        (saint_os__srv__AdoptNode_Response*)response;

    printf("[ADOPT] Role: %s, Name: %s\n",
           req->role.data, req->display_name.data);

    // Parse role
    node_role_t role = NODE_ROLE_NONE;
    if (strcmp(req->role.data, "head") == 0) {
        role = NODE_ROLE_HEAD;
    } else if (strcmp(req->role.data, "arms_left") == 0) {
        role = NODE_ROLE_ARMS_LEFT;
    } else if (strcmp(req->role.data, "arms_right") == 0) {
        role = NODE_ROLE_ARMS_RIGHT;
    } else if (strcmp(req->role.data, "tracks") == 0) {
        role = NODE_ROLE_TRACKS;
    } else if (strcmp(req->role.data, "console") == 0) {
        role = NODE_ROLE_CONSOLE;
    }

    if (role != NODE_ROLE_NONE) {
        res->success = node_adopt(role, req->display_name.data);
        if (res->success) {
            snprintf(res->message.data, res->message.capacity,
                     "Adopted as %s", req->role.data);
        } else {
            snprintf(res->message.data, res->message.capacity,
                     "Adoption failed");
        }
    } else {
        res->success = false;
        snprintf(res->message.data, res->message.capacity,
                 "Unknown role: %s", req->role.data);
    }

    res->message.size = strlen(res->message.data);
}

/**
 * Handle reset node service request.
 */
static void reset_service_callback(const void* request, void* response)
{
    const saint_os__srv__ResetNode_Request* req =
        (const saint_os__srv__ResetNode_Request*)request;
    saint_os__srv__ResetNode_Response* res =
        (saint_os__srv__ResetNode_Response*)response;

    printf("[RESET] Factory reset: %s\n",
           req->factory_reset ? "yes" : "no");

    res->success = node_reset(req->factory_reset);

    if (res->success) {
        snprintf(res->message.data, res->message.capacity,
                 "Node reset successfully");
    } else {
        snprintf(res->message.data, res->message.capacity,
                 "Reset failed");
    }

    res->message.size = strlen(res->message.data);
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

    // Initialize support
    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
        printf("Failed to initialize support: %d\n", ret);
        return false;
    }

    // Create node with name based on node_id
    char node_name[64];
    snprintf(node_name, sizeof(node_name), "saint_node_%s", g_node.node_id);

    ret = rclc_node_init_default(&ros_node, node_name, "saint", &support);
    if (ret != RCL_RET_OK) {
        printf("Failed to create node: %d\n", ret);
        return false;
    }

    printf("Created ROS2 node: %s\n", node_name);

    // Create announcement publisher
    ret = rclc_publisher_init_default(
        &announcement_pub,
        &ros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(saint_os, msg, NodeAnnouncement),
        "/saint/nodes/unadopted"
    );
    if (ret != RCL_RET_OK) {
        printf("Failed to create announcement publisher: %d\n", ret);
        return false;
    }

    // Create adopt service
    char adopt_service_name[128];
    snprintf(adopt_service_name, sizeof(adopt_service_name),
             "/saint/node/%s/adopt", g_node.node_id);

    ret = rclc_service_init_default(
        &adopt_service,
        &ros_node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(saint_os, srv, AdoptNode),
        adopt_service_name
    );
    if (ret != RCL_RET_OK) {
        printf("Failed to create adopt service: %d\n", ret);
        return false;
    }

    // Create reset service
    char reset_service_name[128];
    snprintf(reset_service_name, sizeof(reset_service_name),
             "/saint/node/%s/reset", g_node.node_id);

    ret = rclc_service_init_default(
        &reset_service,
        &ros_node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(saint_os, srv, ResetNode),
        reset_service_name
    );
    if (ret != RCL_RET_OK) {
        printf("Failed to create reset service: %d\n", ret);
        return false;
    }

    // Initialize executor
    ret = rclc_executor_init(&executor, &support.context, 2, &allocator);
    if (ret != RCL_RET_OK) {
        printf("Failed to create executor: %d\n", ret);
        return false;
    }

    // Add services to executor
    ret = rclc_executor_add_service(
        &executor, &adopt_service,
        &adopt_request, &adopt_response,
        adopt_service_callback
    );
    if (ret != RCL_RET_OK) {
        printf("Failed to add adopt service: %d\n", ret);
        return false;
    }

    ret = rclc_executor_add_service(
        &executor, &reset_service,
        &reset_request, &reset_response,
        reset_service_callback
    );
    if (ret != RCL_RET_OK) {
        printf("Failed to add reset service: %d\n", ret);
        return false;
    }

    printf("micro-ROS initialized successfully\n");
    return true;
}

/**
 * Publish node announcement.
 */
static void publish_announcement(void)
{
    saint_os__msg__NodeAnnouncement msg;

    // Initialize message
    saint_os__msg__NodeAnnouncement__init(&msg);

    // Fill in node information
    snprintf(msg.node_id.data, msg.node_id.capacity, "%s", g_node.node_id);
    msg.node_id.size = strlen(msg.node_id.data);

    snprintf(msg.mac_address.data, msg.mac_address.capacity,
             "%02X:%02X:%02X:%02X:%02X:%02X",
             g_node.mac_address[0], g_node.mac_address[1],
             g_node.mac_address[2], g_node.mac_address[3],
             g_node.mac_address[4], g_node.mac_address[5]);
    msg.mac_address.size = strlen(msg.mac_address.data);

    snprintf(msg.hardware_model.data, msg.hardware_model.capacity,
             "%s", HARDWARE_MODEL);
    msg.hardware_model.size = strlen(msg.hardware_model.data);

    snprintf(msg.firmware_version.data, msg.firmware_version.capacity,
             "%s", FIRMWARE_VERSION_STRING);
    msg.firmware_version.size = strlen(msg.firmware_version.data);

    snprintf(msg.ip_address.data, msg.ip_address.capacity,
             "%d.%d.%d.%d",
             g_node.static_ip[0], g_node.static_ip[1],
             g_node.static_ip[2], g_node.static_ip[3]);
    msg.ip_address.size = strlen(msg.ip_address.data);

    msg.hardware_ram_mb = 0;  // RP2040 has 264KB, report as 0 MB
    msg.cpu_temp = hardware_get_cpu_temp();
    msg.cpu_usage = 0.0f;  // TODO: implement
    msg.memory_usage = 0.0f;  // TODO: implement
    msg.uptime_seconds = g_node.uptime_ms / 1000;

    // Publish
    rcl_ret_t ret = rcl_publish(&announcement_pub, &msg, NULL);
    if (ret != RCL_RET_OK) {
        printf("Failed to publish announcement: %d\n", ret);
    }

    // Cleanup
    saint_os__msg__NodeAnnouncement__fini(&msg);
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
    snprintf(g_node.node_id, sizeof(g_node.node_id), "rp2040-%s", unique_id);
    printf("Node ID: %s\n", g_node.node_id);

    // Initialize ethernet transport
    printf("Initializing ethernet...\n");
    led_set_state(NODE_STATE_CONNECTING);

    if (!transport_w5500_init()) {
        printf("ERROR: Ethernet initialization failed!\n");
        node_set_state(NODE_STATE_ERROR);
        led_set_state(NODE_STATE_ERROR);
        while (1) {
            led_update();
            sleep_ms(100);
        }
    }

    // Get MAC address from W5500
    transport_w5500_get_mac(g_node.mac_address);
    printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
           g_node.mac_address[0], g_node.mac_address[1],
           g_node.mac_address[2], g_node.mac_address[3],
           g_node.mac_address[4], g_node.mac_address[5]);

    // Connect to network
    if (!transport_w5500_connect()) {
        printf("ERROR: Network connection failed!\n");
        node_set_state(NODE_STATE_ERROR);
        led_set_state(NODE_STATE_ERROR);
        while (1) {
            led_update();
            sleep_ms(100);
        }
    }

    // Get IP address
    transport_w5500_get_ip(g_node.static_ip);
    printf("IP: %d.%d.%d.%d\n",
           g_node.static_ip[0], g_node.static_ip[1],
           g_node.static_ip[2], g_node.static_ip[3]);

    // Set micro-ROS transport
    rmw_uros_set_custom_transport(
        false,  // Not using serial/stream transport
        NULL,   // No transport args needed
        transport_w5500_open,
        transport_w5500_close,
        transport_w5500_write,
        transport_w5500_read
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
    uint32_t last_announce_time = 0;

    while (1) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // Update node state
        node_state_update();
        g_node.uptime_ms = now;

        // Update LED
        led_update();

        // Spin micro-ROS executor (process callbacks)
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        // Publish announcements when unadopted
        if (g_node.state == NODE_STATE_UNADOPTED) {
            if (now - last_announce_time >= ANNOUNCE_INTERVAL_MS) {
                publish_announcement();
                last_announce_time = now;
            }
        }

        // Small delay
        sleep_ms(10);
    }

    return 0;
}
