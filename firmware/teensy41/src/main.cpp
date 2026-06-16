/**
 * SAINT.OS Node Firmware - Main Entry Point (Teensy 4.1)
 *
 * Uses Arduino framework with micro-ROS for communication.
 */

#include <Arduino.h>
// platform.h's #define Serial Serial1 (under SIMULATION) must be in scope
// BEFORE any code in this TU references `Serial`, otherwise Serial.begin()
// here resolves to the USB CDC object — which Renode can't capture, and
// also means LPUART6's CTRL.TE never gets set so its TX FIFO never drains.
#include "platform.h"
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
#include "tic_driver.h"
#include "tmc2208_driver.h"
#include "watchdog.h"
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
// 1024 bytes (was 512). Matched to the RP2040 fix: the announcement
// JSON grew past 512 when tic + tmc2208 drivers were added, snprintf
// silently truncated the closing braces, server's json.loads dropped
// it. See firmware/rp2040/src/main.c for the longer rationale.
static char announcement_buffer[1024];

/* Sync-ACK signal published via /announce. Server's
 * _maybe_handle_announce_sync_ack edge-triggers on either field
 * increasing and flips the UI's Sync pill. /log was the original
 * carrier ("Config saved to flash" text-scan), but post-boot /log
 * frames drop on this platform once the writer's micro-XRCE-DDS
 * output stream saturates; /announce is the only writer that
 * demonstrably keeps publishing even when /log goes quiet. */
static volatile uint32_t g_last_config_save_ok_ms = 0;
static volatile uint32_t g_last_config_save_fail_ms = 0;

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

/* Post-init-hang diagnostics — see firmware/teensy41/docs/POST_INIT_HANG.md.
 *
 * The Teensy intermittently boots through `init_micro_ros()` cleanly,
 * prints `micro-ROS initialized successfully`, and then loop() stops
 * producing any output and no UDP traffic ever leaves the chip. From
 * the outside the chip stays pingable and the agent's DDS view of the
 * node looks fine, so there's no obvious failure to attribute the hang
 * to.
 *
 * These counters give us observable progress at three nested layers —
 * the main loop, the rclc executor, and the transport's RX poll —
 * surfaced in /announce so the operator can `ros2 topic echo` (or read
 * the dashboard) and see which one stopped advancing. Each is plain
 * uint32_t (no locks; we only read+publish from the announce timer
 * callback, which is on the same rclc executor as anything that
 * increments these, so concurrent access can't happen).
 *
 * Counter pairs are written as entries→exits so an arrested call
 * shows as (entries > exits) without needing a timestamp. Reading
 * `entries == exits` AND a freshly-incrementing `loop_iter` means
 * the firmware is alive and just not publishing — likely the agent
 * lost the publisher silently. Reading `loop_iter` frozen means the
 * loop itself wedged before reaching either the executor or the
 * status print. Reading executor entries advancing but exits not
 * keeping up points at the executor (or a callback) blocking.
 *
 * Volatile because the transport hooks are reached from inside
 * micro-XRCE-DDS callbacks the compiler can't see across; without
 * volatile, LTO can prove a non-reentrant write isn't observed and
 * fold the increments. */
static volatile uint32_t g_loop_iter             = 0;
static volatile uint32_t g_executor_spin_entries = 0;
static volatile uint32_t g_executor_spin_exits   = 0;
/* "Furthest stage loop() reached this iteration", incremented at every
 * known-safe checkpoint. The status print emits the last-seen value;
 * if loop_iter keeps advancing but g_loop_stage stays at, say, 3, the
 * wedge is between checkpoint 3 and 4. Acts like a coarse program
 * counter that survives an in-progress hang because we capture it
 * AFTER each stage completes. Stages:
 *   1 = after led_update
 *   2 = after peripheral_update_all
 *   3 = after check_agent_connection
 *   4 = after rclc_executor_spin_some
 *   5 = after saint_log_drain_pending  (end of normal iteration)
 * 0 = haven't reached stage 1 yet (probably wedged in led_update or
 * before, or this is the very first iteration). */
static volatile uint32_t g_loop_stage = 0;
/* Transport-layer counters are defined in the per-transport TU that
 * actually increments them (firmware/shared/src/transport_udp_bridge.cpp
 * for the sim path, firmware/teensy41/transport/transport_native_eth.cpp
 * for hardware). Defining them here would conflict at link time for
 * the sim build (both this file and transport_udp_bridge.cpp ship
 * the same firmware). extern "C" so the C++ name mangling doesn't
 * disagree across the TU boundary. */
extern "C" {
extern volatile uint32_t g_transport_read_entries;
extern volatile uint32_t g_transport_read_exits;
extern volatile uint32_t g_transport_write_entries;
extern volatile uint32_t g_transport_write_exits;
extern volatile uint32_t g_transport_read_data;     // poll calls that pulled bytes
} // extern "C"

/* Subscription callback fire counters — Maestro USB-host polling is
 * suspected of starving the executor; the announce diag reports these
 * so we can see whether config/control/command/state callbacks ever
 * fire at all. Each callback bumps its counter at the very top, BEFORE
 * any Serial.printf, so it can't be masked by a wedged USB CDC. */
static volatile uint32_t g_cb_cfg = 0;
static volatile uint32_t g_cb_ctl = 0;
static volatile uint32_t g_cb_cmd = 0;
static volatile uint32_t g_cb_state = 0;
/* Peripheral-update worst-case wall-clock in ms across all loop iters.
 * If myusb.Task() (USB host poll) is the wedge, this climbs high
 * (50–150 ms vs the loop's nominal 10 ms iter). */
static volatile uint32_t g_periph_update_max_ms = 0;

/* /state JSON for a fully-configured Teensy (19+ pins, including
 * 16 Maestro servo channels) is ~1200 bytes — 2.3× over the 512-byte
 * uxr UDP MTU. Every 10 Hz publish wedges in rmw retry logic for
 * tens of ms, starves rclc_executor_spin_some's transport_read, and
 * inbound /config /control /command frames pile up in NativeEthernet's
 * UDP RX buffer faster than the chip drains them — the OTA-failing
 * symptom we chased today.
 *
 * Temporary fix: drop to 1 Hz. Real fix is paged state publishes
 * (split per-pin or per-peripheral into separate sub-MTU frames) so
 * we can return to 10 Hz without poisoning the executor. */
#define STATE_PUBLISH_INTERVAL_MS 1000

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
    g_cb_cfg++;
    g_loop_stage = 30;
    const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;
    if (!msg || !msg->data.data) { g_loop_stage = 39; return; }

    if (msg->data.size >= sizeof(config_buffer)) {
        Serial.printf("Config message too large: %zu >= %zu, rejecting\n",
                       msg->data.size, sizeof(config_buffer));
        g_loop_stage = 39;
        return;
    }

    config_buffer[msg->data.size] = '\0';

    Serial.printf("Config received: %.*s\n",
                   (int)(msg->data.size < 100 ? msg->data.size : 100),
                   msg->data.data);
    // Mirror to the /log topic so the e2e harness (and dashboard Logs
    // tab) can see config-receipt over the wire — Serial.printf alone
    // only lands in the Renode-captured UART file. RP2040 does the same;
    // test_sync_recovery's Phase 2 substring-matches "Config received"
    // against the server-relayed /log stream.
    saint_log_publish("info", "Config received (%zu bytes), applying…",
                      msg->data.size);

    if (strstr(msg->data.data, "\"action\":\"configure\"") ||
        strstr(msg->data.data, "\"action\": \"configure\"")) {
        bool applied = pin_config_apply_json(msg->data.data, msg->data.size);
        if (applied) {
            Serial.printf("Pin configuration applied successfully\n");
            saint_log_publish("info", "Config applied OK");
            if (pin_config_save()) {
                Serial.printf("Pin configuration saved to flash\n");
                saint_log_publish("info", "Config saved to flash");
                /* /announce-borne sync-ACK — see the declaration of
                 * g_last_config_save_ok_ms (in main.cpp's announce
                 * builder) for why this lives in /announce instead of
                 * /log. */
                g_last_config_save_ok_ms = saint_log_uptime_ms();
            } else {
                saint_log_publish("error", "Config save to flash failed");
                g_last_config_save_fail_ms = saint_log_uptime_ms();
            }
        } else {
            // No pins / peripherals to apply (or apply failed cleanly).
            // That's still a valid "you are adopted" signal — the
            // server only publishes a configure on this node's
            // per-node /config topic when it considers us adopted,
            // and the operator may have adopted without configuring
            // any peripherals yet. Honor the adoption either way.
            //
            // We don't treat empty-pins as a fail: it's an intentional
            // empty configure. Mark it as a successful save so the
            // server's sync pill flips. (Skip the flash write — there
            // are no pins to persist.)
            Serial.printf("Configure received with no pins to apply (empty/no-op)\n");
            g_last_config_save_ok_ms = saint_log_uptime_ms();
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
    g_loop_stage = 38;
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
        // Reboot for a clean slate — per-driver module statics
        // (port_initialized, configured_baud, open Serial claims, etc.)
        // survive flash_storage_erase + pin_config_reset, and the
        // operator pressed Factory Reset to get a true reset. See
        // firmware/rp2040/src/main.c for the RP2040 mirror.
        saint_log_publish("warn",
            "Factory reset: flash erased, rebooting for clean slate");
        Serial.printf("Factory reset: rebooting in 500ms\n");
        delay(500);
#ifdef SIMULATION
        while (1) { yield(); }
#else
        SCB_AIRCR = 0x05FA0004;
        while (1) { }
#endif
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

    /* Server-controlled onboard-LED override. Matches the
     * set_neopixel wire shape the Feather uses so the dashboard can
     * target either board identically — the Teensy's single-color
     * LED just collapses (r,g,b) at brightness > 0 to ON, brightness
     * 0 (or all-black) to OFF. See led_set_override_color() in
     * src/led_status.cpp.
     *
     * Wire format:
     *   {"action":"set_neopixel","r":255,"g":0,"b":0,"brightness":128}
     *   {"action":"set_neopixel","clear":true}     // resume state LED
     */
    if (strstr(data, "\"action\":\"set_neopixel\"") ||
        strstr(data, "\"action\": \"set_neopixel\"")) {
        if (strstr(data, "\"clear\":true") ||
            strstr(data, "\"clear\": true")) {
            led_clear_override();
            Serial.printf("LED: override cleared — resuming state-driven LED\n");
            return;
        }
        int r = 0, g = 0, b = 0, brightness = 255;
        const char* p;
        if ((p = strstr(data, "\"r\""))) {
            p = strchr(p, ':');
            if (p) { p++; while (*p == ' ') p++; r = atoi(p); }
        }
        if ((p = strstr(data, "\"g\""))) {
            p = strchr(p, ':');
            if (p) { p++; while (*p == ' ') p++; g = atoi(p); }
        }
        if ((p = strstr(data, "\"b\""))) {
            p = strchr(p, ':');
            if (p) { p++; while (*p == ' ') p++; b = atoi(p); }
        }
        if ((p = strstr(data, "\"brightness\""))) {
            p = strchr(p, ':');
            if (p) { p++; while (*p == ' ') p++; brightness = atoi(p); }
        }
        if (r < 0) r = 0; if (r > 255) r = 255;
        if (g < 0) g = 0; if (g > 255) g = 255;
        if (b < 0) b = 0; if (b > 255) b = 255;
        if (brightness < 0) brightness = 0; if (brightness > 255) brightness = 255;
        led_set_override_color((uint8_t)r, (uint8_t)g, (uint8_t)b,
                                (uint8_t)brightness);
        Serial.printf("LED: override set RGB=(%d,%d,%d) brightness=%d "
                       "(collapsed to %s)\n",
                       r, g, b, brightness,
                       ((brightness > 0) && ((r | g | b) != 0)) ? "ON" : "OFF");
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
    g_cb_ctl++;
    g_loop_stage = 40;
    const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;
    if (!msg || !msg->data.data) { g_loop_stage = 49; return; }
    if (msg->data.size >= sizeof(control_buffer)) { g_loop_stage = 49; return; }
    control_buffer[msg->data.size] = '\0';
    Serial.printf("Control received: %.*s\n",
                   (int)(msg->data.size < 80 ? msg->data.size : 80),
                   msg->data.data);
    dispatch_action_buffer(msg->data.data, msg->data.size);
    g_loop_stage = 48;
}

static void command_subscription_callback(const void* msgin)
{
    g_cb_cmd++;
    g_loop_stage = 50;
    const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;
    if (!msg || !msg->data.data) { g_loop_stage = 59; return; }
    if (msg->data.size >= sizeof(command_buffer)) { g_loop_stage = 59; return; }
    command_buffer[msg->data.size] = '\0';
    Serial.printf("Command received: %.*s\n",
                   (int)(msg->data.size < 80 ? msg->data.size : 80),
                   msg->data.data);
    dispatch_action_buffer(msg->data.data, msg->data.size);
    g_loop_stage = 58;
}

// ============================================================================
// Publishing
// ============================================================================

static volatile uint32_t g_state_publish_fail = 0;

static void publish_state(void)
{
    pin_control_update_state();

    int len = pin_control_state_to_json(
        state_buffer, sizeof(state_buffer), g_node.node_id);
    if (len < 0) return;

    state_msg.data.data = state_buffer;
    state_msg.data.size = (size_t)len;
    state_msg.data.capacity = sizeof(state_buffer);

    static bool state_size_logged = false;
    if (!state_size_logged) {
        Serial.printf("State JSON size: %d bytes (uxr MTU=512)\n", len);
        state_size_logged = true;
    }

    rcl_ret_t ret = rcl_publish(&state_pub, &state_msg, NULL);
    if (ret != RCL_RET_OK) g_state_publish_fail++;
}

static void state_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer && g_node.state == NODE_STATE_ACTIVE) {
        g_cb_state++;
        g_loop_stage = 20;
        publish_state();
        g_loop_stage = 21;
    }
}

static void announce_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (!timer) return;

    if (g_node.state != NODE_STATE_UNADOPTED && g_node.state != NODE_STATE_ACTIVE) {
        return;
    }

    /* Sub-stage markers — 10-19 = announce callback sub-steps. See the
     * g_loop_stage block comment for the stage-ID space. The diag in
     * /announce captures whatever stage was set BEFORE the snprintf, so
     * the captured S=N tells us the announce-callback substage at the
     * time of the most recent successful publish. Useful when bisecting
     * future executor hangs. */
    g_loop_stage = 10;
    float cpu_temp = hardware_get_cpu_temp();
    g_loop_stage = 11;

    // chip_family lets the server pick the right board YAML to derive
    // pin layout from. Teensy 4.1 uses the NXP iMXRT1062 — we report
    // "teensy41" as the family so the server matches
    // server/config/boards/teensy41/.
    const char* chip_family = "teensy41";

    /* /announce must fit in the XRCE-DDS UDP MTU (UXR_CONFIG_UDP_TRANSPORT_MTU,
     * 512 bytes in the prebuilt libmicroros). An over-MTU payload makes
     * rcl_publish return RCL_RET_ERROR every call AND jams the output
     * stream, which then starves /log too — the post-init-hang symptom.
     * The "d" field is a compact diag string (loop iter, stage, executor
     * entries/exits, transport tx/rx entries/exits) so the last announce
     * before each WDOG reset pinpoints which layer wedged — /log only
     * delivers the first entry per boot reliably, /announce is the
     * surviving channel. Keep "d" short, drop unused fields if it grows. */
    int ann_len = snprintf(announcement_buffer, sizeof(announcement_buffer),
        "{"
        "\"node_id\":\"%s\","
        "\"chip_family\":\"%s\","
        "\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
        "\"ip\":\"%d.%d.%d.%d\","
        "\"hw\":\"%s\","
        "\"fw\":\"%s\","
        "\"state\":\"%s\","
        "\"uptime\":%lu,"
        "\"last_config_save_ok_ms\":%lu,"
        "\"d\":\"L=%lu S=%lu E=%lu/%lu Tw=%lu/%lu Tr=%lu/%lu Td=%lu cb=%lu/%lu/%lu/%lu pdt=%lu\","
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
        node_state_to_string(g_node.state),
        g_node.uptime_ms / 1000,
        (unsigned long)g_last_config_save_ok_ms,
        (unsigned long)g_loop_iter,
        (unsigned long)g_loop_stage,
        (unsigned long)g_executor_spin_entries,
        (unsigned long)g_executor_spin_exits,
        (unsigned long)g_transport_write_entries,
        (unsigned long)g_transport_write_exits,
        (unsigned long)g_transport_read_entries,
        (unsigned long)g_transport_read_exits,
        (unsigned long)g_transport_read_data,
        (unsigned long)g_cb_cfg,
        (unsigned long)g_cb_ctl,
        (unsigned long)g_cb_cmd,
        (unsigned long)g_cb_state,
        (unsigned long)g_periph_update_max_ms
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

    static bool size_logged = false;
    if (!size_logged) {
        Serial.printf("Announce JSON size: %u bytes (uxr MTU=512)\n",
                      (unsigned)announcement_msg.data.size);
        size_logged = true;
    }
    g_loop_stage = 12;
    rcl_ret_t ret = rcl_publish(&announcement_pub, &announcement_msg, NULL);
    g_loop_stage = 13;
    if (ret == RCL_RET_OK) {
        mark_agent_communication();
        // The server creates the per-node /log subscription lazily on
        // first announce, so wait for ≥2 announces before flushing the
        // boot-log buffer — that gives the subscriber time to come up.
        if (g_announce_count < 1000) g_announce_count++;
        // saint_log_drain_pending is intentionally NOT called here. It
        // used to be (as saint_log_drain_boot_queue) but that put
        // rcl_publish calls inside the executor's timer-callback
        // dispatch — the exact context that produced the empty-/log-
        // frame regression. The main loop now drains; this callback
        // just bumps g_announce_count so the loop knows when it's
        // safe to start draining.
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
    g_loop_stage = 14;
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
    /* BEST_EFFORT — see /log publisher's comment below. The Teensy's
     * micro-XRCE-DDS output stream wedges under RELIABLE bursts and
     * /announce stops reaching the server within ~15 s of boot. */
    ret = rclc_publisher_init_best_effort(&announcement_pub, &ros_node,
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
    /* BEST_EFFORT — telemetry at 10 Hz, fresh-wins. Same stream-wedge
     * concern as /announce; freshest reading matters more than every
     * sample arriving. */
    ret = rclc_publisher_init_best_effort(&state_pub, &ros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic);
    if (ret != RCL_RET_OK) return false;

    // Log publisher — per-node /log topic. Genuinely BEST_EFFORT,
    // matching the comment. RELIABLE here used to wedge the
    // micro-XRCE-DDS output stream on the first saint_log burst
    // (RMW_UXRCE_STREAM_HISTORY_OUTPUT=4) — the writer started waiting
    // indefinitely for ACKs that never made it back inside the same
    // callback dispatch, and post-boot /log frames stopped reaching
    // the server. On the Teensy that wedge also took /announce down
    // (the agent saw the first announce, then silence for >15 s, then
    // a fresh discovery → endless reconnect loop). Sync-ACK rides
    // /announce now (see g_last_config_save_ok_ms below), so dropping
    // /log to BEST_EFFORT is safe.
    snprintf(topic, sizeof(topic), "/saint/nodes/%s/log", g_node.node_id);
    for (char* p = topic; *p; p++) { if (*p == '-' || *p == ':') *p = '_'; }
    ret = rclc_publisher_init_best_effort(&log_pub, &ros_node,
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

    // Executor — actual handle count is 2 timers + 3 subscriptions = 5.
    // Provisioned with 8 instead to dodge a suspected rclc off-by-one
    // when handle count exactly equals RMW_UXRCE_MAX_SUBSCRIPTIONS (=5)
    // on micro-ROS. The same provisioning is used on RP2040; without
    // it the Teensy's publishers create their DDS endpoints fine but
    // every publish silently fails.
    ret = rclc_executor_init(&executor, &support.context, 8, &allocator);
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
    /* std_msgs/String's CDR serializer reads `data` as a null-terminated
     * C string for length computation in addition to honoring data.size;
     * if leftover bytes from a previous, longer publish sit past `len`
     * with no intervening null, the serialized payload silently truncates
     * or scrambles at the boundary. Zero the tail so every publish
     * starts from a clean buffer. */
    log_buffer[len] = '\0';
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

    /* Previously gated agent liveness on a periodic
     * rmw_uros_ping_agent (true round-trip) to catch the
     * NativeEthernet-UDP-returns-OK-after-server-restart case. The
     * ping IS the right answer architecturally, but in this build of
     * micro_ros_platformio the ping path hung the main loop after
     * post-reconnect init — no `[N] state:` status print, no
     * announces, while the agent's session looked alive. Until we
     * root-cause that, fall back to the last_successful_comm
     * timeout heuristic: dist installs will reconnect within the
     * usual 15 s instead of the ~2 s the ping enabled. */
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

/* Diagnostic helper for the post-init-hang investigation. Print a
 * stage marker with Serial.flush() so the line is on the wire BEFORE
 * the next call runs (USB CDC otherwise batches lines and the host
 * loses the trailing one when the chip wedges). Also drive the
 * onboard LED to a unique pattern per stage so an operator with no
 * USB CDC visibility can still tell visually how far setup() got:
 * solid = stage 0..3, slow blink (250 ms) for the rest. Watchdog is
 * fed at every stage to keep this debug path from itself causing a
 * spurious WDOG reset (setup() does NOT call loop()'s feeder). */
static void diag_stage(uint8_t stage, const char* label)
{
    /* Used to Serial.printf("[setup-stage N] %s") + flip the LED for
     * post-init-hang bisection (resolved in commit 3ceaaa9). Stripped
     * down to a watchdog feed so setup()'s long-running DHCP retry +
     * server discovery still get a tick between stages. Re-enable
     * prints behind a build flag if you need to bisect a new setup()
     * regression. */
    (void)stage;
    (void)label;
    watchdog_feed();
}

void setup()
{
    // Initialize serial
    Serial.begin(115200);
    delay(100);

    /* Drive the onboard LED immediately so an operator with no USB
     * CDC visibility can see that setup() at least started — LED off
     * after >1 s means we wedged before this line (e.g. Serial.begin
     * or the USB CDC init underneath it). diag_stage() flips the pin
     * per-stage so the final LED state pinpoints how far we got. */
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Arm WDOG1 (hardware build only — see watchdog.cpp). Done right
    // after Serial.begin so the "previous reset was watchdog" line on
    // line 1 of the boot banner makes recovery cycles obvious, and
    // before any of the long-running init paths that could themselves
    // wedge (transport_native_eth_connect's DHCP loop,
    // init_micro_ros). 30 s default timeout — plenty for normal init,
    // tight enough that the operator doesn't think the node is dead.
    watchdog_init();
    diag_stage(0, "watchdog_init done");

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
    diag_stage(1, "banner printed");

    // Initialize node state
    node_state_init();
    diag_stage(2, "node_state_init done");

    // Initialize pin configuration
    pin_config_init();
    diag_stage(3, "pin_config_init done");

    // Initialize pin control
    pin_control_init();
    diag_stage(4, "pin_control_init done");

    // Register peripheral drivers BEFORE loading config from flash.
    // pin_config_load() walks the registered-drivers list and calls
    // each driver's load_config(&storage) so it can read its stored
    // pins/enabled flag. If we load first and register after, that
    // loop runs over zero drivers and the per-driver flash data is
    // silently discarded — leaving drivers in their default state
    // even after a reboot of a previously-configured node. Matches
    // the order used in firmware/rp2040/src/main.c.
    peripheral_register(maestro_get_peripheral_driver());
    diag_stage(5, "maestro registered");
    peripheral_register(syren_get_peripheral_driver());
    diag_stage(6, "syren registered");
    peripheral_register(fas100_get_peripheral_driver());
    diag_stage(7, "fas100 registered");
    peripheral_register(roboclaw_get_peripheral_driver());
    diag_stage(8, "roboclaw registered");
    peripheral_register(pathfinder_bms_get_peripheral_driver());
    diag_stage(9, "pathfinder_bms registered");
    peripheral_register(tic_get_peripheral_driver());
    diag_stage(10, "tic registered");
    peripheral_register(tmc2208_get_peripheral_driver());
    diag_stage(11, "tmc2208 registered");

    // Load saved pin configuration (also fans out to each driver's
    // load_config callback).
    if (pin_config_load()) {
        Serial.printf("Loaded pin configuration from flash\n");
    }
    diag_stage(12, "pin_config_load done");

    peripheral_init_all();
    diag_stage(13, "peripheral_init_all done");

    // Initialize hardware
    hardware_init();
    diag_stage(14, "hardware_init done");

    // Initialize LED
    led_init();
    led_set_state(NODE_STATE_BOOT);
    diag_stage(15, "led_init done");

    // Get unique ID
    char unique_id[32];
    hardware_get_unique_id(unique_id, sizeof(unique_id));
    snprintf(g_node.node_id, sizeof(g_node.node_id), "teensy41_%s", unique_id);
    Serial.printf("Node ID: %s\n", g_node.node_id);
    diag_stage(16, "node_id resolved");

    // Initialize flash storage
    flash_storage_init();
    diag_stage(17, "flash_storage_init done");

    // Initialize transport
    Serial.printf("Initializing transport: %s\n", TRANSPORT_NAME);
    led_set_state(NODE_STATE_CONNECTING);

    if (!TRANSPORT_INIT()) {
        Serial.printf("ERROR: Transport init failed!\n");
        node_set_state(NODE_STATE_ERROR);
        led_set_state(NODE_STATE_ERROR);
        return;
    }
    diag_stage(18, "TRANSPORT_INIT done");

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
    diag_stage(19, "TRANSPORT_CONNECT done (DHCP bound)");

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
    diag_stage(20, "discover_server done");

    TRANSPORT_SET_AGENT(g_node.server_ip, g_node.server_port);

    rmw_uros_set_custom_transport(
        TRANSPORT_FRAMED, NULL,
        TRANSPORT_OPEN, TRANSPORT_CLOSE,
        TRANSPORT_WRITE, TRANSPORT_READ
    );
    diag_stage(21, "transport hooks set, calling init_micro_ros");

    if (!init_micro_ros()) {
        Serial.printf("ERROR: micro-ROS init failed!\n");
        node_set_state(NODE_STATE_ERROR);
        led_set_state(NODE_STATE_ERROR);
        return;
    }
    diag_stage(22, "init_micro_ros done");

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
    diag_stage(23, "setup() complete, entering loop()");

    last_successful_comm = millis();
    agent_connected = true;
}

static uint32_t last_status_print = 0;

void loop()
{
    /* Heartbeat for the post-init-hang investigation. Bumped before any
     * other work so a hang inside led_update / peripheral_update_all /
     * the executor / transport leaves loop_iter pinned at the value it
     * had when the offending call started — diffable across two
     * announces. See g_loop_iter declaration. */
    g_loop_iter++;
    g_loop_stage = 0;

    // Feed WDOG1 at the very top, every iteration. A wedge anywhere
    // after this point will stop the feeds and WDOG1's timeout
    // (default 30 s) auto-resets the chip — no physical power cycle
    // needed. See firmware/teensy41/docs/POST_INIT_HANG.md and
    // firmware/teensy41/src/watchdog.cpp for the rationale.
    watchdog_feed();

    uint32_t now = millis();

    // Update state
    node_state_update();
    g_node.uptime_ms = now;

    // Update LED
    led_update();
    g_loop_stage = 1;

    // Poll peripheral drivers (Maestro USB host, SyRen, etc.). Time
    // the call — Maestro's myusb.Task() is the suspect for slow loop
    // iterations starving the executor's transport_read poll.
    uint32_t periph_t0 = millis();
    peripheral_update_all();
    uint32_t periph_dt = millis() - periph_t0;
    if (periph_dt > g_periph_update_max_ms) g_periph_update_max_ms = periph_dt;
    g_loop_stage = 2;

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
    g_loop_stage = 3;

    // Spin executor — bracketed by entry/exit counters so a hang
    // *inside* the executor (a callback that never returns, or
    // transport read that blocks past its timeout) shows up as
    // exec_in > exec_out in the next announce. See g_loop_iter.
    g_executor_spin_entries++;
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    g_executor_spin_exits++;
    g_loop_stage = 4;

    // Drain queued /log lines, one per iteration. Pacer inside the
    // drain enforces SAINT_LOG_MIN_PUBLISH_INTERVAL_MS between
    // publishes so back-to-back enqueues from a subscription
    // callback (config_subscription_callback's 4-line burst) trickle
    // out at the right cadence instead of overrunning the agent's
    // Cyclone DDS relay. Gated on ≥2 announces — the server creates
    // per-node /log subscribers lazily on first /announce, so
    // draining sooner just leaks bytes into the void.
    if (g_announce_count >= 2) {
        saint_log_drain_pending();
    }
    g_loop_stage = 5;

    /* Periodic diag prints (Serial.printf "[N] state ..." + saint_log_publish
     * "diag ...") used to fire every 2 s while the post-init-hang was
     * being chased. Now resolved (see commit 3ceaaa9). The compact "d"
     * field in /announce still carries the counter snapshot on every
     * announce, so any future wedge investigation has the same data
     * without spamming /log every two seconds. Re-enable behind a
     * build flag if you need higher-cadence sampling. */
    (void)last_status_print;

    /* Tried __WFI here for idle heat reduction, claiming the ENET
     * interrupt would wake us on incoming packets. That assumption is
     * false on Teensy 4.1's NativeEthernet — FNET is polled, not
     * interrupt-driven, so __WFI gated rclc_executor_spin_some() at
     * 1 ms (SysTick-only wake). That fed back into the XRCE-DDS
     * client's transport read: it couldn't keep up with the agent's
     * RELIABLE ACK pacing, the output stream filled, and announce /
     * state publishers stopped landing on the wire end-to-end — the
     * node appeared online to the agent only briefly, then went
     * dark for the 15 s server timeout, repeatedly. delay(1) is a
     * yield-busy-loop on Teensy 4 — chip runs hotter when idle, but
     * the rest of the stack works. */
    uint32_t loop_deadline = now + 10;
    while ((int32_t)(millis() - loop_deadline) < 0) {
        delay(1);
    }
}
