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
#include <rmw/qos_profiles.h>   // rmw_qos_profile_sensor_data (control QoS base)
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
/* 4096 (was 2048). A full 24-channel Maestro peripheral serializes
 * to ~3500 bytes when every channel is emitted in full. Server-side
 * maestro_slim_channels_for_wire collapses default-equal channels
 * to `{}` and drops a typical payload to <1 KB, but an operator
 * who customizes every channel on a maxed-out Maestro still needs
 * room for the full ~3500 bytes. 4096 gives that with margin.
 *
 * Note that this buffer alone isn't enough — the XRCE-DDS UDP MTU
 * (UXR_CONFIG_UDP_TRANSPORT_MTU = 512) and RMW_UXRCE_MAX_HISTORY
 * (4) together cap the reassembled message at ~2 KB. Beyond that
 * the libmicroros build needs MAX_HISTORY bumped (rebuild required).
 * The slim-channels strategy keeps us safely below that ceiling. */
static char config_buffer[4096];

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
extern volatile uint32_t g_transport_last_rx_ms;    // millis() of last RX with data
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
/* Connection monitoring — mirror of RP2040's parameters/algorithm in
 * firmware/rp2040/src/main.c so the two platforms behave identically
 * under server outage. The intent is to lift this whole block into
 * shared code; for now the structure matches so a future refactor is
 * mechanical. The only platform-specific bit is the liveness signal:
 * RP2040 trusts udp.write success (W5500 NACK propagates), Teensy
 * cannot (NativeEthernet udp.endPacket always returns OK) so it
 * tracks last-RX-with-data instead. */
#define CONNECTION_CHECK_INTERVAL_MS 5000   // Check every 5 seconds
/* NOTE: Teensy uses 45 s vs RP2040's 15 s because our liveness signal
 * differs. RP2040 trusts TX success (W5500 propagates link failure),
 * so last_successful_comm updates every announce (1 Hz) and stays
 * fresh under normal operation. Teensy uses RX-with-data
 * (g_transport_last_rx_ms) because NativeEthernet's udp.endPacket
 * always returns OK — and in steady state we only see a few agent
 * frames per ~10 s, so a 15 s window false-tripped constantly. 45 s
 * still recovers from a real server reboot well under a minute. */
#define CONNECTION_TIMEOUT_MS        45000
#define MAX_RECONNECT_ATTEMPTS       10     // Max consecutive failures before error state
#define ERROR_RECOVERY_DELAY_MS      30000  // Stay in ERROR for 30s, then try again

static uint32_t last_successful_comm = 0;   // legacy — TX-based, not trustworthy on NativeEthernet
static uint32_t last_connection_check = 0;
static uint32_t reconnect_attempts = 0;
static bool agent_connected = false;
static uint32_t error_entered_at = 0;        // When we entered NODE_STATE_ERROR (0 = not in error)

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

    // Mirror to /log so the server-side activity feed shows this. The
    // Serial.printf above only reaches the USB CDC console (useful for
    // hands-on dev but invisible in the operator UI). Matches the
    // RP2040 firmware's logging cadence — see firmware/rp2040/src/main.c
    // handle_firmware_update.
    saint_log_publish("info",
        "OTA: firmware_update received (current=%s server=%s force=%s)",
        FIRMWARE_VERSION_FULL, new_version, force_update ? "YES" : "NO");

    if (!force_update) {
        uint32_t current_ts = FIRMWARE_BUILD_UNIX;
        uint32_t server_ts = extract_version_timestamp(new_version);
        if (server_ts == 0 || server_ts <= current_ts) {
            Serial.printf("Firmware is already up to date.\n====================================\n\n");
            saint_log_publish("info",
                "OTA: already up to date (current=%lu server=%lu) — aborting "
                "(use force to override)",
                (unsigned long)current_ts, (unsigned long)server_ts);
            return;
        }
        saint_log_publish("info",
            "OTA: server has newer build (current=%lu server=%lu), proceeding",
            (unsigned long)current_ts, (unsigned long)server_ts);
    } else {
        saint_log_publish("info", "OTA: force update — skipping version check");
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
    saint_log_publish("info",
        "OTA: simulation — halting; node_manager restarts with new ELF");
    delay(500);
    while (1) { yield(); }
#else
    if (img_size > 0 && img_crc != 0) {
        Serial.printf("Performing in-app OTA (size=%lu crc=0x%08lx)\n",
                       (unsigned long)img_size, (unsigned long)img_crc);
        saint_log_publish("info",
            "OTA: performing in-app FlashTxx update (size=%lu crc=0x%08lx) — "
            "chip will reboot on success; further status appears after next boot",
            (unsigned long)img_size, (unsigned long)img_crc);
        saint_ota_result_t r = saint_ota_perform(img_size, img_crc);
        Serial.printf("OTA returned %d — staying on existing firmware\n", (int)r);
        saint_log_publish("error",
            "OTA: FlashTxx returned %d — staying on existing firmware", (int)r);
        return;
    }
    Serial.printf("Update message lacks size/crc32 — bare restart (no OTA)\n");
    Serial.printf("====================================\n\n");
    saint_log_publish("warn",
        "OTA: control message lacks size/crc32 — bare restart (no OTA). "
        "Rebuild the server firmware package so it includes the raw .bin.");
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
        strstr(data, "\"action\": \"restart\"") ||
        strstr(data, "\"action\":\"reboot\"") ||
        strstr(data, "\"action\": \"reboot\"")) {
        Serial.printf("Restart requested\n");
        // Mirror to /log so the operator-side activity feed shows this.
        // Accept both 'restart' (what the server's send_restart_command
        // emits today) and 'reboot' (legacy name some flows still send)
        // so a future server-side rename can't silently break it.
        saint_log_publish("warn",
            "Restart requested — rebooting via SCB_AIRCR");
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
        saint_log_publish("info", "Identify requested — flashing onboard LED");
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
        // Clamp each channel to [0, 255]. Two statements per line
        // wrapped in braces so -Wmisleading-indentation stops
        // treating the trailing `if` as a continuation of the
        // leading one.
        if (r < 0) { r = 0; } if (r > 255) { r = 255; }
        if (g < 0) { g = 0; } if (g > 255) { g = 255; }
        if (b < 0) { b = 0; } if (b > 255) { b = 255; }
        if (brightness < 0) { brightness = 0; } if (brightness > 255) { brightness = 255; }
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
        saint_log_publish("warn", "Estop activated");
        pin_control_estop();
        return;
    }
    // NOTE: Teensy has no clear_estop action handler today because
    // firmware/teensy41/src/pin_control.cpp doesn't implement
    // pin_control_clear_estop. RP2040 does. This is a feature-parity
    // gap, not a logging gap — flagged here so the next person looking
    // at the dispatcher knows it's intentional. Tracked separately
    // from the logging-alignment work.

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
        (unsigned long)g_last_config_save_ok_ms
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
    //
    // KEEP_LAST depth 1 (NOT the depth-5 rmw_qos_profile_sensor_data
    // that rclc_subscription_init_best_effort would install). The
    // control stream — track motors AND Maestro servos, both routed
    // through this one subscription — must be strictly newest-wins: a
    // fast slider/joystick drag can otherwise queue up to 5 samples,
    // and the rclc executor consumes only ONE per loop iteration, so
    // the actuator chases stale setpoints one-at-a-time instead of
    // jumping to the freshest. That backlog is the "laggy / not-smooth"
    // motion, made worse because each command hops controller→agent→
    // node. Depth 1 drops every sample but the latest before the
    // executor ever sees it, matching the server's CONTROL_QOS
    // (BEST_EFFORT, depth=1). The final resting value is always the
    // newest sample, so nothing meaningful is lost.
    snprintf(topic, sizeof(topic), "/saint/nodes/%s/control", g_node.node_id);
    for (char* p = topic; *p; p++) { if (*p == '-' || *p == ':') *p = '_'; }
    rmw_qos_profile_t control_qos = rmw_qos_profile_sensor_data;  // BEST_EFFORT, KEEP_LAST, VOLATILE
    control_qos.depth = 1;
    ret = rclc_subscription_init(&control_sub, &ros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic, &control_qos);
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

    // Timers. rclc_timer_init_default was deprecated upstream in
    // favor of rclc_timer_init_default2(handle, support, period_ns,
    // callback, autostart). Passing `true` for autostart matches the
    // legacy default behaviour (timer fires on its first scheduled
    // tick) so this is a no-op rename for runtime behavior.
    ret = rclc_timer_init_default2(&announce_timer, &support,
        RCL_MS_TO_NS(ANNOUNCE_INTERVAL_MS), announce_timer_callback, true);
    if (ret != RCL_RET_OK) return false;

    ret = rclc_timer_init_default2(&state_timer, &support,
        RCL_MS_TO_NS(STATE_PUBLISH_INTERVAL_MS), state_timer_callback, true);
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
    /* Seed the RX-liveness timestamp so check_agent_connection's
     * timeout has a known reference even if the agent immediately
     * dies before sending us anything else. Session establish always
     * does a round trip, so a successful init implies we received
     * recently. */
    g_transport_last_rx_ms = millis();
    return true;
}

static void cleanup_micro_ros(void)
{
    /* rcl_*_fini are declared with warn_unused_result. In a teardown
     * path we genuinely don't have anywhere to surface a failure to —
     * we're about to recreate everything (cleanup is called from the
     * session-loss recovery path that immediately re-inits) so a fini
     * failure is informational at best. Cast to (void) so the
     * compiler stops nagging without us silently dropping the
     * declaration's intent in some future caller that DOES care. */
    (void)rcl_subscription_fini(&command_sub, &ros_node);
    (void)rcl_subscription_fini(&control_sub, &ros_node);
    (void)rcl_subscription_fini(&config_sub, &ros_node);
    saint_log_set_ros_ready(false);
    (void)rcl_publisher_fini(&log_pub, &ros_node);
    (void)rcl_publisher_fini(&state_pub, &ros_node);
    (void)rcl_publisher_fini(&announcement_pub, &ros_node);
    (void)rcl_timer_fini(&state_timer);
    (void)rcl_timer_fini(&announce_timer);
    rclc_executor_fini(&executor);
    (void)rcl_node_fini(&ros_node);
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

    /* Recovery path: if we're in ERROR state, wait ERROR_RECOVERY_DELAY_MS
     * and then reset the reconnect counter to try again. Without this
     * the node sits in ERROR forever after any transient agent
     * disruption — a `systemctl restart saint-os` on the Pi would
     * permanently brick the chip until power-cycle. Mirrors RP2040's
     * recovery path in firmware/rp2040/src/main.c. */
    if (g_node.state == NODE_STATE_ERROR) {
        if (error_entered_at == 0) {
            error_entered_at = now;
        }
        if ((uint32_t)(now - error_entered_at) < ERROR_RECOVERY_DELAY_MS) {
            return false;
        }
        Serial.printf("Recovering from ERROR state — retrying agent connection\n");
        reconnect_attempts = 0;
        error_entered_at = 0;
        /* Drop back to UNADOPTED so announces resume (they're gated
         * to UNADOPTED/ACTIVE; the server will re-confirm adoption
         * from its persisted config). */
        node_set_state(NODE_STATE_UNADOPTED);
        led_set_state(NODE_STATE_UNADOPTED);
    }

    // Only check periodically
    if (now - last_connection_check < CONNECTION_CHECK_INTERVAL_MS) {
        return agent_connected;
    }
    last_connection_check = now;

    if (!TRANSPORT_CONNECTED()) {
        Serial.printf("Transport disconnected\n");
        agent_connected = false;
    }

    /* RX-based liveness — DO NOT trust TX success on NativeEthernet.
     * udp.endPacket() returns true even when the agent is gone (the
     * Pi's stack happily ACKs the ARP, the gateway drops the UDP).
     * Tracking the last actual RX (g_transport_last_rx_ms, set inside
     * transport_native_eth_read when udp.parsePacket() returns >0)
     * catches a server reboot within ~CONNECTION_TIMEOUT_MS — the
     * agent's XRCE heartbeat stream stops, the chip's read polls go
     * empty, and this branch trips into the reconnect path below.
     *
     * RP2040 uses last_successful_comm (TX-based) for the same purpose
     * — its W5500 stack propagates true link status so TX is a valid
     * signal there. See firmware/rp2040/src/main.c. */
    if (agent_connected && g_transport_last_rx_ms > 0) {
        if ((uint32_t)(now - g_transport_last_rx_ms) > CONNECTION_TIMEOUT_MS) {
            Serial.printf("Agent silent: no RX for %lu ms\n",
                          (unsigned long)(now - g_transport_last_rx_ms));
            agent_connected = false;
        }
    }

    if (!agent_connected) {
        reconnect_attempts++;

        if (reconnect_attempts > MAX_RECONNECT_ATTEMPTS) {
            // Too many failures — drop into ERROR. The recovery path
            // at the top of this function pulls us back out after
            // ERROR_RECOVERY_DELAY_MS so we keep trying forever.
            Serial.printf("Max reconnect attempts exceeded (%d), entering ERROR state\n",
                          MAX_RECONNECT_ATTEMPTS);
            node_set_state(NODE_STATE_ERROR);
            led_set_state(NODE_STATE_ERROR);
            error_entered_at = now;
            return false;
        }

        Serial.printf("Reconnect attempt %u/%d...\n",
                       (unsigned)reconnect_attempts, MAX_RECONNECT_ATTEMPTS);
        led_set_state(NODE_STATE_CONNECTING);

        // Wait before reconnect, feeding the watchdog so the wait
        // itself doesn't trip the 30 s WDOG1.
        for (uint32_t slept = 0; slept < RECONNECT_DELAY_MS; slept += 200) {
            watchdog_feed();
            delay(200);
        }

        if (!TRANSPORT_CONNECTED()) {
            if (!TRANSPORT_CONNECT()) {
                Serial.printf("Transport reconnect failed\n");
                return false;
            }
        }

        Serial.printf("Reinitializing micro-ROS...\n");
        cleanup_micro_ros();
        watchdog_feed();
        delay(500);

        if (!init_micro_ros()) {
            Serial.printf("micro-ROS reinitialization failed\n");
            return false;
        }

        Serial.printf("Reconnected to agent after %u attempts\n",
                       (unsigned)reconnect_attempts);
        agent_connected = true;
        last_successful_comm = now;
        g_transport_last_rx_ms = now;
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
    /* unique_id is exactly 16 hex chars (OCOTP MAC0+MAC1 → "%08lx%08lx"
     * in hardware_get_unique_id, see hardware.cpp). Size the buffer
     * to that bound so the compiler can prove "teensy41_" + unique_id
     * (9 + 16 = 25 chars) fits in g_node.node_id (32 bytes) without
     * truncation — otherwise -Wformat-truncation flags it because
     * a 32-byte input could theoretically overflow. */
    char unique_id[17];
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
    // Retry-forever discovery via the shared helper. The Pi-side server
    // and node typically power up together; the Pi takes 30-60s to
    // boot Linux + start saint-os + bind the discovery listener.
    // discover_server_retry_forever() keeps trying with backoff and
    // drives the LED + watchdog so neither stalls during the wait.
    // See firmware/shared/src/discovery_retry.c — RP2040 uses the same
    // call, so cold-boot resilience now stays in lockstep across
    // platforms by construction (was a divergent open-coded loop on
    // each side, with Teensy notoriously giving up after 20s).
    discover_server_retry_forever(g_node.server_ip, &g_node.server_port,
                                  2000, 10);
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
