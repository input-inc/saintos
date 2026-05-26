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
#include <stdarg.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"
#include "hardware/structs/vreg_and_chip_reset.h"
#include "hardware/regs/vreg_and_chip_reset.h"

/* Bootloader sentinel: when an OTA fails MAX_OTA_RETRIES times, the
 * bootloader writes this to scratch[3] before booting the old app
 * so we can report it. Must match the value in
 * firmware/rp2040/bootloader/main.c. */
#define OTA_GAVE_UP_MAGIC 0xFA11ED01u

/* Hard-fault context captured by the ISR below and inspected at boot
 * to surface "where did we crash last time" in the Logs tab.
 *
 * Lives in the .uninitialized_data section so the crt0 runtime DOES
 * NOT zero it during bss-clear at startup. That means a watchdog or
 * software reset (e.g. the watchdog_reboot() we trigger from the fault
 * handler) preserves these fields, but a power-on / brown-out reset
 * does NOT (RAM contents are undefined after POR/BOR). That's exactly
 * the discrimination we want: if we boot and see crash_magic set to
 * CRASH_MAGIC, the previous reset was a hard fault we caught; if it
 * was cleared/random, the previous reset was likely brown-out.
 *
 * Why not watchdog scratch[]? scratch[0..6] are all claimed by the
 * bootloader's OTA handoff protocol (size/crc/magic/retry counter),
 * and scratch[7] alone isn't enough room for magic+pc+lr. The noinit
 * RAM section is the standard place for crash persistence anyway.
 */
#define CRASH_MAGIC 0xFA1750FFu
typedef struct {
    uint32_t magic;   // == CRASH_MAGIC iff PC/LR below are valid
    uint32_t pc;      // faulting instruction
    uint32_t lr;      // return address at fault
} crash_info_t;
static __attribute__((section(".uninitialized_data")))
    volatile crash_info_t crash_info;

/* Hard-fault handler. Cortex-M0+ on RP2040 doesn't have BusFault /
 * UsageFault — every escalation lands here. The exception entry pushes
 * R0-R3, R12, LR, PC, xPSR onto whichever stack was active (SPSEL bit
 * in EXC_RETURN says which). The naked trampoline reads SPSEL, picks
 * the right SP, and forwards to the C handler with the stack pointer
 * as its single argument. The C handler reads PC (sp[6]) and LR (sp[5])
 * out of the exception frame, stashes them into crash_info, then
 * triggers a clean watchdog reboot so the bootloader runs and the next
 * boot can report what we caught.
 *
 * The function name `isr_hardfault` matches the weak symbol the Pico
 * SDK's crt0 exposes for the HardFault vector — defining a strong
 * symbol with this exact name overrides the default (which just sits
 * in a wfi loop until the watchdog kills it).
 */
static void __attribute__((used)) isr_hardfault_c(uint32_t* sp);

void __attribute__((naked, used)) isr_hardfault(void)
{
    __asm volatile(
        "movs r0, #4         \n"   // bit 2 of EXC_RETURN = SPSEL
        "mov  r1, lr         \n"
        "tst  r0, r1         \n"
        "beq  1f             \n"
        "mrs  r0, psp        \n"   // SPSEL set → process stack was active
        "b    2f             \n"
        "1:                  \n"
        "mrs  r0, msp        \n"   // SPSEL clear → main stack was active
        "2:                  \n"
        "ldr  r1, =isr_hardfault_c \n"
        "bx   r1             \n"
    );
}

static void __attribute__((used)) isr_hardfault_c(uint32_t* sp)
{
    /* Cortex-M0+ exception frame:
     *   sp[0]=R0  sp[1]=R1  sp[2]=R2  sp[3]=R3
     *   sp[4]=R12 sp[5]=LR  sp[6]=PC  sp[7]=xPSR
     */
    crash_info.pc    = sp[6];
    crash_info.lr    = sp[5];
    crash_info.magic = CRASH_MAGIC;
    /* watchdog_reboot(0, 0, 0) arms a 1-tick watchdog and the function
     * hangs until it fires — gives us a clean reset through the
     * bootloader without re-executing the faulting instruction. */
    watchdog_reboot(0, 0, 0);
    while (1) {
        tight_loop_contents();
    }
}

/* Bootloader failure reason codes. Must mirror the OTA_FAIL_* set in
 * firmware/rp2040/bootloader/main.c. We don't share a header because
 * the bootloader builds in isolation; an unknown code maps to
 * "unknown reason" via ota_fail_reason_str() below. */
static const char* ota_fail_reason_str(uint8_t reason)
{
    switch (reason) {
        case 0:  return "unknown";
        case 1:  return "bad image size";
        case 2:  return "W5500 init failed";
        case 3:  return "DHCP failed — node has no IP";
        case 4:  return "couldn't open TCP to server";
        case 5:  return "server returned non-2xx";
        case 6:  return "server response missing Content-Length";
        case 7:  return "server closed connection mid-stream";
        case 8:  return "TCP recv error during download";
        case 9:  return "HTTP error";
        case 10: return "size mismatch (got != expected bytes)";
        case 11: return "stream CRC mismatch";
        case 12: return "post-write flash CRC mismatch";
        default: return "unrecognized failure code";
    }
}

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
#include "peripheral_driver.h"
#include "syren_driver.h"
#include "fas100_driver.h"
#include "roboclaw_driver.h"
#include "pathfinder_bms_driver.h"

#ifdef SAINT_OS_OTA_BOOTLOADER
#include "picowota/reboot.h"
#endif

#if defined(SAINT_OS_OTA_BOOTLOADER) && !defined(SIMULATION)
#include "bootloader_info.h"
#endif

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
static rcl_publisher_t state_pub;          // Pin state publisher
static rcl_publisher_t log_pub;            // Node log lines (best-effort)

// Subscribers
static rcl_subscription_t config_sub;
static rcl_subscription_t control_sub;     // Streaming pin/channel writes (BEST_EFFORT)
static rcl_subscription_t command_sub;     // One-shot operator actions (RELIABLE)

// Timers
static rcl_timer_t announce_timer;
static rcl_timer_t state_timer;            // State publish timer (10Hz)

// Message buffers
static std_msgs__msg__String announcement_msg;
static char announcement_buffer[512];

static std_msgs__msg__String config_msg;
static char config_buffer[2048];        // Buffer for incoming config

static std_msgs__msg__String control_msg;
static char control_buffer[512];        // Buffer for incoming control commands

static std_msgs__msg__String command_msg;
static char command_buffer[512];        // Buffer for incoming one-shot commands

static std_msgs__msg__String state_msg;
static char state_buffer[2048];         // Buffer for outgoing state

static std_msgs__msg__String log_msg;
static char log_buffer[256];            // One log line at a time

// State publish interval
#define STATE_PUBLISH_INTERVAL_MS 100   // 10Hz

// Connection monitoring
#define CONNECTION_CHECK_INTERVAL_MS 5000   // Check every 5 seconds
#define CONNECTION_TIMEOUT_MS        15000  // Consider disconnected after 15s
#define MAX_RECONNECT_ATTEMPTS       10     // Max consecutive failures before error state
#define ERROR_RECOVERY_DELAY_MS      30000  // Stay in ERROR for 30s, then try again
// Note: RECONNECT_DELAY_MS is defined in saint_node.h

// Hardware watchdog. The RP2040 watchdog caps at ~8.4 seconds, which
// is plenty for the post-strip apply path (parse JSON → set pin modes
// → flash write → ACTIVE — all sub-second on real hardware). It used
// to be 0 because pre-strip firmware did a synchronous capability
// publish during apply which sometimes blocked >8 s on W5500 ARP
// retransmit; that publish is gone. With this armed, an in-the-robot
// hang is now visible: the chip resets, the server sees the node
// drop, and the boot path emits a "Recovered from watchdog reset"
// log line that pairs with the per-node Logs tab.
#define WATCHDOG_TIMEOUT_MS          8000

static uint32_t last_successful_comm = 0;   // Timestamp of last successful communication
static uint32_t last_connection_check = 0;  // Timestamp of last connection check
static uint32_t reconnect_attempts = 0;     // Consecutive reconnect attempts
static bool agent_connected = false;        // Agent connection state
static uint32_t error_entered_at = 0;       // When we entered NODE_STATE_ERROR (0 = not in error)

// Forward declarations for connection monitoring
static void mark_agent_communication(void);
static bool check_agent_connection(void);
static bool init_micro_ros(void);

// Track whether init_micro_ros() has run so saint_log_publish can be a
// safe no-op during pre-agent boot. Set to true after the log publisher
// is registered with the agent.
static bool ros_log_ready = false;

// Cached bootloader version string. Resolved once at boot from the
// fixed-address descriptor in bootloader flash (see
// shared/include/bootloader_info.h) so we don't poke memory-mapped
// flash on every announcement tick. "unknown" when the descriptor is
// absent — older bootloader, simulation build, or app running without
// SAINT_OS_OTA_BOOTLOADER. Used in /announce JSON as the "bl_fw"
// field; surfaced in the deferred boot log.
static char g_bl_version[48] = "unknown";

static void resolve_bootloader_version(void)
{
#if defined(SAINT_OS_OTA_BOOTLOADER) && !defined(SIMULATION)
    const bootloader_info_t* info = saint_bootloader_info();
    if (info && info->struct_version <= BOOTLOADER_INFO_STRUCT_VERSION) {
        // Copy + force null-termination — flash is read-only but the
        // string could theoretically be unterminated if the descriptor
        // is corrupt; defensive bound matters here because everything
        // downstream prints it as a C string.
        snprintf(g_bl_version, sizeof(g_bl_version), "%s", info->version_string);
        g_bl_version[sizeof(g_bl_version) - 1] = '\0';
    }
#endif
}

// -----------------------------------------------------------------------------
// Deferred boot-status logging buffer
// -----------------------------------------------------------------------------
//
// The boot-log queue stores log entries created BEFORE the server has
// created our per-node /saint/nodes/<id>/log subscription. boot_log_queue
// and boot_log_flush_if_due (defined further down) drive its public
// API, but the struct + globals live up here so saint_log_publish can
// fall into the same buffer when ros_log_ready is still false.
//
// Bumped from 5 to 24 because saint_log_publish now also enqueues here
// when ROS isn't ready, so every driver's boot-time log (RoboClaw
// restored/bound/probe lines, FAS100/SyRen equivalents, etc.) has to
// fit alongside the handful of main.c lines. 24 × ~210 B ≈ 5 KB of
// RAM, comfortable on RP2040.
#define BOOT_LOG_MAX        24
#define BOOT_LOG_LEVEL_LEN  8
#define BOOT_LOG_TEXT_LEN   200
typedef struct {
    char level[BOOT_LOG_LEVEL_LEN];
    char text[BOOT_LOG_TEXT_LEN];
} boot_log_entry_t;
static boot_log_entry_t g_boot_log[BOOT_LOG_MAX];
static size_t           g_boot_log_count   = 0;
static bool             g_boot_log_flushed = false;
static unsigned         g_announce_count   = 0;

// =============================================================================
// Remote logging
// =============================================================================
//
// saint_log_publish() ships a single line to the server over the
// /saint/nodes/<id>/log topic AND prints to UART. When ROS isn't
// connected yet, the entry is buffered into the boot-log queue above
// and replayed later by boot_log_flush_if_due — that's the only way
// early-boot driver logs (e.g. RoboClaw's "bound PIO UART" line
// emitted from inside pin_config_load) reach the dashboard.
// Format: {"level":"info|warn|error","text":"...","uptime_ms":N}.
//
// Keep individual lines under ~200 chars to fit log_buffer with the
// JSON envelope; longer ones are truncated. Don't call from ISRs
// (printf isn't ISR-safe) and don't call faster than ~10 Hz; this is
// for events, not telemetry.
void saint_log_publish(const char* level, const char* fmt, ...)
{
    char text[200];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(text, sizeof(text), fmt, ap);
    va_end(ap);

    // Always echo to UART so a serial dev console still sees everything.
    printf("[%s] %s\n", level, text);

    if (!ros_log_ready) {
        // Buffer for later replay so early-boot driver logs (e.g.
        // RoboClaw's "bound PIO UART" line emitted from inside
        // pin_config_load → drv_load → roboclaw_init) survive the
        // window where ROS isn't connected yet. boot_log_flush_if_due
        // drains this queue once the second announcement has gone
        // out. The buffer is bounded — once full we drop subsequent
        // entries silently rather than evicting earlier ones, since
        // the EARLY logs are the diagnostic gold we'd lose first
        // otherwise. If you hit the cap, raise BOOT_LOG_MAX.
        if (g_boot_log_count < BOOT_LOG_MAX && !g_boot_log_flushed) {
            boot_log_entry_t* e = &g_boot_log[g_boot_log_count++];
            snprintf(e->level, sizeof(e->level), "%s", level);
            snprintf(e->text,  sizeof(e->text),  "%s", text);
        }
        return;
    }

    uint32_t up = to_ms_since_boot(get_absolute_time());

    // JSON-escape inside the text so the server can json.loads() it.
    // The big three escapes are quote, backslash, and any control char
    // (0x00-0x1F). Control chars MUST be escaped in JSON — a literal
    // newline in a string is the most common offender (the RoboClaw
    // GETVERSION response ends with \n, and the version log line
    // embeds that). Previous version of this function only escaped
    // " and \, so the server saw `"...v4.4.8\n"` with a real newline
    // and json.loads() bailed with "Invalid control character at
    // line 1 column N". We now expand the common controls into their
    // backslash forms and pass everything else through.
    char escaped[256];
    size_t ei = 0;
    for (size_t i = 0; text[i] && ei < sizeof(escaped) - 7; i++) {
        unsigned char c = (unsigned char)text[i];
        if (c == '"' || c == '\\') {
            escaped[ei++] = '\\';
            escaped[ei++] = (char)c;
        } else if (c == '\n') {
            escaped[ei++] = '\\'; escaped[ei++] = 'n';
        } else if (c == '\r') {
            escaped[ei++] = '\\'; escaped[ei++] = 'r';
        } else if (c == '\t') {
            escaped[ei++] = '\\'; escaped[ei++] = 't';
        } else if (c < 0x20) {
            // Rare control char — emit \u00XX so the server stays parseable
            // without us having to remember every escape rule.
            static const char hex[] = "0123456789abcdef";
            escaped[ei++] = '\\'; escaped[ei++] = 'u';
            escaped[ei++] = '0';  escaped[ei++] = '0';
            escaped[ei++] = hex[(c >> 4) & 0xF];
            escaped[ei++] = hex[c & 0xF];
        } else {
            escaped[ei++] = (char)c;
        }
    }
    escaped[ei] = '\0';

    int len = snprintf(log_buffer, sizeof(log_buffer),
        "{\"level\":\"%s\",\"text\":\"%s\",\"uptime_ms\":%lu}",
        level, escaped, (unsigned long)up);
    if (len < 0 || (size_t)len >= sizeof(log_buffer)) return;

    log_msg.data.data = log_buffer;
    log_msg.data.size = (size_t)len;
    log_msg.data.capacity = sizeof(log_buffer);
    /* Ignore the publish return — log is best-effort. */
    (void)rcl_publish(&log_pub, &log_msg, NULL);
}

// -----------------------------------------------------------------------------
// Deferred boot-status logging
// -----------------------------------------------------------------------------
//
// Why: the server creates the per-node /saint/nodes/<id>/log subscriber
// LAZILY, on first receipt of an announcement (server_node.py
// _on_node_announcement → _ensure_node_log_subscriber). If we
// saint_log_publish() immediately after init_micro_ros(), it goes out
// before any announcement has been published, so the server has no
// subscriber yet and the message is silently dropped. Boot context
// ("Boot OK", "Recovered from watchdog reset", "OTA failed after
// retries: ...") is exactly the information you most need in the
// Logs tab — so we can't lose it.
//
// Fix: queue those first lines in RAM and replay them from the
// announce-timer callback once at least two announcements have gone
// out. That window (≥1 s) is enough for the server to create its
// subscription on receipt of the first announce.
//
// The buffer (boot_log_entry_t / g_boot_log[] / counters) is declared
// up above saint_log_publish so that function can fall into the same
// buffer when ROS isn't ready yet.

static void boot_log_queue(const char* level, const char* fmt, ...)
{
    // Always echo to UART so a serial dev console still sees it
    // immediately, even before the ROS-side replay happens.
    char text[BOOT_LOG_TEXT_LEN];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(text, sizeof(text), fmt, ap);
    va_end(ap);
    printf("[%s] (pending) %s\n", level, text);

    if (g_boot_log_count >= BOOT_LOG_MAX) return;
    boot_log_entry_t* e = &g_boot_log[g_boot_log_count++];
    snprintf(e->level, sizeof(e->level), "%s", level);
    snprintf(e->text,  sizeof(e->text),  "%s", text);
}

static void boot_log_flush_if_due(void)
{
    if (g_boot_log_flushed) return;
    // Need ≥2 announcements: the first one triggers server-side
    // subscription creation; from the second onward we know the
    // subscriber is up.
    if (g_announce_count < 2) return;
    for (size_t i = 0; i < g_boot_log_count; i++) {
        saint_log_publish(g_boot_log[i].level, "%s", g_boot_log[i].text);
    }
    g_boot_log_flushed = true;
}

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

    // Check for configure action
    if (strstr(msg->data.data, "\"action\":\"configure\"") ||
        strstr(msg->data.data, "\"action\": \"configure\"")) {
        saint_log_publish("info", "Config received (%zu bytes), applying…",
                          msg->data.size);

        if (pin_config_apply_json(msg->data.data, msg->data.size)) {
            saint_log_publish("info", "Config applied OK");

            // Save to flash
            if (pin_config_save()) {
                saint_log_publish("info", "Config saved to flash");
            } else {
                saint_log_publish("error", "Flash save failed");
            }

            // Transition to ACTIVE state (node is now adopted)
            if (g_node.state != NODE_STATE_ACTIVE) {
                saint_log_publish("info", "Adopted — entering ACTIVE state");
                node_set_state(NODE_STATE_ACTIVE);
                led_set_state(NODE_STATE_ACTIVE);
            }
        } else {
            saint_log_publish("error", "Config apply failed");
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
        saint_log_publish("error",
            "Firmware update: invalid command (len=%zu)", len);
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

    saint_log_publish("info",
        "OTA: firmware_update received (current=%s server=%s force=%s)",
        FIRMWARE_VERSION_FULL, new_version, force_update ? "YES" : "NO");

    // If not forced, compare version timestamps
    if (!force_update) {
        uint32_t current_ts = FIRMWARE_BUILD_UNIX;
        uint32_t server_ts = extract_version_timestamp(new_version);

        if (server_ts == 0) {
            saint_log_publish("warn",
                "OTA: cannot parse server version timestamp '%s' — "
                "aborting (use force to override)", new_version);
            return;
        }

        if (server_ts <= current_ts) {
            saint_log_publish("info",
                "OTA: already up to date (current=%lu server=%lu) — "
                "aborting (use force to override)",
                (unsigned long)current_ts, (unsigned long)server_ts);
            return;
        }

        saint_log_publish("info",
            "OTA: server has newer build (current=%lu server=%lu), proceeding",
            (unsigned long)current_ts, (unsigned long)server_ts);
    } else {
        saint_log_publish("info", "OTA: force update — skipping version check");
    }

#if defined(SAINT_OS_OTA_BOOTLOADER) && !defined(SIMULATION)
    // Pull image size + CRC32 out of the control JSON and hand off to
    // the bootloader for an HTTP-over-W5500 download. The bootloader
    // knows its own app load address — we don't transmit it.
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
            // Accept "0xNN..." or decimal
            int base = (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) ? 16 : 10;
            img_crc = (uint32_t)strtoul(p, NULL, base);
        }
    }
    if (img_size == 0 || img_crc == 0) {
        saint_log_publish("error",
            "OTA: missing size or crc32 in control message "
            "(size=%lu crc=0x%08lx) — aborting. Rebuild the server "
            "firmware package so it includes the raw .bin.",
            (unsigned long)img_size, (unsigned long)img_crc);
        return;
    }
    saint_log_publish("info",
        "OTA: handing off to bootloader (size=%lu crc=0x%08lx) — "
        "rebooting now; further status will appear after next boot.",
        (unsigned long)img_size, (unsigned long)img_crc);
    sleep_ms(200);
    saint_ota_reboot_with_image(img_size, img_crc);
    /* not reached */
#else
    saint_log_publish("warn",
        "OTA: no bootloader compiled in (SAINT_OS_OTA_BOOTLOADER unset) — "
        "rebooting into same firmware");
    sleep_ms(500);
#  ifdef SIMULATION
    /* Renode tooling restarts the simulator with a different ELF. */
    while (1) { tight_loop_contents(); }
#  else
    watchdog_enable(1, 1);
    while (1) { tight_loop_contents(); }
#  endif
#endif
}

/**
 * Subscription callback for control messages.
 * Handles runtime pin value changes from the server.
 */
/* Dispatch action JSON from either /control (BEST_EFFORT, streaming
 * writes) or /command (RELIABLE, operator one-shots). Both callbacks
 * funnel here so the firmware accepts each action regardless of which
 * topic the server publishes on — keeps backward-compat with servers
 * that haven't been upgraded to the topic split, and forward-compat
 * with future actions that may move between topics. The split exists
 * for *delivery* semantics (QoS), not action authorization. */
static void dispatch_action_buffer(const char* data, size_t size)
{
    // Check for firmware update command
    if (strstr(data, "\"action\":\"firmware_update\"") ||
        strstr(data, "\"action\": \"firmware_update\"")) {
        handle_firmware_update(data, size);
        return;
    }

    // Check for factory reset command
    if (strstr(data, "\"action\":\"factory_reset\"") ||
        strstr(data, "\"action\": \"factory_reset\"")) {
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
    if (strstr(data, "\"action\":\"restart\"") ||
        strstr(data, "\"action\": \"restart\"")) {
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
    if (strstr(data, "\"action\":\"identify\"") ||
        strstr(data, "\"action\": \"identify\"")) {
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

    // Emergency stop — engage. Sets all outputs to safe values AND
    // drives peripheral e-stop latches (e.g. RoboClaw estop_pin → HIGH).
    if (strstr(data, "\"action\":\"estop\"") ||
        strstr(data, "\"action\": \"estop\"")) {
        printf("\n");
        printf("====================================\n");
        printf("EMERGENCY STOP ACTIVATED\n");
        printf("====================================\n");
        pin_control_estop();
        printf("====================================\n\n");
        return;
    }

    // Emergency stop — release. Drives peripheral latches back to
    // their deasserted state (RoboClaw estop_pin → LOW) so motor
    // commands flow again. Direct pin outputs (PWM/servo/digital)
    // are NOT auto-restored — the host should resend whatever
    // values it wants.
    if (strstr(data, "\"action\":\"clear_estop\"") ||
        strstr(data, "\"action\": \"clear_estop\"")) {
        printf("\n");
        printf("====================================\n");
        printf("EMERGENCY STOP RELEASED\n");
        printf("====================================\n");
        pin_control_clear_estop();
        printf("====================================\n\n");
        return;
    }

    // Server-controlled NeoPixel color override. Without this handler
    // every byte we send from the server to drive the status LED gets
    // overwritten by the next led_update() tick (~10-100 Hz). With it,
    // led_update() honors the supplied color until a clear command
    // arrives. JSON forms:
    //   {"action":"set_neopixel","r":255,"g":0,"b":0,"brightness":128}
    //   {"action":"set_neopixel","clear":true}    // resume state LED
    if (strstr(data, "\"action\":\"set_neopixel\"") ||
        strstr(data, "\"action\": \"set_neopixel\"")) {
        if (strstr(data, "\"clear\":true") ||
            strstr(data, "\"clear\": true")) {
            led_clear_override();
            saint_log_publish("info",
                "NeoPixel: override cleared — resuming state-driven LED");
            return;
        }

        // Tiny inline parser — matches the style used by the other
        // action handlers in this file. Defaults make a partial JSON
        // do something predictable (all-off + full brightness would
        // leave the pixel dark, which is at least non-misleading).
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
        saint_log_publish("info",
            "NeoPixel: override set RGB=(%d,%d,%d) brightness=%d",
            r, g, b, brightness);
        return;
    }

    // RoboClaw wire-level debug passthrough. Diagnostic-only; the host
    // CLI uses this to send raw bytes, sniff the idle line, and rebind
    // the PIO UART without an OTA. See roboclaw_debug_handle_json().
    if (strstr(data, "\"action\":\"roboclaw_debug\"")
        || strstr(data, "\"action\": \"roboclaw_debug\"")) {
        roboclaw_debug_handle_json(data);
        return;
    }

    // Apply pin control command
    if (pin_control_apply_json(data, size)) {
        printf("Control command applied\n");
    }
}

/* Streaming pin/channel writes on /control (BEST_EFFORT). */
static void control_subscription_callback(const void* msgin)
{
    const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;
    if (!msg || !msg->data.data) return;
    if (msg->data.size >= sizeof(control_buffer)) {
        saint_log_publish("error",
            "Control message too large: %zu >= %zu, rejecting",
            msg->data.size, sizeof(control_buffer));
        return;
    }
    control_buffer[msg->data.size] = '\0';
    printf("Control received: %.*s\n",
           (int)(msg->data.size < 80 ? msg->data.size : 80),
           msg->data.data);
    dispatch_action_buffer(msg->data.data, msg->data.size);
}

/* One-shot operator actions on /command (RELIABLE). */
static void command_subscription_callback(const void* msgin)
{
    const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;
    if (!msg || !msg->data.data) return;
    if (msg->data.size >= sizeof(command_buffer)) {
        saint_log_publish("error",
            "Command message too large: %zu >= %zu, rejecting",
            msg->data.size, sizeof(command_buffer));
        return;
    }
    command_buffer[msg->data.size] = '\0';
    printf("Command received: %.*s\n",
           (int)(msg->data.size < 80 ? msg->data.size : 80),
           msg->data.data);
    dispatch_action_buffer(msg->data.data, msg->data.size);
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

    // Announce when unadopted or active (for heartbeat/online detection)
    if (g_node.state != NODE_STATE_UNADOPTED && g_node.state != NODE_STATE_ACTIVE) {
        return;
    }

    // Get CPU temperature
    float cpu_temp = hardware_get_cpu_temp();

    // Build announcement JSON string.
    //
    // chip_family identifies the silicon family ("rp2040") — the server
    // matches this against server/config/boards/<chip>/global.yaml to
    // pick a pin-layout YAML. Read from the SYSINFO_CHIP_ID register so
    // the value is observable, not just a compile-time string. Keeping
    // hw, fw, fw_build as before for display + version comparison.
    //
    // SYSINFO_CHIP_ID layout (RP2040 datasheet § 2.20.4):
    //   bits 31:28  REVISION   (silicon stepping — variable across batches)
    //   bits 27:12  PART       = 0x0002 for RP2040
    //   bits 11:0   MANUFACTURER = 0x927 (Raspberry Pi)
    // So the lower 28 bits = (PART << 12) | MANUFACTURER = 0x00002927.
    uint32_t chip_id_reg = *(volatile uint32_t *)0x40000000;
    const char* chip_family =
        (chip_id_reg & 0x0FFFFFFF) == 0x00002927u ? "rp2040" : "unknown";

    int ann_len = snprintf(announcement_buffer, sizeof(announcement_buffer),
        "{"
        "\"node_id\":\"%s\","
        "\"chip_family\":\"%s\","
        "\"chip_id\":\"0x%08lx\","
        "\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
        "\"ip\":\"%d.%d.%d.%d\","
        "\"hw\":\"%s\","
        "\"fw\":\"%s\","
        "\"bl_fw\":\"%s\","
        "\"fw_build\":\"%s\","
        "\"state\":\"%s\","
        "\"uptime\":%lu,"
        "\"cpu_temp\":%.1f,"
        "\"peripherals\":{",
        g_node.node_id,
        chip_family,
        (unsigned long)chip_id_reg,
        g_node.mac_address[0], g_node.mac_address[1],
        g_node.mac_address[2], g_node.mac_address[3],
        g_node.mac_address[4], g_node.mac_address[5],
        g_node.static_ip[0], g_node.static_ip[1],
        g_node.static_ip[2], g_node.static_ip[3],
        HARDWARE_MODEL,
        FIRMWARE_VERSION_FULL,
        g_bl_version,
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
        // First announcement triggers server-side log subscription;
        // once we're past it, deferred boot-status lines can be sent.
        if (g_announce_count < 1000) g_announce_count++;
        boot_log_flush_if_due();
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

    // Control commands are streaming joystick state. Use BEST_EFFORT
    // QoS to match the server-side publisher and avoid the deadstick-
    // queues-behind-stale-values pathology: with RELIABLE + KEEP_LAST(10)
    // a single lost UDP packet stalls the queue while DDS retransmits,
    // and the return-to-zero ends up sitting at queue position 10. Must
    // match the publisher's QoS (BEST_EFFORT + KEEP_LAST(1)) or DDS
    // won't even establish the connection.
    ret = rclc_subscription_init_best_effort(
        &control_sub,
        &ros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        control_topic
    );
    if (ret != RCL_RET_OK) {
        printf("Failed to create control subscription: %d\n", ret);
        return false;
    }
    printf("Subscribed to control: %s (best-effort)\n", control_topic);

    // Initialize control message buffer
    control_msg.data.data = control_buffer;
    control_msg.data.size = 0;
    control_msg.data.capacity = sizeof(control_buffer);

    // Create command subscriber — operator one-shots (firmware_update,
    // factory_reset, identify, estop, …). RELIABLE so a single dropped
    // packet doesn't lose an OTA trigger or estop. Topic is
    // /saint/nodes/<id>/command — separate from /control so this
    // subscription's QoS doesn't have to share the streaming-write
    // tradeoffs. Server-side counterpart: server_node.py COMMAND_QOS.
    char command_topic[64];
    snprintf(command_topic, sizeof(command_topic),
             "/saint/nodes/%s/command", g_node.node_id);
    for (char* p = command_topic; *p; p++) {
        if (*p == '-' || *p == ':') *p = '_';
    }
    ret = rclc_subscription_init_default(
        &command_sub,
        &ros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        command_topic
    );
    if (ret != RCL_RET_OK) {
        printf("Failed to create command subscription: %d\n", ret);
        return false;
    }
    printf("Subscribed to command: %s (reliable)\n", command_topic);

    command_msg.data.data = command_buffer;
    command_msg.data.size = 0;
    command_msg.data.capacity = sizeof(command_buffer);

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

    // Create log publisher (best-effort). The server subscribes per
    // adopted node and feeds each line into the per-node Logs tab.
    char log_topic[64];
    snprintf(log_topic, sizeof(log_topic),
             "/saint/nodes/%s/log", g_node.node_id);
    for (char* p = log_topic; *p; p++) {
        if (*p == '-' || *p == ':') *p = '_';
    }

    ret = rclc_publisher_init_default(
        &log_pub,
        &ros_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        log_topic
    );
    if (ret != RCL_RET_OK) {
        printf("Failed to create log publisher: %d\n", ret);
        return false;
    }
    printf("Created log publisher: %s\n", log_topic);
    ros_log_ready = true;

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

    // Initialize executor with 2 timers + 3 subscriptions (config,
    // control, command).
    ret = rclc_executor_init(&executor, &support.context, 5, &allocator);
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

    // Add command subscription to executor
    ret = rclc_executor_add_subscription(
        &executor,
        &command_sub,
        &command_msg,
        command_subscription_callback,
        ON_NEW_DATA
    );
    if (ret != RCL_RET_OK) {
        printf("Failed to add command subscription: %d\n", ret);
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
    rcl_subscription_fini(&command_sub, &ros_node);
    rcl_subscription_fini(&control_sub, &ros_node);
    rcl_subscription_fini(&config_sub, &ros_node);
    ros_log_ready = false;
    rcl_publisher_fini(&log_pub, &ros_node);
    rcl_publisher_fini(&state_pub, &ros_node);
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

    // Recovery path: if we're in ERROR state, wait ERROR_RECOVERY_DELAY_MS
    // and then reset the reconnect counter to try again. Without this the
    // node sits in ERROR forever after any transient agent disruption.
    if (g_node.state == NODE_STATE_ERROR) {
        if (error_entered_at == 0) {
            error_entered_at = now;
        }
        if (now - error_entered_at < ERROR_RECOVERY_DELAY_MS) {
            return false;
        }
        printf("Recovering from ERROR state — retrying agent connection\n");
        reconnect_attempts = 0;
        error_entered_at = 0;
        // Drop back to UNADOPTED so announcements resume (they're
        // gated to UNADOPTED/ACTIVE; the agent will re-confirm
        // adoption from the server-side config).
        node_set_state(NODE_STATE_UNADOPTED);
        led_set_state(NODE_STATE_UNADOPTED);
    }

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
            // Too many failures - enter error state. Recovery path at
            // the top of this function will pull us out after the
            // ERROR_RECOVERY_DELAY_MS cooldown.
            printf("Max reconnect attempts exceeded (%d), entering ERROR state\n",
                   MAX_RECONNECT_ATTEMPTS);
            node_set_state(NODE_STATE_ERROR);
            led_set_state(NODE_STATE_ERROR);
            error_entered_at = now;
            return false;
        }

        printf("Reconnect attempt %lu/%d...\n", reconnect_attempts, MAX_RECONNECT_ATTEMPTS);
        led_set_state(NODE_STATE_CONNECTING);

        // Wait before reconnect, kicking the watchdog every chunk so
        // a 5s wait doesn't trip the 8s timer (cleanup + init adds more).
        for (uint32_t slept = 0; slept < RECONNECT_DELAY_MS; slept += 200) {
            watchdog_update();
            sleep_ms(200);
        }

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
        watchdog_update();
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
        saint_log_publish("info", "micro-ROS agent reconnected");
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

    // Snapshot the chip-level reset reason BEFORE anything else has a
    // chance to influence it. The HAD_POR / HAD_RUN / HAD_PSM_RESTART
    // bits in VREG_AND_CHIP_RESET.CHIP_RESET are latched at reset and
    // tell us which path the silicon took:
    //   HAD_POR         — power-on or brown-out (BOR; indistinguishable in HW)
    //   HAD_RUN         — RUN pin pulled low externally (host-issued
    //                     reset over USB enumerates as a RUN reset on
    //                     many host stacks)
    //   HAD_PSM_RESTART — power-state-machine restart (debug, picotool)
    // Prior to this snapshot the only signal we had was "everything
    // else falls through to Boot OK with a generic brown-out hint",
    // which is wrong about half the time. We also capture the watchdog
    // REASON register to separate "watchdog timer fired" (TIMER) from
    // "we asked to reboot via watchdog_reboot()" (FORCE) — the latter
    // happens on our crash handler's clean exit path.
    uint32_t chip_reset_reg = vreg_and_chip_reset_hw->chip_reset;
    uint32_t watchdog_reason = watchdog_hw->reason;
    bool had_por           = (chip_reset_reg & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_POR_BITS)           != 0;
    bool had_run           = (chip_reset_reg & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_RUN_BITS)           != 0;
    bool had_psm_restart   = (chip_reset_reg & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_PSM_RESTART_BITS)   != 0;
    bool wdg_reason_timer  = (watchdog_reason & WATCHDOG_REASON_TIMER_BITS) != 0;
    bool wdg_reason_force  = (watchdog_reason & WATCHDOG_REASON_FORCE_BITS) != 0;

    // Capture the reset cause before anything else touches the
    // watchdog. watchdog_enable_caused_reboot() returns true only when
    // the *timeout* fired — not for the watchdog_reboot()-triggered
    // OTA hand-off — so this cleanly distinguishes "we crashed" from
    // "we asked to reboot." Used below as soon as the log channel is
    // up so the operator sees "Recovered from watchdog reset" right
    // after the previous boot's last log line in the Logs tab.
    bool boot_after_watchdog_timeout = watchdog_enable_caused_reboot();

    // Same idea for hard fault: noinit RAM survives the
    // watchdog_reboot() our fault handler triggers, so if magic ==
    // CRASH_MAGIC the previous boot ended in isr_hardfault and the PC
    // / LR captured there point at the faulting instruction and its
    // caller. Snapshot them before clearing the sentinel so we only
    // report once per crash.
    bool boot_after_hardfault = (crash_info.magic == CRASH_MAGIC);
    uint32_t crashed_pc = boot_after_hardfault ? crash_info.pc : 0;
    uint32_t crashed_lr = boot_after_hardfault ? crash_info.lr : 0;
    if (boot_after_hardfault) {
        crash_info.magic = 0;
    }

    // Also capture the OTA-gave-up sentinel from the bootloader before
    // anything touches scratch[3]. If non-zero, the previous OTA
    // attempt exceeded its retry budget and we're running the
    // PREVIOUS firmware. scratch[2] holds the failure reason in the
    // low byte and an optional http response code in the high 16
    // bits — see OTA_FAIL_* in bootloader/main.c. Clear both so we
    // only report once.
    bool boot_after_ota_giveup = (watchdog_hw->scratch[3] == OTA_GAVE_UP_MAGIC);
    uint32_t ota_fail_word = boot_after_ota_giveup ? watchdog_hw->scratch[2] : 0;
    if (boot_after_ota_giveup) {
        watchdog_hw->scratch[3] = 0;
        watchdog_hw->scratch[2] = 0;
    }
    uint8_t ota_fail_reason = (uint8_t)(ota_fail_word & 0xFF);
    uint16_t ota_fail_http  = (uint16_t)((ota_fail_word >> 16) & 0xFFFF);

    // Resolve the bootloader version from its fixed-address descriptor
    // (or fall back to "unknown") before anything that might want to
    // log or announce it.
    resolve_bootloader_version();

    // Brief delay for UART to stabilize
    sleep_ms(100);

    // Print version immediately - this is the first output
    printf("\n\n");
    printf("****************************************\n");
    printf("* SAINT.OS Node Firmware\n");
    printf("* Version:    %s\n", FIRMWARE_VERSION_FULL);
    printf("* Bootloader: %s\n", g_bl_version);
    printf("* Built:      %s\n", FIRMWARE_BUILD_TIMESTAMP);
    printf("* Hardware:   %s\n", HARDWARE_MODEL);
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

    // Initialize pin control (after pin_config)
    pin_control_init();

    // Register peripheral drivers BEFORE loading config from flash.
    // pin_config_load() walks the registered-drivers list and calls
    // each driver's load_config(&storage) so it can read its stored
    // pins/enabled flag. If we load first and register after, that
    // loop runs over zero drivers and the per-driver flash data is
    // silently discarded — leaving drivers in their default state
    // even after a reboot of a previously-configured node.
    peripheral_register(syren_get_peripheral_driver());
    peripheral_register(fas100_get_peripheral_driver());
    peripheral_register(roboclaw_get_peripheral_driver());
    peripheral_register(pathfinder_bms_get_peripheral_driver());

    // Load saved pin configuration (also fans out to each driver's
    // load_config callback).
    if (pin_config_load()) {
        printf("Loaded pin configuration from flash\n");
    }

    peripheral_init_all();

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
    // Discover the SAINT server via UDP broadcast. The server's
    // discovery responder is the Python saint_server process — separate
    // from dnsmasq — and at cold boot it can come up tens of seconds
    // after DHCP is already handing out leases. Retry forever with
    // backoff instead of stranding the node in ERROR; this mirrors the
    // DHCP retry-forever pattern in transport_w5500_connect(). LED was
    // last set to NODE_STATE_CONNECTING (yellow) by transport_connect,
    // and we keep led_update() ticking inside the backoff so the
    // operator sees the node is still trying.
    printf("Discovering SAINT server...\n");
    uint32_t disc_batch = 0;
    while (!discover_server(g_node.server_ip, &g_node.server_port, 2000, 10)) {
        disc_batch++;
        uint32_t backoff_ms = (disc_batch < 3) ? 2000 : 5000;
        printf("Discovery batch %lu failed; retrying in %lu ms "
               "(server not up yet?)\n",
               (unsigned long)disc_batch, (unsigned long)backoff_ms);
        for (uint32_t slept = 0; slept < backoff_ms; slept += 100) {
            sleep_ms(100);
            led_update();
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

    // Now that the log publisher is alive, queue the boot context.
    // These lines fire BEFORE the first announcement goes out, so the
    // server hasn't yet created its per-node log subscriber and a
    // direct saint_log_publish() would be dropped. Stage them via
    // boot_log_queue(); the announce-timer callback replays them once
    // the subscription is established (see boot_log_flush_if_due).
    if (boot_after_hardfault) {
        // A captured hard fault is more specific than "the watchdog
        // fired" — log it first regardless of the watchdog flag, since
        // our handler triggers a watchdog reboot to clean things up.
        // Operator workflow: copy the PC into addr2line against the
        // matching .elf to get file:line:
        //   arm-none-eabi-addr2line -e build/saint_node.elf 0xXXXXXXXX
        boot_log_queue("error",
            "Recovered from hard fault: PC=0x%08lX LR=0x%08lX "
            "(addr2line -e build/saint_node.elf 0x%08lX)",
            (unsigned long)crashed_pc, (unsigned long)crashed_lr,
            (unsigned long)crashed_pc);
    } else if (boot_after_watchdog_timeout) {
        // Watchdog timeout WITHOUT a captured hard fault: the main
        // loop got stuck somewhere that wasn't a CPU exception
        // (deadlock, infinite loop, blocked I/O), so there's no PC to
        // report. Operator's clue: whatever was being commanded right
        // before the reset is the suspect.
        boot_log_queue("error",
            "Recovered from watchdog reset — main loop hung "
            "(no hard-fault PC captured)");
    } else {
        // Neither hard fault nor watchdog timeout. Use the chip-reset
        // reason bits we snapshotted at the top of main() to pin down
        // which path the silicon took — HAD_POR vs HAD_RUN vs
        // HAD_PSM_RESTART. This replaces the previous "suspect
        // brown-out" guess with an actual answer:
        //   HAD_RUN          → RUN pin reset (commonly: USB host
        //                      enumeration kicked the device; check
        //                      `dmesg -w | grep usb` on the host)
        //   HAD_PSM_RESTART  → debugger / picotool / cold reset chain
        //   HAD_POR alone    → POR or brown-out (still ambiguous in HW)
        //   nothing          → reset reason latch was cleared before
        //                      we could read it (shouldn't happen)
        const char* reset_label;
        if (had_run && !had_por) {
            reset_label = "RUN-pin reset (USB host re-enum?)";
        } else if (had_psm_restart && !had_por) {
            reset_label = "PSM restart (debugger or cold-reset chain)";
        } else if (had_por) {
            reset_label = "POR/brown-out";
        } else {
            reset_label = "unknown (no chip-reset bits set)";
        }
        boot_log_queue("info",
            "Boot OK — fw %s, bl %s, watchdog armed at %d ms · "
            "reset cause: %s [CHIP_RESET=0x%08lX, WATCHDOG.REASON=0x%lX%s%s]",
            FIRMWARE_VERSION_FULL, g_bl_version, WATCHDOG_TIMEOUT_MS,
            reset_label,
            (unsigned long)chip_reset_reg, (unsigned long)watchdog_reason,
            wdg_reason_timer ? " TIMER" : "",
            wdg_reason_force ? " FORCE" : "");
    }
    // The bootloader sets the OTA-gave-up sentinel when it exhausts
    // its retry budget and falls back to this app. Tell the operator
    // so they don't think the firmware update "just disappeared." The
    // reason byte tells them which stage failed — without it they'd
    // have to plug in a serial console to see the bootloader's UART
    // output.
    if (boot_after_ota_giveup) {
        const char* reason = ota_fail_reason_str(ota_fail_reason);
        if (ota_fail_reason == 5 /* HTTP_STATUS */ && ota_fail_http) {
            boot_log_queue("error",
                "OTA failed after retries: %s (HTTP %u). "
                "Running previous firmware.",
                reason, (unsigned)ota_fail_http);
        } else {
            boot_log_queue("error",
                "OTA failed after retries: %s. Running previous firmware.",
                reason);
        }
    }

    // Check if we have a saved configuration (previously adopted)
    if (pin_config_has_configured_pins()) {
        // Node was previously adopted and has saved config
        node_set_state(NODE_STATE_ACTIVE);
        led_set_state(NODE_STATE_ACTIVE);
        boot_log_queue("info", "Restored from saved config (ACTIVE)");
    } else {
        // No saved config - enter unadopted state
        node_set_state(NODE_STATE_UNADOPTED);
        led_set_state(NODE_STATE_UNADOPTED);
        boot_log_queue("info", "Waiting for adoption (UNADOPTED)");
    }
    printf("========================================\n");

    // Initialize connection tracking
    last_successful_comm = to_ms_since_boot(get_absolute_time());
    agent_connected = true;

    // Arm the hardware watchdog only when WATCHDOG_TIMEOUT_MS > 0. The
    // RP2040 hardware caps at ~8.4s, which is shorter than legitimate
    // slow operations (W5500 send retransmit, micro-ROS reinit). With
    // it disabled, the DHCP-keepalive + NODE_STATE_ERROR recovery in
    // check_agent_connection still pull the node back from agent
    // disruptions; only a true main-loop hang would be uncovered, and
    // those should be diagnosed not papered over.
    if (WATCHDOG_TIMEOUT_MS > 0) {
        watchdog_enable(WATCHDOG_TIMEOUT_MS, 1);
        printf("Watchdog armed (%lu ms timeout)\n", (unsigned long)WATCHDOG_TIMEOUT_MS);
    } else {
        printf("Watchdog disabled (WATCHDOG_TIMEOUT_MS=0)\n");
    }

    // Main loop
    static uint32_t last_status_print = 0;
    while (1) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // Pet the watchdog every iteration. Any wedge longer than
        // WATCHDOG_TIMEOUT_MS triggers a clean reset.
        watchdog_update();

        // Update node state
        node_state_update();
        g_node.uptime_ms = now;

        // Update LED
        led_update();

        // Poll peripheral drivers
        peripheral_update_all();

        // Pump DHCP — lease half-time renewal lives here. Without this
        // the W5500 ioLibrary never advances dhcp_tick_1s and never
        // notices when the lease is half over. SIMULATION builds use
        // the UDP bridge transport instead, which doesn't need ticking.
#ifndef SIMULATION
        transport_w5500_tick();
#endif

        // Check agent connection (and ERROR-state recovery).
        if (!check_agent_connection()) {
            // Reconnecting or in ERROR-recovery cooldown — skip normal
            // processing this iteration. Watchdog pet above already.
            sleep_ms(100);
            continue;
        }

        // Spin micro-ROS executor (process timers and callbacks)
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        // Periodic status print (every 10 seconds)
        if (now - last_status_print >= 10000) {
            printf("[%lu] Node running, state: %s, agent: %s\n",
                   now / 1000, node_state_to_string(g_node.state),
                   agent_connected ? "connected" : "disconnected");
            last_status_print = now;
        }

        // Small delay
        sleep_ms(10);
    }

    // Cleanup (never reached)
    cleanup_micro_ros();

    return 0;
}
