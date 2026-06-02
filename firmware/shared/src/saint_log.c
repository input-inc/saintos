/**
 * SAINT.OS shared remote-logging implementation.
 *
 * Owns the boot-log queue, the JSON envelope assembly, and the
 * pre-ROS-buffer vs publish-now routing. Per-platform main provides
 * three hooks: saint_log_emit_local (printf/Serial), saint_log_emit_ros
 * (rcl_publish), and saint_log_uptime_ms.
 */

#include "saint_log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* PLATFORM_SLEEP_MS — on Teensy resolves to Arduino delay() (which
 * calls yield() in a loop, giving USB/event handlers cycles to drain);
 * on RP2040 to pico-sdk sleep_ms (busy-wait, but still advances real
 * time so peripheral models tick). Used by the inter-publish pacer
 * below to ensure XRCE-DDS output streams actually flush between
 * back-to-back /log emits. */
#include "platform.h"

/* Boot-log dimensions sized to fit every early-boot log line a node
 * is expected to emit before the second /announce goes out — driver
 * init traces, recovered-from-watchdog, OTA outcome, etc. Bumped from
 * the RP2040's original 5 to 24 once saint_log_publish started feeding
 * the same queue; 24 × ~210 B ≈ 5 KB of RAM, comfortable on both
 * RP2040 (264 KB) and Teensy 4.1 (1 MB OCRAM). */
#define BOOT_LOG_MAX        24
#define BOOT_LOG_LEVEL_LEN  8
#define BOOT_LOG_TEXT_LEN   200

typedef struct {
    char level[BOOT_LOG_LEVEL_LEN];
    char text[BOOT_LOG_TEXT_LEN];
} boot_log_entry_t;

static boot_log_entry_t g_boot_log[BOOT_LOG_MAX];
static size_t           g_boot_log_count   = 0;
/* Index of the next boot entry to publish. Boot drain emits one entry
 * per saint_log_drain_boot_queue() call (caller is the 1Hz announce
 * timer) — see the function's comment for the writer-stream-overflow
 * reason this pacing exists. */
static size_t           g_boot_log_drain_index = 0;
static bool             g_boot_log_flushed = false;
static bool             g_ros_ready        = false;

/* Scratch storage for the JSON envelope built in saint_log_publish.
 * Lives in BSS rather than on the stack to keep that path's stack
 * footprint sane on Teensy + RP2040 (~640 bytes here, dwarfed by the
 * stack saving). Not reentrant — saint_log_publish must not be called
 * from an ISR and must not race with itself. */
static char g_text[BOOT_LOG_TEXT_LEN];
static char g_escaped[BOOT_LOG_TEXT_LEN + 32];
static char g_envelope[BOOT_LOG_TEXT_LEN + 96];

static void enqueue_boot(const char* level, const char* text)
{
    if (g_boot_log_count >= BOOT_LOG_MAX || g_boot_log_flushed) {
        return;
    }
    boot_log_entry_t* e = &g_boot_log[g_boot_log_count++];
    snprintf(e->level, sizeof(e->level), "%s", level);
    snprintf(e->text,  sizeof(e->text),  "%s", text);
}

/* JSON-escape `text` into `out` (capacity `cap`). Escapes the standard
 * set ("\\/", '"', '\') and the common control chars (\n \r \t),
 * passes through anything else, and falls back to \u00XX for other
 * 0x00–0x1F bytes. Without this the RoboClaw version-probe newline
 * (and any other embedded control char) breaks server-side
 * json.loads with "Invalid control character at line 1 column N". */
static void json_escape(const char* text, char* out, size_t cap)
{
    size_t ei = 0;
    for (size_t i = 0; text[i] && ei < cap - 7; i++) {
        unsigned char c = (unsigned char)text[i];
        if (c == '"' || c == '\\') {
            out[ei++] = '\\';
            out[ei++] = (char)c;
        } else if (c == '\n') {
            out[ei++] = '\\'; out[ei++] = 'n';
        } else if (c == '\r') {
            out[ei++] = '\\'; out[ei++] = 'r';
        } else if (c == '\t') {
            out[ei++] = '\\'; out[ei++] = 't';
        } else if (c < 0x20) {
            static const char hex[] = "0123456789abcdef";
            out[ei++] = '\\'; out[ei++] = 'u';
            out[ei++] = '0';  out[ei++] = '0';
            out[ei++] = hex[(c >> 4) & 0xF];
            out[ei++] = hex[c & 0xF];
        } else {
            out[ei++] = (char)c;
        }
    }
    out[ei] = '\0';
}

static uint32_t g_emit_attempts = 0;
static uint32_t g_emit_ok = 0;

/* Inter-publish pacer for the /log publisher.
 *
 * The problem: `config_subscription_callback` (and a few other call
 * sites) emit a burst of saint_log_publish lines back-to-back —
 * "Config received (N bytes)" / "Config applied OK" / "Config saved to
 * flash" — all inside the same executor-callback dispatch, so no
 * `rclc_executor_spin_some` runs between them. Under that pacing,
 * micro-XRCE-DDS's output stream (RMW_UXRCE_STREAM_HISTORY_OUTPUT = 4
 * slots) delivers BEST_EFFORT publishes to the agent with *empty
 * payloads* rather than dropping them — the server sees a sequence of
 * `(malformed log frame: Expecting value: line 1 column 1 (char 0)) ''`
 * warnings and never gets the actual text. RELIABLE wedges the stream
 * entirely (see the QoS comment on `log_pub` init in main.cpp), so the
 * project's pre-existing fix was to drop /log to BEST_EFFORT — but
 * that's what exposed the empty-frame mode.
 *
 * The fix here has two coordinated pieces:
 *
 *   (a) PLATFORM_SLEEP_MS-based minimum interval. Forces real wall time
 *       between publishes. On hardware the two clocks are 1:1 so 150 ms
 *       wall is enough for the agent to drain the previous publish. On
 *       Renode (sim) the firmware-clock runs ~12× wall, so 150 firmware-
 *       ms ≈ 12 wall-ms — necessary but not sufficient on its own.
 *
 *   (b) `saint_log_emit_local` mirror of the JSON envelope right before
 *       `saint_log_emit_ros`. This serves dual duty: it's a useful
 *       diagnostic of intended-bytes-on-wire when debugging /log issues,
 *       AND in Renode each byte of the local UART write generates an
 *       LPUART bus access that forces the sim to dispatch peripherals
 *       (including udp_bridge) for that slice of sim time. Without
 *       this dispatch tick, udp_bridge.cs's pending UDP send doesn't
 *       complete and the next rcl_publish overwrites its output-stream
 *       slot. The local echo's role as a *pacer* (not just a tracer)
 *       is load-bearing for sim — don't remove without verifying
 *       MODE=teensy_full still passes Phase 2.
 *
 * Tradeoff: a /log burst of 4 lines adds at most 4 × interval of
 * callback latency. On hardware that's 4 × 150 = 600 ms; well under
 * the agent's keep-alive timeout (~5 s) and inside the executor-spin
 * loop bound.
 *
 * Sim-vs-hardware: PLATFORM_SLEEP_MS uses firmware time (millis() →
 * SysTick), which Renode runs ~12× faster than wall clock. 150 ms
 * firmware-time ≈ 12 ms wall in sim — too short for the agent's
 * Cyclone DDS layer to drain BEST_EFFORT samples between publishes,
 * so the empty-frame symptom returns. Picking the threshold per
 * platform keeps hardware latency low without sacrificing sim
 * reliability. */
#ifdef SIMULATION
#define SAINT_LOG_MIN_PUBLISH_INTERVAL_MS 1500
#else
#define SAINT_LOG_MIN_PUBLISH_INTERVAL_MS 150
#endif
static uint32_t g_last_publish_ms = 0;

static void publish_one(const char* level, const char* text)
{
    json_escape(text, g_escaped, sizeof(g_escaped));
    int len = snprintf(g_envelope, sizeof(g_envelope),
        "{\"level\":\"%s\",\"text\":\"%s\",\"uptime_ms\":%lu}",
        level, g_escaped, (unsigned long)saint_log_uptime_ms());
    if (len < 0 || (size_t)len >= sizeof(g_envelope)) return;
    g_emit_attempts++;
    /* Time-based pacer — see header comment block. */
    if (g_last_publish_ms != 0) {
        uint32_t elapsed = saint_log_uptime_ms() - g_last_publish_ms;
        if (elapsed < SAINT_LOG_MIN_PUBLISH_INTERVAL_MS) {
            PLATFORM_SLEEP_MS(SAINT_LOG_MIN_PUBLISH_INTERVAL_MS - elapsed);
        }
    }
    /* Bus-activity pacer + intended-bytes trace — see (b) in the header
     * comment block. Load-bearing for Renode sim; harmless extra local
     * line on hardware. */
    saint_log_emit_local("trace", g_envelope);
    if (saint_log_emit_ros(g_envelope, (size_t)len)) {
        g_emit_ok++;
    }
    /* Post-publish settle so the just-published message has wall time to
     * traverse the agent's relay before we either return to the caller
     * (which might rcl_publish again) or update g_last_publish_ms. Covers
     * the case where the FIRST publish in a burst would otherwise race
     * out with no pacing in front of it. */
    PLATFORM_SLEEP_MS(SAINT_LOG_MIN_PUBLISH_INTERVAL_MS / 2);
    g_last_publish_ms = saint_log_uptime_ms();
}

uint32_t saint_log_emit_attempts(void) { return g_emit_attempts; }
uint32_t saint_log_emit_ok(void)       { return g_emit_ok; }

void saint_log_publish(const char* level, const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_text, sizeof(g_text), fmt, ap);
    va_end(ap);

    /* Local echo runs always — a serial dev console must see everything
     * even when the agent is down. */
    saint_log_emit_local(level, g_text);

    if (!g_ros_ready) {
        enqueue_boot(level, g_text);
        return;
    }

    publish_one(level, g_text);
}

void saint_log_boot_queue(const char* level, const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_text, sizeof(g_text), fmt, ap);
    va_end(ap);

    /* Local echo with the "(pending)" annotation so the serial console
     * makes it obvious these are buffered and not yet on the wire. */
    char marked[BOOT_LOG_LEVEL_LEN + 16];
    snprintf(marked, sizeof(marked), "%s/pending", level);
    saint_log_emit_local(marked, g_text);

    enqueue_boot(level, g_text);
}

void saint_log_set_ros_ready(bool ready)
{
    g_ros_ready = ready;
}

/* Drain one buffered boot entry per call. Caller (announce_timer at 1Hz)
 * loops us until g_boot_log_flushed is set.
 *
 * Why one-at-a-time: micro-XRCE-DDS sizes the client output stream at
 * RMW_UXRCE_STREAM_HISTORY_OUTPUT = 4 unacked RELIABLE samples. The old
 * implementation drained the whole queue (up to 24 entries) in a tight
 * loop with no executor spin between publishes — by message 5 the
 * stream was full and RELIABLE rcl_publish either blocked, timed out
 * (RMW_UXRCE_PUBLISH_RELIABLE_TIMEOUT = 1000 ms × 19 leftover messages
 * = potential 19 s stall inside a timer callback, well past our 8 s
 * watchdog), or returned an error we discarded. After the drain
 * finished, the writer's stream was stuck in a permanent
 * waiting-for-ACK state and subsequent post-boot saint_log_publish
 * calls silently no-op'd. Externally that looks like "/log went dead
 * after the boot lines, sync ACK never reaches the server, UI pill
 * stays pending forever" — see docs/SYNC_CONFIG_REGRESSION.md.
 *
 * 1 Hz pacing gives the executor plenty of cycles to spin the
 * micro-XRCE-DDS session, deliver the message, and collect the ACK
 * before the next drain. Worst-case boot-log visibility is delayed by
 * BOOT_LOG_MAX (24) seconds, which is fine — these are diagnostic
 * lines, not anything time-critical. */
void saint_log_drain_boot_queue(void)
{
    if (g_boot_log_flushed) return;
    if (!g_ros_ready)       return;

    if (g_boot_log_drain_index < g_boot_log_count) {
        boot_log_entry_t* e = &g_boot_log[g_boot_log_drain_index];
        publish_one(e->level, e->text);
        g_boot_log_drain_index++;
    }

    if (g_boot_log_drain_index >= g_boot_log_count) {
        g_boot_log_flushed = true;
    }
}
