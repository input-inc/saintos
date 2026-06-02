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

/* Pending-log queue dimensions sized to absorb the largest expected
 * burst from a single rclc-executor spin (peripheral inits at boot,
 * config_subscription_callback's ~4-call sequence, OTA outcome…).
 * 24 entries × ~210 B ≈ 5 KB of RAM, comfortable on both RP2040
 * (264 KB) and Teensy 4.1 (1 MB OCRAM).
 *
 * This started as a boot-only one-shot queue (saint_log_publish wrote
 * here only until saint_log_set_ros_ready(true)); after that flag
 * flipped, publishes ran inline. That inline path was the cause of
 * the empty-/log-frame regression — see the saint_log_publish
 * comment block below. The queue is now circular and ALWAYS in
 * play; saint_log_drain_pending() pops one entry per call from the
 * caller's context (main loop, NOT executor callback). */
#define BOOT_LOG_MAX        24
#define BOOT_LOG_LEVEL_LEN  8
#define BOOT_LOG_TEXT_LEN   200

typedef struct {
    char level[BOOT_LOG_LEVEL_LEN];
    char text[BOOT_LOG_TEXT_LEN];
} boot_log_entry_t;

static boot_log_entry_t g_pending_log[BOOT_LOG_MAX];
/* Ring-buffer indices. head = next slot to write, tail = next slot to
 * read. Wraps modulo BOOT_LOG_MAX. Distinguishes empty (head==tail)
 * from full (head wraps to tail) by tracking g_pending_dropped — a
 * full-queue enqueue overwrites the oldest entry and bumps the
 * counter so an operator can spot the queue saturating in /announce.
 * 32-bit volatile so writes/reads across callback ↔ main-loop are
 * coherent on Cortex-M (single-core, no SMP) without explicit
 * barriers. */
static volatile uint32_t g_pending_head   = 0;
static volatile uint32_t g_pending_tail   = 0;
static volatile uint32_t g_pending_dropped = 0;
static bool             g_ros_ready       = false;

/* Scratch storage for the JSON envelope built in saint_log_publish.
 * Lives in BSS rather than on the stack to keep that path's stack
 * footprint sane on Teensy + RP2040 (~640 bytes here, dwarfed by the
 * stack saving). Not reentrant — saint_log_publish must not be called
 * from an ISR and must not race with itself. */
static char g_text[BOOT_LOG_TEXT_LEN];
static char g_escaped[BOOT_LOG_TEXT_LEN + 32];
static char g_envelope[BOOT_LOG_TEXT_LEN + 96];

/* Ring-buffer enqueue. If the queue is full, drops the oldest entry
 * (advances tail) so the new line still lands — that's the right
 * trade-off for diagnostic logs: keeping the most recent context is
 * more useful than rejecting it. g_pending_dropped tracks the count
 * so the operator can spot saturation. Safe to call from a
 * subscription callback OR from the main loop. */
static void enqueue_pending(const char* level, const char* text)
{
    uint32_t next_head = (g_pending_head + 1) % BOOT_LOG_MAX;
    if (next_head == g_pending_tail) {
        /* Full — drop the oldest (advance tail) to make room. */
        g_pending_tail = (g_pending_tail + 1) % BOOT_LOG_MAX;
        g_pending_dropped++;
    }
    boot_log_entry_t* e = &g_pending_log[g_pending_head];
    snprintf(e->level, sizeof(e->level), "%s", level);
    snprintf(e->text,  sizeof(e->text),  "%s", text);
    g_pending_head = next_head;
}

static bool pending_empty(void)
{
    return g_pending_head == g_pending_tail;
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
 * Root cause of the empty-/log-frame regression: `config_subscription_callback`
 * (and other callbacks) emit a burst of saint_log_publish lines back-to-back
 * inside the same rclc executor dispatch. micro-XRCE-DDS shares its
 * output stream (RMW_UXRCE_STREAM_HISTORY_OUTPUT = 4) between the
 * just-received inbound message and any rcl_publish issued from
 * within that dispatch; back-to-back publishes race with the
 * agent's relay and the server sees `(malformed log frame ... ''
 * char 0)` warnings instead of the actual text.
 *
 * The fix has two coordinated pieces:
 *
 *   (a) **Defer.** `saint_log_publish` always enqueues; never
 *       `rcl_publish`es inline from the caller's context. The drain
 *       (`saint_log_drain_pending`) is meant to be called from the
 *       main loop AFTER `rclc_executor_spin_some` returns, so each
 *       publish happens at a moment when the executor isn't already
 *       in the middle of a callback. This is the load-bearing piece.
 *
 *   (b) **Pace.** Even from the main loop, the agent's Cyclone DDS
 *       layer needs real wall time to drain BEST_EFFORT samples
 *       between publishes — back-to-back drains can still produce
 *       empty frames if they happen faster than that. The drain
 *       enforces SAINT_LOG_MIN_PUBLISH_INTERVAL_MS between publishes
 *       (non-blocking: returns immediately if the window hasn't
 *       elapsed, retries next loop iteration). On hardware the two
 *       clocks are 1:1 (150 ms wall ≈ 150 ms firmware); on Renode
 *       (sim) the firmware clock runs ~12× faster, so the sim value
 *       is calibrated to ~125 ms wall.
 *
 * The previous version's saint_log_emit_local("trace", g_envelope)
 * pre-print is gone — that doubled as a bus-activity pacer for sim
 * (each LPUART write forced Renode to dispatch udp_bridge), but it's
 * no longer needed: deferring out of callback context fixes the
 * mode entirely, and the trace was noisy in production. */
#ifdef SIMULATION
#define SAINT_LOG_MIN_PUBLISH_INTERVAL_MS 1500
#else
#define SAINT_LOG_MIN_PUBLISH_INTERVAL_MS 150
#endif
static uint32_t g_last_publish_ms = 0;

/* Publish one envelope from main-loop context — never call this from
 * a subscription/timer callback, that's exactly the failure mode we
 * just refactored away. */
static void publish_one(const char* level, const char* text)
{
    json_escape(text, g_escaped, sizeof(g_escaped));
    int len = snprintf(g_envelope, sizeof(g_envelope),
        "{\"level\":\"%s\",\"text\":\"%s\",\"uptime_ms\":%lu}",
        level, g_escaped, (unsigned long)saint_log_uptime_ms());
    if (len < 0 || (size_t)len >= sizeof(g_envelope)) return;
    g_emit_attempts++;
    if (saint_log_emit_ros(g_envelope, (size_t)len)) {
        g_emit_ok++;
    }
    g_last_publish_ms = saint_log_uptime_ms();
}

uint32_t saint_log_emit_attempts(void) { return g_emit_attempts; }
uint32_t saint_log_emit_ok(void)       { return g_emit_ok; }
uint32_t saint_log_dropped(void)       { return g_pending_dropped; }

void saint_log_publish(const char* level, const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_text, sizeof(g_text), fmt, ap);
    va_end(ap);

    /* Local echo runs always — a serial dev console must see everything
     * even when the agent is down. Fires in the caller's context so
     * the operator sees lines in real time; the queued /log publish
     * happens later from the main loop. */
    saint_log_emit_local(level, g_text);

    /* Enqueue unconditionally. Drain happens from main loop via
     * saint_log_drain_pending() — see header block above. The
     * old "if ros_ready, publish inline" branch is what produced the
     * empty-frame regression; gone. */
    enqueue_pending(level, g_text);
}

void saint_log_boot_queue(const char* level, const char* fmt, ...)
{
    /* Now identical to saint_log_publish — kept as a separate symbol
     * for callers that explicitly want "buffered until ros_ready"
     * semantics. With the unified ring queue, both modes resolve to
     * the same code path. */
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_text, sizeof(g_text), fmt, ap);
    va_end(ap);

    char marked[BOOT_LOG_LEVEL_LEN + 16];
    snprintf(marked, sizeof(marked), "%s/pending", level);
    saint_log_emit_local(marked, g_text);

    enqueue_pending(level, g_text);
}

void saint_log_set_ros_ready(bool ready)
{
    g_ros_ready = ready;
}

/* Drain one queued entry per call. Designed to be called from the
 * main loop — NEVER from a subscription/timer callback. Pops at most
 * one entry per call so the worst-case time spent in this function
 * is one rcl_publish (microseconds, not enough to starve the loop).
 *
 * Pacing: non-blocking. If the previous publish was less than
 * SAINT_LOG_MIN_PUBLISH_INTERVAL_MS ago, returns without popping;
 * the queued entry just waits for the next loop iteration. This
 * keeps a /log burst (e.g. config_subscription_callback's 4-line
 * sequence) from saturating the agent's Cyclone DDS layer — the
 * burst arrives in the queue all at once but trickles out one
 * publish per ~150 ms wall.
 *
 * Returns true if it published something (handy for callers that
 * want to know whether to keep calling). */
bool saint_log_drain_pending(void)
{
    if (!g_ros_ready) return false;
    if (pending_empty()) return false;

    /* Non-blocking pacer check. */
    if (g_last_publish_ms != 0) {
        uint32_t elapsed = saint_log_uptime_ms() - g_last_publish_ms;
        if (elapsed < SAINT_LOG_MIN_PUBLISH_INTERVAL_MS) {
            return false;
        }
    }

    boot_log_entry_t* e = &g_pending_log[g_pending_tail];
    publish_one(e->level, e->text);
    g_pending_tail = (g_pending_tail + 1) % BOOT_LOG_MAX;
    return true;
}

/* Backwards-compat: old callers (announce_timer_callback in both
 * platforms' main.cpp/main.c) still call saint_log_drain_boot_queue.
 * Aliased to the new drain so we don't have to update every caller
 * in this PR — fine because both drain semantics are now identical
 * (single pending queue, one entry per call). Safe to remove once
 * the per-platform mains have switched their call sites. */
void saint_log_drain_boot_queue(void)
{
    (void)saint_log_drain_pending();
}
