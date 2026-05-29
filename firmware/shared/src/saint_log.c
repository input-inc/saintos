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

static void publish_one(const char* level, const char* text)
{
    json_escape(text, g_escaped, sizeof(g_escaped));
    int len = snprintf(g_envelope, sizeof(g_envelope),
        "{\"level\":\"%s\",\"text\":\"%s\",\"uptime_ms\":%lu}",
        level, g_escaped, (unsigned long)saint_log_uptime_ms());
    if (len < 0 || (size_t)len >= sizeof(g_envelope)) return;
    (void)saint_log_emit_ros(g_envelope, (size_t)len);
}

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

void saint_log_drain_boot_queue(void)
{
    if (g_boot_log_flushed) return;
    if (!g_ros_ready)       return;
    for (size_t i = 0; i < g_boot_log_count; i++) {
        publish_one(g_boot_log[i].level, g_boot_log[i].text);
    }
    g_boot_log_flushed = true;
}
