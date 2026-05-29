/**
 * SAINT.OS shared remote-logging API.
 *
 * saint_log_publish(level, fmt, ...) ships a single line over the
 * /saint/nodes/<id>/log topic AND prints to the local console
 * (UART / Serial). The server subscribes per adopted node and routes
 * incoming entries into the per-node Logs tab.
 *
 * Buffering: until per-platform main calls saint_log_set_ros_ready(true),
 * each call is enqueued in a small RAM buffer. The announce-timer
 * callback eventually calls saint_log_drain_boot_queue() once the
 * server has had a chance to subscribe to the /log topic — and that
 * replays the buffered lines as real ROS publishes. Without this, the
 * early-boot lines (peripheral init logs, OTA messages, etc.) would
 * publish before the server-side subscriber exists and silently drop.
 *
 * Per-platform contract:
 *   - saint_log_emit_local(level, text) prints to stdout/Serial; the
 *     core saint_log_publish() always calls it even if ROS isn't ready.
 *   - saint_log_emit_ros(json, len) hands a formatted JSON envelope to
 *     rcl_publish on the platform's log publisher. Only called once
 *     saint_log_set_ros_ready(true) has been signalled.
 *   - saint_log_uptime_ms() returns ms-since-boot for the envelope.
 *
 * Best-effort throughout. Don't call from ISRs and keep lines under
 * ~200 chars. Levels match the server's: "info", "warn", "error".
 */
#ifndef SAINT_LOG_H
#define SAINT_LOG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Public log API ────────────────────────────────────────────────── */

void saint_log_publish(const char* level, const char* fmt, ...);

/* Mirror of saint_log_publish that ALWAYS buffers (regardless of
 * ros_ready) — for the small number of pre-init call sites that want to
 * defer log lines explicitly until the server is listening. */
void saint_log_boot_queue(const char* level, const char* fmt, ...);

/* Flip the publish path from buffer to ROS once the log publisher is
 * attached to the agent. After this returns true, saint_log_publish()
 * sends synchronously via saint_log_emit_ros(). The boot queue is NOT
 * drained yet — drain timing is controlled separately so the server
 * can subscribe first (see saint_log_drain_boot_queue). */
void saint_log_set_ros_ready(bool ready);

/* Replay any buffered lines through saint_log_publish, then mark the
 * buffer as flushed. Idempotent. Call from the announce-tick callback
 * once ≥2 announcements have gone out (the server creates per-node
 * /log subscriptions lazily on first /announce). */
void saint_log_drain_boot_queue(void);

/* ── Per-platform hooks (implemented in <platform>/src/main.*) ─────── */

/* Print one fully-formatted line locally (UART / Serial). The shared
 * code passes in the level and post-vsnprintf text — implementation
 * decides the prefix format. */
void saint_log_emit_local(const char* level, const char* text);

/* Hand a JSON envelope to rcl_publish on the platform's log publisher.
 * Return true on success, false if the publish failed (the shared code
 * doesn't currently re-buffer on failure — log is best-effort). */
bool saint_log_emit_ros(const char* json, size_t len);

/* ms-since-boot — used in the JSON envelope's uptime_ms field. */
uint32_t saint_log_uptime_ms(void);

#ifdef __cplusplus
}
#endif

#endif // SAINT_LOG_H
