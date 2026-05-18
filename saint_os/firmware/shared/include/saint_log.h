/**
 * SAINT.OS shared remote-logging API.
 *
 * Per-platform `saint_log_publish(level, fmt, ...)` ships a single
 * line over the /saint/nodes/<id>/log topic AND prints to the local
 * console (UART / Serial). The server subscribes per adopted node
 * and routes incoming entries into the per-node Logs tab.
 *
 * Best-effort: if the agent isn't connected yet (or fails to publish),
 * the printf still runs so a serial dev console sees everything. Don't
 * call from ISRs and keep lines under ~200 chars.
 *
 * Levels match the server's: "info", "warn", "error".
 */
#ifndef SAINT_LOG_H
#define SAINT_LOG_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void saint_log_publish(const char* level, const char* fmt, ...);

#ifdef __cplusplus
}
#endif

#endif // SAINT_LOG_H
