/**
 * SAINT.OS Firmware — control message parsing (shared).
 *
 * See control_message.h. Pointer-only JSON field scanning — no
 * allocation, no full parser — matching the hand-rolled extraction the
 * per-platform pin_control files used, unified here. Strict JSON string
 * values (require the opening quote) — the server always quotes string
 * fields, and requiring it avoids the looser RP2040 variant silently
 * accepting malformed input.
 */
#include "control_message.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

/* Extract a quoted string field's value into `out` (NUL-terminated,
 * capped at out_size). Returns true iff the key was found AND its value
 * is a non-empty quoted string. */
static bool extract_str_field(const char* json, const char* key,
                              char* out, size_t out_size)
{
    out[0] = '\0';
    const char* p = strstr(json, key);
    if (!p) return false;
    p = strchr(p, ':');
    if (!p) return false;
    p++;
    while (*p == ' ' || *p == '\t') p++;
    if (*p != '"') return false;     /* require a JSON string value */
    p++;
    size_t n = 0;
    while (*p && *p != '"' && n + 1 < out_size) {
        out[n++] = *p++;
    }
    out[n] = '\0';
    return n > 0;
}

/* Find a bare numeric field value (no quotes): seek `key`, the ':',
 * skip spaces, and point at the number. Returns NULL if absent. */
static const char* find_number_field(const char* json, const char* key)
{
    const char* p = strstr(json, key);
    if (!p) return NULL;
    p = strchr(p, ':');
    if (!p) return NULL;
    p++;
    while (*p == ' ' || *p == '\t') p++;
    return p;
}

control_parse_result_t control_parse_set_channel(const char* json,
                                                 control_set_channel_t* out)
{
    memset(out, 0, sizeof(*out));
    out->us = -1;

    if (!extract_str_field(json, "\"peripheral\"", out->peripheral,
                           sizeof(out->peripheral))) {
        return CONTROL_PARSE_NO_PERIPHERAL;
    }
    if (!extract_str_field(json, "\"channel\"", out->channel,
                           sizeof(out->channel))) {
        return CONTROL_PARSE_NO_CHANNEL;
    }

    /* type is optional — absent leaves out->type == "". */
    extract_str_field(json, "\"type\"", out->type, sizeof(out->type));

    /* us is optional (Maestro extent-dial preview, absolute µs). When
     * present, value may be omitted. Keyed as the quoted "us" so it
     * can't accidentally match inside another key. */
    const char* us_str = find_number_field(json, "\"us\"");
    if (us_str) {
        out->has_us = true;
        out->us = atol(us_str);
    }

    const char* value_str = find_number_field(json, "\"value\"");
    if (value_str) {
        out->has_value = true;
        out->value = (float)atof(value_str);
    }

    if (!out->has_value && !out->has_us) {
        return CONTROL_PARSE_NO_VALUE_OR_US;
    }
    return CONTROL_PARSE_OK;
}
