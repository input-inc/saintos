/**
 * SAINT.OS Firmware — control message parsing (shared).
 *
 * Platform-agnostic decode of the `set_channel` control message the
 * server publishes on each node's /control topic. This is the
 * platform-independent core of the "ROS message → peripheral" path:
 * the JSON string → a structured command. Routing the decoded command
 * to hardware (pin_config lookup, driver dispatch, NeoPixel override)
 * stays platform-specific because it's bound to each platform's
 * pin_config storage + hardware APIs.
 *
 * Previously this parse was copy-pasted into firmware/rp2040/src/
 * pin_control.c and firmware/teensy41/src/pin_control.cpp — and had
 * DIVERGED (Teensy parsed the `us` Maestro-preview field; RP2040 did
 * not). Sharing it dedups the two and fixes that drift.
 *
 * Wire format (all on one JSON object):
 *   {"action":"set_channel","peripheral":"<id>","channel":"<ch>",
 *    "value":<float>[, "type":"<catalog type>"][, "us":<int>]}
 *   - peripheral, channel: required.
 *   - value: the normalized −1..+1 (or channel-specific) setpoint.
 *   - type: optional catalog type tag (e.g. "neopixel") for routing
 *     peripherals that aren't in pin_config.
 *   - us: optional absolute-microseconds Maestro extent-dial preview;
 *     when present, value may be omitted.
 *   At least one of value / us must be present.
 */
#ifndef SAINT_CONTROL_MESSAGE_H
#define SAINT_CONTROL_MESSAGE_H

#include <stdbool.h>
#include "pin_types.h"   /* PIN_CONFIG_MAX_NAME_LEN */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char  peripheral[PIN_CONFIG_MAX_NAME_LEN];
    char  channel[PIN_CONFIG_MAX_NAME_LEN];
    char  type[PIN_CONFIG_MAX_NAME_LEN];   /* "" when absent */
    bool  has_value;
    float value;
    bool  has_us;
    long  us;
} control_set_channel_t;

typedef enum {
    CONTROL_PARSE_OK = 0,
    CONTROL_PARSE_NO_PERIPHERAL,    /* required "peripheral" missing */
    CONTROL_PARSE_NO_CHANNEL,       /* required "channel" missing */
    CONTROL_PARSE_NO_VALUE_OR_US,   /* neither "value" nor "us" present */
} control_parse_result_t;

/* Parse a set_channel control message. `out` is always fully
 * initialized (zeroed) before parsing, so the caller may inspect
 * partially-populated fields even on a non-OK result. Returns
 * CONTROL_PARSE_OK only when peripheral + channel + (value | us) are
 * all present; the specific reason otherwise so the caller can emit a
 * meaningful diagnostic. */
control_parse_result_t control_parse_set_channel(const char* json,
                                                 control_set_channel_t* out);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_CONTROL_MESSAGE_H */
