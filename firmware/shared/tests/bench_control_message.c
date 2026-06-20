/**
 * Benchmark: shared control-message parser (hop 5, platform-independent
 * core of the ROS-message → peripheral path).
 *
 * Times control_parse_set_channel over a realistic streaming
 * set_channel message — the per-/control-tick decode the firmware does
 * before routing to a driver. The pin_config lookup + hardware dispatch
 * that follow are platform-bound (they touch each MCU's pin_config
 * storage + hardware) and aren't measured here.
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "bench_harness.h"
#include "../src/control_message.c"

int main(void)
{
    /* Realistic streaming control message (no type/us — the common
     * gamepad-axis case the server publishes per tick). */
    static const char* MSGS[] = {
        "{\"peripheral\":\"maestro-1\",\"channel\":\"ch3\",\"value\":0.42}",
        "{\"peripheral\":\"roboclaw-1\",\"channel\":\"motor\",\"value\":-0.8}",
        "{\"peripheral\":\"maestro-1\",\"channel\":\"ch0\",\"type\":\"maestro\",\"us\":1750}",
    };
    enum { NMSG = 3 };

    printf("\nbench_control_message — set_channel parse (ROS msg -> struct)\n");

    control_set_channel_t c;
    BENCH("control_parse_set_channel", 2000000, {
        control_parse_result_t r = control_parse_set_channel(MSGS[_i % NMSG], &c);
        BENCH_SINK((uint64_t)r + (uint64_t)c.peripheral[0]
                   + (uint64_t)(c.value * 1000.0f) + (uint64_t)c.us);
    });

    /* sanity: confirm the representative messages parse as expected. */
    control_parse_set_channel(MSGS[0], &c);
    if (control_parse_set_channel(MSGS[0], &c) != CONTROL_PARSE_OK) {
        fprintf(stderr, "FAIL: representative message did not parse\n");
        return 1;
    }
    printf("  sanity: '%s'/'%s' value=%.2f\n", c.peripheral, c.channel, (double)c.value);
    return 0;
}
