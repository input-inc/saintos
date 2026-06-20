/**
 * Host-runnable test for the shared control-message parser.
 *
 * Pins the behavior of control_parse_set_channel so the firmware-side
 * "ROS message → structured command" decode (hop 5's platform-
 * independent core) can't regress when the RP2040/Teensy pin_control
 * files are rewired to use it. Covers the field set both platforms
 * relied on (peripheral/channel/type/value) PLUS the `us` Maestro-
 * preview field that only Teensy parsed before — sharing unifies them.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "../src/control_message.c"

static int fail_count = 0;
#define EXPECT(cond, label)                                                    \
    do {                                                                        \
        if (cond) printf("  ok   %s\n", label);                                 \
        else { printf("  FAIL %s\n", label); fail_count++; }                    \
    } while (0)

static void case_full_message(void)
{
    printf("case_full_message\n");
    control_set_channel_t c;
    const char* j = "{\"action\":\"set_channel\",\"peripheral\":\"maestro-1\","
                    "\"channel\":\"ch3\",\"type\":\"maestro\",\"value\":0.42}";
    control_parse_result_t r = control_parse_set_channel(j, &c);
    EXPECT(r == CONTROL_PARSE_OK,                 "parse OK");
    EXPECT(strcmp(c.peripheral, "maestro-1") == 0, "peripheral");
    EXPECT(strcmp(c.channel, "ch3") == 0,          "channel");
    EXPECT(strcmp(c.type, "maestro") == 0,         "type");
    EXPECT(c.has_value && fabsf(c.value - 0.42f) < 1e-6f, "value 0.42");
    EXPECT(!c.has_us,                              "no us");
}

static void case_type_optional(void)
{
    printf("case_type_optional\n");
    control_set_channel_t c;
    /* The server's streaming set_channel_value omits "type". */
    const char* j = "{\"peripheral\":\"roboclaw-1\",\"channel\":\"motor\",\"value\":-1}";
    control_parse_result_t r = control_parse_set_channel(j, &c);
    EXPECT(r == CONTROL_PARSE_OK,           "parse OK without type");
    EXPECT(c.type[0] == '\0',               "type empty when absent");
    EXPECT(c.has_value && c.value == -1.0f, "negative value");
}

static void case_us_preview(void)
{
    printf("case_us_preview\n");
    control_set_channel_t c;
    /* Extent-dial preview: us present, value omitted. (RP2040's old
     * copy dropped this — the shared parser handles it for both.) */
    const char* j = "{\"peripheral\":\"maestro-1\",\"channel\":\"ch0\",\"us\":1750}";
    control_parse_result_t r = control_parse_set_channel(j, &c);
    EXPECT(r == CONTROL_PARSE_OK,  "parse OK with us, no value");
    EXPECT(c.has_us && c.us == 1750, "us 1750");
    EXPECT(!c.has_value,           "value absent");
}

static void case_value_and_us(void)
{
    printf("case_value_and_us\n");
    control_set_channel_t c;
    const char* j = "{\"peripheral\":\"m\",\"channel\":\"ch0\",\"value\":0.1,\"us\":2000}";
    control_parse_result_t r = control_parse_set_channel(j, &c);
    EXPECT(r == CONTROL_PARSE_OK,       "both present OK");
    EXPECT(c.has_value && c.has_us,     "both flagged");
    EXPECT(c.us == 2000,                "us 2000");
    EXPECT(fabsf(c.value - 0.1f) < 1e-6f, "value 0.1");
}

static void case_missing_required(void)
{
    printf("case_missing_required\n");
    control_set_channel_t c;
    EXPECT(control_parse_set_channel(
               "{\"channel\":\"ch0\",\"value\":0}", &c) == CONTROL_PARSE_NO_PERIPHERAL,
           "missing peripheral -> NO_PERIPHERAL");
    EXPECT(control_parse_set_channel(
               "{\"peripheral\":\"m\",\"value\":0}", &c) == CONTROL_PARSE_NO_CHANNEL,
           "missing channel -> NO_CHANNEL");
    EXPECT(control_parse_set_channel(
               "{\"peripheral\":\"m\",\"channel\":\"ch0\"}", &c) == CONTROL_PARSE_NO_VALUE_OR_US,
           "no value/us -> NO_VALUE_OR_US");
}

static void case_whitespace_and_order(void)
{
    printf("case_whitespace_and_order\n");
    control_set_channel_t c;
    /* Reordered fields + spaces after colons must still parse. */
    const char* j = "{ \"value\" : 0.75 , \"channel\" : \"left\" , "
                    "\"peripheral\" : \"syren-1\" }";
    control_parse_result_t r = control_parse_set_channel(j, &c);
    EXPECT(r == CONTROL_PARSE_OK,                "reordered + spaced parses");
    EXPECT(strcmp(c.peripheral, "syren-1") == 0, "peripheral");
    EXPECT(strcmp(c.channel, "left") == 0,       "channel");
    EXPECT(fabsf(c.value - 0.75f) < 1e-6f,       "value 0.75");
}

static void case_neopixel_packed_color(void)
{
    printf("case_neopixel_packed_color\n");
    control_set_channel_t c;
    /* The State tab packs #RRGGBB into a uint24 carried as a float. The
     * parser just decodes the number; routing is platform-side. */
    const char* j = "{\"peripheral\":\"onboard_neopixel\",\"type\":\"neopixel\","
                    "\"channel\":\"color\",\"value\":16711680}";  /* 0xFF0000 */
    control_parse_result_t r = control_parse_set_channel(j, &c);
    EXPECT(r == CONTROL_PARSE_OK,                "parse OK");
    EXPECT(strcmp(c.type, "neopixel") == 0,      "type neopixel");
    EXPECT((uint32_t)c.value == 0xFF0000u,       "packed color round-trips through float");
}

int main(void)
{
    case_full_message();
    case_type_optional();
    case_us_preview();
    case_value_and_us();
    case_missing_required();
    case_whitespace_and_order();
    case_neopixel_packed_color();

    if (fail_count) {
        printf("\ntest_control_message: %d failure(s)\n", fail_count);
        return 1;
    }
    printf("\ntest_control_message: OK\n");
    return 0;
}
