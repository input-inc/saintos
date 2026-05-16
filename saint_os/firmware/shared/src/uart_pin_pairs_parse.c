/**
 * SAINT.OS Firmware - UART Pin Pair JSON Parsing (shared)
 *
 * Platform-agnostic JSON snippet parser used by every UART peripheral
 * driver's parse_json_params callback. Keeps the parsing rules in one
 * place so all drivers behave identically.
 */

#include "uart_pin_pairs.h"

#include <stdlib.h>
#include <string.h>

static int extract_uint8_field(const char* json_start, const char* json_end,
                               const char* key, uint8_t* out)
{
    const char* p = strstr(json_start, key);
    if (!p || p >= json_end) return 0;

    p = strchr(p, ':');
    if (!p || p >= json_end) return 0;

    p++;
    while (p < json_end && (*p == ' ' || *p == '\t')) p++;
    *out = (uint8_t)atoi(p);
    return 1;
}

bool uart_pin_pair_parse_json(const char* json_start, const char* json_end,
                               uint8_t* out_tx, uint8_t* out_rx,
                               uint8_t* out_instance)
{
    if (!json_start || !json_end || json_end <= json_start) return false;

    uint8_t tx = 0;
    uint8_t rx = 0;
    if (!extract_uint8_field(json_start, json_end, "\"tx_pin\"", &tx)) return false;
    if (!extract_uint8_field(json_start, json_end, "\"rx_pin\"", &rx)) return false;

    uint8_t inst;
    if (!uart_pin_pair_lookup(tx, rx, &inst)) return false;

    if (out_tx) *out_tx = tx;
    if (out_rx) *out_rx = rx;
    if (out_instance) *out_instance = inst;
    return true;
}
