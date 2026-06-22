/**
 * SAINT.OS Firmware - Dimension Engineering Kangaroo X2 driver (shared)
 *
 * Drives up to 8 Kangaroo motor channels over packet OR simplified
 * serial (selected per channel by the operator). Closed-loop: each
 * channel takes position / speed setpoints and reports back current
 * position, current speed, a "moving" (busy) flag and the last error.
 *
 * Platform-agnostic: protocol byte assembly lives in
 * shared/include/kangaroo_protocol.h, channel state + the
 * peripheral_driver_t glue in shared/src/kangaroo_driver.c, and each
 * platform's read-capable UART adapter implements
 * kangaroo_transport_ops (shared/include/kangaroo_transport.h).
 * Mirrors the Tic driver's architecture.
 */

#ifndef SAINT_KANGAROO_DRIVER_H
#define SAINT_KANGAROO_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "kangaroo_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Per-Channel Configuration ──────────────────────────────────── */

typedef struct {
    uint8_t  address;       /* 128-135                                  */
    uint8_t  channel_name;  /* '1'/'2' (independent) or 'D'/'T' (mixed) */
    uint8_t  protocol;      /* KANGAROO_PROTO_PACKET | _SIMPLE          */
    uint8_t  home_on_start; /* 1 = send Home after Start                */
    int32_t  max_position;  /* operator scaling for target_position     */
    int32_t  max_speed;     /* operator scaling for target_speed (units/s) */
} kangaroo_channel_config_t;

/* ── Driver API ─────────────────────────────────────────────────── */

void     kangaroo_init(void);
void     kangaroo_update(void);
bool     kangaroo_is_connected(void);

/* Scaled motion entry points (unit = channel index 0..7). value is the
 * already-scaled machine value: position in units, speed in units/s. */
bool     kangaroo_set_position(uint8_t unit, int32_t position);
bool     kangaroo_set_speed(uint8_t unit, int32_t speed);
bool     kangaroo_powerdown(uint8_t unit);
void     kangaroo_powerdown_all(void);

/* ── Registration with the peripheral manager ───────────────────── */

struct peripheral_driver;
const struct peripheral_driver* kangaroo_get_peripheral_driver(void);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_KANGAROO_DRIVER_H */
