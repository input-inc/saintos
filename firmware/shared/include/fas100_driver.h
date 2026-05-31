/**
 * SAINT.OS Firmware - FrSky FAS100 ADV Sensor Driver (shared)
 *
 * Read-only current/voltage/temperature sensor over half-duplex
 * inverted UART. Auto-detects S.Port (57600 baud, slow LED) vs FBUS
 * (460800 baud, fast LED) — operators don't have to pick a mode; the
 * driver probes both and locks to whichever responds.
 *
 * Exposes 4 virtual GPIO channels (232-235): current, voltage, temp1,
 * temp2.
 *
 * Platform-agnostic: protocol parsers, auto-detect state machine, and
 * peripheral_driver_t glue live in shared/src/fas100_driver.c. Each
 * platform's UART adapter implements fas100_transport_ops — see
 * shared/include/fas100_transport.h. The inverted-signaling concern
 * is handled inside transport.open(invert=true): RP2040 uses
 * gpio_set_outover/inover; Teensy uses SERIAL_8N1_RXINV_TXINV.
 */

#ifndef SAINT_FAS100_DRIVER_H
#define SAINT_FAS100_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "sport_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Driver API ─────────────────────────────────────────────────── */

void  fas100_init(void);
void  fas100_update(void);
bool  fas100_is_connected(void);

float fas100_get_current(void);
float fas100_get_voltage(void);
float fas100_get_temp1(void);
float fas100_get_temp2(void);

struct peripheral_driver;
const struct peripheral_driver* fas100_get_peripheral_driver(void);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_FAS100_DRIVER_H */
