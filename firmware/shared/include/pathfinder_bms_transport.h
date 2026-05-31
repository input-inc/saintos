/**
 * SAINT.OS Firmware - Pathfinder BMS transport ops (shared)
 *
 * The JBD-protocol BMS uses bidirectional 9600 baud UART — we send
 * 7-byte read-request frames and the BMS responds with framed data
 * payloads. The shared driver core (shared/src/pathfinder_bms_driver.c)
 * runs the framing state machine and dispatches the byte-level I/O
 * through this ops table.
 *
 * Read is non-blocking — returns 0 when no bytes are available so the
 * driver's update() loop can drain whatever's buffered each tick
 * without blocking on a response.
 */

#ifndef SAINT_PATHFINDER_BMS_TRANSPORT_H
#define SAINT_PATHFINDER_BMS_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct pathfinder_bms_transport_ops {
    const char* name;

    bool   (*open)(uint8_t tx_pin, uint8_t rx_pin, uint16_t baud);
    bool   (*is_open)(void);
    bool   (*write)(const uint8_t* data, size_t len);
    /* Non-blocking. Returns count read, possibly 0. */
    size_t (*read)(uint8_t* data, size_t max_len);
    uint8_t (*resolved_instance)(void);
} pathfinder_bms_transport_ops_t;

const pathfinder_bms_transport_ops_t* pathfinder_bms_get_transport(void);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_PATHFINDER_BMS_TRANSPORT_H */
