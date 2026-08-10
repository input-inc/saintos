/**
 * SAINT.OS Firmware - Pimoroni Servo 2040 transport ops (shared)
 *
 * The Servo 2040 host link is I2C on the board's Qwiic/STEMMA-QT
 * connector. The shared driver core (shared/src/
 * pimoroni_servo2040_driver.c) is transport-agnostic and issues register
 * reads/writes (see the register map in pimoroni_servo2040_protocol.h)
 * through one of these per-platform ops tables — an I2C master on the
 * controller's Qwiic/I2C bus.
 *
 * The ops-table shape mirrors maestro_transport.h in spirit but is
 * register-oriented rather than byte-stream, because I2C is inherently a
 * register/command bus. A platform that can't supply an I2C master
 * returns NULL from its getter; the driver then logs a config error and
 * stays inert.
 */

#ifndef SAINT_PIMORONI_SERVO2040_TRANSPORT_H
#define SAINT_PIMORONI_SERVO2040_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "flash_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct pimoroni_servo2040_transport_ops {
    const char* name;             /* "i2c" — used in logs */

    /* Open/initialize the I2C master using the saved config (which I2C
     * instance + the SDA/SCL pins from flash_uart_pins_t, reused for the
     * Qwiic pins). Returns true on success. */
    bool (*open)(const flash_storage_data_t* storage);

    /* Periodic transport work (no-op for a hardware I2C master). */
    void (*update)(void);

    /* Is the I2C bus initialized/usable? This is bus-level only — actual
     * device presence is decided by the driver polling REG_WHOAMI. */
    bool (*is_connected)(void);

    /* Write `len` bytes to `reg` on the board (one I2C write of
     * [reg][data...]). Returns true if the transfer was ACKed. */
    bool (*write_reg)(uint8_t reg, const uint8_t* data, size_t len);

    /* Read `len` bytes starting at `reg` (write [reg], repeated-START,
     * read len). Returns true on success; `data` is filled. */
    bool (*read_reg)(uint8_t reg, uint8_t* data, size_t len);
} pimoroni_servo2040_transport_ops_t;

/* Platform-provided lookup. Declared unconditionally so the shared
 * driver compiles on every target; the per-platform definition decides
 * what to return (NULL if the platform can't host an I2C master). */
const pimoroni_servo2040_transport_ops_t* pimoroni_servo2040_get_transport_i2c(void);

#ifdef __cplusplus
}
#endif

#endif /* SAINT_PIMORONI_SERVO2040_TRANSPORT_H */
