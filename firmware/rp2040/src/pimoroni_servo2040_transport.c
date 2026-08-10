/**
 * SAINT.OS Firmware - Pimoroni Servo 2040 transport (RP2040)
 *
 * Pico SDK I2C-master adapter for the shared Servo 2040 driver core
 * (shared/src/pimoroni_servo2040_driver.c). Talks to the board (an I2C
 * target on its Qwiic/STEMMA-QT connector) over one of the RP2040's two
 * I2C instances, selected from the configured SDA pin.
 *
 * Default SDA/SCL = GP2/GP3 — the Adafruit Feather RP2040's STEMMA-QT
 * (Qwiic) pins, i2c1 — matching the board's Qwiic link. The operator can
 * override the pins from the Peripherals tab; the server validates them
 * against the board YAML before pushing.
 *
 * Under SIMULATION there is no physical Servo 2040 on the Renode bus, so
 * the transport reports "not connected" and the driver stays inert.
 */

#include "pimoroni_servo2040_transport.h"
#include "pimoroni_servo2040_protocol.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef SIMULATION
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#endif

#define PIMORONI_DEFAULT_SDA_PIN 2   /* Feather RP2040 STEMMA-QT SDA (i2c1) */
#define PIMORONI_DEFAULT_SCL_PIN 3   /* Feather RP2040 STEMMA-QT SCL (i2c1) */

#ifndef SIMULATION
static i2c_inst_t* g_i2c = NULL;
#endif
static uint8_t g_sda = PIMORONI_DEFAULT_SDA_PIN;
static uint8_t g_scl = PIMORONI_DEFAULT_SCL_PIN;
static bool    g_open = false;

static bool i2c_open(const flash_storage_data_t* storage)
{
    uint8_t sda = PIMORONI_DEFAULT_SDA_PIN;
    uint8_t scl = PIMORONI_DEFAULT_SCL_PIN;
    if (storage) {
        if (storage->uart_pins.pimoroni_tx_pin) sda = storage->uart_pins.pimoroni_tx_pin;
        if (storage->uart_pins.pimoroni_rx_pin) scl = storage->uart_pins.pimoroni_rx_pin;
    }
    if (g_open && sda == g_sda && scl == g_scl) return true;   /* idempotent */
    g_sda = sda;
    g_scl = scl;

#ifndef SIMULATION
    /* Instance is fixed by the RP2040 pinmux: SDA on GP{0,4,8,...} = i2c0,
     * GP{2,6,10,...} = i2c1. */
    g_i2c = (((sda / 2) & 1) ? i2c1 : i2c0);
    i2c_init(g_i2c, PIMORONI_SERVO2040_I2C_BAUD);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);
#endif
    g_open = true;
    return true;
}

static void i2c_update(void) { /* nothing periodic for a hardware master */ }

static bool i2c_is_connected(void) { return g_open; }

static bool i2c_write_reg(uint8_t reg, const uint8_t* data, size_t len)
{
    if (!g_open) return false;
#ifndef SIMULATION
    uint8_t buf[1 + PIMORONI_SERVO2040_MAX_XFER];
    if (len > PIMORONI_SERVO2040_MAX_XFER) return false;
    buf[0] = reg;
    for (size_t i = 0; i < len; i++) buf[1 + i] = data[i];
    int w = i2c_write_blocking(g_i2c, PIMORONI_SERVO2040_I2C_ADDR,
                               buf, len + 1, false);
    return w == (int)(len + 1);
#else
    (void)reg; (void)data; (void)len;
    return false;
#endif
}

static bool i2c_read_reg(uint8_t reg, uint8_t* data, size_t len)
{
    if (!g_open) return false;
#ifndef SIMULATION
    /* Point the board's register pointer (no STOP), then repeated-START
     * read. */
    if (i2c_write_blocking(g_i2c, PIMORONI_SERVO2040_I2C_ADDR, &reg, 1, true) != 1)
        return false;
    int r = i2c_read_blocking(g_i2c, PIMORONI_SERVO2040_I2C_ADDR, data, len, false);
    return r == (int)len;
#else
    (void)reg; (void)data; (void)len;
    return false;
#endif
}

static const pimoroni_servo2040_transport_ops_t i2c_ops = {
    .name         = "i2c",
    .open         = i2c_open,
    .update       = i2c_update,
    .is_connected = i2c_is_connected,
    .write_reg    = i2c_write_reg,
    .read_reg     = i2c_read_reg,
};

const pimoroni_servo2040_transport_ops_t* pimoroni_servo2040_get_transport_i2c(void)
{
    return &i2c_ops;
}
