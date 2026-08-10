/**
 * SAINT.OS Firmware - Pimoroni Servo 2040 transport (Teensy 4.1)
 *
 * Arduino Wire (I2C-master) adapter for the shared Servo 2040 driver core
 * (shared/src/pimoroni_servo2040_driver.c). Talks to the board (an I2C
 * target on its Qwiic/STEMMA-QT connector) over one of the Teensy's I2C
 * buses, selected from the configured SDA pin:
 *   SDA 18 / SCL 19 → Wire  (default)
 *   SDA 17 / SCL 16 → Wire1
 *   SDA 25 / SCL 24 → Wire2
 */

#include <Arduino.h>
#include <Wire.h>

extern "C" {
#include "pimoroni_servo2040_transport.h"
#include "pimoroni_servo2040_protocol.h"
}

static TwoWire* g_wire = &Wire;
static bool     g_open = false;

static TwoWire* wire_for_sda(uint8_t sda)
{
    switch (sda) {
        case 17: return &Wire1;
        case 25: return &Wire2;
        case 18:
        default: return &Wire;
    }
}

static bool i2c_open(const flash_storage_data_t* storage)
{
    uint8_t sda = 18;   /* Teensy 4.1 primary Wire SDA */
    if (storage && storage->uart_pins.pimoroni_tx_pin) {
        sda = storage->uart_pins.pimoroni_tx_pin;
    }
    g_wire = wire_for_sda(sda);
    g_wire->begin();
    g_wire->setClock(PIMORONI_SERVO2040_I2C_BAUD);
    g_open = true;
    return true;
}

static void i2c_update(void) { /* nothing periodic for a hardware master */ }

static bool i2c_is_connected(void) { return g_open; }

static bool i2c_write_reg(uint8_t reg, const uint8_t* data, size_t len)
{
    if (!g_open) return false;
    g_wire->beginTransmission(PIMORONI_SERVO2040_I2C_ADDR);
    g_wire->write(reg);
    for (size_t i = 0; i < len; i++) g_wire->write(data[i]);
    return g_wire->endTransmission() == 0;
}

static bool i2c_read_reg(uint8_t reg, uint8_t* data, size_t len)
{
    if (!g_open) return false;
    g_wire->beginTransmission(PIMORONI_SERVO2040_I2C_ADDR);
    g_wire->write(reg);
    /* repeated-START: endTransmission(false) keeps the bus for the read */
    if (g_wire->endTransmission(false) != 0) return false;
    size_t got = g_wire->requestFrom((int)PIMORONI_SERVO2040_I2C_ADDR, (int)len);
    if (got != len) return false;
    for (size_t i = 0; i < len; i++) data[i] = (uint8_t)g_wire->read();
    return true;
}

static const pimoroni_servo2040_transport_ops_t i2c_ops = {
    /* .name         = */ "i2c",
    /* .open         = */ i2c_open,
    /* .update       = */ i2c_update,
    /* .is_connected = */ i2c_is_connected,
    /* .write_reg    = */ i2c_write_reg,
    /* .read_reg     = */ i2c_read_reg,
};

extern "C" const pimoroni_servo2040_transport_ops_t*
pimoroni_servo2040_get_transport_i2c(void)
{
    return &i2c_ops;
}
