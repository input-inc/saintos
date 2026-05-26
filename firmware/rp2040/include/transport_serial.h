/**
 * SAINT.OS Node Firmware - Serial Transport Header
 *
 * micro-ROS transport over UART for Renode simulation.
 */

#ifndef TRANSPORT_SERIAL_H
#define TRANSPORT_SERIAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <uxr/client/transport.h>

/**
 * Initialize the serial transport.
 * Sets up UART1 for micro-ROS communication.
 *
 * @return true if initialization succeeded
 */
bool transport_serial_init(void);

/**
 * Connect the serial transport.
 * For serial, this always succeeds if initialized.
 *
 * @return true if connected
 */
bool transport_serial_connect(void);

/**
 * Check if serial transport is connected.
 *
 * @return true if connected
 */
bool transport_serial_is_connected(void);

/**
 * micro-ROS transport open callback.
 */
bool transport_serial_open(struct uxrCustomTransport* transport);

/**
 * micro-ROS transport close callback.
 */
bool transport_serial_close(struct uxrCustomTransport* transport);

/**
 * micro-ROS transport write callback.
 */
size_t transport_serial_write(
    struct uxrCustomTransport* transport,
    const uint8_t* buf,
    size_t len,
    uint8_t* err);

/**
 * micro-ROS transport read callback.
 */
size_t transport_serial_read(
    struct uxrCustomTransport* transport,
    uint8_t* buf,
    size_t len,
    int timeout_ms,
    uint8_t* err);

#endif // TRANSPORT_SERIAL_H
