/**
 * SAINT.OS Node Firmware - Clock Implementation
 *
 * Provides POSIX clock_gettime() for micro-ROS on RP2040.
 */

#include <stdint.h>
#include <time.h>
#include <errno.h>

#include "pico/stdlib.h"
#include "hardware/timer.h"

/**
 * POSIX clock_gettime implementation using Pico SDK timer.
 */
int clock_gettime(clockid_t clk_id, struct timespec *tp)
{
    (void)clk_id;  // We only support one clock

    if (tp == NULL) {
        errno = EINVAL;
        return -1;
    }

    // Get time in microseconds since boot
    uint64_t us = time_us_64();

    // Convert to seconds and nanoseconds
    tp->tv_sec = us / 1000000ULL;
    tp->tv_nsec = (us % 1000000ULL) * 1000ULL;

    return 0;
}
