/**
 * SAINT.OS Firmware - Discovery Retry Helper
 *
 * Cross-platform retry-forever wrapper around discover_server(). Pulled
 * out of firmware/{rp2040,teensy41}/src/main.* to keep cold-boot
 * resilience behaviour in lockstep across targets: a divergence here
 * (Teensy bailing after one batch while RP2040 retried forever) left
 * a Teensy node permanently parked in NODE_STATE_ERROR when the
 * Pi-side server wasn't ready yet at boot time.
 *
 * See firmware/shared/include/discovery.h for the signature + cadence
 * documentation. PLATFORM_LED_TICK() and PLATFORM_WATCHDOG_FEED()
 * (declared in each platform's platform.h) are invoked inside the
 * backoff sleep so the status LED keeps animating and the hardware
 * watchdog stays fed across the wait.
 */

#include "platform.h"
#include "discovery.h"

void discover_server_retry_forever(uint8_t* server_ip, uint16_t* server_port,
                                    uint32_t attempt_timeout_ms,
                                    int attempts_per_batch)
{
    uint32_t batch = 0;
    PLATFORM_PRINTF("Discovering SAINT server...\n");
    while (!discover_server(server_ip, server_port,
                            attempt_timeout_ms, attempts_per_batch)) {
        batch++;
        // First few batches: 2s backoff. After that, 5s — keeps log
        // volume sane if the network is actually broken while still
        // pouncing fast when the server lands during the Pi-boot window.
        uint32_t backoff_ms = (batch < 3) ? 2000u : 5000u;
        PLATFORM_PRINTF("Discovery batch %lu failed; retrying in %lu ms "
                        "(server not up yet?)\n",
                        (unsigned long)batch, (unsigned long)backoff_ms);
        // Sleep in 100ms slices so the LED animates smoothly and the
        // watchdog gets fed regularly. PLATFORM_SLEEP_MS may itself
        // be a blocking call on one platform (Arduino delay()) and a
        // yielding one on the other (Pico sleep_ms()); the 100ms
        // granularity keeps both well under any reasonable WDT
        // timeout.
        for (uint32_t slept = 0; slept < backoff_ms; slept += 100) {
            PLATFORM_SLEEP_MS(100);
            PLATFORM_LED_TICK();
            PLATFORM_WATCHDOG_FEED();
        }
    }
}
