/**
 * SAINT.OS Node Firmware — Teensy 4.1 WDOG1 hardware watchdog.
 *
 * Why this exists: firmware/teensy41/docs/POST_INIT_HANG.md describes
 * a class of post-init hangs where loop() stops iterating and the
 * only recovery on hardware is a physical power cycle. WDOG1 in the
 * iMXRT1062 lets us bound the maximum time the firmware can be wedged
 * before it self-resets, so an operator who isn't physically near
 * the rig still gets the node back without intervention. The reset
 * is a clean full-chip reset (equivalent to power-on); the bootloader
 * runs, then our firmware. SRC_SRSR's TOUT bit on the new boot
 * carries the cause across, so the post-watchdog boot can log
 * "previous reset was a watchdog timeout" for diagnostics.
 *
 * WDOG choice: iMXRT1062 has WDOG1, WDOG2, WDOG3, plus RTWDOG.
 * WDOG1 is the canonical "system" watchdog — 128 s max timeout in
 * 0.5 s steps, hooked to the chip's reset line. RTWDOG is shorter
 * range (typically 2 s) and isn't broad enough for our worst-case
 * blocking call (DHCP retry can take 5–10 s on a slow gateway).
 *
 * Disabled under SIMULATION because (a) Renode doesn't model WDOG1
 * meaningfully and a sim-time advance can outrun any timeout we set,
 * and (b) the sim doesn't have an analogous "physical power cycle"
 * problem — node_manager.py owns the sim node lifecycle. Hardware
 * builds are the only ones that need the safety net. */

#ifndef SIMULATION

#include <Arduino.h>
#include "imxrt.h"

extern "C" {
#include "saint_log.h"
}

#include "watchdog.h"

/* Last-resort timeout: how long loop() is allowed to be silent before
 * WDOG1 resets the chip. Picked to be comfortably longer than the
 * worst-case blocking call we know about (DHCP retry attempts, full
 * cleanup+init_micro_ros after a session loss — both observed under
 * ~10 s in normal operation) but short enough that an operator
 * doesn't think the node has died for good. */
#ifndef SAINT_WATCHDOG_TIMEOUT_S
#define SAINT_WATCHDOG_TIMEOUT_S 30
#endif

/* WDOG1's WCR.WT field encodes timeout in (WT+1) × 0.5 s units, where
 * WT is 8 bits wide. So range is 0.5 s..128 s. */
#if SAINT_WATCHDOG_TIMEOUT_S < 1 || SAINT_WATCHDOG_TIMEOUT_S > 128
#error "SAINT_WATCHDOG_TIMEOUT_S out of WDOG1 range (1..128 seconds)"
#endif

static bool g_armed = false;

/* Decode SRC_SRSR into a stable short string for the boot banner.
 * iMXRT1062 RM 9.7.34 — each bit is set when the corresponding event
 * caused the LAST reset and stays set until we explicitly clear it
 * (write-1-to-clear). Multiple bits can be set simultaneously (e.g.
 * a watchdog timeout right after a power-on dip sets both WDOG_RST_B
 * and IPP_RESET_B). We prioritize the "most informative" cause: a
 * watchdog/lockup/tempsense reset is much more notable than the
 * power-on bit that's set on every cold boot. */
static const char* srsr_cause_str(uint32_t srsr)
{
    if (srsr & SRC_SRSR_TEMPSENSE_RST_B)    return "over-temperature";
    if (srsr & SRC_SRSR_LOCKUP_SYSRESETREQ) return "software (SYSRESETREQ or CPU lockup)";
    if (srsr & SRC_SRSR_WDOG3_RST_B)        return "WDOG3 timeout";
    if (srsr & SRC_SRSR_WDOG_RST_B)         return "WDOG1/WDOG2 timeout";
    if (srsr & SRC_SRSR_CSU_RESET_B)        return "CSU (security)";
    if (srsr & SRC_SRSR_JTAG_SW_RST)        return "JTAG software";
    if (srsr & SRC_SRSR_JTAG_RST_B)         return "JTAG";
    if (srsr & SRC_SRSR_IPP_USER_RESET_B)   return "external reset button";
    if (srsr & SRC_SRSR_IPP_RESET_B)        return "power-on";
    return "unknown";
}

extern "C" void watchdog_init(void)
{
    /* SRSR is sticky across resets until cleared (write-1-to-clear).
     * Snapshot it before clearing so the boot banner can announce
     * WHY we just booted — useful for distinguishing a clean power-on
     * from a watchdog-recovery reboot from an operator-triggered
     * restart (firmware writes SCB_AIRCR = 0x05FA0004 for factory
     * reset / OTA → SYSRESETREQ shows up as
     * SRC_SRSR_LOCKUP_SYSRESETREQ on the next boot). The iMXRT1062
     * SRSR bits are documented at RM 9.7.34. */
    uint32_t srsr = SRC_SRSR;
    const char* cause = srsr_cause_str(srsr);

    Serial.printf("Reset cause: %s (SRSR=0x%08lX)\n",
                  cause, (unsigned long)srsr);

    /* Mirror the cause to /log on every boot — matches RP2040's
     * `Boot OK ... reset cause: <X>` line so the dashboard's Logs
     * tab carries a stable boot-marker entry the operator can scan
     * for. Severity is per-cause: warn for unexpected resets
     * (watchdog, lockup), error for thermal, info for benign causes
     * (power-on, deliberate reset). The line goes through
     * saint_log_boot_queue so it lands on the wire on first
     * /announce (the server hasn't subscribed to /log yet at the
     * moment of this call). */
    const char* level;
    if (srsr & SRC_SRSR_TEMPSENSE_RST_B) {
        level = "error";
    } else if ((srsr & SRC_SRSR_WDOG_RST_B)
            || (srsr & SRC_SRSR_WDOG3_RST_B)
            || (srsr & SRC_SRSR_LOCKUP_SYSRESETREQ)) {
        /* LOCKUP_SYSRESETREQ covers both CPU lockup (genuinely bad)
         * AND our own factory_reset / OTA-driven SCB_AIRCR write.
         * We can't distinguish them from SRSR alone, so "warn" is
         * the conservative call — a clean operator-initiated reset
         * gets an unexpectedly-loud log entry, which is fine. */
        level = "warn";
    } else {
        /* Power-on, external button, JTAG, CSU — all benign. */
        level = "info";
    }
    saint_log_boot_queue(level,
        "Boot — reset cause: %s [SRSR=0x%08lX, watchdog armed at %u s]",
        cause, (unsigned long)srsr,
        (unsigned)SAINT_WATCHDOG_TIMEOUT_S);

    /* Clear all sticky bits so the next boot's SRSR snapshot is
     * uncontaminated. Writing 1 to each bit clears it per the
     * iMXRT1062 RM. */
    SRC_SRSR = srsr;

    /* Compute WT: timeout = (WT+1) × 0.5 s → WT = 2*timeout − 1. */
    uint32_t wt = (uint32_t)(SAINT_WATCHDOG_TIMEOUT_S * 2 - 1);

    /* WCR programming notes (IMXRT1062 RM 60.5.1):
     *   - WT  [15:8] : timeout count
     *   - WDE [2]    : enable (write-once-to-1; can't be cleared)
     *   - SRS [4]    : software reset signal (=1 means "no reset now")
     *   - WRE [3]    : reset signal type (0=internal reset, 1=WDOG_B
     *                  external signal). Leave default (internal).
     *   - WDA [5]    : WDOG_B assertion (=1 means "do not assert")
     *
     * WCR is 16-bit. The WT and WDE/SRS/WDA bits must be written
     * together (one 16-bit store) because WDE is write-once. */
    uint16_t wcr = (uint16_t)((wt << 8)
                              | (1 << 5)    /* WDA: don't assert WDOG_B */
                              | (1 << 4)    /* SRS: no SW reset now */
                              | (1 << 2));  /* WDE: enable watchdog */
    WDOG1_WCR = wcr;

    g_armed = true;
    Serial.printf("Watchdog: WDOG1 armed @ %u s timeout\n",
                  (unsigned)SAINT_WATCHDOG_TIMEOUT_S);
}

extern "C" void watchdog_feed(void)
{
    if (!g_armed) return;
    /* WSR is the service register. Per IMXRT1062 RM, the sequence
     * 0x5555 followed by 0xAAAA reloads the counter. Both writes
     * must happen within ~0.5 s of each other; doing them back-to-
     * back is the safe pattern. */
    WDOG1_WSR = 0x5555;
    WDOG1_WSR = 0xAAAA;
}

#else  /* SIMULATION */

/* Stub bodies so callers don't need to #ifdef. The sim has no chip
 * reset semantics to wire up; node_manager.py handles lifecycle. */
extern "C" void watchdog_init(void) {}
extern "C" void watchdog_feed(void) {}

#endif  /* !SIMULATION */
