# Renode Teensy 4.1 sim bring-up — handoff

> **Looking to just run firmware tests against the simulator?** See
> `firmware/simulation/docker/README.md` (RP2040 path is currently the
> only e2e-integrated target). This doc is the historical record of the
> Teensy 4.1 (iMXRT1062) sim bring-up and a snapshot of where the work
> currently sits.

## Goal

Get the Teensy 4.1 firmware (`firmware/teensy41/build/simulation/firmware.elf`)
to boot inside Renode, establish a micro-ROS XRCE-DDS session against the
agent, reach `BOOT → UNADOPTED`, and pass `test_sync_recovery.py` against
it the same way `MODE=full` does for the RP2040.

## Where things stand right now

### What works

- **Host PlatformIO install.** `~/.platformio/penv/bin/pio` builds the sim
  firmware (`pio run -e simulation` from `firmware/teensy41/`). PIO is
  installed via `~/.platformio/penv/bin/pip install platformio` — the
  `penv` already existed but was empty.
- **`Serial` → `Serial1` redirect for sim.** Teensy's default `Serial`
  is the USB CDC, which Renode can't capture. Under `SIMULATION`,
  `firmware/teensy41/include/platform.h` redefines `Serial` to `Serial1`
  (= LPUART6 on Teensy 4.1, pins 0/1), and `platform.h` is now included
  by every Serial-using `.cpp` file. `Serial.begin(115200)` becomes
  `Serial1.begin(115200)` and all `Serial.printf(...)` calls route to a
  hardware UART that Renode's `NXP_LPUART` model captures.
- **Renode platform model.** `firmware/teensy41/simulation/renode_teensy41/boards/teensy41.repl`
  is now hand-derived from Renode's in-tree `platforms/cpus/imxrt1064.repl`
  (the 1064 and Teensy's 1062 are the same SoC family). We can't use a
  bare `using "..."` clause because the base registers `flex_spi` with a
  "ciphertext" BusMultiRegistration at 0x60000000..0x6EFFFFFF that
  routes reads/writes through the FlexSPI command sequencer — fine if
  you wire an SPI flash chip, useless for our `LoadELF`-into-XIP setup.
  We re-declare flex_spi locally with only its control-register
  registration, freeing 0x60000000 for a plain `Memory.MappedMemory`
  region (`external_flash`, 8 MB) that the firmware ELF loads into
  directly. The repl also adds:
  - `udp_bridge` at 0x50300000 (saint-os custom Renode peripheral —
    shared with the RP2040 sim via the .cs sources under
    `firmware/rp2040/simulation/renode_rp2040/...`).
  - `persistent_storage` at 0x50400000 (same).
  - `numberOfMPURegions: 16` on the CPU (the default 8 trips the
    Teensy startup's MPU region-15 write with a CPU abort).
  - OCRAM extended to 1 MB (0x20200000..0x202FFFFF) so the firmware's
    `.bss.dma` section at 0x20200000 fits.
- **node_manager.py Teensy path.** Fixed the bugs that were carrying
  over from when the .repl was a stub:
  - `teensy41_build_dir` was `.pio/build/simulation`; `platformio.ini`
    sets `build_dir = build`, so it's `build/simulation` now.
  - Dropped `--console` for the background invocation (same EOF-disposes-machine
    trap as the RP2040 path — see `feedback_renode_invocation_traps.md`).
  - Resc generator now uses `lpuart6` (Serial1 destination) instead of
    the old `uart0` stub name, and wires `CreateFileBackend` so UART
    bytes mirror to `firmware/simulation/logs/<id>.uart.log`.
  - Resc sets `sysbus.cpu PC 0x6000164D` + `sysbus.cpu SP 0x20080000`
    after `LoadELF`. Reason: Teensy's reset path is bootrom → IVT@0x60001000
    → IVT[entry]=0x6000164D → `ResetHandler2`. We can't model the
    bootrom, so jump to the application's ResetHandler directly.
    Setting `VectorTableOffset` doesn't help because the firmware's
    vector table isn't at 0x60000000 — that's the FlexSPI config
    block; the actual VT is built at runtime by ResetHandler2 (it
    copies `.text.itcm` to ITCM@0 and sets VTOR).

### What's still broken

**The CPU runs through early init (MPU, IOMUXC pads, CCM clocks) and
then hangs silently inside `configure_cache` at 0x600016D8.**

The boot path observed in `firmware/simulation/logs/teensy_smoke.log`:

```
PC=0x60001656..0x6000165C   IOMUXC_GPR pad-config writes (DSE, ODE, SRE)
PC=0x600014EC..0x60001512   CCM_CSCDR1 + IOMUXC pad-strength setup
PC=0x60001514               bl  600016d8 <configure_cache>
                            ← all log activity stops here
```

`configure_cache` (in the Teensy core) writes to ARM system-control
space registers at `0xE000E000+`:

- `0xE000ED94` (MPU_CTRL)
- `0xE000ED9C` (MPU_RNR) — region number
- `0xE000EDA0` (MPU_RBAR) — region base address

It configures all 16 MPU regions (one per iteration) then enables the
MPU, invalidates the I-cache (`ICIALLU` @ `0xE000EF50`), and twiddles
the CCR (`0xE000ED14`).

Working hypotheses for where it stalls (in priority order):

1. **Renode's NVIC/MPU model rejects one of the region configs.**
   The function writes RBAR + RNR + RASR in a specific order; Renode's
   model may interpret a particular bit pattern as a fault. Verify by
   adding `cpu LogFunctionNames true` + `sysbus LoadSymbolsFrom
   @firmware.elf` (do **not** set `MaximumBlockSize 1` — see
   `feedback_renode_invocation_traps.md`) and watching where the PC
   actually parks.
2. **I-cache invalidation hangs.** `ICIALLU` write at
   `0xE000EF50` should complete instantly but Renode's Cortex-M model
   might not support it; the write could trap into an
   unimplemented-feature handler.
3. **CCR `[BFHFNMIGN | DIV_0_TRP]` write triggers a fault** the
   firmware can't handle (we set up no fault handlers in the sim
   context).

### Suggested next experiments

#### 1. Function-trace the hang

Add to `node_manager.py:_generate_teensy41_resc` (temporary diagnostic):

```
sysbus LoadSymbolsFrom @{firmware_path}
cpu LogFunctionNames true
```

Run, watch where the PC parks. If it's inside `configure_cache`, the
mpu/cache theory is right. If it's elsewhere (HardFault handler,
infinite loop in `__libc_init_array`, etc.), pivot accordingly.

**Do not** add `cpu MaximumBlockSize 1` — that slows the JIT ~1000× and
makes ordinary delay loops look like hangs.

#### 2. Skip cache configuration entirely

If (1) confirms `configure_cache` is the problem, the fastest unblock
is to patch the firmware's startup to no-op the cache enable under
`SIMULATION`. The Teensy core's `configure_cache` lives in
`~/.platformio/packages/framework-arduinoteensy/cores/teensy4/startup.c`
(or similar). Wrap its body in `#ifndef SIMULATION` and rebuild. The
sim has no real cache to enable, so this is a benign skip.

#### 3. PIT timer model

Once cache config is past, the next likely stall point is `delay(100)`
in `setup()` — Teensy uses the PIT timer (at 0x40084000, currently a
TAG-only entry in our repl) for `millis()`/`micros()`. Renode has no
IMX_PIT model in the in-tree imxrt1064 set; we'd need to either:

- (a) Model PIT as a Python peripheral that increments a counter on
  every read (cheap, inaccurate but works for `delay`).
- (b) Find / port an upstream IMX PIT model.

The shared `firmware/rp2040/simulation/renode_rp2040/emulation/peripherals/`
dir has the saint-os custom peripherals if you want a starting template
for option (a).

#### 4. Docker e2e integration (deferred)

When the firmware actually boots far enough to print the SAINT.OS banner,
wire it into the existing Docker e2e:

- Add PIO install to `Dockerfile.e2e` (`~/.platformio/penv/bin/pip install
  platformio`).
- Add a Teensy build step alongside the existing colcon-build step.
- Mount `firmware/teensy41/build/simulation/` into the container the
  same way `firmware/rp2040/install/simulation/` is mounted today.
- Parameterize `test_sync_recovery.py` to optionally create a teensy41
  node instead of rp2040.
- Add `MODE=teensy_full` to `entrypoint.sh`.

## How to reproduce locally

Prerequisites: PIO installed in `~/.platformio/penv/`, Renode 1.16+ at
`~/Applications/Renode.app/` (macOS) or `$RENODE_PATH`.

```bash
# 1. Build the sim firmware.
cd firmware/teensy41 && ~/.platformio/penv/bin/pio run -e simulation

# 2. Create + start a sim node.
cd /Users/hackman/Projects/OpenSAINT/SaintOS/source
./firmware/simulation/node_manager.py create teensy_smoke --type teensy41
./firmware/simulation/node_manager.py start teensy_smoke

# 3. Watch for UART output (currently stays empty — see "What's still
#    broken" above).
tail -F firmware/simulation/logs/teensy_smoke.uart.log

# 4. Watch Renode's operational log (last warning's PC tells you how
#    far the CPU got).
tail -F firmware/simulation/logs/teensy_smoke.log

# 5. Stop / clean up.
./firmware/simulation/node_manager.py stop teensy_smoke
./firmware/simulation/node_manager.py remove teensy_smoke
```

## File map of changes in this branch (Teensy-related only)

| File | What changed |
|---|---|
| `firmware/teensy41/include/platform.h` | Under `SIMULATION`, `#define Serial Serial1` — routes USB-CDC printf calls to LPUART6 so Renode's `NXP_LPUART` model can capture them. |
| `firmware/teensy41/src/*.cpp` | Added `#include "platform.h"` after `#include <Arduino.h>` to all .cpp files that call `Serial.*`. Carries the `Serial`-rewrite to every TU. |
| `firmware/teensy41/simulation/renode_teensy41/boards/teensy41.repl` | Replaced stub-peripheral repl with one hand-derived from Renode's `platforms/cpus/imxrt1064.repl`. Drops the flex_spi ciphertext registration, adds `external_flash` at 0x60000000 (plain MappedMemory), adds saint-os `udp_bridge` + `persistent_storage`, sets `numberOfMPURegions: 16`, extends OCRAM to 1 MB. |
| `firmware/simulation/node_manager.py` | Teensy path: build_dir corrected to `build/simulation`, `--console` dropped from background invocation, resc generator uses `lpuart6` + `CreateFileBackend` for UART log capture, manually sets `sysbus.cpu PC/SP` after LoadELF to skip the bootrom. |
