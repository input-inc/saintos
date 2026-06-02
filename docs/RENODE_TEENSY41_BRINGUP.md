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
  is hand-derived from Renode's in-tree `platforms/cpus/imxrt1064.repl`
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
- **Boot reaches `main()`.** A post-link binary patch
  (`firmware/teensy41/patch_sim_elf.py`, wired into `env:simulation` via
  `extra_scripts`) overwrites the entry of five Teensy-core startup
  functions with `bx lr` (Thumb `0x4770`), turning each into an
  immediate return. The patched symbols all touch hardware Renode
  doesn't model, and the sim has no real semantics to preserve:
  - `configure_cache` — MPU/ICACHE/DCACHE enable trips a fault.
  - `usb_pll_start` — spin-polls PLL lock/enable bits that never set.
  - `set_arm_clock` — same story for the ARM core PLL.
  - `configure_external_ram` — PSRAM init over FlexSPI2 (unwired).
  - `usb_init` — USB controller (unmodeled).
  With these out of the way the firmware executes `ResetHandler2`
  through `main()` and into Arduino `setup()`. Function-trace evidence
  in `firmware/simulation/logs/teensy_smoke.log` shows entries into
  `HardwareSerialIMXRT::write` / `::write9bit` and `Serial1.begin()`'s
  IRQ-attach path, confirming the firmware is alive and trying to
  print.
- **`SAINT_TEENSY_TRACE=1` diagnostic mode.**
  `node_manager.py:_generate_teensy41_resc` reads this env var when
  generating the .resc; if set, the script emits
  `sysbus LoadSymbolsFrom @firmware.elf`, `cpu LogFunctionNames true`,
  and `logLevel -1 sysbus.lpuart6`. Useful while debugging — leave
  unset for routine runs. Do not pair with `cpu MaximumBlockSize 1`
  (see `feedback_renode_invocation_traps.md`).

### What's still broken

**LPUART6 IRQ asserts but is never dispatched, so `Serial.printf(...)`
fills the TX buffer once and then blocks forever.** The UART log
(`firmware/simulation/logs/teensy_smoke.uart.log`) stays empty even
though the firmware is calling `write9bit` continuously.

Trace evidence (with `SAINT_TEENSY_TRACE=1`):

- `[NOISY] lpuart6: Setting IRQ to True, ... tx True, ...` repeats every
  ~100 µs of simulated time.
- No function-entry log for `HardwareSerialIMXRT::IRQHandler` ever
  appears.
- SysTick *is* delivering (`Entering function systick_isr` fires at
  the configured rate), so basic NVIC + vector-table dispatch works.

`write9bit`'s inline transmit path (the one that writes `port->DATA`
directly without an IRQ) only fires when
`nvic_execution_priority() <= hardware->irq_priority`. In mainline
context that priority is 256 and `irq_priority` is 64
(`HardwareSerial1.cpp:41`), so the inline path is skipped and the
function relies on the IRQ to drain the TX ring buffer. The IRQ never
runs, the buffer fills, and the next `write9bit` call spins in the
`while (tx_buffer_tail_ == head)` busy-wait.

Working hypotheses for why LPUART6 IRQ isn't reaching the CPU:

1. **NVIC ISER0 bit-25 write isn't being honored by Renode's NVIC
   model.** `Serial1.begin()` calls `NVIC_ENABLE_IRQ(IRQ_LPUART6 = 25)`
   which writes `1 << 25` to `0xE000E100`. If Renode's NVIC doesn't
   track this enable correctly the pending IRQ never escalates.
   Verify by tagging `0xE000E100..0xE000E10F` and re-running with the
   trace on; check whether the enable write is observed and whether
   the bit stays set on read-back.
2. **Priority/grouping mismatch.** SysTick has system-exception
   priority (set via SHPR3 = 0x20200000) and is delivered. LPUART6's
   IRQ priority is 64 via `NVIC_SET_PRIORITY(25, 64)` — set, but
   maybe Renode's NVIC is treating it as masked by something we
   wrote to BASEPRI elsewhere. Inspect priority/BASEPRI values after
   the begin() call.
3. **NXP_LPUART IRQ output not actually connected.** The .repl wires
   `lpuart6 IRQ -> nvic@25` but it's worth verifying with
   `sysbus.nvic GetIRQs` from the monitor that pin 25 is bound and
   matches LPUART6's output.

### Suggested next experiments

#### 1. Decide whether to chase NVIC delivery or sidestep it

Easiest sidestep: patch `HardwareSerialIMXRT::write9bit` (post-link, same
mechanism as `patch_sim_elf.py`) to write `port->DATA` synchronously
under SIMULATION and return — losing async buffering but guaranteeing
the byte hits Renode's `CreateFileBackend`. Trivial to verify, removes
the IRQ-delivery question from the critical path, and the sim doesn't
care about TX backpressure. Downside: a non-trivial function rewrite vs.
a 2-byte `bx lr` shim.

If chasing NVIC delivery: targeted logging on the ARM SCS region
(`logLevel -1 sysbus.nvic` plus tags at 0xE000E100/0xE000E104/0xE000E180)
plus a small `monitor` script that dumps `sysbus.nvic GetIRQs` after
boot will tell you whether IRQ 25 is even being recognized as enabled.

#### 2. PIT timer model (still pending, may come up next)

Teensy uses the PIT (at 0x40084000, currently a TAG-only entry) for
`millis()`/`micros()`. SysTick keeps `millis()` ticking, but anything
that reads `PIT_*` registers directly (some peripheral drivers do) will
see zeros. If/when the next stall is in PIT-land:

- (a) Model PIT as a Python peripheral that increments a counter on
  every read (cheap, inaccurate but works for `delay`).
- (b) Port an upstream IMX_PIT model.

The shared `firmware/rp2040/simulation/renode_rp2040/emulation/peripherals/`
dir has the saint-os custom peripherals if you want a starting template
for option (a).

#### 3. Docker e2e integration (deferred)

When the firmware actually prints the SAINT.OS banner, wire it into the
existing Docker e2e:

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
| `firmware/simulation/node_manager.py` | Teensy path: build_dir corrected to `build/simulation`, `--console` dropped from background invocation, resc generator uses `lpuart6` + `CreateFileBackend` for UART log capture, manually sets `sysbus.cpu PC/SP` after LoadELF to skip the bootrom. `SAINT_TEENSY_TRACE=1` env var emits LoadSymbolsFrom + LogFunctionNames + verbose lpuart6 logging into the .resc. |
| `firmware/teensy41/patch_sim_elf.py` | New post-link script (sim env only). Rewrites the entry of `configure_cache`, `usb_pll_start`, `set_arm_clock`, `configure_external_ram`, `usb_init` to `bx lr` so the firmware skips hardware Renode can't model. Uses pyelftools (already in PIO's penv). See script header for why `-Wl,--wrap=` doesn't work here (same-TU calls). |
| `firmware/teensy41/platformio.ini` | `env:simulation` `extra_scripts` now includes `post:patch_sim_elf.py`. |
