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
- **Boot reaches `main()` *and prints*.** A post-link binary patch
  (`firmware/teensy41/patch_sim_elf.py`, wired into `env:simulation` via
  `extra_scripts`) overwrites the entry of five Teensy-core startup
  functions with `bx lr` (Thumb `0x4770`), turning each into an
  immediate return. All touch hardware Renode doesn't model, and the
  sim has no real semantics to preserve:
  - `configure_cache` — MPU/ICACHE/DCACHE enable trips a fault.
  - `usb_pll_start` — spin-polls PLL lock/enable bits that never set.
  - `set_arm_clock` — same story for the ARM core PLL.
  - `configure_external_ram` — PSRAM init over FlexSPI2 (unwired).
  - `usb_init` — USB controller (unmodeled).
  The same script replaces `HardwareSerialIMXRT::write9bit` with an
  8-byte shim that bypasses the IRQ-driven TX ring entirely:
  `ldr r3,[r0,#16]; str r1,[r3,#28]; movs r0,#1; bx lr` — read
  `port_addr` from `this+16`, write the byte to `port_addr+0x1C`
  (LPUART DATA register), return 1. With this in place the firmware
  boots all the way to `Serial.printf("Initializing micro-ROS...")` and
  fails — as expected — at the micro-ROS handshake when no agent is
  listening on port 8888. SAINT.OS banner, hardware init, MAC/IP
  printouts, and node ID all land in `teensy_smoke.uart.log`.
- **`#include "platform.h"` in `src/main.cpp`.** Required because
  `platform.h`'s `#define Serial Serial1` (under SIMULATION) must be
  in scope when `Serial.begin(115200)` is parsed. Without it,
  `Serial.begin` in `setup()` resolved to the framework's USB CDC
  object — and since `usb_init` is patched out, that begin() call did
  nothing observable, and crucially **LPUART6's `CTRL.TE` never got
  set**, leaving Renode's NXP_LPUART model unable to drain TX bytes
  even after the `write9bit` shim was writing to DATA. The other
  Teensy .cpp files already pulled `platform.h` in; main.cpp was the
  outlier.
- **`SAINT_TEENSY_TRACE=1` diagnostic mode.**
  `node_manager.py:_generate_teensy41_resc` reads this env var when
  generating the .resc; if set, the script emits
  `sysbus LoadSymbolsFrom @firmware.elf`, `cpu LogFunctionNames true`,
  and `logLevel -1 sysbus.lpuart6`. Useful while debugging — leave
  unset for routine runs. Do not pair with `cpu MaximumBlockSize 1`
  (see `feedback_renode_invocation_traps.md`).

### What's still broken

**Phase 2/3 of `test_sync_recovery.py` in `MODE=teensy_full` fail
because /log messages after the boot-queue drain reach the server with
empty payloads** — server log fills with
`(malformed log frame: Expecting value: line 1 column 1 (char 0)) ''`.
Phase 1 (announce + adopt + sync request) passes cleanly. The actual
firmware-side work is happening — the UART log shows the
`[info] Config received (56 bytes), applying…` /
`Config applied OK` / `Config saved to flash` lines locally, and the
server's `/announce` parser sees the sync-ACK
(`Config saved to flash (sync ACK via /announce, uptime_ms=8553)`) — but
the /log topic publish path drops the JSON payload somewhere between
`saint_log_emit_ros` and the server's `/log` subscription. This is
shared firmware code (`firmware/shared/src/saint_log.c` +
`firmware/teensy41/src/main.cpp:saint_log_emit_ros`) so a bug here
likely affects RP2040 too once that path runs — worth bisecting
against an RP2040 capture before assuming the fix lives in Teensy
code.

**Chip-unique-ID is zero in Renode** so the firmware always announces
as `teensy41_0000000000000000`, not the host-assigned
`teensy41_synctest`. The harness now auto-detects this by matching the
`<chip_family>_` prefix in `list_unadopted` and rewires subscriptions
to the announced ID. Not a bug per se, but worth noting if you ever
expect persistent storage `NodeId` to flow into the firmware — it
only sets the backing filename, not the firmware's view of itself.

The original IRQ-delivery question (LPUART6 IRQ asserted but
`HardwareSerialIMXRT::IRQHandler` never entered) is still open in the
abstract — we sidestepped it twice: first with the `bx lr`-only
`write9bit` shim, then by upgrading the shim to poll TDRE so Renode's
4-deep TX FIFO doesn't silently drop bytes during bursts (the polling
shim is what made `Initializing UDP bridge transport...` and other
log lines that follow a TX burst actually land). It'll matter again
if/when something in the firmware (network RX path is the likely
candidate) needs a peripheral IRQ to be dispatched. Working
hypothesis: `NVIC_ENABLE_IRQ` writes to ISER0 (`0xE000E100`) aren't
being honored by Renode's NVIC for the iMXRT, even though SysTick (a
system exception) is delivered fine.

### Suggested next experiments

#### 1. Diagnose the empty-/log-frame regression

`saint_log_emit_ros` in `firmware/teensy41/src/main.cpp:734` sets
`log_msg.data.data = log_buffer; data.size = len`, the same shape as
RP2040. Either the `log_buffer` content is getting clobbered between
the `memcpy` and the `rcl_publish` (re-entrancy?  shared `g_envelope`
/`g_text` / `g_escaped` in `saint_log.c`?), or rcl's std_msgs/String
serialization is rejecting the buffer somehow. A short-circuit
diagnostic: log the first 16 bytes of `log_buffer` right before
`rcl_publish` and a few server-side warnings on receipt.

Once /log frames carry their payload, `test_sync_recovery.py`'s
Phase 2 substring match for `"Config received"` and Phase 3's
`"Adopted node announced UNADOPTED — re-pushing peripheral config"`
should both light up — the firmware-side `saint_log_publish` calls
that emit those strings are already in place.

#### 2. PIT timer model (probably still pending)

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

#### 3. Re-examine the NVIC IRQ-delivery question (lower priority)

Only worth doing if a future stall traces back to a peripheral IRQ.
Targeted logging on the ARM SCS region (`logLevel -1 sysbus.nvic` plus
tags at `0xE000E100`/`0xE000E104`/`0xE000E180`) plus
`sysbus.nvic GetIRQs` from the monitor will tell you whether IRQs are
being recognized as enabled. If they are, look at priority/BASEPRI;
if they aren't, that's a Renode iMXRT NVIC model gap.

#### 4. Docker e2e integration (deferred)

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
| `firmware/teensy41/patch_sim_elf.py` | New post-link script (sim env only). (a) Rewrites the entry of `configure_cache`, `usb_pll_start`, `set_arm_clock`, `configure_external_ram`, `usb_init` to `bx lr` so the firmware skips hardware Renode can't model. (b) Replaces `HardwareSerialIMXRT::write9bit` with an 8-byte synchronous-direct-write shim (`ldr r3,[r0,#16]; str r1,[r3,#28]; movs r0,#1; bx lr`) so Serial output reaches LPUART6 without depending on IRQ dispatch. Uses pyelftools (already in PIO's penv). See script header for why `-Wl,--wrap=` doesn't work here (same-TU calls). |
| `firmware/teensy41/platformio.ini` | `env:simulation` `extra_scripts` now includes `post:patch_sim_elf.py`. |
| `firmware/teensy41/src/main.cpp` | Added `#include "platform.h"` right after `<Arduino.h>` so the `Serial → Serial1` redirect is in scope when `Serial.begin(115200)` is parsed. Without it, main.cpp's `Serial` resolved to the USB CDC object and LPUART6 was never initialized. |
