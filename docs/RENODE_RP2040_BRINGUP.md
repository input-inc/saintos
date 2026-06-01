# Renode RP2040 sim bring-up — handoff

> **Looking to just run firmware tests against the simulator?** See
> `firmware/simulation/docker/README.md` for the standard runbook.
> This doc is the historical record of the bring-up + a snapshot of
> where the work currently sits, including the bugs we already found
> and don't want re-introduced.

## Goal

Get the dockerised end-to-end test (`firmware/simulation/docker/`) to
boot the real saint_node firmware in Renode and pass
`firmware/simulation/test_sync_recovery.py` in `MODE=full`. That lets
developers iterate on firmware changes without flashing hardware.

`MODE=fake` (Python stand-in firmware) already works — but it doesn't
exercise the C firmware's micro-ROS executor, so it can't catch
real firmware regressions. `MODE=full` is what we want green.

## Where things stand right now

### What works

- **Apple Silicon support.** Dockerfile picks the right Renode arch
  via `TARGETARCH` and pulls a native arm64 build for Apple Silicon,
  amd64 elsewhere. No more Rosetta / Mono crashes
  (`Assertion at mini-amd64.c:217, amd64_is_imm32 (disp)`). See
  `firmware/simulation/docker/Dockerfile.e2e`.
- **Renode 1.16.1 .NET-based** — bundled runtime, no host Mono
  dependency, native arm64. Pinned in
  `firmware/simulation/docker/docker-compose.e2e.yml`.
- **Upstream peripheral sync.** `matgla/Renode_RP2040` is still
  actively maintained (last commit 2026-03-22). Our snapshot was
  months behind. Synced 13 peripheral C# files from upstream HEAD
  to pick up their "fixed bug that was randomly stopping SSI
  clocking thread" and "Renode 1.16.0" commits. Files updated:
  - `emulation/externals/w25q16.cs`
  - `emulation/peripherals/clocks/{rp2040_clocks,rp2040_rosc,rp2040_xosc}.cs`
  - `emulation/peripherals/dma/{rpdma,rpdma_engine}.cs`
  - `emulation/peripherals/gpio/rp2040_gpio.cs`
  - `emulation/peripherals/i2c/rp2040_i2c.cs`
  - `emulation/peripherals/memory/memory_alias.cs`
  - `emulation/peripherals/pio/rp2040_pio.cs`
  - `emulation/peripherals/spi/{rp2040_spi,rp2040_xip_ssi}.cs`
  - `emulation/peripherals/watchdog/rp2040_watchdog.cs`
- **W25Q model patches re-applied** on top of synced upstream, marked
  with `SAINT patch:` comments for future upstreaming:
  - NOP handling for `0x00` / `0xFF` operation bytes (RP2040 bootrom
    sends these as continuous-mode reset / dummy clocks during flash
    wake-up).
  - JEDEC ID returns `EF 40 15` (Winbond W25Q16JV) instead of
    `00 00 00`. Without this the bootrom thinks no flash is present.
  - Silent FinishTransmission when CS goes high in
    `RecognizeOperation` state (normal protocol, not an error).
- **I2C constructor signature fix.** Upstream's `RP2040I2C` now
  takes `gpio` and `id` parameters. Updated
  `firmware/rp2040/simulation/renode_rp2040/cores/rp2040_simple.repl`
  i2c0 block to pass them.

### What's still broken

**Renode + agent are now end-to-end happy.** The remaining gap is
in the firmware/server announcement plumbing, not the simulation
infrastructure.

Working:
- Bootrom → boot2 → user firmware (all green)
- All 7 peripherals registered (`maestro`, `syren`, `fas100`,
  `roboclaw`, `pathfinder_bms`, `tic`, `tmc2208`)
- UDP bridge sends to `127.0.0.1:8888`
- **micro-ROS XRCE-DDS session established with the agent.** Agent
  log shows `create_client → establish_session → create_participant
  → create_publisher × N → create_datawriter × N → create_subscriber
  × N → create_datareader × N`.
- Firmware reaches state `BOOT → UNADOPTED`, watchdog armed, main
  loop running (`[10] Node running, state: UNADOPTED, agent: connected`)
- Server receives the firmware's log topic — e.g.
  `[saint_server]: [+3.153s] Boot OK — fw 1.2.0-... watchdog armed
  at 8000 ms` shows up in `/tmp/server.log` (now also captured to
  `firmware/simulation/logs/server.log`).

Still off (firmware/server logic, not Renode):
1. **Node ID mismatch.** Firmware self-IDs as `rp2040_000000000000`
   (unique-ID register reads zero in sim); test harness passes
   `rp2040_synctest` to `node_manager create`. Persistent storage
   was empty on boot so the firmware fell back to the unique-ID
   path instead of picking up the test's node_id.
2. **Server's `list_unadopted` stays empty** even though the
   firmware reports `State: BOOT → UNADOPTED` and the server is
   receiving its log topic. The announcement → server discovery
   plumbing isn't bridging through.

### How the original agent crash was solved

The agent (from `microros/micro-ros-agent:jazzy`, published
2025-09-11) was linked against **libfastrtps.so.2.14.5** and
**libfastcdr.so.2.2.5**. `ros:jazzy-ros-base`'s apt feeds now ship
**.6 / .7** of these libs, which:

- (a) subtly break the Fast-DDS SHM transport's stack-frame layout
  — so the agent aborts with `*** stack smashing detected ***` on
  the first received packet, and
- (b) drop a `fastcdr` symbol that the agent's typesupport libs
  require — `undefined symbol:
  _ZN8eprosima7fastcdr3Cdr9serializeEj` once a session establishes.

Fix landed in `Dockerfile.e2e` + `entrypoint.sh`: copy the bare
agent image's entire `/opt/ros/jazzy/lib` to a sidecar at
`/opt/uros_libs`, and prepend that to `LD_LIBRARY_PATH` only when
starting the agent. The apt-installed libs stay in place for the
saint_os colcon build + saint_server runtime — only the agent's
process inherits the matched set. Also: bypass `ros2 run` and
invoke the agent binary directly so the ros2 CLI doesn't drag in
ROS2 typesupport libs that were built against the newer fastcdr.

### Secondary, non-blocking

- **The unique-ID is reading zero.** `Node ID: rp2040_000000000000`
  means the firmware's hardware-ID source (probably the W25Q's
  READ_UNIQUE_ID instruction or RP2040 chip ID) is returning zeros.
  Fix in `w25q16.cs` (add the `0x4B READ UNIQUE ID` opcode) and/or
  the RP2040 SYSINFO peripheral. Compounding issue: the test passes
  a node_id to `node_manager create rp2040_synctest` but the
  firmware never picks it up — its persistent-storage backing comes
  up empty on boot and it falls back to the unique-ID path.

### What was previously broken (now fixed)

The original symptom in earlier passes of this doc was "bootrom
hangs in `flash_put_get`". That was a **misdiagnosis** caused by
two compounding issues in `node_manager.py:_generate_rp2040_resc`
and `_start_rp2040`:

- **`cpu0 MaximumBlockSize 1` + `LogFunctionNames true` made the
  CPU run ~100×–1000× slower than normal.** With block size = 1,
  Renode's translator recompiles every instruction; combined with
  per-instruction symbol logging, a 2048-iteration busy-wait
  (`subs r3, #1; bne 0x2430` inside `flash_exit_xip`) took a
  measurable fraction of wall time and looked like a hang. With
  these stripped, the bootrom finishes in milliseconds.
- **`--console` mode + `stdin=subprocess.DEVNULL` made Renode
  exit immediately after `start`.** Renode's monitor reads stdin;
  `/dev/null` returns EOF on the first read; the monitor exits
  and disposes the machine ~100 ms after `Machine started`. That
  cut the CPU off before the bootrom even reached user code.
  Fixed by dropping `--console` for the background invocation
  path — `--disable-xwt` already implies `HideMonitor`, so there's
  no monitor to feed stdin to.

The "no SSI activity in xip_ssi log" reading was also misleading:
`rp2040_xip_ssi.cs` only emits `Log()` calls in `RecalculateFrequencies`
(Debug) and the `SSIENR` write callback (Noisy). Actual `DR0` /
status-register accesses go through register value-fields with no
logging, so absence of those lines told us nothing about whether
the bootrom was actually driving SSI.

## How to reproduce locally

Prerequisites: Docker Desktop with the compose plugin.

```bash
# 1. Build the sim firmware (host).
cd firmware/rp2040 && ./build.sh sim && cd build_sim && make install_sim

# 2. Run the dockerised e2e in MODE=full (Renode-based).
cd /Users/hackman/Projects/OpenSAINT/SaintOS/source
MODE=full docker compose -f firmware/simulation/docker/docker-compose.e2e.yml \
    up --build --abort-on-container-exit --exit-code-from e2e

# 3. Inspect Renode operational + UART logs.
cat firmware/simulation/logs/rp2040_synctest.log         # Renode-side log
cat firmware/simulation/logs/rp2040_synctest.uart.log    # firmware printf stream
```

Expected red-state output (current):

```
[setup   ] ✓ creating sim node rp2040_synctest
[setup   ] ✓ starting node in Renode
[setup   ] ✓ connected + authenticated to server
[phase 1 ] ✓ waiting for node to announce as UNADOPTED…
[phase 1 ] ✗ node never announced — agent or sim failure
```

The UART log now goes all the way through boot, peripheral
registration, UDP bridge open, micro-ROS session establishment,
ROS2 node creation, and `State: BOOT → UNADOPTED`. The remaining
gap is announcement bridging — see `What's still broken` above.

Expected green-state output:

```
[phase 1] ✓ node announced
[phase 2] ✓ adopting node
[phase 2] ✓ firmware acknowledged Sync to Node
[phase 2] ✓ config persisted to flash
[phase 3] ✓ server detected divergence + pushed config
[phase 3] ✓ node healed itself end-to-end
=== all phases passed ===
```

`MODE=fake` should continue to pass at any point during this work —
it's an unrelated path and gives a sanity check that the harness +
server stack itself is healthy.

## Suggested next experiments

### 1. Wire the test's node_id into the firmware

The harness creates `rp2040_synctest` via
`node_manager.py create rp2040_synctest`, but the firmware ignores
that and self-IDs as `rp2040_000000000000` from its unique-ID
register. Two paths:

- (a) Have `node_manager.py create` write the node_id into the
  persistent-storage backing (`rp2040_storage/<id>.bin`) so the
  firmware picks it up on boot via the storage peripheral, OR
- (b) Pass the node_id through a Renode CLI parameter / env var
  the firmware reads at startup.

Path (a) matches the production behavior (real boards persist
their node_id to flash) and dovetails with how the harness already
sets up `persistent_storage` in the resc.

### 2. Bridge announcements server-side

Server log shows it receives the firmware's `/saint/nodes/<id>/log`
frames fine, but `list_unadopted` keeps returning empty. The
firmware reaches `UNADOPTED` and presumably publishes to either
`/saint/announcements` (DDS) or the discovery UDP channel on port
8889 — confirm where, and verify the server's subscription /
discovery listener is plumbed to its unadopted-list state. The
parallel work in `server/saint_server/server_node.py` is in this
area.

### 3. Fix the unique-ID reading zero

`Node ID: rp2040_000000000000` indicates whatever the firmware
reads for its unique ID (probably `flash_get_unique_id()` /
W25Q's `0x4B READ UNIQUE ID` opcode, or the RP2040 chip ID
register at `0x4000C040` SYSINFO) returns all zeros. Adds noise
to logs even after (1) is solved, since real hardware has a
unique ID baked in.

## Structural follow-up (mentioned but out of scope for this work)

`firmware/rp2040/simulation/renode_rp2040/` is mostly a vendored
snapshot of `matgla/Renode_RP2040`. Only a small surface is genuinely
saint-specific:

- `boards/adafruit_feather_rp2040.repl`, `boards/initialize_adafruit_feather.resc`
- `cores/initialize_peripherals_simple.resc`, `cores/rp2040_simple.repl`
- `emulation/peripherals/network/` (UDP bridge for micro-ROS transport)
- `emulation/peripherals/storage/` (persistent flash backing)
- `run_*.resc` orchestration scripts

Everything else is just an old copy of upstream and is why we shipped
local hacks for things upstream had already fixed. A reasonable
follow-up is to:

1. Move saint-specific files to a `saint_extensions/` directory.
2. Have the Dockerfile clone `matgla/Renode_RP2040@<pinned-sha>` to a
   known path at image-build time.
3. Update `node_manager.py:_generate_rp2040_resc` to emit two
   `path add` lines (one for each).
4. Delete the vendored snapshot.

That keeps us close to upstream automatically and shrinks our repo
by a few thousand files. Not blocking the bring-up; do it after the
bootrom is unblocked so we don't churn the file paths mid-debug.

## Diagnostics — when to add and when to strip

`node_manager.py:_generate_rp2040_resc` is currently clean of perf-
killing diagnostics. The two settings that previously masked the
real failure mode were:

```
cpu0 MaximumBlockSize 1     # forces JIT to retranslate every instruction
cpu0 LogFunctionNames true  # one log line per instruction with symbols loaded
```

Combined, they slowed the CPU by ~100×–1000× and made an ordinary
busy-wait look like a hang. Don't re-add them together; if you
need a function trace, do it **without** `MaximumBlockSize 1` so the
log is per-basic-block instead of per-instruction.

Targeted peripheral logging is fine and cheap:

```
logLevel -1 sysbus.udp_bridge   # for the current micro-ROS investigation
logLevel -1 sysbus.uart0        # if you want UART byte-level activity
```

Strip these before merging once the sim is green.

## File map of changes in this branch (Renode-related only)

| File | What changed |
|---|---|
| `firmware/simulation/docker/Dockerfile.e2e` | (a) Multi-arch Renode pick via `TARGETARCH`; Renode 1.16.1 .NET (no Mono apt deps); arm64-native on Apple Silicon. (b) Stash bare `microros/micro-ros-agent:jazzy`'s `/opt/ros/jazzy/lib` to `/opt/uros_libs` so the agent can load its linked-against Fast-DDS lib versions at runtime (the apt-installed 2.14.6/2.2.7 crash it on first packet). (c) Conditional `--break-system-packages` for pip3 (forwards-compat with humble fallback). |
| `firmware/simulation/docker/docker-compose.e2e.yml` | Dropped `platform: linux/amd64` pin; bumped `RENODE_VERSION` to 1.16.1; documented ROS_DISTRO must match the Pico SDK branch. |
| `firmware/simulation/docker/entrypoint.sh` | (a) Trap copies `/tmp/{agent,server,fake_firmware}.log` to the host-mounted logs dir on exit, so post-mortem of the agent/server is possible. (b) Dropped `exec` for the harness invocation so the trap actually fires. (c) Start agent via direct binary invocation with `LD_LIBRARY_PATH=/opt/uros_libs:…` rather than `ros2 run` (avoids dragging in the apt-installed typesupport libs that demand the 2.2.7 fastcdr symbol). |
| `firmware/rp2040/lib/micro_ros_raspberrypi_pico_sdk` | Submodule branch switched from `humble` → `jazzy` so the firmware's XRCE-DDS client matches the agent's version. |
| `firmware/rp2040/simulation/renode_rp2040/emulation/externals/w25q16.cs` | Synced from upstream; layered SAINT patches (NOP for 0x00/0xFF, JEDEC ID, silent CS-with-no-op). |
| `firmware/rp2040/simulation/renode_rp2040/emulation/peripherals/{clocks,dma,gpio,i2c,memory,pio,spi,watchdog}/*.cs` | Synced from matgla upstream HEAD (`205a5e4` as of 2026-03-22). |
| `firmware/rp2040/simulation/renode_rp2040/cores/rp2040_simple.repl` | I2C0 instantiation updated for new ctor signature (`gpio: gpio; id: 0;`). |
| `firmware/simulation/node_manager.py` | (a) Dropped `--console` from the background Renode invocation — `--disable-xwt` already implies `HideMonitor`; with `--console` the monitor read EOF from `stdin=DEVNULL` and disposed the machine ~100 ms after start. (b) Removed `MaximumBlockSize 1` + `LogFunctionNames true` + `logLevel -1 sysbus.{xip_ssi,uart0}` diagnostics that were slowing the CPU by orders of magnitude and masking real progress as a hang. |

Sync-config-bug related changes (separate work item — see
`docs/SYNC_CONFIG_REGRESSION.md`) are in
`firmware/simulation/test_sync_recovery.py` (the `--existing-node`
mode). Those should stay regardless of how Renode bring-up lands.
