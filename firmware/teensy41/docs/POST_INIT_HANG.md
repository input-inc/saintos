# Teensy 4.1 post-init main-loop hang — handoff to testing

## What this is

The Teensy 4.1 firmware boots cleanly all the way through
`init_micro_ros()`, prints `micro-ROS initialized successfully` to USB
serial, and then **the main `loop()` produces no further output and no
network traffic for the duration of the run**. The chip stays
pingable, the agent's view of the XRCE session looks healthy, but the
firmware is functionally dead from the outside.

This is **not** the `rmw_uros_ping_agent` hang (see § Already-tried);
that one was isolated and reverted, but the symptom persists on the
reverted firmware too. So whatever this is, it's a separate bug.

The session that exposed this layered a lot of firmware changes onto
the Teensy build at once. Before the testing agent isolates the cause
in the simulator, it's worth knowing that we couldn't pinpoint it from
the hardware side because the iteration loop is slow (USB-flash to
test, ~5–10 min per cycle, and several attempts each ate a power
cycle).

## Symptoms — what you'll observe

After boot:

```
* SAINT.OS Node Firmware
* Version: 1.0.0-…
…
Initializing transport: NativeEthernet (UDP)
Transport: bringing up Ethernet (DHCP)...
Transport: DHCP attempt 1...
Transport: DHCP bound IP=192.168.10.81 gw=192.168.10.1
IP: 192.168.10.81
Discovering SAINT server...
Starting server discovery (port 8889)...
Discovery attempt 1/10...
Discovery response from 192.168.10.1: SAINT!192.168.10.1:8888
Discovered server at 192.168.10.1:8888
Transport: agent set to 192.168.10.1:8888
Initializing micro-ROS...
micro-ROS initialized successfully
```

…and then **nothing**. The 10 s periodic status print
(`[N] state: ACTIVE, agent: connected`, in `loop()` around
`firmware/teensy41/src/main.cpp:939`) never fires.

Meanwhile, on the Pi:

| Check | Result |
|---|---|
| `ping 192.168.10.81` | replies (FNET handles ICMP below firmware) |
| `ros2 topic list` for `/saint/nodes/teensy41_*` | shows announce/command/config/control/log/state |
| `ros2 topic info -v /saint/nodes/announce` | Teensy listed as a publisher, BEST_EFFORT/VOLATILE |
| `ros2 topic echo /saint/nodes/announce` | **no Teensy announces, ever** |
| Server log `[Control] SENT teensy41_… onboard_led/state: 1.000` | dashboard publish succeeds, but Teensy never acts |
| Raw UDP sniff between Pi and `192.168.10.81` | **0 bytes either direction** (ICMP excluded) |
| Agent log (`micro_ros_agent-2`) | full `establish_session` + `create_datawriter` / `create_datareader` sequence, then silence |

So the agent has the Teensy's DDS endpoints in its local view (because
the firmware DID complete `init_micro_ros` at boot), but the firmware
is not actually running its publish path after that.

## Reproduction (hardware path)

1. Build + USB-flash one of the firmware versions in § Versions.
2. Plug Teensy into the Pi (or any host serving the SAINT agent on
   port 8888 with the discovery service on 8889).
3. Wait through DHCP + discovery + `micro-ROS initialized successfully`.
4. Either passively wait 30 s, or do `sudo systemctl restart saint-os`
   on the Pi — both end in the hung state.
5. `sudo cat /dev/ttyACM0` shows no further output. `ros2 topic echo
   --qos-reliability best_effort /saint/nodes/announce` shows no
   Teensy frames.

There is no consistent symptom while in the hung state — the loop is
silent, but the chip's ICMP path and the agent's stale DDS view both
make it look "fine" from the outside if you don't look carefully.

## Versions in play

| Version | Notes |
|---|---|
| `1.0.0-1780366651` | Last firmware that ran reliably for an extended session (used `rmw_uros_ping_agent` for active liveness; reconnected after a saint-os restart in ~27 s, beating the RP2040). Verified WORKING earlier. |
| `1.0.0-1780368897` | First version that hung. Added: `mono_led` peripheral routing in `pin_control.cpp`, `analogWrite` in `led_status.cpp` for PWM brightness, separate r/g/b/brightness state tracking. Hangs after `micro-ROS initialized successfully` 100% of the time. |
| `1.0.0-1780370371` | Reverted `rmw_uros_ping_agent` back to `last_successful_comm` timeout (the original ping-was-suspected fix). Kept everything else from 1780368897. Still hangs identically. |
| `1.0.0-1781323750` (2026-06-13 build, observed 2026-06-12) | Latest. Watchdog band-aid (`firmware/teensy41/src/watchdog.cpp`) is in place and auto-resets the chip 30 s after the wedge, but the chip is now in a tight 47-second reboot loop: boot → +6 s setup → +20 s healthy → wedge → +30 s WDOG → reset → … On the Head Node with ONLY the `onboard_led` (`mono_led`) peripheral configured — no Maestro, no SyRen, no RoboClaw. All 7 driver types still register in the driver table at boot, but only `mono_led` is actually bound. |

The fact that `1780370371` hangs but `1780366651` doesn't, despite
both lacking the ping call, says the hang IS in the new code added
between them — but isolating which piece (mono_led routing /
analogWrite / overflowed-state field set) is the testing agent's job.

## Already-tried (not the cause)

Each of these was suspected, fixed, and confirmed not load-bearing
for this particular hang:

- `__WFI` idle hack in `transport_native_eth_read` + main loop tail.
  Caused a different problem (publishers wedged because RX poll was
  gated to 1 ms SysTick wakes — NativeEthernet has no ENET RX
  interrupt). Reverted to `delay(1)` and that fix is in all of the
  versions above. The hang here is different.
- `pin_config_has_mode` peripheral gate in `pin_config_load`.
  Works correctly — the relevant `Peripheral '…' has no pins in
  pin_config — skipping load_config` lines print at every boot.
- BEST_EFFORT QoS on `/log`, `/announce`, `/state`. Applied to both
  the hung and the previously-working version.
- Executor cap (`rclc_executor_init` handle count: 5 → 8). Same in
  both.
- `EEPROM.update()` vs `EEPROM.write()` in `flash_storage_save`.
  Cosmetic perf fix; not on the hot path here.
- OTA staging-erase fix in `saint_ota_perform` (pre-erase
  `[FLASH_BASE+FLASH_SIZE/2 .. FLASH_BASE+FLASH_SIZE-FLASH_RESERVE]`).
  Confirmed working — the post-OTA boot log prints
  `OTA: buffer 0x6007B000 size=7868416 bytes`. The hang isn't OTA.
- `rmw_uros_ping_agent(150, 1)` in `check_agent_connection`. Added
  in `1780368897`, ostensibly to detect the
  NativeEthernet-UDP-returns-OK-after-server-restart case. Reverted
  in `1780370371` and the hang persists. Independent of the hang
  itself, the ping appears to introduce its own loop-blocking issue
  on first call after `init_micro_ros` — see § Hypotheses 5.

## Session log — 2026-06-13 — non-reproducible builds + /log dropping

Investigated the reboot loop further. Three reference points flashed
today via Mac-side USB (Teensy connected to the dev box's USB-C, USB
flash via `~/.platformio/packages/tool-teensy/teensy_loader_cli`):

| Firmware | Source | Behavior |
|---|---|---|
| Yesterday's `1.0.0-1781323750` (built ~Jun 12 21:09 PDT, no longer on disk) | HEAD per `git log` — no commits since | **Ran to ACTIVE**, delivered the multi-line setup /log sequence (`Maestro: driver registered` / `Config received` / `Config applied OK` / `Config saved to flash`). Wedged at +20 s of loop with healthy counters at [10] and [20]. |
| Fresh build today, `1.0.0-1781378848` (clean HEAD, no local patches) | identical source to yesterday's | **Boots, micro-ROS session establishes**, but USB CDC output stops at `Pin control: initialized with 0 pins` and **only `[+6.x s] Boot — reset cause` makes it to the server's /log** — no Maestro / Config / Apply / Save lines. 50 s reboot cycle (WDOG). |
| Diag-patched build, `1.0.0-1781378338` (added `saint_log_publish` mirror of counters in loop's 2 s status block) | source + 27-line patch on `firmware/teensy41/src/main.cpp:1138`-ish | Same as fresh clean: USB output stops at `Pin control`, only `Boot — reset cause` reaches /log, diag lines never appear. |
| Jun-2 dist (`1.0.0-1780445362`, extracted from `~/.cache/saint-os/.../saint-os_0.5.0-local.998d458_arm64_kilted.tar.zst`'s `teensy41/saint_node.bin`, dated Jun 2 17:09 PDT) | ~10 days behind HEAD | **Same wedge pattern as today's fresh builds**: USB output ends at `Pin control: initialized`, 50 s cycle. |

What today's findings add to the picture:

- **Same source, different binaries.** `git log` shows no commits to
  `firmware/` since yesterday's `1781323750` build time, yet a fresh
  build from that same source today produces a binary with different
  observable behavior (no Maestro / Config / Apply /log lines).
  Build environment is suspicious:
    - `~/.platformio/platforms/teensy/platform.json` is dated **Feb
      2024** — not touched today.
    - `framework-arduinoteensy` / `toolchain-gccarmnoneeabi-teensy`
      same vintage.
    - `micro_ros_platformio @ 0.0.1+sha.cfee17f` — pinned by SHA.
  None of those obviously changed. Possible non-determinism sources:
  `FIRMWARE_VERSION_FULL` / `FIRMWARE_BUILD_TIMESTAMP` (per-build
  Unix-timestamp baked in by `generate_version.py`), `__DATE__`,
  `__TIME__`, or a build-script side-effect that mutates a tracked
  artifact.

- **/log channel drops everything after the first message per boot.**
  Only `[+6.x s] Boot — reset cause: WDOG1/WDOG2 timeout` survives;
  later setup-time publishes (Maestro register, Config receive,
  Config apply, Config save) — which yesterday's binary delivered
  reliably — are gone. Server-side `_ensure_node_log_subscriber`
  fires the moment the node announces, and yesterday's chip's
  announces clearly worked, so subscription isn't the issue. The
  drop is on the firmware side OR somewhere between
  `saint_log_drain_pending` and the agent.

- **USB CDC is unreliable for early-boot diagnostics.** Every WDOG
  reset re-enumerates USB CDC. Mac-side `cat /dev/cu.usbmodem...`
  reliably catches the boot banner through `Pin control:
  initialized` (those land in the device's TX buffer pre-host-open
  and flush on enumerate) but everything after is lost until the
  host has the port open AND the device is past whatever caused the
  buffer to stop draining. The doc's primary tactic (capture the
  last counter line before silence) requires either:
    1. The diagnostic to flow via /log instead of USB CDC (currently
       broken — see above)
    2. Serial1 wired to a separate UART (D0/D1, not connected on
       this hardware)
    3. A different chip-side instrumentation that survives USB CDC
       glitching

- **Reboot cycle period is consistent across all three firmware
  variants tested today**: 50 s from one micro-ROS session-establish
  to the next on the server side. WDOG1 fires roughly 30 s after the
  last `watchdog_feed`. The wedge timing within `loop()` looks the
  same across firmwares; only the /log delivery differs.

### Reproducer recipes (Mac-side flash + capture)

Mac-side flash from a hex (incremental rebuild):
```bash
cd firmware/teensy41
~/.platformio/penv/bin/pio run -e hardware -t upload
# When prompted "Waiting for Teensy device", press the program
# button on the Teensy (small black/white button near USB-C).
```

Mac-side flash from a bin (e.g. an OTA artifact):
```bash
~/.platformio/packages/toolchain-gccarmnoneeabi-teensy/bin/arm-none-eabi-objcopy \
    -I binary -O ihex --change-address 0x60000000 input.bin /tmp/out.hex
~/.platformio/packages/tool-teensy/teensy_loader_cli \
    --mcu=TEENSY41 -wsv /tmp/out.hex
```

Durable USB serial capture (cat-loop that reopens through WDOG
USB CDC re-enums):
```bash
stty -f /dev/cu.usbmodem<id> sane raw -echo 115200
nohup bash -c '
  while true; do
    cat /dev/cu.usbmodem<id> 2>/dev/null | while IFS= read -r line; do
      printf "%s %s\n" "$(date +%H:%M:%S.%3N)" "$line"
    done
    sleep 0.5
  done
' < /dev/null > /tmp/teensy.log 2>&1 &
disown
```

### Next-pickup hypotheses

1. **Why does /log drop after the first message today?** Compare the
   2026-06-13 binary's `saint_log_drain_pending` flow against
   yesterday's working binary — if we can find a saved copy of
   `1781323750`. Otherwise instrument to count `enqueue_pending` vs
   `saint_log_emit_ros` calls and surface those via /announce or
   Serial1.

2. **Non-determinism in the build.** Check whether two consecutive
   `pio run -e hardware` invocations from the same clean source
   produce identical `firmware.bin`. If yes, the regression is
   somewhere we haven't touched. If no, the random timestamp /
   __DATE__ / __TIME__ flow may be reorganizing memory layout enough
   to land a bug differently.

3. **Hypothesis 7 still standing.** Yesterday's session log noted the
   "All counters frozen" pattern, suggesting CPU stuck in ISR /
   `delay()` / USB CDC. None of today's reference firmwares produced
   counter trace at the moment of the wedge, so the hypothesis
   remains open but unobserved. The doc's `g_loop_iter` /
   `g_loop_stage` counters aren't visible via /log right now — fix
   that path first OR wire Serial1.



Captured the Teensy's USB CDC console directly from a Mac plugged into
the USB-B port (`/dev/cu.usbmodem193084701`) while the chip was in its
~47 s reboot loop on the Head Node. The periodic status counters
from `main.cpp:1138` (the doc's primary diagnostic lever) print
cleanly until the wedge:

```
[10] state: ACTIVE, agent: connected | loop=563  exec=563/563   tx=86/86   rx=602/602
[20] state: ACTIVE, agent: connected | loop=1563 exec=1563/1563 tx=196/196 rx=1602/1602
(silence until WDOG1 fires at ~+30 s, then reset)
```

What this rules in / out:

- **Loop is healthy through +20 s.** ~100 iterations/sec, executor
  entries == exits, transport read/write entries == exits. So the
  wedge does NOT happen in `setup()`, during config-apply, in
  `init_micro_ros()`, or in the first ~20 s of `loop()`.
- **The `[30]` print never fires.** Combined with WDOG firing at
  ~+30 s, that means the wedge starts in the +20 s … +30 s window
  AND prevents both `Serial.printf` AND `watchdog_feed()` from
  running. Per the table in "How to read the counters" below, that's
  the **All counters frozen** row — CPU stuck in an ISR, a `delay()`
  derivative, or the `usb_isr` (USB CDC). Hypothesis 7 territory, NOT
  hypothesis 2/3/6.
- **The /log ROS topic goes silent at the same time the USB CDC
  console does.** Server journal shows the four lines `Maestro:
  driver registered` / `Config received` / `Config applied OK` /
  `Config saved to flash` between +6.4 s and +6.8 s of each boot,
  then nothing until the next boot's "Boot — reset cause" line.
  Counters were healthy at +20 s on USB CDC, so both surfaces wedge
  together — this isn't a buffer fill on one transport, it's a CPU
  freeze.
- **No peripheral driver is actually bound on this config.** The
  Head Node YAML has only `onboard_led` (`mono_led`, built-in).
  `maestro_update`/`syren_update`/etc all return early when their
  `g_transport`/instance lists are empty. So `peripheral_update_all`
  is effectively a no-op every iteration here — yet the wedge still
  happens. That tightens the suspect to:
    - `led_update()` (always runs; `g_override_active` should be
      false unless an inbound /set_channel set it, which the operator
      reports they did not do)
    - `node_state_update()` (always runs)
    - `check_agent_connection()` (always runs; depends on transport
      timing)
    - the executor's inbound callback path
    - Teensy USB CDC ISR (Hypothesis 7) — host-side draining was
      active throughout the capture, so it isn't a stalled host

The "wedge starts between +20 s and +30 s" timing is the most useful
new constraint. Whatever it is, it has a startup latency around
20–25 s on this config. Things that fire at that cadence and could
plausibly wedge the chip:
- First post-boot inbound from the server beyond config? The
  `Sent config to node` happens within the first few seconds after
  the server sees the announce, so this would be a *second*
  inbound. Worth checking what /control or /state messages the
  agent buffers for the first 20 s and then delivers in a burst.
- A delayed initialization that fires the first time some condition
  is true (e.g., first time `g_node.state == NODE_STATE_ACTIVE` AND
  some other flag).
- A long-running `analogWriteResolution` / `analogWriteFrequency`
  reconfiguration that doesn't run until first set_pin / set_channel
  hits a PWM-capable pin.

Server-side reproducer to keep handy (no firmware change needed —
the watchdog auto-resets, so just leave the Head Node adopted with
only the onboard_led configured and watch the journal):

```bash
ssh opensaint.local 'journalctl -u saint-os -f --no-pager' \
  | grep -E "Head Node|teensy41_|reset cause|Maestro: driver"
```

Mac-side USB serial capture (the Teensy's USB-B end, separate from
its Ethernet link to the Pi):

```bash
stty -f /dev/cu.usbmodem<id> sane raw -echo 115200
cat /dev/cu.usbmodem<id> | while IFS= read -r l; do
  printf '%s %s\n' "$(date '+%H:%M:%S.%3N')" "$l"
done | tee teensy_serial.log
```

Next thing to try when picking this up: add a `saint_log_publish`
mirror of the diag counter line alongside the existing
`Serial.printf` at `main.cpp:1138-1149` so the same counter trace
shows up in the server's /log topic. That way the counter row right
before each wedge is captured in the SAME timeline as the server's
view (config arrival, /control deliveries, etc.) without needing
the USB cable plugged into a host. The mirror line is one
`saint_log_publish("info", ...)` with the same format string.

## Session log — 2026-06-13 PM — USB-flash + per-stage diag in setup() + /announce-borne counters

Picked this up with the chip on the Mac's USB (`/dev/cu.usbmodem193084701`)
and connected via Ethernet to the Pi (`opensaint.local`). Chip is at
`192.168.10.81`, MAC `02:e5:1d:76:5f:e9`, node_id
`teensy41_e51d765f000004e9`.

### What was confirmed

- **Mac-side USB CDC stopped capturing output entirely** after a few
  reset cycles — `cat /dev/cu.usbmodem193084701` returned 0 bytes for
  35+ s even though `/dev/cu.usbmodem*` and `/dev/tty.usbmodem*`
  both existed and `ioreg -p IOUSB` showed the device enumerated
  ("Teensyduino USB Serial", idVendor=5824). This is *worse* than
  the doc's "reliably catches through Pin control" expectation —
  the symptom now is "USB CDC enumerates but transmits nothing." It
  isn't a per-boot drop; it persisted across reset cycles and only
  changed after re-flashing.

- **The wedge is NOT in `setup()`.** Once a fresh hex was loaded
  (`build/hardware/firmware.hex`, version `1.0.0-1781382778`-ish,
  CRC32 `0x724e0078`), the chip:
    1. Boots through `setup()` in ~6.3 s.
    2. Establishes the XRCE-DDS session with the agent.
    3. Receives `/config`, applies it, ACKs via `/announce`
       (`last_config_save_ok_ms` advances → server logs
       `Config saved to flash (sync ACK via /announce, uptime_ms=540X)`).
    4. Reaches `NODE_STATE_ACTIVE`.
    5. Continues publishing `/announce` for **~3–8 s** of loop time
       after `ACTIVE`.
    6. **Goes silent.** Server's `NODE_TIMEOUT_SECONDS` fires
       (`Adopted node offline: Head Node`), WDOG1 fires ~12 s later,
       cycle repeats every ~50 s. This matches the doc's
       "wedge starts between +20 s and +30 s" of *boot uptime* —
       but expressed as wall-time-from-ACTIVE it's only **~3–8 s
       after the node first publishes**, not 13–14 s.

- **`/log` channel really does only deliver the first message per boot.**
  Confirmed via `ros2 topic echo /saint/nodes/teensy41_e51d765f000004e9/log
  --qos-reliability best_effort`: exactly one line per boot lands,
  always `Boot — reset cause: WDOG1/WDOG2 timeout` at uptime
  `~6260 ms`. The `saint_log_publish` mirror lines added at
  `main.cpp:1224-1234` never reach the server. The doc's
  hypothesis 1 still stands; the cause is somewhere between
  `saint_log_drain_pending` and the agent's relay.

### What was instrumented (uncommitted local changes)

Two diagnostic levers are in place on top of the previous session's
diff. Reviewing them is the natural starting point for the next pickup.

1. **`setup()` stage markers — `firmware/teensy41/src/main.cpp:907-1107`.**
   A new `diag_stage(n, label)` helper printing
   `[setup-stage N] <label>\n` with `Serial.flush()` + `watchdog_feed()`
   between each major `setup()` call (`watchdog_init`, banner,
   `node_state_init`, `pin_config_init`, `pin_control_init`, each of
   the seven `peripheral_register` calls, `pin_config_load`,
   `peripheral_init_all`, `hardware_init`, `led_init`, `flash_storage_init`,
   `TRANSPORT_INIT`, `TRANSPORT_CONNECT`, `discover_server`,
   `init_micro_ros`, end-of-setup). Numbered 0..23. Per-stage LED
   toggle (`(stage & 1) ? LOW : HIGH`) preserves "last successful
   stage" visibility when USB CDC is dead — LED set to OUTPUT/HIGH
   right after `Serial.begin` at `main.cpp:929-931` so this works
   before `led_init()` runs. **This made it clear the wedge is in
   `loop()`, not `setup()` — see the symptom above.** Keep or revert
   based on whether `setup()` regressions reappear.

2. **Diag counters in `/announce` JSON —
   `firmware/teensy41/src/main.cpp:566-614`.** A new `"diag"` string
   field embedded directly in the announcement envelope, mirroring
   the periodic Serial line:
   ```
   "diag":"loop=L stage=S exec=A/B tx=C/D rx=E/F logq=K drop=P emit=Q/R"
   ```
   The `logq`/`drop`/`emit*` fields surface `saint_log_emit_attempts`,
   `saint_log_emit_ok`, `saint_log_dropped`, and `g_announce_count`
   so the next pickup can tell **how full the /log queue was at the
   wedge moment** and **whether `rcl_publish` on /log was returning
   OK** — both directly address "is /log dropping locally or
   downstream?" Buffer size (`announcement_buffer[1024]` at
   `main.cpp:108`) currently fits the larger JSON; bump to 1536 if
   adding more peripherals overflows it.

### Where to pick up

The chip is currently in a steady ~50 s WDOG reboot cycle with the
diag-in-announce build (`saint_node.bin` size 507904, version
`1.0.0-1781382778`). Each boot publishes `/announce` for ~3–8 s
into `ACTIVE`, then stops. **The next step is to capture the
last announce-frame's `diag` field before each silence window.**
Two paths:

1. **Server-side subscriber.** `ros2 topic echo /saint/nodes/announce`
   does NOT receive Teensy frames in practice (FastDDS QoS quirk —
   the BEST_EFFORT/VOLATILE publisher only matches certain
   subscribers; saint_server's subscription works because it's
   created on the same DDS participant as the agent's relay). A
   standalone Python subscriber on the Pi *would* see them — a
   stub at `/tmp/teensy_dump.py` on `opensaint.local` exists but
   ran into a teardown race (`ExternalShutdownException` from
   `rclpy.spin`); rerun with `try/except` around the spin loop
   and tail the output for the `diag=` line.

2. **Server-side `_on_node_announcement` change.** A two-line patch
   to `server/saint_server/server_node.py:367-468` to log the
   `diag` field at INFO whenever the node is `teensy41_*` (or
   whenever the publishing node is in `adopted_nodes` and the
   field exists). Lands in `journalctl -u saint-os` automatically.
   Per repo memory, no server hot-patches via scp — full
   `build-local-dist.sh` + install loop is required.

Once the last-pre-wedge `diag` snapshot lands, the counter row
maps directly to the table in "How to read the counters" above to
identify which layer froze. Current best guess given the symptom
pattern (chip alive enough for ICMP to reply, FNET stack
servicing pings, but `/announce` publishes stop): a
`transport_native_eth_write` blocking on `udp.endPacket()` after
the agent loses session bookkeeping, OR `rclc_executor_spin_some`
not returning from an inbound callback — the diag row will
distinguish these.

### Notes worth carrying forward

- **`Ethernet.begin()` doesn't feed WDOG.** Inside
  `transport_native_eth_connect()` at
  `firmware/teensy41/transport/transport_native_eth.cpp:125`,
  `Ethernet.begin(mac_addr, DHCP_ATTEMPT_TIMEOUT_MS=3000)` is a
  blocking call with no watchdog feed inside. The retry loop's
  `wait_with_led` does feed during backoff, but a single
  `Ethernet.begin` that blocks longer than its 3 s nominal could
  contribute to spurious WDOG resets during DHCP-server-down
  scenarios. Not the cause of the current hang (we get past it),
  but worth feeding mid-`Ethernet.begin` if NativeEthernet exposes
  a hook.
- **USB-flash recipe that worked through the WDOG-loop chip:**
  `~/.platformio/packages/tool-teensy/teensy_loader_cli
   --mcu=TEENSY41 -wsv -v firmware/teensy41/build/hardware/firmware.hex`.
  First attempt sometimes errored with "Found HalfKay Bootloader
  ... error writing to Teensy" — second attempt right after
  succeeded.
- **`ros2 topic echo /saint/nodes/announce` does not see Teensy
  publishes** (even with `--qos-reliability best_effort`) despite
  `ros2 topic info -v` listing the Teensy as a publisher. The
  saint_server's own subscription works (we see "Node reconnected"
  / "Sent config" / "Config saved" log lines in journalctl). For
  this debug pass we relied on the saint_server journal as the
  source of truth.

## Hypotheses worth testing in the simulator

In rough order of "easiest to falsify first":

1. **Is `loop()` executing at all?** Toggle a GPIO pin (one that's
   NOT pin 13 / LED_BUILTIN) once per iteration and scope it. If the
   pin doesn't toggle, the main loop never starts running — the
   problem is between `init_micro_ros()` returning and the first
   `loop()` invocation.

2. **Is `transport_native_eth_read()` (in
   `firmware/teensy41/transport/transport_native_eth.cpp`) blocking
   indefinitely?** Add a counter for `parsePacket()` entries vs
   exits, surface via `/state`. If `entries > exits`, FNET's UDP
   read is hanging, possibly because the agent rejected something
   and FNET's stack is waiting on a reply that won't come.

3. **Is `rclc_executor_spin_some(..., 10 ms)` returning?** Same
   technique — entries/exits counters. If the executor never returns
   from spin, that's the hang point.

4. **`Serial.printf` over USB CDC is the only visibility we have on
   hardware, and the symptom is "no Serial output."** Try writing to
   Serial1 (hardware UART on D0/D1) in parallel — if Serial1 produces
   periodic prints but Serial (USB) doesn't, the loop IS running and
   only the USB CDC path is broken. That's a much smaller bug than a
   total hang and changes the response entirely.

5. **`rmw_uros_ping_agent` introduces a separate hang on first call
   right after `init_micro_ros`.** This is the second hang we hit and
   it's still un-isolated. When the testing agent is set up in sim,
   re-adding the ping path should reproduce immediately.

6. **`analogWrite(LED_PIN, …)` interaction.** The current firmware
   only calls `analogWrite` inside the LED override branch in
   `led_update()`, and at boot `g_override_active == false` so that
   branch never runs. So this shouldn't be implicated — but the
   FlexPWM peripheral could still be in a weird state from a previous
   firmware version if EEPROM/flash persist anything that affects it.
   Worth a sanity-check sim run with the `digitalWrite` version to
   confirm.

7. **`micro_ros_platformio` build-specific compile/link issue.** The
   exact same shared sources work on RP2040 (`micro_ros_raspberrypi_pico_sdk`).
   `rmw_microxrcedds_c/config.h` headers match between the two
   builds (verified — same `RMW_UXRCE_*` values). What differs is the
   compiled lib. If it comes to it, swapping the platformio dep for a
   known-good version (or building micro-ROS from source against the
   platformio firmware) might unstick this.

## Where the diagnostic levers are

In-firmware:

- `firmware/shared/src/saint_log.c` — has `g_emit_attempts` and
  `g_emit_ok` static counters, plus a `saint_log_emit_local("trace",
  g_envelope);` call inside `publish_one` that mirrors the JSON
  envelope to local serial. If the loop is running, these counters
  will tick and the trace lines will appear on Serial. If neither
  shows up, the loop isn't running.
- `firmware/teensy41/src/main.cpp` — the announce JSON includes
  `last_config_save_ok_ms` and `last_config_save_fail_ms` so the
  server can detect config-apply success via `/announce` instead of
  `/log` (in case `/log` is wedged but `/announce` isn't).
- **Post-init-hang counters (Hypotheses 1–3 made directly testable).**
  Six `g_*` volatile uint32_t counters in `main.cpp` track nested
  forward progress: `g_loop_iter`, `g_executor_spin_entries/exits`,
  `g_transport_write_entries/exits`, `g_transport_read_entries/exits`.
  Surfaced every 10 s in the periodic status print as a single line:
  ```
  [N] state: <state>, agent: <state> | loop=L exec=A/B tx=C/D rx=E/F
  ```
  Reading this line after the hang tells you exactly which layer
  wedged: see "How to read the counters" below. The increments live
  at `loop()` top, around the `rclc_executor_spin_some` call, and
  inside `transport_native_eth_read/write` (sim equivalents live in
  `firmware/shared/src/transport_udp_bridge.cpp`). Same hooks fire
  in sim and hardware, so a future sim-side reproduction will produce
  exactly the same trace shape — but **the hang doesn't reproduce in
  the Docker e2e** as of 2026-06-01 (server-restart with the Teensy
  adopted ran 70 s+ wall-time and ~870 s sim-time without hanging).
- **Serial1 hwuart heartbeat — hypothesis 4 directly testable.**
  The same status print also emits a parallel `Serial1.printf` line:
  ```
  [N] hwuart-alive loop=L exec=A/B
  ```
  `Serial1` on Teensy 4.1 is the hardware UART on D0/D1, completely
  independent of the USB CDC `Serial` device. If during a hang
  `cat /dev/ttyACM0` produces nothing but a logic-analyzer / FTDI
  cable on D0 still shows `[N] hwuart-alive ...` lines marching
  forward, the loop IS running and only USB CDC is broken — a
  totally different bug than "loop hung." Under SIMULATION the
  `Serial → Serial1` redefinition (`firmware/teensy41/include/platform.h`)
  makes these two prints both land in the same captured UART file,
  which is harmless duplication.

### How to read the counters

| Counter shape | Interpretation |
|---|---|
| `loop` frozen | `loop()` itself wedged before reaching the executor — earlier code path (`led_update`, `peripheral_update_all`, `node_state_update`, `check_agent_connection`) is the suspect. Hypothesis 6 (analogWrite/FlexPWM interaction). |
| `loop` advancing, `exec` `A=B` advancing, no `/announce` | Loop and executor are both fine; the executor is just not finding callbacks to fire OR all rcl_publish calls return RCL_RET_ERROR. Likely an agent-side bookkeeping issue (the agent silently lost the publisher), independent of the loop. |
| `loop` advancing, `exec` `A>B` (A growing faster) | Executor entered a callback and didn't return. The callback is either an inbound subscription (config / control / command) or the announce/state timer. Look for transport-read `C>D` to confirm whether it's stuck in the transport-read poll specifically (hypothesis 2). |
| `loop` + `exec` advancing, `rx` `C>D` | Transport read entered but didn't exit — FNET / `udp.parsePacket()` is blocking past its timeout. Hypothesis 2 confirmed. |
| `loop` + `exec` advancing, `tx` `E>F` | Transport write entered but didn't exit — `udp.beginPacket/endPacket` blocking. Likely a NativeEthernet TX path issue. |
| All counters frozen | Loop+executor+transport all wedged together. CPU is either stuck in an ISR, in a `delay()` derivative, or in the `usb_isr` (USB CDC). Hypothesis 7 territory. |

Off-board:

- `ros2 topic info -v /saint/nodes/announce` distinguishes
  "DDS endpoint exists" from "data is flowing".
- A raw `python3 + AF_PACKET` sniffer reliably shows whether ANY UDP
  bytes are crossing the wire to / from the node's IP. Useful for
  distinguishing "firmware silent" from "firmware sending, agent or
  server dropping."
- micro-ROS agent log lines (`micro_ros_agent-2 …
  SessionManager.hpp | establish_session`, `destroy_session`,
  `delete_client`) are in `journalctl -u saint-os` and tell you
  what the agent's bookkeeping is doing — independent of what the
  firmware thinks.

### Watchdog safety net

WDOG1 (iMXRT1062's primary hardware watchdog) is armed in `setup()`
via `firmware/teensy41/src/watchdog.cpp` with a 30 s timeout. `loop()`
feeds it at the top of every iteration (before any of the suspect
calls). If `loop()` wedges anywhere for >30 s, the chip self-resets —
**no physical power cycle needed.** Long-running calls in setup
(DHCP retry in `transport_native_eth_connect`, server-discovery in
`discover_server`) explicitly feed mid-loop so they don't trigger
spurious resets while still working.

After a watchdog-triggered reset, the boot log prints
`Watchdog: previous reset was WDOG1 timeout (SRSR=0x…)` and a /log
line `"Recovered from watchdog reset (post-init-hang protection)"`
goes out on first /announce — so the operator can see in the
dashboard that the node hit the hang and self-recovered. Override
the timeout at build time with `-DSAINT_WATCHDOG_TIMEOUT_S=N` (range
1..128 s).

Disabled under `SIMULATION` (no chip-reset semantics to model in
Renode; node_manager handles sim lifecycle).

## Recovery while debugging

The 30 s WDOG1 (above) auto-resets the chip when `loop()` actually
wedges, so a power cycle is no longer required in the common case.
The watchdog covers:

- Hard hang in `loop()` or anywhere it calls (peripheral_update_all,
  rclc_executor_spin_some, transport read/write blocking past the
  timeout, etc.) — loop stops feeding, 30 s later chip resets.
- Wedged init paths in `setup()` — DHCP retry + server discovery feed
  mid-loop so they don't trigger spurious resets while making
  progress, but DO trigger a reset if they themselves wedge.

The watchdog does NOT cover the silent-session-loss case:
`/command restart` won't reach a wedged loop, and the
`last_successful_comm` reconnect heuristic never fires because
NativeEthernet UDP's `udp.endPacket()` always returns success even
when the agent has no session for this client. The firmware happily
keeps publishing into the void; `loop()` is alive, watchdog is fed,
no reset. (Fixing this is the "session-liveness without
ping_agent hang" follow-up — needs a real round-trip liveness check
to detect the dead session.)
