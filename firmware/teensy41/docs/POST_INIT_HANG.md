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
