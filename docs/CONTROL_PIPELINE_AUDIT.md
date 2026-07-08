# Control-pipeline latency & overhead audit — 2026-07-05

Full-path audit of the streaming control pipeline (Steam Deck controller
app → Pi server → RP2040 / Teensy 4.1 / Pi nodes), covering both
latency hitches and unnecessary send-rate / CPU overhead. Follow-up to
`docs/LATENCY_REDUCTION.md`; done source-only (no robot/Deck attached),
with new local lock-down tests at every testable layer.

## Verdict in one paragraph

Tier 1 of the latency plan is genuinely in the source everywhere it was
claimed: `/control` is BEST_EFFORT + KEEP_LAST(1) end-to-end, neutral
values bypass every throttle, and both controller binding types
heartbeat. The remaining controllable latency lives in two constants
that were planned but never changed — the controller's 50 ms send
throttle and the firmware's 10 ms loop sleep — plus two Teensy-side
hot-path stalls (blocking RoboClaw ACK wait, ungated `Serial.printf` in
callbacks). On the overkill axis the system is mostly disciplined; the
one notable waste is by design (see "the 250 Hz / 50 ms pair").

## Current per-layer state (verified against source, not the plan doc)

### Controller (Rust / Tauri)

| Constant | Value | Where |
|---|---|---|
| Input/mapper loop | 4 ms (250 Hz) | `lib.rs:25` |
| Send throttle (all 4 paths) | **50 ms** | `protocol/client.rs:13` |
| Heartbeat re-emit | 500 ms | `bindings/mapper.rs:108` |
| mpsc command queue | cap 100, `try_send` | `protocol/client.rs:97` |

Verified good: stop values bypass the throttle on every streaming path;
change-detection keys are per (input, target) so tank setups can't
strand a track; estop gates streaming sends client-side; async design
keeps input processing off the network path.

**The 250 Hz / 50 ms pair (key design insight).** While a stick is
deflected, the mapper re-emits the value on *every* 4 ms tick
(`value_active`, `mapper.rs:284,374`), and the 50 ms client throttle
discards ~92 % of those. This looks like waste, but the steady re-emit
is what plugs the throttle's trailing-edge hole: a non-stop value
dropped inside the window is replaced 4 ms later, so the freshest value
always makes the next open window. **Consequence:** plan item 8
(“remove the WS-input throttle”) must NOT be done naively — it would
put 250 Hz per active axis on the WebSocket. Any throttle change has to
keep some rate cap or add mapper-side emission limiting.

### Server (Python / ROS 2)

| Item | Value | Where |
|---|---|---|
| `CONTROL_QOS` | BEST_EFFORT, KEEP_LAST(1), VOLATILE | `server_node.py:38` |
| `COMMAND_QOS` | RELIABLE, KEEP_LAST(8) | `server_node.py:44` |
| Control throttle | 50 ms per channel/endpoint | `websocket_handler.py:29`, `bridge.py:29` |
| Neutral bypass | ε = 0.02, both paths | `websocket_handler.py:1980`, `bridge.py:390` |
| Change dedup | ε = 0.005, updates **only on send** | `websocket_handler.py:1988-2012` |
| Executor | MultiThreadedExecutor | `server_node.py:2292` |

Verified good: no file I/O or INFO logging on the hot path (sampled
`_hot_log`, default WARNING level); dedup keeps a held stick's 20 Hz
identical-value stream off DDS entirely; per-channel throttle keys are
independent. The dedup cache recording only *sent* values is
correctness-critical (a throttle-dropped value must not be treated as
delivered) — now pinned by test.

Minor: each control message is JSON-parsed then re-encoded to a new
JSON string for the ROS publish (`server_node.py:1225-1243`), ~1-2 ms
on a Pi. Only worth touching if profiling shows it matters.

### RP2040

- Control sub: explicit sensor-data profile with depth=1
  (`main.c:1122-1130`) — the KEEP_LAST(5) stale-setpoint bug fix is in.
- Main loop: `spin_some(10 ms)` + `sleep_ms(10)` (`main.c:1880,1903`)
  → ~10-20 ms floor per tick. **Plan item 5 (sleep → 2 ms) not done.**
- Control callback is clean: register-level PWM/GPIO writes, Maestro
  UART writes are FIFO-bounded, `/log` drains outside callback
  dispatch, one line per tick.
- Telemetry: announce 1 Hz, state 10 Hz — reasonable, size-budgeted.

### Teensy 4.1

- Control sub: explicit depth=1 (`main.cpp:861`) — fix is in.
- Deadline-locked ~10 ms loop; watchdog budget (30 s) is comfortable.
- Maestro provisioning correctly chunked one channel per tick;
  status poll at 500 ms is cheap.
- State publish ~1200 B > 512 B MTU → fragments; already mitigated by
  dropping to 1 Hz. Pagination would allow faster state without
  wedging the executor (known, commented at `main.cpp:232-241`).

**Hot-path stalls (new findings):**

1. `roboclaw_set_duty` does `send_command()` + blocking `read_ack()`
   (`shared/src/roboclaw_driver.c:1061`, ack loop at `:286-305`).
   Healthy unit: ~1-2 ms. Unresponsive unit: a 50 ms busy-wait
   (`ROBOCLAW_RESPONSE_TIMEOUT_MS`) with no yield, per control
   message, inside the executor callback. Fix direction: fire-and-
   forget the duty write and collect the ACK in `update()` (the
   RoboClaw's serial watchdog feeds on incoming bytes, so the write
   itself is what matters), or at minimum shrink the timeout for the
   duty path.
2. Ungated `Serial.printf` in the control/command callbacks
   (`main.cpp:618,633`; several in `pin_control.cpp`). With a USB
   host attached but not draining CDC, writes block until the core's
   TX timeout (tens of ms once, then fast-fail). Same failure family
   as the Serial1-no-receiver hang. Fix: gate behind a debug flag or
   `Serial.availableForWrite()`.

### Raspberry Pi node

- Control sub: BEST_EFFORT depth=1 (`node.py:174-178`) — in.
- `rclpy.spin()` (`node.py:925`): control callback dispatches on
  arrival, not on the 10 Hz timer. Callback path is JSON parse + dict
  lookups + GPIO write, ~1-3 ms. Single-threaded executor means a slow
  *other* callback delays control by its duration — currently nothing
  in `_main_loop()`/`peripherals.update()` is slow. Keep it that way.
- RoboClaw duty keepalive only fires when duty ≠ 0, one packet per
  tick — well-behaved.

## Claims from the fan-out audit that were checked and REFUTED

Recorded so they don't resurface as folklore:

- ~~"RP2040 executor starvation: control waits extra spins behind
  config"~~ — `rclc_executor_spin_some` services every ready handle
  per spin; config traffic costs control only the callbacks' runtime.
- ~~"Pi node has a 100 ms control latency floor from the 10 Hz
  timer"~~ — control is a subscription callback under `rclpy.spin()`.
- ~~"Teensy printf can block 1 s per call"~~ — bounded by the core's
  CDC write timeout; still worth gating, but it's tens of ms once.
- ~~"Server dedup-before-throttle silently swallows values"~~ — the
  dedup cache updates only on actual send; ordering is correct.
  Pinned by `test_throttle_dropped_value_does_not_poison_dedup`.
- ~~"Preset activation takes 250 ms for 5 servos"~~ — each servo is a
  different (topic, channel) throttle key; they all pass immediately.
  The *real* preset issue is below.

## Open items, in recommended order

1. ~~**Controller throttle 50 → 20 ms**~~ — **APPLIED 2026-07-05**
   (`protocol/client.rs`, plan item 4). Note the server-side
   `CONTROL_THROTTLE_MS` (50 ms) is now the tighter window on the
   channel-addressed path; lowering it to match is the next lever.
2. ~~**Firmware loop sleep 10 → 2 ms**~~ — **APPLIED 2026-07-05**
   (RP2040 `sleep_ms(2)`; Teensy loop deadline `now + 2` — idle rate
   unchanged since spin_some's 10 ms timeout already exceeds it, only
   the post-message blind window shrinks). All four builds
   (sim + hw × both targets) compile. *Still needs reflash +
   on-hardware verify.* Fixing the Teensy SIMULATION link along the
   way surfaced that `g_transport_read_data` / `g_transport_last_rx_ms`
   (passive-RX timeout work) were defined only in the hardware
   transport; `shared/src/transport_udp_bridge.cpp` now defines and
   stamps them under `#ifdef SIMULATION`.
3. ~~**mpsc drop visibility**~~ — **APPLIED 2026-07-05**
   (`WebSocketClient::note_dropped_write()`: per-connection counters +
   warn once per 5 s window on all three streaming paths; behavior
   test-pinned).
4. **Gate Teensy callback printfs** (finding 2 above).
5. **De-block the RoboClaw ACK wait** (finding 1 above).
6. **Preset delivery guarantee**: preset activation rides the lossy
   streaming path (client throttle drop + `try_send` drop + BEST_EFFORT
   QoS) with no heartbeat covering it — a single lost packet strands
   one servo of a pose. Route one-shot poses via a reliable path or
   re-assert them briefly.
7. **Plan item 8 (drop WS-input throttle): keep parked** until it
   ships together with a mapper-side rate cap (see the 250 Hz / 50 ms
   pair).

Not worth doing: server JSON re-encode removal, orjson, TCP_NODELAY
tuning, controller UI input-state emit thinning — all sub-millisecond
or off the control path; revisit only with profiling evidence.

## Local tests added (all green without hardware)

- `server/test/test_control_throttle_semantics.py` (13 tests): neutral
  bypass, per-channel throttle windows, trailing-edge drop semantics,
  dedup-records-only-sent (the strand-regression tripwire), and the
  CONTROL_QOS/COMMAND_QOS contract.
- `controller/src-tauri/src/protocol/client.rs` (4 tests): throttle
  swallow vs stop-bypass on all three streaming paths, per-target
  window independence, estop gate — testable disconnected because the
  throttle gate runs before the connection check.
- `controller/src-tauri/src/bindings/mapper.rs` (3 tests): held
  deflection re-emits every tick (the throttle's partner), idle stick
  stays quiet inside the heartbeat window (no zero-spam), heartbeat
  re-asserts after 500 ms.

Suite status at audit time: server 242 passed / 28 skipped; controller
58 passed; `firmware/shared/tests/run_tests.sh` OK.

## What still needs the robot

Items 1-2's *feel* (deadstick-under-churn test from
`docs/LATENCY_REDUCTION.md` §Testing), the RoboClaw ACK timing against
a real unit, watchdog margins after the sleep change, and any hard
end-to-end latency number (controller send-side vs firmware
`set_channel` timestamps, target < 80 ms p99).
