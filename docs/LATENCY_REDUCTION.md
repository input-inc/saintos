# Control-pipeline latency reduction

The controller → server → peripheral movement-control pipeline had up to
**~1 second of lag on deadstick** (joystick release → motors actually
stopping). This doc tracks the diagnosis, the staged fixes, and the
testing plan.

Status as of 2026-07-05: items 1-7 implemented in source (items 4, 5, 7
landed with the control-pipeline audit — see
`docs/CONTROL_PIPELINE_AUDIT.md`). Items 4-5 need a node reflash +
on-hardware verification. Item 8 is PARKED — see its section; removing
the throttle without a mapper-side rate cap would put ~250 Hz per
active axis on the socket.

**2026-08-10 second round — see "Round 2" at the bottom.** With all of
the above flashed and deployed, RoboClaw deadstick still ran 1-2 s when
the stick was rotated in circles. Root causes were two FIFO queues the
release-zero could not jump (controller shared mpsc(100); server's
sequential router/set_input receive path, which never got the July
protections) plus no dead-man anywhere — the firmware duty keepalive
actively re-fed the RoboClaw's serial watchdog with the last non-zero
duty forever. All three layers fixed in source, locally tested.

## The pipeline (where latency can hide)

```
Steam Deck                              Server (Pi)                            Node (RP2040/Teensy/Pi5)
────────────                            ────────────                           ─────────────────────────
 Gamepad poll (16 ms)
       ↓
 HID merge (16 ms, parallel)
       ↓
 InputMapper.process (16 ms loop) ──→  router/set_input  OR  set_topic_channel
       ↓                                       ↓                       ↓
 WS client throttle (50 ms/target)      routing_evaluator         50 ms throttle
 deadstick bypasses                     → dispatch_sink           (had bug: buffered
       ↓                                → send_channel_command      but never flushed)
 tokio mpsc (cap=100, try_send)                ↓                       ↓
       ↓                                ROS publisher: was         ROS publisher (same)
 WebSocket frame ─────────────────────► RELIABLE + KEEP_LAST(10)   ────► UDP via
                                        now BEST_EFFORT + depth 1        micro-ROS agent
                                                                              ↓
                                                                       rclc_executor_spin_some (10 ms)
                                                                              ↓
                                                                       sleep_ms(10)
                                                                              ↓
                                                                       apply to actuator
```

## The diagnosis

Three issues compounded to produce the ~1 s lag:

1. **ROS2 QoS was wrong for streaming control.** The control topic
   used the rclpy default profile, which is RELIABLE + KEEP_LAST(10).
   For a fire-and-forget joystick stream, RELIABLE is the wrong choice
   — a single lost UDP packet stalls the queue while DDS retransmits,
   and a return-to-zero ends up queued at position 10 behind nine
   stale non-zero values.
2. **`set_topic_channel` throttle had a deadstick hole.** Throttled
   values were merged into a per-topic buffer but no flush was
   scheduled. If the controller's next push came >50 ms later (or
   never), the buffered deadstick sat there indefinitely.
3. **DifferentialDrive bindings had no heartbeat.** DirectControl had a
   500 ms re-emit so a dropped deadstick packet was retried, but
   DifferentialDrive's send path was fire-and-forget — a single lost
   frame on return-to-zero stranded both tracks at the last commanded
   velocity until the operator nudged the stick again.

The 500 ms heartbeat on DirectControl is what bounded the worst case
to ~1 s; without it the motors would have stayed running indefinitely.

## Tier 1 — biggest wins (done, awaiting test)

### 1. Split `/control` (BEST_EFFORT) from `/command` (RELIABLE)

Streaming peripheral writes (set_pin, set_channel) go on
`/saint/nodes/<id>/control` with BEST_EFFORT + KEEP_LAST(1) QoS so the
deadstick is always the freshest thing at the head of the queue.
Operator one-shots (factory_reset, restart, identify, estop,
firmware_update, roboclaw_debug) moved onto a separate
`/saint/nodes/<id>/command` topic with RELIABLE + KEEP_LAST(8) so a
single dropped UDP packet doesn't silently lose an estop or an OTA
trigger.

Files:
- `server/saint_server/server_node.py` — `CONTROL_QOS` and
  `COMMAND_QOS` declared at module scope; `_ensure_node_control_publisher`
  uses `CONTROL_QOS` instead of `depth=10`.
- `firmware/rp2040/src/main.c` — control subscription switched to
  `rclc_subscription_init_best_effort`.
- `firmware/teensy41/src/main.cpp` — same.
- `firmware/raspberrypi/saint_node/node.py` — added a `_qos_control`
  (BEST_EFFORT + depth=1) profile; control subscription uses it,
  everything else stays on `_qos_reliable`.

Compatibility caveat: nodes flashed before the `/control` ↔ `/command`
split don't subscribe to `/command`, so operator one-shots silently
no-op on them until they're re-flashed once over BOOTSEL/USB. After
that, OTA self-update keeps them current.

### 2. Deadstick bypass on `set_topic_channel`

`server/saint_server/ros_bridge/bridge.py` now defines
`NEUTRAL_EPSILON = 0.02`. The throttle check reads
`if not is_neutral and now - last_publish < CONTROL_THROTTLE_MS` so
near-zero values bypass the throttle and publish immediately. Mirrors
the existing `is_neutral_value` pattern in
`server/saint_server/webserver/websocket_handler.py`.

### 3. DifferentialDrive heartbeat

`controller/src-tauri/src/bindings/mapper.rs` — DifferentialDrive now
uses the same `heartbeat_due` + `last_send_times` pattern DirectControl
has, keyed on the left channel (left + right are always sent together
so one timer covers both tracks). Re-uses the existing `HEARTBEAT_MS =
500` constant.

## Tier 2 — moderate wins

### 4. Shrink controller-side throttle 50 → 20 ms (done 2026-07-05)

`controller/src-tauri/src/protocol/client.rs` — `THROTTLE_MS` is now
20 ms (50 Hz per target). Lower bound is the 4 ms input loop; the
server-side `CONTROL_THROTTLE_MS` (50 ms) is now the tighter window on
the channel-addressed path, so a follow-up there is the next lever if
more rate is wanted.

### 5. Shrink firmware main-loop sleep 10 → 2 ms (done 2026-07-05, needs reflash)

`firmware/rp2040/src/main.c` — `sleep_ms(10)` → `sleep_ms(2)` after
the executor spin. `firmware/teensy41/src/main.cpp` — the loop-pacing
deadline (`now + 10`) → `now + 2`; when spin_some idles its full 10 ms
timeout the deadline is already met, so idle loop rate is unchanged —
only the post-message blind window shrinks. Watchdog budgets (500 ms
RP2040 / 30 s Teensy) are untouched. Verify deadstick feel on hardware
after reflashing.

### 6. Hot-path file logging (done)

Three operator-visible streaming lines — `set_ws_input`,
`set_channel` in `router/routing_evaluator.py`, and
`set_topic_channel` in `ros_bridge/bridge.py` — now go through a
sampled `_hot_log()` helper that emits 1-of-20 during steady streams
but always logs the first message after a ≥500 ms idle gap, so
binding-fire-up is still immediate in the live log. The mid-pipeline
per-operator eval trace (`routing_evaluator.py` around L272) was
demoted to DEBUG since it's an internal step, not operator-facing.
Net effect: at 50 Hz streaming the file handler sees ~2.5 lines/s
instead of ~50, while the operator still sees a binding light up the
moment it starts firing.

Constants `_HOT_LOG_SAMPLE_N = 20` and `_HOT_LOG_IDLE_MS = 500.0` live
in each module at the top of file — tune there if the sampled rate
turns out to be too sparse for a particular debugging session.

On top of the sampling, the server now defaults to **`logging.level =
WARNING`** in `config/server_config.yaml` (see `LoggingConfig` in
`config/__init__.py`). The new module `log_level.py` applies that to
every `saint_os.*` Python logger plus the rclpy `saint_server` logger
on startup (`server_node.py`) and live whenever `set_settings`
includes a `logging` block (`webserver/websocket_handler.py`). The
Logs page in the dashboard has the dropdown — flipping it to INFO /
DEBUG re-enables the sampled streams immediately, no restart. With
this in place the per-tick lines are effectively a debug-time tool:
sampled when on, fully gated when off.

**Verifying the win locally.** `server/scripts/bench_hot_log.py`
exercises the routing-evaluator hot path under four configurations
(`before` = INFO + no sampling, `level-only`, `sampling-only`,
`current`) and prints ops/sec, µs/op, and lines/bytes written. Run it
on the dev box and on the Pi to confirm the actual delta — on a
laptop it's typically ~2.5× throughput over the unmodified baseline.
What it does NOT measure: the rclpy emit path or the DDS publisher,
both of which are real costs in production. If the bench shows no
win, the work has moved somewhere else and the diagnosis needs to
re-start.

## Tier 3 — nice-to-have

### 7. mpsc `try_send` visibility (done 2026-07-05)

All three streaming send paths now route queue-full drops through
`WebSocketClient::note_dropped_write()`: a per-connection counter plus
a warn line rate-limited to once per 5 s window (`DROP_WARN_INTERVAL_MS`)
that carries the burst size. Counters reset on `connect()`. Behavior is
pinned by `dropped_write_warns_once_per_window_but_counts_every_drop`.

### 8. Drop controller-side throttle on WS-input path (PARKED — do not do naively)

The 2026-07 audit (`docs/CONTROL_PIPELINE_AUDIT.md`) found the mapper
re-emits held deflections on every 4 ms tick (`value_active`), which
the per-target throttle relies on to plug its trailing-edge drop — and
which would hit the socket at ~250 Hz per active axis if the throttle
were removed. Only ship this together with a mapper-side rate cap.

## Testing plan

After flashing/deploying each tier:

1. **Smoke test:** verify nodes still adopt and basic peripheral
   writes still work (a single slider should still drive a servo /
   motor as before).
2. **Deadstick under input churn:** rapidly waggle the joystick for a
   few seconds, then release sharply. Both tracks should come to rest
   within ~80 ms (was: up to ~1 s).
3. **Deadstick on flaky Wi-Fi:** repeat #2 with the Deck on the edge
   of WiFi range, or with a deliberate `tc netem` packet-loss rule on
   the link. The motors should still come to rest — the heartbeat
   covers the case where the single zero packet is dropped.
4. **One-shot commands on the `/command` topic** (Tier 1 only):
   factory_reset, restart, identify, estop, firmware_update.
   Re-flash any pre-split node via BOOTSEL once before testing, then
   verify each click lands.
5. **Latency measurement:** if you want a hard number, instrument the
   send-side (controller log) and the receive-side (firmware
   `set_channel` log) with timestamps, then look at the wall-clock
   delta on the same printout for the same value. Aim for <80 ms p99.

---

# Round 2 — 2026-08-10: the circle-stick 1-2 s run-on

With everything above flashed, RoboClaw deadstick still ran on 1-2 s,
worst when the stick was rotated in circles. Circles defeat every
dedup/throttle in the chain (both axes change every 4 ms tick on
multiple targets ≈ 200 msg/s) — and the release-zero had two FIFO
queues it could not jump, plus no dead-man behind it.

## Diagnosis

1. **Controller shared FIFO (primary).** All streaming sends went
   through one `mpsc::channel(100)` drained one-at-a-time by the same
   tokio task that services inbound telemetry. The stop bypassed the
   20 ms throttle but NOT the queue; under Wi-Fi backpressure it either
   waited behind up to 100 stale deflection frames or — because the
   channel drops **newest** when full — the stop itself was discarded.
2. **Server router path never got the July protections.** The gamepad
   path is `router/set_input` (not `control`): no throttle, no neutral
   bypass, an unsampled per-tick INFO log line, and an awaited JSON ack
   back to the Deck for every axis tick, all inside the strictly
   sequential `async for` receive loop.
3. **No dead-man + keepalive worked against the stop.** Nothing zeroed
   motors on control silence, and the firmware re-sent the last
   NON-zero duty every 400 ms forever, defeating the RoboClaw's own
   ~1 s serial watchdog. A lost zero frame = indefinite run-on.
4. **Secondary: blocking serial.** Every duty write busy-waited up to
   50 ms for its ACK inside the executor callback (audit open item 5),
   and the 2026-07-19 fault poll (cmd 90) burned up to 2×50 ms per
   cycle on units that don't answer it — same UART as the duty writes.

## Fixes (all in source, all locally tested)

- **Controller — latest-wins send slots**
  (`controller/src-tauri/src/protocol/client.rs`): streaming sends now
  coalesce into one slot per target (`StreamCoalescer`) with a token
  channel waking the writer. Max queue depth = number of active
  targets; a stop replaces its target's stale value instantly and can
  never be dropped by backlog. One-shots (estop, discovery, wifi,
  library) keep the reliable FIFO. 8 new cargo tests.
- **Server — router path cheapened + motor re-assert**
  (`webserver/websocket_handler.py`, `router/routing_evaluator.py`):
  `router/set_input` logs at debug; successful set_input no longer
  sends a per-tick ack (handler returns None; errors still respond).
  The evaluator change-gate re-asserts unchanged NON-neutral values
  every 300 ms for motor types only (`roboclaw`, `syren`) — the
  liveness feed for the firmware dead-man; servos stay fully
  change-gated (idle_disengage). 14 new pytest tests
  (`test_router_drive_path_semantics.py`).
- **Firmware — fire-and-forget duty ACK, status-poll backoff, dead-man**
  (`firmware/shared/src/roboclaw_driver.c`): duty writes no longer
  block on the ACK; `drain_duty_acks()` collects them in update() and
  telemetry defers while one is in flight (control > telemetry). The
  cmd-90 fault poll backs off 10 s per unit after 3 failed cycles.
  **Dead-man:** a unit at duty ≠ 0 with no external setpoint for
  1250 ms (`ROBOCLAW_DEADMAN_MS`) is zeroed and logged; this runs
  BEFORE the keepalive, so a dead link now stops the motor instead of
  keeping it alive. Same dead-man mirrored in the Pi driver
  (`ROBOCLAW_DEADMAN_S`). 9 new C tests + 3 new Pi tests.

Sizing note: the dead-man window (1250 ms) > server re-assert cadence
under a held stick (~500 ms, gated by the controller heartbeat) with
one lost BEST_EFFORT frame of margin. If the heartbeat or re-assert
constants change, re-check this inequality — tripping mid-hold is the
failure mode to avoid.

Worst-case run-on after these fixes: normal path stops within one
send-slot drain (<~50 ms); a lost zero is bounded by heartbeat retry
(500 ms, now unclogged); total link death is bounded by the firmware
dead-man (1.25 s) even though the keepalive exists — previously
unbounded.

## Round 2 verification

- Local: controller `cargo test --lib` (66), server `python3 -m pytest
  test` (306 net of 4 pre-existing servo2040-related failures),
  `firmware/rp2040/tests/run_tests.sh` (46 roboclaw), Pi
  `python3 -m pytest tests/` (241), all four firmware builds.
- On hardware, repeat tests 2-3 above; additionally: drive, then kill
  the server process mid-deflection — motors must stop within ~1.3 s
  (dead-man) instead of running until reconnect.

---

# Round 3 — 2026-09-22: the delay was downstream traffic, not the control path

Same operator report as Round 2 — the track drives lag behind a stick
rotated in circles, and keep rotating after deadstick, "as if messages
are queued up". This time the whole chain was already current, so the
cause had to be somewhere Rounds 1-2 never looked.

## What was ruled out first

Before changing anything, every layer's *deployed* version was checked
against source on the live rig:

- Server: `md5sum` of the installed `routing_evaluator.py` and
  `websocket_handler.py` matched source exactly. Round 2's router
  re-assert and no-ack were live.
- Nodes: all four RP2040s reported `fw=1.2.0-1790137825`,
  `built=2026-09-22 21:30:25` on `/saint/nodes/announce` — the current
  build, dead-man and fire-and-forget ACK included.
- Firmware serial: `RoboClaw wire` stats showed ~77 pkts/s of telemetry
  with `resp=382 ok / 0 short / 0 crc_bad` and zero ACK timeouts. The
  wire was healthy and duty writes were not blocking.
- Zero `DEAD-MAN` events in the journal, so no lost-zero run-on either.

So the control path itself was behaving. The measurement that broke the
case open was timing the *arrival* of control messages rather than their
processing.

## The measurement

From the server journal's microsecond ROS timestamps, over 90 s of
driving:

| quantity | p50 | p90 | p99 | max |
|---|---|---|---|---|
| `set_input` inter-arrival | 3.1 ms | 54.6 ms | **324 ms** | **463 ms** |
| `set_input` → `Sent channel control` (in-server) | 5.0 ms | 29 ms | 53 ms | 160 ms |

The arrival gaps were the problem — but dumping every log line *inside*
the six largest gaps showed only routine announcements, keepalives and
temp probes. **The server was idle during those stalls.** The delay was
therefore upstream of the server: in the air.

Then, on the socket itself:

```
server -> Steam Deck:  ~76,000 B/s   (sustained, steady)
Steam Deck -> server:   ~6,330 B/s   (the control stream)
```

The server was pushing **12x more data at the controller than the
controller sent it**, continuously.

## Root cause

`SaintServerNode._broadcast_pin_state` fired on every `/state` message a
node published, and each broadcast carried that node's *complete* runtime
state — every pin, every channel, ~1.9 KB of JSON. RP2040 nodes publish
at 10 Hz (`ros2 topic hz` confirmed 9.98 Hz) and there are four of them:

    4 nodes x 10 Hz x ~1.9 KB = ~76 KB/s

which is exactly what the socket showed.

Why that wrecks control latency: the Deck is a station on the Pi's own
2.4 GHz AP (`wlan0`, channel 9, 20 MHz, single radio). That radio is
strictly half-duplex — every frame the Pi transmits downstream is airtime
the Deck cannot use to transmit the next joystick setpoint. A steady
600 kbit/s downstream is trivial as *bandwidth* and ruinous as *latency*:
the control stream gets squeezed into clumps separated by the observed
300-460 ms gaps. During a circle the tracks execute a stale rotation; on
deadstick the release-zero is stuck in the same clump.

Nothing in the control path reads that broadcast — `update_pin_actual`
has already run and the routing evaluator consults the state manager
directly. It is display data, and it was being delivered at the
publisher's rate rather than at any rate a UI needs.

## Fixes

- **`_broadcast_pin_state` is coalesced per node**, leading edge plus
  trailing edge, at `_PIN_STATE_INTERVAL_S = 0.2` — the same pattern
  `_broadcast_routing_values` already used. Ceiling drops from 40
  frames/s to at most 2 per node per window. The trailing edge is
  load-bearing: without it the last frame of a burst never lands and a
  gauge sticks on an intermediate value, which on a motor channel means
  the dashboard shows the robot driving after it has stopped.
  The window is `websocket.pin_state_interval_ms` in
  `server_config.yaml` (default 200; 0 restores the old behaviour),
  because the right value depends on the link and is not knowable from
  source — raise it if control still feels laggy, lower it if gauges feel
  steppy. Measured effect at the default, with 4 nodes at 10 Hz: 40
  frames/s down to 24, so ~76 KB/s down to ~46 KB/s. That is a real cut
  but only 1.7x; the remaining cost is that every frame is still the
  WHOLE node state, so sending per-channel deltas is the next big lever
  (it needs a merge on the consuming side, so it is a cross-layer change).
  `server/test/test_pin_state_broadcast_throttle.py` (18 tests).
- **The throttled-control branch no longer broadcasts.** It had sent a
  full-node-state frame to advertise a desired value it had deliberately
  *not* forwarded to the node.
- **Broadcast fan-out no longer serialises behind one slow client, and no
  longer holds `self._lock` across network I/O.** A stale controller
  connection was found wedged at 413,689 bytes in `Send-Q`, not draining,
  keepalive timer 116 minutes out — a peer gone without a FIN. Writes to
  it blocked forever, so it held the lock that also guards client
  registration, while every broadcast task (`create_task`, ~50/s under
  stick motion) piled up behind it unbounded. Now: resolve recipients
  under the lock, release, `asyncio.gather` with a per-client
  `BROADCAST_SEND_TIMEOUT_S = 2.0`. A send that blocks past the deadline
  closes that client rather than being retried — `wait_for` cancels the
  write and may already have emitted a partial frame, so the stream
  cannot be trusted afterwards; closing forces a clean reconnect.
  `server/test/test_broadcast_fanout.py` (11 tests).

## Measured but NOT fixed — next levers

- **The server runs at `logging.level: DEBUG`** (deliberately — logs must
  stay available for field diagnosis). At idle that is ~80 lines/s, and
  **2,709 of 4,573 DEBUG lines per 60 s are the per-axis-tick
  `[WS] router: set_input params={...}` line** — the one whose own
  comment says an INFO line there "runs synchronous file I/O inside the
  sequential receive loop and directly slows the drain that keeps
  deadstick fast". At DEBUG it runs anyway, plus two `eval ...` lines per
  evaluated op. This is the likely bulk of the 29 ms p90 / 160 ms max
  in-server latency above. Fixing it without losing information means
  moving the log I/O off the event loop (queue + writer thread), not
  logging less.
- **2.4 GHz channel 9, 20 MHz** for the control link. Moving the Deck to
  5 GHz, or to a dedicated AP, addresses the airtime contention directly
  rather than just reducing what we put in it.
- **The Pi's clock is ~4 months off** (reported 2026-05-25 while files
  installed that day were stamped 2026-09-22). NTP is not syncing, which
  makes cross-layer log correlation unreliable — every timing figure
  above had to be derived from monotonic ROS stamps and relative deltas.
- A node rebooted mid-session (`[+5.8s] RoboClaw: probe complete`),
  unexplained.

## Round 3 verification

- `python3 -m pytest server/test` — 679 passed, 28 skipped.
- Not yet verified on hardware: the traffic reduction should be
  re-measured with the same `ss -tni` sampling (expect ~76 KB/s to fall
  to roughly a quarter), and the arrival-gap percentiles re-derived from
  the journal, before calling this fixed.

# Round 4 — 2026-09-23: the node could not consume setpoints fast enough

Same operator report again — circle the stick, release, the tracks replay
the stale rotation. Rounds 2 and 3 were both still intact in source
(verified by diffing the whole control path against `065af91`, the last
commit before the window the operator reported as good), and neither was
at fault. This time the bottleneck was on the node.

## Where the queue actually was

`rclc` takes **one message per subscription per `spin_some()`**. The
RP2040 main loop called `spin_some` once per iteration, so the loop period
was a hard cap on `/control` intake. Measured before the fix:

| quantity | value |
|---|---|
| main loop period | **~157 ms** (design: 2-12 ms) |
| `/control` consumed | ~6.4 setpoints/s |
| `/control` produced | ~50/s |
| publish→apply delay, start of gesture | 20 ms |
| publish→apply delay, end of gesture | **3555 ms** |
| run-on after release | **~3.5 s** |

Two things that look like they should have prevented this, and did not:

- **depth-1 QoS does not cover this hop.** It governs the *agent's* DDS
  subscriber, and the agent keeps up with DDS fine. The backlog was one
  hop further downstream, in the agent→node XRCE stream queue.
- **The RoboClaw dead-man (1250 ms) cannot fire.** Every queued stale
  non-zero re-stamps `last_setpoint_ms` as it is finally consumed, so the
  window never elapses until the queue has drained. The dead-man protects
  against a *lost* zero, not a *late* one.

## Fixes

- **The executor is drained, not spun once** (`firmware/rp2040/src/main.c`).
  Bounded on both count and time — `CONTROL_DRAIN_MAX_PASSES` 16 and
  `CONTROL_DRAIN_BUDGET_MS` 20 — so a fast publisher can never starve the
  watchdog pet, the dead-man check, or peripheral updates. Only the first
  pass may wait (10 ms); later passes poll with a zero timeout, because
  `spin_some` blocks for its full timeout when there is no work and
  re-probing with 10 ms would add a stall to every iteration that received
  anything. This makes the queue self-limiting: whatever arrived is
  consumed in the iteration it arrived in, so run-on is bounded by one
  loop period instead of by how long the operator kept moving.
  Per-channel latest-wins still falls out naturally, while other channels
  on the same topic are each still applied — which a "keep only the newest
  message" shortcut would have dropped.
- **A permanent main-loop profiler**, dumping avg+max per phase
  (`periph` / `net` / `exec` / `log`) plus `ctrl rx` and `drain_trunc`
  every `LOOP_PROFILE_INTERVAL_MS` (5 s). The drain bounds the
  *consequence*; this measures the *cause*, which was the signal missing
  while this was being chased from the server side.

## Why the operator was still seeing it

**The Round 4 firmware had never reached the robot.** On 2026-09-23 the
server's staged artifact was still built from `443d81e`, and both
RoboClaw nodes (`rp2040_48405f4f3d28`, `rp2040_5857c7555f34`) reported
running exactly that. The fix existed only in the repository.

Deploying it is two steps, and the first is easy to forget: stage the
build into `/opt/saint-os/firmware/rp2040/` (`saint_node.bin` is what the
OTA bootloader fetches; `generated/version.h` is where the server reads
the version it offers), then trigger the per-node OTA. Verify with
`management/check_firmware_update` — it compares the node's reported
`version_full` against the server's, and the build-timestamp suffix is
what actually distinguishes two builds of the same commit.

## Measured after deploying

All four RP2040 nodes, at rest:

```
loop 74-88 Hz  avg 9.3-11.4 ms  max 18 ms
periph 0-2/5   net 0/1   exec 8-10/18   log 0/1   drain_trunc=0
```

So the loop is healthy at rest and the drain is never truncating. The
~9 ms in `exec` is the design idle cost: `spin_some`'s 10 ms wait with
nothing to do, plus the 2 ms pacing sleep.

**The 157 ms is still unattributed, and the profiler has already narrowed
it to the wrong phase from what was guessed.** While two nodes were
pulling OTA images over HTTP through the Pi, the other two logged
`exec 9/284` and `exec 10/282` — a single `spin_some` taking **282-284 ms**
with `periph` still at 0-1 ms. That points at the executor phase, not at
blocking peripheral UART: a callback or an XRCE publish stalling while
the network is congested. The `/state` publish (~1.9 KB of JSON per node)
is the obvious candidate to look at first.

Next measurement, and the one that actually closes this out: drive the
robot in circles and watch `exec` max and `drain_trunc` during the
gesture. At rest proves nothing about the loaded case, which is the only
case the operator ever complained about.

## Not the cause: the Kangaroo telemetry poll

`kangaroo_update()` ran a blocking 9600-baud Get on **every** main-loop
iteration — no cadence gate on the connected path, `read_byte()` a
busy-wait with no yield, and `read_packet_reply()` allowing 50 ms for the
first byte plus a further sync deadline at 10 ms/byte. It is the right
shape for this defect and it was investigated as the cause, but **no
Kangaroo is configured on any node** (checked against
`/etc/saint-os/nodes/*.yaml`), so `unit_count == 0` and the function
returns on its first line. It cost nothing.

Gated anyway, at `KANGAROO_POLL_INTERVAL_MS` (100 ms), because it is a
live trap for whoever configures the first one: position and speed feed a
UI gauge on a slow actuator and the node's own `/state` only publishes at
10 Hz, so a per-iteration poll buys nothing. Contrast `roboclaw_update`,
which returns early while duty ACKs are outstanding and so yields its
UART during active driving — this driver had no equivalent. Deliberately
still un-gated: the tune branch (it carries its own keep-alive interval,
and its jog dead-man must be evaluated every iteration or a dropped link
pins an open-loop axis against a hard stop) and the per-unit reprobe,
which already has `KANGAROO_REPROBE_INTERVAL_MS` and is what bounds an
absent-but-not-yet-dropped unit.

The decision is split into `poll_due()` so it can be tested at all:
`kangaroo_update`'s body is inside `#ifndef SIMULATION` and the host test
runner builds with `-DSIMULATION=1`, so the function itself is compiled
out there. Three tests in `firmware/rp2040/tests/test_kangaroo_driver.c`
pin the interval boundary, that telemetry resumes (a cadence limit, not a
mute), and that the unsigned subtraction survives the ~49.7-day
`PLATFORM_MILLIS()` wrap.

## Still open

- **Attribute the loop period under load** — the `exec` spike above.
- **The server runs at `logging.level: DEBUG`** and the per-axis-tick
  `[WS] router: set_input params={...}` line still does synchronous file
  I/O inside the sequential receive loop, which its own comment says it
  must not. Carried over from Round 3; fixing it means moving log I/O off
  the event loop, not logging less.
- **The router drive path has no server-side rate limit at all.**
  `_handle_control` throttles at `CONTROL_THROTTLE_MS` (50 ms); the
  gamepad `router/set_input` path only change-gates, so a node can be
  asked to swallow whatever the controller emits. A per-(node, channel)
  cap there would bound this independently of how fast any firmware runs.
- **2.4 GHz channel 9, 20 MHz** for the control link (Round 3).
- **The Pi's clock is still wrong** — it reported May 25 while installing
  files on 2026-09-23. All timing here came from monotonic ROS stamps and
  node uptimes.

## Round 4 verification

- `python3 -m pytest server/test` — 741 passed, 28 skipped.
- `firmware/rp2040/tests/run_tests.sh` — 7 suites green, including
  `test_kangaroo_driver` at 60 tests (57 + the 3 new cadence tests).
- `firmware/rp2040/build.sh hw` and `sim`, and `firmware/teensy41/build.sh hw`
  all build clean. (`kangaroo_driver.c` is not compiled into the Teensy
  image, so the gate is RP2040-only in practice.)
- Deployed and confirmed on hardware: both RoboClaw nodes report
  `1.2.0-1790204269`, and the staged `saint_node.bin` md5 matches the
  local build byte for byte.
- **Not yet verified: the run-on itself.** The drain is deployed and
  `drain_trunc=0` at rest, but nobody has driven the robot since.

# Round 5 — 2026-09-23: the server's set_input path was the queue

Round 4 deployed a firmware executor drain and a loop profiler, and the
operator reported **no change**. The profiler is what made the next step
cheap: it showed the nodes were fine (74-88 Hz, `drain_trunc=0`), so the
backlog had to be upstream of them. It was in the server.

## The measurement

Driven from a script **on the Pi itself, with the Steam Deck controller
disconnected** — so the radio, the Deck, and Wi-Fi airtime are all out of
the picture. Rotation injected through the same `router/set_input` path
the controller uses: 4 inputs at 40 Hz = 160 msg/s for 3.0 s, then
release-zeros, then silence. Observable is the RoboClaw's own `current`
and the commanded `motor` value out of each node's `/state`.

```
sent:       480 set_input over 3.0 s   (160/s)
processed:  480 set_input over 8.99 s  (53/s)   <-- the ceiling
publishes:  430 to the two track nodes over 8.77 s
```

The motors executed the **entire** stale trajectory: at **+6.6 s** after
the stick stopped, `motor` was still swinging through the injected
sinusoid (0.48 → -0.52 → 0.35 → -0.38), only reaching 0.000 at **+6.77 s**
— even though release-zeros were sent during +0 to +1.0 s. They were at
the back of the queue. This is the operator's report, exactly: "the motors
continue to move until it finishes all the commands it was sent."

## Root cause

`_handle_router`'s `set_input` called `state_manager.push_ws_input`, which
called `RoutingEvaluator.set_ws_input`, which **evaluated the owning sheet
synchronously on the websocket receive loop** — sheet evaluation, the
arbitration gate, the ROS publish, `record_commanded_channel`, and a log
line, per message. That is ~19 ms, so the path tops out at **~53 msg/s**.

What makes it pathological rather than merely slow is the arrival rate.
The controller's mapper re-emits **every** binding on a 150 ms heartbeat;
with 7 bindings that is a **46.7/s floor — 88% of the ceiling before the
operator touches anything**. Each *moving* axis may then emit up to 50/s
of its own (the client's 20 ms per-target throttle). So any real stick
input pushes arrival past service rate, and the excess queues in the
websockets receive buffer, unbounded. There was no rate limit anywhere on
this path: `_handle_control` throttles at `CONTROL_THROTTLE_MS`, the
gamepad path only change-gated, which cannot help when every value
differs.

Note which earlier conclusions this corrects:

- It is **not** the agent→node XRCE queue (Round 4). That queue was real
  but secondary; the node was never the constraint here.
- It is **not** downstream telemetry airtime (Round 3). This reproduces
  with the controller disconnected and the loopback interface only.
- The firmware dead-man still cannot save it, for the Round 4 reason: the
  backlog keeps *delivering* setpoints, each re-stamping
  `last_setpoint_ms`, so the 1250 ms window never elapses.

## Fix

Split absorbing a value from acting on it.

- `RoutingEvaluator.stage_ws_input()` validates and caches the value and
  marks the sheet staged. No evaluation. Same validation as before, so a
  mis-bound axis still returns an error and stays diagnosable.
- `RoutingEvaluator.flush_staged()` evaluates each staged sheet **once**
  and emits one `routing_values` broadcast for the whole flush.
- `_handle_router` stages, then arms a single `WS_INPUT_FLUSH_MS` (20 ms)
  timer. Every message landing inside the window collapses into that one
  evaluation, latest-value-wins per input. One timer at a time; anything
  staged *during* a flush gets its own following window rather than
  extending the current one, so a saturated stream still yields to the
  receive loop.

Latest-wins is the correct collapse and matches the rest of the stack —
`/control` is depth-1 newest-wins for the same reason, and an intermediate
joystick sample is meaningless once a fresher one exists. 20 ms sits under
the nodes' own ~13 ms loop period, so the added latency is not observable.

## Measured after the fix

Same script, same amplitude, same duration, hot-patched server:

| | before | after |
|---|---|---|
| set_input processed | 480 over **8.99 s** (53/s) | 511 over **3.76 s** (**136/s**) |
| last publish vs. stick stop | **+6.77 s** | **~+0.0 s** |
| `motor` reaches 0.000 | **+6.77 s** | **+0.29 s** |
| RoboClaw current quiet | +7.7 s | **~+1.2 s** |

The residual +0.29 s is mostly instrumentation: the 25 ms send interval,
the 20 ms window, and `pin_state` telemetry itself being coalesced at
200 ms. The remaining current decay to ~+1.2 s is motor coast and the
RoboClaw's own ramp, not commanded motion.

## Found on the way, not fixed

- **A server restart trips node watchdog resets.** Four
  `Recovered from watchdog reset — main loop hung` across the nodes,
  17 s after the drive and interleaved with
  `Config saved to flash (sync ACK ...)` — i.e. the config-sync push on
  reconnect blocks the firmware main loop long enough to trip WDOG. One
  of those nodes carries a Maestro, which is the known shape of this (the
  provisioning sweep must yield, one channel per `maestro_update()`).
  Worth its own round; it will look like a random node reboot in the field.
- **The controller's 46.7/s heartbeat floor.** Harmless against a 136/s
  ceiling, but it is pure airtime on the Deck's half-duplex link (Round 3)
  and the obvious next reduction.
- **The RoboClaw encoder reads 0** — nothing is wired to it, so `encoder`
  is not a usable observable. `current` is.
- The DEBUG `set_input` log line is still on the receive loop (Round 3).

## Round 5 verification

- `python3 -m pytest server/test` — **747 passed**, 28 skipped. Six new
  tests in `server/test/test_router_drive_path_semantics.py` pin the
  property that matters: 100 received messages cost ONE evaluation, the
  flush uses the freshest value, per-sheet coalescing still drives both
  track sheets, only one timer is armed at a time, input staged during a
  flush is not stranded, and a failed stage arms nothing.
- Verified on hardware by the table above.
- **Deployed as a HOT-PATCH ONLY** (operator's call, for the test):
  `routing_evaluator.py`, `state_manager.py`, `websocket_handler.py` under
  `/opt/saint-os/install/lib/python3.11/site-packages/saint_server/`,
  originals saved in `/opt/saint-os/hotpatch-bak/`. **A formal
  dist build + install is still owed**, or the next install silently
  reverts the fix.
