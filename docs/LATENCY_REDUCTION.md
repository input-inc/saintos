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
