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
