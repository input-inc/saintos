# Control-pipeline latency reduction

The controller → server → peripheral movement-control pipeline had up to
**~1 second of lag on deadstick** (joystick release → motors actually
stopping). This doc tracks the diagnosis, the staged fixes, and the
testing plan.

Status as of 2026-05-25: items 1-3 + 6 implemented; items 1-3 awaiting
on-hardware verification (now possible — node firmware reflashed past
the topic split). Items 4-5 and 7-8 queued.

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

## Tier 2 — moderate wins (queued)

### 4. Shrink controller-side throttle 50 → 20 ms

`controller/src-tauri/src/protocol/client.rs:13` — `THROTTLE_MS: u64 =
50`. Bumping to 20 ms gives 50 Hz update rate, still gentle on the
link. Lower bound is the input poll rate (16 ms).

### 5. Shrink firmware main-loop sleep 10 → 2 ms

`firmware/rp2040/src/main.c` (~line 1785) and
`firmware/teensy41/src/main.cpp` (~line 747) currently do:

```c
rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
sleep_ms(10);
```

Worst-case ~20 ms from packet arrival to actuator write. Dropping the
sleep to 2 ms saves up to 8 ms per node tick. Watchdog still has
plenty of headroom. Verify on hardware before merging — the spin
budget interacts with the rest of the executor work.

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

## Tier 3 — nice-to-have (queued)

### 7. mpsc `try_send` visibility

`controller/src-tauri/src/protocol/client.rs:167` — currently:

```rust
Err(tokio::sync::mpsc::error::TrySendError::Full(_)) => {
    log::trace!("Channel full, dropped command for {}:{}", role, function);
    Ok(())
}
```

The `trace!` log is below default level so dropped commands are
invisible. Add a counter + warn-once-per-window log so this becomes
visible during testing.

### 8. Drop controller-side throttle on WS-input path

The 50 ms per-target throttle predates the routing graph. The server
side has no rate limit on `router/set_input`, and the WebSocket frame
rate is already capped by the 16 ms input poll. Removing the throttle
on `send_ws_input_value` eliminates a source of jitter.

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
