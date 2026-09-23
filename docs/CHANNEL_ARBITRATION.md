# Channel write arbitration — who wins when two controls fight

_2026-09-22. Companion to `CONTROL_PIPELINE_AUDIT.md`, which covers
latency; this one covers **correctness**: whether a commanded servo
position actually reaches the hardware._

## The rule

Three kinds of writer command a peripheral channel, and they are not
equal:

| Writer | Source | Authority |
|---|---|---|
| **Board** | pose board activation (`apply_pose`, `preview_setpoints`) | **Latched.** Overrides everything; force-sends every channel in the pose. |
| **Slider** | State tab control under the operator's hand | Owns its channel while it moves. Its value stands until the next board activation. |
| **Stream** | routing sheets — controller sticks, RC, animation player | Unattended and continuous, so rate-gated: sends only on real change. |

A parked slider is **not** a writer. It must never block a board, and
nothing re-asserts a board underneath a slider the operator just moved.

All of this lives in `server/saint_server/channel_arbiter.py`, consulted
from the one place every writer already passes through:
`server_node.send_channel_command`.

## Why it exists

There used to be two independent "I already sent that" caches:

- `websocket_handler._control_last_value` — slider writes
- `routing_evaluator._last_channel_sent` — sheet and pose dispatches

Neither could see the other, and each was cleared only by one narrow
event (a manual Sync; an e-stop release). Every other thing that moves
hardware left both caches confidently wrong:

- the *other* writer commanding the channel
- a dropped `/control` frame (BEST_EFFORT, depth 1 — nothing retries a
  servo; `_MOTOR_REASSERT_TYPES` covers motors only)
- the firmware's `idle_disengage_ms` releasing a Maestro channel
- a node reconnecting and re-initializing micro-ROS
- a Maestro re-enumerating on USB and driving every channel to home

Once a cache was wrong, the next command for that value was suppressed
as "unchanged" and the channel stayed wrong until something happened to
command a *different* number.

On the robot (Head Node, 24-channel Maestro) that read as:

- re-activating a pose board silently did nothing for exactly the
  channels last nudged by hand, and
- dragging a slider back to a value a pose had overwritten did nothing
  either.

Both reported success. `apply_pose` returned `{"success": true,
"applied": 5}` while dispatching nothing, because `applied` counted
setpoints pushed into the evaluator, not channels that reached firmware.

## What that buys

- **Re-clicking a board always re-asserts.** This is the system's only
  recovery path for a servo write lost in flight.
- **One cache, invalidated properly.** `invalidate_node` fires on node
  reconnect, config sync, and e-stop; `invalidate_peripheral` on a
  peripheral re-enumerating. A µs extent-dial jog drops the cached
  normalized value, since the servo has moved off the normalized map.
- **Honest reporting.** `set_channel_value` answers `{"unchanged":
  true}` only when the write really was redundant, and `apply_pose`
  carries `dispatched` / `suppressed` alongside `applied`.

## Rules that did NOT change

- Unattended streams stay change-gated — a 60 fps animation must not
  republish every holding channel and flood a depth-1 topic.
- Motors (`roboclaw`, `syren`) re-assert an unchanged non-neutral value
  every `MOTOR_REASSERT_MS` so the firmware dead-man has a liveness
  feed. Servos are excluded on purpose: `idle_disengage` depends on a
  held channel going quiet.
- Neutral values never re-assert.
- A suppressed write is never recorded — caching a value the firmware
  never received is how the old caches went stale.

## If you are tempted to add a per-writer cache

Don't. That is the bug. Add an owner to `channel_arbiter` instead, and
a test in `server/test/test_channel_arbitration.py` next to the ones
pinning the board-beats-slider cases.
