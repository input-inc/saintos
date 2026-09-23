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
| **Stream** | routing sheets — controller sticks, RC, animation player | Unattended and continuous, so rate-gated: sends only when **its own** value changes. Never re-asserts because someone else moved the channel. |

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
- **A pose sets a channel once.** After the activation the operator's
  slider holds it, because the pose value the evaluator keeps
  re-offering is gated as an unchanged stream write.

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

## Two questions, not one

The subtlety that cost a second round of dead sliders: "has this
changed?" is not one question, and the writers do not ask the same one.

- **Board / slider — "does the hardware already hold this?"** They
  compare against `_Entry.value`, the shared record every writer
  updates. That is what lets a slider command a value a pose
  overwrote, and it is the whole point of having one cache.
- **Stream — "has *my* value changed since *I* last sent it?"** It
  compares against `_Entry.stream_value`, which only a stream write
  updates. A board or slider write deliberately leaves it alone.

Gate a stream against the shared value and it re-asserts every time
anyone else writes the channel. That is not hypothetical: a pose
activation does **not** end when the fan-out returns.
`apply_animation_frame` writes the setpoints into the evaluator's
`_urdf_joint_values` / `_ws_input_values`, which persist, and every
later sheet evaluation re-dispatches them down the same sinks — only
the activation itself runs inside `dispatch_as(BOARD)`, so all the
re-dispatches arrive as ordinary `STREAM` writes. With the wrong
reference, each one saw "hardware holds -0.35, I want 0.40, that's a
change" and snapped the servo back to the pose on the next tick. Every
slider nudge was undone within milliseconds of the operator releasing
it.

The *timing* exceptions stay on the shared record on purpose:
`idle_disengage` and the motor dead-man both count writes from anyone,
so they measure from `_Entry.sent_ms`, not from the stream's own last
send.

## If you are tempted to add a cache

One shared record of what the firmware holds — that part is not
negotiable, and re-introducing a private "already sent that" cache per
writer is the original bug (see the two caches above). `stream_value`
is not that: it is not a second opinion about hardware state, it is a
stream's record of its own output, which no one else can know.

Anything new goes in `channel_arbiter` as an owner, with a test in
`server/test/test_channel_arbitration.py` beside the board-beats-slider
cases, and — if it touches the dispatch loop — one in
`test_router_drive_path_semantics.py::TestPoseDoesNotStompTheOperator`,
which drives the real evaluator.
