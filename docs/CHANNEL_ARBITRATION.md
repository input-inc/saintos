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
- Neutral values never re-assert *for the dead-man* — but they can
  still re-engage a disengaged channel (see below).

### The liveness exceptions are not authority

Both exceptions (`idle_disengage` re-engage, motor dead-man) measure
from the **shared** last-send, because the firmware's own timers count
writes from every writer. But the idle re-engage fires **only for
operator writes** (slider, board), never for a stream.

"This channel may have gone limp" licenses a deliberate act — a slider
re-touch, a pose re-applied — to take hold again. It does not license
an unattended sheet to re-assert a parked value: that sheet did so once
per idle window (`owner=stream`, a burst every ~1.3 s on the Head Node),
so every State slider move on a channel with `idle_disengage_ms` set was
undone within a second.

**Precedence belongs to whoever is acting, not to whoever touched the
channel last.** A slider takes precedence while the operator is moving
it and not after; once released its value simply stands as the last
thing written, and any writer with something *new* to say takes the
channel immediately. An earlier version gated this rule on "does the
current owner match", which let a released slider block a sheet
indefinitely — precedence that outlived the interaction that earned it.

Note what this means in normal use: on a channel with
`idle_disengage_ms` set, the firmware releases PWM that long after the
*last write from anyone*, so a slider position is held electrically for
one window and then the servo goes limp. That is the configured
behaviour, not a fault. Channels whose position must hold want
`idle_disengage_ms: 0`.
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

## What the State slider displays

Arbitration decides what reaches the firmware. A separate question is
what the operator *sees*, and they are answered by different code.

`State.vue` renders each writable channel's slider from
`pin_state/<node_id>`, which the server builds from `NodeRuntimeState`.
That state is filled from firmware `/state` messages — and most
actuator channels never report anything. A Maestro's `/state` carries
exactly three entries (`connected`, `error_flags`, `moving`); its 24
servo channels are write-only on the wire, because reading a position
means polling the Maestro over USB, the transfer that used to wedge the
Teensy.

So every servo slider bound to `channelValues[ch.id]` was reading
`undefined`. A pose would drive a servo correctly and its slider would
sit at zero, and the operator's next drag started from the wrong place
instead of continuing from where the hardware actually was. The writes
were fine the whole time; only the feedback was missing.

`send_channel_command` therefore calls
`state_manager.record_commanded_channel` after a publish: for a
write-only channel the last commanded value is the best truth
available. Firmware readings still win — a real reading for the same
channel overwrites it through the normal `set_channel` path.

Two consequences worth knowing:

- **A suppressed write records nothing**, so the displayed value never
  claims a write that arbitration stopped.
- **Commanded values are in-memory only.** After a server restart the
  sliders have no value again until something commands each channel —
  there is genuinely nothing to restore from, since the hardware cannot
  be asked.

The slider updates when the node's next `/state` arrives (Teensy: 1 Hz),
coalesced by the `_broadcast_pin_state` throttle. Commanding a
broadcast directly would tighten that to ~200 ms, at the cost of more
frames on an AP that has already been measured saturating — so it
deliberately rides the existing cadence.

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
