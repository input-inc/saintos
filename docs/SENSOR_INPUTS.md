# Sensor Inputs — Design

## Status: Implemented (2026-08-18). Not hardware-verified.

Built:

- The `switch_input` peripheral — firmware driver, per-platform pin reads,
  catalog entry.
- The local interlock (Tier 1) and the Kangaroo's per-instance `estop` /
  `clear_estop` verbs.
- Peripheral channels as routing sources (Tier 2): `InputNode.kind="channel"`,
  the evaluator's channel cache and feed, and the Routes canvas picker.

Not built: a dedicated latch-clear button on the Live card. `clear_latch` is
reachable as a `peripheral_command` in the meantime.

### Cross-sheet propagation

A targeted feed only evaluates the sheets referencing what changed. If one of
those writes a cross-sheet signal, the sheets *reading* it would not see it
until they evaluated for their own reasons — never, for a sheet with no other
live input. Since that is exactly the case signals exist to serve (a sensor on
one node driving a sink on another), `set_peripheral_channel_value` chases
signal readers after evaluating, bounded to 4 rounds so mutually-referencing
sheets can't spin.

`set_ws_input` and `set_urdf_joint_value` still have the original
eventually-consistent behaviour. They could adopt `_propagate_signal_readers`
too; not done here because those paths are hot and well-exercised, and
changing their evaluation set is a bigger blast radius than this task
warranted.

### Interlock targets

The target field is a checklist of the node's motion peripherals, driven by
two catalog flags:

- `commands_motion` — drives a motor, servo, or actuator. Set on servo, pwm,
  roboclaw, syren, maestro, pimoroni_servo2040, tic, tmc2208, kangaroo. This
  filters the list.
- `supports_interlock` — the firmware driver implements the per-instance
  `estop` / `clear_estop` verbs. **Currently only `roboclaw` and `kangaroo`.**

Motion peripherals without interlock support are shown **disabled**, with the
reason. Omitting them would look like the peripheral doesn't exist; accepting
them would build an interlock that logs "could NOT stop" at the worst possible
moment. `test_switch_input_catalog.py` greps each flagged driver for a
`.command` handler and an `"estop"` verb so the flag can't drift silently.

`servo` and `pwm` can never be targets — they are pin_control modes, not
registered peripheral drivers, so they cannot receive a `peripheral_command`.

**To make another driver a valid target** it needs: a per-unit
`peripheral_id` (roboclaw, maestro, pimoroni already store one), a latched
`interlocked` flag plus `interlock_block`, a direction guard on its motion
entry point that still permits a stop, and a `drv_command` resolving
`peripheral_id` → unit. Then set `supports_interlock=True`. Remaining:
maestro, syren, tic, tmc2208, pimoroni_servo2040.

### Direction — blocking motion *into* the switch

An end-of-travel switch that froze the axis outright would strand the
mechanism on the switch with no way off but a manual clear. So each target
carries the direction that switch blocks, and the opposite direction still
moves.

Wire format is `"<peripheral_id>:<dir>"` — `+` blocks extend/forward, `-`
blocks retract/reverse, no suffix blocks both. Sign convention matches the
control channels (positive = extend/forward/increasing).

Per *target*, not per switch, because the direction describes where the switch
sits relative to that axis's travel — which the switch itself cannot know.

Two deliberate choices:

- **No default direction.** A newly-checked target has none and the peripheral
  refuses to save until one is picked. There is no safe guess: defaulting to
  "blocks extend" on a switch that is actually at the retract end would let the
  first trip drive further *into* it. A missing or out-of-range direction on the
  wire decodes to BOTH — over-blocking is recoverable.
- **Position commands are judged against current position**, so a move that
  retreats is allowed and one that drives further in is refused. This trusts the
  last reported position; a stale reading is the known limitation of comparing
  rather than using sign alone. Speed and duty commands carry their direction in
  their sign and need no feedback.

A stop (zero) is never blocked in any direction — otherwise the interlock could
not stop anything.

### Known gap

`switch_input` declares `pin_kind="gpio"`, so the pin picker doesn't filter to
ADC-capable pins when analog sense is on. Pick GP26-29 on RP2040; anywhere
else the firmware's analog read returns false, the driver holds its last
state (deliberately — reading 0 V would look like an assert on an active-low
input), and the voltage channel stays at 0.

## Why

The Kangaroo teach-tune work needed end-of-travel limit switches, and the
obvious shortcut was to hang two pin numbers off the Kangaroo peripheral and
have its driver read them. That's wrong the moment a second thing wants the
same signal — and it usually does. An end-of-travel switch legitimately wants
to stop the actuator, disable a pose, light a status LED, gate an animation,
and show up in the log.

So the sensor should be a peripheral in its own right, and "what it affects"
should be wiring, not a hard-coded driver relationship.

## What already exists

Most of this is here; the gap is narrower than it looks.

| Piece | Status |
|---|---|
| `button` peripheral — GPIO, `pull_up` / `active_low` / `debounce_ms`, `pressed` channel | exists |
| `analog_in` peripheral — ADC pin, `voltage` channel | exists |
| Routing graph — `InputNode` → `OperatorNode` → `Wire` → peripheral-channel sinks | exists |
| `SignalNode` — named global float crossing sheet boundaries | exists |
| Peripheral channels as routing **sinks** | exists |
| Peripheral channels as routing **sources** | **missing** |
| Firmware-local input → action binding | **missing** |

`InputNode` sources are ROS `(topic, field)` pairs or URDF joints. A node's
channel state never becomes a source, so today a sensor on a node can be
displayed but not *wired to anything*. That single gap is what makes limit
switches feel like they need a private back-channel.

## The two-tier model

The important structural point: **there are two different jobs here and they
need different mechanisms.**

### Tier 1 — Local interlock (firmware, no server)

A limit switch that stops an actuator cannot depend on a server round trip.
Node → server → routing evaluator → node is tens of milliseconds on a good
day, and *infinite* when the link is down — which is exactly when a runaway
is most likely. Anything protective must be resolved on the node, by the
node, with no network in the path.

### Tier 2 — Routing graph (server)

Everything that isn't protective: triggering poses, gating animations,
lighting LEDs on a different node, driving widgets, logging. This is what the
routing graph is already for, and it's where "affects more than one thing"
naturally lives — including fan-out across nodes via `SignalNode`.

Tier 1 is a small, fixed, auditable behaviour. Tier 2 is unbounded and
operator-editable. Conflating them would mean either putting a slow network
hop in a safety path, or reimplementing the routing graph in firmware.

## Proposed pieces

### 1. A generic switch/sensor input peripheral

Neither existing type covers the real hardware. `button` is GPIO-only, and
the IDC PSR-2 reed sensor **cannot drive a 3.3 V digital pin** — its
anti-parallel diode pair drops ~1.9 V, landing between the logic thresholds
(see `KANGAROO_BRINGUP.md`). `analog_in` can read it but has no notion of a
threshold, so it produces a voltage, not an event.

What's needed is one input type that spans both:

| Param | Notes |
|---|---|
| `sense` | `digital` \| `analog` — analog is required for 2-wire sensors with a series voltage drop |
| `threshold_v`, `hysteresis_v` | analog only; hysteresis prevents chatter at the crossing |
| `active_low` | already on `button`; a normally-closed sensor asserts on *open* |
| `debounce_ms` | already on `button`; mechanical reeds bounce 0.5–2 ms |
| `latch` | hold the asserted state until explicitly cleared |

Channels: `state` (live, debounced), `latched` (sticky), and `voltage` when
`sense = analog`. Command: `clear_latch`.

**Debounce and latch must be in firmware.** A magnet sweeping past a reed is
a pulse, not a state — sample it from the server and you will miss it. This
is the same reason the Kangaroo can't use these switches on L1/L2.

### 2. Local interlock binding

The Tier 1 mechanism. On assert, the node performs a bounded local action
against peripherals *on the same node*:

- `on_trip`: `none` | `stop_targets` | `estop_node`
- `targets`: peripheral ids on this node

Firmware resolves the ids to registered drivers and calls their existing
`estop()` — the vtable entry every driver already implements, and the same
path `peripheral_estop_all()` uses. No new driver-side concept.

Deliberately limited to the local node and to stopping. Anything richer
belongs in Tier 2, where it can be inspected and edited.

### 3. Peripheral channels as routing sources

The generalising change, and useful well beyond limit switches — it makes
*every* input peripheral routable (BMS SOC gating a behaviour, a current
reading tripping a warning, a button firing a pose).

Add `InputNode.kind = "channel"` carrying `(node_id, peripheral_id,
channel_id)`, fed from the channel-addressed state the Live tab already
consumes. From there the existing graph does the rest: operators to
threshold or invert, `SignalNode` to reach other sheets, any sink.

Note the ordering trap: the evaluator's `(topic, field)` addressing could
technically reach a channel today via a `channels[N].value` path, but N
depends on emission order and would silently re-bind when a peripheral is
added. A first-class channel source addresses by id and can't drift.

### 4. UI

- Switch config in the peripheral modal (threshold + a live voltage readout
  makes setting it a two-second job instead of a guess).
- The channel appears as a source node on the Routes canvas.
- Latched state and a Clear button on the Live card.

## What this replaces

`docs/KANGAROO_BRINGUP.md` currently describes limit inputs as Kangaroo
driver params. Under this design the Kangaroo has no limit-switch config at
all: a switch peripheral names the Kangaroo in its `targets`, and anything
non-protective is wired on the Routes canvas.

## Decisions (2026-08-15)

**A new `switch_input` type**, not an extension of `button`. `button` stays
the simple digital case; `switch_input` is what you reach for when wiring a
sensor, and it can be labelled and documented as such.

**Source-side interlock binding.** The switch carries `on_trip` + `targets`;
drivers stay ignorant of switches entirely. One place to audit what a given
sensor stops, and it scales to many targets — which matches the fact that the
sensor is the shared thing.

### How the interlock reaches a driver

Reuses the `peripheral_command` routing added for the Kangaroo teach tune
rather than inventing a second mechanism: on trip, the switch driver calls
`peripheral_dispatch_command(target_id, "estop", …)` for each target, and the
owning driver handles the verb for **that instance**.

This matters because the `peripheral_driver_t.estop()` vtable entry is
driver-wide — it stops every unit that driver owns. A node with two Kangaroos
and one limit switch should not have both stop. Routing by `peripheral_id`
through the command path gives per-instance granularity with no new vtable
entry.

Drivers that don't implement the `estop` verb simply aren't valid targets,
and the attempt is logged rather than silently dropped.
