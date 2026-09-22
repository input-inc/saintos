# The robot model: URDF + SRDF + rig file

SAINT.OS describes a robot with three files. This is the operator-facing
walkthrough; `docs/RIG_SCHEMA.md` is the reference for the third one.

| File | ROS parameter | Owns | Standard? |
|---|---|---|---|
| `robot.urdf` | `robot_description` | Links, joints, limits, geometry, anchor frames. **Every name.** | Yes |
| `robot.srdf` | `robot_description_semantic` | Joint groups and named poses (`<group_state>`) | Yes (MoveIt) |
| `robot.rig.xml` | `robot_description_rig` | On-screen controls, blend curves, bindings | Ours |

**The URDF owns every name.** The other two are annotation layers, and
every joint, link, and pose they mention is a dangling reference without
it. That has one consequence worth internalising: **an unresolved
reference is a silent no-op, not a load error.** An SRDF naming a joint
you renamed still loads — it just quietly stops moving that joint. So
Settings → Robot Model lists every unresolved reference it finds, and
that panel is the only place they surface.

Three files is not a compromise; it's the convention. A standard MoveIt
config is six to eight files, and the ecosystem's unit of packaging is a
config *package*, not a document. The SRDF is already published as
`robot_description_semantic` alongside `robot_description`, and the rig
file follows that precedent exactly.

Start from `server/resources/examples/example.srdf` and
`example.rig.xml`.

---

## Units: the one thing to get right

**Everything inside SAINT.OS is normalized −1..+1**, across each joint's
URDF `<limit>`. Not radians.

- Normalized `0` is the **midpoint of a joint's travel**, not its native
  zero. A joint with `lower="-0.5" upper="0.7"` has its native 0 at
  normalized `-0.167`.
- `scale="0.35"` on a rig `<drive>` means "35% of this joint's travel".

Native units (radians / metres) appear in exactly two places:

1. **SRDF `<group_state>` values**, which are authored native and
   converted once, on import. The import dialog shows both columns side
   by side so you can check it against your SRDF.
2. **URDF `<mimic>`**, whose `multiplier` is defined in native units and
   so round-trips through the limits. Applying it to normalized values
   would be wrong for any two joints with different travel.

A radian value that reaches a −1..+1 sink unconverted commands a servo
straight to its stop, which is why nothing in this pipeline passes an
unresolvable joint through — it's dropped and reported instead.

---

## 1. Upload the URDF

**Settings → Robot Model → URDF → Upload.**

Either a bare `.urdf`/`.xacro`, or a `.zip` containing the URDF plus a
`meshes/` subdirectory — and optionally the SRDF and rig file, so one
upload installs the whole set.

Meshes are stored under their **URDF-relative** paths, so two files
sharing a basename in different folders stay distinct. (This was a real
bug: johnny5 ships `SimpleMouth/static_97a3da.stl` *and*
`SimplifiedHead2/static_97a3da.stl` with different geometry. Flattening
made one render twice and the other vanish.)

Replacing the URDF **keeps** any SRDF and rig file already installed,
then re-validates them. A re-export of the geometry shouldn't delete your
rig — but if a joint got renamed, expect warnings.

> URDF and SRDF share the same `<robot>` root element and are
> indistinguishable by tag. Uploads are classified by **content**, so a
> `robot.srdf.xacro` or a bare `model.xml` still routes correctly, and an
> SRDF uploaded to the URDF slot gets told what happened rather than
> being installed as the robot's structure.

## 2. Upload the SRDF, and import its poses

**Settings → Robot Model → SRDF → Upload.**

An SRDF `<group_state>` is a named list of joint values — which is
exactly what a pose is. On upload you're asked whether to import them:

- Both unit columns are shown per joint (authored native, stored
  normalized). Expand a row to check the conversion.
- Poses that **already exist are unchecked by default**, and overwriting
  is a separate opt-in. If you tuned an imported pose by hand,
  re-uploading the SRDF won't quietly discard that — and the dialog says
  so when it detects local edits.
- Joints that don't resolve against the URDF are dropped and listed, so
  you know a pose landed partial.

Reachable again later from the SRDF card ("Import group states as
poses…"), so a revised SRDF is a two-click re-import.

Imported poses carry `source: "srdf"`, and behave like any other pose:
usable on the soundboard, as animation pose tracks, and as rig control
targets.

## 3. Upload the rig file

**Settings → Robot Model → Rig file → Upload.**

Validate first — it's much easier than reading a load error:

```bash
xmllint --noout --schema server/resources/schema/rig-1.1.xsd your.rig.xml
```

The XSD catches structural mistakes; the server additionally cross-checks
every joint, link, and pose reference against the URDF and SRDF, which no
XSD can do. See `docs/RIG_SCHEMA.md` for the element reference.

---

## Animating with poses

### Pose tracks

In the animation editor, **+ Pose** adds a whole named pose as one track.
Its curve is the pose's **weight** (0…1), not a joint value.

### Pose tracks are clips

A pose track contributes **only between its first and last keyframe**.
Outside that span it contributes nothing — it does *not* hold its last
value forever the way a joint track's curve does.

That's what makes handing off between poses automatic. Give `happy` a
span over 0–2 s and `mad` a span over 2–4 s and mad simply takes over,
because happy's clip has **ended** — not because mad sits higher in the
list. **Order only decides where two spans actually overlap.** Once no
clip covers the playhead, those joints fall back to neutral.

> A pose track with a **single keyframe** has a zero-length clip and so
> does essentially nothing. That's why **+ Pose** creates two keys, why
> the disclosed rows draw the clip extent as a bar, and why a one-key
> pose track shows a ⚠ badge.

### Layering

Within an overlap, value tracks layer bottom-up:

- a **pose track** lerps the joints its pose names from the accumulated
  value toward the pose's value, by its weight;
- a **joint track** hard-sets its one joint (and *does* extrapolate
  outside its keys — clip semantics are for pose tracks only);
- a **ws-input track** writes to the routing graph and never touches
  joints.

So a joint track *above* a pose track overrides it, and one *below* is
overridden by it. Drag the handle at the left of a row to reorder.

Two properties worth knowing:

- A pose at **weight 0 contributes nothing** — it doesn't zero its
  joints. That's what lets a pose fade out without fighting the track
  below it.
- A pose that **doesn't name a joint leaves it alone** rather than
  centring it. Poses are sparse by nature; a missing joint means "don't
  care". That's what lets a `blink` pose layer over any expression.

A track pointing at a deleted pose contributes nothing, so the editor
shows a "missing pose" badge on the viewport rather than letting it be
silently inert.

### Refining one joint inside a clip

Click the **disclosure triangle** on a pose track to reveal the joints it
affects. Each gets its own row, and the row is keyable:

- **Locked anchors** (hollow diamonds) sit at the pose track's own
  keyframe times. The pose owns those, so you set them on the pose row
  above — the anchor value is the pose's contribution at that time,
  `neutral + (target − neutral) × weight`.
- **Your keys** (solid diamonds) go anywhere between them. Click an empty
  spot on the row to add one, then:
  - **drag sideways** to retime (clamped to the clip — a key outside it
    would never be evaluated),
  - **drag up/down** to change the value. The mapping is relative to
    where the drag started, about 120 px per unit, because the row is
    only 22 px tall and an absolute mapping would squeeze the whole
    −1…+1 range into a 22-pixel throw,
  - **alt-click** to remove,
  - or select it and use the **Properties panel** for exact time, value,
    and easing.

The point is that you key the pose on **one line** and only drop down to
a joint when that joint specifically needs to deviate. Adding the first
key is a no-op by construction — it's seeded with the value the curve
already had — so nothing jumps when you start refining.

Once a joint has override keys, the override **replaces** that joint's
layered value inside the clip; the pose keeps driving every other joint
it names. Remove the last key and the joint goes back to being driven by
the pose blend. Overrides live in `joint_overrides` on the track, and the
anchors are derived at resolve time, so retiming the pose carries them
along instead of stranding a stale copy.

> **Not the same as rig control composition.** Rig controls are
> simultaneous, so they compose as commutative additive deltas and
> reordering the sliders changes nothing. Pose tracks are sequential in
> time and deliberately order-dependent. Two different problems that both
> look like "blending poses".

### Control rig

With a rig file installed, the animation editor grows a **Control Rig**
panel beside the 3D view:

- **Sliders** for 1D channel controls — a mood slider that blends between
  two poses, a nod that drives several joints at different weights and
  curves.
- **XY pads** for eye look. Draggable, and arrow-key nudgeable (Shift for
  coarse steps).
- Each control lists **which joints it's currently moving, and by how
  much**, so "the mouth is wrong" is answerable without guessing which
  slider did it.
- An **At limit** badge when the clamp policy had to pull the frame back,
  with the scale-back percentage — otherwise a slider just mysteriously
  stops having an effect.

The rig evaluates **on the server**, and the resolved joint values come
back to drive the viewport. That keeps one implementation of the blend
math, the clamp policy, and the mimic round-trip.

Moving a control **poses the robot without keyframing anything**, so you
can explore freely. The key button (🔑) commits the current rig pose as
keyframes at the playhead, one per joint the controls are driving. The
rig is the *input device*; the tracks stay the animation's source of
truth, so a saved animation plays back without the rig file present.

Turn on **Live** in the toolbar to push what you're doing to the physical
robot as you drag.

---

## What's not implemented

`<control kind="spatial">` with a `<gaze>` binding — a true look-at that
aims frames at a 3D point — is **fully specified, parsed, validated, and
not evaluated**. Rig files can declare it now and stay valid. It needs
joint origins and axes (which the URDF model deliberately doesn't carry
yet) plus a damped-least-squares solve over the frame Jacobians.

The evaluator reports these controls as skipped rather than ignoring
them, and the panel labels them inactive rather than rendering a gizmo
that silently does nothing.

For a pan/tilt animatronic head, a `pad` control with weighted `<drive>`
elements covers eye look directly and more predictably than a solver
would. Gaze matters when you want to aim at a moving world-space point.

---

## Reference

| Topic | Where |
|---|---|
| Rig file elements, curves, composition | `docs/RIG_SCHEMA.md` |
| Rig XSD | `server/resources/schema/rig-1.1.xsd` |
| Example SRDF + rig | `server/resources/examples/` |
| Collision geometry and the ACM | `docs/COLLISION_AVOIDANCE.md` |

**HTTP API** — one endpoint per file, mirroring the three ROS parameters.
`GET`, `POST` (multipart), and `DELETE` on each:

```
/api/robot/urdf          /api/robot/srdf          /api/robot/rig
/api/robot/metadata      full description + cross-file warnings
/api/robot/joints        actuatable joints, with limits
/api/robot/groups        SRDF groups, expanded to joint lists
/api/robot/group_states  importable pose candidates, both unit systems
/api/robot/meshes/{path}
```

**WebSocket (management channel)** — `get_rig`, `evaluate_rig`,
`list_group_states`, `import_group_states`.

**Tests** — `test_srdf_parsing`, `test_rig`, `test_robot_model_store`,
`test_group_state_import`, `test_frame_resolve`, `test_rig_controls`, and
`web/src/composables/__tests__/frameResolve.spec.js` (the client mirror of
the frame resolver — same fixtures, so drift fails a test).
