# Rig file schema (`robot.rig.xml`, v1.0)

The rig file is the third file in the robot-description stack and the only
custom schema in it. It holds the **control layer**: what an operator can grab
on screen, and how moving it reaches the joints.

| File | ROS parameter | Owns |
|---|---|---|
| `robot.urdf` | `robot_description` | Links, joints, limits, geometry, anchor frames. **Every name.** |
| `robot.srdf` | `robot_description_semantic` | Joint groups and named poses (`<group_state>`) |
| **`robot.rig.xml`** | **`robot_description_rig`** | **Controls, curves, bindings, widget hints** |

Namespace: `urn:saintos:rig:1.0` · Parser: `server/saint_server/animation/rig.py`
· Evaluator: `rig_eval.py` · Schema: `server/resources/schema/rig-1.1.xsd`

---

## Why a separate file

Three options were live while scoping this, and two were rejected:

- **Custom block inside the URDF** (`<rig>` under `<robot>`). Legitimate —
  Drake, Gazebo, and `ros2_control` all own private URDF vocabularies, and
  `urdfdom` ignores unknown top-level children. Rejected because custom tags
  survive *parsing* but not *rewriting*: `gz sdf -p`, urdfdom's serializer, and
  most third-party importers drop them. Annotation data belongs in the URDF; a
  control graph doesn't.
- **Migrate to SDFormat**, which sanctions namespaced custom elements.
  Rejected: SRDF is defined against URDF semantics, the bridge
  (`sdformat_urdf`) parses *into* the URDF data structure, so you adopt SDF and
  immediately constrain yourself to the URDF-expressible subset anyway. All of
  the cost, none of the benefit.
- **A separate file** — chosen. The apparent downside was that a sidecar has no
  distribution channel, but that turned out to be false: the SRDF is
  conventionally published as `robot_description_semantic` alongside
  `robot_description`. The rig file follows that precedent exactly. A standard
  MoveIt config is already six to eight files; the ecosystem's unit of
  packaging is a config package, not a document. **The sidecar is the idiom.**

## Two rules that hold the design together

**1. Everything except `<widget>` is contract.** `<widget>` carries
presentation hints only — shape, colour, which panel a slider lands in. A
scripted performance, a CLI, a test harness, or recorded playback needs the
bindings and has no use for a colour. `rig_eval.py` never reads a `Widget`;
fusing the two is what makes a rig unable to run without a viewport.

**2. Evaluation is declarative, never a graph.** Controls resolve in a fixed
order — pose blends, then direct drives, then solver-backed bindings. There is
no node graph, no expression language, and no control-drives-another-control
edge. Unreal's Rig Graph is a full visual programming environment with its own
execution model and debugger; the moment arbitrary logic sits between a control
and a joint, this stops being a feature and becomes a language.

---

## Units: everything is normalized −1..+1

**The single most important convention in this file.** Joint values in the rig
are normalized to −1..+1 across each joint's URDF `<limit>` — *not* radians.

- Normalized `0` is the **midpoint of a joint's travel**, not its native zero.
  For an asymmetric joint (`lower="-0.5" upper="0.7"`), native `0` normalizes
  to `-0.167`.
- `scale="0.35"` on a `<drive>` therefore reads as "35% of this joint's
  travel", which is directly authorable.
- It matches what every sink downstream of the routing graph already speaks,
  and what `URDFViewer.setJointValue()` expects in the web UI.

Native units (radians / metres) appear in exactly two places:

1. **SRDF import.** `<group_state>` values are URDF-native. They're converted
   once, at import, by `GroupState.normalized_values(urdf)`.
2. **`<mimic>` evaluation.** The coupling `q_slave = multiplier · q_master +
   offset` is *defined* in native units, so it round-trips through the limits.
   Applying the multiplier to normalized values is wrong whenever the two
   joints have different travel — which is the common case.

---

## Document structure

```xml
<?xml version="1.0" encoding="utf-8"?>
<rig xmlns="urn:saintos:rig:1.0" version="1.0" robot="johnny5">

  <settings clamp="scale_back" neutral_pose="neutral"/>

  <control name="mood" kind="channel" label="Mood" group="Face" order="10"
           min="-1" max="1" default="0">
    <target pose="mad"   at="-1" curve="easeInOut"/>
    <target pose="happy" at="1"  curve="easeInOut"/>
  </control>

  <control name="head_nod" kind="channel" label="Nod" group="Head" order="20"
           min="-1" max="1" default="0">
    <drive joint="head_pitch" scale="1.0"  curve="easeInOut"/>
    <drive joint="neck_pitch" scale="0.35" curve="linear"/>
  </control>

  <control name="eye_look" kind="pad" label="Eye look" group="Head" order="30">
    <axis name="x" min="-1" max="1" default="0">
      <drive joint="eye_l_pan" scale="1.0"/>
      <drive joint="eye_r_pan" scale="1.0"/>
      <drive joint="neck_yaw"  scale="0.25" curve="easeIn"/>
    </axis>
    <axis name="y" min="-1" max="1" default="0">
      <drive joint="eye_l_tilt" scale="1.0"/>
      <drive joint="eye_r_tilt" scale="1.0"/>
      <drive joint="head_pitch" scale="0.30" curve="easeIn"/>
    </axis>
    <widget kind="pad" invert_y="true"/>
  </control>

  <control name="look_at" kind="spatial" label="Look at" group="Head"
           order="40" anchor="ctrl_lookat">
    <gaze>
      <frame link="eye_l" axis="0 0 1" weight="1.0"/>
      <frame link="eye_r" axis="0 0 1" weight="1.0"/>
      <frame link="head"  axis="1 0 0" weight="0.2"/>
      <regularize joints="eye_l_pan eye_l_tilt eye_r_pan eye_r_tilt"
                  weight="0.05"/>
    </gaze>
    <widget kind="gizmo" shape="sphere" scale="0.05"
            color="1 0.7 0 0.8" dofs="move_3d"/>
  </control>
</rig>
```

The `xmlns` is **optional** — the parser matches on local names. Requiring it
would make every hand-authored file fail its first load for a reason the error
message can't easily explain, and the namespace matters for tools that *aren't*
this one.

---

## Elements

### `<rig>`

| Attribute | Default | Meaning |
|---|---|---|
| `version` | `1.0` | Schema version. A **higher major is rejected**, not best-effort parsed — an unknown control kind dropped in silence is worse than a load error an operator can see. A higher minor loads fine. |
| `robot` | — | Must match the URDF's `<robot name>`. Mismatch is a validation warning. |

### `<settings>`

| Attribute | Default | Meaning |
|---|---|---|
| `clamp` | `scale_back` | Limit policy after all controls compose. See below. |
| `neutral_pose` | — | Named pose used as the blend origin `q₀`. Absent → all-zeros (each joint at its travel midpoint). |

**`clamp` policy.** Two controls at +1 can easily sum past a joint's limit, and
how that resolves is a visible authoring decision:

- `clamp` — each joint stops independently. The brow pins while the mouth keeps
  travelling, so the face **breaks** at the extremes.
- `scale_back` — the whole frame's delta scales back by the worst offender's
  factor, so the pose **saturates as a unit** and keeps its shape. Default for
  that reason.

### `<control>`

| Attribute | Default | Meaning |
|---|---|---|
| `name` | required | Unique identifier. Duplicates are a load error. |
| `kind` | `channel` | `channel` \| `pad` \| `spatial` |
| `label` | = `name` | Display text |
| `group` | `""` | UI panel grouping. Groups render in first-seen file order. |
| `order` | `0` | Sort key within the group |
| `min` / `max` | `-1` / `1` | Value range |
| `default` | `0` | Rest value. **Deflection is measured outward from here**, so a control contributes nothing at rest — see below. |
| `anchor` | — | `spatial` only: the URDF link whose frame the target lives in |

**Control kinds:**

- **`channel`** — one scalar, renders as a slider. Binds via `<target>` and/or
  `<drive>`. A scalar with no spatial meaning must never get a 3D gizmo; that's
  strictly worse than a slider, which is why Unreal keeps float curves in the
  Anim panel rather than the viewport.
- **`pad`** — two scalars, renders as an XY pad. Each `<axis>` carries its own
  bindings. The right shape for eye look on a pan/tilt mechanism: direct,
  predictable, no solver in the loop.
- **`spatial`** — a 3D point in `anchor`'s frame, renders as a draggable gizmo.
  Requires a `<gaze>` binding. **Declared but not yet evaluated** — see
  [Status](#status).

**Deflection is measured from `default`, not from `min`.** A control at rest
contributes zero and ramps to ±`scale` at its extremes, measured outward in
each direction independently. So an asymmetric control (`min="-0.25" max="1"
default="0"`) reaches full magnitude at *both* ends. Measuring from `min`
instead would leave a `0..1` control permanently half-applied while sitting at
its own resting position, and every pose built on it would lean.

### `<drive>` — drive a joint directly

```xml
<drive joint="head_pitch" scale="0.35" offset="0" curve="easeInOut"/>
```

`contribution = curve(deflection) · sign · scale + offset`, in normalized joint
space. This is the head-nod primitive: no pose needs to exist, and one control
can drive many joints at different scales and with different curves.

Per-joint curves are where the organic feel comes from — the brow snapping up
while the mouth corner lags is the whole difference between a rig and a lerp.
This is also why `<mimic>` can't do this job: mimic is one linear ratio,
forever, with no intermediate neutral.

| Attribute | Default | Meaning |
|---|---|---|
| `joint` | required | URDF joint name. Must be actuatable (not `fixed`). |
| `scale` | `1.0` | Fraction of the joint's travel at full deflection |
| `offset` | `0.0` | Constant added to the contribution |
| `curve` | `linear` | Easing name — see [catalog](#curve-catalog) |

### `<target>` — blend toward a named pose

```xml
<target pose="happy" at="1" curve="easeInOut"/>
```

Targets make a control a **1D blend space**. Stops are the targets' `at` values
plus an implicit neutral stop at the control's `default` (unless a target
already sits there). The control's value lands in one segment and blends
between that segment's two endpoints.

The consequence worth noticing: with `mad` at −1 and `happy` at +1, one slider
covers both and they can **never be active at once** — the same reason riggers
use a single smile/frown slider instead of two. The `curve` belongs to the
target being *approached*, so each half of a ±1 slider can ease differently.

A pose that doesn't name a joint leaves it at neutral rather than dragging it
to zero. Poses are sparse by nature, and a missing joint means "don't care",
not "centre it".

| Attribute | Default | Meaning |
|---|---|---|
| `pose` | required | Pose name — an SRDF `<group_state>` or a pose saved in the UI. Either the authored name or its slug works; see below. |
| `at` | `1.0` | Control value at which this pose is fully applied. Two targets sharing an `at` is a validation warning (zero-width segment). |
| `curve` | `linear` | Easing into this target |

**Naming a pose.** The pose library slugifies on lookup, so a
`<group_state name="Very Happy">` imports as a pose with id `very_happy`
and a rig can name it either way — `pose="Very Happy"` and
`pose="very_happy"` both resolve. Validation accepts both forms too, and
it accepts a reference to a group_state the SRDF defines even before it's
been imported, since importing is what materialises it. What *is* flagged
is a name that matches neither a known pose nor a group_state.

When nothing is known about the pose library — no SRDF installed and no
pose list supplied — pose references go unchecked rather than all being
reported broken, which would be noise.

### `<axis>` — one axis of a `pad`

Takes `name` (`x` or `y`), `min`, `max`, `default`, and its own `<drive>` /
`<target>` children. Callers address axes as `"<control>.<axis>"`; a bare
control name feeds the `x` axis so a 1D caller degrades gracefully.

### `<gaze>` — look-at binding (`spatial` only)

```xml
<gaze>
  <frame link="eye_l" axis="0 0 1" weight="1.0"/>
  <frame link="head"  axis="1 0 0" weight="0.2"/>
  <regularize joints="eye_l_pan eye_l_tilt" weight="0.05"/>
</gaze>
```

A gaze task is a **2-DOF residual**, not a 6-DOF pose goal. Transform the
target into the frame and ask that it land on the forward axis:

```
p_E = T_frame→world⁻¹ · target
error = [ p_E.x / p_E.z , p_E.y / p_E.z ]
```

Two numbers, zero when aligned, scale-invariant so it behaves the same near and
far. The obvious shortcut — a full orientation target — **over-constrains**: it
pins roll about the gaze axis, forcing a specific head tilt as a side effect of
asking the rig to look somewhere. In rigging terms, that's an aim constraint
with a spurious up-vector.

`<regularize>` is what makes eye/head distribution fall out of the solve
instead of being hand-authored. Strong gaze tasks on the eyes plus a weak
posture task pulling them toward centre means the eyes snap to the target, the
posture task complains they're off-centre, and the only way to satisfy both is
for the head to rotate and re-centre them. Eyes lead, head follows — tuned with
two weights instead of a curve per joint. And the URDF's own `<limit>` gives
the rest for free: when the eyes hit their stops, the residual can only shrink
by moving the head.

| Element | Attribute | Default | Meaning |
|---|---|---|---|
| `<frame>` | `link` | required | URDF link that does the pointing |
| | `axis` | `0 0 1` | Frame-local forward direction |
| | `weight` | `1.0` | How hard this frame competes |
| `<regularize>` | `joints` | — | Space- or comma-separated joint names |
| | `weight` | `0.05` | Posture-task strength |

### `<widget>` — how the control is drawn and grabbed (v1.1)

Never read by the evaluator, so the rig still runs headless. But
"presentation" is not the same as "vague": a control shape sits at a real
place on the robot with a real orientation and a real drag axis, and that
geometry is what turns an abstract scalar into something you can grab. It's
the Unreal Control Rig arrangement — a shape parented to a bone by an offset
transform.

This is also the resolution of an argument recorded in the design thread,
which held that a scalar should never get a 3D gizmo because "a scalar with
no spatial meaning rendered as a draggable widget in 3D is worse than a
slider". True as far as it goes — the fix isn't to avoid the viewport, it's
to give the scalar spatial meaning. A ring on the head that you sweep to nod
is not a slider floating in space. That's what `axis` is for.

| Attribute | Default | Meaning |
|---|---|---|
| `kind` | follows control kind | `slider` \| `pad` \| `gizmo` |
| `shape` | per control kind (see below) | Geometry — closed set, see catalog |
| `scale` | `0.05` | One number (uniform) or three (per-axis), metres |
| `color` | `1 0.7 0 0.8` | RGB or RGBA, 0–1 |
| `offset` | `0 0 0` | Position **in the anchor link's frame**, so the shape rides the joint |
| `rotation` | `0 0 0` | Roll pitch yaw, radians, in the anchor's frame |
| `axis` | `1 0 0` | Drag direction, in the widget's own post-rotation frame |
| `dofs` | `move_3d` | Interaction mode |
| `invert_y` | `false` | Pad only: flip the y axis for screen-space feel |
| `visible` | `true` | Draw in the 3D view at all |

**Shape catalog.** A closed set on purpose — a name that silently fell back
to a sphere would leave you wondering why your arrow never appeared:

`sphere` · `box` · `circle` · `ring` · `cylinder` · `cone` · `arrow` ·
`diamond` · `torus` · `wedge` · `plane`

Chosen for the interaction vocabulary rather than to mirror Unreal's full
library: a **ring** for rotation, an **arrow** for a single axis, a **plane**
for two, a **sphere**/**box** for a position, a **wedge** for a bounded
sweep. Defaults follow the control kind — `ring` for a channel, `plane` for a
pad, `sphere` for a spatial — so a control that says nothing about its
appearance still gets something grabbable and appropriate.

**How dragging works.** Pointer motion is projected onto `axis` *as it
appears on screen* and mapped across the control's `min`…`max` over about
260 px. Because it's a screen-space projection, it behaves the same at any
camera angle, and an axis pointing nearly at the camera just gets less
sensitive rather than inverting. Pad controls ignore `axis` and use the two
screen axes directly — a pad is inherently a 2D screen gesture, and forcing
it through one axis would lose a degree of freedom.

Shapes are drawn with `depthTest: false`, i.e. always on top. A handle buried
inside the head is unusable, and Unreal does the same.

### `anchor` — where the shape lives

An explicit `anchor="<link>"` always wins. Absent one it is **derived** from
what the control drives: the child link of the first joint it moves, or (for
a pure pose-blend control with no `<drive>`) the first joint its poses name.

Deriving matters more than it looks. Without it, every rig file would need an
`anchor` on every control before anything appeared in the viewport, and a
file written before v1.1 would silently draw nothing. Controls with no
resolvable link are **omitted** rather than defaulted to the robot root,
where a cluster of unrelated handles would pile onto the base and look like a
bug.

---

## Curve catalog

`curve=` takes any name from the shared easing catalog — the same one the
animation timeline uses, so the two never diverge. Names are camelCase, derived
mechanically from `CurveInterpolation` in
`server/saint_server/unreal/animation.py` and matching the `name` field of each
entry in `server/web/src/composables/easings.js`.

`constant`, `linear`, `cubic`, `ease`, `easeIn`, `easeOut`, `easeInOut`, then
the easings.net palette in `In` / `Out` / `InOut` variants across `Sine`,
`Quad`, `Cubic`, `Quart`, `Quint`, `Expo`, `Circ`, `Back`, `Elastic`, `Bounce`
— e.g. `easeInOutSine`, `easeOutBack`, `easeInBounce`.

An unrecognized name is a **load error**, not a silent fallback: a typo that
costs you the wrong feel is much harder to notice than one that refuses to
load.

---

## Composition

All controls compose **additively as deltas from neutral**:

```
q = q₀ + Σᵢ fᵢ(wᵢ) · (qᵢ − q₀)      then clamp per <settings clamp=>
```

Two properties earn that choice. It's **commutative**, so stacking "happy" and
"surprised" is order-independent and nobody ever debugs why the result changed
when the UI reordered the sliders. And each control's contribution stays
**independently inspectable** — `RigFrame.contributions` reports per-control
joint deltas, so "the mouth is wrong" is answerable without guessing which
slider did it.

> **Contrast with animation pose tracks.** Pose *tracks* on the animation
> timeline deliberately layer **in order**, so later tracks win. Different
> problem: there an operator is stacking takes over time and wants override
> semantics, which is what makes track reordering meaningful. Here controls are
> simultaneous and order is meaningless. See `animation/frame.py`.

`<mimic>` couplings are applied **last**, as an overwrite, in native units. A
mimicking joint has no independent authored value, so letting a pose set one
directly would just fight the coupling.

---

## Status

| Feature | State |
|---|---|
| `channel` controls — `<drive>` | Implemented |
| `channel` controls — `<target>` pose blending | Implemented |
| `pad` controls (eye look, 2-axis) | Implemented |
| Clamp policies (`clamp`, `scale_back`) | Implemented |
| `<mimic>` coupling in native units | Implemented |
| Per-control contribution reporting | Implemented |
| `spatial` / `<gaze>` controls | **Parsed and validated, not evaluated** |

`<gaze>` is fully specified and round-trips through the parser, so rig files can
declare it now and stay valid. Evaluation needs joint origins and axes (which
`urdf_model.py` deliberately doesn't carry yet) plus a damped-least-squares
solve over the frame Jacobians. `RigEvaluator` reports these controls in
`RigFrame.skipped` rather than silently ignoring them, so an operator dragging a
gizmo that does nothing gets a reason.

For a pan/tilt animatronic head, a `pad` control with weighted `<drive>`
elements covers eye look directly and more predictably than a solver would —
`<gaze>` matters when you want to aim at a moving world-space point.

---

## Related

- `docs/COLLISION_AVOIDANCE.md` — the SRDF `<disable_collisions>` ACM this
  stack can now consume, versus the URDF-only pair whitelist (decision D3)
- `server/test/test_rig.py` — parser, validation, and evaluator behaviour
- `server/test/test_srdf_parsing.py` — SRDF parsing plus the radians↔normalized
  conversion
