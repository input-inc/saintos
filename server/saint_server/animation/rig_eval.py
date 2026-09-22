"""Rig evaluator: control values → joint values.

Additive pose blending in joint space. The whole thing is arithmetic —
no IK, no solver, no deformation:

    q = q₀ + Σᵢ fᵢ(wᵢ) · (qᵢ − q₀)          then clamp

``q₀`` is neutral, ``qᵢ`` is the pose a control blends toward, ``wᵢ``
is the control's value and ``fᵢ`` its easing curve. Maya calls this a
Set Driven Key, Blender calls it a pose library with drivers, game
engines call it a 1D blend space.

**Additive deltas, not overrides.** Two properties earn that choice.
They're commutative, so stacking "happy" and "surprised" is
order-independent and nobody ever debugs why the result changed when
the UI reordered the sliders. And each control's contribution stays
independently inspectable, which matters the moment someone says "the
mouth is wrong" and you need to know which slider did it.

(Animation *pose tracks* deliberately layer in order instead — see
``frame.py``. Different problem: there, the operator is stacking takes
over time and wants later tracks to win, which is what makes reordering
meaningful. Here, controls are simultaneous and order is meaningless.)

**Everything is normalized −1..+1**, not radians. That's the convention
every sink downstream of the routing graph already speaks, and it makes
``scale="0.35"`` on a drive read as "35% of this joint's travel", which
is directly authorable. Native units appear in exactly two places: the
SRDF import boundary (radians in, converted once) and mimic evaluation
(where the coupling is defined in native units and would otherwise be
wrong for any two joints with different ranges).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

from saint_server.animation.rig import Control, Rig, Target
from saint_server.animation.urdf_model import UrdfModel
from saint_server.unreal.animation import ease_at


# Joint values are normalized to this range, so "the limit" is ±1.
_LIMIT = 1.0


@dataclass
class RigFrame:
    """One evaluated frame, plus what went wrong producing it."""

    # joint name → normalized −1..+1
    joints: Dict[str, float] = field(default_factory=dict)
    # Per-control joint deltas, kept so the UI can answer "which slider
    # moved this joint?" without re-running the evaluation.
    contributions: Dict[str, Dict[str, float]] = field(default_factory=dict)
    # True when the clamp policy had to pull the frame back.
    clamped: bool = False
    # Proportional scale-back factor actually applied (1.0 = untouched).
    scale_applied: float = 1.0
    # Controls the evaluator could not evaluate: [{control, reason}].
    # STRUCTURED rather than a list of sentences, because the UI has to
    # act on it — an inert control's 3D shape must be drawn as inert and
    # excluded from hit-testing, and parsing a control name back out of a
    # human-readable message would be fragile.
    skipped: List[Dict[str, str]] = field(default_factory=list)

    def to_dict(self) -> Dict:
        return {
            "joints": dict(self.joints),
            "contributions": {k: dict(v) for k, v in self.contributions.items()},
            "clamped": self.clamped,
            "scale_applied": self.scale_applied,
            "skipped": [dict(s) for s in self.skipped],
        }


class RigEvaluator:
    """Evaluates a Rig against control values.

    Stateless between calls — hand it the control values and it returns
    a frame. Keeping it stateless is what lets the same evaluator serve
    the live UI, the animation player, and a headless test with no
    lifecycle to get wrong.
    """

    def __init__(self, rig: Rig, urdf: Optional[UrdfModel] = None,
                 poses: Optional[Dict[str, Dict[str, float]]] = None):
        """``poses`` maps pose name → {joint: normalized value}.

        Poses arrive already normalized because the conversion needs
        the URDF limits and belongs at the import boundary, done once,
        rather than on every frame.
        """
        self.rig = rig
        self.urdf = urdf
        self.poses: Dict[str, Dict[str, float]] = poses or {}

    # ── neutral ─────────────────────────────────────────────────────

    def neutral(self) -> Dict[str, float]:
        """The blend origin q₀.

        Either a named pose from ``<settings neutral_pose="...">`` or,
        absent one, implicit zeros — and normalized zero is the midpoint
        of each joint's travel, not its native zero, which is the right
        default for a rig at rest.
        """
        name = self.rig.settings.neutral_pose
        if name and name in self.poses:
            return dict(self.poses[name])
        return {}

    # ── evaluation ──────────────────────────────────────────────────

    def evaluate(self, control_values: Dict[str, float]) -> RigFrame:
        """Evaluate every control and compose their deltas.

        ``control_values`` is keyed by control name for channel controls
        and by ``"<control>.<axis>"`` for pad controls — a pad takes two
        scalars, so one flat key can't express it. A bare pad name still
        feeds its x axis, so a 1D caller degrades gracefully.

        Anything absent sits at its declared ``default``, so a partial
        dict from the UI is fine and a freshly loaded rig evaluates to
        its rest pose.
        """
        frame = RigFrame()
        q0 = self.neutral()

        # Deltas are kept per control rather than summed as we go, so
        # `contributions` can answer "which slider moved this joint?".
        for control in self.rig.controls:
            delta, skip = self._control_delta(control, control_values, q0)
            if skip:
                frame.skipped.append(skip)
            if delta:
                frame.contributions[control.name] = delta

        # Sum: q = q₀ + Σ deltas, over the union of every joint touched.
        touched = set(q0)
        for delta in frame.contributions.values():
            touched |= set(delta)

        total: Dict[str, float] = {}
        for joint in touched:
            base = q0.get(joint, 0.0)
            total[joint] = base + sum(
                d.get(joint, 0.0) for d in frame.contributions.values())

        frame.joints = self._apply_clamp(total, q0, frame)
        self._apply_mimics(frame.joints)
        return frame

    # ── per-control deltas ──────────────────────────────────────────

    def _control_delta(self, control: Control, values: Dict[str, float],
                       q0: Dict[str, float]):
        """Return ``(delta, skip_reason)`` for one control."""
        delta: Dict[str, float] = {}

        if control.kind == "spatial":
            # Declared in the schema, not evaluated here: a gaze binding
            # is a 2-DOF residual solved against frame Jacobians, which
            # needs the joint origins and axes this model doesn't carry.
            # Reported rather than silently ignored, so an operator sees
            # why the handle does nothing — and so the viewport can draw
            # it as inert instead of offering a grab that goes nowhere.
            return delta, {
                "control": control.name,
                "reason": "gaze binding needs the IK solver; not evaluated",
            }

        if control.kind == "pad":
            for axis in control.axes:
                value = values.get(f"{control.name}.{axis.name}")
                if value is None and axis.name == "x":
                    value = values.get(control.name)
                if value is None:
                    value = axis.default
                _accumulate(delta, self._axis_delta(axis, value, q0))
            return delta, ""

        # channel
        value = values.get(control.name, control.default)
        _accumulate(delta, self._drives_delta(control.drives, value,
                                              control.min, control.max,
                                              control.default))
        _accumulate(delta, self._targets_delta(control.targets, value,
                                               control.min, control.max,
                                               control.default, q0))
        return delta, ""

    def _axis_delta(self, axis, value: float, q0: Dict[str, float]) -> Dict[str, float]:
        out: Dict[str, float] = {}
        _accumulate(out, self._drives_delta(axis.drives, value, axis.min,
                                            axis.max, axis.default))
        _accumulate(out, self._targets_delta(axis.targets, value, axis.min,
                                             axis.max, axis.default, q0))
        return out

    def _drives_delta(self, drives, value: float, vmin: float, vmax: float,
                      vdefault: float) -> Dict[str, float]:
        """Direct joint drives — the head-nod primitive.

        Deflection is measured from the control's ``default`` (its rest
        value), not from zero or from ``min``: a control contributes
        nothing at rest and ramps to ±scale at its extremes. Measuring
        from ``min`` instead would make a 0..1 control apply half its
        scale while sitting at its own resting position.
        """
        out: Dict[str, float] = {}
        if not drives:
            return out
        t, sign = _deflection(value, vmin, vmax, vdefault)
        for d in drives:
            shaped = ease_at(d.curve, t)
            out[d.joint] = out.get(d.joint, 0.0) + sign * shaped * d.scale + d.offset
        return out

    def _targets_delta(self, targets: List[Target], value: float,
                       vmin: float, vmax: float, vdefault: float,
                       q0: Dict[str, float]) -> Dict[str, float]:
        """1D blend space across named poses.

        Stops are the targets' ``at`` values plus an implicit neutral
        stop at the control's ``default`` (unless a target already sits
        there). The value lands in one segment and blends between that
        segment's two endpoints — which is why a single ±1 slider with
        "mad" at −1 and "happy" at +1 can never have both active at
        once. That's the same reason riggers use one smile/frown slider
        instead of two.
        """
        if not targets:
            return {}

        stops = sorted(
            ((t.at, t.pose, t.curve) for t in targets),
            key=lambda s: s[0])
        if not any(abs(s[0] - vdefault) < 1e-9 for s in stops):
            stops.append((vdefault, None, 0))
            stops.sort(key=lambda s: s[0])

        v = max(vmin, min(vmax, float(value)))

        if v <= stops[0][0]:
            blended = self._pose_values(stops[0][1], q0)
        elif v >= stops[-1][0]:
            blended = self._pose_values(stops[-1][1], q0)
        else:
            blended = None
            for i in range(len(stops) - 1):
                a_at, a_pose, _ = stops[i]
                b_at, b_pose, b_curve = stops[i + 1]
                if not (a_at <= v <= b_at):
                    continue
                span = b_at - a_at
                u = 0.0 if span <= 0 else (v - a_at) / span
                # The curve belongs to the target being approached, so
                # each half of a ±1 slider can ease differently.
                eased = ease_at(b_curve, u)
                va = self._pose_values(a_pose, q0)
                vb = self._pose_values(b_pose, q0)
                blended = {}
                for joint in set(va) | set(vb):
                    base = q0.get(joint, 0.0)
                    # A pose that doesn't name a joint leaves it at
                    # neutral rather than dragging it to zero — poses are
                    # sparse by nature and a missing joint means "don't
                    # care", not "centre it".
                    fa = va.get(joint, base)
                    fb = vb.get(joint, base)
                    blended[joint] = fa + eased * (fb - fa)
                break
            if blended is None:
                blended = self._pose_values(None, q0)

        # Contribution is a delta from neutral, which is what makes
        # stacking commutative.
        return {j: blended[j] - q0.get(j, 0.0) for j in blended}

    def _pose_values(self, pose_name: Optional[str],
                     q0: Dict[str, float]) -> Dict[str, float]:
        """Absolute joint values for a stop. ``None`` means neutral."""
        if pose_name is None:
            return dict(q0)
        return dict(self.poses.get(pose_name) or {})

    # ── post-processing ─────────────────────────────────────────────

    def _apply_clamp(self, total: Dict[str, float], q0: Dict[str, float],
                     frame: RigFrame) -> Dict[str, float]:
        """Enforce ±1 under the rig's declared clamp policy.

        Two controls at +1 can easily sum past a joint's limit, and how
        that resolves is a visible authoring decision rather than an
        implementation detail:

          ``clamp``      — each joint stops independently. The brow pins
                           while the mouth keeps travelling, so the face
                           *breaks* at the extremes.
          ``scale_back`` — the whole frame's delta scales back by the
                           worst offender's factor, so the pose
                           saturates as a unit and keeps its shape.
        """
        policy = self.rig.settings.clamp
        over = {j: v for j, v in total.items() if abs(v) > _LIMIT + 1e-12}
        if not over:
            return total

        frame.clamped = True

        if policy == "clamp":
            return {j: max(-_LIMIT, min(_LIMIT, v)) for j, v in total.items()}

        # scale_back: find the largest s ≤ 1 keeping every joint in range.
        s = 1.0
        for joint in over:
            base = q0.get(joint, 0.0)
            delta = total[joint] - base
            if abs(delta) < 1e-12:
                continue        # the neutral pose itself is out of range
            bound = _LIMIT if delta > 0 else -_LIMIT
            s = min(s, (bound - base) / delta)
        s = max(0.0, min(1.0, s))
        frame.scale_applied = s

        scaled = {j: q0.get(j, 0.0) + (v - q0.get(j, 0.0)) * s
                  for j, v in total.items()}
        # A neutral pose that itself sits out of range can't be rescued
        # by scaling, so hard-clamp as a floor.
        return {j: max(-_LIMIT, min(_LIMIT, v)) for j, v in scaled.items()}

    def _apply_mimics(self, joints: Dict[str, float]) -> None:
        """Overwrite mimicking joints from their masters, in place.

        ``<mimic>`` is defined in NATIVE units — ``q_slave =
        multiplier · q_master + offset`` in radians or metres — so it
        has to round-trip through the URDF limits. Applying the
        multiplier to normalized values would be wrong for any two
        joints whose ranges differ, which is the common case (an eyelid
        mimicking a brow rarely shares its travel).

        Applied last, and as an overwrite: a mimicking joint has no
        independent authored value, so letting a pose set one directly
        would just fight the coupling.
        """
        if self.urdf is None:
            return
        for name, info in self.urdf.joints.items():
            if info.mimic is None:
                continue
            master = self.urdf.joint(info.mimic.joint)
            if master is None or info.mimic.joint not in joints:
                continue
            native_master = master.denormalize(joints[info.mimic.joint])
            native_slave = info.mimic.multiplier * native_master + info.mimic.offset
            joints[name] = info.normalize(info.clamp_native(native_slave))

    # ── introspection for the UI ────────────────────────────────────

    def control_defaults(self) -> Dict[str, float]:
        """Rest values keyed the way :meth:`evaluate_pad` expects."""
        out: Dict[str, float] = {}
        for c in self.rig.controls:
            if c.kind == "pad":
                for axis in c.axes:
                    out[f"{c.name}.{axis.name}"] = axis.default
            else:
                out[c.name] = c.default
        return out

    def driven_joints(self) -> List[str]:
        """Every joint any control can move, in stable order.

        Lets the editor grey out joints the rig owns, so an operator
        doesn't hand-key a joint a control is about to overwrite.
        """
        out: List[str] = []
        for c in self.rig.controls:
            for j in c.referenced_joints():
                if j not in out:
                    out.append(j)
        for pose in self.rig.referenced_poses():
            for j in (self.poses.get(pose) or {}):
                if j not in out:
                    out.append(j)
        return out


def _deflection(value: float, vmin: float, vmax: float,
                vdefault: float) -> tuple:
    """Map a control value to ``(magnitude 0..1, sign ±1)``.

    Measured outward from ``vdefault`` in each direction independently,
    so an asymmetric control (say −0.3..+1 resting at 0) reaches full
    magnitude at both ends rather than only at the wider one.
    """
    v = max(vmin, min(vmax, float(value)))
    if v >= vdefault:
        span = vmax - vdefault
        return (0.0, 1.0) if span <= 0 else (min(1.0, (v - vdefault) / span), 1.0)
    span = vdefault - vmin
    return (0.0, -1.0) if span <= 0 else (min(1.0, (vdefault - v) / span), -1.0)


def _accumulate(into: Dict[str, float], more: Dict[str, float]) -> None:
    for joint, value in more.items():
        into[joint] = into.get(joint, 0.0) + value
