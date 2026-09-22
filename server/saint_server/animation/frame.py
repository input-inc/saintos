"""Resolve an animation's value tracks into one frame of setpoints.

Extracted from the player because three callers need the identical
answer: the server-side player, the editor's Live Preview, and the
client's 3D viewport. The client's copy lives in
``web/src/composables/useFrameResolve.js`` and
``test_animation_frame_equivalence`` guards the two against drift.

**Track order is significant.** Value tracks resolve in list order, and
each one layers over what came before:

  * a ``pose`` track lerps the joints its pose names from the
    accumulated value toward the pose's value, by the track's weight;
  * a ``urdf_joint`` track hard-sets its one joint;
  * a ``ws_input`` track writes to a separate bucket and never
    interacts with joints at all.

So a joint track *below* a pose track gets overridden by it, and one
*above* wins.

**Pose tracks are CLIPS, not infinite layers.** A pose track contributes
only between its first and last keyframe. Outside that span it
contributes nothing at all — it does not hold its last value forever the
way a joint track's curve does.

That's what makes handing off between poses automatic: give "happy" a
span over 0–2 s and "mad" a span over 2–4 s and mad simply takes over,
because happy's span has ended rather than because mad sits higher in the
list. **Order only decides where two spans actually overlap.** Once no
span covers the playhead, the joints fall back to neutral.

The consequence worth knowing: a pose track with a SINGLE keyframe has a
zero-length span and therefore contributes essentially nothing. That's
why the editor's "+ Pose" creates two keys, and why the timeline draws
the span as a bar — a degenerate clip should be visible at a glance.

**Per-joint overrides.** A pose track may carry
``joint_overrides[joint]`` — a curve of absolute joint values used to
refine one joint inside the clip without touching the others. When
present for a joint, it REPLACES that joint's layered value within the
span. Its keys are anchored at each of the pose track's own keyframe
times (value ``neutral + (target − neutral) · weight(T)``, i.e. the
pose's own contribution, independent of other tracks so the anchors stay
deterministic); the operator's keys deviate between those anchors.

This is deliberately NOT how rig controls compose. Controls are
simultaneous, so they sum as commutative deltas from neutral and
reordering the sliders changes nothing (see ``rig_eval.py``). Two
different problems that both look like "blending poses":

    pose tracks   sequential in time    order matters   clip / override
    rig controls  simultaneous          order is moot   additive deltas
"""

from __future__ import annotations

from typing import Any, Callable, Dict, List, Optional, Tuple

from saint_server.unreal.animation import (
    AnimationCurve,
    CurveInterpolation,
    CurveKey,
)


# pose name/id → {joint: normalized −1..+1}
PoseLookup = Callable[[str], Optional[Dict[str, float]]]

WSKey = Tuple[str, str]

# Tolerance for "is this override key at the same time as an anchor".
# One millisecond: finer than any keyframe an operator can place, coarse
# enough that float drift through a JSON round-trip can't split an anchor
# into two keys.
_TIME_EPS = 1e-3


def track_span(track) -> Optional[Tuple[float, float]]:
    """``(first_key_time, last_key_time)`` for a track, or None if it has
    no keys.

    The clip extent of a pose track. A single key yields a zero-length
    span, which is a real (if useless) state the editor surfaces rather
    than silently widening.
    """
    curve = getattr(track, "curve", None)
    keys = getattr(curve, "keys", None) or []
    if not keys:
        return None
    times = [float(k.time) for k in keys]
    return (min(times), max(times))


def _in_span(track, t: float) -> bool:
    span = track_span(track)
    if span is None:
        return False
    lo, hi = span
    return (lo - _TIME_EPS) <= t <= (hi + _TIME_EPS)


def _weight_at(track, t: float) -> float:
    """The track's weight curve, clamped to 0..1.

    A pose track's curve says "how much of this pose", so whatever the
    operator drew gets clamped regardless.
    """
    try:
        v = track.value_at(t)
    except Exception:                            # noqa: BLE001
        return 0.0
    return 0.0 if v < 0.0 else (1.0 if v > 1.0 else v)


def _override_curve(track, joint: str):
    """The operator's override curve for one joint, or None."""
    overrides = getattr(track, "joint_overrides", None) or {}
    curve = overrides.get(joint)
    if curve is None:
        return None
    keys = getattr(curve, "keys", None) or []
    return curve if keys else None


def override_anchors(track, joint: str, target: float,
                     neutral_value: float) -> List[CurveKey]:
    """Locked anchor keys for a joint's override curve.

    One per keyframe on the pose track itself, valued at the pose's own
    contribution at that time — ``neutral + (target − neutral) ·
    weight(T)``. Deliberately independent of the other tracks: an anchor
    whose value depended on the surrounding layers would move when an
    unrelated track was reordered, and the operator would have no way to
    reason about it.

    These are the keys the editor renders locked. The operator's own keys
    live in the override curve and deviate between them.
    """
    curve = getattr(track, "curve", None)
    keys = getattr(curve, "keys", None) or []
    out: List[CurveKey] = []
    for k in keys:
        w = _weight_at(track, float(k.time))
        out.append(CurveKey(
            time=float(k.time),
            value=neutral_value + (target - neutral_value) * w,
            interp=CurveInterpolation(int(getattr(k, "interp", 1))),
        ))
    return out


def effective_override_keys(track, joint: str, target: float,
                            neutral_value: float) -> List[CurveKey]:
    """Anchors merged with the operator's keys, sorted by time.

    An operator key at (within epsilon of) an anchor time wins, so a
    locked anchor can still be deviated from if the UI ever allows it —
    and so a float-drifted duplicate can't produce two keys a hair apart,
    which would read as a vertical jump.
    """
    anchors = override_anchors(track, joint, target, neutral_value)
    curve = _override_curve(track, joint)
    user = list(getattr(curve, "keys", None) or []) if curve is not None else []

    merged: List[CurveKey] = list(user)
    for a in anchors:
        if not any(abs(float(u.time) - a.time) <= _TIME_EPS for u in user):
            merged.append(a)
    merged.sort(key=lambda k: float(k.time))
    return merged


def resolve_frame(anim, t: float,
                  pose_lookup: Optional[PoseLookup] = None,
                  neutral: Optional[Dict[str, float]] = None,
                  on_error: Optional[Callable[[str, Exception], None]] = None
                  ) -> Tuple[Dict[str, float], Dict[WSKey, float]]:
    """Sample every value track at ``t`` and layer them in order.

    Returns ``(joint_values, ws_values)`` — joint names → normalized
    −1..+1, and ``(sheet_id, ws_input_id)`` → value.

    ``neutral`` is the base a pose track blends *up from* on joints
    nothing has touched yet. It's the rig's neutral pose when there is
    one; absent that, an untouched joint starts at 0 (the midpoint of
    its travel).

    ``on_error`` is called as ``(track_id, exception)`` for a track that
    fails to sample, so the caller can log it in its own voice instead of
    this module guessing at a logger. A failing track is skipped rather
    than aborting the frame — one bad curve shouldn't freeze the rig
    mid-performance.
    """
    joint_values: Dict[str, float] = {}
    ws_values: Dict[WSKey, float] = {}
    base = neutral or {}

    for track in getattr(anim, "value_tracks", None) or []:
        kind = getattr(track, "target_kind", "urdf_joint") or "urdf_joint"
        try:
            value = track.value_at(t)
        except Exception as e:                      # noqa: BLE001
            if on_error is not None:
                on_error(getattr(track, "id", "?"), e)
            continue

        if kind == "ws_input":
            target = getattr(track, "target", None) or []
            if len(target) >= 2:
                ws_values[(target[0], target[1])] = value

        elif kind == "pose":
            pose_id = _pose_id(track)
            if not pose_id or pose_lookup is None:
                continue
            # CLIP semantics: outside the span between its first and last
            # keyframe a pose track contributes nothing at all. This is
            # what lets a later pose take over from an earlier one
            # automatically — the earlier clip has ended, rather than
            # holding its last value forever and having to be out-ranked
            # by list position.
            if not _in_span(track, t):
                continue
            try:
                pose_joints = pose_lookup(pose_id)
            except Exception as e:                  # noqa: BLE001
                if on_error is not None:
                    on_error(getattr(track, "id", "?"), e)
                continue
            if not pose_joints:
                continue
            # Weight, not a joint value: a pose track's curve says "how
            # much of this pose", so it clamps to 0..1 regardless of what
            # the operator drew.
            weight = 0.0 if value < 0.0 else (1.0 if value > 1.0 else value)

            for joint, target_value in pose_joints.items():
                merged = effective_override_keys(
                    track, joint, target_value, base.get(joint, 0.0))
                has_user_keys = _override_curve(track, joint) is not None
                if has_user_keys:
                    # An override REPLACES the layered value for this
                    # joint inside the clip. Anchored to the pose at the
                    # track's own keyframes, free in between — which is
                    # the point: refine one joint without unpicking the
                    # pose that drives the rest.
                    joint_values[joint] = AnimationCurve(
                        name=joint, keys=merged).get_value_at_time(t)
                    continue
                if weight <= 0.0:
                    continue
                current = joint_values.get(joint, base.get(joint, 0.0))
                joint_values[joint] = current + (target_value - current) * weight

        else:   # urdf_joint (default) — the track id IS the joint name
            joint_values[track.id] = value

    return joint_values, ws_values


def relaxed_frame(anim, pose_lookup: Optional[PoseLookup] = None,
                  neutral: Optional[Dict[str, float]] = None
                  ) -> Tuple[Dict[str, float], Dict[WSKey, float]]:
    """The frame to apply when playback stops.

    Every target the animation can touch, driven back to neutral, so
    stopping settles the rig instead of stranding it in the last frame.

    Not the same as ``resolve_frame(anim, t)`` with zero weights: a pose
    track at weight 0 contributes *nothing*, which would leave the
    joints it had been moving wherever the last frame left them. The
    joints a pose names have to be collected explicitly and driven to
    neutral themselves.
    """
    joint_values: Dict[str, float] = {}
    ws_values: Dict[WSKey, float] = {}
    base = neutral or {}

    for track in getattr(anim, "value_tracks", None) or []:
        kind = getattr(track, "target_kind", "urdf_joint") or "urdf_joint"
        if kind == "ws_input":
            target = getattr(track, "target", None) or []
            if len(target) >= 2:
                ws_values[(target[0], target[1])] = 0.0
        elif kind == "pose":
            pose_id = _pose_id(track)
            if not pose_id or pose_lookup is None:
                continue
            try:
                pose_joints = pose_lookup(pose_id) or {}
            except Exception:                       # noqa: BLE001
                continue
            for joint in pose_joints:
                joint_values[joint] = base.get(joint, 0.0)
        else:
            joint_values[track.id] = base.get(track.id, 0.0)

    return joint_values, ws_values


def make_pose_lookup(pose_store) -> PoseLookup:
    """Cache-backed pose lookup over a PoseStore.

    Poses are read from disk, and a 60 fps animation with three pose
    tracks would otherwise hit the filesystem 180 times a second. The
    cache lives for the lifetime of the returned closure, which is one
    playback — so editing a pose mid-playback won't be picked up until
    the next start, and that's the right trade: a pose changing shape
    underneath a running performance is worse than a stale read.
    """
    cache: Dict[str, Optional[Dict[str, float]]] = {}

    def lookup(pose_id: str) -> Optional[Dict[str, float]]:
        if pose_id not in cache:
            pose = pose_store.get(pose_id)
            cache[pose_id] = pose.joint_values() if pose is not None else None
        return cache[pose_id]

    return lookup


def _pose_id(track) -> str:
    """Pose id for a pose track.

    Canonically ``target[0]``. Falls back to the track id so a
    hand-written or migrated track that put the pose id there still
    resolves rather than silently doing nothing.
    """
    target = getattr(track, "target", None) or []
    if target and target[0]:
        return str(target[0])
    return str(getattr(track, "id", "") or "")


def referenced_pose_ids(anim) -> list:
    """Every pose id an animation's tracks reference, in order.

    Lets the editor warn about a track pointing at a deleted pose, and
    lets the player pre-warm its lookup cache.
    """
    out = []
    for track in getattr(anim, "value_tracks", None) or []:
        if (getattr(track, "target_kind", "") or "") != "pose":
            continue
        pose_id = _pose_id(track)
        if pose_id and pose_id not in out:
            out.append(pose_id)
    return out
