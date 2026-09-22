"""Animation frame resolution, including pose-track layering.

The load-bearing property here is that **track order matters**. Pose
tracks layer over each other and over joint tracks in list order, so
reordering rows in the timeline changes the output — which is the whole
reason the editor grew drag-to-reorder.

That is deliberately the opposite of how rig controls compose (additive,
commutative, order-irrelevant — see test_rig.py). Two different problems
that both look like "blending poses":

    pose tracks   sequential in time    order matters   override/lerp
    rig controls  simultaneous          order is moot   additive deltas

The fixtures in FIXTURES below are mirrored verbatim in
``web/src/composables/__tests__/frameResolve.spec.js``. The client
resolves frames itself to drive the 3D viewport, so both
implementations have to agree; keeping one fixture table in two places
makes a divergence show up as a failing test rather than as a rig that
looks different in the viewport than it does on the robot.
"""
from __future__ import annotations

import pytest

from saint_server.animation.frame import (
    referenced_pose_ids,
    relaxed_frame,
    resolve_frame,
)
from saint_server.animation.models import Animation


# Poses are normalized −1..+1, as stored.
POSES = {
    "happy":     {"brow_l": 0.8, "brow_r": 0.8, "mouth": 0.6},
    "mad":       {"brow_l": -0.6, "brow_r": -0.6},
    "mouth_open": {"mouth": 1.0},
}


def pose_lookup(pose_id):
    return POSES.get(pose_id)


def anim(*tracks, duration=1.0):
    """Build an animation from compact track specs.

    ``("pose", "happy", [(t, w), …])`` — a pose track and its weight
    curve. ``("joint", "brow_l", [(t, v), …])`` — a joint track.
    ``("ws", ["sheet", "input"], [(t, v), …])`` — a WS-input track.

    A pose spec may carry a 4th element: ``{joint: [(t, v), …]}`` of
    per-joint override keys.
    """
    value_tracks = []
    for i, spec in enumerate(tracks):
        kind, target, keys = spec[0], spec[1], spec[2]
        overrides = spec[3] if len(spec) > 3 else None
        curve = {"name": f"c{i}",
                 "keys": [{"time": t, "value": v, "interp": 1} for t, v in keys]}
        if kind == "pose":
            track = {"id": f"pose{i}", "name": target,
                     "target_kind": "pose", "target": [target],
                     "curve": curve}
            if overrides:
                track["joint_overrides"] = {
                    joint: {"name": joint, "keys": [
                        {"time": t, "value": v, "interp": 1} for t, v in ks]}
                    for joint, ks in overrides.items()
                }
            value_tracks.append(track)
        elif kind == "joint":
            value_tracks.append({"id": target, "name": target,
                                 "target_kind": "urdf_joint", "target": [],
                                 "curve": curve})
        else:
            value_tracks.append({"id": f"ws{i}", "name": "ws",
                                 "target_kind": "ws_input", "target": target,
                                 "curve": curve})
    return Animation.from_dict({
        "id": "a", "name": "a", "duration": duration,
        "value_tracks": value_tracks,
    })


# ── joint + ws tracks (unchanged behaviour) ────────────────────────


def test_joint_track_sets_its_joint():
    a = anim(("joint", "brow_l", [(0, 0.0), (1, 1.0)]))
    joints, ws = resolve_frame(a, 0.5, pose_lookup)
    assert joints == {"brow_l": pytest.approx(0.5)}
    assert ws == {}


def test_ws_track_never_touches_joints():
    a = anim(("ws", ["sheet1", "in1"], [(0, 0.0), (1, 1.0)]))
    joints, ws = resolve_frame(a, 0.25, pose_lookup)
    assert joints == {}
    assert ws == {("sheet1", "in1"): pytest.approx(0.25)}


def test_track_without_target_kind_defaults_to_joint():
    """Every track authored before target_kind existed must keep
    behaving as a URDF joint keyed by its id."""
    a = Animation.from_dict({
        "id": "a", "name": "a", "duration": 1,
        "value_tracks": [{"id": "brow_l", "name": "brow_l",
                          "curve": {"name": "c", "keys": [
                              {"time": 0, "value": 0.4, "interp": 1}]}}],
    })
    joints, _ = resolve_frame(a, 0.0, pose_lookup)
    assert joints == {"brow_l": pytest.approx(0.4)}


# ── pose tracks ────────────────────────────────────────────────────


def test_pose_track_at_full_weight_applies_the_pose():
    a = anim(("pose", "happy", [(0, 1.0)]))
    joints, _ = resolve_frame(a, 0.0, pose_lookup)
    assert joints == {
        "brow_l": pytest.approx(0.8),
        "brow_r": pytest.approx(0.8),
        "mouth": pytest.approx(0.6),
    }


def test_pose_track_at_half_weight_blends_from_neutral():
    a = anim(("pose", "happy", [(0, 0.5)]))
    joints, _ = resolve_frame(a, 0.0, pose_lookup)
    assert joints["brow_l"] == pytest.approx(0.4)
    assert joints["mouth"] == pytest.approx(0.3)


def test_pose_track_at_zero_weight_contributes_nothing():
    """Not "sets the pose's joints to 0" — it must leave them alone, so
    a pose fading out doesn't fight a track below it."""
    a = anim(("pose", "happy", [(0, 0.0)]))
    joints, _ = resolve_frame(a, 0.0, pose_lookup)
    assert joints == {}


def test_pose_weight_clamps_to_zero_one():
    """The curve is a weight, not a joint value, so whatever the operator
    drew gets clamped."""
    over = anim(("pose", "happy", [(0, 3.0)]))
    assert resolve_frame(over, 0.0, pose_lookup)[0]["brow_l"] == pytest.approx(0.8)
    under = anim(("pose", "happy", [(0, -2.0)]))
    assert resolve_frame(under, 0.0, pose_lookup)[0] == {}


def test_pose_blends_up_from_the_neutral_base():
    a = anim(("pose", "happy", [(0, 0.5)]))
    joints, _ = resolve_frame(a, 0.0, pose_lookup,
                              neutral={"brow_l": 0.2, "mouth": 0.0})
    # 0.2 → 0.8 at half weight lands on 0.5.
    assert joints["brow_l"] == pytest.approx(0.5)
    assert joints["mouth"] == pytest.approx(0.3)


def test_unknown_pose_is_skipped_not_fatal():
    a = anim(("pose", "deleted_pose", [(0, 1.0)]),
             ("joint", "brow_l", [(0, 0.25)]))
    joints, _ = resolve_frame(a, 0.0, pose_lookup)
    assert joints == {"brow_l": pytest.approx(0.25)}


def test_pose_track_with_no_lookup_contributes_nothing():
    a = anim(("pose", "happy", [(0, 1.0)]))
    assert resolve_frame(a, 0.0, pose_lookup=None)[0] == {}


# ── ORDER MATTERS ──────────────────────────────────────────────────


def test_later_pose_track_wins_on_shared_joints():
    a = anim(("pose", "happy", [(0, 1.0)]),
             ("pose", "mad", [(0, 1.0)]))
    joints, _ = resolve_frame(a, 0.0, pose_lookup)
    assert joints["brow_l"] == pytest.approx(-0.6)      # mad, on top
    # happy also set `mouth`, which mad says nothing about — it survives.
    assert joints["mouth"] == pytest.approx(0.6)


def test_reversing_the_order_reverses_the_winner():
    """The pin for reorder-ability: same two tracks, same weights,
    different output purely because of list order."""
    forward = anim(("pose", "happy", [(0, 1.0)]), ("pose", "mad", [(0, 1.0)]))
    reverse = anim(("pose", "mad", [(0, 1.0)]), ("pose", "happy", [(0, 1.0)]))
    assert resolve_frame(forward, 0.0, pose_lookup)[0]["brow_l"] == \
        pytest.approx(-0.6)
    assert resolve_frame(reverse, 0.0, pose_lookup)[0]["brow_l"] == \
        pytest.approx(0.8)


def test_partial_weight_on_top_blends_from_the_layer_below():
    """happy fully applied, then mad at half weight — brow_l goes from
    0.8 halfway toward -0.6, landing at 0.1."""
    a = anim(("pose", "happy", [(0, 1.0)]),
             ("pose", "mad", [(0, 0.5)]))
    joints, _ = resolve_frame(a, 0.0, pose_lookup)
    assert joints["brow_l"] == pytest.approx(0.1)


def test_joint_track_above_a_pose_overrides_it():
    a = anim(("pose", "happy", [(0, 1.0)]),
             ("joint", "brow_l", [(0, -1.0)]))
    joints, _ = resolve_frame(a, 0.0, pose_lookup)
    assert joints["brow_l"] == pytest.approx(-1.0)      # joint track wins
    assert joints["brow_r"] == pytest.approx(0.8)       # untouched by it


def test_joint_track_below_a_pose_is_overridden_by_it():
    a = anim(("joint", "brow_l", [(0, -1.0)]),
             ("pose", "happy", [(0, 1.0)]))
    joints, _ = resolve_frame(a, 0.0, pose_lookup)
    assert joints["brow_l"] == pytest.approx(0.8)       # pose wins


def test_pose_blends_from_a_joint_track_below_it():
    """The interesting case: a pose at partial weight layered over an
    explicitly keyed joint blends from that keyed value, not from
    neutral."""
    a = anim(("joint", "mouth", [(0, 0.0)]),
             ("pose", "mouth_open", [(0, 0.5)]))
    joints, _ = resolve_frame(a, 0.0, pose_lookup)
    assert joints["mouth"] == pytest.approx(0.5)        # 0.0 → 1.0 at 50%


def test_three_layers_compose_bottom_up():
    a = anim(("pose", "happy", [(0, 1.0)]),
             ("pose", "mouth_open", [(0, 1.0)]),
             ("joint", "brow_r", [(0, 0.0)]))
    joints, _ = resolve_frame(a, 0.0, pose_lookup)
    assert joints["brow_l"] == pytest.approx(0.8)       # happy only
    assert joints["mouth"] == pytest.approx(1.0)        # mouth_open over happy
    assert joints["brow_r"] == pytest.approx(0.0)       # joint track on top


# ── relaxing on stop ───────────────────────────────────────────────


def test_relaxed_frame_zeroes_pose_touched_joints():
    """A pose track at weight 0 contributes nothing, so stopping can't
    just resolve at zero weight — it would strand the joints the pose had
    been moving wherever the last frame left them."""
    a = anim(("pose", "happy", [(0, 1.0)]),
             ("joint", "neck", [(0, 0.5)]),
             ("ws", ["s", "i"], [(0, 0.9)]))
    joints, ws = relaxed_frame(a, pose_lookup)
    assert joints == {"brow_l": 0.0, "brow_r": 0.0, "mouth": 0.0, "neck": 0.0}
    assert ws == {("s", "i"): 0.0}


def test_relaxed_frame_settles_to_the_neutral_pose():
    a = anim(("pose", "happy", [(0, 1.0)]))
    joints, _ = relaxed_frame(a, pose_lookup, neutral={"brow_l": 0.2})
    assert joints["brow_l"] == pytest.approx(0.2)
    assert joints["brow_r"] == pytest.approx(0.0)


# ── introspection ──────────────────────────────────────────────────


def test_referenced_pose_ids_in_order_without_duplicates():
    a = anim(("pose", "happy", [(0, 1)]), ("joint", "x", [(0, 1)]),
             ("pose", "mad", [(0, 1)]), ("pose", "happy", [(0, 1)]))
    assert referenced_pose_ids(a) == ["happy", "mad"]


def test_pose_id_falls_back_to_the_track_id():
    """A hand-written or migrated track that put the pose id in `id`
    instead of `target[0]` still resolves."""
    a = Animation.from_dict({
        "id": "a", "name": "a", "duration": 1,
        "value_tracks": [{"id": "happy", "name": "happy",
                          "target_kind": "pose", "target": [],
                          "curve": {"name": "c", "keys": [
                              {"time": 0, "value": 1.0, "interp": 1}]}}],
    })
    joints, _ = resolve_frame(a, 0.0, pose_lookup)
    assert joints["brow_l"] == pytest.approx(0.8)


def test_empty_animation_resolves_to_nothing():
    a = anim()
    assert resolve_frame(a, 0.0, pose_lookup) == ({}, {})


# ── CLIP semantics ─────────────────────────────────────────────────
#
# A pose track contributes only between its first and last keyframe. This
# is what makes handing off between poses automatic: the earlier clip has
# ENDED, so the later one takes over without needing to out-rank it by
# list position. Order only decides where two spans overlap.


def test_pose_contributes_inside_its_span():
    a = anim(("pose", "happy", [(1.0, 1.0), (2.0, 1.0)]), duration=4.0)
    assert resolve_frame(a, 1.5, pose_lookup)[0]["brow_l"] == pytest.approx(0.8)


def test_pose_contributes_nothing_before_its_span():
    a = anim(("pose", "happy", [(1.0, 1.0), (2.0, 1.0)]), duration=4.0)
    assert resolve_frame(a, 0.5, pose_lookup)[0] == {}


def test_pose_contributes_nothing_after_its_span():
    """The crux: a joint track's curve holds its last value forever, but a
    pose CLIP does not. Without this, an earlier pose keeps asserting for
    the rest of the animation and a later one can only win by sitting
    higher in the list."""
    a = anim(("pose", "happy", [(1.0, 1.0), (2.0, 1.0)]), duration=4.0)
    assert resolve_frame(a, 3.0, pose_lookup)[0] == {}


def test_span_endpoints_are_inclusive():
    a = anim(("pose", "happy", [(1.0, 1.0), (2.0, 1.0)]), duration=4.0)
    assert resolve_frame(a, 1.0, pose_lookup)[0]["brow_l"] == pytest.approx(0.8)
    assert resolve_frame(a, 2.0, pose_lookup)[0]["brow_l"] == pytest.approx(0.8)


def test_later_clip_takes_over_regardless_of_list_order():
    """happy 0-2 s, mad 2-4 s, with happy listed ABOVE mad. Under the old
    always-layer model happy would win everywhere; under clips it simply
    stops existing after 2 s."""
    a = anim(("pose", "happy", [(0.0, 1.0), (2.0, 1.0)]),
             ("pose", "mad", [(2.0, 1.0), (4.0, 1.0)]),
             duration=4.0)
    assert resolve_frame(a, 1.0, pose_lookup)[0]["brow_l"] == pytest.approx(0.8)
    assert resolve_frame(a, 3.0, pose_lookup)[0]["brow_l"] == pytest.approx(-0.6)


def test_earlier_clip_takes_over_when_listed_second():
    """Same spans, opposite list order — the handoff is unchanged, because
    it's driven by time rather than by position."""
    a = anim(("pose", "mad", [(2.0, 1.0), (4.0, 1.0)]),
             ("pose", "happy", [(0.0, 1.0), (2.0, 1.0)]),
             duration=4.0)
    assert resolve_frame(a, 1.0, pose_lookup)[0]["brow_l"] == pytest.approx(0.8)
    assert resolve_frame(a, 3.0, pose_lookup)[0]["brow_l"] == pytest.approx(-0.6)


def test_order_still_decides_where_spans_overlap():
    """The one thing reordering is still for."""
    forward = anim(("pose", "happy", [(0.0, 1.0), (3.0, 1.0)]),
                   ("pose", "mad", [(1.0, 1.0), (4.0, 1.0)]), duration=4.0)
    reverse = anim(("pose", "mad", [(1.0, 1.0), (4.0, 1.0)]),
                   ("pose", "happy", [(0.0, 1.0), (3.0, 1.0)]), duration=4.0)
    # t=2 falls inside BOTH spans, so list order breaks the tie.
    assert resolve_frame(forward, 2.0, pose_lookup)[0]["brow_l"] == \
        pytest.approx(-0.6)
    assert resolve_frame(reverse, 2.0, pose_lookup)[0]["brow_l"] == \
        pytest.approx(0.8)


def test_joints_fall_back_to_neutral_once_no_clip_covers_the_playhead():
    a = anim(("pose", "happy", [(0.0, 1.0), (1.0, 1.0)]), duration=4.0)
    joints, _ = resolve_frame(a, 3.0, pose_lookup, neutral={"brow_l": 0.15})
    # Nothing writes brow_l, so the caller's neutral stands unopposed.
    assert "brow_l" not in joints


def test_single_key_pose_is_a_degenerate_zero_length_clip():
    """Worth pinning as a known consequence: this is why "+ Pose" creates
    two keys and why the timeline draws the span as a bar."""
    from saint_server.animation.frame import track_span
    a = anim(("pose", "happy", [(1.0, 1.0)]), duration=4.0)
    assert track_span(a.value_tracks[0]) == (1.0, 1.0)
    assert resolve_frame(a, 1.0, pose_lookup)[0]["brow_l"] == pytest.approx(0.8)
    assert resolve_frame(a, 1.1, pose_lookup)[0] == {}


def test_keyless_pose_track_has_no_span_and_is_skipped():
    a = anim(("pose", "happy", []), duration=4.0)
    from saint_server.animation.frame import track_span
    assert track_span(a.value_tracks[0]) is None
    assert resolve_frame(a, 0.0, pose_lookup)[0] == {}


def test_joint_tracks_still_hold_outside_their_keys():
    """Clip semantics apply to POSE tracks only. A joint track is a plain
    curve and must keep extrapolating, or every existing animation
    changes behaviour."""
    a = anim(("joint", "brow_l", [(1.0, 0.5)]), duration=4.0)
    assert resolve_frame(a, 0.0, pose_lookup)[0]["brow_l"] == pytest.approx(0.5)
    assert resolve_frame(a, 3.0, pose_lookup)[0]["brow_l"] == pytest.approx(0.5)


# ── per-joint overrides ────────────────────────────────────────────
#
# A pose track may refine ONE joint inside its clip without disturbing
# the others. Anchored to the pose's own keyframes, free in between.


def test_override_replaces_the_joint_it_names():
    a = anim(("pose", "happy", [(0.0, 1.0), (2.0, 1.0)],
              {"mouth": [(1.0, -0.9)]}), duration=2.0)
    joints, _ = resolve_frame(a, 1.0, pose_lookup)
    assert joints["mouth"] == pytest.approx(-0.9)     # the override
    assert joints["brow_l"] == pytest.approx(0.8)     # untouched by it


def test_override_is_anchored_to_the_pose_at_parent_keyframes():
    """At a parent keyframe the override must agree with the pose, or the
    locked anchors the UI draws would be a lie."""
    a = anim(("pose", "happy", [(0.0, 1.0), (2.0, 1.0)],
              {"mouth": [(1.0, -0.9)]}), duration=2.0)
    for t in (0.0, 2.0):
        assert resolve_frame(a, t, pose_lookup)[0]["mouth"] == pytest.approx(0.6)


def test_override_interpolates_between_anchor_and_user_key():
    a = anim(("pose", "happy", [(0.0, 1.0), (2.0, 1.0)],
              {"mouth": [(1.0, 0.0)]}), duration=2.0)
    # Anchor 0.6 at t=0, user key 0.0 at t=1 → halfway is 0.3.
    assert resolve_frame(a, 0.5, pose_lookup)[0]["mouth"] == pytest.approx(0.3)


def test_override_anchor_follows_the_weight_curve():
    """Anchors are the pose's own contribution, so a half-weight parent
    key anchors at half the pose value."""
    a = anim(("pose", "happy", [(0.0, 0.5), (2.0, 1.0)],
              {"mouth": [(1.0, 0.0)]}), duration=2.0)
    assert resolve_frame(a, 0.0, pose_lookup)[0]["mouth"] == pytest.approx(0.3)


def test_override_anchor_accounts_for_neutral():
    a = anim(("pose", "happy", [(0.0, 1.0), (2.0, 1.0)],
              {"mouth": [(1.0, 0.0)]}), duration=2.0)
    joints, _ = resolve_frame(a, 0.0, pose_lookup, neutral={"mouth": 0.2})
    # Full weight lands on the pose value regardless of neutral.
    assert joints["mouth"] == pytest.approx(0.6)


def test_override_applies_even_at_zero_weight_inside_the_clip():
    """The override drives the joint directly, so it isn't gated on the
    pose's weight — that's what "replaces" means. Still clip-bounded."""
    a = anim(("pose", "happy", [(0.0, 0.0), (2.0, 0.0)],
              {"mouth": [(1.0, 0.75)]}), duration=2.0)
    assert resolve_frame(a, 1.0, pose_lookup)[0]["mouth"] == pytest.approx(0.75)
    assert "mouth" not in resolve_frame(a, 3.0, pose_lookup)[0]


def test_override_for_a_joint_the_pose_does_not_name_is_ignored():
    a = anim(("pose", "mad", [(0.0, 1.0), (2.0, 1.0)],
              {"mouth": [(1.0, 0.5)]}), duration=2.0)
    # "mad" names brow_l/brow_r only.
    assert "mouth" not in resolve_frame(a, 1.0, pose_lookup)[0]


def test_a_user_key_at_an_anchor_time_wins():
    """Otherwise a float-drifted duplicate produces two keys a hair apart
    and the curve reads as a vertical jump."""
    a = anim(("pose", "happy", [(0.0, 1.0), (2.0, 1.0)],
              {"mouth": [(0.0, -1.0)]}), duration=2.0)
    assert resolve_frame(a, 0.0, pose_lookup)[0]["mouth"] == pytest.approx(-1.0)


def test_override_survives_a_round_trip():
    a = anim(("pose", "happy", [(0.0, 1.0), (2.0, 1.0)],
              {"mouth": [(1.0, -0.4)]}), duration=2.0)
    again = Animation.from_dict(a.to_dict())
    assert resolve_frame(again, 1.0, pose_lookup)[0]["mouth"] == \
        pytest.approx(-0.4)


def test_tracks_without_overrides_serialize_unchanged():
    """Every animation authored before overrides existed must re-save
    byte-identical, or a load/save cycle rewrites the whole library."""
    a = anim(("pose", "happy", [(0.0, 1.0)]))
    assert "joint_overrides" not in a.to_dict()["value_tracks"][0]


def test_effective_override_keys_merges_anchors_and_user_keys():
    from saint_server.animation.frame import effective_override_keys
    a = anim(("pose", "happy", [(0.0, 1.0), (2.0, 1.0)],
              {"mouth": [(0.5, 0.1), (1.5, 0.2)]}), duration=2.0)
    keys = effective_override_keys(a.value_tracks[0], "mouth", 0.6, 0.0)
    assert [round(k.time, 3) for k in keys] == [0.0, 0.5, 1.5, 2.0]
    assert [round(k.value, 3) for k in keys] == [0.6, 0.1, 0.2, 0.6]


def test_failing_track_is_reported_and_skipped():
    """One bad curve must not freeze the rig mid-performance."""
    class Boom:
        id = "boom"
        target_kind = "urdf_joint"
        target = []

        def value_at(self, t):
            raise ValueError("bad curve")

    a = anim(("joint", "good", [(0, 0.5)]))
    a.value_tracks.insert(0, Boom())
    seen = []
    joints, _ = resolve_frame(a, 0.0, pose_lookup,
                              on_error=lambda tid, e: seen.append(tid))
    assert joints == {"good": pytest.approx(0.5)}
    assert seen == ["boom"]
