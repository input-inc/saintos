"""The editor's Live Preview must dispatch like the player, not beside it.

``preview_animation_frame`` used to apply a frame one setpoint at a time
(``set_ws_input`` / ``set_urdf_joint_value`` per value) while both other
callers of the same idea — the AnimationPlayer and the pose board's
fan-out — went through ``apply_animation_frame``. Two things were wrong
with that, and only the second is visible in a screenshot:

  1. Every intermediate evaluation saw this tick's values for the tracks
     already applied and the PREVIOUS tick's for the rest, so a channel a
     sheet computes from more than one track was briefly driven with a
     blend of two frames that never existed.
  2. It cost one sheet evaluation and one snapshot rebuild PER setpoint
     rather than per frame, which at the editor's ~30 Hz preview rate is
     what made the rig lag the playhead.

These pin the batching and, more importantly, that a previewed frame and
a played frame land the same values on the same channels.
"""
from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.peripheral_model import (
    InputNode, OperatorNode, RouteEndpoint, SystemRouting,
    WebSocketInputNode, Wire,
)
from saint_server.router.routing_evaluator import RoutingEvaluator
from saint_server.webserver.state_manager import StateManager


def _routing():
    """One ws_input and one urdf_joint straight through to a channel,
    plus a sheet where TWO inputs feed one channel — that last one is
    where per-setpoint application showed intermediate blends."""
    r = SystemRouting()

    panel = r.get_sheet("panel")
    panel.ws_inputs.append(WebSocketInputNode(id="lift", label="lift"))
    panel.wires.append(Wire(
        id="w1",
        source=RouteEndpoint(kind="ws_input", parts=["lift"]),
        sink=RouteEndpoint(kind="peripheral", parts=["panel", "maestro-1", "ch0"])))

    arm = r.get_sheet("arm")
    arm.inputs.append(InputNode(id="jin", topic="", field="value",
                                label="j0", kind="urdf_joint", joint="j0"))
    arm.wires.append(Wire(
        id="w2",
        source=RouteEndpoint(kind="input", parts=["jin"]),
        sink=RouteEndpoint(kind="peripheral", parts=["arm", "maestro-2", "ch0"])))

    # add(a, b) -> one channel. Both operands come from the same frame.
    mix = r.get_sheet("mix")
    mix.ws_inputs.append(WebSocketInputNode(id="a", label="a"))
    mix.ws_inputs.append(WebSocketInputNode(id="b", label="b"))
    mix.operators.append(OperatorNode(id="sum", op="add"))
    mix.wires.append(Wire(id="m1",
        source=RouteEndpoint(kind="ws_input", parts=["a"]),
        sink=RouteEndpoint(kind="operator", parts=["sum", "a"])))
    mix.wires.append(Wire(id="m2",
        source=RouteEndpoint(kind="ws_input", parts=["b"]),
        sink=RouteEndpoint(kind="operator", parts=["sum", "b"])))
    mix.wires.append(Wire(id="m3",
        source=RouteEndpoint(kind="operator", parts=["sum", "out"]),
        sink=RouteEndpoint(kind="peripheral", parts=["mix", "maestro-3", "ch0"])))
    return r


@pytest.fixture
def rig(tmp_path):
    """StateManager wired to a recording evaluator."""
    sent = []

    def send_channel(node_id, pid, cid, value, _ptype, owner="stream"):
        sent.append((f"{node_id}/{pid}/{cid}", value))
        return True

    ev = RoutingEvaluator(ros_bridge=None, send_channel=send_channel,
                          peripheral_type_lookup=lambda *_: "maestro",
                          on_values_changed=lambda s: None)
    ev.reconcile(_routing())
    sm = StateManager(config_dir=str(tmp_path))
    sm._routing_evaluator = ev
    return sm, ev, sent


def _last(sent):
    out = {}
    for key, value in sent:
        out[key] = value
    return out


# ── values reach the right channels, with the right sign ────────────


def test_a_positive_ws_input_track_arrives_positive(rig):
    sm, _, sent = rig
    res = sm.preview_animation_frame(
        [{"target_kind": "ws_input", "target": ["panel", "lift"], "value": 0.75}], [])
    assert res["success"] is True
    assert _last(sent)["panel/maestro-1/ch0"] == pytest.approx(0.75)


def test_a_positive_joint_track_arrives_positive(rig):
    sm, _, sent = rig
    sm.preview_animation_frame(
        [{"target_kind": "urdf_joint", "id": "j0", "value": 0.4}], [])
    assert _last(sent)["arm/maestro-2/ch0"] == pytest.approx(0.4)


def test_negative_values_are_not_inverted(rig):
    sm, _, sent = rig
    sm.preview_animation_frame(
        [{"target_kind": "ws_input", "target": ["panel", "lift"], "value": -0.6}], [])
    assert _last(sent)["panel/maestro-1/ch0"] == pytest.approx(-0.6)


def test_a_scrub_of_positive_values_never_dips_negative(rig):
    # The reported symptom: positive keyframes, negative on the wire.
    sm, _, sent = rig
    for v in (0.2, 0.35, 0.5, 0.65, 0.8, 0.65, 0.5):
        sm.preview_animation_frame(
            [{"target_kind": "ws_input", "target": ["panel", "lift"], "value": v}], [])
    lifted = [value for key, value in sent if key == "panel/maestro-1/ch0"]
    assert lifted, "nothing reached the channel"
    assert all(x > 0 for x in lifted), lifted


# ── preview == playback ─────────────────────────────────────────────


def test_preview_matches_what_apply_animation_frame_lands(rig):
    """The player applies frames through apply_animation_frame; preview
    must land identically or the editor lies about what will play."""
    sm, ev, sent = rig
    frame = [
        {"target_kind": "ws_input", "target": ["panel", "lift"], "value": 0.3},
        {"target_kind": "urdf_joint", "id": "j0", "value": -0.2},
    ]
    sm.preview_animation_frame(frame, [])
    previewed = _last(sent)

    sent.clear()
    ev.apply_animation_frame({"j0": -0.2}, {("panel", "lift"): 0.3})
    played = _last(sent)

    assert previewed == played


# ── one evaluation per frame, not per setpoint ──────────────────────


def test_a_multi_track_frame_is_applied_in_one_batch(rig):
    sm, ev, _ = rig
    calls = []
    real = ev.apply_animation_frame

    def spy(joints, ws=None):
        calls.append((dict(joints), dict(ws or {})))
        return real(joints, ws)

    ev.apply_animation_frame = spy
    sm.preview_animation_frame([
        {"target_kind": "ws_input", "target": ["mix", "a"], "value": 0.2},
        {"target_kind": "ws_input", "target": ["mix", "b"], "value": 0.5},
        {"target_kind": "urdf_joint", "id": "j0", "value": 0.1},
    ], [])
    assert len(calls) == 1, "a frame must cost one apply, not one per value"
    joints, ws = calls[0]
    assert joints == {"j0": 0.1}
    assert ws == {("mix", "a"): 0.2, ("mix", "b"): 0.5}


def test_a_channel_fed_by_two_tracks_never_sees_a_split_frame(rig):
    """With per-setpoint application the add() sink was driven once with
    (new a, old b) before the correct value — a blend of two frames."""
    sm, _, sent = rig
    sm.preview_animation_frame([
        {"target_kind": "ws_input", "target": ["mix", "a"], "value": 0.0},
        {"target_kind": "ws_input", "target": ["mix", "b"], "value": 0.0},
    ], [])
    sent.clear()
    sm.preview_animation_frame([
        {"target_kind": "ws_input", "target": ["mix", "a"], "value": 0.4},
        {"target_kind": "ws_input", "target": ["mix", "b"], "value": 0.4},
    ], [])
    seen = [value for key, value in sent if key == "mix/maestro-3/ch0"]
    # 0.4 alone (the split frame) must never appear -- only the sum.
    assert all(v == pytest.approx(0.8) for v in seen), seen


def test_an_empty_frame_is_a_no_op(rig):
    sm, _, sent = rig
    res = sm.preview_animation_frame([], [])
    assert res["success"] is True
    assert res["applied"] == 0
    assert sent == []
