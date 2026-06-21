"""Equivalence guard for the animation-frame dispatch refactor.

An animation tick delivers N URDF-joint values that are all known at the
same instant. Today the player pushes them one at a time via
``set_urdf_joint_value`` (re-evaluating the shared sheet and rebuilding
the UI snapshot once PER joint). The optimization collapses that into a
single ``apply_animation_frame(joint_values)`` call.

These tests pin the OBSERVABLE result of a frame — the latest commanded
value at every peripheral sink, and the value snapshot — so the batch
path can be proven to produce byte-identical output to the per-track
path it replaces. They assert on FINAL state, not on dispatch counts:
how many times an unchanged sink is re-sent within a frame is an
implementation detail (newest-wins control path).

If ``apply_animation_frame`` isn't implemented yet, the equivalence
tests skip (the per-track reference values are still computed and the
self-consistency test still runs).
"""
from __future__ import annotations

import math
from typing import Dict, List, Tuple

import pytest

from saint_server.peripheral_model import (
    InputNode,
    OperatorNode,
    RouteEndpoint,
    SystemRouting,
    WebSocketInputNode,
    Wire,
)
from saint_server.router.routing_evaluator import RoutingEvaluator


# ── harness ─────────────────────────────────────────────────────────


def _build_rig(joints: int = 6, depth: int = 3) -> Tuple[SystemRouting, List[str]]:
    """Multi-DOF arm on one sheet, plus a second sheet that shares one of
    the joints — so we also cover the multi-sheet-touch case (a joint
    wired into two sheets must re-evaluate both)."""
    routing = SystemRouting()
    joint_names: List[str] = []
    chain_ops = ["deadband", "scale", "clamp", "curve", "invert"]

    arm = routing.get_sheet("arm")
    for i in range(joints):
        joint = f"joint_{i}"
        joint_names.append(joint)
        inp_id = f"jin{i}"
        arm.inputs.append(InputNode(id=inp_id, topic="", field="value",
                                    label=joint, kind="urdf_joint", joint=joint))
        prev = RouteEndpoint(kind="input", parts=[inp_id])
        for d in range(depth):
            op_id = f"op{i}_{d}"
            arm.operators.append(OperatorNode(id=op_id, op=chain_ops[d % len(chain_ops)]))
            arm.wires.append(Wire(id=f"w{i}_{d}",
                                  source=prev,
                                  sink=RouteEndpoint(kind="operator", parts=[op_id, "value"])))
            prev = RouteEndpoint(kind="operator", parts=[op_id, "out"])
        arm.wires.append(Wire(id=f"w{i}_sink", source=prev,
                              sink=RouteEndpoint(kind="peripheral",
                                                 parts=["arm", "maestro-1", f"ch{i}"])))

    # Second sheet taps joint_0 into a different peripheral — exercises a
    # joint that touches more than one sheet per frame.
    head = routing.get_sheet("head")
    head.inputs.append(InputNode(id="hin", topic="", field="value",
                                 label="joint_0", kind="urdf_joint", joint="joint_0"))
    head.operators.append(OperatorNode(id="hop", op="scale"))
    head.wires.append(Wire(id="hw_in",
                           source=RouteEndpoint(kind="input", parts=["hin"]),
                           sink=RouteEndpoint(kind="operator", parts=["hop", "value"])))
    head.wires.append(Wire(id="hw_sink",
                           source=RouteEndpoint(kind="operator", parts=["hop", "out"]),
                           sink=RouteEndpoint(kind="peripheral",
                                              parts=["head", "maestro-2", "ch0"])))

    # Third sheet driven by ws_input value tracks (no URDF involved) —
    # an animation can target ws_inputs and urdf_joints on DIFFERENT
    # sheets in the same frame. "panel" carries two ws_inputs, each
    # through its own chain into a separate peripheral channel.
    panel = routing.get_sheet("panel")
    for j, ws_id in enumerate(("led", "tilt")):
        panel.ws_inputs.append(WebSocketInputNode(id=ws_id, label=ws_id))
        panel.operators.append(OperatorNode(id=f"pop{j}", op="scale"))
        panel.wires.append(Wire(id=f"pw{j}_in",
                                source=RouteEndpoint(kind="ws_input", parts=[ws_id]),
                                sink=RouteEndpoint(kind="operator", parts=[f"pop{j}", "value"])))
        panel.wires.append(Wire(id=f"pw{j}_sink",
                                source=RouteEndpoint(kind="operator", parts=[f"pop{j}", "out"]),
                                sink=RouteEndpoint(kind="peripheral",
                                                   parts=["panel", "maestro-3", f"ch{j}"])))
    return routing, joint_names


# ws_input value-track targets the rig above exposes: (sheet_id, input_id).
_WS_TARGETS = [("panel", "led"), ("panel", "tilt")]


def _ws_frame(tick: int) -> Dict[Tuple[str, str], float]:
    return {tgt: math.cos(tick * 0.07 + k) for k, tgt in enumerate(_WS_TARGETS)}


def _make_evaluator(routing):
    last_by_sink: Dict[str, float] = {}
    snapshots: List[dict] = []

    def send_channel(node_id, peripheral_id, channel_id, value, _ptype):
        last_by_sink[f"{node_id}/{peripheral_id}/{channel_id}"] = value

    def on_values_changed(snap):
        snapshots.append(snap)

    ev = RoutingEvaluator(
        ros_bridge=None,
        send_channel=send_channel,
        peripheral_type_lookup=lambda *_: "maestro",
        on_values_changed=on_values_changed,
    )
    ev.reconcile(routing)
    return ev, last_by_sink, snapshots


def _frame(joint_names: List[str], tick: int) -> Dict[str, float]:
    n = len(joint_names)
    return {jn: math.sin(tick * 0.05 + i * (math.pi / max(1, n)))
            for i, jn in enumerate(joint_names)}


def _has_batch() -> bool:
    return hasattr(RoutingEvaluator, "apply_animation_frame")


# ── tests ───────────────────────────────────────────────────────────


def test_per_track_reaches_all_sinks():
    """Self-consistency: per-track dispatch of a frame commands every
    joint's peripheral channel + the shared-joint head channel."""
    routing, joints = _build_rig(joints=6, depth=3)
    ev, last_by_sink, _ = _make_evaluator(routing)
    for jn, v in _frame(joints, 7).items():
        ev.set_urdf_joint_value(jn, v)
    for i in range(6):
        assert f"arm/maestro-1/ch{i}" in last_by_sink
    assert "head/maestro-2/ch0" in last_by_sink


@pytest.mark.skipif(not _has_batch(),
                    reason="apply_animation_frame not implemented yet")
def test_batch_frame_matches_per_track_sinks():
    """A frame applied via apply_animation_frame produces byte-identical
    peripheral sink values to the same frame applied per-track."""
    routing_a, joints = _build_rig(joints=6, depth=3)
    ev_a, sinks_a, _ = _make_evaluator(routing_a)

    routing_b, _ = _build_rig(joints=6, depth=3)
    ev_b, sinks_b, _ = _make_evaluator(routing_b)

    for tick in (3, 11, 29):
        frame = _frame(joints, tick)
        for jn, v in frame.items():
            ev_a.set_urdf_joint_value(jn, v)
        ev_b.apply_animation_frame(dict(frame))

    assert sinks_a == sinks_b


@pytest.mark.skipif(not _has_batch(),
                    reason="apply_animation_frame not implemented yet")
def test_batch_frame_matches_per_track_snapshot():
    """Final value snapshot after a frame is identical between paths."""
    routing_a, joints = _build_rig(joints=6, depth=3)
    ev_a, _, _ = _make_evaluator(routing_a)
    routing_b, _ = _build_rig(joints=6, depth=3)
    ev_b, _, _ = _make_evaluator(routing_b)

    frame = _frame(joints, 17)
    for jn, v in frame.items():
        ev_a.set_urdf_joint_value(jn, v)
    ev_b.apply_animation_frame(dict(frame))

    assert ev_a.get_value_snapshot() == ev_b.get_value_snapshot()


@pytest.mark.skipif(not _has_batch(),
                    reason="apply_animation_frame not implemented yet")
def test_batch_frame_broadcasts_once_per_frame():
    """The whole point: one snapshot broadcast per frame, not one per
    joint. (Per-track does N; batch does 1.)"""
    routing, joints = _build_rig(joints=6, depth=3)
    ev, _, snapshots = _make_evaluator(routing)
    ev.apply_animation_frame(_frame(joints, 5))
    assert len(snapshots) == 1


@pytest.mark.skipif(not _has_batch(),
                    reason="apply_animation_frame not implemented yet")
def test_batch_frame_empty_is_noop():
    """An empty frame touches nothing and broadcasts nothing."""
    routing, _ = _build_rig(joints=3, depth=2)
    ev, sinks, snapshots = _make_evaluator(routing)
    ev.apply_animation_frame({})
    assert sinks == {}
    assert snapshots == []


def test_per_track_mixed_reaches_all_sinks():
    """Self-consistency for a MIXED frame: urdf_joint tracks (arm+head)
    and ws_input tracks (panel) dispatched per-track reach every sink."""
    routing, joints = _build_rig(joints=6, depth=3)
    ev, sinks, _ = _make_evaluator(routing)
    for jn, v in _frame(joints, 9).items():
        ev.set_urdf_joint_value(jn, v)
    for (sid, iid), v in _ws_frame(9).items():
        ev.set_ws_input(sid, iid, v)
    for i in range(6):
        assert f"arm/maestro-1/ch{i}" in sinks
    assert "head/maestro-2/ch0" in sinks
    assert "panel/maestro-3/ch0" in sinks
    assert "panel/maestro-3/ch1" in sinks


@pytest.mark.skipif(not _has_batch(),
                    reason="apply_animation_frame not implemented yet")
def test_batch_mixed_frame_matches_per_track():
    """The case the operator actually hits: one animation frame drives
    urdf_joints on the arm/head sheets AND ws_inputs on the panel sheet.
    Batch (joint_values + ws_values in one call) must produce identical
    sinks and snapshot to per-track set_urdf_joint_value + set_ws_input."""
    routing_a, joints = _build_rig(joints=6, depth=3)
    ev_a, sinks_a, _ = _make_evaluator(routing_a)
    routing_b, _ = _build_rig(joints=6, depth=3)
    ev_b, sinks_b, _ = _make_evaluator(routing_b)

    for tick in (4, 13, 27):
        jframe = _frame(joints, tick)
        wframe = _ws_frame(tick)
        # Per-track: the order the player used before batching — value
        # tracks fan out one at a time regardless of target kind.
        for jn, v in jframe.items():
            ev_a.set_urdf_joint_value(jn, v)
        for (sid, iid), v in wframe.items():
            ev_a.set_ws_input(sid, iid, v)
        # Batch: both kinds in a single frame call.
        ev_b.apply_animation_frame(dict(jframe), dict(wframe))

    assert sinks_a == sinks_b
    assert ev_a.get_value_snapshot() == ev_b.get_value_snapshot()


@pytest.mark.skipif(not _has_batch(),
                    reason="apply_animation_frame not implemented yet")
def test_batch_mixed_frame_broadcasts_once():
    """A mixed frame spanning three sheets still broadcasts exactly once
    (per-track would broadcast once per joint + once per ws_input)."""
    routing, joints = _build_rig(joints=6, depth=3)
    ev, _, snapshots = _make_evaluator(routing)
    ev.apply_animation_frame(dict(_frame(joints, 6)), dict(_ws_frame(6)))
    assert len(snapshots) == 1
