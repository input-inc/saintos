"""Tests for RoutingEvaluator end-to-end signal propagation.

Regression-tests for the bug where chains like
`WS Input → Add → Clamp → Peripheral` silently produced no peripheral
write because `_collect_operator_inputs` was called with the wrong
arity. The exception was being swallowed by set_ws_input's outer
try/except so the failure surfaced as "direct works, math chain
doesn't" rather than a hard crash.
"""

from saint_server.peripheral_model import (
    OperatorNode,
    RouteEndpoint,
    SystemRouting,
    WebSocketInputNode,
    Wire,
)
from saint_server.router.routing_evaluator import RoutingEvaluator


def _build_chained_routing(sheet_id: str = "node-a") -> SystemRouting:
    """WS Input → Add (a) → Clamp (value) → Peripheral motor channel."""
    routing = SystemRouting()
    sheet = routing.get_sheet(sheet_id)
    sheet.ws_inputs.append(WebSocketInputNode(id="wsin1", label="left stick"))
    sheet.operators.append(OperatorNode(id="add1", op="add"))
    sheet.operators.append(OperatorNode(id="clamp1", op="clamp"))

    sheet.wires.append(Wire(
        id="w1",
        source=RouteEndpoint(kind="ws_input", parts=["wsin1"]),
        sink=RouteEndpoint(kind="operator", parts=["add1", "a"]),
    ))
    sheet.wires.append(Wire(
        id="w2",
        source=RouteEndpoint(kind="operator", parts=["add1", "out"]),
        sink=RouteEndpoint(kind="operator", parts=["clamp1", "value"]),
    ))
    sheet.wires.append(Wire(
        id="w3",
        source=RouteEndpoint(kind="operator", parts=["clamp1", "out"]),
        sink=RouteEndpoint(kind="peripheral",
                           parts=[sheet_id, "roboclaw-1", "motor"]),
    ))
    return routing


def test_ws_input_through_operator_chain_reaches_peripheral():
    """Driving a WS input must propagate through every operator and
    land on the peripheral channel write. Reproduces the
    `_collect_operator_inputs` arity bug that returned silently for any
    chained-math wiring."""
    routing = _build_chained_routing()

    sent: list = []
    evaluator = RoutingEvaluator(
        ros_bridge=None,
        send_channel=lambda *a: sent.append(a),
        peripheral_type_lookup=lambda *_: "roboclaw",
    )
    evaluator.reconcile(routing)

    ok = evaluator.set_ws_input("node-a", "wsin1", 0.42)
    assert ok is True
    # The chain is Add(0.42, 0) → 0.42, then Clamp(0.42, -1, 1) → 0.42,
    # then send_channel(node, peripheral, channel, value, ptype).
    assert sent, "send_channel was never called — chain did not propagate"
    assert sent[-1] == ("node-a", "roboclaw-1", "motor", 0.42, "roboclaw")


def test_snapshot_includes_operator_and_peripheral_buckets():
    """The value snapshot must contain operator outputs (so the routing
    canvas can light up the math nodes' `out` pin pills) and the
    peripheral pin values (so the operator can confirm the value
    reached hardware)."""
    routing = _build_chained_routing()
    evaluator = RoutingEvaluator(
        ros_bridge=None,
        send_channel=lambda *a: None,
        peripheral_type_lookup=lambda *_: "roboclaw",
    )
    evaluator.reconcile(routing)

    evaluator.set_ws_input("node-a", "wsin1", 0.42)
    snap = evaluator.get_value_snapshot()
    sheet = snap["sheets"]["node-a"]
    assert sheet["ws_inputs"] == {"wsin1": 0.42}
    assert sheet["operators"] == {"add1": 0.42, "clamp1": 0.42}
    assert sheet["peripherals"] == {"node-a/roboclaw-1/motor": 0.42}


def test_orphan_operator_evaluates_via_pass3():
    """An operator chain not terminating at any sink should still
    populate the operators bucket (Pass 3 eager-eval)."""
    routing = SystemRouting()
    sheet = routing.get_sheet("node-b")
    sheet.ws_inputs.append(WebSocketInputNode(id="wsin1"))
    sheet.operators.append(OperatorNode(id="add1", op="add"))
    sheet.wires.append(Wire(
        id="w1",
        source=RouteEndpoint(kind="ws_input", parts=["wsin1"]),
        sink=RouteEndpoint(kind="operator", parts=["add1", "a"]),
    ))
    # Note: no wire from add1.out to anything.

    evaluator = RoutingEvaluator(
        ros_bridge=None,
        send_channel=lambda *a: None,
        peripheral_type_lookup=lambda *_: "",
    )
    evaluator.reconcile(routing)
    evaluator.set_ws_input("node-b", "wsin1", 0.7)
    snap = evaluator.get_value_snapshot()
    assert snap["sheets"]["node-b"]["operators"] == {"add1": 0.7}


def test_clamp_constrains_input_to_range():
    """Sanity-check the operator math itself: clamp should bound the
    value into [min, max]."""
    routing = _build_chained_routing()
    evaluator = RoutingEvaluator(
        ros_bridge=None,
        send_channel=lambda *a: None,
        peripheral_type_lookup=lambda *_: "",
    )
    evaluator.reconcile(routing)
    evaluator.set_ws_input("node-a", "wsin1", 5.0)  # well above max=1
    snap = evaluator.get_value_snapshot()
    assert snap["sheets"]["node-a"]["operators"]["clamp1"] == 1.0
