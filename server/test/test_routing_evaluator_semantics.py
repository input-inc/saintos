"""Semantics lock-down for RoutingEvaluator.

These tests pin the OBSERVABLE behavior of the evaluator so a later
refactor toward dirty-subgraph (per-input) evaluation can be proven not
to change results. They deliberately assert on the *latest commanded
value per sink* and on snapshot contents — NOT on how many times a sink
was dispatched — because whether an unchanged chain is re-sent on an
unrelated input's tick is an implementation detail (whole-sheet eval
re-sends it; dirty-subgraph eval won't, and that's fine for the
newest-wins control path).

Coverage:
  * topic-input and urdf-joint-input drive paths
  * operator fan-in (shared op) and fan-out (one input → many sinks)
  * independent chains: changing one input leaves others' values correct
  * Output node dispatch + same-sheet output-tap ordering
  * cross-sheet signal read/write
  * e-stop gate (peripheral/output suppressed, widget still updates)
  * widget sinks
  * cycle guard + stale-wire tolerance
  * snapshot completeness across successive ticks
"""
from __future__ import annotations

from typing import Dict, List, Tuple

from saint_server.peripheral_model import (
    InputNode,
    OperatorNode,
    OutputNode,
    RouteEndpoint,
    SignalNode,
    SystemRouting,
    WebSocketInputNode,
    WidgetInstance,
    Wire,
)
from saint_server.router.routing_evaluator import RoutingEvaluator


# ── harness ─────────────────────────────────────────────────────────


class _Bridge:
    """Captures set_topic_channel calls (Output-node dispatch)."""
    def __init__(self):
        self.topic_writes: List[Tuple[str, str, float, str]] = []
        self.subscribed: List[str] = []

    def set_topic_channel(self, topic, field, value, client_id=""):
        self.topic_writes.append((topic, field, value, client_id))

    def subscribe(self, topic, client_id):
        self.subscribed.append(topic)

    def unsubscribe(self, topic, client_id):
        pass


def _make_evaluator(routing, bridge=None, ptype="roboclaw"):
    """Build an evaluator; returns (evaluator, sends, last_by_sink).

    `sends` is the ordered list of (node, periph, channel, value, ptype);
    `last_by_sink` maps "node/periph/channel" → most-recent value.
    """
    sends: List[tuple] = []
    last_by_sink: Dict[str, float] = {}

    def send_channel(node_id, peripheral_id, channel_id, value, ptype_):
        sends.append((node_id, peripheral_id, channel_id, value, ptype_))
        last_by_sink[f"{node_id}/{peripheral_id}/{channel_id}"] = value

    ev = RoutingEvaluator(
        ros_bridge=bridge,
        send_channel=send_channel,
        peripheral_type_lookup=lambda *_: ptype,
    )
    ev.reconcile(routing)
    return ev, sends, last_by_sink


def _peri(node, periph, channel):
    return RouteEndpoint(kind="peripheral", parts=[node, periph, channel])


# ── topic + urdf inputs ─────────────────────────────────────────────


def test_topic_input_drives_peripheral():
    routing = SystemRouting()
    sheet = routing.get_sheet("n")
    sheet.inputs.append(InputNode(id="i1", topic="/joy", field="axes[1]"))
    sheet.wires.append(Wire(
        id="w", source=RouteEndpoint(kind="input", parts=["i1"]),
        sink=_peri("n", "roboclaw-1", "motor")))
    bridge = _Bridge()
    ev, sends, last = _make_evaluator(routing, bridge)

    ev.on_topic_message("/joy", {"axes": [0.0, 0.75]})
    assert last["n/roboclaw-1/motor"] == 0.75
    # And the evaluator subscribed to the topic on reconcile.
    assert "/joy" in bridge.subscribed


def test_urdf_joint_input_drives_peripheral():
    routing = SystemRouting()
    sheet = routing.get_sheet("n")
    sheet.inputs.append(InputNode(id="j", topic="", field="",
                                  kind="urdf_joint", joint="pan"))
    sheet.wires.append(Wire(
        id="w", source=RouteEndpoint(kind="input", parts=["j"]),
        sink=_peri("n", "maestro-1", "ch0")))
    ev, sends, last = _make_evaluator(routing, ptype="maestro")

    ev.set_urdf_joint_value("pan", -0.4)
    assert last["n/maestro-1/ch0"] == -0.4


# ── fan-in / fan-out ────────────────────────────────────────────────


def test_operator_fan_in_recomputes_on_either_input():
    """Two WS inputs feed one `add`; changing either must update the
    shared operator's output to the sum of the LATEST values of both."""
    routing = SystemRouting()
    sheet = routing.get_sheet("n")
    sheet.ws_inputs.append(WebSocketInputNode(id="a"))
    sheet.ws_inputs.append(WebSocketInputNode(id="b"))
    sheet.operators.append(OperatorNode(id="sum", op="add"))
    sheet.wires += [
        Wire(id="wa", source=RouteEndpoint(kind="ws_input", parts=["a"]),
             sink=RouteEndpoint(kind="operator", parts=["sum", "a"])),
        Wire(id="wb", source=RouteEndpoint(kind="ws_input", parts=["b"]),
             sink=RouteEndpoint(kind="operator", parts=["sum", "b"])),
        Wire(id="ws", source=RouteEndpoint(kind="operator", parts=["sum", "out"]),
             sink=_peri("n", "roboclaw-1", "motor")),
    ]
    ev, sends, last = _make_evaluator(routing)

    ev.set_ws_input("n", "a", 0.3)
    assert abs(last["n/roboclaw-1/motor"] - 0.3) < 1e-9
    ev.set_ws_input("n", "b", 0.4)               # changing b must see a=0.3
    assert abs(last["n/roboclaw-1/motor"] - 0.7) < 1e-9
    ev.set_ws_input("n", "a", -0.1)              # changing a must see b=0.4
    assert abs(last["n/roboclaw-1/motor"] - 0.3) < 1e-9


def test_operator_fan_out_updates_all_sinks():
    """One WS input feeds two operators → two peripherals; one tick
    must update both peripheral sinks."""
    routing = SystemRouting()
    sheet = routing.get_sheet("n")
    sheet.ws_inputs.append(WebSocketInputNode(id="a"))
    sheet.operators.append(OperatorNode(id="pos", op="scale",
                                         defaults={"factor": 1.0}))
    sheet.operators.append(OperatorNode(id="neg", op="invert"))
    sheet.wires += [
        Wire(id="w1", source=RouteEndpoint(kind="ws_input", parts=["a"]),
             sink=RouteEndpoint(kind="operator", parts=["pos", "value"])),
        Wire(id="w2", source=RouteEndpoint(kind="ws_input", parts=["a"]),
             sink=RouteEndpoint(kind="operator", parts=["neg", "value"])),
        Wire(id="w3", source=RouteEndpoint(kind="operator", parts=["pos", "out"]),
             sink=_peri("n", "p", "left")),
        Wire(id="w4", source=RouteEndpoint(kind="operator", parts=["neg", "out"]),
             sink=_peri("n", "p", "right")),
    ]
    ev, sends, last = _make_evaluator(routing)

    ev.set_ws_input("n", "a", 0.5)
    assert abs(last["n/p/left"] - 0.5) < 1e-9
    assert abs(last["n/p/right"] - (-0.5)) < 1e-9


def test_independent_chains_keep_correct_values():
    """The Option-B scenario: many independent input→peripheral chains
    on one sheet. After driving several, EACH peripheral must hold the
    value implied by ITS OWN latest input — regardless of which input
    was touched last."""
    routing = SystemRouting()
    sheet = routing.get_sheet("head")
    n = 6
    for i in range(n):
        sheet.ws_inputs.append(WebSocketInputNode(id=f"in{i}"))
        sheet.operators.append(OperatorNode(id=f"clamp{i}", op="clamp"))
        sheet.wires += [
            Wire(id=f"a{i}", source=RouteEndpoint(kind="ws_input", parts=[f"in{i}"]),
                 sink=RouteEndpoint(kind="operator", parts=[f"clamp{i}", "value"])),
            Wire(id=f"b{i}", source=RouteEndpoint(kind="operator", parts=[f"clamp{i}", "out"]),
                 sink=_peri("head", "maestro-1", f"ch{i}")),
        ]
    ev, sends, last = _make_evaluator(routing, ptype="maestro")

    expected = {}
    # Drive each input once, in an interleaved/out-of-order pattern.
    for i in [0, 3, 1, 5, 2, 4]:
        v = round(0.1 * (i + 1), 3)
        ev.set_ws_input("head", f"in{i}", v)
        expected[f"head/maestro-1/ch{i}"] = v
    # Re-drive a couple to make sure earlier ones aren't clobbered.
    ev.set_ws_input("head", "in0", -0.2); expected["head/maestro-1/ch0"] = -0.2
    ev.set_ws_input("head", "in4", 0.9);  expected["head/maestro-1/ch4"] = 0.9

    for key, want in expected.items():
        assert abs(last[key] - want) < 1e-9, f"{key}: got {last.get(key)} want {want}"
    # Snapshot must agree with the last dispatched values.
    snap = ev.get_value_snapshot()["sheets"]["head"]["peripherals"]
    for key, want in expected.items():
        assert abs(snap[key] - want) < 1e-9


# ── outputs + output-tap ordering ───────────────────────────────────


def test_output_dispatch_and_same_sheet_tap():
    """An Output node publishes to the bridge, and a peripheral wire on
    the same sheet taps that output as its source — the peripheral must
    see the output value computed earlier in the same evaluation."""
    routing = SystemRouting()
    sheet = routing.get_sheet("n")
    sheet.ws_inputs.append(WebSocketInputNode(id="a"))
    sheet.outputs.append(OutputNode(id="o1", topic="/cmd", field="value"))
    sheet.wires += [
        Wire(id="w1", source=RouteEndpoint(kind="ws_input", parts=["a"]),
             sink=RouteEndpoint(kind="output", parts=["o1"])),
        Wire(id="w2", source=RouteEndpoint(kind="output", parts=["o1"]),
             sink=_peri("n", "p", "motor")),
    ]
    bridge = _Bridge()
    ev, sends, last = _make_evaluator(routing, bridge)

    ev.set_ws_input("n", "a", 0.66)
    assert bridge.topic_writes[-1][:3] == ("/cmd", "value", 0.66)
    assert abs(last["n/p/motor"] - 0.66) < 1e-9


# ── cross-sheet signals ─────────────────────────────────────────────


def test_signal_written_on_one_sheet_read_on_another():
    """Cross-sheet signal contract (verified against current code):

      1. Writing a signal on sheet A updates the global signal table but
         writes no peripheral on A.
      2. Triggering sheet B by ITS OWN unrelated input re-evaluates ALL
         of B's sinks — including the peripheral fed by the signal — so
         the signal value lands on B's motor.

    Point 2 is the load-bearing invariant: a sheet trigger must visit
    every sink, not just the triggering input's subgraph, because
    signals are mutated out-of-band by other sheets. Any incremental-
    eval optimization MUST preserve this."""
    routing = SystemRouting()
    a = routing.get_sheet("sheetA")
    a.ws_inputs.append(WebSocketInputNode(id="src"))
    a.signals.append(SignalNode(id="sigA", name="speed"))
    a.wires.append(Wire(
        id="wA", source=RouteEndpoint(kind="ws_input", parts=["src"]),
        sink=RouteEndpoint(kind="signal", parts=["speed"])))

    b = routing.get_sheet("sheetB")
    b.ws_inputs.append(WebSocketInputNode(id="tick"))
    b.signals.append(SignalNode(id="sigB", name="speed"))
    b.wires.append(Wire(
        id="wB", source=RouteEndpoint(kind="signal", parts=["speed"]),
        sink=_peri("sheetB", "p", "motor")))
    b.wires.append(Wire(
        id="wAux", source=RouteEndpoint(kind="ws_input", parts=["tick"]),
        sink=_peri("sheetB", "p", "aux")))

    ev, sends, last = _make_evaluator(routing)

    ev.set_ws_input("sheetA", "src", 0.8)
    assert ev._signals["speed"] == 0.8
    assert "sheetB/p/motor" not in last      # A's write didn't touch B yet

    # Trigger B via its own unrelated input — the signal-fed motor sink
    # must be (re)written even though "tick" doesn't feed it.
    ev.set_ws_input("sheetB", "tick", 0.1)
    assert abs(last["sheetB/p/motor"] - 0.8) < 1e-9
    assert abs(last["sheetB/p/aux"] - 0.1) < 1e-9

    # A later signal change is picked up on B's next trigger.
    ev.set_ws_input("sheetA", "src", -0.5)
    ev.set_ws_input("sheetB", "tick", 0.2)
    assert abs(last["sheetB/p/motor"] - (-0.5)) < 1e-9


# ── e-stop gate ─────────────────────────────────────────────────────


def test_estop_suppresses_peripheral_but_not_widget():
    routing = SystemRouting()
    sheet = routing.get_sheet("n")
    sheet.ws_inputs.append(WebSocketInputNode(id="a"))
    sheet.widgets.append(WidgetInstance(id="wg", type="gauge", label="G"))
    sheet.wires += [
        Wire(id="wp", source=RouteEndpoint(kind="ws_input", parts=["a"]),
             sink=_peri("n", "p", "motor")),
        Wire(id="ww", source=RouteEndpoint(kind="ws_input", parts=["a"]),
             sink=RouteEndpoint(kind="widget", parts=["wg", "value"])),
    ]
    ev, sends, last = _make_evaluator(routing)

    ev.set_estop_active(True)
    ev.set_ws_input("n", "a", 0.5)
    assert "n/p/motor" not in last, "peripheral write must be suppressed under estop"
    assert ev.widget_value("wg", "value") == 0.5, "widget must still update under estop"

    ev.set_estop_active(False)
    ev.set_ws_input("n", "a", 0.6)
    assert abs(last["n/p/motor"] - 0.6) < 1e-9, "peripheral resumes after estop release"


# ── robustness ──────────────────────────────────────────────────────


def test_cycle_guard_does_not_crash():
    routing = SystemRouting()
    sheet = routing.get_sheet("n")
    sheet.operators.append(OperatorNode(id="x", op="add"))
    # x.a ← x.out  (self-cycle)
    sheet.wires += [
        Wire(id="c", source=RouteEndpoint(kind="operator", parts=["x", "out"]),
             sink=RouteEndpoint(kind="operator", parts=["x", "a"])),
        Wire(id="s", source=RouteEndpoint(kind="operator", parts=["x", "out"]),
             sink=_peri("n", "p", "m")),
    ]
    sheet.ws_inputs.append(WebSocketInputNode(id="drv"))
    ev, sends, last = _make_evaluator(routing)
    # Should evaluate without raising; cycle breaks to 0.0.
    ev.set_ws_input("n", "drv", 1.0)
    assert last.get("n/p/m") == 0.0


def test_stale_operator_wire_is_tolerated():
    routing = SystemRouting()
    sheet = routing.get_sheet("n")
    sheet.ws_inputs.append(WebSocketInputNode(id="a"))
    # Wire references operator "ghost" that doesn't exist on the sheet.
    sheet.wires.append(Wire(
        id="w", source=RouteEndpoint(kind="operator", parts=["ghost", "out"]),
        sink=_peri("n", "p", "m")))
    ev, sends, last = _make_evaluator(routing)
    ev.set_ws_input("n", "a", 1.0)   # must not raise
    # ghost resolves to 0 → peripheral commanded 0.0
    assert last.get("n/p/m") == 0.0


def test_snapshot_reflects_latest_across_ticks():
    routing = SystemRouting()
    sheet = routing.get_sheet("n")
    sheet.ws_inputs.append(WebSocketInputNode(id="a"))
    sheet.ws_inputs.append(WebSocketInputNode(id="b"))
    sheet.operators.append(OperatorNode(id="m", op="multiply"))
    sheet.wires += [
        Wire(id="wa", source=RouteEndpoint(kind="ws_input", parts=["a"]),
             sink=RouteEndpoint(kind="operator", parts=["m", "a"])),
        Wire(id="wb", source=RouteEndpoint(kind="ws_input", parts=["b"]),
             sink=RouteEndpoint(kind="operator", parts=["m", "b"])),
        Wire(id="wp", source=RouteEndpoint(kind="operator", parts=["m", "out"]),
             sink=_peri("n", "p", "m")),
    ]
    ev, sends, last = _make_evaluator(routing)

    ev.set_ws_input("n", "a", 2.0)
    ev.set_ws_input("n", "b", 3.0)
    snap = ev.get_value_snapshot()["sheets"]["n"]
    assert snap["ws_inputs"] == {"a": 2.0, "b": 3.0}
    assert abs(snap["operators"]["m"] - 6.0) < 1e-9
    assert abs(snap["peripherals"]["n/p/m"] - 6.0) < 1e-9
