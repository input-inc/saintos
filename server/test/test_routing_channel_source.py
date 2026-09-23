"""Peripheral input channels as routing sources.

This is what makes a sensor routable rather than merely displayable — a
limit switch tripping a pose, a BMS state-of-charge gating a behaviour.
Before this, peripheral channels were sinks only. See
docs/SENSOR_INPUTS.md (Tier 2).
"""

from saint_server.peripheral_model import (
    InputNode,
    OperatorNode,
    RouteEndpoint,
    SignalNode,
    SystemRouting,
    Wire,
)
from saint_server.router.routing_evaluator import RoutingEvaluator


SWITCH = ("node-a", "limit-1", "latched")


def _sheet_with_channel_input(routing, sheet_id="node-a",
                              channel=SWITCH, target="kangaroo-1"):
    """limit switch `latched` channel → peripheral channel write."""
    sheet = routing.get_sheet(sheet_id)
    sheet.inputs.append(InputNode(
        id="chin1", topic="", field="", kind="channel",
        channel_node_id=channel[0],
        peripheral_id=channel[1],
        channel_id=channel[2],
        label="Front limit",
    ))
    sheet.wires.append(Wire(
        id="w1",
        source=RouteEndpoint(kind="input", parts=["chin1"]),
        sink=RouteEndpoint(kind="peripheral",
                           parts=[sheet_id, target, "target_speed"]),
    ))
    return sheet


def _evaluator(sent, ptype="kangaroo"):
    return RoutingEvaluator(
        ros_bridge=None,
        send_channel=lambda *a, **kw: sent.append(a),
        peripheral_type_lookup=lambda *_: ptype,
    )


def test_channel_reading_propagates_to_a_sink():
    routing = SystemRouting()
    _sheet_with_channel_input(routing)
    sent = []
    ev = _evaluator(sent)
    ev.reconcile(routing)

    assert ev.set_peripheral_channel_value(*SWITCH, 1.0) is True
    assert sent, "channel source never reached the sink"
    assert sent[-1] == ("node-a", "kangaroo-1", "target_speed", 1.0, "kangaroo")


def test_unreferenced_channel_is_ignored():
    """Every channel update on every node flows through here, so an
    unwired sensor must cost nothing and must not evaluate sheets."""
    routing = SystemRouting()
    _sheet_with_channel_input(routing)
    sent = []
    ev = _evaluator(sent)
    ev.reconcile(routing)

    assert ev.set_peripheral_channel_value(
        "node-a", "some-other-sensor", "state", 1.0) is False
    assert sent == []


def test_unchanged_value_does_not_re_evaluate():
    """A sensor sitting still reports on every telemetry tick. Without a
    change gate that would re-run every sheet it touches, forever."""
    routing = SystemRouting()
    _sheet_with_channel_input(routing)
    sent = []
    ev = _evaluator(sent)
    ev.reconcile(routing)

    assert ev.set_peripheral_channel_value(*SWITCH, 1.0) is True
    first = len(sent)
    assert ev.set_peripheral_channel_value(*SWITCH, 1.0) is False
    assert len(sent) == first, "unchanged reading re-evaluated the sheet"

    assert ev.set_peripheral_channel_value(*SWITCH, 0.0) is True
    assert len(sent) > first


def test_channel_addressing_is_per_node():
    """peripheral_ids are only unique within a node, so the same
    (peripheral, channel) on a different node must not collide."""
    routing = SystemRouting()
    _sheet_with_channel_input(routing)
    sent = []
    ev = _evaluator(sent)
    ev.reconcile(routing)

    assert ev.set_peripheral_channel_value(
        "node-b", "limit-1", "latched", 1.0) is False
    assert sent == []


def test_channel_source_through_operator_chain():
    """A raw sensor reading is rarely what you want to command — the
    point of routing it is to transform it first."""
    routing = SystemRouting()
    sheet = routing.get_sheet("node-a")
    sheet.inputs.append(InputNode(
        id="chin1", topic="", field="", kind="channel",
        channel_node_id=SWITCH[0], peripheral_id=SWITCH[1],
        channel_id=SWITCH[2],
    ))
    # invert: 1 (tripped) → 0 (stop), 0 (clear) → 1 (allow)
    sheet.operators.append(OperatorNode(
        id="inv", op="subtract", defaults={"a": 1.0}))
    sheet.wires.append(Wire(
        id="w1",
        source=RouteEndpoint(kind="input", parts=["chin1"]),
        sink=RouteEndpoint(kind="operator", parts=["inv", "b"]),
    ))
    sheet.wires.append(Wire(
        id="w2",
        source=RouteEndpoint(kind="operator", parts=["inv", "out"]),
        sink=RouteEndpoint(kind="peripheral",
                           parts=["node-a", "kangaroo-1", "target_speed"]),
    ))

    sent = []
    ev = _evaluator(sent)
    ev.reconcile(routing)

    ev.set_peripheral_channel_value(*SWITCH, 1.0)
    assert sent[-1][3] == 0.0, "tripped switch should invert to 0"
    ev.set_peripheral_channel_value(*SWITCH, 0.0)
    assert sent[-1][3] == 1.0


def test_channel_source_reaches_another_sheet_via_signal():
    """The 'affects more than one thing' case: a sensor on one node
    driving a sink on another, through the global signal table."""
    routing = SystemRouting()
    a = routing.get_sheet("node-a")
    a.inputs.append(InputNode(
        id="chin1", topic="", field="", kind="channel",
        channel_node_id=SWITCH[0], peripheral_id=SWITCH[1],
        channel_id=SWITCH[2],
    ))
    a.signals.append(SignalNode(id="s1", name="front_limit"))
    a.wires.append(Wire(
        id="wa",
        source=RouteEndpoint(kind="input", parts=["chin1"]),
        sink=RouteEndpoint(kind="signal", parts=["front_limit"]),
    ))

    b = routing.get_sheet("node-b")
    b.signals.append(SignalNode(id="s2", name="front_limit"))
    b.wires.append(Wire(
        id="wb",
        source=RouteEndpoint(kind="signal", parts=["front_limit"]),
        sink=RouteEndpoint(kind="peripheral",
                           parts=["node-b", "led-1", "on"]),
    ))

    sent = []
    ev = _evaluator(sent, ptype="led")
    ev.reconcile(routing)

    # One update is enough: sheet A writes the signal, and the evaluator
    # chases the sheets reading it rather than leaving sheet B to notice
    # on a later tick it may never get.
    ev.set_peripheral_channel_value(*SWITCH, 1.0)
    b_writes = [s for s in sent if s[1] == "led-1"]
    assert b_writes, (
        f"signal did not cross to sheet B; sends were {sent}")
    assert b_writes[-1][3] == 1.0


def test_non_numeric_value_rejected():
    routing = SystemRouting()
    _sheet_with_channel_input(routing)
    sent = []
    ev = _evaluator(sent)
    ev.reconcile(routing)

    assert ev.set_peripheral_channel_value(*SWITCH, None) is False
    assert ev.set_peripheral_channel_value(*SWITCH, "high") is False
    assert sent == []


def test_no_routing_loaded_is_not_an_error():
    """Telemetry can arrive before the graph loads; that must not raise."""
    ev = _evaluator([])
    assert ev.set_peripheral_channel_value(*SWITCH, 1.0) is False


def test_reconcile_prunes_unreferenced_channel_values():
    """A re-pointed sensor input must not inherit the stale reading left
    behind by whatever used to be wired there."""
    routing = SystemRouting()
    _sheet_with_channel_input(routing)
    sent = []
    ev = _evaluator(sent)
    ev.reconcile(routing)
    ev.set_peripheral_channel_value(*SWITCH, 1.0)
    assert ev._channel_values.get(SWITCH) == 1.0

    # Re-point the input at a different channel and reconcile.
    routing2 = SystemRouting()
    _sheet_with_channel_input(
        routing2, channel=("node-a", "limit-2", "latched"))
    ev.reconcile(routing2)

    assert SWITCH not in ev._channel_values, (
        "stale reading survived a reconcile that dropped its input")


def test_input_node_survives_serialization():
    """Sheets persist to disk as dicts. A channel input that loses its
    addressing on reload would come back as a dead node."""
    node = InputNode(
        id="chin1", topic="", field="", kind="channel",
        channel_node_id="node-a", peripheral_id="limit-1",
        channel_id="latched", label="Front limit",
    )
    restored = InputNode.from_dict(node.to_dict())
    assert restored.kind == "channel"
    assert restored.channel_key() == ("node-a", "limit-1", "latched")
    assert restored.label == "Front limit"


def test_legacy_input_dict_still_loads():
    """Sheets saved before channel inputs existed have none of the new
    keys; they must keep loading as topic inputs."""
    legacy = {"id": "in1", "topic": "/joy", "field": "axes[0]",
              "label": "left x", "position": [10, 20]}
    restored = InputNode.from_dict(legacy)
    assert restored.kind == "topic"
    assert restored.topic == "/joy"
    assert restored.channel_key() == ("", "", "")


def test_sheet_add_input_derives_channel_label():
    routing = SystemRouting()
    sheet = routing.get_sheet("node-a")
    node = sheet.add_input(
        topic="", field="", kind="channel",
        channel_node_id="node-a", peripheral_id="limit-1",
        channel_id="latched")
    assert node.label == "limit-1.latched"


def test_snapshot_exposes_channel_input_value():
    """The Routes canvas lights up source pills from this snapshot, so a
    channel input has to appear in the inputs bucket like any other."""
    routing = SystemRouting()
    _sheet_with_channel_input(routing)
    sent = []
    ev = _evaluator(sent)
    ev.reconcile(routing)
    ev.set_peripheral_channel_value(*SWITCH, 1.0)

    snap = ev.get_value_snapshot()
    inputs = snap.get("sheets", {}).get("node-a", {}).get("inputs", {})
    assert inputs.get("chin1") == 1.0, (
        f"channel input missing from snapshot: {snap}")
