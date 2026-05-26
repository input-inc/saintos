"""Auto-migration of state-only ROS InputNodes to WebSocketInputNodes.

Before the WS-input refactor the controller's bindings drove ROS topic
channels directly via the bridge's `set_topic_channel`. State-only
endpoints (e.g. /saint/track) silently rejected those writes. The new
path expects bindings to address WebSocketInputNodes on routing sheets;
old sheets persisted to disk should be rewritten on load so they keep
working.
"""

import os
import textwrap
import tempfile

import pytest

from saint_server.webserver.state_manager import StateManager, SystemState


@pytest.fixture
def sm(tmp_path):
    # Construct a StateManager without going through __init__ — the real
    # constructor scans hardware paths and loads board configs we don't
    # need here. We only exercise _load_system_routing + the migration.
    s = StateManager.__new__(StateManager)
    s.state = SystemState()
    s.logger = None
    s._routing_evaluator = None
    s.system_routing_path = str(tmp_path / "system_routing.yaml")
    s.config_dir = str(tmp_path)
    return s


def _write_routing(path: str, body: str) -> None:
    with open(path, "w") as f:
        f.write(body)


def test_state_only_input_gets_migrated(sm):
    _write_routing(sm.system_routing_path, textwrap.dedent("""\
        version: 5
        sheets:
          ctrl1:
            node_id: ctrl1
            inputs:
              - id: in1
                topic: /saint/track
                field: left_velocity
                label: Old track input
                position: [10, 20]
            outputs: []
            operators: []
            widgets: []
            wires:
              - id: w1
                source: { kind: input, parts: [in1] }
                sink:   { kind: peripheral, parts: [ctrl1, roboclaw, left_motor] }
    """))
    sm._load_system_routing()
    sheet = sm.state.system_routing.sheets["ctrl1"]
    assert sheet.inputs == []
    assert len(sheet.ws_inputs) == 1
    ws = sheet.ws_inputs[0]
    # Migration reuses the original id so existing wires stay correct.
    assert ws.id == "in1"
    assert ws.label == "Old track input"
    assert ws.position == (10, 20)
    wire = sheet.wires[0]
    assert wire.source.kind == "ws_input"
    assert wire.source.parts == ["in1"]


def test_bidirectional_endpoint_is_left_alone(sm):
    # /saint/head has both state and command types — it's a legitimate
    # ROS-state input and should stay an InputNode.
    _write_routing(sm.system_routing_path, textwrap.dedent("""\
        version: 1
        sheets:
          ctrl1:
            node_id: ctrl1
            inputs:
              - id: in1
                topic: /saint/head
                field: pan
                label: Head pan feedback
                position: [0, 0]
            outputs: []
            operators: []
            widgets: []
            wires: []
    """))
    sm._load_system_routing()
    sheet = sm.state.system_routing.sheets["ctrl1"]
    assert sheet.ws_inputs == []
    assert len(sheet.inputs) == 1
    assert sheet.inputs[0].topic == "/saint/head"
