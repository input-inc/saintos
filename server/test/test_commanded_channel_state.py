"""The State tab's sliders need a value to sit at.

`State.vue` renders each writable channel's slider from
`pin_state/<node_id>`, which the server builds from NodeRuntimeState.
That state is populated from firmware `/state` messages — but most
actuator channels never report anything back. A Maestro's /state
carries exactly three entries (`connected`, `error_flags`, `moving`);
its 24 servo channels are write-only on the wire, because reading a
position means polling the Maestro over USB, the transfer that used to
wedge the Teensy.

So a pose could drive a servo to its commanded position while the
slider for that channel sat at zero with no value at all, and the
operator's next drag started from the wrong place instead of from
where the hardware actually was.

For a write-only channel the last commanded value is the best truth
available, and these tests pin that it reaches the payload the UI
reads — while a genuine firmware reading still wins.
"""
from __future__ import annotations

import json
import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import types
from unittest.mock import MagicMock

from saint_server.channel_arbiter import BOARD, SLIDER, ChannelArbiter
from saint_server.server_node import SaintServerNode
from saint_server.webserver.state_manager import StateManager

REPO_CONFIG_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "config"))

NODE = "rp2040_TEST"


@pytest.fixture
def sm(tmp_path):
    inst = StateManager(config_dir=REPO_CONFIG_DIR)
    inst.nodes_config_dir = str(tmp_path / "nodes")
    os.makedirs(inst.nodes_config_dir, exist_ok=True)
    inst.update_node_from_announcement(json.dumps({
        "node_id": NODE,
        "mac": "02:0:0:0:0:9",
        "ip": "10.0.0.9",
        "hw": "TestHW",
        "fw": "1.0.0",
        "state": "UNADOPTED",
        "chip_family": "rp2040",
    }))
    inst.adopt_node(NODE, role="cradle_base", display_name="Test",
                    board_id="feather_rp2040_w5500")
    return inst


def channels_of(sm):
    """Exactly what the websocket ships as pin_state/<node_id>."""
    state = sm.get_runtime_state(NODE)
    return {(c["peripheral_id"], c["channel_id"]): c["value"]
            for c in (state or {}).get("channels", [])}


def test_commanded_value_reaches_the_ui_payload(sm):
    sm.record_commanded_channel(NODE, "maestro-1", "ch12", 0.42)
    assert channels_of(sm)[("maestro-1", "ch12")] == pytest.approx(0.42)


def test_without_it_the_slider_has_no_value(sm):
    """The regression this fixes: nothing else populates a write-only
    channel, so the slider binds to undefined."""
    assert ("maestro-1", "ch12") not in channels_of(sm)


def test_latest_command_wins(sm):
    sm.record_commanded_channel(NODE, "maestro-1", "ch12", 0.42)
    sm.record_commanded_channel(NODE, "maestro-1", "ch12", -0.15)
    assert channels_of(sm)[("maestro-1", "ch12")] == pytest.approx(-0.15)


def test_a_pose_touching_many_channels_records_each(sm):
    for i, v in enumerate([0.1, 0.2, 0.3, 0.4]):
        sm.record_commanded_channel(NODE, "maestro-1", f"ch{i}", v)
    chs = channels_of(sm)
    assert [chs[("maestro-1", f"ch{i}")] for i in range(4)] == \
        pytest.approx([0.1, 0.2, 0.3, 0.4])


def test_a_real_firmware_reading_overrides_a_commanded_value(sm):
    """Commanded value is a fallback for channels nothing reports. Where
    the firmware does report, its reading is the truth."""
    sm.record_commanded_channel(NODE, "maestro-1", "connected", 0.0)
    rt = sm.get_or_create_runtime_state(NODE)
    rt.update_channels_from_firmware([
        {"peripheral_id": "maestro-1", "channel_id": "connected", "value": 1.0},
    ])
    assert channels_of(sm)[("maestro-1", "connected")] == pytest.approx(1.0)


def test_channels_are_kept_per_peripheral(sm):
    sm.record_commanded_channel(NODE, "maestro-1", "ch0", 0.5)
    sm.record_commanded_channel(NODE, "servo2040-1", "ch0", -0.5)
    chs = channels_of(sm)
    assert chs[("maestro-1", "ch0")] == pytest.approx(0.5)
    assert chs[("servo2040-1", "ch0")] == pytest.approx(-0.5)


def test_unknown_node_is_a_no_op(sm):
    sm.record_commanded_channel("nope", "maestro-1", "ch0", 0.5)  # must not raise


def test_non_numeric_value_is_ignored(sm):
    sm.record_commanded_channel(NODE, "maestro-1", "ch0", "banana")
    assert ("maestro-1", "ch0") not in channels_of(sm)


# ── the wiring: send_channel_command must feed the State tab ─────────

def make_node_stub(sm):
    """Minimal stand-in exposing only what send_channel_command touches.

    Drives the REAL SaintServerNode.send_channel_command against a real
    StateManager and a real ChannelArbiter, so the call site is covered
    rather than just the recorder it calls. Everything the unit tests
    above verify is reachable only if this link exists.
    """
    published = []

    def ensure_pub(node_id):
        pub = MagicMock()
        pub.publish.side_effect = lambda msg: published.append((node_id, msg.data))
        return pub

    return types.SimpleNamespace(
        state_manager=sm,
        channel_arbiter=ChannelArbiter(
            idle_disengage_lookup=sm.channel_idle_disengage_ms),
        get_logger=lambda: MagicMock(),
        _ensure_node_control_publisher=ensure_pub,
        _ensure_node_state_subscriber=lambda nid: None,
        _mark_node_tx=lambda nid: None,
        _host_peripheral_manager=None,
        published=published,
    )


def send(stub, value, owner=SLIDER, raw_us=None, channel="ch12"):
    return SaintServerNode.send_channel_command(
        stub, NODE, "maestro-1", channel, value,
        peripheral_type="maestro", raw_us=raw_us, owner=owner)


def test_a_slider_write_lands_in_the_state_payload(sm):
    stub = make_node_stub(sm)
    assert send(stub, 0.42) is True
    assert len(stub.published) == 1, "precondition: it actually published"
    assert channels_of(sm)[("maestro-1", "ch12")] == pytest.approx(0.42)


def test_a_pose_write_lands_too(sm):
    """The operator-facing ask: a pose drives the channel through the
    sheet, and its value shows up on that channel's State slider."""
    stub = make_node_stub(sm)
    send(stub, -0.30, owner=BOARD)
    assert channels_of(sm)[("maestro-1", "ch12")] == pytest.approx(-0.30)


def test_a_suppressed_write_does_not_move_the_slider(sm):
    """Arbitration said the firmware already holds this, so nothing went
    on the wire — the displayed value must not claim otherwise."""
    stub = make_node_stub(sm)
    send(stub, 0.42)
    sm.record_commanded_channel(NODE, "maestro-1", "ch12", 999.0)  # sentinel
    assert send(stub, 0.42) is False
    assert channels_of(sm)[("maestro-1", "ch12")] == pytest.approx(999.0)


def test_a_raw_us_jog_does_not_write_a_normalized_value(sm):
    """`value` is ignored firmware-side when `us` is present, so
    recording it would put a number on the slider that does not
    describe where the servo is."""
    stub = make_node_stub(sm)
    assert send(stub, 0.0, raw_us=2200) is True
    assert ("maestro-1", "ch12") not in channels_of(sm)
