"""Listing the current sensors a servo can be calibrated against.

Dialing a servo's extents is a mechanical judgement — you watch the
draw climb as it approaches the end of its travel and stop before it
stalls against a hard stop. The sensor is almost never the servo: on
this rig the FAS100 sits on the Cradle Base while the Maestro is on the
Head Node, so the dashboard's old same-node search found nothing and
the indicator stayed blank.

Which channels count as current readings comes from the catalog, not
from matching channel-id spellings in the dashboard — that private copy
had already drifted, missing the Servo 2040's aggregate `current_a`.
"""
from __future__ import annotations

import json
import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.peripheral_model import current_reading_channels
from saint_server.webserver.state_manager import StateManager

REPO_CONFIG_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "config"))


@pytest.fixture
def sm(tmp_path):
    inst = StateManager(config_dir=REPO_CONFIG_DIR)
    inst.nodes_config_dir = str(tmp_path / "nodes")
    os.makedirs(inst.nodes_config_dir, exist_ok=True)
    return inst


def adopt(sm, node_id, name):
    sm.update_node_from_announcement(json.dumps({
        "node_id": node_id, "hw": "TestHW", "fw": "1.0.0",
        "state": "UNADOPTED", "chip_family": "rp2040"}))
    sm.adopt_node(node_id, role="cradle_base", display_name=name,
                  board_id="feather_rp2040_w5500")


class TestCatalogIsTheAuthority:
    def test_it_finds_every_current_sensing_type(self):
        assert current_reading_channels("fas100") == [("amps", "Current (A)")]
        assert current_reading_channels("roboclaw") == [("current", "Motor current")]
        assert current_reading_channels("pathfinder_bms") == [("current", "Current")]

    def test_it_finds_the_servo2040_aggregate(self):
        """The one the dashboard's hardcoded list missed."""
        assert current_reading_channels("pimoroni_servo2040") == \
            [("current_a", "Current (A)")]

    def test_a_servo_driver_senses_nothing(self):
        assert current_reading_channels("maestro") == []
        assert current_reading_channels("neopixel") == []

    def test_an_unknown_type_is_empty_not_an_error(self):
        assert current_reading_channels("no_such_type") == []


class TestListing:
    def test_empty_with_no_nodes(self, sm):
        assert sm.list_current_sources() == []

    def test_a_sensor_is_listed_with_everything_needed_to_subscribe(self, sm):
        adopt(sm, "rp2040_A", "Cradle Base")
        sm.upsert_node_peripheral("rp2040_A", {
            "type": "fas100", "label": "Current Sensor",
            "pins": {"uart_tx": 28, "uart_rx": 29}, "params": {}})
        srcs = sm.list_current_sources()
        assert len(srcs) == 1
        s = srcs[0]
        # The dashboard subscribes to pin_state/<node_id> and matches on
        # (peripheral_id, channel_id) — all three must be present.
        assert s["node_id"] == "rp2040_A"
        assert s["peripheral_id"] == "fas100-1"
        assert s["channel_id"] == "amps"
        assert s["node_name"] == "Cradle Base"
        assert s["channel_label"] == "Current (A)"

    def test_it_spans_nodes(self, sm):
        """THE point: the sensor and the servo are on different nodes."""
        adopt(sm, "rp2040_A", "Cradle Base")
        adopt(sm, "rp2040_B", "Track Drive")
        sm.upsert_node_peripheral("rp2040_A", {
            "type": "fas100", "label": "Current Sensor",
            "pins": {"uart_tx": 28, "uart_rx": 29}, "params": {}})
        sm.upsert_node_peripheral("rp2040_B", {
            "type": "roboclaw", "label": "Track RoboClaw",
            "pins": {"uart_tx": 0, "uart_rx": 1}, "params": {"address": 128}})
        nodes = {s["node_id"] for s in sm.list_current_sources()}
        assert nodes == {"rp2040_A", "rp2040_B"}

    def test_peripherals_without_current_are_not_listed(self, sm):
        adopt(sm, "rp2040_A", "Cradle Base")
        sm.upsert_node_peripheral("rp2040_A", {
            "type": "neopixel", "label": "Strip", "pins": {"data": 20},
            "params": {"pixel_count": 16, "default_color": "#1e293b"}})
        assert sm.list_current_sources() == []

    def test_offline_nodes_are_listed_but_flagged(self, sm):
        """Still selectable — a node that drops out mid-calibration
        shouldn't silently vanish from the picker and reset the choice."""
        adopt(sm, "rp2040_A", "Cradle Base")
        sm.upsert_node_peripheral("rp2040_A", {
            "type": "fas100", "label": "Current Sensor",
            "pins": {"uart_tx": 28, "uart_rx": 29}, "params": {}})
        sm.state.adopted_nodes["rp2040_A"].online = False
        srcs = sm.list_current_sources()
        assert len(srcs) == 1 and srcs[0]["online"] is False

    def test_order_is_stable(self, sm):
        """The picker must not reshuffle under the operator."""
        for nid, name in (("rp2040_C", "Zebra"), ("rp2040_A", "Alpha")):
            adopt(sm, nid, name)
            sm.upsert_node_peripheral(nid, {
                "type": "fas100", "label": "Current Sensor",
                "pins": {"uart_tx": 28, "uart_rx": 29}, "params": {}})
        names = [s["node_name"] for s in sm.list_current_sources()]
        assert names == sorted(names)
        assert sm.list_current_sources() == sm.list_current_sources()
