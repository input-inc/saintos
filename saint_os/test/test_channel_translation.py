"""Server-side virtual-GPIO → channel translation.

Firmware still publishes peripheral channels on virtual GPIO slots
(transitional). The state manager translates them into the routing
system's (peripheral_id, channel_id) form. Widget dashboard + Live
Readings tab both consume the channel-addressed view; the legacy
virtual-GPIO map in widgets.js was retired in favor of this single
server-side translation.
"""
from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.webserver.state_manager import (
    _firmware_channel_id,
    NodeRuntimeState,
)


class TestFirmwareChannelIdMap:

    def test_fas100_indexes(self):
        assert _firmware_channel_id("fas100_sensor", 232) == "amps"
        assert _firmware_channel_id("fas100_sensor", 233) == "volts"
        assert _firmware_channel_id("fas100_sensor", 234) == "temp1"
        assert _firmware_channel_id("fas100_sensor", 235) == "temp2"

    def test_bms_indexes_match_catalog(self):
        # Catalog channel ids: pack_voltage / current / soc / temp_1 / temp_2.
        # Firmware indexes 0,1,2,4,5 map to those; 3 (remain_cap) and
        # 6,7 (cycles, protection) are intentionally unmapped.
        assert _firmware_channel_id("pathfinder_bms_sensor", 276) == "pack_voltage"
        assert _firmware_channel_id("pathfinder_bms_sensor", 277) == "current"
        assert _firmware_channel_id("pathfinder_bms_sensor", 278) == "soc"
        assert _firmware_channel_id("pathfinder_bms_sensor", 279) is None
        assert _firmware_channel_id("pathfinder_bms_sensor", 280) == "temp_1"
        assert _firmware_channel_id("pathfinder_bms_sensor", 281) == "temp_2"
        assert _firmware_channel_id("pathfinder_bms_sensor", 282) is None  # cycles
        # Cells start at firmware index 8 / GPIO 284 — also unmapped.
        assert _firmware_channel_id("pathfinder_bms_sensor", 284) is None

    def test_syren_motor_channel(self):
        # Each of the 8 firmware slots represents a separate SyRen unit;
        # all map to "motor" channel. peripheral_id (logical_name)
        # disambiguates which one.
        for ch in range(8):
            assert _firmware_channel_id("syren_motor", 224 + ch) == "motor"

    def test_unknown_mode_returns_none(self):
        assert _firmware_channel_id("digital_in", 5) is None
        assert _firmware_channel_id("pwm", 5) is None
        assert _firmware_channel_id("unknown_thing", 9999) is None

    def test_out_of_range_returns_none(self):
        # GPIO 999 is well past any peripheral base.
        assert _firmware_channel_id("fas100_sensor", 999) is None


class TestRuntimeIngestion:

    def test_pin_state_with_peripheral_id_populates_channels(self):
        rs = NodeRuntimeState(node_id="rp2040_test")
        rs.update_from_firmware([
            {"gpio": 276, "mode": "pathfinder_bms_sensor",
             "value": 24.3, "name": "pathfinder_bms-1"},
            {"gpio": 277, "mode": "pathfinder_bms_sensor",
             "value": -1.2, "name": "pathfinder_bms-1"},
            {"gpio": 280, "mode": "pathfinder_bms_sensor",
             "value": 26.5, "name": "pathfinder_bms-1"},
        ])
        # Channel values are addressed by (peripheral_id, channel_id).
        ch = rs.channels
        assert ch[("pathfinder_bms-1", "pack_voltage")].value == 24.3
        assert ch[("pathfinder_bms-1", "current")].value == -1.2
        assert ch[("pathfinder_bms-1", "temp_1")].value == 26.5

    def test_physical_gpio_doesnt_create_a_channel(self):
        # A PWM pin on a physical GPIO (e.g. 5) has no peripheral
        # type — only per-pin runtime state, no channel.
        rs = NodeRuntimeState(node_id="rp2040_test")
        rs.update_from_firmware([
            {"gpio": 5, "mode": "pwm", "value": 0.5, "name": "headlight"},
        ])
        assert rs.channels == {}
        assert rs.pins[5].actual_value == 0.5

    def test_unmapped_bms_channel_is_silent(self):
        # Firmware can report cell voltages on GPIO 284..299; those
        # have no catalog channel, so they live in pins[] but not
        # channels[].
        rs = NodeRuntimeState(node_id="rp2040_test")
        rs.update_from_firmware([
            {"gpio": 284, "mode": "pathfinder_bms_sensor",
             "value": 3.62, "name": "bms-1"},
        ])
        assert rs.channels == {}
        assert rs.pins[284].actual_value == 3.62

    def test_to_dict_exposes_channels(self):
        rs = NodeRuntimeState(node_id="rp2040_test")
        rs.update_from_firmware([
            {"gpio": 232, "mode": "fas100_sensor", "value": 2.4, "name": "fas100-1"},
        ])
        d = rs.to_dict()
        channels = [c for c in d["channels"]
                    if c["peripheral_id"] == "fas100-1" and c["channel_id"] == "amps"]
        assert len(channels) == 1
        assert channels[0]["value"] == 2.4
