"""End-to-end test of the capability ingestion/retrieval flow.

Simulates what happens when the firmware publishes a capability JSON
in response to a request_capabilities — the server has to parse it
into NodeCapabilities, expose it via get_node_capabilities, and seed
any builtin_peripherals declared in the JSON.

No rclpy/network involved — we just call StateManager directly.
"""
from __future__ import annotations

import json
import os
import sys
import tempfile

import pytest

# Ensure saint_server is importable (lives at saint_os/saint_server/).
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.webserver.state_manager import (
    StateManager,
    HOST_CONTROLLER_NODE_ID,
)


@pytest.fixture()
def sm(tmp_path):
    """Fresh StateManager in a throwaway config dir."""
    return StateManager(config_dir=str(tmp_path))


# ─── helpers ──────────────────────────────────────────────────────────


def _announce(sm: StateManager, node_id: str = "rp2040_TEST01"):
    """Push an announcement so the node exists in unadopted_nodes."""
    sm.update_node_from_announcement(json.dumps({
        "node_id": node_id,
        "mac": "02:00:00:00:00:01",
        "ip": "192.168.10.50",
        "hw": "Adafruit Feather RP2040",
        "fw": "1.2.0-1779075119",
        "state": "UNADOPTED",
    }))


def _adopt(sm: StateManager, node_id: str = "rp2040_TEST01"):
    _announce(sm, node_id)
    return sm.adopt_node(node_id, role="cradle_base", display_name="Test")


def _sample_capabilities(node_id: str = "rp2040_TEST01") -> dict:
    """Shape of the JSON the firmware sends from pin_config_capabilities_to_json."""
    return {
        "node_id": node_id,
        "pins": [
            {"gpio": 5,  "name": "D5",  "capabilities": ["digital_in", "digital_out", "pwm"]},
            {"gpio": 0,  "name": "TX",  "capabilities": ["digital_in", "digital_out", "pwm", "uart_tx"]},
            {"gpio": 1,  "name": "RX",  "capabilities": ["digital_in", "digital_out", "pwm", "uart_rx"]},
            {"gpio": 28, "name": "A2",  "capabilities": ["digital_in", "digital_out", "pwm", "adc"]},
        ],
        "reserved_pins": [10, 11, 16, 18, 19, 20],
        "uart_pairs": [
            {"uart": 0, "tx": 0,  "rx": 1},
            {"uart": 0, "tx": 28, "rx": 29},
        ],
        "builtin_peripherals": [
            {"id": "onboard_neopixel", "type": "neopixel",
             "label": "Onboard NeoPixel",
             "pins": {"data": 16},
             "params": {}},
        ],
    }


# ─── tests ────────────────────────────────────────────────────────────


class TestCapabilityIngestion:
    """The path firmware → server when the firmware publishes capabilities."""

    def test_capabilities_stored_on_unadopted_node(self, sm):
        _announce(sm)
        ok = sm.update_node_capabilities(
            "rp2040_TEST01", json.dumps(_sample_capabilities())
        )
        assert ok is True

        caps = sm.get_node_capabilities("rp2040_TEST01")
        assert caps is not None
        gpios = sorted(p["gpio"] for p in caps["pins"])
        assert gpios == [0, 1, 5, 28]
        assert caps["reserved_pins"] == [10, 11, 16, 18, 19, 20]
        assert {"uart": 0, "tx": 0, "rx": 1} in caps["uart_pairs"]

    def test_capabilities_stored_on_adopted_node(self, sm):
        _adopt(sm)
        sm.update_node_capabilities(
            "rp2040_TEST01", json.dumps(_sample_capabilities())
        )
        caps = sm.get_node_capabilities("rp2040_TEST01")
        assert caps is not None
        assert len(caps["pins"]) == 4

    def test_builtin_peripherals_seeded(self, sm):
        """Firmware-declared built-ins should auto-populate peripheral_config."""
        _adopt(sm)
        sm.update_node_capabilities(
            "rp2040_TEST01", json.dumps(_sample_capabilities())
        )
        peripherals = sm.get_node_peripherals("rp2040_TEST01")
        assert peripherals is not None
        ids = [p["id"] for p in peripherals["peripherals"]]
        assert "onboard_neopixel" in ids
        neopixel = next(p for p in peripherals["peripherals"]
                        if p["id"] == "onboard_neopixel")
        assert neopixel["type"] == "neopixel"
        assert neopixel["builtin"] is True
        assert neopixel["pins"] == {"data": 16}

    def test_builtin_peripherals_idempotent(self, sm):
        """Receiving the same caps twice mustn't duplicate the built-in entry."""
        _adopt(sm)
        sm.update_node_capabilities("rp2040_TEST01", json.dumps(_sample_capabilities()))
        sm.update_node_capabilities("rp2040_TEST01", json.dumps(_sample_capabilities()))
        peripherals = sm.get_node_peripherals("rp2040_TEST01")
        np_count = sum(1 for p in peripherals["peripherals"]
                       if p["id"] == "onboard_neopixel")
        assert np_count == 1

    def test_invalid_json_returns_false(self, sm):
        _adopt(sm)
        assert sm.update_node_capabilities("rp2040_TEST01", "not-json") is False
        assert sm.update_node_capabilities("rp2040_TEST01", "") is False

    def test_unknown_node_returns_false(self, sm):
        ok = sm.update_node_capabilities(
            "rp2040_GHOST", json.dumps(_sample_capabilities("rp2040_GHOST"))
        )
        assert ok is False


class TestCapabilitiesReadback:
    """The path UI → server when peripherals.js calls get_node_capabilities."""

    def test_returns_none_before_firmware_responds(self, sm):
        _adopt(sm)
        assert sm.get_node_capabilities("rp2040_TEST01") is None

    def test_returns_dict_after_firmware_responds(self, sm):
        _adopt(sm)
        sm.update_node_capabilities("rp2040_TEST01", json.dumps(_sample_capabilities()))
        caps = sm.get_node_capabilities("rp2040_TEST01")
        assert isinstance(caps, dict)
        assert "pins" in caps and "uart_pairs" in caps

    def test_host_controller_capabilities_present_but_empty(self, sm):
        """Host controller has caps (so the UI doesn't say 'no caps yet')
        but the pin list is intentionally empty (no operator pins yet)."""
        caps = sm.get_node_capabilities(HOST_CONTROLLER_NODE_ID)
        assert caps is not None
        assert caps["pins"] == []
        assert caps["reserved_pins"] == []
