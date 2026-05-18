"""Shape contract: what `pin_config_capabilities_to_json()` MUST produce.

This isn't a test of the firmware's C code (that needs Pico SDK header
stubbing — separate effort). It's a contract test: the JSON shape the
server expects FROM the firmware, written down so:

  * If the server's ingestion logic ever drifts, this fails.
  * If/when we get host-compilation working for pin_config.c, the C
    test runner can produce a JSON and assert it satisfies this same
    shape contract (jsonschema-style).

The capability JSON ships as the body of std_msgs/String on
/saint/nodes/<node_id>/capabilities. The server's
StateManager.update_node_capabilities() parses it.
"""
from __future__ import annotations

import json
import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.webserver.state_manager import StateManager


# ─── canonical shape ──────────────────────────────────────────────────


REQUIRED_TOP_KEYS = {"node_id", "pins", "reserved_pins", "uart_pairs"}
OPTIONAL_TOP_KEYS = {"builtin_peripherals"}

REQUIRED_PIN_KEYS = {"gpio", "name", "capabilities"}

REQUIRED_UART_KEYS = {"uart", "tx", "rx"}

REQUIRED_BUILTIN_KEYS = {"id", "type"}
OPTIONAL_BUILTIN_KEYS = {"label", "pins", "params"}


def assert_shape(blob: dict) -> None:
    """Raise AssertionError if the blob doesn't match the contract."""
    assert isinstance(blob, dict)
    missing = REQUIRED_TOP_KEYS - blob.keys()
    assert not missing, f"missing required keys: {missing}"

    extra = blob.keys() - (REQUIRED_TOP_KEYS | OPTIONAL_TOP_KEYS)
    assert not extra, f"unexpected top-level keys: {extra}"

    assert isinstance(blob["node_id"], str) and blob["node_id"]

    assert isinstance(blob["pins"], list)
    for pin in blob["pins"]:
        missing = REQUIRED_PIN_KEYS - pin.keys()
        assert not missing, f"pin {pin} missing {missing}"
        assert isinstance(pin["gpio"], int)
        assert isinstance(pin["name"], str)
        assert isinstance(pin["capabilities"], list)
        for cap in pin["capabilities"]:
            assert isinstance(cap, str)

    assert isinstance(blob["reserved_pins"], list)
    for r in blob["reserved_pins"]:
        assert isinstance(r, int)

    assert isinstance(blob["uart_pairs"], list)
    for u in blob["uart_pairs"]:
        missing = REQUIRED_UART_KEYS - u.keys()
        assert not missing, f"uart_pair {u} missing {missing}"
        for k in REQUIRED_UART_KEYS:
            assert isinstance(u[k], int)

    if "builtin_peripherals" in blob:
        assert isinstance(blob["builtin_peripherals"], list)
        for bp in blob["builtin_peripherals"]:
            missing = REQUIRED_BUILTIN_KEYS - bp.keys()
            assert not missing, f"builtin {bp} missing {missing}"
            extra = bp.keys() - (REQUIRED_BUILTIN_KEYS | OPTIONAL_BUILTIN_KEYS)
            assert not extra, f"builtin {bp} has unexpected keys: {extra}"
            assert isinstance(bp["id"], str) and bp["id"]
            assert isinstance(bp["type"], str) and bp["type"]


# Representative samples — these are what we expect the firmware to emit.
# When the firmware-side emit code changes, the corresponding sample
# here should change too. That's the contract.


SAMPLE_RP2040_WITH_NEOPIXEL = {
    "node_id": "rp2040_585783812c33",
    "pins": [
        {"gpio": 5,  "name": "D5",  "capabilities": ["digital_in", "digital_out", "pwm"]},
        {"gpio": 6,  "name": "D6",  "capabilities": ["digital_in", "digital_out", "pwm"]},
        {"gpio": 0,  "name": "TX",  "capabilities": ["digital_in", "digital_out", "pwm", "uart_tx"]},
        {"gpio": 1,  "name": "RX",  "capabilities": ["digital_in", "digital_out", "pwm", "uart_rx"]},
        {"gpio": 2,  "name": "SDA", "capabilities": ["digital_in", "digital_out", "pwm", "i2c_sda"]},
        {"gpio": 3,  "name": "SCL", "capabilities": ["digital_in", "digital_out", "pwm", "i2c_scl"]},
        {"gpio": 26, "name": "A0",  "capabilities": ["digital_in", "digital_out", "pwm", "adc"]},
    ],
    "reserved_pins": [10, 11, 16, 18, 19, 20],
    "uart_pairs": [
        {"uart": 0, "tx": 0,  "rx": 1},
        {"uart": 0, "tx": 12, "rx": 13},
        {"uart": 0, "tx": 28, "rx": 29},
        {"uart": 1, "tx": 24, "rx": 25},
    ],
    "builtin_peripherals": [
        {"id": "onboard_neopixel", "type": "neopixel",
         "label": "Onboard NeoPixel",
         "pins": {"data": 16},
         "params": {}},
    ],
}


SAMPLE_TEENSY_NO_BUILTINS = {
    "node_id": "teensy41_ABCD1234",
    "pins": [
        {"gpio": 13, "name": "D13", "capabilities": ["digital_in", "digital_out"]},
    ],
    "reserved_pins": [],
    "uart_pairs": [{"uart": 1, "tx": 1, "rx": 0}],
    "builtin_peripherals": [],
}


# ─── tests ────────────────────────────────────────────────────────────


class TestContractShape:
    """Each sample is what the firmware emits; assert shape contract."""

    def test_rp2040_with_neopixel(self):
        assert_shape(SAMPLE_RP2040_WITH_NEOPIXEL)

    def test_teensy_no_builtins(self):
        assert_shape(SAMPLE_TEENSY_NO_BUILTINS)

    def test_extra_top_key_caught(self):
        bad = dict(SAMPLE_RP2040_WITH_NEOPIXEL)
        bad["something_new"] = 1
        with pytest.raises(AssertionError):
            assert_shape(bad)

    def test_missing_uart_pairs_caught(self):
        bad = dict(SAMPLE_RP2040_WITH_NEOPIXEL)
        del bad["uart_pairs"]
        with pytest.raises(AssertionError):
            assert_shape(bad)


class TestIngestionAcceptsContract:
    """Server's ingestion must accept every sample that satisfies the contract."""

    @pytest.fixture()
    def sm(self, tmp_path):
        return StateManager(config_dir=str(tmp_path))

    def _seed(self, sm, node_id):
        sm.update_node_from_announcement(json.dumps({
            "node_id": node_id, "mac": "02:0:0:0:0:1", "ip": "10.0.0.5",
            "hw": "Test", "fw": "1.0.0", "state": "UNADOPTED",
        }))

    def test_rp2040_sample_ingested(self, sm):
        self._seed(sm, "rp2040_585783812c33")
        ok = sm.update_node_capabilities(
            "rp2040_585783812c33", json.dumps(SAMPLE_RP2040_WITH_NEOPIXEL))
        assert ok
        caps = sm.get_node_capabilities("rp2040_585783812c33")
        assert {p["gpio"] for p in caps["pins"]} == {0, 1, 2, 3, 5, 6, 26}

    def test_teensy_sample_ingested(self, sm):
        self._seed(sm, "teensy41_ABCD1234")
        ok = sm.update_node_capabilities(
            "teensy41_ABCD1234", json.dumps(SAMPLE_TEENSY_NO_BUILTINS))
        assert ok
