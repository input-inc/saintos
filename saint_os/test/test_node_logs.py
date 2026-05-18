"""Per-node log buffer tests.

When _log_activity gets a node_id, the entry must (a) land in that
node's NodeInfo.log_entries ring buffer and (b) fire the node-log
callback so the WS handler can broadcast on node_log/<id>.
"""
from __future__ import annotations

import json
import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.webserver.state_manager import StateManager


def _seed_unadopted(sm: StateManager, node_id: str = "rp2040_TEST"):
    sm.update_node_from_announcement(json.dumps({
        "node_id": node_id,
        "mac": "02:0:0:0:0:1",
        "ip": "10.0.0.5",
        "hw": "TestHW",
        "fw": "1.0.0",
        "state": "UNADOPTED",
    }))


class TestNodeLogBuffer:

    def test_log_node_event_appends_to_buffer(self, tmp_path):
        sm = StateManager(config_dir=str(tmp_path))
        _seed_unadopted(sm)
        sm.log_node_event("rp2040_TEST", "hello", "info")
        entries = sm.get_node_logs("rp2040_TEST")
        assert any(e["text"] == "hello" for e in entries)

    def test_unknown_node_returns_empty(self, tmp_path):
        sm = StateManager(config_dir=str(tmp_path))
        assert sm.get_node_logs("nope") == []

    def test_callback_fires_with_node_id_and_entry(self, tmp_path):
        sm = StateManager(config_dir=str(tmp_path))
        _seed_unadopted(sm)
        captured = []
        sm.set_node_log_callback(lambda nid, entry: captured.append((nid, entry)))
        sm.log_node_event("rp2040_TEST", "fired", "warn")
        assert len(captured) >= 1
        nid, entry = captured[-1]
        assert nid == "rp2040_TEST"
        assert entry["text"] == "fired"
        assert entry["level"] == "warn"
        assert "time" in entry

    def test_buffer_is_capped(self, tmp_path):
        sm = StateManager(config_dir=str(tmp_path))
        _seed_unadopted(sm)
        cap = sm.MAX_NODE_LOG_ENTRIES
        for i in range(cap + 50):
            sm.log_node_event("rp2040_TEST", f"line {i}", "info")
        entries = sm.get_node_logs("rp2040_TEST")
        assert len(entries) == cap
        # Newest entries retained, oldest dropped
        assert entries[-1]["text"] == f"line {cap + 49}"
        assert entries[0]["text"] != "line 0"

    def test_clear_node_logs_empties_buffer(self, tmp_path):
        sm = StateManager(config_dir=str(tmp_path))
        _seed_unadopted(sm)
        sm.log_node_event("rp2040_TEST", "one", "info")
        sm.log_node_event("rp2040_TEST", "two", "info")
        assert sm.get_node_logs("rp2040_TEST")
        assert sm.clear_node_logs("rp2040_TEST") is True
        assert sm.get_node_logs("rp2040_TEST") == []

    def test_state_transition_announced_logs_to_node(self, tmp_path):
        sm = StateManager(config_dir=str(tmp_path))
        _seed_unadopted(sm)
        sm.update_node_from_announcement(json.dumps({
            "node_id": "rp2040_TEST",
            "mac": "02:0:0:0:0:1", "ip": "10.0.0.5", "hw": "TestHW",
            "fw": "1.0.0", "state": "ACTIVE",
        }))
        texts = [e["text"] for e in sm.get_node_logs("rp2040_TEST")]
        assert any("UNADOPTED → ACTIVE" in t for t in texts), texts

    def test_firmware_version_change_logs_to_node(self, tmp_path):
        sm = StateManager(config_dir=str(tmp_path))
        _seed_unadopted(sm)
        sm.update_node_from_announcement(json.dumps({
            "node_id": "rp2040_TEST",
            "mac": "02:0:0:0:0:1", "ip": "10.0.0.5", "hw": "TestHW",
            "fw": "1.2.0", "state": "UNADOPTED",
        }))
        texts = [e["text"] for e in sm.get_node_logs("rp2040_TEST")]
        assert any("1.0.0 → 1.2.0" in t for t in texts), texts

    def test_peripheral_connect_transition_logged_once(self, tmp_path):
        sm = StateManager(config_dir=str(tmp_path))
        _seed_unadopted(sm)
        # First announcement establishes baseline (no log).
        sm.update_node_from_announcement(json.dumps({
            "node_id": "rp2040_TEST",
            "mac": "02:0:0:0:0:1", "ip": "10.0.0.5", "hw": "TestHW",
            "fw": "1.0.0", "state": "UNADOPTED",
            "peripherals": {"fas100": False},
        }))
        # Second flips fas100 to connected.
        sm.update_node_from_announcement(json.dumps({
            "node_id": "rp2040_TEST",
            "mac": "02:0:0:0:0:1", "ip": "10.0.0.5", "hw": "TestHW",
            "fw": "1.0.0", "state": "UNADOPTED",
            "peripherals": {"fas100": True},
        }))
        # Third keeps it connected (no new log).
        sm.update_node_from_announcement(json.dumps({
            "node_id": "rp2040_TEST",
            "mac": "02:0:0:0:0:1", "ip": "10.0.0.5", "hw": "TestHW",
            "fw": "1.0.0", "state": "UNADOPTED",
            "peripherals": {"fas100": True},
        }))
        texts = [e["text"] for e in sm.get_node_logs("rp2040_TEST")]
        connects = [t for t in texts if "fas100" in t and "connected" in t]
        assert len(connects) == 1, texts
