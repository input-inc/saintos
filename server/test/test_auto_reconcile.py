"""Coverage for the firmware-server state-mismatch reconciler.

`SaintServerNode._maybe_reconcile_adopted_unadopted` watches incoming
announcements and re-pushes peripheral config when a node we have in
adopted_nodes claims to be in UNADOPTED state — the typical aftermath
of a firmware OTA that wiped or invalidated the saved flash config.
The reconciler is rate-limited per node via
NodeInfo.last_reconcile_push_at to avoid hammering a firmware that
keeps failing apply at the 1 Hz announcement cadence.

These tests bind the method to a SimpleNamespace standing in for a
SaintServerNode — constructing a real one would require rclpy
initialisation, a full async loop, and file-system log dirs, none of
which we need just to verify the reconcile decision logic.
"""
from __future__ import annotations

import json
import os
import sys
import time
import types

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.webserver.state_manager import StateManager
from saint_server.server_node import SaintServerNode


REPO_CONFIG_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "config")
)


@pytest.fixture()
def sm(tmp_path):
    inst = StateManager(config_dir=REPO_CONFIG_DIR)
    inst.nodes_config_dir = str(tmp_path / "nodes")
    os.makedirs(inst.nodes_config_dir, exist_ok=True)
    return inst


def _adopt(sm: StateManager, node_id: str = "rp2040_TEST") -> None:
    sm.update_node_from_announcement(json.dumps({
        "node_id": node_id,
        "mac": "02:0:0:0:0:1",
        "ip": "10.0.0.5",
        "hw": "TestHW",
        "fw": "1.0.0",
        "state": "UNADOPTED",
        "chip_family": "rp2040",
    }))
    sm.adopt_node(
        node_id, role="cradle_base",
        display_name="Test",
        board_id="feather_rp2040_w5500",
    )


def _make_stub(sm: StateManager) -> types.SimpleNamespace:
    """Minimal self-stand-in for the bound method under test."""
    sent = []
    return types.SimpleNamespace(
        state_manager=sm,
        send_config_to_node=lambda node_id, config_json: sent.append((node_id, config_json)),
        _RECONCILE_COOLDOWN_S=SaintServerNode._RECONCILE_COOLDOWN_S,
        sent=sent,
    )


def _reconcile(stub, node_id, state):
    SaintServerNode._maybe_reconcile_adopted_unadopted(stub, node_id, state)


class TestReconcileDecision:
    """When to push, when to skip."""

    def test_active_announce_does_not_push(self, sm):
        """Steady state — no mismatch, no push."""
        _adopt(sm)
        stub = _make_stub(sm)
        _reconcile(stub, "rp2040_TEST", "ACTIVE")
        assert stub.sent == []

    def test_unknown_node_does_not_push(self, sm):
        """Announcement from a node the server doesn't track is dropped."""
        stub = _make_stub(sm)
        _reconcile(stub, "rp2040_GHOST", "UNADOPTED")
        assert stub.sent == []

    def test_unadopted_for_unadopted_node_does_not_push(self, sm):
        """If the server already considers the node unadopted, nothing
        to reconcile — it's truly waiting for the operator."""
        sm.update_node_from_announcement(json.dumps({
            "node_id": "rp2040_NEW",
            "hw": "TestHW",
            "fw": "1.0.0",
            "state": "UNADOPTED",
        }))
        assert "rp2040_NEW" in sm.state.unadopted_nodes
        stub = _make_stub(sm)
        _reconcile(stub, "rp2040_NEW", "UNADOPTED")
        assert stub.sent == []

    def test_adopted_announcing_unadopted_pushes_config(self, sm):
        """The main repair path — adopted server-side, UNADOPTED
        firmware-side, push the canonical config."""
        _adopt(sm)
        stub = _make_stub(sm)
        _reconcile(stub, "rp2040_TEST", "UNADOPTED")
        assert len(stub.sent) == 1
        target_id, payload = stub.sent[0]
        assert target_id == "rp2040_TEST"
        # Payload must look like a configure message.
        body = json.loads(payload)
        assert body["action"] == "configure"
        assert "peripherals" in body

    def test_empty_config_falls_back_to_empty_peripherals_array(self, sm):
        """Adopted but no non-builtin peripherals → still push an empty
        configure so the firmware transitions ACTIVE (per Teensy
        main.cpp:176-186 and RP2040 apply_peripherals_json accepting
        an empty array)."""
        _adopt(sm)
        # peripheral_config is empty/builtin-only — get_firmware_config_json
        # returns a stub payload. Confirm our fallback isn't triggered when
        # there's a valid config, and IS triggered when there isn't.
        node = sm.state.adopted_nodes["rp2040_TEST"]
        node.peripheral_config = None  # force the "no config" branch
        stub = _make_stub(sm)
        _reconcile(stub, "rp2040_TEST", "UNADOPTED")
        assert len(stub.sent) == 1
        _, payload = stub.sent[0]
        body = json.loads(payload)
        assert body["action"] == "configure"
        assert body["peripherals"] == []


class TestReconcileCooldown:
    """Rate-limiting so firmware-apply failures don't get hammered."""

    def test_second_push_within_cooldown_is_suppressed(self, sm):
        _adopt(sm)
        stub = _make_stub(sm)
        _reconcile(stub, "rp2040_TEST", "UNADOPTED")
        _reconcile(stub, "rp2040_TEST", "UNADOPTED")
        # Two adjacent UNADOPTED announcements (1 Hz cadence) should
        # produce ONE push.
        assert len(stub.sent) == 1

    def test_push_after_cooldown_elapses(self, sm):
        _adopt(sm)
        stub = _make_stub(sm)
        _reconcile(stub, "rp2040_TEST", "UNADOPTED")
        assert len(stub.sent) == 1

        # Backdate the per-node timestamp so the next announcement is
        # past the cooldown without sleeping in the test.
        node = sm.state.adopted_nodes["rp2040_TEST"]
        node.last_reconcile_push_at = time.time() - SaintServerNode._RECONCILE_COOLDOWN_S - 1.0

        _reconcile(stub, "rp2040_TEST", "UNADOPTED")
        assert len(stub.sent) == 2

    def test_cooldown_is_per_node(self, sm):
        """Two nodes both stuck — both should get one push each, not
        share a single global cooldown."""
        _adopt(sm, node_id="rp2040_A")
        _adopt(sm, node_id="rp2040_B")
        stub = _make_stub(sm)
        _reconcile(stub, "rp2040_A", "UNADOPTED")
        _reconcile(stub, "rp2040_B", "UNADOPTED")
        assert len(stub.sent) == 2
        assert {target for target, _ in stub.sent} == {"rp2040_A", "rp2040_B"}
