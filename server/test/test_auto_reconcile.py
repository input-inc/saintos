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


# ── oversized config must never become an empty one ──────────────────

class TestOversizedConfigIsNotPushed:
    """A config the node cannot receive is refused at the source.

    On 2026-09-23 the Head Node's Maestro config reached 2150 bytes —
    past the ~2048-byte XRCE-DDS reassembly cap. The guard logged a
    warning and published anyway: the node WDOG-reset mid-apply, came
    back announcing UNADOPTED, and this reconcile path re-pushed the
    same oversized payload, resetting it again.

    Two properties are pinned here. The build refuses, and — the
    dangerous one — refusal must NOT be mistaken for "this node has no
    peripherals", because the empty-configure fallback below would then
    adopt the node with nothing on it and erase the operator's config
    from the only place it was still intact.
    """

    def _oversize(self, sm, node_id="rp2040_TEST"):
        """Push the node's config past the cap with enough peripherals
        that no amount of per-field slimming saves it."""
        added = 0
        for pin in range(0, 30):
            if pin == 16:      # taken by the built-in status NeoPixel
                continue
            r = sm.upsert_node_peripheral(node_id, {
                "type": "neopixel",
                "label": f"Strip on {pin} with a descriptive name",
                "pins": {"data": pin},
                "params": {"pixel_count": 16, "default_color": "#1e293b"},
            })
            if r.get("success"):
                added += 1
        assert added >= 15, f"only added {added} peripherals"

    def test_build_refuses_and_records_why(self, sm):
        _adopt(sm)
        self._oversize(sm)
        assert sm.get_firmware_config_json("rp2040_TEST") is None
        why = sm.last_config_push_error("rp2040_TEST")
        assert why and "too large" in why

    def test_reconcile_does_not_push_an_empty_config_over_it(self, sm):
        _adopt(sm)
        self._oversize(sm)
        stub = _make_stub(sm)
        _reconcile(stub, "rp2040_TEST", "UNADOPTED")
        assert stub.sent == [], (
            "refusing to build the config must not be read as 'no "
            "peripherals' — pushing an empty configure here wipes the node")

    def test_the_error_clears_once_the_config_fits_again(self, sm):
        _adopt(sm)
        self._oversize(sm)
        assert sm.get_firmware_config_json("rp2040_TEST") is None
        node = sm.state.adopted_nodes["rp2040_TEST"]
        for per in [p for p in node.peripheral_config.peripherals
                    if not p.builtin]:
            sm.remove_node_peripheral("rp2040_TEST", per.id)
        assert sm.get_firmware_config_json("rp2040_TEST") is not None
        assert sm.last_config_push_error("rp2040_TEST") is None

    def test_a_node_with_no_peripherals_still_gets_the_empty_configure(self, sm):
        """The other meaning of None must keep working, or a freshly
        adopted node never leaves UNADOPTED."""
        _adopt(sm)
        stub = _make_stub(sm)
        _reconcile(stub, "rp2040_TEST", "UNADOPTED")
        assert len(stub.sent) == 1
        assert json.loads(stub.sent[0][1])["peripherals"] == []


class TestRecoveryDoesNotPromoteUnsyncedEdits:
    """The UNADOPTED path restores a node that lost its config. It must
    put back what the node was RUNNING, not whatever the dashboard
    currently holds — config goes live on an explicit Sync and nowhere
    else, so an operator can edit, look, and revert.
    """

    def test_it_restores_the_last_synced_config(self, sm):
        _adopt(sm)
        sm.upsert_node_peripheral(sm_node_id(), {
            "type": "neopixel", "label": "Original", "pins": {"data": 20},
            "params": {"pixel_count": 16, "default_color": "#111111"},
        })
        pushed = sm.get_firmware_config_json(sm_node_id())
        sm.record_config_push(sm_node_id(), pushed)

        # Operator stages an edit but has NOT hit Sync.
        sm.upsert_node_peripheral(sm_node_id(), {
            "id": "neopixel-1", "type": "neopixel", "label": "Edited",
            "pins": {"data": 20},
            "params": {"pixel_count": 4, "default_color": "#999999"},
        })
        assert sm.get_firmware_config_json(sm_node_id()) != pushed

        stub = _make_stub(sm)
        _reconcile(stub, sm_node_id(), "UNADOPTED")
        assert len(stub.sent) == 1
        assert stub.sent[0][1] == pushed, (
            "recovery pushed the staged edit instead of what the node "
            "was actually running")

    def test_it_falls_back_when_there_is_no_record(self, sm):
        """Server restarted since the last push. A blank node left dead
        is worse than restoring the current config."""
        _adopt(sm)
        stub = _make_stub(sm)
        _reconcile(stub, sm_node_id(), "UNADOPTED")
        assert len(stub.sent) == 1


def sm_node_id():
    return "rp2040_TEST"
