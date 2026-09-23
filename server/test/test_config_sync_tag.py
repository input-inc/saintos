"""Config sync tag — the server/node reconciliation loop.

Until now the server pushed the entire config on every sync and had no
way to know what the node was actually holding. /announce carried
node_id, state, uptime and a save timestamp, but nothing identifying the
config itself, so these were all invisible:

  * a node rebooting onto a stale flash blob
  * a node coming back empty after a reflash or a flash-version bump
    (both happened on 2026-09-23)
  * a config push that never landed

The node now echoes the tag it was given with its config, and the server
compares it against the tag its CURRENT config would carry.

The comparison is OBSERVATION, not a trigger. Config reaches a node
during an explicit Sync and at no other time — an operator has to be
able to edit a channel, look at it, decide against it and revert,
without the server having shipped the intermediate state to the
hardware on the next announcement. What the tag buys is a truthful
`sync_status`: it used to be set to "synced" the moment we published,
which said only that a message left the server.

The tag is a CRC32 of the payload, not an issued token, so the server
stays stateless — it recomputes what it expects rather than remembering
it, which means a server restart causes no resync storm and there is no
second source of truth to drift.
"""
from __future__ import annotations

import json
import os
import sys
import types
from unittest.mock import MagicMock

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

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
        "node_id": NODE, "hw": "TestHW", "fw": "1.0.0",
        "state": "UNADOPTED", "chip_family": "rp2040",
    }))
    inst.adopt_node(NODE, role="cradle_base", display_name="Test",
                    board_id="feather_rp2040_w5500")
    inst.upsert_node_peripheral(NODE, {
        "type": "neopixel", "label": "Strip", "pins": {"data": 20},
        "params": {"pixel_count": 16, "default_color": "#1e293b"},
    })
    return inst


def observe(sm, node_tag):
    return sm.observe_node_config_tag(NODE, node_tag)


def status(sm):
    return sm.state.adopted_nodes[NODE].peripheral_config.sync_status


class TestObservationNeverPushes:
    """THE property the operator depends on. Nothing in this path may
    send config — otherwise a half-finished edit goes live within a
    second of being typed, and reverting it is impossible."""

    def test_the_observer_has_no_way_to_push(self, sm):
        import inspect
        from saint_server.server_node import SaintServerNode
        src = inspect.getsource(SaintServerNode._observe_node_config_tag)
        assert "send_config_to_node" not in src
        assert "plan_config_push" not in src

    def test_a_mismatch_marks_pending_rather_than_pushing(self, sm):
        sm.state.adopted_nodes[NODE].peripheral_config.sync_status = "synced"
        assert observe(sm, 999999) is True
        assert status(sm) == "pending"

    def test_a_match_marks_synced(self, sm):
        sm.state.adopted_nodes[NODE].peripheral_config.sync_status = "pending"
        assert observe(sm, sm.expected_config_tag(NODE)) is True
        assert status(sm) == "synced"

    def test_synced_means_the_node_confirmed_it(self, sm):
        """Not 'we published a message'. A push that never landed leaves
        the node reporting its old tag, which now reads as pending."""
        sm.state.adopted_nodes[NODE].peripheral_config.sync_status = "synced"
        observe(sm, 12345)
        assert status(sm) == "pending"


class TestObservationIsCheap:
    def test_no_write_when_the_status_is_unchanged(self, sm):
        """Runs on every announcement, ~1 Hz per node — it must only
        touch the YAML on a real transition."""
        tag = sm.expected_config_tag(NODE)
        assert observe(sm, tag) is True      # pending -> synced
        for _ in range(10):
            assert observe(sm, tag) is False


class TestObservationDeclinesToGuess:
    def test_absent_tag_leaves_the_status_alone(self, sm):
        """Firmware predating the field says nothing; claiming it is out
        of sync would be inventing information."""
        sm.state.adopted_nodes[NODE].peripheral_config.sync_status = "synced"
        assert observe(sm, None) is False
        assert status(sm) == "synced"

    def test_unknown_node_is_ignored(self, sm):
        assert sm.observe_node_config_tag("rp2040_GHOST", 1) is False

    def test_no_expected_tag_leaves_the_status_alone(self, sm):
        """An oversized config we refused to build gives nothing to
        compare against."""
        for pin in range(0, 30):
            if pin != 20:
                sm.upsert_node_peripheral(NODE, {
                    "type": "neopixel",
                    "label": f"Strip on {pin} with a descriptive name",
                    "pins": {"data": pin},
                    "params": {"pixel_count": 16, "default_color": "#1e293b"},
                })
        assert sm.expected_config_tag(NODE) is None
        assert observe(sm, 12345) is False


class TestManualSyncStillWorks:
    def test_sync_still_produces_a_payload(self, sm):
        """Observation replaced the automatic push, not the deliberate
        one."""
        plan = sm.plan_config_push(NODE, sm.expected_config_tag(NODE))
        assert json.loads(plan)["action"] == "configure"


class TestTagDescribesConfigNotEditCount:
    """Edit a channel, change your mind, revert — the node is running
    exactly what the dashboard holds again, so it must read as synced.

    The tag used to include `version`, the counter bumped on every save.
    Identical config after a revert therefore hashed differently and the
    node sat at "pending" forever, clearable only by a full push the
    operator did not need. Observed live on 2026-09-23.
    """

    def _edit_ch(self, sm, idx, value):
        node = sm.state.adopted_nodes[NODE]
        per = node.peripheral_config.get("maestro-1")
        params = dict(per.params)
        chans = [dict(c) for c in params["channels"]]
        chans[idx]["idle_disengage_ms"] = value
        params["channels"] = chans
        sm.upsert_node_peripheral(NODE, {
            "id": "maestro-1", "type": "maestro", "label": per.label,
            "pins": per.pins, "params": params})

    @pytest.fixture
    def sm_maestro(self, sm):
        sm.upsert_node_peripheral(NODE, {
            "type": "maestro", "label": "Maestro", "pins": {},
            "params": {"transport": "usb_vendor", "channel_count": 24}})
        return sm

    def test_edit_then_revert_restores_the_tag(self, sm_maestro):
        sm = sm_maestro
        original = sm.expected_config_tag(NODE)
        self._edit_ch(sm, 23, 1000)
        assert sm.expected_config_tag(NODE) != original
        self._edit_ch(sm, 23, 0)
        assert sm.expected_config_tag(NODE) == original, (
            "reverting an edit must return to the original tag, or the "
            "node reads as out-of-sync while running the right config")

    def test_a_bare_resave_does_not_change_the_tag(self, sm_maestro):
        sm = sm_maestro
        before = sm.expected_config_tag(NODE)
        per = sm.state.adopted_nodes[NODE].peripheral_config.get("maestro-1")
        sm.upsert_node_peripheral(NODE, {
            "id": "maestro-1", "type": "maestro", "label": per.label,
            "pins": per.pins, "params": dict(per.params)})
        assert sm.expected_config_tag(NODE) == before
