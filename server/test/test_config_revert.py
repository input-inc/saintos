"""Reverting unsynced peripheral edits.

Config reaches a node only on an explicit Sync, so anything edited but
not synced exists purely on the server — it can be thrown away without
touching hardware. This is the operator's escape hatch, and the reason
the no-auto-push rule is safe to rely on.

The snapshot is taken when the node CONFIRMS a config (its reported tag
matches ours), not when we publish one. Snapshotting at publish time
would record an optimistic state: a push that never landed would leave
a "confirmed" snapshot the node never received, and Revert would
restore fiction.
"""
from __future__ import annotations

import json
import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

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
        "state": "UNADOPTED", "chip_family": "rp2040"}))
    inst.adopt_node(NODE, role="cradle_base", display_name="Test",
                    board_id="feather_rp2040_w5500")
    inst.upsert_node_peripheral(NODE, {
        "type": "neopixel", "label": "Original", "pins": {"data": 20},
        "params": {"pixel_count": 16, "default_color": "#111111"}})
    return inst


def confirm(sm):
    """Simulate the node announcing the tag we expect."""
    sm.observe_node_config_tag(NODE, sm.expected_config_tag(NODE))


def first_id(sm):
    """The added peripheral's id. The board's built-in NeoPixel already
    occupies the type, so the generator starts at neopixel-2."""
    return [p.id for p in
            sm.state.adopted_nodes[NODE].peripheral_config.peripherals
            if not p.builtin][0]


def label_of(sm):
    return sm.state.adopted_nodes[NODE].peripheral_config.get(
        first_id(sm)).label


class TestSnapshot:
    def test_no_snapshot_before_the_node_confirms(self, sm):
        assert sm.has_synced_snapshot(NODE) is False

    def test_confirmation_creates_one(self, sm):
        confirm(sm)
        assert sm.has_synced_snapshot(NODE) is True

    def test_a_mismatch_does_not_create_one(self, sm):
        """A node reporting a different tag has NOT confirmed our
        config, so there is nothing to call a synced state."""
        sm.observe_node_config_tag(NODE, 999999)
        assert sm.has_synced_snapshot(NODE) is False


class TestRevert:
    def test_it_restores_the_confirmed_config(self, sm):
        confirm(sm)
        sm.upsert_node_peripheral(NODE, {
            "id": first_id(sm), "type": "neopixel", "label": "Edited",
            "pins": {"data": 20},
            "params": {"pixel_count": 4, "default_color": "#999999"}})
        assert label_of(sm) == "Edited"

        assert sm.revert_node_peripherals(NODE)["success"] is True
        assert label_of(sm) == "Original"
        params = sm.state.adopted_nodes[NODE].peripheral_config.get(
            first_id(sm)).params
        assert params["pixel_count"] == 16

    def test_revert_restores_the_tag_too(self, sm):
        """The point of reverting: the node is running this config, so
        afterwards we must agree with it again."""
        confirm(sm)
        before = sm.expected_config_tag(NODE)
        sm.upsert_node_peripheral(NODE, {
            "id": first_id(sm), "type": "neopixel", "label": "Edited",
            "pins": {"data": 20},
            "params": {"pixel_count": 4, "default_color": "#999999"}})
        assert sm.expected_config_tag(NODE) != before
        sm.revert_node_peripherals(NODE)
        assert sm.expected_config_tag(NODE) == before

    def test_status_returns_to_synced(self, sm):
        confirm(sm)
        sm.upsert_node_peripheral(NODE, {
            "id": first_id(sm), "type": "neopixel", "label": "Edited",
            "pins": {"data": 20},
            "params": {"pixel_count": 4, "default_color": "#999999"}})
        assert sm.state.adopted_nodes[NODE].peripheral_config.sync_status == "pending"
        sm.revert_node_peripherals(NODE)
        assert sm.state.adopted_nodes[NODE].peripheral_config.sync_status == "synced"

    def test_it_reverts_an_added_peripheral(self, sm):
        confirm(sm)
        before = {p.id for p in
                  sm.state.adopted_nodes[NODE].peripheral_config.peripherals}
        sm.upsert_node_peripheral(NODE, {
            "type": "neopixel", "label": "Extra", "pins": {"data": 21},
            "params": {"pixel_count": 8, "default_color": "#222222"}})
        after_add = {p.id for p in
                     sm.state.adopted_nodes[NODE].peripheral_config.peripherals}
        assert after_add > before, "precondition: a peripheral was added"
        sm.revert_node_peripherals(NODE)
        assert {p.id for p in
                sm.state.adopted_nodes[NODE].peripheral_config.peripherals} == before

    def test_it_refuses_when_nothing_was_ever_confirmed(self, sm):
        """Without a confirmed state there is nothing truthful to
        restore — better to say so than to invent a baseline."""
        r = sm.revert_node_peripherals(NODE)
        assert r["success"] is False and "not confirmed" in r["message"]

    def test_unknown_node_is_refused(self, sm):
        assert sm.revert_node_peripherals("rp2040_GHOST")["success"] is False

    def test_revert_leaves_our_record_of_the_node_untouched(self, sm):
        """It discards server-side state only — nothing reaches the
        node, which is what makes it safe to offer as an undo. Our
        record of what the node holds must therefore be unchanged."""
        confirm(sm)
        sm.record_config_push(NODE, sm.get_firmware_config_json(NODE))
        pushed = sm.last_pushed_config(NODE)
        sm.upsert_node_peripheral(NODE, {
            "id": first_id(sm), "type": "neopixel", "label": "Edited",
            "pins": {"data": 20},
            "params": {"pixel_count": 4, "default_color": "#999999"}})
        sm.revert_node_peripherals(NODE)
        assert sm.last_pushed_config(NODE) == pushed


class TestSnapshotIsSelfHealing:
    """A node already sitting at "synced" when this shipped never
    transitions, so a transition-only snapshot would leave Revert
    permanently unavailable. Observed on the robot: the Head Node was
    already synced and reported `snapshot available: False`."""

    def test_a_node_already_synced_still_gets_a_snapshot(self, sm):
        # Arrive at "synced" WITHOUT going through the observer.
        sm.state.adopted_nodes[NODE].peripheral_config.sync_status = "synced"
        assert sm.has_synced_snapshot(NODE) is False
        sm.observe_node_config_tag(NODE, sm.expected_config_tag(NODE))
        assert sm.has_synced_snapshot(NODE) is True

    def test_it_does_not_overwrite_an_existing_snapshot(self, sm):
        """Repeated confirmations must not keep re-copying — and must
        never quietly re-baseline onto later edits."""
        confirm(sm)
        original = open(sm._synced_snapshot_path(NODE)).read()
        sm.observe_node_config_tag(NODE, sm.expected_config_tag(NODE))
        assert open(sm._synced_snapshot_path(NODE)).read() == original
