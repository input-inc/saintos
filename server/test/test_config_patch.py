"""Per-channel config deltas (step 2 of docs/CONFIG_SYNC.md).

Nudging one Maestro channel used to cost a full ~1.5 KB config push
against a ~2048-byte wire ceiling. With the sync tag proving what the
node holds, the change can go out as a patch expressed against that
exact base.

The interesting tests here are the REFUSALS. A delta applied to a base
we merely assume is precisely the divergence the tag exists to prevent,
so anything we cannot prove falls back to a full push. Falling back
costs bytes; guessing costs a servo in the wrong place.
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
NODE = "teensy41_TEST"


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
        "type": "maestro", "label": "Maestro", "pins": {},
        "params": {"transport": "usb_vendor", "channel_count": 24},
    })
    return inst


def synced(sm):
    """Push the current config and pretend the node applied it."""
    full = sm.get_firmware_config_json(NODE)
    sm.record_config_push(NODE, full)
    return json.loads(full)["tag"]


def edit_channel(sm, idx, **fields):
    node = sm.state.adopted_nodes[NODE]
    per = node.peripheral_config.get("maestro-1")
    params = dict(per.params)
    chans = [dict(c) for c in params["channels"]]
    chans[idx].update(fields)
    params["channels"] = chans
    sm.upsert_node_peripheral(NODE, {
        "id": "maestro-1", "type": "maestro", "label": per.label,
        "pins": per.pins, "params": params,
    })


class TestPatchIsUsed:
    def test_one_channel_edit_becomes_a_patch(self, sm):
        tag = synced(sm)
        edit_channel(sm, 12, home_us=1300)
        plan = sm.plan_config_push(NODE, tag)
        body = json.loads(plan)
        assert body["action"] == "patch_config"
        assert body["from"] == tag
        assert body["to"] == sm.expected_config_tag(NODE)
        assert body["peripheral"] == "maestro-1"
        assert "12" in body["channels"]

    def test_a_patch_is_far_smaller_than_the_full_config(self, sm):
        tag = synced(sm)
        edit_channel(sm, 12, home_us=1300)
        patch = sm.plan_config_push(NODE, tag)
        full = sm.get_firmware_config_json(NODE)
        assert len(patch) < len(full) / 2

    def test_a_patch_fits_one_xrce_frame(self, sm):
        """The whole point: never fragmented, so the reassembly buffer
        that the full push fights is never involved."""
        tag = synced(sm)
        for ch in (3, 7, 12):
            edit_channel(sm, ch, home_us=1300 + ch)
        patch = sm.plan_config_push(NODE, tag)
        assert json.loads(patch)["action"] == "patch_config"
        assert len(patch) <= 512

    def test_the_patch_carries_the_channel_whole(self, sm):
        """Whole-channel, not field-level: a field RESET to default
        vanishes from the slimmed wire form, and a field-level diff
        would leave the node on the old value."""
        tag = synced(sm)
        edit_channel(sm, 5, home_us=1700, min_pulse_us=900)
        body = json.loads(sm.plan_config_push(NODE, tag))
        ch = body["channels"]["5"]
        assert ch["home_us"] == 1700 and ch["min_pulse_us"] == 900


class TestFallsBackToFullPush:
    def test_when_the_node_is_not_where_we_think(self, sm):
        """The safety property. The node reports a tag that isn't the
        base we'd diff against, so a patch would be applied to unknown
        state."""
        synced(sm)
        edit_channel(sm, 12, home_us=1300)
        plan = sm.plan_config_push(NODE, 999999)
        assert json.loads(plan)["action"] == "configure"

    def test_when_the_node_reports_no_tag(self, sm):
        synced(sm)
        edit_channel(sm, 12, home_us=1300)
        assert json.loads(sm.plan_config_push(NODE, None))["action"] == "configure"

    def test_when_we_have_never_pushed_to_this_node(self, sm):
        edit_channel(sm, 12, home_us=1300)
        tag = sm.expected_config_tag(NODE)
        assert json.loads(sm.plan_config_push(NODE, tag))["action"] == "configure"

    def test_when_a_peripheral_level_param_changed(self, sm):
        tag = synced(sm)
        node = sm.state.adopted_nodes[NODE]
        per = node.peripheral_config.get("maestro-1")
        params = dict(per.params)
        params["min_pulse_us"] = 900          # peripheral-level, not a channel
        sm.upsert_node_peripheral(NODE, {
            "id": "maestro-1", "type": "maestro", "label": per.label,
            "pins": per.pins, "params": params,
        })
        assert json.loads(sm.plan_config_push(NODE, tag))["action"] == "configure"

    def test_when_a_peripheral_was_added(self, sm):
        tag = synced(sm)
        sm.upsert_node_peripheral(NODE, {
            "type": "neopixel", "label": "Strip", "pins": {"data": 20},
            "params": {"pixel_count": 16, "default_color": "#1e293b"},
        })
        assert json.loads(sm.plan_config_push(NODE, tag))["action"] == "configure"

    def test_when_a_non_maestro_peripheral_changed(self, sm):
        sm.upsert_node_peripheral(NODE, {
            "type": "neopixel", "label": "Strip", "pins": {"data": 20},
            "params": {"pixel_count": 16, "default_color": "#1e293b"},
        })
        tag = synced(sm)
        sm.upsert_node_peripheral(NODE, {
            "id": "neopixel-1", "type": "neopixel", "label": "Strip",
            "pins": {"data": 20},
            "params": {"pixel_count": 8, "default_color": "#1e293b"},
        })
        assert json.loads(sm.plan_config_push(NODE, tag))["action"] == "configure"

    def test_when_the_patch_would_be_too_large(self, sm):
        """Past a few channels a patch loses its reason to exist — it
        would fragment like the full push, so send the full push."""
        tag = synced(sm)
        for ch in range(24):
            edit_channel(sm, ch, home_us=1200 + ch, min_pulse_us=800 + ch,
                         max_pulse_us=2200 - ch, neutral_us=1500 + ch)
        assert json.loads(sm.plan_config_push(NODE, tag))["action"] == "configure"

    def test_nothing_changed_still_yields_the_full_config(self, sm):
        """Sync with no pending edit is an explicit operator request to
        re-assert — answer it, don't send an empty patch."""
        tag = synced(sm)
        assert json.loads(sm.plan_config_push(NODE, tag))["action"] == "configure"


class TestPushRecording:
    def test_only_a_full_push_becomes_the_new_base(self, sm):
        """A patch is expressed relative to the base; recording one as
        the base would make the next delta diff against a fragment."""
        tag = synced(sm)
        edit_channel(sm, 12, home_us=1300)
        patch = sm.plan_config_push(NODE, tag)
        assert json.loads(patch)["action"] == "patch_config"
        # Base unchanged, so a second edit still diffs from the same place.
        edit_channel(sm, 13, home_us=1400)
        again = json.loads(sm.plan_config_push(NODE, tag))
        assert again["action"] == "patch_config"
        assert set(again["channels"]) == {"12", "13"}
