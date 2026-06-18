"""Server-side ingest of peripheral-first channel-addressed state.

Phase 1 of docs/PERIPHERAL_FIRST_MIGRATION.md added a parallel
ingest path: firmware drivers that have migrated emit a `channels[]`
array alongside the legacy GPIO-keyed `pins[]` array, and the
server consumes it through `NodeRuntimeState.update_channels_from_firmware`
+ the `channels_data` kwarg on `StateManager.update_pin_actual`.

These tests pin the new path's shape so a future driver migration
can't regress it silently, and confirm the legacy GPIO path still
works alongside it (the half-migrated-driver case).
"""
from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.webserver.state_manager import NodeRuntimeState


@pytest.fixture
def state():
    return NodeRuntimeState(node_id="test-node")


class TestUpdateChannelsFromFirmware:
    """Direct ingest path — no slab math, no _FIRMWARE_CHANNEL_MAP."""

    def test_single_record_lands_in_channels(self, state):
        updates = state.update_channels_from_firmware([
            {"peripheral_id": "maestro-1", "channel_id": "connected", "value": 1.0},
        ])
        assert updates == [("maestro-1", "connected", 1.0)]
        # And the runtime state actually holds it.
        d = state.to_dict()
        chs = {(c["peripheral_id"], c["channel_id"]): c["value"]
               for c in d["channels"]}
        assert chs[("maestro-1", "connected")] == 1.0

    def test_multiple_records_one_call(self, state):
        updates = state.update_channels_from_firmware([
            {"peripheral_id": "maestro-1", "channel_id": "connected", "value": 1.0},
            {"peripheral_id": "maestro-1", "channel_id": "errors",    "value": 0.0},
            {"peripheral_id": "maestro-1", "channel_id": "ch0",       "value": 1500.0},
        ])
        assert len(updates) == 3
        assert ("maestro-1", "errors", 0.0) in updates
        assert ("maestro-1", "ch0", 1500.0) in updates

    def test_value_coerced_to_float(self, state):
        # JSON-decoded ints / strings shouldn't crash; the channel
        # store is float-typed end-to-end.
        updates = state.update_channels_from_firmware([
            {"peripheral_id": "p1", "channel_id": "c1", "value": 42},      # int
            {"peripheral_id": "p1", "channel_id": "c2", "value": "3.14"},  # str
        ])
        vals = {ch: v for (_, ch, v) in updates}
        assert vals["c1"] == 42.0
        assert isinstance(vals["c1"], float)
        assert vals["c2"] == pytest.approx(3.14)

    def test_short_field_names_also_accepted(self, state):
        # Firmware emits `peripheral_id` / `channel_id`. Some early
        # consumers used `peripheral` / `channel` — the ingest
        # accepts both so a partial-migration test fixture can use
        # whichever is shorter to type.
        updates = state.update_channels_from_firmware([
            {"peripheral": "p1", "channel": "c1", "value": 1.0},
        ])
        assert updates == [("p1", "c1", 1.0)]

    def test_malformed_entries_skipped(self, state):
        # Missing fields, wrong types — must not raise.
        updates = state.update_channels_from_firmware([
            {"peripheral_id": "p1"},                                    # no channel_id
            {"channel_id": "c1", "value": 1.0},                         # no peripheral_id
            {"peripheral_id": "p1", "channel_id": "c1"},                # no value
            {"peripheral_id": "p1", "channel_id": "c1", "value": None}, # null value
            {"peripheral_id": "p1", "channel_id": "c1", "value": "x"},  # non-numeric
            {"peripheral_id": "p1", "channel_id": "c1", "value": 7.0},  # the only good one
        ])
        assert updates == [("p1", "c1", 7.0)]

    def test_empty_or_none_inputs(self, state):
        assert state.update_channels_from_firmware([]) == []
        assert state.update_channels_from_firmware(None) == []

    def test_last_writer_wins_within_one_batch(self, state):
        # Two records for the same (peripheral, channel) → the
        # second value is what's stored.
        state.update_channels_from_firmware([
            {"peripheral_id": "p1", "channel_id": "c1", "value": 1.0},
            {"peripheral_id": "p1", "channel_id": "c1", "value": 2.0},
        ])
        d = state.to_dict()
        chs = {(c["peripheral_id"], c["channel_id"]): c["value"]
               for c in d["channels"]}
        assert chs[("p1", "c1")] == 2.0

    def test_updates_last_feedback(self, state):
        before = state.last_feedback
        state.update_channels_from_firmware([
            {"peripheral_id": "p1", "channel_id": "c1", "value": 1.0},
        ])
        assert state.last_feedback > before


class TestDualIngestPath:
    """Half-migrated drivers: some channels via pins[], others via
    channels[]. The two paths must coexist without stomping each other."""

    def test_legacy_pins_path_still_works(self, state):
        # Use FAS100 (base 232) since it's in the existing channel map.
        updates = state.update_from_firmware([
            {"gpio": 232, "mode": "fas100_sensor", "value": 12.3, "name": "fas100-1"},
        ])
        assert updates == [("fas100-1", "amps", 12.3)]

    def test_legacy_and_new_combine_in_runtime_state(self, state):
        state.update_from_firmware([
            {"gpio": 232, "mode": "fas100_sensor", "value": 12.3, "name": "fas100-1"},
        ])
        state.update_channels_from_firmware([
            {"peripheral_id": "maestro-1", "channel_id": "connected", "value": 1.0},
        ])
        d = state.to_dict()
        chs = {(c["peripheral_id"], c["channel_id"]): c["value"]
               for c in d["channels"]}
        assert chs[("fas100-1", "amps")] == 12.3
        assert chs[("maestro-1", "connected")] == 1.0

    def test_same_channel_via_both_paths_last_writer_wins(self, state):
        # Half-migrated driver edge case: imagine the firmware
        # emits one channel via pins[] AND via channels[] (a bug,
        # but we don't want crashes). The order in update_pin_actual
        # is pins[] first, then channels[], so channels[] wins.
        state.update_from_firmware([
            {"gpio": 232, "mode": "fas100_sensor", "value": 1.0, "name": "fas100-1"},
        ])
        state.update_channels_from_firmware([
            {"peripheral_id": "fas100-1", "channel_id": "amps", "value": 2.0},
        ])
        d = state.to_dict()
        chs = {(c["peripheral_id"], c["channel_id"]): c["value"]
               for c in d["channels"]}
        assert chs[("fas100-1", "amps")] == 2.0


class TestStateManagerIntegration:
    """The StateManager.update_pin_actual entry point — exercises
    both code paths together as the ROS-bridge ingest does."""

    def _adopt(self, sm, node_id):
        # get_or_create_runtime_state returns None unless the node is
        # in adopted_nodes; the real adoption flow happens elsewhere,
        # so seed the minimum directly.
        from saint_server.webserver.state_manager import NodeInfo
        sm.state.adopted_nodes[node_id] = NodeInfo(node_id=node_id)

    def test_update_pin_actual_with_channels_data(self):
        # StateManager pulls in rclpy at the package level; conftest
        # already stubs it, so the import here is safe.
        from saint_server.webserver.state_manager import StateManager
        sm = StateManager()
        self._adopt(sm, "rig-1")

        ok = sm.update_pin_actual(
            node_id="rig-1",
            pins_data=[],
            channels_data=[
                {"peripheral_id": "maestro-1", "channel_id": "connected", "value": 1.0},
                {"peripheral_id": "maestro-1", "channel_id": "errors",    "value": 0.0},
            ],
        )
        assert ok is True
        rt = sm.get_runtime_state("rig-1")
        chs = {(c["peripheral_id"], c["channel_id"]): c["value"]
               for c in rt["channels"]}
        assert chs[("maestro-1", "connected")] == 1.0
        assert chs[("maestro-1", "errors")] == 0.0

    def test_update_pin_actual_channels_data_optional(self):
        # Calling without the new kwarg must keep the legacy
        # signature working (callers that haven't been updated).
        from saint_server.webserver.state_manager import StateManager
        sm = StateManager()
        self._adopt(sm, "rig-1")
        ok = sm.update_pin_actual(node_id="rig-1", pins_data=[])
        assert ok is True
