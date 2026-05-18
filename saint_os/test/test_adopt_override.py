"""Adopt-flow chip override behavior.

When the firmware's chip_family announcement is "unknown" (or some
other value the server doesn't recognize) the operator must still be
able to adopt the node by explicitly picking a chip + board in the
dialog. The operator's choice wins over what the firmware reported.
"""
from __future__ import annotations

import json
import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.webserver.state_manager import StateManager


# StateManager needs a config dir that has boards/ underneath so it can
# resolve board_ids. We point it at the real repo config dir so the
# shipped feather_rp2040_w5500 board is available — but redirect the
# nodes/ subdir to a tmp_path so test adoptions don't write persistent
# node YAML files into the working tree.
REPO_CONFIG_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "config")
)


@pytest.fixture()
def sm(tmp_path):
    inst = StateManager(config_dir=REPO_CONFIG_DIR)
    inst.nodes_config_dir = str(tmp_path / "nodes")
    os.makedirs(inst.nodes_config_dir, exist_ok=True)
    return inst


def _announce_node(sm: StateManager, node_id: str, chip_family: str = ""):
    payload = {
        "node_id": node_id,
        "mac": "02:0:0:0:0:1",
        "ip": "10.0.0.5",
        "hw": "TestHW",
        "fw": "1.0.0",
        "state": "UNADOPTED",
    }
    if chip_family:
        payload["chip_family"] = chip_family
    sm.update_node_from_announcement(json.dumps(payload))


class TestAdoptOverride:
    """Verify chip_family override from the Adopt dialog."""

    def test_operator_can_adopt_when_firmware_reports_unknown(self, sm):
        _announce_node(sm, "rp2040_TEST", chip_family="unknown")
        result = sm.adopt_node(
            "rp2040_TEST", role="cradle_base",
            display_name="Test",
            board_id="feather_rp2040_w5500",
            chip_family="rp2040",
        )
        assert result["success"] is True, result
        node = sm.state.adopted_nodes["rp2040_TEST"]
        # Operator's pick wins — chip_family is now correct.
        assert node.chip_family == "rp2040"
        assert node.board_id == "feather_rp2040_w5500"

    def test_operator_can_adopt_when_firmware_reports_nothing(self, sm):
        _announce_node(sm, "rp2040_TEST")
        assert sm.state.unadopted_nodes["rp2040_TEST"].chip_family == ""
        result = sm.adopt_node(
            "rp2040_TEST", role="cradle_base",
            display_name="Test",
            board_id="feather_rp2040_w5500",
            chip_family="rp2040",
        )
        assert result["success"] is True
        assert sm.state.adopted_nodes["rp2040_TEST"].chip_family == "rp2040"

    def test_board_pick_overrides_firmware_chip_silently(self, sm):
        """If the firmware says 'unknown' and the operator picks a board
        whose chip is 'rp2040', the server syncs node.chip_family to the
        board's chip — no error. Previously this raised a mismatch."""
        _announce_node(sm, "rp2040_TEST", chip_family="unknown")
        # No explicit chip_family param — the board pick should sync it.
        result = sm.adopt_node(
            "rp2040_TEST", role="cradle_base",
            display_name="Test",
            board_id="feather_rp2040_w5500",
        )
        assert result["success"] is True
        assert sm.state.adopted_nodes["rp2040_TEST"].chip_family == "rp2040"
