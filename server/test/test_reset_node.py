"""Coverage for StateManager.reset_node.

The Unadopt button on the node-detail page calls reset_node with
factory_reset=False. Before the fix that call returned "success" but
did nothing — the node stayed in adopted_nodes, the operator's card
never moved to the Unadopted section, and the node was effectively
inaccessible. Both code paths (soft = Unadopt, hard = Factory reset)
are exercised here so a future regression flips a test, not the UI.
"""
from __future__ import annotations

import json
import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.webserver.state_manager import StateManager


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
    """Announce + adopt a node so it lands in adopted_nodes."""
    sm.update_node_from_announcement(json.dumps({
        "node_id": node_id,
        "mac": "02:0:0:0:0:1",
        "ip": "10.0.0.5",
        "hw": "TestHW",
        "fw": "1.0.0",
        "state": "UNADOPTED",
        "chip_family": "rp2040",
    }))
    result = sm.adopt_node(
        node_id, role="cradle_base",
        display_name="Test Display",
        board_id="feather_rp2040_w5500",
    )
    assert result["success"] is True, result


class TestResetNodeSoft:
    """Unadopt path: factory_reset=False."""

    def test_moves_node_to_unadopted(self, sm):
        _adopt(sm)
        assert "rp2040_TEST" in sm.state.adopted_nodes
        assert "rp2040_TEST" not in sm.state.unadopted_nodes

        result = sm.reset_node("rp2040_TEST", factory_reset=False)

        assert result["success"] is True
        assert "rp2040_TEST" not in sm.state.adopted_nodes
        assert "rp2040_TEST" in sm.state.unadopted_nodes

    def test_preserves_role_and_display_name(self, sm):
        """Soft unadopt keeps the operator's labelling for easy re-adoption."""
        _adopt(sm)
        result = sm.reset_node("rp2040_TEST", factory_reset=False)
        assert result["success"] is True
        node = sm.state.unadopted_nodes["rp2040_TEST"]
        assert node.role == "cradle_base"
        assert node.display_name == "Test Display"

    def test_keeps_on_disk_config_yaml(self, sm):
        """Soft unadopt is server-bookkeeping only — the operator's
        peripheral config stays on disk so re-adoption picks back up."""
        _adopt(sm)
        config_path = os.path.join(sm.nodes_config_dir, "rp2040_TEST.yaml")
        # adopt_node calls _save_node_config so the yaml exists.
        assert os.path.exists(config_path)
        sm.reset_node("rp2040_TEST", factory_reset=False)
        assert os.path.exists(config_path)

    def test_returns_error_for_unknown_node(self, sm):
        result = sm.reset_node("nonexistent", factory_reset=False)
        assert result["success"] is False
        assert "not found" in result["message"].lower()


class TestResetNodeHard:
    """Factory-reset path: factory_reset=True."""

    def test_moves_node_to_unadopted(self, sm):
        _adopt(sm)
        result = sm.reset_node("rp2040_TEST", factory_reset=True)
        assert result["success"] is True
        assert "rp2040_TEST" not in sm.state.adopted_nodes
        assert "rp2040_TEST" in sm.state.unadopted_nodes

    def test_clears_role_and_display_name(self, sm):
        """Hard reset is a clean slate — labels go away too."""
        _adopt(sm)
        sm.reset_node("rp2040_TEST", factory_reset=True)
        node = sm.state.unadopted_nodes["rp2040_TEST"]
        assert node.role == ""
        assert node.display_name == ""

    def test_deletes_on_disk_config_yaml(self, sm):
        """Hard reset deletes the peripheral config yaml so re-adoption
        doesn't auto-restore the same config to a wiped firmware."""
        _adopt(sm)
        config_path = os.path.join(sm.nodes_config_dir, "rp2040_TEST.yaml")
        assert os.path.exists(config_path)
        sm.reset_node("rp2040_TEST", factory_reset=True)
        assert not os.path.exists(config_path)

    def test_returns_error_for_unknown_node(self, sm):
        result = sm.reset_node("nonexistent", factory_reset=True)
        assert result["success"] is False
