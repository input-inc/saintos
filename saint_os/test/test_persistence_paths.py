"""Persistence-path resolution in StateManager.

Runtime state (adopted nodes, system routing) must live in a directory
that *survives* an install — install.sh does `rm -rf` on the install
tree on every update. Shipped data (board YAMLs) lives in the install
tree and gets refreshed each update. These tests pin the resolution
rules so the bug that wiped adoptions on every install can't return.
"""
from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.webserver.state_manager import StateManager


class TestRuntimeConfigDirResolution:

    def test_explicit_config_dir_wins(self, tmp_path):
        sm = StateManager(config_dir=str(tmp_path))
        assert sm.config_dir == str(tmp_path)
        assert sm.nodes_config_dir == os.path.join(str(tmp_path), "nodes")

    def test_env_var_override(self, tmp_path, monkeypatch):
        monkeypatch.setenv("SAINT_RUNTIME_CONFIG_DIR", str(tmp_path))
        sm = StateManager()  # no config_dir arg
        assert sm.config_dir == str(tmp_path)

    def test_explicit_arg_beats_env_var(self, tmp_path, monkeypatch):
        # Explicit constructor arg should win over the env var. Tests
        # that pass config_dir= shouldn't have their behavior changed
        # by an operator's environment.
        other = tmp_path / "from-env"
        other.mkdir()
        monkeypatch.setenv("SAINT_RUNTIME_CONFIG_DIR", str(other))
        sm = StateManager(config_dir=str(tmp_path))
        assert sm.config_dir == str(tmp_path)

    def test_runtime_dir_separate_from_install_tree(self, tmp_path):
        """The whole point of the split: nodes_config_dir must not be
        inside <install>/share/saint_os/. That path gets nuked on
        every update."""
        sm = StateManager(config_dir=str(tmp_path))
        bad_substrings = ["install/share/saint_os", "/share/saint_os/config"]
        for needle in bad_substrings:
            assert needle not in sm.nodes_config_dir
            assert needle not in sm.system_routing_path


class TestPersistenceRoundTrip:
    """Adopt → reload-from-disk → adoption survives."""

    def test_adopted_node_survives_restart(self, tmp_path):
        # First StateManager: adopt + save.
        repo_config = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "..", "config"))
        sm1 = StateManager(config_dir=str(tmp_path))
        # Pretend we know about a board so adopt won't reject the board_id.
        # Reuse the real shipped feather board catalog.
        sm1.boards_config_dir = os.path.join(repo_config, "boards")
        from saint_server.board_config import BoardConfigManager
        sm1.board_config = BoardConfigManager(sm1.boards_config_dir)

        import json
        sm1.update_node_from_announcement(json.dumps({
            "node_id": "rp2040_PERSIST", "mac": "02:0:0:0:0:1",
            "ip": "10.0.0.5", "hw": "TestHW", "fw": "1.0.0",
            "chip_family": "rp2040", "state": "UNADOPTED",
        }))
        result = sm1.adopt_node(
            "rp2040_PERSIST", role="cradle_base",
            display_name="Persistent Test",
            board_id="feather_rp2040_w5500",
        )
        assert result["success"], result
        # File must actually exist on disk.
        node_file = os.path.join(str(tmp_path), "nodes", "rp2040_PERSIST.yaml")
        assert os.path.isfile(node_file)

        # Second StateManager (simulated server restart) reads the
        # same config_dir; the previously-adopted node should be back.
        sm2 = StateManager(config_dir=str(tmp_path))
        sm2.boards_config_dir = os.path.join(repo_config, "boards")
        sm2.board_config = BoardConfigManager(sm2.boards_config_dir)
        sm2.load_all_node_configs()  # idempotent — also called in __init__
        assert "rp2040_PERSIST" in sm2.state.adopted_nodes
        restored = sm2.state.adopted_nodes["rp2040_PERSIST"]
        assert restored.role == "cradle_base"
        assert restored.display_name == "Persistent Test"
        assert restored.board_id == "feather_rp2040_w5500"
