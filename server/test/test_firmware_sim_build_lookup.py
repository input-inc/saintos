"""Sim-firmware build discovery for force_firmware_update.

`get_firmware_build_info('simulation')` must locate the RP2040 sim build
so the OTA path (and the Renode e2e's Phase 5) can push it. The e2e
container mounts the firmware at `firmware/rp2040/install/simulation`
(the canonical `make install_sim` location) — NOT `build_sim`, which is
a dev-only output dir. Before the fix the lookup only checked
`build_sim`, so in the container it reported "No simulation firmware
build found".

These pin: install/simulation is discovered as a simulation build, the
dev build_sim path still works, and absence is reported correctly.
"""
from __future__ import annotations

import os

import pytest

from saint_server.webserver.state_manager import StateManager

REPO_CONFIG_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "config"))


@pytest.fixture()
def sm(tmp_path):
    inst = StateManager(config_dir=REPO_CONFIG_DIR)
    # Point the firmware-source resolver at an isolated tree so the test
    # doesn't pick up the repo's real build dirs.
    fw_root = tmp_path / "firmware" / "rp2040"
    fw_root.mkdir(parents=True)
    inst._get_firmware_dir = lambda: str(fw_root)   # type: ignore[assignment]
    return inst, fw_root


def _make_elf(d):
    d.mkdir(parents=True, exist_ok=True)
    (d / "saint_node.elf").write_bytes(b"\x7fELF" + b"\x00" * 64)


def test_install_simulation_is_discovered(sm):
    """The e2e scenario: only install/simulation exists (no build_sim)."""
    inst, fw_root = sm
    _make_elf(fw_root / "install" / "simulation")
    info = inst.get_firmware_build_info("simulation")
    assert info["available"] is True
    assert info["build_type"] == "simulation"
    assert info["elf_path"].endswith("install/simulation/saint_node.elf")


def test_dev_build_sim_still_works(sm):
    """The dev scenario must keep working after the install/simulation add."""
    inst, fw_root = sm
    _make_elf(fw_root / "build_sim")
    info = inst.get_firmware_build_info("simulation")
    assert info["available"] is True
    assert info["build_type"] == "simulation"


def test_absent_sim_build_reported(sm):
    """No sim artifact anywhere → available=False (the honest 'not found')."""
    inst, fw_root = sm
    # only a hardware build present
    _make_elf(fw_root / "build")
    info = inst.get_firmware_build_info("simulation")
    assert info["available"] is False


def test_env_override_resolves_firmware_dir(tmp_path, monkeypatch):
    """SAINT_FIRMWARE_RP2040_DIR lets the server find the firmware tree
    when it runs from a colcon install with no source above it (the
    Renode e2e container case). With the override + a mounted
    install/simulation ELF, the sim build is discovered end-to-end —
    no _get_firmware_dir monkeypatch."""
    fw_root = tmp_path / "firmware" / "rp2040"
    _make_elf(fw_root / "install" / "simulation")
    monkeypatch.setenv("SAINT_FIRMWARE_RP2040_DIR", str(fw_root))

    inst = StateManager(config_dir=REPO_CONFIG_DIR)
    assert inst._get_firmware_dir() == str(fw_root)
    info = inst.get_firmware_build_info("simulation")
    assert info["available"] is True
    assert info["build_type"] == "simulation"
