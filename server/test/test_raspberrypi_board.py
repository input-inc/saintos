"""Tests for the raspberrypi board YAMLs + the audio_player peripheral type.

Pins down the wire-shape contract that the raspberrypi Pi-side firmware
relies on:

  * `chip_family: raspberrypi` is registered and resolves a default board
    (`raspberrypi`) — required for adoption to auto-assign the board.
  * `audio_player` lives in the catalog with the channels and params
    the Pi driver expects (channel name → sub-channel index mapping
    has to stay in lockstep with PiAudioPlayerDriver.SUB_CHANNEL_NAMES;
    that test lives next to the driver, see firmware/raspberrypi/tests).
  * The raspberrypi board's `builtin_peripherals` list includes the
    `onboard_audio` entry of type `audio_player` so adoption seeds it
    automatically.
  * Built-ins refuse to be removed from a NodePeripheralConfig (the
    behavior the `builtin=True` flag is supposed to enforce).
"""
from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.board_config import BoardConfigManager, derive_capabilities
from saint_server.peripheral_model import (
    DEFAULT_CATALOG,
    NodePeripheralConfig,
    PeripheralInstance,
)


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
BOARDS_ROOT = os.path.join(REPO_ROOT, "server", "config", "boards")


@pytest.fixture(scope="module")
def board_manager() -> BoardConfigManager:
    return BoardConfigManager(BOARDS_ROOT)


# ── chip + board discovery ─────────────────────────────────────────


def test_raspberrypi_chip_loads(board_manager):
    chip = board_manager.get_chip("raspberrypi")
    assert chip is not None, "raspberrypi chip YAML missing from boards/"
    assert chip.chip_family == "raspberrypi"
    assert chip.gpio_count > 0
    # The four hardware-PWM pins must carry the `pwm` cap so the UI
    # can offer them for servo outputs.
    pwm_pins = {p.gpio for p in chip.pin_capabilities if "pwm" in p.caps}
    assert {12, 13, 18, 19}.issubset(pwm_pins)


def test_raspberrypi_board_loads_and_is_default(board_manager):
    board = board_manager.get_board("raspberrypi")
    assert board is not None, "raspberrypi board YAML missing"
    assert board.builtin is True, \
        "raspberrypi must be shipped (builtin=True) so the UI marks it uneditable"
    assert board.chip_family == "raspberrypi"

    default = board_manager.default_board_for_chip("raspberrypi")
    assert default is not None
    assert default.board_id == "raspberrypi", \
        "raspberrypi must be the default board so adoption auto-assigns it"


def test_reserved_pins_match_firmware_list(board_manager):
    """The Pi-side gpio_control.py keeps RESERVED_PINS = [0, 1, 14, 15].
    The board YAML must list the same set or adoption hands the operator
    pins the kernel won't actually let them poke."""
    board = board_manager.get_board("raspberrypi")
    reserved = {p.gpio for p in board.reserved_pins}
    assert {0, 1, 14, 15} <= reserved


def test_derive_capabilities_intersects_chip_and_board(board_manager):
    """End-to-end check: derive_capabilities (called from the
    websocket capabilities derive) returns a coherent view with pins
    + uart_pairs + builtin_peripherals all wired through."""
    chip = board_manager.get_chip("raspberrypi")
    board = board_manager.get_board("raspberrypi")
    caps = derive_capabilities(chip, board)

    assert caps["chip_family"] == "raspberrypi"
    assert caps["board_id"] == "raspberrypi"
    # Every available pin must come back with its chip-level cap list.
    assert all("capabilities" in p for p in caps["pins"])
    # Built-ins list the audio_player entry.
    builtins = caps["builtin_peripherals"]
    types_seen = {b["type"] for b in builtins}
    assert "audio_player" in types_seen


# ── catalog entry ─────────────────────────────────────────────────


def test_audio_player_catalog_entry_shape():
    entry = DEFAULT_CATALOG.get("audio_player")
    assert entry is not None
    assert entry.pin_kind == "builtin"

    # Channels are what the trigger track / routing canvas binds against;
    # locking the order keeps the Pi-side SUB_CHANNEL_NAMES list in
    # sync. (Driver-side test pins the index→name mapping; this test
    # pins the catalog's view of those same names.)
    channel_ids = [c.id for c in entry.channels]
    assert channel_ids == [
        "play", "stop", "pause", "seek", "volume",
        "is_playing", "position",
    ]

    # Triggers are "out" (server → driver) with digital_out cap so a
    # logic source (joystick button, animation trigger that fires a
    # numeric pulse) can drive them.
    by_id = {c.id: c for c in entry.channels}
    for name in ("play", "stop", "pause"):
        assert by_id[name].dir == "out"
        assert by_id[name].cap == "digital_out"
    for name in ("seek", "volume"):
        assert by_id[name].dir == "out"
        assert by_id[name].cap == "analog"
    for name in ("is_playing",):
        assert by_id[name].dir == "in"
        assert by_id[name].cap == "digital_in"
    for name in ("position",):
        assert by_id[name].dir == "in"
        assert by_id[name].cap == "analog"


def test_audio_player_catalog_params():
    entry = DEFAULT_CATALOG.get("audio_player")
    by_id = {p.id: p for p in entry.params}

    # library_path tells the driver where to find files; default
    # matches the systemd unit's expected state dir layout.
    assert by_id["library_path"].type == "string"
    assert by_id["library_path"].default

    # backend should default to the only shipped option so adoption
    # works with no operator input.
    assert by_id["backend"].type == "string"
    assert by_id["backend"].default == "pi_alsa"

    # ALSA device defaults to "default" so the system sink is used —
    # works on Pi 5 (no 3.5mm jack), Pi 4 (3.5mm), USB DAC, I²S HAT.
    assert by_id["alsa_device"].type == "string"
    assert by_id["alsa_device"].default == "default"

    # Initial volume bounded 0..1 so a typo in the YAML can't blast
    # the speakers on adopt.
    iv = by_id["initial_volume"]
    assert iv.type == "float"
    assert 0.0 <= float(iv.default) <= 1.0
    assert iv.min == 0.0
    assert iv.max == 1.0


# ── seeding behavior ─────────────────────────────────────────────


def test_onboard_audio_is_in_board_builtin_peripherals(board_manager):
    board = board_manager.get_board("raspberrypi")
    by_id = {b.id: b for b in board.builtin_peripherals}
    assert "onboard_audio" in by_id, \
        "raspberrypi board must declare an onboard_audio audio_player"
    assert by_id["onboard_audio"].type == "audio_player"
    # Params should carry sensible defaults so the operator can adopt
    # without touching anything.
    p = by_id["onboard_audio"].params
    assert p.get("backend") == "pi_alsa"
    assert p.get("alsa_device")  # any non-empty string


def test_builtin_peripheral_refuses_removal():
    """Once seeded, builtin=True peripherals must be unremovable —
    NodePeripheralConfig.remove() filters them out of the deletion
    pass. Otherwise a stray DELETE from the UI could wipe the onboard
    audio player and the operator would have no way to get it back
    without re-adoption."""
    cfg = NodePeripheralConfig()
    cfg.upsert(PeripheralInstance(
        id="onboard_audio", type="audio_player",
        label="On-board Audio", pins={}, params={"backend": "pi_alsa"},
        builtin=True,
    ))
    assert cfg.get("onboard_audio") is not None
    removed = cfg.remove("onboard_audio")
    assert removed is False, "built-in peripheral should not be removable"
    assert cfg.get("onboard_audio") is not None


def test_seed_builtin_idempotent_signature():
    """Adopting a node twice (or restarting the server) must not
    duplicate the onboard_audio peripheral. The state_manager
    seeding logic uses peripheral_config.get(id) to check — verify
    the data model honors that lookup so the dedup actually works."""
    cfg = NodePeripheralConfig()
    inst = PeripheralInstance(
        id="onboard_audio", type="audio_player",
        label="On-board Audio", pins={}, params={}, builtin=True,
    )
    cfg.upsert(inst)
    assert cfg.get("onboard_audio") is inst
    # Re-upsert (same id) should replace in place, not duplicate.
    cfg.upsert(PeripheralInstance(
        id="onboard_audio", type="audio_player",
        label="On-board Audio (updated)", pins={}, params={}, builtin=True,
    ))
    matches = [p for p in cfg.peripherals if p.id == "onboard_audio"]
    assert len(matches) == 1
