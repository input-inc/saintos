"""Tests for the chip/board YAML loader + derived-capabilities view."""
from __future__ import annotations

import os
import sys
import textwrap

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.board_config import (
    BoardConfigManager,
    derive_capabilities,
)


# ─── filesystem fixtures ──────────────────────────────────────────────


def _write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(textwrap.dedent(text).lstrip())


@pytest.fixture()
def boards_root(tmp_path):
    """Synthetic chip + board layout under a temp dir."""
    root = tmp_path / "boards"
    _write(str(root / "rp2040" / "global.yaml"), """
        chip_family: rp2040
        display_name: "Test RP2040"
        chip_id_value: 0x00002927
        gpio_count: 30
        pin_capabilities:
          - {gpio: 0, default_name: "GP0", caps: ["digital_in", "digital_out", "uart_tx"]}
          - {gpio: 1, default_name: "GP1", caps: ["digital_in", "digital_out", "uart_rx"]}
          - {gpio: 5, default_name: "GP5", caps: ["digital_in", "digital_out", "pwm"]}
          - {gpio: 26, default_name: "GP26", caps: ["digital_in", "digital_out", "adc"]}
        uart_pairs:
          - {uart: 0, tx: 0, rx: 1}
        adc_pins: [26, 27, 28, 29]
    """)
    _write(str(root / "rp2040" / "test_board.yaml"), """
        board_id: test_board
        display_name: "Test Board"
        chip_family: rp2040
        builtin: true
        available_pins:
          - {gpio: 0, name: "TX"}
          - {gpio: 1, name: "RX"}
          - {gpio: 5, name: "D5"}
          - {gpio: 26, name: "A0"}
        reserved_pins:
          - {gpio: 16, name: "NEOPIXEL", reason: "Test"}
        builtin_peripherals:
          - {id: "test_np", type: "neopixel", label: "Test NeoPixel", pins: {data: 16}, params: {}}
    """)
    # Second board with a different chip_family (should still load — chip
    # may have different boards). And one with mismatched dir (should be
    # rejected).
    _write(str(root / "rp2040" / "bad_chip.yaml"), """
        board_id: bad_chip
        display_name: "Mismatched"
        chip_family: teensy41
        available_pins: []
    """)
    return str(root)


# ─── tests ────────────────────────────────────────────────────────────


class TestLoader:

    def test_loads_chip_global_conf(self, boards_root):
        m = BoardConfigManager(boards_root)
        chip = m.get_chip("rp2040")
        assert chip is not None
        assert chip.display_name == "Test RP2040"
        assert chip.gpio_count == 30
        assert chip.chip_id_value == 0x00002927
        assert len(chip.pin_capabilities) == 4
        assert {p.gpio for p in chip.pin_capabilities} == {0, 1, 5, 26}

    def test_loads_board_yaml(self, boards_root):
        m = BoardConfigManager(boards_root)
        b = m.get_board("test_board")
        assert b is not None
        assert b.chip_family == "rp2040"
        assert b.builtin is True
        assert {p.gpio for p in b.available_pins} == {0, 1, 5, 26}
        assert b.reserved_pins[0].reason == "Test"

    def test_rejects_board_with_wrong_chip_dir(self, boards_root):
        m = BoardConfigManager(boards_root)
        # bad_chip claims chip_family=teensy41 but lives in rp2040/.
        # The loader should skip it rather than silently associating
        # it with the wrong chip.
        assert m.get_board("bad_chip") is None

    def test_list_boards_for_chip(self, boards_root):
        m = BoardConfigManager(boards_root)
        boards = m.get_boards_for_chip("rp2040")
        ids = {b.board_id for b in boards}
        assert "test_board" in ids
        assert "bad_chip" not in ids   # rejected above

    def test_skips_macos_appledouble_shadows(self, boards_root):
        # Simulate a Finder-touched copy that left ._global.yaml and
        # ._test_board.yaml next to the real files. The loader must
        # ignore them; binary AppleDouble would otherwise UTF-8-decode-
        # fail and the chip dir would look broken.
        import os
        rp2040_dir = os.path.join(boards_root, "rp2040")
        with open(os.path.join(rp2040_dir, "._global.yaml"), "wb") as f:
            f.write(b"\x00\x05\x16\x07" + b"\x00" * 32 + b"\x90garbage")
        with open(os.path.join(rp2040_dir, "._test_board.yaml"), "wb") as f:
            f.write(b"\x00\x05\x16\x07" + b"\x00" * 32 + b"\x90garbage")
        m = BoardConfigManager(boards_root)
        assert m.get_chip("rp2040") is not None
        assert m.get_board("test_board") is not None
        # And the shadows weren't loaded as boards either.
        assert m.get_board("._test_board") is None

    def test_default_board_picks_built_in_first(self, boards_root):
        # Add an operator-authored board after the built-in; default
        # should still pick the built-in.
        _write(os.path.join(boards_root, "rp2040", "my_custom.yaml"), """
            board_id: my_custom
            display_name: "Custom"
            chip_family: rp2040
            builtin: false
            available_pins: []
        """)
        m = BoardConfigManager(boards_root)
        default = m.default_board_for_chip("rp2040")
        assert default is not None
        assert default.board_id == "test_board"
        assert default.builtin is True


class TestDerivedCapabilities:

    def test_intersects_chip_caps_with_board_pins(self, boards_root):
        m = BoardConfigManager(boards_root)
        view = derive_capabilities(m.get_chip("rp2040"), m.get_board("test_board"))
        gpios = sorted(p["gpio"] for p in view["pins"])
        assert gpios == [0, 1, 5, 26]
        a0 = next(p for p in view["pins"] if p["gpio"] == 26)
        assert "adc" in a0["capabilities"]
        assert a0["name"] == "A0"   # board name override wins

    def test_filters_uart_pairs_to_available_pins(self, boards_root):
        m = BoardConfigManager(boards_root)
        view = derive_capabilities(m.get_chip("rp2040"), m.get_board("test_board"))
        # uart0 tx=0 rx=1 — both available — should be in the view
        assert {"uart": 0, "tx": 0, "rx": 1} in view["uart_pairs"]

    def test_reserved_pins_listed(self, boards_root):
        m = BoardConfigManager(boards_root)
        view = derive_capabilities(m.get_chip("rp2040"), m.get_board("test_board"))
        assert 16 in view["reserved_pins"]

    def test_builtin_peripherals_passed_through(self, boards_root):
        m = BoardConfigManager(boards_root)
        view = derive_capabilities(m.get_chip("rp2040"), m.get_board("test_board"))
        ids = [b["id"] for b in view["builtin_peripherals"]]
        assert ids == ["test_np"]


class TestShippedConfigs:
    """Sanity-check the YAMLs we actually ship — make sure they load."""

    def test_ship_feather_rp2040_w5500(self):
        """The Feather RP2040 + W5500 board YAML must load + derive."""
        # Boards live under saint_os/config/boards at repo root.
        here = os.path.dirname(__file__)
        root = os.path.abspath(os.path.join(here, "..", "config", "boards"))
        m = BoardConfigManager(root)
        chip = m.get_chip("rp2040")
        board = m.get_board("feather_rp2040_w5500")
        assert chip is not None, "chip global.yaml failed to load"
        assert board is not None, "board YAML failed to load"
        view = derive_capabilities(chip, board)
        gpios = {p["gpio"] for p in view["pins"]}
        # The header-pad pins the operator should see.
        assert {0, 1, 2, 3, 5, 6, 9, 12, 13, 24, 25, 26, 27, 28, 29}.issubset(gpios)
        # Reserved should include the W5500 wing + NeoPixel.
        assert 10 in view["reserved_pins"]   # ETH_CS
        assert 16 in view["reserved_pins"]   # NEOPIXEL
        assert 18 in view["reserved_pins"]   # SPI_SCK
        # NeoPixel should be a builtin
        ids = [b["id"] for b in view["builtin_peripherals"]]
        assert "onboard_neopixel" in ids
