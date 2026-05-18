"""SAINT.OS chip + board configuration loader.

Reads the YAML files under ``saint_os/config/boards/<chip_family>/``:

  - ``global.yaml`` per directory: the chip-family definition (what's
    possible at the silicon level — every legal GPIO, every legal UART
    pair, ADC pins, the chip_id_value the firmware can sanity-check
    against).
  - Every other file in the directory: a board definition. Layers on
    top of the chip with the subset of GPIOs broken out to headers,
    the pins reserved by board-level hardware (FeatherWing CS, onboard
    NeoPixel, …), and any built-in peripherals.

The combined view (chip + board) is what the rest of the server uses
where it used to consume the firmware-emitted capability JSON: pin
selection in the Peripherals tab, validation when an operator saves a
peripheral, etc. The firmware no longer needs to advertise its pin
layout — it just identifies its chip and the server has the rest.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml


# ─────────────────────────────────────────────────────────────────────────────
# Dataclasses — mirror the YAML shape, used everywhere the server passes
# chip / board info around (state_manager, websocket_handler, etc.).
# ─────────────────────────────────────────────────────────────────────────────


@dataclass
class ChipPinCapability:
    """One row from the chip-level pin_capabilities table."""
    gpio: int
    default_name: str
    caps: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {"gpio": self.gpio, "default_name": self.default_name, "caps": list(self.caps)}

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "ChipPinCapability":
        return cls(
            gpio=int(d["gpio"]),
            default_name=str(d.get("default_name", f"GP{d['gpio']}")),
            caps=list(d.get("caps", [])),
        )


@dataclass
class ChipConfig:
    """Parsed contents of ``<chip>/global.yaml``."""
    chip_family: str
    display_name: str
    description: str = ""
    chip_id_value: Optional[int] = None
    gpio_count: int = 0
    pin_capabilities: List[ChipPinCapability] = field(default_factory=list)
    uart_pairs: List[Dict[str, int]] = field(default_factory=list)
    adc_pins: List[int] = field(default_factory=list)

    def get_pin(self, gpio: int) -> Optional[ChipPinCapability]:
        for p in self.pin_capabilities:
            if p.gpio == gpio:
                return p
        return None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "chip_family": self.chip_family,
            "display_name": self.display_name,
            "description": self.description,
            "chip_id_value": self.chip_id_value,
            "gpio_count": self.gpio_count,
            "pin_capabilities": [p.to_dict() for p in self.pin_capabilities],
            "uart_pairs": list(self.uart_pairs),
            "adc_pins": list(self.adc_pins),
        }

    @classmethod
    def from_yaml_dict(cls, d: Dict[str, Any]) -> "ChipConfig":
        return cls(
            chip_family=str(d["chip_family"]),
            display_name=str(d.get("display_name", d["chip_family"])),
            description=str(d.get("description", "")),
            chip_id_value=d.get("chip_id_value"),
            gpio_count=int(d.get("gpio_count", 0)),
            pin_capabilities=[ChipPinCapability.from_dict(p) for p in d.get("pin_capabilities", [])],
            uart_pairs=list(d.get("uart_pairs", [])),
            adc_pins=list(d.get("adc_pins", [])),
        )


@dataclass
class BoardPinEntry:
    """One entry in a board's available_pins or reserved_pins list."""
    gpio: int
    name: str
    reason: Optional[str] = None   # only set for reserved_pins

    def to_dict(self) -> Dict[str, Any]:
        d = {"gpio": self.gpio, "name": self.name}
        if self.reason is not None:
            d["reason"] = self.reason
        return d

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "BoardPinEntry":
        return cls(
            gpio=int(d["gpio"]),
            name=str(d.get("name", f"GP{d['gpio']}")),
            reason=d.get("reason"),
        )


@dataclass
class BoardBuiltinPeripheral:
    """One entry under a board's builtin_peripherals list."""
    id: str
    type: str
    label: str = ""
    pins: Dict[str, int] = field(default_factory=dict)
    params: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "type": self.type,
            "label": self.label or self.id,
            "pins": dict(self.pins),
            "params": dict(self.params),
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "BoardBuiltinPeripheral":
        return cls(
            id=str(d["id"]),
            type=str(d["type"]),
            label=str(d.get("label", "")),
            pins={k: int(v) for k, v in d.get("pins", {}).items()},
            params=dict(d.get("params", {})),
        )


@dataclass
class BoardConfig:
    """Parsed contents of a board YAML file."""
    board_id: str
    display_name: str
    chip_family: str
    builtin: bool = False
    available_pins: List[BoardPinEntry] = field(default_factory=list)
    reserved_pins: List[BoardPinEntry] = field(default_factory=list)
    builtin_peripherals: List[BoardBuiltinPeripheral] = field(default_factory=list)
    # Internal: the file path we loaded from, so we can write back to it
    # when an operator edits an operator-authored board. Built-in boards
    # carry this too for diagnostics but the UI refuses to write to them.
    source_path: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "board_id": self.board_id,
            "display_name": self.display_name,
            "chip_family": self.chip_family,
            "builtin": self.builtin,
            "available_pins": [p.to_dict() for p in self.available_pins],
            "reserved_pins": [p.to_dict() for p in self.reserved_pins],
            "builtin_peripherals": [b.to_dict() for b in self.builtin_peripherals],
        }

    @classmethod
    def from_yaml_dict(cls, d: Dict[str, Any], source_path: Optional[str] = None) -> "BoardConfig":
        return cls(
            board_id=str(d["board_id"]),
            display_name=str(d.get("display_name", d["board_id"])),
            chip_family=str(d["chip_family"]),
            builtin=bool(d.get("builtin", False)),
            available_pins=[BoardPinEntry.from_dict(p) for p in d.get("available_pins", [])],
            reserved_pins=[BoardPinEntry.from_dict(p) for p in d.get("reserved_pins", [])],
            builtin_peripherals=[BoardBuiltinPeripheral.from_dict(b)
                                  for b in d.get("builtin_peripherals", [])],
            source_path=source_path,
        )


# ─────────────────────────────────────────────────────────────────────────────
# Derived view — what the rest of the server uses as the equivalent of the
# firmware's old capability JSON. A board joined with its chip.
# ─────────────────────────────────────────────────────────────────────────────


def derive_capabilities(chip: ChipConfig, board: BoardConfig) -> Dict[str, Any]:
    """Return the dict the UI's pin picker expects.

    Same shape the firmware used to emit so the rest of the server
    (Peripherals tab, validation) doesn't have to change.
    """
    # For each available pin, intersect with the chip-level cap list.
    reserved_gpios = {r.gpio for r in board.reserved_pins}
    chip_by_gpio = {p.gpio: p for p in chip.pin_capabilities}

    pins = []
    for entry in board.available_pins:
        if entry.gpio in reserved_gpios:
            continue   # shouldn't happen but guard against typos
        chip_pin = chip_by_gpio.get(entry.gpio)
        if not chip_pin:
            continue
        pins.append({
            "gpio": entry.gpio,
            "name": entry.name,
            "capabilities": list(chip_pin.caps),
        })

    # UART pairs: filter chip-level legal pairs to only those where
    # BOTH tx and rx are operator-available on this board.
    available_gpios = {e.gpio for e in board.available_pins}
    uart_pairs = [
        dict(p) for p in chip.uart_pairs
        if p.get("tx") in available_gpios and p.get("rx") in available_gpios
    ]

    return {
        "node_id": None,    # filled in by caller
        "chip_family": chip.chip_family,
        "board_id": board.board_id,
        "pins": pins,
        "reserved_pins": [r.gpio for r in board.reserved_pins],
        "uart_pairs": uart_pairs,
        "builtin_peripherals": [b.to_dict() for b in board.builtin_peripherals],
    }


# ─────────────────────────────────────────────────────────────────────────────
# Loader — scans saint_os/config/boards/ and exposes get_chip / get_board.
# ─────────────────────────────────────────────────────────────────────────────


GLOBAL_CHIP_FILENAME = "global.yaml"


class BoardConfigManager:
    """Loads chip + board YAML at startup and serves lookups.

    Operator-authored boards can be added by dropping a YAML file into
    the chip's directory and calling reload(). Built-in board files are
    detected by ``builtin: true`` in their content and refuse server-
    side edits.
    """

    def __init__(self, root: str, logger=None):
        self.root = root
        self.logger = logger
        self.chips: Dict[str, ChipConfig] = {}
        self.boards: Dict[str, BoardConfig] = {}
        self.reload()

    def reload(self) -> None:
        self.chips.clear()
        self.boards.clear()
        if not os.path.isdir(self.root):
            self._warn(f"Board config root not found: {self.root}")
            return

        for entry in sorted(os.listdir(self.root)):
            # Skip macOS AppleDouble shadows that occasionally appear
            # next to real directories after a Finder copy.
            if entry.startswith("._"):
                continue
            chip_dir = os.path.join(self.root, entry)
            if not os.path.isdir(chip_dir):
                continue
            self._load_chip_dir(chip_dir)

    def _load_chip_dir(self, chip_dir: str) -> None:
        global_path = os.path.join(chip_dir, GLOBAL_CHIP_FILENAME)
        if not os.path.isfile(global_path):
            self._warn(f"Chip directory missing {GLOBAL_CHIP_FILENAME}: {chip_dir}")
            return
        try:
            with open(global_path, "r") as f:
                chip = ChipConfig.from_yaml_dict(yaml.safe_load(f) or {})
        except Exception as e:
            self._warn(f"Failed to load {global_path}: {e}")
            return
        self.chips[chip.chip_family] = chip

        for fname in sorted(os.listdir(chip_dir)):
            # Skip macOS AppleDouble metadata shadows (._foo.yaml). They
            # appear when a tarball is unpacked through Finder or moved
            # across HFS+/APFS — binary content with a .yaml extension
            # that the loader would otherwise try to parse.
            if fname.startswith("._"):
                continue
            if fname == GLOBAL_CHIP_FILENAME:
                continue
            if not fname.endswith((".yaml", ".yml")):
                continue
            board_path = os.path.join(chip_dir, fname)
            try:
                with open(board_path, "r") as f:
                    board = BoardConfig.from_yaml_dict(yaml.safe_load(f) or {}, source_path=board_path)
            except Exception as e:
                self._warn(f"Failed to load board {board_path}: {e}")
                continue
            if board.chip_family != chip.chip_family:
                self._warn(
                    f"{board_path}: chip_family '{board.chip_family}' doesn't match "
                    f"its directory '{chip.chip_family}' — skipping"
                )
                continue
            self.boards[board.board_id] = board

    def get_chip(self, chip_family: str) -> Optional[ChipConfig]:
        return self.chips.get(chip_family)

    def get_board(self, board_id: str) -> Optional[BoardConfig]:
        return self.boards.get(board_id)

    def get_boards_for_chip(self, chip_family: str) -> List[BoardConfig]:
        return [b for b in self.boards.values() if b.chip_family == chip_family]

    def list_chips(self) -> List[ChipConfig]:
        return list(self.chips.values())

    def list_boards(self) -> List[BoardConfig]:
        return list(self.boards.values())

    def default_board_for_chip(self, chip_family: str) -> Optional[BoardConfig]:
        """Pick a default board (first by sort order) for the chip family."""
        boards = sorted(
            self.get_boards_for_chip(chip_family),
            key=lambda b: (not b.builtin, b.board_id),  # built-ins first
        )
        return boards[0] if boards else None

    # ─── operator-authored board CRUD ────────────────────────────────

    def save_operator_board(self, board_yaml_text: str) -> Dict[str, Any]:
        """Write an operator-authored board YAML to disk + reload.

        Refuses to overwrite a board whose existing record has
        builtin=True. Validates chip_family points at a known chip
        directory. Returns {success, board_id} or {success: False,
        message}.
        """
        try:
            payload = yaml.safe_load(board_yaml_text) or {}
        except yaml.YAMLError as e:
            return {"success": False, "message": f"YAML parse error: {e}"}

        if not isinstance(payload, dict):
            return {"success": False, "message": "Top-level YAML must be a mapping"}

        for required in ("board_id", "display_name", "chip_family"):
            if required not in payload:
                return {"success": False, "message": f"Missing required field: {required}"}

        board_id = payload["board_id"]
        chip_family = payload["chip_family"]
        chip = self.get_chip(chip_family)
        if not chip:
            return {"success": False,
                    "message": f"Unknown chip_family '{chip_family}' "
                               f"(no global.yaml under boards/{chip_family}/)"}

        # Existing built-in? Refuse.
        existing = self.get_board(board_id)
        if existing and existing.builtin:
            return {"success": False,
                    "message": f"'{board_id}' is a built-in board and cannot be modified. "
                               f"Use a different board_id."}

        # Force builtin=false for operator-authored boards so the UI marks
        # them editable even if the operator pasted in builtin: true.
        payload["builtin"] = False

        chip_dir = os.path.join(self.root, chip_family)
        os.makedirs(chip_dir, exist_ok=True)
        out_path = os.path.join(chip_dir, f"{board_id}.yaml")

        try:
            with open(out_path, "w") as f:
                yaml.safe_dump(payload, f, sort_keys=False)
        except OSError as e:
            return {"success": False, "message": f"Write failed: {e}"}

        self.reload()
        return {"success": True, "board_id": board_id, "path": out_path}

    def delete_operator_board(self, board_id: str) -> Dict[str, Any]:
        """Delete an operator-authored board file. Refuses built-ins."""
        board = self.get_board(board_id)
        if not board:
            return {"success": False, "message": f"Unknown board_id '{board_id}'"}
        if board.builtin:
            return {"success": False,
                    "message": f"'{board_id}' is built-in and cannot be deleted"}
        if not board.source_path or not os.path.isfile(board.source_path):
            return {"success": False, "message": "Board has no source file on disk"}

        try:
            os.remove(board.source_path)
        except OSError as e:
            return {"success": False, "message": f"Delete failed: {e}"}

        self.reload()
        return {"success": True, "board_id": board_id}

    def get_board_yaml_text(self, board_id: str) -> Optional[str]:
        """Read the raw YAML for a board (for the editor view)."""
        board = self.get_board(board_id)
        if not board or not board.source_path:
            return None
        try:
            with open(board.source_path, "r") as f:
                return f.read()
        except OSError:
            return None

    def _warn(self, msg: str) -> None:
        if self.logger:
            self.logger.warning(msg)
        else:
            print(f"BoardConfigManager: {msg}")
