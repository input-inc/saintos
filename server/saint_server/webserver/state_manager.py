"""
SAINT.OS State Manager

Manages system state and provides data for WebSocket clients.
"""

import contextlib
import hashlib
import json
import zlib
import logging
import os
import re
import shutil
import subprocess
import time
import yaml
import psutil

# Map our string log levels onto stdlib logging levels for the file
# sinks. Anything not in the map falls through to INFO.
_LEVEL_TO_PY = {
    "debug": logging.DEBUG,
    "info": logging.INFO,
    "warn": logging.WARNING,
    "warning": logging.WARNING,
    "error": logging.ERROR,
    "critical": logging.CRITICAL,
}
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Any, Optional, Callable, Tuple

from saint_server.peripheral_model import (
    DEFAULT_CATALOG,
    DEFAULT_WIDGET_CATALOG,
    InputNode,
    NodePeripheralConfig,
    OPERATOR_CATALOG,
    OperatorNode,
    OutputNode,
    PeripheralInstance,
    PeripheralType,
    RouteEndpoint,
    SignalNode,
    SystemRouting,
    WidgetInstance,
    WidgetType,
    Wire,
    detect_pin_conflicts,
    maestro_normalize_channels,
    kangaroo_slim_params_for_wire,
    switch_input_params_for_wire,
    maestro_slim_channels_for_wire,
    pimoroni_normalize_channels,
    pimoroni_slim_channels_for_wire,
    strip_server_only_params,
    current_reading_channels,
)
from saint_server.board_config import BoardConfigManager, derive_capabilities
from saint_server.channel_arbiter import BOARD
from saint_server.router.routing_evaluator import DispatchTally

# Installed-version metadata — populated once by _read_installed_version_info()
# on first call to get_system_status() so we don't stat files every second.
_INSTALL_PREFIX = Path(os.environ.get("SAINT_INSTALL_PREFIX", "/opt/saint-os"))
_version_info_cache: Optional[Dict[str, Any]] = None


def _read_cpu_temp() -> Optional[float]:
    """Read the system CPU temperature in degrees Celsius.

    Uses /sys/class/thermal/thermal_zone0/temp which is the most portable
    source on Linux (Pi, Ubuntu Server, etc.). Returns None when the file
    isn't readable — happens on dev machines without thermal zones.
    """
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            return int(f.read().strip()) / 1000.0
    except (OSError, ValueError):
        return None


# Bit positions returned by `vcgencmd get_throttled`. See:
# https://www.raspberrypi.com/documentation/computers/os.html#get_throttled
_THROTTLE_BITS = {
    0x1:     ("undervolt",       "currently undervolted"),
    0x2:     ("freq_cap",        "ARM frequency currently capped"),
    0x4:     ("throttle",        "currently throttled"),
    0x8:     ("soft_temp_limit", "currently at soft temperature limit"),
    0x10000: ("undervolt_past",       "undervoltage has occurred"),
    0x20000: ("freq_cap_past",        "ARM frequency capping has occurred"),
    0x40000: ("throttle_past",        "throttling has occurred"),
    0x80000: ("soft_temp_limit_past", "soft temperature limit has occurred"),
}


def _read_throttle_status() -> Optional[Dict[str, Any]]:
    """Decode `vcgencmd get_throttled` into a structured dict.

    Pi-specific. Returns None on systems without vcgencmd (dev boxes).
    The raw value is a bitfield; we flag each set bit with a human-
    readable name so the UI can render badges per condition.
    """
    try:
        proc = subprocess.run(
            ["vcgencmd", "get_throttled"],
            capture_output=True, text=True, timeout=2,
        )
        if proc.returncode != 0:
            return None
        # Output format: "throttled=0x50000"
        raw_str = proc.stdout.strip().split("=", 1)[-1]
        raw = int(raw_str, 16)
    except (FileNotFoundError, subprocess.TimeoutExpired, ValueError, IndexError):
        return None

    flags = []
    descriptions = []
    for bit, (name, desc) in _THROTTLE_BITS.items():
        if raw & bit:
            flags.append(name)
            descriptions.append(desc)

    # Status summary: ok | warning (only past events) | critical (currently)
    currently_bad = bool(raw & 0xF)
    historically_bad = bool(raw & 0xF0000)
    if currently_bad:
        status = "critical"
        summary = "Currently throttled or undervolted"
    elif historically_bad:
        status = "warning"
        summary = "Throttling has occurred previously"
    else:
        status = "ok"
        summary = "No throttling"

    return {
        "raw": f"0x{raw:x}",
        "status": status,
        "summary": summary,
        "flags": flags,
        "descriptions": descriptions,
    }


def _read_installed_version_info() -> Dict[str, Any]:
    """Read the installed version from /opt/saint-os/VERSION and manifest.json.

    Cached after first successful read since these files don't change at
    runtime — the systemd unit restarts on install which re-imports this
    module.
    """
    global _version_info_cache
    if _version_info_cache is not None:
        return _version_info_cache

    info: Dict[str, Any] = {"version": "unknown", "git_sha": None, "built_at": None}
    vf = _INSTALL_PREFIX / "VERSION"
    if vf.is_file():
        try:
            info["version"] = vf.read_text().strip() or "unknown"
        except OSError:
            pass
    mf = _INSTALL_PREFIX / "manifest.json"
    if mf.is_file():
        try:
            m = json.loads(mf.read_text())
            info["git_sha"] = m.get("git_sha")
            info["built_at"] = m.get("built_at")
            # Prefer manifest's version if VERSION file was missing.
            if info["version"] == "unknown":
                info["version"] = m.get("version", "unknown")
        except (OSError, json.JSONDecodeError):
            pass

    _version_info_cache = info
    return info

# Timeout for considering a node offline (no announcements received)
# Increased from 5.0 to 10.0 to handle network jitter and temporary disconnects
# Nodes announce at 1Hz, so 10s allows for multiple missed announcements
NODE_TIMEOUT_SECONDS = 10.0

# Grace period before marking a node as truly offline (prevents flapping)
NODE_OFFLINE_GRACE_SECONDS = 3.0


# ─────────────────────────────────────────────────────────────────────────────
# Host controller (the Pi server itself) — modeled as a built-in node so its
# CPU temp / memory / throttle / uptime metrics are routable through the same
# system the rest of the dashboard uses.
# ─────────────────────────────────────────────────────────────────────────────
HOST_CONTROLLER_NODE_ID = "host_controller"


def _resolve_server_ip() -> str:
    """Best-effort: find an operator-useful IP for the host_controller
    NodeInfo. Prefers the configured internal-network server_ip (the
    address nodes use to reach us), falls back to the primary
    non-loopback address bound to any interface, or to the hostname.

    Returns "" when nothing usable is available — the Overview card
    just shows "—" in that case."""
    # Try server.yaml's network.internal.server_ip first; that's
    # what the install script reserves on eth0 for the node net.
    try:
        from saint_server.config import get_config
        cfg = get_config()
        ip = getattr(getattr(getattr(cfg, "network", None),
                             "internal", None), "server_ip", "")
        if ip and ip != "0.0.0.0":
            return str(ip)
    except Exception:
        pass
    # Then try to figure out what interface has a route to the
    # outside world. socket.gethostbyname is sometimes pegged at
    # 127.0.1.1 on Debian, so probe via a UDP connect dance instead.
    try:
        import socket
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 1))     # no packet actually sent
            return s.getsockname()[0]
    except Exception:
        pass
    try:
        import socket
        return socket.gethostname()
    except Exception:
        return ""


# ─────────────────────────────────────────────────────────────────────────────
# Virtual-GPIO → channel translation (transitional)
# ─────────────────────────────────────────────────────────────────────────────
#
# RP2040 firmware still publishes peripheral readings on a per-type
# `virtual_gpio_base + firmware_channel_index` integer (see
# firmware/shared/include/*_protocol.h). The routing system addresses
# things as (peripheral_id, channel_id) — operator-visible names. This
# table bridges the two so the server can emit channel-shaped readings
# regardless of the firmware's internal encoding. Once the firmware
# emits channel-addressed data directly, _firmware_channel_id() can
# return None for every mode and this table can be deleted.
#
# Retirement plan: docs/PERIPHERAL_FIRST_MIGRATION.md. Adding a NEW
# channel to a driver in this table is a hazard — read the doc's
# "hazards" section first (the slabs are contiguous and silently
# collide).
#
# Keys: the `mode` string the firmware emits in pin_state pin entries.
# Values: (virtual_gpio_base, {firmware_channel_index: catalog_channel_id})
# A firmware channel that doesn't map to any operator-visible catalog
# channel (e.g. BMS REMAIN_CAP, cycle count, individual cell voltages
# beyond what the BMS catalog exposes) is omitted — those values are
# kept in the per-pin runtime state but don't fire route dispatches.

_FIRMWARE_CHANNEL_MAP: Dict[str, Tuple[int, Dict[int, str]]] = {
    # FAS100_VIRTUAL_GPIO_BASE = 232, 4 channels
    "fas100_sensor": (232, {
        0: "amps", 1: "volts", 2: "temp1", 3: "temp2",
    }),
    # SYREN_VIRTUAL_GPIO_BASE = 224. Each SyRen peripheral instance has
    # a single "motor" channel; the firmware-side channel index just
    # picks which of the 8 multi-drop slots the instance occupies.
    # The peripheral_id (from pin_config_t.logical_name) disambiguates.
    "syren_motor": (224, {i: "motor" for i in range(8)}),
    # ROBOCLAW_VIRTUAL_GPIO_BASE = 236, 5 channels per unit * 8 units.
    # Same disambiguation pattern as syren.
    "roboclaw_motor": (236, {
        (unit * 5 + sub): name
        for unit in range(8)
        for sub, name in enumerate(("motor", "encoder", "voltage", "current", "temp"))
    }),
    # JBD_BMS_VIRTUAL_GPIO_BASE = 276. Catalog exposes 5 channels;
    # firmware indexes 3 (REMAIN_CAP), 6 (CYCLES), 7 (PROTECTION) and
    # 8..23 (per-cell) have no catalog channel and are intentionally
    # skipped from this map.
    "pathfinder_bms_sensor": (276, {
        0: "pack_voltage", 1: "current", 2: "soc", 4: "temp_1", 5: "temp_2",
    }),
    # TIC_VIRTUAL_GPIO_BASE = 300, 6 channels per unit * 8 units.
    # Same multi-unit pattern as RoboClaw; the peripheral_id from
    # pin_config_t.logical_name disambiguates which unit.
    "tic_stepper": (300, {
        (unit * 6 + sub): name
        for unit in range(8)
        for sub, name in enumerate((
            "target_position", "target_velocity",
            "current_position", "current_velocity",
            "vin_voltage", "error_status",
        ))
    }),
    # TMC2208_VIRTUAL_GPIO_BASE = 348, 4 channels per axis * 4 axes.
    "tmc2208_stepper": (348, {
        (axis * 4 + sub): name
        for axis in range(4)
        for sub, name in enumerate((
            "target_position", "target_velocity",
            "current_position", "error_flags",
        ))
    }),
    # KANGAROO_VIRTUAL_GPIO_BASE = 364, 10 channels per unit * 8 units.
    # One Kangaroo motor channel = one unit; the peripheral_id from
    # pin_config_t.logical_name disambiguates which (address, channel).
    #
    # The stride and the order below MUST match KANGAROO_CHANNELS_PER_UNIT
    # and the KANGAROO_SUB_* indices in
    # firmware/shared/include/kangaroo_protocol.h. They are separate
    # declarations of one wire contract: get the stride wrong and unit 1's
    # channels silently decode as unit 0's.
    "kangaroo_motion": (364, {
        (unit * 10 + sub): name
        for unit in range(8)
        for sub, name in enumerate((
            "target_position", "target_speed",
            "current_position", "current_speed",
            "moving", "error_status",
            "jog", "tune_state",
            "taught_min", "taught_max",
        ))
    }),
}


def _firmware_channel_id(mode: str, gpio: int) -> Optional[str]:
    """Resolve a (mode, virtual gpio) into a catalog channel_id.

    Returns None if this gpio doesn't represent an operator-visible
    peripheral channel (e.g. a physical GPIO running PWM, or a
    firmware-internal BMS channel that isn't routed to widgets).
    """
    entry = _FIRMWARE_CHANNEL_MAP.get(mode)
    if not entry:
        return None
    base, channel_map = entry
    return channel_map.get(gpio - base)


HOST_CONTROLLER_PERIPHERAL_ID = "system_monitor"


# =============================================================================
# Pin Configuration Dataclasses
# =============================================================================

@dataclass
class PinCapability:
    """Capability of a single physical pin."""
    gpio: int
    name: str
    capabilities: List[str] = field(default_factory=list)

    def supports_mode(self, mode: str) -> bool:
        """Check if pin supports the given mode."""
        mode_map = {
            'digital_in': 'digital_in',
            'digital_out': 'digital_out',
            'pwm': 'pwm',
            'servo': 'pwm',  # Servo uses PWM
            'adc': 'adc',
            'i2c_sda': 'i2c_sda',
            'i2c_scl': 'i2c_scl',
            'uart_tx': 'uart_tx',
            'uart_rx': 'uart_rx',
            'maestro_servo': 'maestro_servo',
            'syren_motor': 'syren_motor',
            'fas100_sensor': 'fas100_sensor',
            'roboclaw_motor': 'roboclaw_motor',
            'pathfinder_bms_sensor': 'pathfinder_bms_sensor',
            'tic_stepper': 'tic_stepper',
            'tmc2208_stepper': 'tmc2208_stepper',
            'kangaroo_motion': 'kangaroo_motion',
        }
        required_cap = mode_map.get(mode, mode)
        return required_cap in self.capabilities


@dataclass
class NodeCapabilities:
    """Complete pin capabilities for a node."""
    node_id: str
    pins: List[PinCapability] = field(default_factory=list)
    reserved_pins: List[int] = field(default_factory=list)
    # Legal (uart_instance, tx_pin, rx_pin) tuples for UART peripherals.
    # Each entry is a dict like {"uart": 0, "tx": 0, "rx": 1}.
    uart_pairs: List[Dict[str, int]] = field(default_factory=list)
    last_updated: float = field(default_factory=time.time)

    def get_pin(self, gpio: int) -> Optional[PinCapability]:
        """Get capability for a specific GPIO."""
        for pin in self.pins:
            if pin.gpio == gpio:
                return pin
        return None

    def get_pins_for_mode(self, mode: str) -> List[PinCapability]:
        """Get all pins that support a given mode."""
        return [pin for pin in self.pins if pin.supports_mode(mode)]

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            "node_id": self.node_id,
            "pins": [
                {
                    "gpio": p.gpio,
                    "name": p.name,
                    "capabilities": p.capabilities,
                }
                for p in self.pins
            ],
            "reserved_pins": self.reserved_pins,
            "uart_pairs": self.uart_pairs,
        }


@dataclass
class PinRuntimeState:
    """Runtime state for a single pin."""
    gpio: int
    mode: str
    logical_name: str = ""
    desired_value: Optional[float] = None  # User-requested value (0-100 for PWM, 0-180 for servo, 0/1 for digital)
    actual_value: Optional[float] = None   # Feedback from firmware
    voltage: Optional[float] = None        # ADC voltage reading
    last_updated: float = 0.0              # Timestamp of last actual value update

    @property
    def synced(self) -> bool:
        """Check if desired and actual values are in sync."""
        if self.desired_value is None:
            return True
        if self.actual_value is None:
            return False
        # Allow small tolerance for PWM/servo
        tolerance = 1.0 if self.mode in ('pwm', 'servo') else 0.01
        return abs(self.desired_value - self.actual_value) < tolerance

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        result = {
            "gpio": self.gpio,
            "mode": self.mode,
            "logical_name": self.logical_name,
            "synced": self.synced,
        }
        if self.desired_value is not None:
            result["desired"] = self.desired_value
        if self.actual_value is not None:
            result["actual"] = self.actual_value
        if self.voltage is not None:
            result["voltage"] = self.voltage
        result["last_updated"] = self.last_updated
        return result


@dataclass
class ChannelRuntimeState:
    """Latest reading for one peripheral channel.

    The peripheral-first equivalent of PinRuntimeState. Indexed by
    (peripheral_id, channel_id) since that's the routing graph's
    addressing scheme.
    """
    peripheral_id: str
    channel_id: str
    value: Optional[float] = None
    last_updated: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "peripheral_id": self.peripheral_id,
            "channel_id": self.channel_id,
            "value": self.value,
            "last_updated": self.last_updated,
        }


@dataclass
class NodeRuntimeState:
    """Runtime state for a node — per-pin and per-channel readings."""
    node_id: str
    pins: Dict[int, PinRuntimeState] = field(default_factory=dict)  # gpio -> state
    # New peripheral-first channel readings, keyed by (peripheral_id, channel_id).
    channels: Dict[Tuple[str, str], ChannelRuntimeState] = field(default_factory=dict)
    last_feedback: float = 0.0  # Timestamp of last state feedback from firmware

    def get_pin(self, gpio: int) -> Optional[PinRuntimeState]:
        """Get runtime state for a pin."""
        return self.pins.get(gpio)

    def set_channel(self, peripheral_id: str, channel_id: str, value: float) -> None:
        key = (peripheral_id, channel_id)
        ch = self.channels.get(key)
        if not ch:
            ch = ChannelRuntimeState(peripheral_id=peripheral_id, channel_id=channel_id)
            self.channels[key] = ch
        ch.value = value
        ch.last_updated = time.time()

    def set_pin_desired(self, gpio: int, value: float):
        """Set desired value for a pin."""
        if gpio in self.pins:
            self.pins[gpio].desired_value = value

    def update_from_firmware(self, pins_data: List[Dict[str, Any]]) -> List[Tuple[str, str, float]]:
        """Update actual values from firmware feedback (legacy GPIO path).

        The firmware still publishes some peripheral channels on
        ``virtual GPIO`` slots (a transitional artifact of the
        peripheral-first refactor — see
        ``docs/PERIPHERAL_FIRST_MIGRATION.md``). We translate them
        here into the peripheral-first ``channels`` view that the
        routing graph and the UI consume. The channel-addressed
        emission path (``update_channels_from_firmware`` below) is
        the destination — once every driver migrates, this method
        only handles real GPIOs (PWM / ADC / digital_io).

        Returns the list of (peripheral_id, channel_id, value) tuples
        that were resolved on this call — the caller uses this to feed
        the optional peripheral logger without re-deriving the mapping.
        """
        now = time.time()
        self.last_feedback = now
        channel_updates: List[Tuple[str, str, float]] = []

        for pin_data in pins_data:
            gpio = pin_data.get('gpio')
            if gpio is None:
                continue

            if gpio not in self.pins:
                # Create new runtime state for this pin
                self.pins[gpio] = PinRuntimeState(
                    gpio=gpio,
                    mode=pin_data.get('mode', 'unknown'),
                    logical_name=pin_data.get('name', ''),
                )

            pin = self.pins[gpio]
            pin.mode = pin_data.get('mode', pin.mode)
            pin.logical_name = pin_data.get('name', pin.logical_name)
            pin.actual_value = pin_data.get('value')
            pin.voltage = pin_data.get('voltage')
            pin.last_updated = now

            # Peripheral-first translation: if this pin belongs to a
            # peripheral driver, also surface the reading as a
            # (peripheral_id, channel_id) channel value.
            value = pin_data.get('value')
            mode = pin.mode
            peripheral_id = pin.logical_name
            channel_id = _firmware_channel_id(mode, gpio)
            if value is not None and peripheral_id and channel_id:
                fvalue = float(value)
                self.set_channel(peripheral_id, channel_id, fvalue)
                channel_updates.append((peripheral_id, channel_id, fvalue))

        return channel_updates

    def update_channels_from_firmware(
            self, channels_data: List[Dict[str, Any]],
    ) -> List[Tuple[str, str, float]]:
        """Direct ingest of channel-addressed firmware state.

        Firmware drivers that have migrated under the peripheral-first
        plan emit a `channels` array alongside the legacy `pins`
        array, with shape `[{peripheral_id, channel_id, value}, ...]`.
        Each entry lands straight in NodeRuntimeState.channels — no
        slab math, no `_FIRMWARE_CHANNEL_MAP` lookup. Drivers can
        emit through this path on a per-channel basis, so partial
        migrations are allowed (servo positions via pins[], diagnostic
        channels via channels[]) while individual drivers move over.
        See docs/PERIPHERAL_FIRST_MIGRATION.md.

        Returns the resolved tuples for the logger, mirroring
        update_from_firmware.
        """
        now = time.time()
        self.last_feedback = now
        out: List[Tuple[str, str, float]] = []
        for entry in channels_data or ():
            peripheral_id = entry.get('peripheral_id') or entry.get('peripheral')
            channel_id    = entry.get('channel_id')    or entry.get('channel')
            value         = entry.get('value')
            if not peripheral_id or not channel_id or value is None:
                continue
            try:
                fvalue = float(value)
            except (TypeError, ValueError):
                continue
            self.set_channel(peripheral_id, channel_id, fvalue)
            out.append((peripheral_id, channel_id, fvalue))
        return out

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            "node_id": self.node_id,
            "pins": [pin.to_dict() for pin in self.pins.values()],
            "channels": [ch.to_dict() for ch in self.channels.values()],
            "last_feedback": self.last_feedback,
            "stale": (time.time() - self.last_feedback) > 1.5 if self.last_feedback > 0 else True,
        }


@dataclass
class NodeInfo:
    """Information about an adopted or unadopted node."""
    node_id: str
    hardware_model: str = "Unknown"
    mac_address: str = ""
    ip_address: str = ""
    firmware_version: str = "0.0.0"
    # Bootloader version the node reports in its announcement ("bl_fw"
    # field). Tracked separately from firmware_version because the
    # bootloader is NOT OTA-updatable — it's the thing that performs
    # updates. A board's bootloader is effectively frozen at the
    # version it was BOOTSEL-flashed with, so we need independent
    # visibility to know which boards in the field still need a
    # physical reflash to pick up bootloader-side fixes (DHCP, retry
    # budget, failure reporting, etc.). "unknown" is reported by
    # firmware running without an OTA bootloader, by sim builds, or by
    # older bootloaders that predate the bl_info descriptor.
    bootloader_version: str = "unknown"
    firmware_build: str = ""  # Build timestamp
    state: str = "UNKNOWN"  # Node state from firmware (UNADOPTED, ACTIVE, ERROR, etc.)
    online: bool = True
    cpu_temp: float = 0.0
    cpu_usage: float = 0.0
    memory_usage: float = 0.0
    uptime_seconds: int = 0
    # Adopted node specific
    display_name: str = ""
    role: str = ""
    # Chip family the node announced (e.g. "rp2040"). The server matches
    # this against ``config/boards/<chip_family>/global.yaml``.
    chip_family: str = ""
    # Board the operator picked at adoption time (e.g. "feather_rp2040_w5500").
    # Pin layout, reserved pins, and built-in peripherals are derived from
    # the chip + board YAML on the server — the firmware doesn't advertise
    # them any more.
    board_id: str = ""
    # Peripheral attachments + per-pin capabilities reported by the firmware.
    capabilities: Optional[NodeCapabilities] = None
    peripheral_config: Optional[NodePeripheralConfig] = None
    # Runtime state — per-channel readings (formerly per-GPIO pin state).
    runtime_state: Optional[NodeRuntimeState] = None
    # Tracking
    last_seen: float = field(default_factory=time.time)
    # Grace period tracking - when we first detected potential disconnect
    going_offline_at: Optional[float] = None
    # Per-node log ring buffer. Capped to MAX_NODE_LOG_ENTRIES on the
    # StateManager that owns this NodeInfo. Surfaced to the UI via the
    # Logs tab + node_log/<id> subscription stream. In-memory only.
    log_entries: List[Dict[str, Any]] = field(default_factory=list)
    # Snapshot of the announcement's `peripherals: {id: connected}` map
    # from the previous announcement, so we can detect connect/disconnect
    # transitions and log them once instead of every announcement.
    peripheral_connected: Dict[str, bool] = field(default_factory=dict)
    # Last time we re-pushed our config to recover this adopted node
    # from a firmware-side UNADOPTED state (typically after an OTA wiped
    # or invalidated the saved flash config). Unix seconds. Rate-limits
    # the auto-reconcile in server_node._on_node_announcement so we
    # don't hammer a node every 1 s announcement when apply keeps
    # failing.
    last_reconcile_push_at: float = 0.0
    # Config sync tag this node last reported in /announce — what it
    # says it is actually holding. None = never reported one (firmware
    # predating the field). See docs/CONFIG_SYNC.md.
    reported_cfg_tag: Optional[int] = None


@dataclass
class SystemState:
    """Current system state."""
    server_online: bool = True
    start_time: float = field(default_factory=time.time)
    server_name: str = "SAINT-01"
    server_version: str = "0.5.0"
    adopted_nodes: Dict[str, NodeInfo] = field(default_factory=dict)
    unadopted_nodes: Dict[str, NodeInfo] = field(default_factory=dict)
    websocket_client_count: int = 0
    livelink_enabled: bool = True
    livelink_source_count: int = 0
    rc_enabled: bool = False
    rc_connected: bool = False
    # System-wide routing graph (routes + dashboard widgets)
    system_routing: SystemRouting = field(default_factory=SystemRouting)


def _board_dispatch(ev, tally):
    """Mark a fan-out as board-owned when the evaluator supports it.

    Production has exactly one evaluator and it always does; the getattr
    keeps partial test doubles (and any future evaluator built against
    the older interface) from turning a pose application into an
    exception. Losing the marker only costs the force-send, not
    correctness, so degrading is safe — but it must never be silent in
    production, which is why the real evaluator is the one that defines
    dispatch_as.
    """
    dispatch_as = getattr(ev, "dispatch_as", None)
    if dispatch_as is None:
        return contextlib.nullcontext()
    return dispatch_as(BOARD, tally)


class _BoardControl:
    """Board-item control handed to animation players.

    An animation can fire a sound or start another animation at a
    keyframe. Those need the soundboard callback and the player registry
    — both of which live on the StateManager — so this adapts them to
    the narrow surface AnimationPlayer wants (see BoardControl in
    animation/player.py) instead of handing players the whole manager.

    Every method degrades rather than raises: a missing sound or an
    unconfigured callback must not take down a running show.
    """

    def __init__(self, manager: "StateManager"):
        self._m = manager

    def play_sound(self, sound_id: str) -> Optional[str]:
        resolved = self._m.resolve_sound_play(sound_id)
        if not resolved or not resolved.get("node_id"):
            return None
        cb = self._m._soundboard_dispatch
        if cb is None:
            return None
        try:
            cb(resolved["node_id"], "soundboard_play", resolved["args"], "")
        except Exception:
            return None
        return str(resolved["node_id"])

    def stop_sound(self, node_id: str) -> None:
        cb = self._m._soundboard_dispatch
        if cb is None or not node_id:
            return
        try:
            cb(node_id, "soundboard_stop", {}, "")
        except Exception:
            pass

    async def start_animation(self, animation_id: str, depth: int) -> bool:
        registry = self._m._animation_registry
        if registry is None:
            return False
        anim = self._m.animation_store.get(animation_id)
        if anim is None:
            return False
        await registry.start(anim, depth=depth)
        return True

    async def stop_animation(self, animation_id: str) -> None:
        registry = self._m._animation_registry
        if registry is None:
            return
        await registry.stop(animation_id)

    def sound_is_looping(self, sound_id: str) -> bool:
        snd = self._m.sound_store.get(sound_id)
        return bool(snd and snd.loop)

    def animation_is_looping(self, animation_id: str) -> bool:
        anim = self._m.animation_store.get(animation_id)
        return bool(anim and anim.loop)


class StateManager:
    """Manages system state and provides data for clients."""

    MAX_LOG_ENTRIES = 500          # Global activity-log ring buffer cap
    MAX_NODE_LOG_ENTRIES = 200     # Per-node log ring buffer cap

    def __init__(self, server_name: str = "SAINT-01", logger=None, config_dir: Optional[str] = None):
        self.state = SystemState(server_name=server_name)
        self.logger = logger
        self._activity_callback: Optional[Callable[[str, str], None]] = None
        # Optional callback fired whenever host_controller's peripheral
        # config mutates (upsert / remove / load). Wired by server_node
        # to the HostPeripheralManager so it can reconcile its BLE
        # driver set. Takes the new peripherals list (each entry is the
        # standard {id, type, pins, params} dict).
        self._host_peripheral_reconcile_cb: Optional[
            Callable[[List[Dict[str, Any]]], None]] = None
        # Optional callback fired when a node-scoped log entry is recorded.
        # Wired by server_node.py to broadcast on node_log/<id>.
        self._node_log_callback: Optional[Callable[[str, Dict[str, Any]], None]] = None
        # Optional file-backed activity sinks (set by set_activity_file_logger).
        # server_node.py wires these to /var/log/saint-os/saint-server.log
        # and /var/log/saint-os/nodes/<node_id>.log so activity is still
        # available after a restart, separate from the systemd journal.
        self._activity_server_logger = None
        self._activity_per_node_logger = None
        self._log_entries: List[Dict[str, Any]] = []  # Circular buffer for logs

        # Runtime config (persists across installs) vs. shipped config
        # (boards/, replaced from the dist on every install).
        #
        # The install layout splits them:
        #   /etc/saint-os/                  ← runtime: nodes/, system_routing.yaml
        #   /opt/saint-os/install/share/    ← shipped: boards/*.yaml
        #
        # Old layout (everything under share/saint_os/config) gets wiped
        # by install.sh's `rm -rf ${PREFIX}/install`, so any adoptions
        # made before this change would be lost on update. This is the
        # bug that prompted the split.
        #
        # ``config_dir`` (constructor arg) and the SAINT_RUNTIME_CONFIG_DIR
        # env var both override the runtime path — used by tests + ops
        # who want to point at a different location.
        if config_dir is None:
            config_dir = os.environ.get("SAINT_RUNTIME_CONFIG_DIR")
        if config_dir is None:
            etc_dir = "/etc/saint-os"
            if os.path.isdir(etc_dir):
                config_dir = etc_dir
        if config_dir is None:
            # Dev fallback — repo's server/config/ tree.
            config_dir = os.path.join(
                os.path.dirname(__file__), '..', '..', 'config'
            )
        self.config_dir = os.path.abspath(config_dir)
        self.nodes_config_dir = os.path.join(self.config_dir, 'nodes')
        self.system_routing_path = os.path.join(self.config_dir, 'system_routing.yaml')

        # Boards live in the shipped share/saint_os/config tree (or the
        # repo config/boards in dev). They're read-only from the
        # server's POV — operator edits flow through save_board_yaml(),
        # which writes back to wherever this resolves to. On a Pi
        # install /opt/saint-os/install/share/saint_os/config/boards is
        # part of the install tree and gets refreshed each update; an
        # operator-authored custom board has to be re-applied via the
        # Settings → Boards UI after each install.
        boards_config_dir = None
        try:
            from ament_index_python.packages import get_package_share_directory
            boards_config_dir = os.path.join(
                get_package_share_directory('saint_os'), 'config', 'boards'
            )
        except Exception:
            pass
        if not boards_config_dir or not os.path.isdir(boards_config_dir):
            # Dev fallback: alongside the runtime config dir.
            boards_config_dir = os.path.join(self.config_dir, 'boards')
        self.boards_config_dir = boards_config_dir

        # In-memory peripheral type catalog. Starts from the built-in
        # DEFAULT_CATALOG and gets extended when nodes report platform-
        # specific types in their capabilities.
        self.peripheral_catalog: Dict[str, PeripheralType] = dict(DEFAULT_CATALOG)
        self.widget_catalog: Dict[str, WidgetType] = dict(DEFAULT_WIDGET_CATALOG)

        # Optional peripheral telemetry logger. Wired by server_node at
        # startup so that load_all_node_configs() (called below) can
        # rehydrate the enabled set without a separate pass — see the
        # set_peripheral_logger() seeding logic.
        self.peripheral_logger = None

        # Why the last config build for a node was refused, keyed by
        # node_id. Populated by get_firmware_config_json's budget guard
        # and cleared on the next successful build, so the Sync action
        # can report the real reason rather than "nothing to sync".
        self._config_push_errors: Dict[str, str] = {}

        # Last config payload actually pushed to each node, so the next
        # change can go out as a delta instead of a full re-push. An
        # optimization only — see record_config_push.
        self._last_pushed_json: Dict[str, str] = {}

        # Optional routing graph evaluator. Set by server_node once the
        # ROS bridge is up. Whenever sheets change we call .reconcile()
        # on it so it can refresh its subscriptions.
        self._routing_evaluator = None
        self._channel_arbiter = None
        # Out-of-band peripheral-command publisher (set by
        # server_node.py once ROS publishers are up). Used by the
        # animation player when a trigger track of kind
        # peripheral_command fires.
        self._peripheral_command_sender = None
        # The animation player needs to publish to ROS topics for the
        # trigger-track `topic` dispatch path. Wired alongside the
        # routing evaluator at server_node startup.
        self._ros_bridge = None

        # Animation / pose stores + playback registry. Stores are
        # immediately usable from disk; the player registry can only
        # start animations once the routing evaluator + bridge are
        # wired (it depends on both for the dispatch fan-out).
        from saint_server.animation.store import (
            AnimationStore, PlaylistStore, PoseStore, SoundStore,
        )
        self.animation_store = AnimationStore(self.config_dir, logger=self.logger)
        self.pose_store = PoseStore(self.config_dir, logger=self.logger)
        self.sound_store = SoundStore(self.config_dir, logger=self.logger)
        # Playlists replaced the per-item `group` string (an item can be
        # in several now). Migrating here, at construction, means the
        # first list_* call already sees playlists rather than the UI
        # having to cope with a half-converted library. It is a no-op
        # after the first run — see PlaylistStore.migrate_legacy_groups.
        self.playlist_store = PlaylistStore(self.config_dir, logger=self.logger)
        try:
            self.playlist_store.migrate_legacy_groups({
                "animations": self.animation_store.list(),
                "poses": self.pose_store.list(),
                "sounds": self.sound_store.list(),
            })
        except Exception as e:
            # A library that can't be migrated must not stop the server
            # from booting — the operator just sees no playlists.
            if self.logger:
                self.logger.error(
                    f"Playlist migration failed: {type(e).__name__}: {e}")
        self._animation_registry = None
        # Node soundboard sender, mirrored here by
        # WebSocketHandler.set_soundboard_callback so an animation's
        # sound board-items can reach a node without the player needing
        # the websocket handler. None until the server node wires it.
        self._soundboard_dispatch: Optional[
            Callable[[str, str, dict, str], None]] = None
        # Cached rig evaluator + the pose generation that invalidates it.
        # See _rig_evaluator: rebuilding parses the URDF, so it must not
        # happen per slider tick.
        self._rig_eval_cache = None
        self._rig_eval_key = None
        self._pose_generation = 0
        # Robot model store (URDF + SRDF + rig). Owned and constructed by
        # http_server, which injects it here — we need it to resolve
        # group_state imports, convert SRDF radians to the normalized
        # −1..+1 the pose library stores, and evaluate the rig. Left None
        # in headless/test setups that never serve HTTP, so every reader
        # has to tolerate its absence.
        self.robot_store = None

        # Chip + board YAML catalog. Replaces the firmware-emitted
        # capability JSON as the source of truth for "what pins this
        # node has." Loaded at startup; the Settings UI will support
        # adding operator-authored boards in a later phase.
        self.board_config = BoardConfigManager(self.boards_config_dir, logger=self.logger)

        # Load adopted nodes from disk on startup
        self.load_all_node_configs()
        # Migrate any pre-board_id nodes so their pin layouts resolve.
        self._migrate_missing_board_ids()
        # Load system-wide routes + widgets
        self._load_system_routing()
        # Always-present synthetic node for the host controller itself
        self._ensure_host_controller_node()

    def set_activity_callback(self, callback: Callable[[str, str], None]):
        """Set callback for activity logging. Callback takes (message, level)."""
        self._activity_callback = callback

    def set_node_log_callback(self, callback: Callable[[str, Dict[str, Any]], None]):
        """Set callback for per-node log events. Takes (node_id, entry)."""
        self._node_log_callback = callback

    def set_peripheral_logger(self, logger) -> None:
        """Attach the optional peripheral telemetry logger.

        Walks the already-loaded node configs and seeds the logger's
        enabled set so persisted log_enabled flags take effect without
        a restart-after-config-edit. Safe to call with None to detach.
        """
        self.peripheral_logger = logger
        if logger is None:
            return
        for node_id, node in self.state.adopted_nodes.items():
            if not node.peripheral_config:
                continue
            for p in node.peripheral_config.peripherals:
                if p.log_enabled:
                    logger.set_enabled(node_id, p.id, True)

    def set_activity_file_logger(self, server_logger, per_node_logger) -> None:
        """Attach optional file-backed activity loggers.

        ``server_logger`` is a stdlib ``Logger`` with a daily-rotating
        file handler attached — every activity event goes here.
        ``per_node_logger`` is a ``PerKeyLogger`` (see file_log.py);
        per-node events are routed to ``<node_id>.log``.

        Either may be ``None`` to disable that sink.
        """
        self._activity_server_logger = server_logger
        self._activity_per_node_logger = per_node_logger

    def get_node_logs(self, node_id: str) -> List[Dict[str, Any]]:
        """Return the log buffer for a node (oldest first). Empty if unknown."""
        node = (self.state.adopted_nodes.get(node_id)
                or self.state.unadopted_nodes.get(node_id))
        if not node:
            return []
        return list(node.log_entries)

    def clear_node_logs(self, node_id: str) -> bool:
        node = (self.state.adopted_nodes.get(node_id)
                or self.state.unadopted_nodes.get(node_id))
        if not node:
            return False
        node.log_entries.clear()
        return True

    def log_node_event(self, node_id: str, message: str, level: str = "info",
                       peripheral: Optional[str] = None) -> None:
        """Public entry point for callers outside StateManager (e.g.
        server_node.py) that want to record a node-scoped event.

        ``peripheral`` is an optional tag (e.g. ``"roboclaw-1"``) so
        the Logs tab can render the originating peripheral as a column
        without having to parse it back out of the message text.
        Untagged callers stay backward-compatible.
        """
        self._log_activity(message, level, node_id=node_id, peripheral=peripheral)

    def _log_activity(self, message: str, level: str = "info",
                      node_id: Optional[str] = None,
                      peripheral: Optional[str] = None):
        """Log an activity event.

        If ``node_id`` is provided, the entry is also appended to that
        node's per-node ring buffer and broadcast on ``node_log/<id>``
        so the node-detail Logs tab sees it. ``peripheral`` is an
        optional tag stored alongside the entry — the Logs UI uses it
        to render the source column without text-parsing.
        """
        # Store in log buffer
        entry = {
            "time": time.time(),
            "text": message,
            "level": level,
        }
        if peripheral:
            entry["peripheral"] = peripheral
        self._log_entries.append(entry)

        # Trim to max size
        if len(self._log_entries) > self.MAX_LOG_ENTRIES:
            self._log_entries = self._log_entries[-self.MAX_LOG_ENTRIES:]

        # Broadcast to clients
        if self._activity_callback:
            self._activity_callback(message, level)

        # File sink: every activity event goes to the server-wide log,
        # and per-node events ALSO go to that node's log file. Done
        # after the broadcast so a slow disk can't delay the live UI.
        if self._activity_server_logger is not None:
            try:
                self._activity_server_logger.log(
                    _LEVEL_TO_PY.get(level, logging.INFO),
                    f"[{level}] {message}" if not node_id
                    else f"[{level}] [{node_id}] {message}",
                )
            except Exception:
                pass  # Never let log I/O kill the caller.
        if node_id and self._activity_per_node_logger is not None:
            try:
                self._activity_per_node_logger.log(
                    node_id, f"[{level}] {message}",
                    level=_LEVEL_TO_PY.get(level, logging.INFO),
                )
            except Exception:
                pass

        # Per-node log buffer + targeted broadcast
        if node_id:
            node = (self.state.adopted_nodes.get(node_id)
                    or self.state.unadopted_nodes.get(node_id))
            if node:
                node.log_entries.append(entry)
                if len(node.log_entries) > self.MAX_NODE_LOG_ENTRIES:
                    node.log_entries = node.log_entries[-self.MAX_NODE_LOG_ENTRIES:]
            if self._node_log_callback:
                self._node_log_callback(node_id, entry)

        # Also log to ROS2 logger
        if self.logger:
            # Use explicit level mapping to avoid ROS2 logger issues
            try:
                if level == "debug":
                    self.logger.debug(message)
                elif level == "warn" or level == "warning":
                    self.logger.warning(message)
                elif level == "error":
                    self.logger.error(message)
                else:
                    self.logger.info(message)
            except ValueError:
                # ROS2 logger can throw errors in certain callback contexts
                pass

    def get_logs(self, limit: int = 100, level: Optional[str] = None) -> List[Dict[str, Any]]:
        """Get recent log entries.

        Args:
            limit: Maximum number of entries to return
            level: Optional filter by log level

        Returns:
            List of log entries (newest first)
        """
        logs = self._log_entries
        if level and level != 'all':
            logs = [e for e in logs if e['level'] == level]
        return list(reversed(logs[-limit:]))

    def update_node_from_announcement(self, announcement_json: str) -> bool:
        """
        Update or add an unadopted node from a JSON announcement.

        Expected JSON format:
        {
            "node_id": "rp2040_XXXX",
            "mac": "02:XX:XX:XX:XX:XX",
            "ip": "192.168.1.100",
            "hw": "Adafruit Feather RP2040",
            "fw": "0.5.0",
            "state": "UNADOPTED",
            "uptime": 120
        }

        Returns True if this is a new node, False if existing node updated.
        """
        try:
            data = json.loads(announcement_json)
        except json.JSONDecodeError as e:
            if self.logger:
                self.logger.warning(f"Invalid announcement JSON: {e}")
            return False

        node_id = data.get("node_id", "")
        if not node_id:
            return False

        # Check if this node is already adopted
        if node_id in self.state.adopted_nodes:
            node = self.state.adopted_nodes[node_id]
            is_new = False
        elif node_id in self.state.unadopted_nodes:
            node = self.state.unadopted_nodes[node_id]
            is_new = False
        else:
            # New unadopted node
            node = NodeInfo(node_id=node_id)
            self.state.unadopted_nodes[node_id] = node
            self._log_activity(f"New node discovered: {node_id}", "info", node_id=node_id)
            is_new = True

        # Capture pre-update values so we can log transitions below.
        prev_state = node.state
        prev_fw = node.firmware_version
        prev_bl_fw = node.bootloader_version
        prev_online = node.online

        # Update node info from announcement (for both adopted and unadopted)
        node.mac_address = data.get("mac", node.mac_address)
        node.ip_address = data.get("ip", node.ip_address)
        node.hardware_model = data.get("hw", node.hardware_model)
        node.firmware_version = data.get("fw", node.firmware_version)
        node.bootloader_version = data.get("bl_fw", node.bootloader_version)
        node.firmware_build = data.get("fw_build", node.firmware_build)
        node.uptime_seconds = data.get("uptime", node.uptime_seconds)
        node.cpu_temp = data.get("cpu_temp", node.cpu_temp)
        node.state = data.get("state", node.state)
        node.last_seen = time.time()
        # Chip family the firmware identifies itself as (set when the
        # firmware emits the new announcement format). Used to pick a
        # matching board YAML and to populate the chip-family dropdown
        # in the adopt dialog.
        if "chip_family" in data:
            node.chip_family = str(data["chip_family"])

        # State transition — interesting for the Logs tab. Skip the
        # initial UNKNOWN → first-reported transition on a brand-new
        # node so we don't double-log alongside "New node discovered".
        if (prev_state and prev_state != "UNKNOWN"
                and prev_state != node.state):
            self._log_activity(
                f"State: {prev_state} → {node.state}", "info", node_id=node_id)

        # Firmware version changed — most likely a successful OTA.
        if (prev_fw and prev_fw != "0.0.0"
                and prev_fw != node.firmware_version):
            self._log_activity(
                f"Firmware updated: {prev_fw} → {node.firmware_version}",
                "info", node_id=node_id)

        # Bootloader version changed — can only happen via a physical
        # BOOTSEL reflash since the bootloader isn't OTA-updatable. Worth
        # logging so the operator has a clear record of which boards got
        # touched on a reflash sweep.
        if (prev_bl_fw and prev_bl_fw != "unknown"
                and prev_bl_fw != node.bootloader_version
                and node.bootloader_version != "unknown"):
            self._log_activity(
                f"Bootloader changed: {prev_bl_fw} → {node.bootloader_version}",
                "info", node_id=node_id)

        # Per-peripheral connection transitions out of the announcement.
        # The firmware ships a `peripherals: {id: bool}` map. We log
        # each connect↔disconnect flip once.
        peripherals = data.get("peripherals")
        if isinstance(peripherals, dict):
            for pid, val in peripherals.items():
                connected = bool(val)
                prev = node.peripheral_connected.get(pid)
                if prev is not None and prev != connected:
                    self._log_activity(
                        f"Peripheral {pid}: {'connected' if connected else 'disconnected'}",
                        "info" if connected else "warn", node_id=node_id)
                    # A peripheral that just re-enumerated does not hold
                    # what we last sent it — a Maestro drives every
                    # channel to its configured home on connect. Drop
                    # the cached channel state for this node so the next
                    # command is never suppressed as "already there".
                    # Coarse (whole node) on purpose: the announcement
                    # keys are driver types, not instance ids, and these
                    # transitions are rare.
                    self._invalidate_channel_cache(
                        node_id, f"peripheral {pid} re-enumerated")
                node.peripheral_connected[pid] = connected

        # Clear any pending offline state and mark online
        node.online = True
        node.going_offline_at = None

        # Log reconnection
        if not prev_online:
            name = node.display_name or node.node_id
            self._log_activity(f"Node reconnected: {name}", "info", node_id=node_id)
            # Everything commanded before the gap may have been lost:
            # /control is best-effort, and a node that re-initialized
            # micro-ROS came back with its outputs at whatever the
            # firmware's boot/home state is. Forget what we think it
            # holds so the next write always goes out.
            self._invalidate_channel_cache(node_id, "node reconnected")

        return is_new

    def check_node_timeouts(self) -> List[str]:
        """
        Check for nodes that have timed out (stopped announcing).

        Uses a two-phase approach:
        1. When timeout is first detected, start a grace period
        2. Only mark offline after grace period expires (prevents flapping)

        Returns list of node_ids that went offline.
        """
        now = time.time()
        timed_out = []

        def check_node(node: NodeInfo, is_adopted: bool) -> bool:
            """Check a single node. Returns True if node went offline."""
            if not node.online:
                return False

            time_since_seen = now - node.last_seen

            if time_since_seen <= NODE_TIMEOUT_SECONDS:
                # Node is responding normally - clear any pending offline state
                node.going_offline_at = None
                return False

            # Node has exceeded timeout
            if node.going_offline_at is None:
                # First detection - start grace period
                node.going_offline_at = now
                if self.logger:
                    name = node.display_name or node.node_id
                    self.logger.debug(
                        f"Node {name} timeout detected, starting grace period"
                    )
                return False

            # Check if grace period has expired
            grace_elapsed = now - node.going_offline_at
            if grace_elapsed < NODE_OFFLINE_GRACE_SECONDS:
                return False

            # Grace period expired - mark offline
            node.online = False
            node.going_offline_at = None
            return True

        # Check unadopted nodes
        for node_id, node in list(self.state.unadopted_nodes.items()):
            if check_node(node, is_adopted=False):
                timed_out.append(node_id)
                self._log_activity(
                    f"Node offline: {node_id}", "warn", node_id=node_id)

        # Check adopted nodes — except the host controller, which has
        # no announcement loop and is always "online" by definition.
        for node_id, node in self.state.adopted_nodes.items():
            if node_id == HOST_CONTROLLER_NODE_ID:
                continue
            if check_node(node, is_adopted=True):
                timed_out.append(node_id)
                self._log_activity(
                    f"Adopted node offline: {node.display_name or node_id}",
                    "warn", node_id=node_id
                )

        return timed_out

    def remove_offline_unadopted_nodes(self, max_offline_seconds: float = 60.0) -> int:
        """
        Remove unadopted nodes that have been offline for too long.

        Returns count of removed nodes.
        """
        now = time.time()
        to_remove = []

        for node_id, node in self.state.unadopted_nodes.items():
            if not node.online and (now - node.last_seen) > max_offline_seconds:
                to_remove.append(node_id)

        for node_id in to_remove:
            del self.state.unadopted_nodes[node_id]
            self._log_activity(f"Removed stale node: {node_id}", "debug")

        return len(to_remove)

    def get_system_status(self) -> Dict[str, Any]:
        """Get current system status for broadcasting."""
        try:
            cpu_usage = psutil.cpu_percent(interval=None)
            memory = psutil.virtual_memory()
            memory_usage = memory.percent
        except Exception:
            cpu_usage = 0.0
            memory_usage = 0.0

        cpu_temp = _read_cpu_temp()
        throttle = _read_throttle_status()
        version_info = _read_installed_version_info()

        # Get server firmware info
        fw_info = self.get_server_firmware_info()

        return {
            "server_online": self.state.server_online,
            "server_name": self.state.server_name,
            "server_version": version_info["version"],
            "server_git_sha": version_info["git_sha"],
            "server_built_at": version_info["built_at"],
            "uptime_seconds": int(time.time() - self.state.start_time),
            "cpu_usage": cpu_usage,
            "cpu_temp_c": cpu_temp,
            "throttle": throttle,
            "memory_usage": memory_usage,
            "adopted_node_count": len(self.state.adopted_nodes),
            "unadopted_node_count": len(self.state.unadopted_nodes),
            "websocket_client_count": self.state.websocket_client_count,
            "livelink_enabled": self.state.livelink_enabled,
            "livelink_source_count": self.state.livelink_source_count,
            "rc_enabled": self.state.rc_enabled,
            "rc_connected": self.state.rc_connected,
            # Node firmware info
            "node_firmware_version": fw_info.get("version_full") or fw_info.get("version"),
            "node_firmware_hash": fw_info.get("git_hash"),
            "node_firmware_build": fw_info.get("build_date"),
            "node_firmware_path": fw_info.get("elf_path"),
            "node_firmware_available": fw_info.get("available", False),
        }

    def get_adopted_nodes(self) -> List[Dict[str, Any]]:
        """Get list of adopted nodes."""
        result = []

        for node in self.state.adopted_nodes.values():
            # Host controller isn't an RP2040/Teensy firmware target —
            # it has no .uf2/.hex update flow, so skip the comparison.
            if node.node_id == HOST_CONTROLLER_NODE_ID:
                fw_update_info = {
                    "available": False,
                    "message": "Host controller — runs from the server install",
                }
                # Use the server's own build info on the host controller —
                # it's running from the install and there's no per-chip
                # firmware bundle for it.
                server_fw = self.get_server_firmware_info()
            else:
                fw_update_info = self.is_firmware_update_available(node.node_id)
                # Resolve the SERVER-SIDE firmware info using the node's
                # chip family, NOT the global RP2040 walker. This used
                # to be a single get_server_firmware_info() call hoisted
                # above the loop, which meant every node's
                # server_firmware_version was the RP2040 build — Pi
                # nodes on the Nodes-view page got tagged with the
                # RP2040 version pill like "1.2.0-<unix_ts>", and
                # Teensy nodes the same. Now each node sees its own
                # chip-family's staged build. Mirrors the dispatch in
                # is_firmware_update_available so the two fields stay
                # in sync.
                chip = (node.chip_family or '').lower() or None
                if chip and chip in ('rp2040', 'teensy41', 'raspberrypi'):
                    server_fw = self.get_firmware_info_for_type(chip)
                else:
                    server_fw = self.get_server_firmware_info()

            node_data = {
                "node_id": node.node_id,
                "display_name": node.display_name,
                "role": node.role,
                "hardware_model": node.hardware_model,
                "ip_address": node.ip_address,
                "mac_address": node.mac_address,
                "firmware_version": node.firmware_version,
                "bootloader_version": node.bootloader_version,
                "firmware_build": node.firmware_build,
                "online": node.online,
                "cpu_temp": node.cpu_temp,
                "cpu_usage": node.cpu_usage,
                "memory_usage": node.memory_usage,
                "uptime_seconds": node.uptime_seconds,
                "state": node.state,
                "last_seen": node.last_seen,
                "has_capabilities": node.capabilities is not None or bool(node.board_id),
                "chip_family": node.chip_family,
                "board_id": node.board_id,
                "peripheral_count": len(node.peripheral_config.peripherals) if node.peripheral_config else 0,
                "peripheral_sync_status": (
                    node.peripheral_config.sync_status if node.peripheral_config else "unconfigured"
                ),
                "firmware_update_available": fw_update_info["available"],
                "firmware_update_message": fw_update_info["message"],
                "server_firmware_version": server_fw.get("version_full") or server_fw.get("version"),
                "server_firmware_build": server_fw.get("build_date"),
                "server_firmware_hash": server_fw.get("file_hash"),
            }
            result.append(node_data)
        return result

    def get_unadopted_nodes(self) -> List[Dict[str, Any]]:
        """Get list of unadopted nodes."""
        return [
            {
                "node_id": node.node_id,
                "hardware_model": node.hardware_model,
                "mac_address": node.mac_address,
                "ip_address": node.ip_address,
                "firmware_version": node.firmware_version,
                "bootloader_version": node.bootloader_version,
                "firmware_build": node.firmware_build,
                "cpu_temp": node.cpu_temp,
                "cpu_usage": node.cpu_usage,
                "memory_usage": node.memory_usage,
                "uptime_seconds": node.uptime_seconds,
                "state": node.state,
                "last_seen": node.last_seen,
                "online": node.online,
                # Chip family the firmware announced — populates the
                # board picker on the Adopt dialog.
                "chip_family": node.chip_family,
            }
            for node in self.state.unadopted_nodes.values()
        ]

    def adopt_node(self, node_id: str, role: str = "",
                    display_name: Optional[str] = None,
                    board_id: Optional[str] = None,
                    chip_family: Optional[str] = None) -> Dict[str, Any]:
        """Adopt an unadopted node, assigning it a role + board.

        The operator picks the board_id (and optionally the chip_family)
        from the Adopt dialog; the server uses board_id to derive the
        node's pin layout. When the operator picks a board, the
        operator's choice wins — node.chip_family is overridden by the
        board's chip_family. That's what lets us still adopt a node
        whose firmware reported chip_family="unknown" (e.g. a chip ID
        sanity-check mismatch). If board_id is omitted, defaults to
        the first matching board for the node's chip family.
        """
        if node_id not in self.state.unadopted_nodes:
            return {"success": False, "message": f"Node {node_id} not found"}

        node = self.state.unadopted_nodes.pop(node_id)
        # role is now an optional human label chosen from the active
        # robot manifest's role list; the node's display_name is its
        # real identity. Fall back to a role-derived name, then node_id.
        node.role = role or ""
        node.display_name = display_name or (f"{role} Node" if role else node_id)
        node.online = True

        # Operator's explicit chip choice wins over the firmware-announced
        # value. Useful when the firmware can't recognize its own silicon
        # (e.g. announced "unknown") but the operator knows what board
        # they're plugging in.
        if chip_family:
            node.chip_family = chip_family

        # Resolve board: explicit > default for chip > none (operator can fix later).
        if board_id:
            if not self.board_config.get_board(board_id):
                # Put the node back in unadopted so the operator can retry.
                self.state.unadopted_nodes[node_id] = node
                return {"success": False, "message": f"Unknown board_id '{board_id}'"}
            board = self.board_config.get_board(board_id)
            node.board_id = board_id
            # Operator's board pick is authoritative — sync chip_family to
            # whatever the board declares. This unblocks adoption of nodes
            # whose firmware announced an unknown / mismatched chip.
            node.chip_family = board.chip_family
        elif node.chip_family:
            default_board = self.board_config.default_board_for_chip(node.chip_family)
            if default_board:
                node.board_id = default_board.board_id
                if self.logger:
                    self.logger.info(
                        f"adopt_node({node_id}): no board_id given, defaulted "
                        f"to '{node.board_id}' (chip {node.chip_family})"
                    )

        self.state.adopted_nodes[node_id] = node

        # Load any existing peripheral configuration from disk
        self._load_node_config(node_id)

        # Ensure an empty peripheral_config exists (so callers don't see None)
        if not node.peripheral_config:
            node.peripheral_config = NodePeripheralConfig()

        # Seed built-in peripherals declared by the board YAML.
        self._seed_builtin_peripherals_from_board(node)

        # Save initial config
        self._save_node_config(node_id)

        self._log_activity(
            f"Adopted as '{node.display_name or node_id}' "
            f"(role={role}, board={node.board_id or 'default'})",
            "info", node_id=node_id)

        topic_prefix = f"/saint/{role}"
        return {
            "success": True,
            "message": "Node adopted successfully",
            "assigned_topic_prefix": topic_prefix,
            "board_id": node.board_id,
        }

    def list_chips(self) -> List[Dict[str, Any]]:
        """JSON-friendly list of known chip families (for the UI)."""
        return [c.to_dict() for c in self.board_config.list_chips()]

    def list_boards(self, chip_family: Optional[str] = None) -> List[Dict[str, Any]]:
        """JSON-friendly list of known boards, optionally filtered by chip."""
        boards = (self.board_config.get_boards_for_chip(chip_family)
                  if chip_family else self.board_config.list_boards())
        return [b.to_dict() for b in boards]

    def get_board_yaml(self, board_id: str) -> Optional[str]:
        """Read the raw YAML text for a board (for the Settings editor)."""
        return self.board_config.get_board_yaml_text(board_id)

    def save_board_yaml(self, yaml_text: str) -> Dict[str, Any]:
        """Persist an operator-authored board. Forbids built-in overwrites."""
        return self.board_config.save_operator_board(yaml_text)

    def delete_board(self, board_id: str) -> Dict[str, Any]:
        """Delete an operator-authored board. Forbids built-in deletion."""
        return self.board_config.delete_operator_board(board_id)

    def update_node(self, node_id: str, *,
                    role: Optional[str] = None,
                    display_name: Optional[str] = None,
                    board_id: Optional[str] = None,
                    chip_family: Optional[str] = None) -> Dict[str, Any]:
        """Edit an already-adopted node's metadata.

        Symmetric counterpart to ``adopt_node`` for nodes that are
        already in ``adopted_nodes`` — lets the operator fix role,
        board, chip, or display name after adoption without going
        through factory-reset → re-adopt (which would also wipe
        peripheral configs, a heavy hammer when all you want to do
        is correct a typo).

        Any subset of fields may be passed; unspecified fields (None)
        are left as-is. Passing the same value as already set is a
        no-op for that field. The same board/chip authority rule as
        adopt_node applies: an explicit board choice overrides
        chip_family to match the board, since the board YAML defines
        the chip — letting these diverge would silently break the
        pin layout. Setting board re-seeds builtin peripherals from
        the new board's YAML, the same way adopt_node does.
        """
        node = self.state.adopted_nodes.get(node_id)
        if not node:
            return {"success": False, "message": f"Node {node_id} not found"}

        changes: List[str] = []

        if role is not None and role != node.role:
            node.role = role
            changes.append(f"role={role}")

        if display_name is not None and display_name != (node.display_name or ""):
            node.display_name = display_name
            changes.append(f"display_name={display_name!r}")

        if chip_family is not None and chip_family != node.chip_family:
            node.chip_family = chip_family
            changes.append(f"chip_family={chip_family}")

        if board_id is not None and board_id != node.board_id:
            board = self.board_config.get_board(board_id)
            if not board:
                return {"success": False,
                        "message": f"Unknown board_id '{board_id}'"}
            node.board_id = board_id
            # Operator's board pick is authoritative — same rule as
            # adopt_node. Don't allow chip_family / board.chip_family
            # to diverge.
            node.chip_family = board.chip_family
            self._seed_builtin_peripherals_from_board(node)
            changes.append(f"board_id={board_id}")

        if not changes:
            return {"success": True, "message": "No changes",
                    "node_id": node_id,
                    "role": node.role,
                    "display_name": node.display_name,
                    "board_id": node.board_id,
                    "chip_family": node.chip_family}

        self._save_node_config(node_id)
        self._log_activity(
            f"Updated: {', '.join(changes)}", "info", node_id=node_id,
        )
        return {
            "success": True,
            "message": "Node updated",
            "node_id": node_id,
            "role": node.role,
            "display_name": node.display_name,
            "board_id": node.board_id,
            "chip_family": node.chip_family,
        }

    def set_node_board(self, node_id: str, board_id: str) -> Dict[str, Any]:
        """Change which board a node is assigned to. Re-derives capabilities."""
        node = self.state.adopted_nodes.get(node_id)
        if not node:
            return {"success": False, "message": f"Node {node_id} not found"}
        board = self.board_config.get_board(board_id)
        if not board:
            return {"success": False, "message": f"Unknown board_id '{board_id}'"}
        if node.chip_family and board.chip_family != node.chip_family:
            return {"success": False,
                    "message": f"Board '{board_id}' is for chip '{board.chip_family}' "
                               f"but node reports '{node.chip_family}'"}
        node.board_id = board_id
        if not node.chip_family:
            node.chip_family = board.chip_family
        self._seed_builtin_peripherals_from_board(node)
        self._save_node_config(node_id)
        self._log_activity(
            f"Node {node.display_name or node_id}: board set to '{board_id}'", "info",
            node_id=node_id
        )
        return {"success": True, "board_id": board_id}

    def refresh_node_builtins(self, node_id: str) -> Dict[str, Any]:
        """Re-seed builtin_peripherals from the current board YAML for
        an already-adopted node, without changing role/board/chip.

        Adoption time + ``set_node_board`` are the only existing paths
        that seed builtin peripherals into a node's config. When the
        operator edits a board's YAML to add a new builtin (e.g. the
        onboard LED), already-adopted nodes assigned to that board
        miss the update — their saved peripheral_config was frozen at
        adoption time. The traditional workarounds were factory-reset
        + re-adopt (destructive) or hand-editing the YAML on the
        server (ops-only). This action wires up a non-destructive UI
        path: "Refresh from board" pulls the latest YAML, idempotently
        seeds anything missing, and saves.

        Returns the new + previously-present built-in IDs so the UI
        can highlight what just got added. Idempotent — calling it
        twice in a row is a no-op the second time.
        """
        node = self.state.adopted_nodes.get(node_id)
        if not node:
            return {"success": False, "message": f"Node {node_id} not found"}
        if not node.board_id:
            return {"success": False,
                    "message": f"Node {node_id} has no board assigned"}

        before_ids = (set(p.id for p in node.peripheral_config.peripherals)
                      if node.peripheral_config else set())
        self._seed_builtin_peripherals_from_board(node)
        after_ids = (set(p.id for p in node.peripheral_config.peripherals)
                     if node.peripheral_config else set())
        added = sorted(after_ids - before_ids)

        if added:
            self._save_node_config(node_id)
            self._log_activity(
                f"Refreshed built-ins from board '{node.board_id}': "
                f"added {', '.join(added)}",
                "info", node_id=node_id,
            )
        return {
            "success": True,
            "node_id": node_id,
            "board_id": node.board_id,
            "added": added,
            "already_present": sorted(after_ids & before_ids),
        }

    def _seed_builtin_peripherals_from_board(self, node: NodeInfo) -> None:
        """Apply builtin_peripherals from the node's board YAML.

        This used to happen via update_node_capabilities when the
        firmware emitted them. Now the source is the board YAML so the
        seeding has to happen here (and again on board change). Idempotent.
        """
        if not node.board_id:
            return
        board = self.board_config.get_board(node.board_id)
        if not board:
            return
        if not node.peripheral_config:
            node.peripheral_config = NodePeripheralConfig()
        for entry in board.builtin_peripherals:
            if entry.type not in self.peripheral_catalog:
                continue
            if node.peripheral_config.get(entry.id):
                continue   # already seeded
            node.peripheral_config.peripherals.append(PeripheralInstance(
                id=entry.id,
                type=entry.type,
                label=entry.label or entry.id,
                pins=dict(entry.pins),
                params=dict(entry.params),
                builtin=True,
            ))
            node.peripheral_config.version += 1

    def remove_node(self, node_id: str) -> Dict[str, Any]:
        """Remove a node completely from the server (both adopted and unadopted)."""
        if node_id == HOST_CONTROLLER_NODE_ID:
            return {"success": False,
                    "message": "Host controller is built-in and cannot be removed"}
        removed_from = None

        # Check adopted nodes
        if node_id in self.state.adopted_nodes:
            del self.state.adopted_nodes[node_id]
            removed_from = "adopted"

            # Also remove config file if it exists
            config_path = os.path.join(self.nodes_config_dir, f"{node_id}.yaml")
            if os.path.exists(config_path):
                try:
                    os.remove(config_path)
                except Exception as e:
                    if self.logger:
                        self.logger.warning(f"Failed to remove config file for {node_id}: {e}")

        # Check unadopted nodes
        elif node_id in self.state.unadopted_nodes:
            del self.state.unadopted_nodes[node_id]
            removed_from = "unadopted"

        if removed_from:
            self._log_activity(f"Removed node {node_id} from {removed_from} list", "info")
            return {"success": True, "message": f"Node {node_id} removed"}
        else:
            return {"success": False, "message": f"Node {node_id} not found"}

    def set_client_count(self, count: int):
        """Update WebSocket client count."""
        self.state.websocket_client_count = count

    # =========================================================================
    # Pin Configuration Methods
    # =========================================================================

    def update_node_capabilities(self, node_id: str, capabilities_json: str) -> bool:
        """
        Update node capabilities from JSON received from firmware.

        Expected JSON format:
        {
            "node_id": "rp2040_XXXX",
            "pins": [
                {"gpio": 5, "name": "D5", "capabilities": ["digital_in", "digital_out", "pwm"]},
                ...
            ],
            "reserved_pins": [10, 11, 16, 18, 19, 20]
        }
        """
        try:
            data = json.loads(capabilities_json)
        except json.JSONDecodeError as e:
            if self.logger:
                self.logger.warning(f"Invalid capabilities JSON: {e}")
            return False

        # Find the node (in adopted or unadopted)
        node = self.state.adopted_nodes.get(node_id) or self.state.unadopted_nodes.get(node_id)
        if not node:
            if self.logger:
                self.logger.warning(f"Node {node_id} not found for capabilities update")
                self.logger.warning(f"Available adopted nodes: {list(self.state.adopted_nodes.keys())}")
                self.logger.warning(f"Available unadopted nodes: {list(self.state.unadopted_nodes.keys())}")
            return False

        # Parse capabilities
        pins = []
        for pin_data in data.get('pins', []):
            pins.append(PinCapability(
                gpio=pin_data.get('gpio', 0),
                name=pin_data.get('name', ''),
                capabilities=pin_data.get('capabilities', []),
            ))

        node.capabilities = NodeCapabilities(
            node_id=node_id,
            pins=pins,
            reserved_pins=data.get('reserved_pins', []),
            uart_pairs=data.get('uart_pairs', []),
            last_updated=time.time(),
        )

        # Seed any built-in peripherals declared by the firmware.
        # Firmware capability JSON may include:
        #   "builtin_peripherals": [
        #     {"id": "onboard_neopixel", "type": "neopixel", "label": "Onboard NeoPixel",
        #      "pins": {"data": 16}, "params": {}}
        #   ]
        # We add them to peripheral_config so they're routable like any other
        # peripheral. Existing built-ins keep their operator-set label/params.
        builtins = data.get('builtin_peripherals', [])
        if builtins:
            if not node.peripheral_config:
                node.peripheral_config = NodePeripheralConfig()
            seeded_any = False
            for entry in builtins:
                pid = entry.get('id')
                tid = entry.get('type')
                if not pid or tid not in self.peripheral_catalog:
                    continue
                existing = node.peripheral_config.get(pid)
                if existing:
                    continue
                node.peripheral_config.peripherals.append(PeripheralInstance(
                    id=pid,
                    type=tid,
                    label=entry.get('label', tid),
                    pins=dict(entry.get('pins', {})),
                    params=dict(entry.get('params', {})),
                    builtin=True,
                ))
                seeded_any = True
            if seeded_any:
                node.peripheral_config.version += 1
                if node_id in self.state.adopted_nodes:
                    self._save_node_config(node_id)

        if self.logger:
            self.logger.info(f"Stored {len(pins)} pin capabilities for node {node_id}")
        self._log_activity(
            f"Updated capabilities ({len(pins)} pins)", "info", node_id=node_id)
        return True

    def get_node(self, node_id: str) -> Optional[Dict[str, Any]]:
        """Get node info by ID (adopted or unadopted)."""
        node = self.state.adopted_nodes.get(node_id) or self.state.unadopted_nodes.get(node_id)
        if not node:
            return None
        return {
            "node_id": node.node_id,
            "hw": node.hardware_model,
            "fw": node.firmware_version,
            "bl_fw": node.bootloader_version,
            "ip": node.ip_address,
            "mac": node.mac_address,
            "display_name": node.display_name,
            "role": node.role,
            "online": node.online,
        }

    def get_node_capabilities(self, node_id: str) -> Optional[Dict[str, Any]]:
        """Get capabilities for a node.

        Preferred path: when the node has a board_id assigned, the
        capability view is *derived* from the chip + board YAML on the
        server. No round-trip to the firmware needed — the operator
        sees pins immediately on adoption.

        Fallback: if board_id isn't set yet (truly fresh node before
        adoption picks a board), fall back to whatever capability JSON
        the firmware reported. With the new flow that should be empty,
        but it keeps old firmware on existing nodes working.
        """
        node = self.state.adopted_nodes.get(node_id) or self.state.unadopted_nodes.get(node_id)
        if not node:
            return None

        # YAML-derived path
        if node.board_id:
            board = self.board_config.get_board(node.board_id)
            chip = self.board_config.get_chip(node.chip_family) if board else None
            if board and chip:
                view = derive_capabilities(chip, board)
                view["node_id"] = node.node_id
                return view

        # Firmware-emitted fallback (old flow)
        if not node.capabilities:
            return None
        return node.capabilities.to_dict()

    # =========================================================================
    # Peripheral catalog (types of peripherals known to the system)
    # =========================================================================

    def get_peripheral_catalog(self) -> List[Dict[str, Any]]:
        """Return the list of known peripheral types, JSON-serializable."""
        return [t.to_dict() for t in self.peripheral_catalog.values()]

    def get_widget_catalog(self) -> List[Dict[str, Any]]:
        """Return the list of known widget types, JSON-serializable."""
        return [t.to_dict() for t in self.widget_catalog.values()]

    def get_operator_catalog(self) -> List[Dict[str, Any]]:
        """Return the list of routing-graph operators (Max/Min/Clamp/…)."""
        return [t.to_dict() for t in OPERATOR_CATALOG.values()]

    # =========================================================================
    # Per-node peripherals (the things attached to a node)
    # =========================================================================

    def get_node_peripherals(self, node_id: str) -> Optional[Dict[str, Any]]:
        """Return per-node peripheral list for a node, with sync state."""
        node = self.state.adopted_nodes.get(node_id)
        if not node:
            return None
        if not node.peripheral_config:
            node.peripheral_config = NodePeripheralConfig()
        return node.peripheral_config.to_dict()

    def lookup_channel(self, node_id: str, peripheral_id: str,
                       channel_id: str) -> Optional[Dict[str, Any]]:
        """Look up a (node, peripheral, channel) in the catalog.

        Returns the channel's direction + capability so the WS handler
        can validate a write request. Addressing is operator-visible
        names end-to-end — no GPIO translation happens here.
        """
        node = self.state.adopted_nodes.get(node_id)
        if not node or not node.peripheral_config:
            return None
        peripheral = node.peripheral_config.get(peripheral_id)
        if not peripheral:
            return None
        ptype = self.peripheral_catalog.get(peripheral.type)
        if not ptype:
            return None
        channel = next((c for c in ptype.channels if c.id == channel_id), None)
        if not channel:
            return None
        return {
            "direction": channel.dir,
            "capability": channel.cap,
            "peripheral_type": peripheral.type,
        }

    def channel_idle_disengage_ms(self, node_id: str, peripheral_id: str,
                                  channel_id: str) -> int:
        """Per-channel idle-disengage window (ms) for a Maestro channel,
        or 0 if none / not applicable.

        The firmware releases PWM on a channel after this many ms with no
        SET_TARGET (servo goes limp). The control change-filter uses this
        to expire its "identical value already sent" dedupe once the
        channel has likely disengaged, so a repeat of the held value
        (State slider re-touch, pose) re-engages it instead of being
        silently swallowed. 0 = always-on / non-Maestro → dedupe stands."""
        node = self.state.adopted_nodes.get(node_id)
        if not node or not node.peripheral_config:
            return 0
        peripheral = node.peripheral_config.get(peripheral_id)
        if not peripheral:
            return 0
        channels = (peripheral.params or {}).get("channels")
        if not isinstance(channels, list):
            return 0
        # Maestro channel ids are "ch0".."ch23" (see peripheral_model
        # catalog); params["channels"] is indexed by the bare number.
        cid = str(channel_id)
        if cid.startswith("ch"):
            cid = cid[2:]
        try:
            idx = int(cid)
        except (TypeError, ValueError):
            return 0
        if 0 <= idx < len(channels) and isinstance(channels[idx], dict):
            try:
                return max(0, int(channels[idx].get("idle_disengage_ms", 0)))
            except (TypeError, ValueError):
                return 0
        return 0

    def upsert_node_peripheral(
        self, node_id: str, peripheral_payload: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Add or replace a peripheral on a node.

        If `id` is missing in the payload, a new id is generated. If
        `id` matches an existing peripheral, that entry is replaced.
        Built-in peripherals can be edited (label/params) but their
        builtin flag and pins are preserved.
        """
        node = self.state.adopted_nodes.get(node_id)
        if not node:
            return {"success": False, "message": f"Node {node_id} not found or not adopted"}

        type_id = peripheral_payload.get("type")
        if type_id not in self.peripheral_catalog:
            return {"success": False, "message": f"Unknown peripheral type: {type_id}"}

        if not node.peripheral_config:
            node.peripheral_config = NodePeripheralConfig()

        pid = peripheral_payload.get("id") or self._generate_peripheral_id(node, type_id)
        existing = node.peripheral_config.get(pid)
        # Preserve log_enabled across upserts unless the payload sets it
        # explicitly — the typical config-edit path (label/params) should
        # not flip logging off.
        prev_log_enabled = existing.log_enabled if existing else False
        log_enabled = bool(peripheral_payload.get("log_enabled", prev_log_enabled))
        if existing and existing.builtin:
            # Preserve hardwired pins + builtin flag for built-in peripherals
            peripheral = PeripheralInstance(
                id=pid,
                type=existing.type,
                label=peripheral_payload.get("label", existing.label),
                pins=dict(existing.pins),
                params=dict(peripheral_payload.get("params", existing.params)),
                builtin=True,
                log_enabled=log_enabled,
            )
        else:
            peripheral = PeripheralInstance(
                id=pid,
                type=type_id,
                label=peripheral_payload.get("label", type_id),
                pins=dict(peripheral_payload.get("pins", {})),
                params=dict(peripheral_payload.get("params", {})),
                builtin=False,
                log_enabled=log_enabled,
            )

        # Maestro: ensure params["channels"] exists and matches
        # channel_count. Fresh adds get default per-channel entries
        # sourced from the peripheral-level Advanced fields; existing
        # entries are sanitized + clamped. See _maestro_sanitize_channel
        # in peripheral_model.py.
        if peripheral.type == "maestro":
            maestro_normalize_channels(peripheral.params)

        # Pimoroni Servo 2040: normalize the 18-entry per-channel extents
        # list (same rationale as Maestro above).
        if peripheral.type == "pimoroni_servo2040":
            pimoroni_normalize_channels(peripheral.params)

        # Validate pin assignments don't conflict (call out to peripheral_model)
        node.peripheral_config.upsert(peripheral)
        conflicts = detect_pin_conflicts(
            node.peripheral_config,
            uart_pairs=node.capabilities.uart_pairs if node.capabilities else None,
            catalog=self.peripheral_catalog,
        )
        if conflicts:
            # Roll back the upsert if conflicts found
            node.peripheral_config.remove(pid) if not existing else node.peripheral_config.upsert(existing)
            return {"success": False, "message": "; ".join(conflicts)}

        self._save_node_config(node_id)
        # Keep the logger's enabled set in sync with the persisted flag.
        if self.peripheral_logger is not None:
            self.peripheral_logger.set_enabled(node_id, pid, peripheral.log_enabled)
        self._log_activity(
            f"Saved peripheral '{peripheral.label}'", "info", node_id=node_id
        )
        self._maybe_notify_host_peripheral_change(node_id)
        return {
            "success": True,
            "peripheral": peripheral.to_dict(),
            "version": node.peripheral_config.version,
        }

    def set_peripheral_log_enabled(self, node_id: str, peripheral_id: str,
                                   enabled: bool) -> Dict[str, Any]:
        """Toggle the log_enabled flag for one peripheral and persist.

        Lighter-weight than upsert_node_peripheral — doesn't re-run pin
        validation and doesn't cascade to routes; just flips the flag,
        saves, and updates the logger.
        """
        node = self.state.adopted_nodes.get(node_id)
        if not node or not node.peripheral_config:
            return {"success": False, "message": "Node has no peripherals"}
        peripheral = node.peripheral_config.get(peripheral_id)
        if not peripheral:
            return {"success": False, "message": f"Peripheral {peripheral_id} not found"}
        peripheral.log_enabled = bool(enabled)
        self._save_node_config(node_id)
        if self.peripheral_logger is not None:
            self.peripheral_logger.set_enabled(node_id, peripheral_id, peripheral.log_enabled)
        return {"success": True, "log_enabled": peripheral.log_enabled}

    def remove_node_peripheral(self, node_id: str, peripheral_id: str) -> Dict[str, Any]:
        """Remove a peripheral from a node. Cascades: drops routes touching it."""
        node = self.state.adopted_nodes.get(node_id)
        if not node or not node.peripheral_config:
            return {"success": False, "message": "Node has no peripherals"}

        if not node.peripheral_config.remove(peripheral_id):
            return {"success": False, "message": f"Peripheral {peripheral_id} not found"}

        # Cascade: drop any wires referencing this peripheral across every sheet.
        dropped = self.state.system_routing.drop_wires_touching_peripheral(node_id, peripheral_id)
        if dropped:
            self._save_system_routing()

        self._save_node_config(node_id)
        # Drop the in-memory buffers; deleting a peripheral implies the
        # historical samples are no longer addressable.
        if self.peripheral_logger is not None:
            self.peripheral_logger.set_enabled(node_id, peripheral_id, False)
        self._log_activity(
            f"Removed peripheral {peripheral_id}", "info", node_id=node_id
        )
        self._maybe_notify_host_peripheral_change(node_id)
        return {"success": True}

    def mark_node_synced(self, node_id: str, success: bool = True) -> None:
        """Mark a node's peripheral config as synced or errored after firmware ack."""
        node = self.state.adopted_nodes.get(node_id)
        if node and node.peripheral_config:
            node.peripheral_config.sync_status = "synced" if success else "error"
            node.peripheral_config.last_synced = time.time() if success else None
            self._save_node_config(node_id)

    def get_firmware_config_json(self, node_id: str) -> Optional[str]:
        """Build the JSON payload the firmware expects to (re)configure itself.

        Format:
            {"action": "configure", "version": N,
             "peripherals": [{"id": ..., "type": ..., "pins": {...}, "params": {...}}, ...]}
        Built-in peripherals are omitted — firmware already knows about them.
        """
        node = self.state.adopted_nodes.get(node_id)
        if not node or not node.peripheral_config:
            return None
        peripherals_out = []
        for p in node.peripheral_config.peripherals:
            if p.builtin:
                continue
            params = dict(p.params)
            # console_display: inject the server's kiosk token into the
            # config push so the Pi driver can build a passwordless
            # kiosk URL. The operator never types or sees this; it
            # rides as a private param. None means kiosk auth is
            # unconfigured (no password gate either, hopefully).
            if p.type == "console_display":
                try:
                    from saint_server.config import get_config
                    tok = get_config().websocket.kiosk_token
                    if tok:
                        params["_kiosk_token"] = tok
                except Exception:
                    pass
                # server_url: the kiosk browser loads the dashboard from
                # the SERVER, not from the console Pi — so "localhost"
                # only works in the rare case the Pi IS the server. When
                # the operator leaves it blank (or on the stale
                # localhost default), fill in the server's own reachable
                # address + web port so the kiosk just works without the
                # operator knowing the IP. Mirrors the _kiosk_token
                # injection above. An explicit operator value is left
                # untouched so a custom hostname (e.g. opensaint.local)
                # still wins.
                _su = (params.get("server_url") or "").strip().rstrip("/")
                if not _su or _su in ("http://localhost:8080",
                                      "https://localhost:8080"):
                    try:
                        from saint_server.config import get_config
                        ip = _resolve_server_ip()
                        port = int(getattr(get_config().network,
                                           "web_port", 80) or 80)
                        if ip:
                            params["server_url"] = (
                                f"http://{ip}" if port == 80
                                else f"http://{ip}:{port}")
                    except Exception:
                        pass
            # Maestro: slim default-equal channel entries down to {}
            # on the wire so the full 24-channel array doesn't blow the
            # firmware's config_buffer (2048-4096) or the XRCE-DDS
            # fragmentation budget (UDP MTU 512 × MAX_HISTORY 4 ≈ 2 KB
            # reassembly). The firmware parser brace-counts the array,
            # so an empty {} still occupies the channel's slot and
            # falls back to peripheral-level defaults — exactly the
            # behavior "all default" means. See peripheral_model
            # maestro_slim_channels_for_wire and
            # docs/MAESTRO_BRINGUP.md for the wire-size analysis.
            if p.type == "maestro":
                params = maestro_slim_channels_for_wire(params)
            # Pimoroni Servo 2040: same slim trick for its 18-entry
            # per-channel extents array.
            if p.type == "pimoroni_servo2040":
                params = pimoroni_slim_channels_for_wire(params)
            # Kangaroo: no per-channel array, but KANGAROO_MAX_UNITS is 8
            # and the linear-actuator params are acted on server-side, so
            # drop those rather than spend the XRCE budget on them.
            if p.type == "kangaroo":
                params = kangaroo_slim_params_for_wire(params)
            # switch_input: the operator types interlock targets as a
            # comma-separated string; the firmware parses a JSON array.
            if p.type == "switch_input":
                params = switch_input_params_for_wire(params)
            # Last step before the wire: drop params no firmware reads.
            # The config push has a hard ~2048-byte ceiling and every
            # byte spent on a value no driver looks at is a byte closer
            # to the overrun that watchdogs the node.
            params = strip_server_only_params(p.type, params)
            peripherals_out.append({
                "id": p.id,
                "type": p.type,
                "pins": p.pins,
                "params": params,
            })
        payload = {
            "action": "configure",
            "version": node.peripheral_config.version,
            "peripherals": peripherals_out,
        }
        # Compact separators (no spaces after , or :) shave ~300 bytes
        # off a 24-channel Maestro + 2 NeoPixel push. Even after
        # maestro_slim_channels_for_wire, the default `json.dumps`
        # spacing was pushing us to 2027 bytes — under the 2048 XRCE
        # reassembly cap on paper but over it once XRCE submessage
        # headers + stream-framing overhead get added on the wire. The
        # firmware silently dropped messages it couldn't reassemble,
        # which looked exactly like "Teensy in a reboot loop" because
        # the server kept seeing UNADOPTED and re-pushing. The firmware
        # JSON parser handles either spacing.
        # Config sync tag: a CRC32 over the payload the node is about
        # to receive. The node stores it, echoes it in /announce, and
        # the server pushes only when it differs from the tag its
        # CURRENT config would carry.
        #
        # Deriving it from the content rather than issuing a random
        # token keeps the server stateless — it can recompute what it
        # expects at any time, so a server restart doesn't trigger a
        # resync storm, and two servers built from the same config agree.
        # Computed over the payload WITHOUT the tag (it cannot contain
        # its own checksum) and inserted afterwards; expected_config_tag
        # reads it back out of the same builder, so the two can't drift.
        # Computed over the CONFIGURATION, deliberately excluding
        # `version` — that counter increments on every edit, so
        # including it meant editing a channel and then reverting it
        # produced byte-identical config with a different tag, and the
        # node stayed "pending" forever despite running exactly what the
        # dashboard held. Reverting a change has to actually come back
        # to the same state, or the sync indicator lies and the only way
        # to clear it is a full push the operator does not need.
        _for_tag = {k: v for k, v in payload.items() if k != "version"}
        payload["tag"] = zlib.crc32(
            json.dumps(_for_tag, separators=(",", ":")).encode()) & 0xFFFFFFFF

        out = json.dumps(payload, separators=(",", ":"))
        # Budget guard: alert at the source if a config push approaches
        # the firmware's reassembly cap. UXR_CONFIG_UDP_TRANSPORT_MTU is
        # 512, RMW_UXRCE_MAX_HISTORY is 4 → reassembly cap ≈ 2048 bytes.
        # The firmware's config_buffer is 4096; getting close to that
        # means we're also close to the XRCE cap (which is the harder
        # limit). The wire-size regression test in
        # server/test/test_maestro_wire_size_budget.py catches CI
        # regressions ahead of time; this runtime log catches
        # operator-set values that pushed past the test's coverage.
        XRCE_REASSEMBLY_CAP = 2048
        if len(out) > XRCE_REASSEMBLY_CAP:
            # REFUSE, don't warn-and-send. This guard used to log and
            # publish anyway, which is how a 2150-byte push reached the
            # Head Node on 2026-09-23: the node WDOG-reset mid-apply,
            # came back with "applied home positions to 0 channels", and
            # the server — seeing UNADOPTED — re-pushed the same
            # oversized payload, resetting it again.
            #
            # A config we know the node cannot survive is not worth
            # attempting. Returning None leaves the node on its last
            # good config with a message the operator can act on,
            # instead of a reboot loop they have to diagnose.
            self._config_push_errors[node_id] = (
                f"Config is {len(out)} bytes, over the ~{XRCE_REASSEMBLY_CAP}-byte "
                f"limit the node can receive ({len(out) - XRCE_REASSEMBLY_CAP} "
                f"bytes too large). Sending it would crash the node, so it was "
                f"not sent. Reduce per-channel overrides — channels that share a "
                f"power timeout or pulse range cost nothing extra."
            )
            self._log_activity(
                f"Refused config push for {node_id}: {len(out)} bytes "
                f"exceeds the ~{XRCE_REASSEMBLY_CAP}-byte XRCE-DDS "
                f"reassembly cap, which crashes the node on receive. The "
                f"node keeps its previous config. Reduce per-channel "
                f"overrides (channels sharing one power timeout or one "
                f"pulse range cost nothing extra) — see "
                f"docs/MAESTRO_BRINGUP.md wire-size section.",
                "error", node_id=node_id,
            )
            return None
        self._config_push_errors.pop(node_id, None)
        return out

    # Largest patch we will send. A patch exists to stay inside ONE
    # XRCE frame (MTU 512) so it is never fragmented and never touches
    # the reassembly buffer that the full push has to fight. A patch
    # approaching that size has lost its reason to exist — send the
    # full config instead, which at least gets the node to a known
    # state in one shot.
    _MAX_PATCH_BYTES = 400

    def list_current_sources(self) -> List[Dict[str, Any]]:
        """Every current-reading channel across all adopted nodes.

        For calibrating a servo you want to watch what it draws while
        you dial its extents — and the sensor is very often on a
        DIFFERENT node than the servo. On this rig the FAS100 sits on
        the Cradle Base while the Maestro is on the Head, so a
        same-node-only search (what the dashboard did before) found
        nothing at all and the indicator stayed blank.

        Which channels read current is decided from the catalog
        (`current_reading_channels`), not from the dashboard guessing at
        channel-id spellings — that copy had already drifted, missing
        the Servo 2040's aggregate `current_a`.
        """
        out: List[Dict[str, Any]] = []
        for node_id, node in self.state.adopted_nodes.items():
            if not node.peripheral_config:
                continue
            for p in node.peripheral_config.peripherals:
                for ch_id, ch_label in current_reading_channels(p.type):
                    out.append({
                        "node_id": node_id,
                        "node_name": node.display_name or node_id,
                        "online": bool(node.online),
                        "peripheral_id": p.id,
                        "peripheral_label": p.label or p.id,
                        "peripheral_type": p.type,
                        "channel_id": ch_id,
                        "channel_label": ch_label,
                    })
        # Stable, human order: node then peripheral, so the picker does
        # not reshuffle under the operator as nodes come and go.
        out.sort(key=lambda s: (s["node_name"], s["peripheral_label"],
                                s["channel_id"]))
        return out

    def record_config_push(self, node_id: str, config_json: str) -> None:
        """Remember what we last sent a node, so the next change can be
        expressed as a delta against it.

        Purely an optimization cache: if it is missing, stale, or wrong,
        :meth:`plan_config_push` falls back to a full push. Correctness
        rests on the tag comparison, never on this.
        """
        if config_json:
            self._last_pushed_json[node_id] = config_json

    def last_pushed_config(self, node_id: str) -> Optional[str]:
        """The last full config this server actually sent to a node.

        Distinct from `get_firmware_config_json`, which reflects what
        the dashboard currently holds — including edits the operator has
        not synced. Recovery paths want this one: restoring a node that
        lost its config should put back what it was running, not
        silently promote work in progress to live hardware.
        """
        return self._last_pushed_json.get(node_id)

    def plan_config_push(self, node_id: str,
                         reported_tag: Optional[int]) -> Optional[str]:
        """What to send to bring this node up to date — a patch if one
        is provably safe, otherwise the full config, otherwise None.

        Both the operator's Sync and the announce-driven reconcile go
        through here, so there is one place that decides and one set of
        rules to reason about.
        """
        full = self.get_firmware_config_json(node_id)
        if not full:
            return None
        patch = self._build_config_patch(node_id, reported_tag, full)
        return patch or full

    def _build_config_patch(self, node_id: str, reported_tag: Optional[int],
                            full_json: str) -> Optional[str]:
        """A patch_config payload, or None when a full push is required.

        A patch is only safe when we can prove what the node currently
        holds AND that the difference is confined to Maestro channel
        fields. Every other case falls back, deliberately — a delta
        applied to a base we only *assume* is the divergence this whole
        mechanism exists to prevent.
        """
        prev_json = self._last_pushed_json.get(node_id)
        if not prev_json or reported_tag is None:
            return None
        try:
            prev = json.loads(prev_json)
            cur = json.loads(full_json)
        except ValueError:
            return None

        # The node must be holding exactly the config we last sent —
        # that is the base the patch is expressed against.
        if prev.get("tag") != reported_tag:
            return None
        if prev.get("tag") == cur.get("tag"):
            return None            # nothing changed; caller sends full only if asked

        prev_by_id = {p.get("id"): p for p in prev.get("peripherals", [])}
        cur_by_id = {p.get("id"): p for p in cur.get("peripherals", [])}
        if set(prev_by_id) != set(cur_by_id):
            return None            # peripheral added or removed

        changed: Dict[str, Dict[str, Any]] = {}
        target_id: Optional[str] = None
        for pid, cur_p in cur_by_id.items():
            prev_p = prev_by_id[pid]
            if cur_p == prev_p:
                continue
            # Only Maestro channel arrays are patchable today; anything
            # else differing means a full push.
            if cur_p.get("type") != "maestro":
                return None
            if target_id is not None:
                return None        # two peripherals changed; not worth a patch
            cur_params = dict(cur_p.get("params") or {})
            prev_params = dict(prev_p.get("params") or {})
            cur_ch = cur_params.pop("channels", None)
            prev_ch = prev_params.pop("channels", None)
            if cur_params != prev_params:
                return None        # a peripheral-level param moved too
            if not isinstance(cur_ch, list) or not isinstance(prev_ch, list):
                return None
            if len(cur_ch) != len(prev_ch):
                return None
            for i, (a, b) in enumerate(zip(prev_ch, cur_ch)):
                if a == b:
                    continue
                # Send the channel's full new field set, not a
                # field-level diff: a field the operator RESET to its
                # default disappears from the slimmed wire form, and a
                # field-level diff would silently leave the node on the
                # old value. Whole-channel is both smaller to reason
                # about and correct by construction.
                changed[str(i)] = b
            target_id = pid

        if not changed or target_id is None:
            return None

        payload = json.dumps({
            "action": "patch_config",
            "from": reported_tag,
            "to": cur.get("tag"),
            "peripheral": target_id,
            "channels": changed,
        }, separators=(",", ":"))
        if len(payload) > self._MAX_PATCH_BYTES:
            return None
        return payload

    def _synced_snapshot_path(self, node_id: str) -> str:
        """Sidecar holding the last config a node CONFIRMED running.

        A separate file rather than a second copy inside the node's YAML
        so the live config stays the readable one an operator can open
        and reason about.
        """
        return os.path.join(self.nodes_config_dir, f"{node_id}.yaml.synced")

    def has_synced_snapshot(self, node_id: str) -> bool:
        """Whether there is a confirmed state to revert to."""
        return os.path.exists(self._synced_snapshot_path(node_id))

    def _snapshot_synced_config(self, node_id: str) -> None:
        """Record the current config as the node's confirmed state.

        Called only when the node's reported tag matches ours, which is
        exactly when the stored config IS what the hardware runs. Doing
        it at push time instead would record an optimistic state — a
        push that never lands would leave a 'confirmed' snapshot the
        node never received, and Revert would restore fiction.
        """
        src = os.path.join(self.nodes_config_dir, f"{node_id}.yaml")
        if not os.path.exists(src):
            return
        try:
            shutil.copyfile(src, self._synced_snapshot_path(node_id))
        except OSError as e:
            if self.logger:
                self.logger.warn(f"Could not snapshot synced config: {e}")

    def revert_node_peripherals(self, node_id: str) -> Dict[str, Any]:
        """Discard unsynced edits, restoring what the node is running.

        The operator's escape hatch: config only reaches hardware on an
        explicit Sync, so anything edited but not synced exists solely
        on the server and can be thrown away without touching the node.
        Nothing is published here.
        """
        node = self.state.adopted_nodes.get(node_id)
        if not node:
            return {"success": False, "message": f"Node {node_id} not found"}
        snap = self._synced_snapshot_path(node_id)
        if not os.path.exists(snap):
            return {"success": False,
                    "message": "No synced configuration to revert to — this "
                               "node has not confirmed a config yet."}
        dest = os.path.join(self.nodes_config_dir, f"{node_id}.yaml")
        try:
            shutil.copyfile(snap, dest)
        except OSError as e:
            return {"success": False, "message": f"Revert failed: {e}"}
        if not self._load_node_config(node_id):
            return {"success": False,
                    "message": "Restored file but could not load it"}
        if node.peripheral_config:
            # It matches the hardware again by construction.
            node.peripheral_config.sync_status = "synced"
        self._save_node_config(node_id)
        self._log_activity("Reverted unsynced peripheral changes", "info",
                           node_id=node_id)
        self._maybe_notify_host_peripheral_change(node_id)
        return {"success": True}

    def observe_node_config_tag(self, node_id: str,
                                node_tag: Optional[int]) -> bool:
        """Record whether a node is actually holding our config.

        Observation only — this NEVER pushes. Config reaches a node
        during an explicit Sync and at no other time, so an operator can
        edit, look at it, and revert without the server having quietly
        shipped the half-finished version to the hardware.

        What the tag buys us here is that `sync_status` stops being a
        belief and becomes a fact. It used to be set to "synced" at the
        moment we published, which said only that a message left the
        server; a push that never landed, or a node that later rebooted
        onto an older blob, still read as "synced". Now "synced" means
        the node told us which config it is holding and it is ours.

        Returns True if the status changed, so the caller can broadcast
        it. Writes the node's YAML only on a real transition — this runs
        on every announcement, at roughly 1 Hz per node.
        """
        node = self.state.adopted_nodes.get(node_id)
        if not node or not node.peripheral_config or node_tag is None:
            return False
        expected = self.expected_config_tag(node_id)
        if expected is None:
            return False
        want = "synced" if node_tag == expected else "pending"

        # Snapshot on any confirmation, not only on a transition. A node
        # already sitting at "synced" when this shipped would otherwise
        # never produce one — no transition ever comes — and Revert
        # would stay unavailable indefinitely. The existence check keeps
        # this to one stat() on the ~1 Hz announcement path.
        if want == "synced" and not self.has_synced_snapshot(node_id):
            self._snapshot_synced_config(node_id)

        if node.peripheral_config.sync_status == want:
            return False
        node.peripheral_config.sync_status = want
        self._save_node_config(node_id)
        if want == "synced":
            # Freshly confirmed: this is the state Revert returns to.
            self._snapshot_synced_config(node_id)
        return True

    def expected_config_tag(self, node_id: str) -> Optional[int]:
        """The tag this node's CURRENT server-side config would carry.

        Compared against the `cfg_tag` the node reports in /announce to
        decide whether it is holding what we intend. None means there is
        nothing to compare — no config, or a config we refused to build
        — and the caller must not treat that as a mismatch.

        Deliberately derived from the same builder as the push rather
        than tracked separately: a second source of truth for "what
        should the node have" is precisely the drift this exists to
        detect.
        """
        js = self.get_firmware_config_json(node_id)
        if not js:
            return None
        try:
            tag = json.loads(js).get("tag")
            return int(tag) if tag is not None else None
        except (ValueError, TypeError):
            return None

    def last_config_push_error(self, node_id: str) -> Optional[str]:
        """Why the most recent config build for this node was refused,
        or None if the last one was fine. Lets the Sync action tell the
        operator what actually happened instead of the generic "nothing
        to sync"."""
        return self._config_push_errors.get(node_id)

    def _generate_peripheral_id(self, node: NodeInfo, type_id: str) -> str:
        config = node.peripheral_config or NodePeripheralConfig()
        n = sum(1 for p in config.peripherals if p.type == type_id) + 1
        existing_ids = {p.id for p in config.peripherals}
        while f"{type_id}-{n}" in existing_ids:
            n += 1
        return f"{type_id}-{n}"

    # =========================================================================
    # System-wide routing graph (routes + widgets)
    # =========================================================================

    def get_system_routing(self) -> Dict[str, Any]:
        return self.state.system_routing.to_dict()

    # ── Sheet-scoped graph ops ────────────────────────────────────────

    def get_routing_sheet(self, node_id: str) -> Dict[str, Any]:
        """Return the sheet for `node_id` (creates an empty one on demand)."""
        return self.state.system_routing.get_sheet(node_id).to_dict()

    def add_routing_input(self, node_id: str, topic: str = "", field: str = "",
                          label: str = "",
                          position: Optional[List[int]] = None,
                          kind: str = "topic",
                          joint: str = "",
                          channel_node_id: str = "",
                          peripheral_id: str = "",
                          channel_id: str = "") -> Dict[str, Any]:
        # Each input kind has its own required addressing: a topic, a
        # joint name, or a (node, peripheral, channel) triple.
        if kind == "urdf_joint":
            if not joint:
                return {"success": False, "message": "Missing joint name"}
        elif kind == "channel":
            if not (channel_node_id and peripheral_id and channel_id):
                return {"success": False,
                        "message": "Channel inputs need node, peripheral, "
                                   "and channel"}
        else:
            if not topic:
                return {"success": False, "message": "Missing topic"}
        err = self._validate_sheet_owner(node_id)
        if err:
            return {"success": False, "message": err}
        pos = self._coerce_position(position)
        sheet = self.state.system_routing.get_sheet(node_id)
        node = sheet.add_input(topic=topic or "", field=field or "",
                               label=label, position=pos,
                               kind=kind, joint=joint or "",
                               channel_node_id=channel_node_id or "",
                               peripheral_id=peripheral_id or "",
                               channel_id=channel_id or "")
        self.state.system_routing.bump_version()
        self._save_system_routing()
        return {"success": True, "input": node.to_dict()}

    def add_routing_ws_input(self, node_id: str, label: str = "",
                             position: Optional[List[int]] = None
                             ) -> Dict[str, Any]:
        err = self._validate_sheet_owner(node_id)
        if err:
            return {"success": False, "message": err}
        pos = self._coerce_position(position)
        sheet = self.state.system_routing.get_sheet(node_id)
        node = sheet.add_ws_input(label=label, position=pos)
        self.state.system_routing.bump_version()
        self._save_system_routing()
        return {"success": True, "ws_input": node.to_dict()}

    def list_ws_inputs(self) -> List[Dict[str, Any]]:
        """Flat enumeration of every WS input across all sheets.

        Powers the controller's binding picker — the picker shows
        {sheet_id, sheet_label, input_id, label} tuples and writes back
        via routing/set_input. `sheet_label` is the adopted node's
        display_name so the picker shows "Track Drive Right" rather
        than "teensy41_A1B2C3D4".
        """
        out: List[Dict[str, Any]] = []
        for sheet_id, sheet in self.state.system_routing.sheets.items():
            node = self.state.adopted_nodes.get(sheet_id)
            sheet_label = (node.display_name if node and node.display_name
                           else sheet_id)
            for ws in sheet.ws_inputs:
                out.append({
                    "sheet_id": sheet_id,
                    "sheet_label": sheet_label,
                    "input_id": ws.id,
                    "label": ws.label,
                    # "command" or "state". Frontend binding picker
                    # filters to "command" only — state nodes are
                    # migration-only echoes of state-only ROS
                    # endpoints, not real controller targets.
                    "kind": ws.kind,
                })
        return out

    def push_ws_input(self, sheet_id: str, input_id: str, value: float) -> bool:
        """Forward a controller write into the live routing evaluator."""
        if self._routing_evaluator is None:
            return False
        try:
            return bool(self._routing_evaluator.set_ws_input(
                sheet_id, input_id, value))
        except Exception as e:
            if self.logger:
                self.logger.error(f"push_ws_input failed: {e}")
            return False

    def stage_ws_input(self, sheet_id: str, input_id: str,
                       value: float) -> bool:
        """Absorb a controller write without evaluating (see
        RoutingEvaluator.stage_ws_input). Paired with flush_ws_inputs."""
        if self._routing_evaluator is None:
            return False
        try:
            return bool(self._routing_evaluator.stage_ws_input(
                sheet_id, input_id, value))
        except Exception as e:
            if self.logger:
                self.logger.error(f"stage_ws_input failed: {e}")
            return False

    def has_staged_ws_input(self) -> bool:
        """Whether any controller input is absorbed but not yet evaluated."""
        if self._routing_evaluator is None:
            return False
        try:
            return bool(self._routing_evaluator.has_staged_input())
        except Exception:
            return False

    def flush_ws_inputs(self) -> int:
        """Evaluate sheets with staged controller input. Returns the
        number of sheets evaluated."""
        if self._routing_evaluator is None:
            return 0
        try:
            return int(self._routing_evaluator.flush_staged())
        except Exception as e:
            if self.logger:
                self.logger.error(f"flush_ws_inputs failed: {e}")
            return 0

    def add_routing_output(self, node_id: str, topic: str, field: str,
                           label: str = "",
                           position: Optional[List[int]] = None) -> Dict[str, Any]:
        if not topic:
            return {"success": False, "message": "Missing topic"}
        err = self._validate_sheet_owner(node_id)
        if err:
            return {"success": False, "message": err}
        pos = self._coerce_position(position)
        sheet = self.state.system_routing.get_sheet(node_id)
        node = sheet.add_output(topic=topic, field=field or "", label=label, position=pos)
        self.state.system_routing.bump_version()
        self._save_system_routing()
        return {"success": True, "output": node.to_dict()}

    def add_routing_operator(self, node_id: str, op: str, label: str = "",
                             params: Optional[Dict[str, Any]] = None,
                             defaults: Optional[Dict[str, float]] = None,
                             position: Optional[List[int]] = None) -> Dict[str, Any]:
        if op not in OPERATOR_CATALOG:
            return {"success": False, "message": f"Unknown operator '{op}'"}
        err = self._validate_sheet_owner(node_id)
        if err:
            return {"success": False, "message": err}
        pos = self._coerce_position(position)
        sheet = self.state.system_routing.get_sheet(node_id)
        node = sheet.add_operator(op=op, label=label, params=params,
                                  defaults=defaults, position=pos)
        self.state.system_routing.bump_version()
        self._save_system_routing()
        return {"success": True, "operator": node.to_dict()}

    def add_routing_signal(self, node_id: str, name: str, label: str = "",
                           position: Optional[List[int]] = None) -> Dict[str, Any]:
        """Add a SignalNode on `node_id`'s sheet bound to global signal
        `name`. Two sheets each holding a SignalNode named "foo" share
        the same underlying value — that's the cross-sheet point."""
        if not name or not name.strip():
            return {"success": False, "message": "Signal name required"}
        err = self._validate_sheet_owner(node_id)
        if err:
            return {"success": False, "message": err}
        pos = self._coerce_position(position)
        sheet = self.state.system_routing.get_sheet(node_id)
        node = sheet.add_signal(name=name, label=label, position=pos)
        self.state.system_routing.bump_version()
        self._save_system_routing()
        return {"success": True, "signal": node.to_dict()}

    def remove_routing_signal(self, node_id: str, signal_id: str) -> Dict[str, Any]:
        err = self._validate_sheet_owner(node_id)
        if err:
            return {"success": False, "message": err}
        sheet = self.state.system_routing.get_sheet(node_id)
        if not sheet.remove_signal(signal_id):
            return {"success": False, "message": f"Signal '{signal_id}' not found"}
        self.state.system_routing.bump_version()
        self._save_system_routing()
        return {"success": True}

    def add_routing_wire(self, node_id: str, source: Dict[str, Any],
                         sink: Dict[str, Any]) -> Dict[str, Any]:
        err = self._validate_sheet_owner(node_id)
        if err:
            return {"success": False, "message": err}
        sheet = self.state.system_routing.get_sheet(node_id)
        src = RouteEndpoint.from_dict(source)
        snk = RouteEndpoint.from_dict(sink)
        err = self._validate_wire(node_id, src, snk)
        if err:
            return {"success": False, "message": err}
        wire = sheet.add_wire(src, snk)
        self.state.system_routing.bump_version()
        self._save_system_routing()
        return {"success": True, "wire": wire.to_dict()}

    def update_sheet_node(self, node_id: str, sheet_node_id: str,
                          **changes) -> Dict[str, Any]:
        """Mutate one sheet node in place.

        Supported keys depend on the node kind:
          - any node:               position, label
          - OperatorNode:           params, defaults
          - InputNode/OutputNode:   topic, field
          - InputNode (only):       kind, joint
          - WidgetInstance:         params
          - SignalNode:             name  (renames the global signal
                                    this node binds to — other sheets
                                    holding a SignalNode by the SAME
                                    new name then share its value;
                                    wires keyed by the OLD name stop
                                    resolving until rewired)
        """
        err = self._validate_sheet_owner(node_id)
        if err:
            return {"success": False, "message": err}
        sheet = self.state.system_routing.get_sheet(node_id)
        target = (sheet.find_input(sheet_node_id)
                  or sheet.find_ws_input(sheet_node_id)
                  or sheet.find_output(sheet_node_id)
                  or sheet.find_operator(sheet_node_id)
                  or sheet.find_widget(sheet_node_id)
                  or sheet.find_signal(sheet_node_id))
        if target is None:
            return {"success": False, "message": f"Sheet node '{sheet_node_id}' not found"}
        for k, v in changes.items():
            if k == "position" and isinstance(v, (list, tuple)) and len(v) >= 2:
                target.position = (int(v[0]), int(v[1]))
            elif k == "label" and isinstance(v, str):
                target.label = v
            elif k == "params" and isinstance(target, (OperatorNode, WidgetInstance)):
                target.params = dict(v or {})
            elif k == "defaults" and isinstance(target, OperatorNode):
                target.defaults = {dk: float(dv) for dk, dv in (v or {}).items()}
            elif k == "topic" and isinstance(target, (InputNode, OutputNode)) and isinstance(v, str):
                target.topic = v
            elif k == "field" and isinstance(target, (InputNode, OutputNode)) and isinstance(v, str):
                target.field = v
            elif k == "kind" and isinstance(target, InputNode) and isinstance(v, str):
                target.kind = v
            elif k == "joint" and isinstance(target, InputNode) and isinstance(v, str):
                target.joint = v
            elif k == "name" and isinstance(target, SignalNode) and isinstance(v, str):
                target.name = v.strip()
        self.state.system_routing.bump_version()
        self._save_system_routing()
        return {"success": True}

    def remove_sheet_node(self, node_id: str, sheet_node_id: str) -> Dict[str, Any]:
        sheet = self.state.system_routing.sheets.get(node_id)
        if sheet is None:
            return {"success": False, "message": f"Sheet '{node_id}' not found"}
        removed = (sheet.remove_input(sheet_node_id)
                   or sheet.remove_ws_input(sheet_node_id)
                   or sheet.remove_output(sheet_node_id)
                   or sheet.remove_operator(sheet_node_id)
                   or sheet.remove_widget(sheet_node_id))
        if not removed:
            return {"success": False, "message": f"Sheet node '{sheet_node_id}' not found"}
        self.state.system_routing.bump_version()
        self._save_system_routing()
        return {"success": True}

    def remove_routing_wire(self, node_id: str, wire_id: str) -> Dict[str, Any]:
        sheet = self.state.system_routing.sheets.get(node_id)
        if sheet is None or not sheet.remove_wire(wire_id):
            return {"success": False, "message": f"Wire '{wire_id}' not found"}
        self.state.system_routing.bump_version()
        self._save_system_routing()
        return {"success": True}

    # ── animations & poses ──────────────────────────────────────────

    def list_animations(self) -> List[Dict[str, Any]]:
        return self._with_playlists("animations", self.animation_store.list())

    def _with_playlists(self, kind: str,
                        rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Annotate item summaries with the playlists they belong to.

        One reverse-index build per list call rather than a scan per row.
        `playlists` is ordered the way the sidebar orders them, so a UI
        showing "first playlist" badges shows a stable one.
        """
        try:
            index = self.playlist_store.memberships(kind)
        except Exception:
            index = {}
        for row in rows:
            row["playlists"] = list(index.get(row.get("id"), []))
        return rows

    def get_animation(self, animation_id: str) -> Optional[Dict[str, Any]]:
        anim = self.animation_store.get(animation_id)
        return anim.to_dict() if anim else None

    def save_animation(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Upsert from a dict matching Animation.to_dict() shape."""
        from saint_server.animation.models import Animation
        try:
            anim = Animation.from_dict(payload)
        except (KeyError, ValueError, TypeError) as e:
            return {"success": False, "message": f"Invalid animation payload: {e}"}
        saved = self.animation_store.save(anim)
        return {"success": True, "animation": saved.to_dict()}

    def delete_animation(self, animation_id: str) -> Dict[str, Any]:
        """Delete the animation and remove any sheet nodes referencing it."""
        deleted = self.animation_store.delete(animation_id)
        if not deleted:
            return {"success": False, "message": "Animation not found"}
        # Clear the id out of every playlist, so a later animation that
        # slugs to the same id can't inherit this one's memberships.
        self.playlist_store.forget_item("animations", animation_id)
        # Stop any running playback before scrubbing references so the
        # evaluator's animation cache isn't fed by an already-unbound
        # animation in the interim.
        if self._animation_registry is not None and self._animation_registry.is_active(animation_id):
            try:
                import asyncio
                asyncio.create_task(self._animation_registry.stop(animation_id))
            except Exception:
                pass
        # Animations now drive routing through URDF-joint InputNodes
        # keyed by joint name, not animation id. Deleting an animation
        # doesn't invalidate those nodes — the joint inputs simply stop
        # receiving values until another animation drives the same joint.
        return {"success": True}

    def list_poses(self) -> List[Dict[str, Any]]:
        return self._with_playlists("poses", self.pose_store.list())

    def get_pose(self, pose_id: str) -> Optional[Dict[str, Any]]:
        pose = self.pose_store.get(pose_id)
        return pose.to_dict() if pose else None

    def save_pose(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        from saint_server.animation.models import Pose
        try:
            pose = Pose.from_dict(payload)
        except (KeyError, ValueError, TypeError) as e:
            return {"success": False, "message": f"Invalid pose payload: {e}"}
        saved = self.pose_store.save(pose)
        self.invalidate_rig_cache()
        return {"success": True, "pose": saved.to_dict()}

    def delete_pose(self, pose_id: str) -> Dict[str, Any]:
        if not self.pose_store.delete(pose_id):
            return {"success": False, "message": "Pose not found"}
        self.playlist_store.forget_item("poses", pose_id)
        self.invalidate_rig_cache()
        return {"success": True}

    def apply_pose(self, pose_id: str) -> Dict[str, Any]:
        """Fan out a pose's setpoints into the routing evaluator.

        Two address spaces, matching PoseSetpoint.target_kind:
          * ``ws_input`` → ``set_ws_input(sheet, input, value)``, the
            same path the controller gamepad bindings use, so peripheral
            routing applies identically.
          * ``joint`` → ``set_urdf_joint_value(joint, value)``, the path
            animation value tracks use. This is what SRDF group_state
            imports produce.

        Both go through ``apply_animation_frame`` when available so the
        whole pose costs one sheet evaluation and one UI broadcast rather
        than one of each per setpoint.
        """
        if self._routing_evaluator is None:
            return {"success": False, "message": "Routing evaluator not ready"}
        pose = self.pose_store.get(pose_id)
        if pose is None:
            return {"success": False, "message": "Pose not found"}
        return self._fan_out_setpoints(pose.setpoints, "apply_pose")

    def preview_setpoints(self, setpoints: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Apply an inline list of pose setpoints WITHOUT saving the pose.

        Same fan-out as :meth:`apply_pose`, but driven by setpoints the
        operator is editing rather than a stored pose. Lets the pose
        editor's "Preview" button push the in-progress pose live so the
        operator can A/B it against the saved version (the row's play
        button) before committing the save. Nothing is persisted.
        """
        if self._routing_evaluator is None:
            return {"success": False, "message": "Routing evaluator not ready"}
        from saint_server.animation.models import PoseSetpoint
        parsed = []
        for raw in setpoints or []:
            try:
                parsed.append(PoseSetpoint.from_dict(raw))
            except (TypeError, ValueError) as e:
                if self.logger:
                    self.logger.warn(f"preview_setpoints: bad setpoint {raw!r}: {e}")
        return self._fan_out_setpoints(parsed, "preview_setpoints")

    def _fan_out_setpoints(self, setpoints, where: str) -> Dict[str, Any]:
        """Shared apply path for stored and in-progress poses."""
        ev = self._routing_evaluator
        joint_values: Dict[str, float] = {}
        ws_values: Dict[Any, float] = {}
        skipped: List[str] = []

        for s in setpoints:
            if s.is_joint:
                if not s.joint:
                    skipped.append("(joint setpoint with no joint name)")
                    continue
                joint_values[s.joint] = s.value
            else:
                if not s.sheet_id or not s.ws_input_id:
                    skipped.append(s.address())
                    continue
                ws_values[(s.sheet_id, s.ws_input_id)] = s.value

        applied = 0
        batch = getattr(ev, "apply_animation_frame", None)
        if batch is not None and (joint_values or ws_values):
            # A board activation is latched operator authority: it must
            # override whatever else last touched these channels — a
            # slider nudge, a stick, a write that was dropped in flight.
            # dispatch_as(BOARD) makes every resulting peripheral write
            # bypass the change gate, so re-activating a pose always
            # re-asserts the hardware instead of silently no-opping on
            # exactly the channels the operator had touched.
            tally = DispatchTally()
            try:
                with _board_dispatch(ev, tally):
                    ok = batch(joint_values, ws_values)
                if ok:
                    applied = len(joint_values) + len(ws_values)
                else:
                    skipped.extend(list(joint_values))
                    skipped.extend(f"{a}/{b}" for a, b in ws_values)
            except Exception as e:
                if self.logger:
                    self.logger.warn(f"{where} apply_animation_frame failed: {e}")
                skipped.extend(list(joint_values))
                skipped.extend(f"{a}/{b}" for a, b in ws_values)
            # `applied` counts setpoints accepted into the evaluator;
            # `dispatched` counts channels that actually reached
            # firmware. They differ whenever a sheet maps several
            # setpoints onto one channel, or a sink is wired to nothing
            # — and the second number is the one an operator means when
            # they ask "did the pose take?".
            return {"success": True, "applied": applied, "skipped": skipped,
                    "dispatched": tally.sent, "suppressed": tally.suppressed}

        # Per-setpoint fallback for evaluators without the batch call.
        # Same latched authority as the batch path above — a board is a
        # board regardless of which dispatch API the evaluator exposes.
        with _board_dispatch(ev, None):
            return self._fan_out_per_setpoint(
                ev, joint_values, ws_values, skipped, where)

    def _fan_out_per_setpoint(self, ev, joint_values, ws_values,
                              skipped, where: str) -> Dict[str, Any]:
        applied = 0
        for joint, value in joint_values.items():
            try:
                ok = ev.set_urdf_joint_value(joint, value)
            except Exception as e:
                if self.logger:
                    self.logger.warn(f"{where} set_urdf_joint_value failed: {e}")
                ok = False
            applied += 1 if ok else 0
            if not ok:
                skipped.append(joint)
        for (sheet_id, input_id), value in ws_values.items():
            try:
                ok = ev.set_ws_input(sheet_id, input_id, value)
            except Exception as e:
                if self.logger:
                    self.logger.warn(f"{where} set_ws_input failed: {e}")
                ok = False
            applied += 1 if ok else 0
            if not ok:
                skipped.append(f"{sheet_id}/{input_id}")
        return {"success": True, "applied": applied, "skipped": skipped}

    # ── control rig ───────────────────────────────────────────────
    #
    # The rig evaluates HERE, not in the browser, and the response
    # carries the resolved joint values back so the client can drive its
    # 3D viewport with them. A JS port of the evaluator would be a second
    # implementation of the blend math, the clamp policy, and the mimic
    # round-trip — three places to drift. A local websocket round-trip is
    # a millisecond or two, which a slider drag does not notice.

    def get_rig(self) -> Dict[str, Any]:
        """The full parsed rig, for building the control panel.

        Includes ``<widget>`` hints, which the evaluator itself never
        reads — presentation is the client's business and the contract
        is the server's.
        """
        if self.robot_store is None:
            return {"success": False, "message": "Robot model store not ready"}
        rig = self.robot_store.load_rig()
        if rig is None:
            return {"success": True, "rig": None}
        urdf = self.robot_store.load_urdf()
        srdf = self.robot_store.load_srdf()
        pose_names = [p["id"] for p in self.pose_store.list()]
        evaluator = self._rig_evaluator(rig)
        # Which link each control's 3D shape hangs off. Derived from what
        # the control drives when the file doesn't say, so a rig authored
        # before widget geometry existed still draws in the viewport
        # instead of silently showing nothing.
        anchors = rig.resolve_anchors_with_poses(
            urdf, evaluator.poses if evaluator else None)
        return {
            "success": True,
            "rig": rig.to_dict(),
            "defaults": evaluator.control_defaults() if evaluator else {},
            "anchors": anchors,
            # The resolved neutral pose, not just its name. The animation
            # editor needs the same base the player blends pose tracks up
            # from — resolving `settings.neutral_pose` client-side would
            # be a second implementation of a rule that already lives in
            # rig_neutral(). See preview_animation_frame.
            "neutral": self.rig_neutral(),
            "warnings": rig.validate(urdf, srdf, pose_names),
        }

    def _rig_evaluator(self, rig=None):
        """Evaluator over the installed rig and pose library, cached.

        The cache is not an optimization detail — it's required. Building
        this from scratch parses the rig XML, parses the whole URDF, and
        reads one JSON file per referenced pose. ``evaluate_rig`` is
        called on every slider input event (~30/s while dragging), so
        doing that per call means re-parsing a several-hundred-link URDF
        thirty times a second on a Pi.

        Keyed on the two file hashes plus a pose generation counter, so
        any upload or pose edit that goes through this API invalidates it.
        A pose file edited on disk behind our back won't — same as the
        rest of the runtime config.
        """
        from saint_server.animation.rig_eval import RigEvaluator

        if self.robot_store is None:
            return None

        explicit_rig = rig is not None
        meta = self.robot_store.get_metadata()
        key = None
        if not explicit_rig and meta is not None:
            key = (meta.sha256, meta.rig_sha256, self._pose_generation)
            if key == self._rig_eval_key and self._rig_eval_cache is not None:
                return self._rig_eval_cache

        if rig is None:
            rig = self.robot_store.load_rig()
        if rig is None:
            return None
        urdf = self.robot_store.load_urdf()

        # Only the poses the rig actually references, plus the neutral —
        # loading the whole library would read every pose file off disk.
        wanted = set(rig.referenced_poses())
        if rig.settings.neutral_pose:
            wanted.add(rig.settings.neutral_pose)
        poses: Dict[str, Dict[str, float]] = {}
        for name in wanted:
            pose = self.pose_store.get(name)
            if pose is not None:
                poses[name] = pose.joint_values()

        evaluator = RigEvaluator(rig, urdf=urdf, poses=poses)
        if key is not None:
            self._rig_eval_key = key
            self._rig_eval_cache = evaluator
        return evaluator

    def invalidate_rig_cache(self) -> None:
        """Drop the cached rig evaluator.

        Called whenever a pose changes. Poses are inputs to the blend, so
        a stale cache would keep a control blending toward the old shape
        of a pose the operator just edited.
        """
        self._pose_generation += 1
        self._rig_eval_key = None
        self._rig_eval_cache = None

    def evaluate_rig(self, values: Optional[Dict[str, Any]] = None,
                     apply: bool = False) -> Dict[str, Any]:
        """Evaluate the rig at ``values`` and optionally drive the robot.

        ``values`` is keyed by control name, and by ``"<control>.<axis>"``
        for pad controls. Anything omitted sits at its declared default,
        so a partial dict is fine.

        Returns the resolved joint values plus per-control contributions
        (so the UI can answer "which slider moved this joint?"), whether
        the clamp policy had to pull the frame back, and any controls the
        evaluator couldn't handle. With ``apply``, the same frame is
        pushed into the routing graph through the animation-frame batch
        path — one sheet evaluation and one broadcast for the whole frame.
        """
        evaluator = self._rig_evaluator()
        if evaluator is None:
            return {"success": False, "message": "No rig file installed"}

        clean: Dict[str, float] = {}
        for key, raw in (values or {}).items():
            try:
                clean[str(key)] = float(raw)
            except (TypeError, ValueError):
                if self.logger:
                    self.logger.warn(f"evaluate_rig: non-numeric {key}={raw!r}")

        frame = evaluator.evaluate(clean)
        result = {"success": True, **frame.to_dict()}

        if apply:
            if self._routing_evaluator is None:
                result["applied"] = 0
                result["message"] = "Routing evaluator not ready"
                return result
            try:
                batch = getattr(self._routing_evaluator,
                                "apply_animation_frame", None)
                if batch is not None:
                    batch(frame.joints, {})
                else:
                    for joint, value in frame.joints.items():
                        self._routing_evaluator.set_urdf_joint_value(joint, value)
                result["applied"] = len(frame.joints)
            except Exception as e:
                if self.logger:
                    self.logger.warn(f"evaluate_rig apply failed: {e}")
                result["applied"] = 0
                result["message"] = str(e)
        return result

    # ── SRDF group_state import ───────────────────────────────────

    def list_group_states(self) -> Dict[str, Any]:
        """SRDF group_states available to import as poses.

        Each entry is annotated with what importing it would do —
        ``exists`` when a pose of that id is already present, and
        ``locally_edited`` when that pose was imported before and has
        since been modified in the UI. The prompt needs both to avoid
        silently overwriting an operator's tweaks.
        """
        if self.robot_store is None:
            return {"success": False, "message": "Robot model store not ready",
                    "group_states": []}
        from saint_server.animation.store import slugify

        existing = {p["id"]: p for p in self.pose_store.list()}
        out = []
        for gs in self.robot_store.list_group_states():
            pose_id = slugify(gs["name"])
            prior = existing.get(pose_id)
            entry = dict(gs)
            entry["pose_id"] = pose_id
            entry["exists"] = prior is not None
            entry["locally_edited"] = bool(
                prior and prior.get("modified")
                and prior.get("modified") != prior.get("created"))
            out.append(entry)
        return {"success": True, "group_states": out}

    def import_group_states(self, names: Optional[List[str]] = None,
                            icon: str = "",
                            overwrite: bool = False) -> Dict[str, Any]:
        """Create poses from SRDF ``<group_state>`` definitions.

        ``names`` selects which group_states to import; omitting it takes
        all of them. Existing poses are skipped unless ``overwrite``,
        because an operator may have tuned an imported pose by hand and a
        re-upload of the SRDF shouldn't discard that.

        Values are converted from URDF-native units to the normalized
        −1..+1 the pose library stores, using each joint's ``<limit>``.
        Joints the URDF doesn't have are dropped and reported rather than
        passed through — an unconverted radian value one hop from a servo
        is not an acceptable failure mode.

        Each imported pose joins a playlist named after the SRDF group it
        came from, created on demand. That replaces the import modal's
        old "Pose group" box: the SRDF already says how these poses group
        up, so asking the operator to retype it was always redundant, and
        with many-to-many membership there is no single field to put it
        in anyway.
        """
        if self.robot_store is None:
            return {"success": False, "message": "Robot model store not ready"}
        if not self.robot_store.has_model():
            return {"success": False, "message": "No URDF installed"}

        from saint_server.animation.models import Pose, PoseSetpoint
        from saint_server.animation.store import slugify

        candidates = self.robot_store.list_group_states()
        if not candidates:
            return {"success": False,
                    "message": "No SRDF group_states found — is an SRDF installed?"}

        wanted = set(names) if names else None
        imported: List[Dict[str, Any]] = []
        skipped: List[Dict[str, str]] = []
        warnings: List[str] = []

        for gs in candidates:
            if wanted is not None and gs["name"] not in wanted:
                continue

            if not gs["normalized"]:
                skipped.append({
                    "name": gs["name"],
                    "reason": "no joints resolved against the URDF"})
                continue

            pose = Pose(
                id="", name=gs["name"], icon=icon,
                description=(f"Imported from SRDF group_state "
                             f"'{gs['name']}'"
                             + (f" (group {gs['group']})" if gs.get("group") else "")),
                source="srdf",
                source_ref=gs["name"],
                setpoints=[
                    PoseSetpoint(target_kind="joint", joint=joint, value=value)
                    for joint, value in sorted(gs["normalized"].items())
                ],
            )

            existing = self.pose_store.get(slugify(pose.name))
            if existing is not None and not overwrite:
                skipped.append({"name": gs["name"],
                                "reason": f"pose '{existing.id}' already exists"})
                continue
            if existing is not None:
                # Keep the original creation stamp so overwriting reads
                # as a revision rather than a brand-new pose.
                pose.created = existing.created

            saved = self.pose_store.save(pose)
            imported.append({"id": saved.id, "name": saved.name,
                             "joint_count": len(saved.setpoints)})
            srdf_group = str(gs.get("group") or "").strip()
            if srdf_group:
                self._playlist_named(srdf_group, "poses").add_item_id(saved.id)

            if gs["unresolved"]:
                warnings.append(
                    f"'{gs['name']}': dropped {len(gs['unresolved'])} joint(s) "
                    f"not in the URDF ({', '.join(gs['unresolved'][:4])}"
                    f"{'…' if len(gs['unresolved']) > 4 else ''})")

        if imported:
            self.invalidate_rig_cache()
        if self.logger and imported:
            from saint_server.log_level import log_at
            log_at(self.logger, "info",
                   f"Imported {len(imported)} SRDF group_state(s) as poses"
                   f"{f'; {len(skipped)} skipped' if skipped else ''}")

        return {"success": True, "imported": imported,
                "skipped": skipped, "warnings": warnings}

    # ── soundboard ────────────────────────────────────────────────
    #
    # Persistence + entry resolution only. The actual play/stop/browse
    # ROS round-trips are issued by the websocket handler through the
    # soundboard callback (like ble_scan) — kept out of here so this
    # module stays free of ROS concerns.

    def list_sounds(self) -> List[Dict[str, Any]]:
        rows = self._with_playlists("sounds", self.sound_store.list())
        # Legacy `group` compatibility for the Steam Deck controller,
        # whose panel filter still reads a single group name per sound
        # (controller/src/composables/useLibrary.ts). It gets the name of
        # the first playlist the sound belongs to. Drop this once the
        # controller reads `playlists`.
        try:
            names = {p["id"]: p["name"]
                     for p in self.playlist_store.list("sounds")}
        except Exception:
            names = {}
        for row in rows:
            pls = row.get("playlists") or []
            row["group"] = names.get(pls[0], "") if pls else ""
        return rows

    def get_sound(self, sound_id: str) -> Optional[Dict[str, Any]]:
        snd = self.sound_store.get(sound_id)
        return snd.to_dict() if snd else None

    def save_sound(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        from saint_server.animation.models import Sound
        try:
            snd = Sound.from_dict(payload)
        except (KeyError, ValueError, TypeError) as e:
            return {"success": False, "message": f"Invalid sound payload: {e}"}
        saved = self.sound_store.save(snd)
        return {"success": True, "sound": saved.to_dict()}

    def bulk_add_sounds(self, node_id: str, files: List[str],
                        output_device: str = "default", playlist_id: str = "",
                        volume: float = 1.0, start_time: float = 0.0,
                        loop: bool = False, loop_count: int = 0,
                        icon: str = "volume_up") -> Dict[str, Any]:
        """Create one soundboard entry per file, skipping any already
        added. "Already added" = an existing sound with the same
        (node_id, file_path) — so re-running on a folder only adds the
        new files. Each entry's name/id derives from the filename;
        ids are de-duplicated with a numeric suffix so distinct files
        that slugify alike don't clobber each other. Returns
        {added, skipped, sounds}."""
        from saint_server.animation.models import Sound
        from saint_server.animation.store import slugify

        existing = self.sound_store.list()
        have_paths = {(s.get("node_id"), s.get("file_path")) for s in existing}
        used_ids = {s.get("id") for s in existing}

        added: List[str] = []
        added_ids: List[str] = []
        skipped: List[str] = []
        for raw in files or []:
            path = str(raw).strip()
            if not path:
                continue
            if (node_id, path) in have_paths:
                skipped.append(path)
                continue
            base = path.rsplit("/", 1)[-1]
            name = base.rsplit(".", 1)[0] or base
            root = slugify(name) or "sound"
            sid = root
            n = 2
            while sid in used_ids:
                sid = f"{root}-{n}"
                n += 1
            saved = self.sound_store.save(Sound(
                id=sid, name=name, icon=icon,
                node_id=node_id, file_path=path,
                output_device=output_device or "default",
                volume=float(volume), start_time=float(start_time),
                loop=bool(loop), loop_count=int(loop_count)))
            used_ids.add(saved.id)
            have_paths.add((node_id, path))
            added.append(path)
            added_ids.append(saved.id)
        # Optionally drop the whole batch into a playlist, in the order
        # the files were given. Replaces the old `group` argument — with
        # many-to-many membership there is nothing to set on the sound.
        if playlist_id and added_ids:
            for sid in added_ids:
                self.playlist_store.add_item(playlist_id, sid)
        return {"success": True, "added": len(added),
                "skipped": len(skipped), "sounds": self.list_sounds()}

    def reprobe_sound(self, sound_id: str) -> Dict[str, Any]:
        """Re-measure a clip whose audio file was replaced on disk.

        A plain save keeps a duration it already has, so that editing a
        name or volume doesn't re-read the file each time. This is the
        explicit "the file changed under it" path.
        """
        snd = self.sound_store.reprobe(sound_id)
        if snd is None:
            return {"success": False, "message": "Sound not found"}
        return {"success": True, "sound": snd.to_dict(),
                "duration": snd.duration}

    def delete_sound(self, sound_id: str) -> Dict[str, Any]:
        if not self.sound_store.delete(sound_id):
            return {"success": False, "message": "Sound not found"}
        self.playlist_store.forget_item("sounds", sound_id)
        return {"success": True}

    def reorder_sounds(self, ordered_ids: List[str]) -> Dict[str, Any]:
        return {"success": True,
                "sounds": self.sound_store.reorder(ordered_ids or [])}

    # ── playlists ───────────────────────────────────────────────────
    #
    # Playlists are the many-to-many replacement for the per-item
    # `group` string. They are per-kind (an animations playlist holds
    # animation ids only) and they own both membership and order, so an
    # item can sit at a different slot in each playlist it belongs to.
    # See saint_server.animation.models.Playlist.

    def list_playlists(self, kind: Optional[str] = None) -> List[Dict[str, Any]]:
        return self.playlist_store.list(kind)

    def save_playlist(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Create or update a playlist (name / icon / members / order)."""
        from saint_server.animation.models import Playlist
        try:
            pl = Playlist.from_dict(payload)
        except (KeyError, ValueError, TypeError) as e:
            return {"success": False, "message": f"Invalid playlist payload: {e}"}
        saved = self.playlist_store.save(pl)
        return {"success": True, "playlist": saved.to_dict()}

    def delete_playlist(self, playlist_id: str) -> Dict[str, Any]:
        """Delete the playlist. Its members are NOT deleted — they just
        stop being in it, and fall back to Ungrouped if this was their
        only playlist."""
        if not self.playlist_store.delete(playlist_id):
            return {"success": False, "message": "Playlist not found"}
        return {"success": True}

    def playlist_add_item(self, playlist_id: str, item_id: str,
                          index: Optional[int] = None) -> Dict[str, Any]:
        """Add (or move, if already a member) an item at `index`.

        The item is validated against its playlist's kind so a drag can't
        land a sound in an animations playlist — the UI keeps them in
        separate sections, but the WS action is reachable directly.
        """
        pl = self.playlist_store.get(playlist_id)
        if pl is None:
            return {"success": False, "message": "Playlist not found"}
        if not self._item_exists(pl.kind, item_id):
            return {"success": False,
                    "message": f"No {pl.kind[:-1]} with id {item_id!r}"}
        updated = self.playlist_store.add_item(playlist_id, item_id, index)
        return {"success": True, "playlist": updated.to_dict()}

    def playlist_remove_item(self, playlist_id: str,
                             item_id: str) -> Dict[str, Any]:
        updated = self.playlist_store.remove_item(playlist_id, item_id)
        if updated is None:
            return {"success": False, "message": "Playlist not found"}
        return {"success": True, "playlist": updated.to_dict()}

    def reorder_playlist_items(self, playlist_id: str,
                               ordered_ids: List[str]) -> Dict[str, Any]:
        updated = self.playlist_store.reorder_items(playlist_id,
                                                    ordered_ids or [])
        if updated is None:
            return {"success": False, "message": "Playlist not found"}
        return {"success": True, "playlist": updated.to_dict()}

    def reorder_playlists(self, kind: str,
                          ordered_ids: List[str]) -> Dict[str, Any]:
        from saint_server.animation.models import PLAYLIST_KINDS
        if kind not in PLAYLIST_KINDS:
            return {"success": False, "message": f"Unknown kind {kind!r}"}
        return {"success": True,
                "playlists": self.playlist_store.reorder(kind, ordered_ids or [])}

    def _playlist_named(self, name: str, kind: str):
        """Find-or-create a playlist by display name within one kind.

        Returns a tiny adapter with ``add_item_id`` so callers that are
        filing items in a loop don't re-resolve the playlist each pass.
        Matching is case-insensitive on the name: an operator who already
        has a "Base" playlist should not end up with a second "base".
        """
        from saint_server.animation.models import Playlist

        wanted = name.strip().lower()
        found = next((p for p in self.playlist_store.list(kind)
                      if p["name"].strip().lower() == wanted), None)
        playlist_id = found["id"] if found else self.playlist_store.save(
            Playlist(id="", name=name.strip(), kind=kind)).id

        store = self.playlist_store

        class _Filer:
            def add_item_id(self, item_id: str) -> None:
                store.add_item(playlist_id, item_id)

        return _Filer()

    def _item_exists(self, kind: str, item_id: str) -> bool:
        store = {
            "animations": self.animation_store,
            "poses": self.pose_store,
            "sounds": self.sound_store,
        }.get(kind)
        if store is None:
            return False
        return store.get(item_id) is not None

    def list_audio_nodes(self) -> List[Dict[str, Any]]:
        """Nodes that can play soundboard entries.

        Two kinds of audio target:
          - the synthetic ``host_controller`` — plays in-process on the
            server host via VLC/ALSA (see server_node._dispatch_host_soundboard);
          - adopted ``raspberrypi`` firmware nodes — play on the node via
            its own VLC/ALSA stack.
        The host controller is listed first so it's the obvious default.
        """
        out = []
        host = self.state.adopted_nodes.get(HOST_CONTROLLER_NODE_ID)
        if host is not None:
            out.append({
                "node_id": HOST_CONTROLLER_NODE_ID,
                "name": host.display_name or "Host Controller",
                "online": host.online,
            })
        rpi = []
        for node_id, node in self.state.adopted_nodes.items():
            if node_id == HOST_CONTROLLER_NODE_ID:
                continue
            if node.chip_family == "raspberrypi":
                rpi.append({
                    "node_id": node_id,
                    "name": node.display_name or node.hardware_model or node_id,
                    "online": node.online,
                })
        rpi.sort(key=lambda n: n["name"].lower())
        return out + rpi

    def host_default_audio_device(self) -> str:
        """Resolve the host's default soundboard output from its
        ``audio_mixer`` peripheral — the operator-designated output card.

        The audio_mixer is the single place the operator picks *which*
        card the host plays out of (and controls its volume/mute), so a
        sound left on ``default`` follows it. Returns an ALSA device
        string like ``hw:3,0``; falls back to ``default`` when no
        audio_mixer is configured (then it's plain ALSA default)."""
        node = self.state.adopted_nodes.get(HOST_CONTROLLER_NODE_ID)
        if node and node.peripheral_config:
            for p in node.peripheral_config.peripherals:
                if p.type == "audio_mixer":
                    try:
                        return f"hw:{int((p.params or {}).get('card'))},0"
                    except (TypeError, ValueError):
                        break
        return "default"

    def resolve_sound_play(self, sound_id: str) -> Optional[Dict[str, Any]]:
        """Resolve a saved sound into ``{node_id, args}`` for a play
        command, or ``None`` if the sound doesn't exist."""
        snd = self.sound_store.get(sound_id)
        if snd is None:
            return None
        return {
            "node_id": snd.node_id,
            "args": {
                "path": snd.file_path,
                "device": snd.output_device or "default",
                "volume": snd.volume,
                "start_time_s": snd.start_time,
                "loop": snd.loop,
                "loop_count": snd.loop_count,
            },
        }

    def preview_animation_frame(self, values: List[Dict[str, Any]],
                                triggers: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Dispatch one editor-sampled animation frame live, WITHOUT a
        running player.

        Powers the editor's "Live Preview" toggle: as the operator
        scrubs the timeline or edits a keyframe at the playhead, the
        client samples every value track at the current time and sends
        the frame here, plus any trigger keyframes crossed since the
        last frame. Values go through ``apply_animation_frame`` — the
        same batched call the player uses — and triggers through the
        same ROS bridge and peripheral sender, so a previewed frame is
        wire-identical to a played one. Nothing is persisted and no
        player is created.

        ``values`` entries: ``{target_kind, value, id?, target?}``
          * ``urdf_joint`` → keyed by ``id`` (the joint name)
          * ``ws_input``   → keyed by ``(target[0], target[1])``
        ``triggers`` entries: ``{target_kind, target, value}`` —
        ws_input / topic / peripheral_command, mirroring
        AnimationPlayer._dispatch_trigger.
        """
        if self._routing_evaluator is None:
            return {"success": False, "message": "Routing evaluator not ready"}
        ev = self._routing_evaluator
        applied = 0

        # Collect the whole frame first, then apply it in ONE batch —
        # the same thing the player and the pose fan-out do.
        #
        # Applying per value instead re-evaluated every touched sheet and
        # rebuilt the UI snapshot once per setpoint, so an N-track frame
        # cost N evaluations. Two things went wrong with that. Each
        # intermediate evaluation saw this tick's values for the tracks
        # already applied and the PREVIOUS tick's for the rest, so every
        # channel a sheet computes from more than one track was briefly
        # driven with a blend of two frames that never existed. And at
        # the editor's ~30 Hz preview rate the evaluation count ran well
        # past what the per-setpoint path sustains, so frames queued and
        # the rig lagged behind the playhead.
        joint_values: Dict[str, float] = {}
        ws_values: Dict[Tuple[str, str], float] = {}
        for v in values or []:
            kind = v.get("target_kind", "urdf_joint")
            try:
                val = float(v.get("value") or 0.0)
            except (TypeError, ValueError):
                val = 0.0
            if kind == "ws_input":
                tgt = v.get("target") or []
                if len(tgt) >= 2:
                    ws_values[(tgt[0], tgt[1])] = val
            else:  # urdf_joint
                jid = v.get("id")
                if jid:
                    joint_values[jid] = val

        if joint_values or ws_values:
            try:
                if ev.apply_animation_frame(joint_values, ws_values):
                    applied = len(joint_values) + len(ws_values)
            except Exception as e:
                if self.logger:
                    self.logger.warn(
                        f"preview_animation_frame apply failed: {e}")

        for t in triggers or []:
            kind = t.get("target_kind", "ws_input")
            tgt = t.get("target") or []
            val = t.get("value")
            try:
                if kind == "ws_input" and len(tgt) >= 2:
                    ev.set_ws_input(tgt[0], tgt[1], float(val or 0.0))
                elif kind == "topic" and len(tgt) >= 2 and self._ros_bridge is not None:
                    self._ros_bridge.set_topic_channel(
                        tgt[0], tgt[1], float(val or 0.0), "_animation_preview")
                elif kind == "peripheral_command" and len(tgt) >= 2 \
                        and self._peripheral_command_sender is not None:
                    if isinstance(val, dict):
                        cmd = str(val.get("command") or "")
                        args = val.get("args") or {}
                        args = dict(args) if isinstance(args, dict) else {}
                    elif isinstance(val, str):
                        cmd, args = "play_file", {"filename": val}
                    else:
                        cmd, args = "", {}
                    if cmd:
                        self._peripheral_command_sender(tgt[0], tgt[1], cmd, args)
            except Exception as e:
                if self.logger:
                    self.logger.warn(f"preview_animation_frame trigger failed: {e}")

        return {"success": True, "applied": applied}

    async def start_animation(self, animation_id: str,
                              loop: Optional[bool] = None) -> Dict[str, Any]:
        if self._animation_registry is None:
            return {"success": False, "message": "Animation engine not ready"}
        anim = self.animation_store.get(animation_id)
        if anim is None:
            return {"success": False, "message": "Animation not found"}
        await self._animation_registry.start(anim, loop=loop)
        return {"success": True}

    async def stop_animation(self, animation_id: str) -> Dict[str, Any]:
        if self._animation_registry is None:
            return {"success": False, "message": "Animation engine not ready"}
        ok = await self._animation_registry.stop(animation_id)
        if not ok:
            return {"success": False, "message": "Animation not running"}
        # Clear cached values so peripherals settle.
        if self._routing_evaluator is not None:
            try:
                self._routing_evaluator.clear_animation_value(animation_id)
            except Exception:
                pass
        return {"success": True}

    def pause_animation(self, animation_id: str) -> Dict[str, Any]:
        if self._animation_registry is None:
            return {"success": False, "message": "Animation engine not ready"}
        ok = self._animation_registry.pause(animation_id)
        return {"success": ok, "message": "" if ok else "Animation not running"}

    def resume_animation(self, animation_id: str) -> Dict[str, Any]:
        if self._animation_registry is None:
            return {"success": False, "message": "Animation engine not ready"}
        ok = self._animation_registry.resume(animation_id)
        return {"success": ok, "message": "" if ok else "Animation not running"}

    def seek_animation(self, animation_id: str, t: float) -> Dict[str, Any]:
        if self._animation_registry is None:
            return {"success": False, "message": "Animation engine not ready"}
        ok = self._animation_registry.seek(animation_id, t)
        return {"success": ok, "message": "" if ok else "Animation not running"}

    def animation_state(self) -> List[Dict[str, Any]]:
        if self._animation_registry is None:
            return []
        return self._animation_registry.state()

    def _coerce_position(self, position) -> Tuple[int, int]:
        if isinstance(position, (list, tuple)) and len(position) >= 2:
            try:
                return (int(position[0]), int(position[1]))
            except (TypeError, ValueError):
                pass
        return (0, 0)

    def _validate_sheet_owner(self, node_id: str) -> Optional[str]:
        """A sheet must belong to a known adopted controller node."""
        if node_id in self.state.adopted_nodes:
            return None
        return f"Unknown sheet owner '{node_id}'"

    def _validate_wire(self, sheet_node_id: str,
                       source: RouteEndpoint,
                       sink: RouteEndpoint) -> Optional[str]:
        """Verify that both endpoints exist on `sheet_node_id` or system-wide."""
        for ep, role in ((source, "source"), (sink, "sink")):
            if ep.kind == "input":
                if not ep.parts:
                    return f"{role}: input endpoint needs [input_id]"
                sheet = self.state.system_routing.get_sheet(sheet_node_id)
                if not sheet.find_input(ep.parts[0]):
                    return f"{role}: input '{ep.parts[0]}' not on this sheet"
            elif ep.kind == "ws_input":
                if role == "sink":
                    return "sink: ws_input cannot be a wire sink (controller-driven only)"
                if not ep.parts:
                    return f"{role}: ws_input endpoint needs [ws_input_id]"
                sheet = self.state.system_routing.get_sheet(sheet_node_id)
                if not sheet.find_ws_input(ep.parts[0]):
                    return f"{role}: ws_input '{ep.parts[0]}' not on this sheet"
            elif ep.kind == "operator":
                if len(ep.parts) < 2:
                    return f"{role}: operator endpoint needs [op_id, pin]"
                sheet = self.state.system_routing.get_sheet(sheet_node_id)
                op_node = sheet.find_operator(ep.parts[0])
                if not op_node:
                    return f"{role}: operator '{ep.parts[0]}' not on this sheet"
                op_type = OPERATOR_CATALOG.get(op_node.op)
                pin = ep.parts[1]
                if op_type:
                    valid_pins = {"out"} | {i.id for i in op_type.inputs}
                    if pin not in valid_pins:
                        return f"{role}: operator '{op_node.op}' has no pin '{pin}'"
            elif ep.kind == "peripheral":
                if len(ep.parts) < 3:
                    return f"{role}: peripheral endpoint needs [node_id, peripheral_id, channel_id]"
                node_id, pid, channel_id = ep.parts[0], ep.parts[1], ep.parts[2]
                # Hard-scope: peripheral sinks must live on the sheet owner.
                if role == "sink" and node_id != sheet_node_id:
                    return ("sink: peripheral sink must be on the sheet's controller "
                            f"(got '{node_id}', sheet is '{sheet_node_id}')")
                node = self.state.adopted_nodes.get(node_id)
                if not node or not node.peripheral_config:
                    return f"{role}: unknown node '{node_id}'"
                peripheral = node.peripheral_config.get(pid)
                if not peripheral:
                    return f"{role}: peripheral '{pid}' not on node '{node_id}'"
                t = self.peripheral_catalog.get(peripheral.type)
                if t and not any(c.id == channel_id for c in t.channels):
                    return f"{role}: channel '{channel_id}' not on peripheral type '{peripheral.type}'"
            elif ep.kind == "widget":
                if len(ep.parts) < 2:
                    return f"{role}: widget endpoint needs [widget_id, input_id]"
                # Widgets are sheet-local; only the active sheet's widgets
                # are addressable from its wires.
                sheet = self.state.system_routing.get_sheet(sheet_node_id)
                widget = sheet.find_widget(ep.parts[0])
                if not widget:
                    return f"{role}: widget '{ep.parts[0]}' not on this sheet"
                t = self.widget_catalog.get(widget.type)
                if t and not any(i.id == ep.parts[1] for i in t.inputs):
                    return f"{role}: input '{ep.parts[1]}' not on widget type '{widget.type}'"
            elif ep.kind == "output":
                # Outputs are taps: valid as both sink (the value that
                # gets ROS-published) and source (re-emit to downstream
                # peripherals on the same sheet).
                if not ep.parts:
                    return f"{role}: output endpoint needs [output_id]"
                sheet = self.state.system_routing.get_sheet(sheet_node_id)
                if not sheet.find_output(ep.parts[0]):
                    return f"{role}: output '{ep.parts[0]}' not on this sheet"
            else:
                return f"{role}: unknown endpoint kind '{ep.kind}'"
        # Source pins must be outputs; sink pins must be inputs.
        if source.kind == "peripheral":
            # Outgoing peripheral data: channel must have dir == "in" (peripheral produces it).
            err = self._check_peripheral_dir(source, expected_dir="in", role="source")
            if err:
                return err
        if sink.kind == "peripheral":
            err = self._check_peripheral_dir(sink, expected_dir="out", role="sink")
            if err:
                return err
        return None

    def _check_peripheral_dir(self, ep: RouteEndpoint, expected_dir: str,
                              role: str) -> Optional[str]:
        node = self.state.adopted_nodes.get(ep.parts[0])
        if not node or not node.peripheral_config:
            return None
        peripheral = node.peripheral_config.get(ep.parts[1])
        if not peripheral:
            return None
        t = self.peripheral_catalog.get(peripheral.type)
        if not t:
            return None
        chan = next((c for c in t.channels if c.id == ep.parts[2]), None)
        if not chan:
            return None
        if chan.dir != expected_dir:
            actual = "sensor reading" if chan.dir == "in" else "actuator command"
            return f"{role}: '{chan.display}' is a {actual}; cannot wire it that way"
        return None

    def add_widget(self, node_id: str, type_id: str,
                   label: Optional[str] = None,
                   position: Optional[List[int]] = None,
                   params: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """Attach a widget to a controller sheet. Widgets are sheet-local —
        a wire on that sheet can drive the widget's input pin and the
        operator sees the value on their dashboard."""
        if type_id not in self.widget_catalog:
            return {"success": False, "message": f"Unknown widget type '{type_id}'"}
        err = self._validate_sheet_owner(node_id)
        if err:
            return {"success": False, "message": err}
        pos = self._coerce_position(position)
        sheet = self.state.system_routing.get_sheet(node_id)
        # Widget IDs must be unique across ALL sheets — see the
        # comment in NodeSheet.add_widget. Pre-populate the conflict
        # set from every other sheet's widgets so the new ID skips
        # any number already used elsewhere.
        other_widget_ids = {
            w.id
            for sid, s in self.state.system_routing.sheets.items()
            if sid != node_id
            for w in s.widgets
        }
        widget = sheet.add_widget(
            type_id=type_id,
            label=label or self.widget_catalog[type_id].label,
            position=pos,
            params=params,
            extra_existing_ids=other_widget_ids,
        )
        self.state.system_routing.bump_version()
        self._save_system_routing()
        self._log_activity(f"Added widget {widget.id} ({widget.type}) to {node_id}", "info")
        return {"success": True, "widget": widget.to_dict()}

    def reorder_widgets(self, widget_ids: List[str]) -> Dict[str, Any]:
        """Persist the dashboard card order.

        `widget_ids` is the full ordered list as the operator arranged it.
        Ids are unique across every sheet, so this walks all sheets and
        stamps each widget's `dashboard_order` from its index.

        Widgets the caller didn't mention keep a stable place AFTER the
        ordered ones rather than jumping to the front — a client with a
        stale widget list shouldn't silently reshuffle cards it never
        knew about.
        """
        if not isinstance(widget_ids, list):
            return {"success": False, "message": "widget_ids must be a list"}

        rank = {}
        for i, wid in enumerate(widget_ids):
            if isinstance(wid, str) and wid and wid not in rank:
                rank[wid] = i

        known = {
            w.id
            for s in self.state.system_routing.sheets.values()
            for w in s.widgets
        }
        unknown = [w for w in rank if w not in known]
        if unknown:
            return {"success": False,
                    "message": f"Unknown widget id(s): {', '.join(sorted(unknown))}"}

        # Unlisted widgets sort after the listed ones, in their existing
        # relative order.
        tail = len(rank)
        updated = 0
        for sheet in self.state.system_routing.sheets.values():
            for w in sheet.widgets:
                new_order = rank.get(w.id)
                if new_order is None:
                    new_order = tail
                    tail += 1
                if w.dashboard_order != new_order:
                    w.dashboard_order = new_order
                    updated += 1

        self.state.system_routing.bump_version()
        self._save_system_routing()
        return {"success": True, "reordered": updated, "count": len(known)}

    # =========================================================================
    # Persistence (system_routing.yaml + per-node peripheral configs)
    # =========================================================================

    def _save_system_routing(self) -> None:
        os.makedirs(self.config_dir, exist_ok=True)
        try:
            with open(self.system_routing_path, "w") as f:
                yaml.dump(self.state.system_routing.to_dict(),
                          f, default_flow_style=False, sort_keys=False)
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to save system routing: {e}")
        # Let the live evaluator refresh its subscriptions + graph snapshot.
        if self._routing_evaluator is not None:
            try:
                self._routing_evaluator.reconcile(self.state.system_routing)
            except Exception as e:
                if self.logger:
                    self.logger.error(f"Routing evaluator reconcile failed: {e}")

    def set_channel_arbiter(self, arbiter) -> None:
        """Share the channel arbiter (see channel_arbiter.py). Used to
        invalidate cached channel state when something moves the
        hardware outside the normal write path — a node reconnecting, a
        config sync re-homing servos."""
        self._channel_arbiter = arbiter

    def _invalidate_channel_cache(self, node_id: str, reason: str) -> None:
        """Forget cached channel state for a node after something moved
        its hardware outside the write path. No-ops before the arbiter
        is wired (early startup, and tests that construct a bare
        StateManager)."""
        if self._channel_arbiter is None:
            return
        dropped = self._channel_arbiter.invalidate_node(node_id, reason=reason)
        if dropped and self.logger:
            self.logger.info(
                f"Channel cache: dropped {dropped} entries for {node_id} "
                f"({reason})")

    def set_routing_evaluator(self, evaluator) -> None:
        """Wire the live routing evaluator (set by server_node once the
        ROS bridge is up). Triggers an initial reconcile so the evaluator
        picks up sheets persisted on disk."""
        self._routing_evaluator = evaluator
        if evaluator is not None:
            try:
                evaluator.reconcile(self.state.system_routing)
            except Exception as e:
                if self.logger:
                    self.logger.error(f"Initial routing reconcile failed: {e}")
        self._maybe_init_animation_registry()

    def set_ros_bridge(self, bridge) -> None:
        """Wire the ROS bridge so the animation player can publish to
        topics for trigger-track dispatch. Safe to call before the
        evaluator is set; the registry waits for both."""
        self._ros_bridge = bridge
        self._maybe_init_animation_registry()

    def set_peripheral_command_sender(self, sender) -> None:
        """Wire the out-of-band peripheral-command publisher (normally
        ``ServerNode.send_peripheral_command``) so animation trigger
        tracks of kind ``peripheral_command`` can fire string-arg
        commands at specific timecodes (e.g. audio_player.play_file).
        Optional — if unset, trigger tracks of that kind are dropped
        with a warn at fire time."""
        self._peripheral_command_sender = sender

    def _maybe_init_animation_registry(self) -> None:
        if self._animation_registry is not None:
            return
        if self._routing_evaluator is None or self._ros_bridge is None:
            return
        from saint_server.animation.player import AnimationPlayerRegistry
        evaluator = self._routing_evaluator
        bridge = self._ros_bridge

        def estop_active() -> bool:
            return bool(getattr(evaluator, "_estop_active", False))

        self._animation_registry = AnimationPlayerRegistry(
            board=_BoardControl(self),
            set_urdf_joint_value=evaluator.set_urdf_joint_value,
            set_ws_input=evaluator.set_ws_input,
            set_topic_channel=bridge.set_topic_channel,
            apply_frame=evaluator.apply_animation_frame,
            estop_active=estop_active,
            send_peripheral_command=self._peripheral_command_sender,
            pose_source=self._make_pose_lookup,
            neutral_source=self.rig_neutral,
            logger=self.logger,
        )

    def _make_pose_lookup(self):
        """Fresh pose lookup for one playback (see frame.make_pose_lookup).

        Called per start rather than cached on the registry, so a pose
        edited between runs takes effect on the next start while staying
        stable for the duration of a performance.
        """
        from saint_server.animation.frame import make_pose_lookup
        return make_pose_lookup(self.pose_store)

    def rig_neutral(self) -> Dict[str, float]:
        """Joint values of the rig's declared neutral pose, or {}.

        The base a pose track blends up from on joints nothing else has
        touched. Absent a rig (or a ``neutral_pose`` in it) an untouched
        joint starts at 0, which is the midpoint of its travel.
        """
        if self.robot_store is None:
            return {}
        rig = self.robot_store.load_rig()
        name = rig.settings.neutral_pose if rig else ""
        if not name:
            return {}
        pose = self.pose_store.get(name)
        return pose.joint_values() if pose else {}

    def set_routing_estop_active(self, active: bool) -> None:
        """Mirror the system-wide e-stop latch into the routing evaluator
        so peripheral/output sink writes get suppressed at the routing
        layer. Called from the websocket handler's `estop` action right
        after it fans out the per-node estop calls.
        """
        if self._routing_evaluator is None:
            return
        try:
            self._routing_evaluator.set_estop_active(active)
        except Exception as e:
            if self.logger:
                self.logger.error(f"Routing estop gate toggle failed: {e}")

    def lookup_peripheral_type(self, node_id: str, peripheral_id: str) -> str:
        """Resolve a peripheral type id for the firmware command payload."""
        node = self.state.adopted_nodes.get(node_id)
        if not node or not node.peripheral_config:
            return ""
        peripheral = node.peripheral_config.get(peripheral_id)
        return peripheral.type if peripheral else ""

    def _load_system_routing(self) -> None:
        if not os.path.exists(self.system_routing_path):
            return
        try:
            with open(self.system_routing_path, "r") as f:
                data = yaml.safe_load(f) or {}
            self.state.system_routing = SystemRouting.from_dict(data)
            migrated = self._migrate_state_only_inputs_to_ws()
            if migrated > 0:
                # Persist the rewrite so the next load is clean.
                self._save_system_routing()
            if self.logger:
                routing = self.state.system_routing
                wire_count = sum(len(s.wires) for s in routing.sheets.values())
                self.logger.info(
                    f"Loaded system routing: {len(routing.sheets)} sheets, "
                    f"{wire_count} wires"
                )
                if migrated > 0:
                    self.logger.info(
                        f"Migrated {migrated} state-only ROS input(s) to WS inputs"
                    )
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to load system routing: {e}")

    def _state_only_endpoint_paths(self) -> set:
        """Parse endpoints.yaml directly to find paths with state_type but no command_type.

        We read the yaml ourselves (rather than depending on the ROS
        bridge being initialized) so migration runs early in startup.
        Returns an empty set if the yaml isn't reachable — migration
        becomes a no-op rather than failing.
        """
        try:
            from pathlib import Path as _Path
            yaml_path = _Path(__file__).resolve().parents[1] / "ros_bridge" / "endpoints.yaml"
            with open(yaml_path, "r") as f:
                doc = yaml.safe_load(f) or {}
            endpoints = (doc.get("endpoints") or {})
            return {path for path, cfg in endpoints.items()
                    if isinstance(cfg, dict)
                    and cfg.get("state_type")
                    and not cfg.get("command_type")}
        except Exception:
            return set()

    def _migrate_state_only_inputs_to_ws(self) -> int:
        """Convert old controller-write InputNodes to WebSocketInputNodes.

        Pre-WS-input bindings addressed state-only ROS endpoints
        (e.g. /saint/track left_velocity) via the bridge's mirror loop.
        That path is gone — those nodes need to be WS inputs now or the
        sheet silently breaks. We identify them by checking which paths
        in endpoints.yaml are state-only and rewriting any InputNode
        that targets one. Wires keep working because we reuse the input
        id and flip the wire's source.kind from "input" to "ws_input".
        """
        state_only_paths = self._state_only_endpoint_paths()
        if not state_only_paths:
            return 0
        migrated = 0
        for sheet in self.state.system_routing.sheets.values():
            to_migrate = [inp for inp in sheet.inputs
                          if inp.topic in state_only_paths]
            if not to_migrate:
                continue
            keep_inputs = [inp for inp in sheet.inputs
                           if inp.topic not in state_only_paths]
            migrated_ids = {inp.id for inp in to_migrate}
            for inp in to_migrate:
                # Reuse the original id so existing wires keep pointing
                # at the same node — only their source.kind flips.
                # kind="state" tags this as an echo of a state-only
                # ROS endpoint, NOT a real controller target. The
                # binding picker filters these out so operators don't
                # accidentally bind a joystick to a sensor reading.
                label = inp.label or f"{inp.topic}{('.' + inp.field) if inp.field else ''}"
                from saint_server.peripheral_model import WebSocketInputNode
                sheet.ws_inputs.append(WebSocketInputNode(
                    id=inp.id, label=label, position=inp.position,
                    kind="state",
                ))
                migrated += 1
            sheet.inputs = keep_inputs
            for w in sheet.wires:
                if (w.source.kind == "input"
                        and w.source.parts and w.source.parts[0] in migrated_ids):
                    w.source.kind = "ws_input"
        if migrated > 0:
            self.state.system_routing.bump_version()
        return migrated

    # =========================================================================
    # Host controller — the Pi server modeled as a built-in node
    # =========================================================================

    def _migrate_missing_board_ids(self) -> None:
        """Assign a default board_id to already-adopted nodes that don't have one.

        Picks the first board matching the node's chip family. If the
        firmware didn't report chip_family either (older firmware), tries
        to infer from node_id prefix or hardware_model. Logs every
        defaulted node so the operator can re-pick from Settings later.
        """
        for node in self.state.adopted_nodes.values():
            if node.board_id:
                continue
            # Infer chip family if missing.
            chip = node.chip_family
            if not chip:
                if node.node_id.startswith("rp2040_") or "RP2040" in node.hardware_model:
                    chip = "rp2040"
                elif node.node_id.startswith("teensy41_") or "Teensy 4" in node.hardware_model:
                    chip = "teensy41"
            if not chip:
                continue   # nothing to default to
            node.chip_family = chip
            default_board = self.board_config.default_board_for_chip(chip)
            if not default_board:
                if self.logger:
                    self.logger.warning(
                        f"No board YAML for chip '{chip}' — node {node.node_id} "
                        f"has no pin layout. Add a board in Settings → Boards."
                    )
                continue
            node.board_id = default_board.board_id
            if self.logger:
                self.logger.info(
                    f"Migrated node {node.node_id} → default board '{node.board_id}' "
                    f"(chip {chip}); operator can change in Settings → Boards."
                )

    def _ensure_host_controller_node(self) -> None:
        """Create the synthetic host_controller node + system_monitor peripheral.

        Always present in the adopted node list. Cannot be removed. Its
        peripherals are otherwise normal — the operator can route the
        system_monitor channels into dashboard widgets like any other
        peripheral channel.
        """
        node = self.state.adopted_nodes.get(HOST_CONTROLLER_NODE_ID)
        if not node:
            node = NodeInfo(
                node_id=HOST_CONTROLLER_NODE_ID,
                display_name="Host Controller",
                role="host",
                hardware_model="Server",
                mac_address="",
                ip_address="127.0.0.1",
                state="ACTIVE",
                online=True,
                last_seen=time.time(),
            )
            self.state.adopted_nodes[HOST_CONTROLLER_NODE_ID] = node

        # Make sure the system_monitor peripheral is present (built-in,
        # not operator-removable).
        if not node.peripheral_config:
            node.peripheral_config = NodePeripheralConfig()
        if not node.peripheral_config.get(HOST_CONTROLLER_PERIPHERAL_ID):
            node.peripheral_config.peripherals.append(PeripheralInstance(
                id=HOST_CONTROLLER_PERIPHERAL_ID,
                type="system_monitor",
                label="System Monitor",
                pins={},        # builtin pin_kind, no hardware pins
                params={},
                builtin=True,
            ))
            node.peripheral_config.sync_status = "synced"  # nothing to push to firmware
            node.peripheral_config.version += 1

        # Synthetic capability stub so the UI's pin-availability sidebar
        # has something to render when navigating into the host node.
        if not node.capabilities:
            node.capabilities = NodeCapabilities(
                node_id=HOST_CONTROLLER_NODE_ID,
                pins=[],          # no operator-configurable physical pins yet
                reserved_pins=[],
                uart_pairs=[],
                last_updated=time.time(),
            )

    def set_host_peripheral_reconcile_callback(
            self, cb: Callable[[List[Dict[str, Any]]], None]) -> None:
        """Server wires this to HostPeripheralManager.reconcile so the
        BLE driver set follows host_controller config changes. We also
        fire it once immediately with the currently-loaded config so
        BMSes from /etc/saint-os/nodes/host_controller.yaml come up at
        startup without waiting for an operator edit."""
        self._host_peripheral_reconcile_cb = cb
        self._maybe_notify_host_peripheral_change(HOST_CONTROLLER_NODE_ID)

    def _maybe_notify_host_peripheral_change(self, node_id: str) -> None:
        """Fire the reconcile callback when host_controller's peripheral
        list changes. No-op for other nodes (their drivers run in their
        own Pi-node firmware, not in-process)."""
        if node_id != HOST_CONTROLLER_NODE_ID:
            return
        cb = self._host_peripheral_reconcile_cb
        if cb is None:
            return
        node = self.state.adopted_nodes.get(HOST_CONTROLLER_NODE_ID)
        peripherals: List[Dict[str, Any]] = []
        if node and node.peripheral_config:
            for p in node.peripheral_config.peripherals:
                if p.builtin:
                    continue  # system_monitor handled separately
                peripherals.append({
                    "id": p.id,
                    "type": p.type,
                    "pins": dict(p.pins or {}),
                    "params": dict(p.params or {}),
                })
        try:
            cb(peripherals)
        except Exception as e:
            if self.logger:
                self.logger.warning(
                    f"host_peripheral reconcile callback raised: {e}")

    def update_host_controller_runtime(self) -> None:
        """Push current system metrics into the host node's runtime_state.

        Called from the broadcast loop. Values land on channel keys
        ``(system_monitor, cpu_usage)`` etc. so the dashboard's routing
        engine resolves them by the same peripheral/channel identifiers
        the routing graph uses — no virtual GPIO indirection.

        Also keeps the NodeInfo fields the Overview card reads
        (``online``, ``state``, ``cpu_temp``, ``uptime_seconds``,
        ``firmware_version``, ``ip_address``) in sync with the same
        metrics so the host node's detail page shows the same picture
        as the System Status dashboard card.
        """
        node = self.state.adopted_nodes.get(HOST_CONTROLLER_NODE_ID)
        if not node:
            return
        if not node.runtime_state:
            node.runtime_state = NodeRuntimeState(node_id=HOST_CONTROLLER_NODE_ID)

        # Synthetic node — the server IS the host. The YAML loader
        # initialises every adopted node at online=False/last_seen=0
        # (correct default for real Pi nodes that may not be on the
        # net at server start), so without these overwrites the
        # host_controller's connection dot stays gray and its state
        # stays "UNKNOWN" forever.
        node.last_seen = time.time()
        node.online = True
        node.state = "ACTIVE"
        node.going_offline_at = None

        try:
            cpu_usage = psutil.cpu_percent(interval=None)
            mem_usage = psutil.virtual_memory().percent
        except Exception:
            cpu_usage = 0.0
            mem_usage = 0.0
        cpu_temp = _read_cpu_temp()
        throttle = _read_throttle_status()
        uptime_s = float(int(time.time() - self.state.start_time))

        # Mirror the metrics into NodeInfo so list_adopted (which
        # feeds the Overview card) carries them. Channel-side push
        # below still happens — that drives the Live tab + routing.
        node.cpu_usage = float(cpu_usage)
        node.memory_usage = float(mem_usage)
        if cpu_temp is not None:
            node.cpu_temp = float(cpu_temp)
        node.uptime_seconds = int(uptime_s)

        # Server version + reachable address. Both are stable for the
        # process lifetime, so backfill only when the YAML-loaded
        # defaults are still in place. firmware_version here means
        # the installed saint-os version (NOT a Pi-node firmware
        # build); the Overview card just calls it "Firmware".
        if not node.firmware_version or node.firmware_version == "0.0.0":
            try:
                v = _read_installed_version_info()
                if v and v.get("version") and v["version"] != "unknown":
                    node.firmware_version = v["version"]
                if v and v.get("built_at"):
                    node.firmware_build = v["built_at"]
            except Exception:
                pass
        if not node.ip_address or node.ip_address == "127.0.0.1":
            node.ip_address = _resolve_server_ip()

        readings = {
            'cpu_usage': float(cpu_usage),
            'mem_usage': float(mem_usage),
            'uptime':    uptime_s,
            # Throttle is a digital signal — 1 when any throttle bit is
            # current or historical, 0 when clean.
            'throttle':  1.0 if (throttle and throttle.get('status') != 'ok') else 0.0,
        }
        if cpu_temp is not None:
            readings['cpu_temp'] = float(cpu_temp)

        # WiFi telemetry — only published when the host actually has a
        # wireless interface and `iw` is installed, so dev boxes and
        # ethernet-only deployments don't end up with bogus zeros on
        # their dashboard widgets. See wifi_stats.py for what each
        # value means and how it's collected.
        try:
            from saint_server.wifi_stats import collect as _wifi_collect
            wifi = _wifi_collect()
            if wifi.signal_dbm is not None:
                readings['wifi_signal'] = float(wifi.signal_dbm)
            if wifi.retry_pct is not None:
                readings['wifi_retry_pct'] = float(wifi.retry_pct)
            if wifi.noise_dbm is not None:
                readings['wifi_noise'] = float(wifi.noise_dbm)
            if wifi.bitrate_mbps is not None:
                readings['wifi_bitrate'] = float(wifi.bitrate_mbps)
        except Exception as e:
            # Never let a WiFi-collection hiccup take down the broadcast
            # loop — the cpu/mem/temp metrics are more important and
            # need to keep flowing.
            if self.logger:
                self.logger.debug(f'WiFi telemetry skipped: {e}')

        for channel_id, value in readings.items():
            node.runtime_state.set_channel(HOST_CONTROLLER_PERIPHERAL_ID, channel_id, value)
        node.runtime_state.last_feedback = time.time()

    def _save_node_config(self, node_id: str):
        """Save per-node YAML (peripherals + identity)."""
        node = self.state.adopted_nodes.get(node_id)
        if not node:
            return

        os.makedirs(self.nodes_config_dir, exist_ok=True)
        config = {
            "node_id": node.node_id,
            "display_name": node.display_name,
            "role": node.role,
            "hardware_model": node.hardware_model,
            "mac_address": node.mac_address,
            "chip_family": node.chip_family,
            "board_id": node.board_id,
        }
        if node.peripheral_config:
            config["peripherals"] = node.peripheral_config.to_dict()

        filepath = os.path.join(self.nodes_config_dir, f"{node_id}.yaml")
        try:
            with open(filepath, 'w') as f:
                yaml.dump(config, f, default_flow_style=False, sort_keys=False)
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to save node config: {e}")

    def _load_node_config(self, node_id: str) -> bool:
        """Load per-node YAML — for use when re-adopting an existing node."""
        node = self.state.adopted_nodes.get(node_id)
        if not node:
            return False

        filepath = os.path.join(self.nodes_config_dir, f"{node_id}.yaml")
        if not os.path.exists(filepath):
            return False

        try:
            with open(filepath, 'r') as f:
                config = yaml.safe_load(f)
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to load node config: {e}")
            return False

        if not config:
            return False

        peripherals_data = config.get('peripherals', {})
        if peripherals_data:
            node.peripheral_config = NodePeripheralConfig.from_dict(peripherals_data)
        return True

    def load_all_node_configs(self):
        """Load all adopted nodes from disk on startup."""
        if not os.path.isdir(self.nodes_config_dir):
            return

        for filename in os.listdir(self.nodes_config_dir):
            if not filename.endswith('.yaml'):
                continue

            node_id = filename[:-5]
            filepath = os.path.join(self.nodes_config_dir, f"{node_id}.yaml")

            try:
                with open(filepath, 'r') as f:
                    config = yaml.safe_load(f)
            except Exception as e:
                if self.logger:
                    self.logger.error(f"Failed to load node config {filename}: {e}")
                continue

            if not config:
                continue

            node = NodeInfo(
                node_id=config.get('node_id', node_id),
                display_name=config.get('display_name', ''),
                role=config.get('role', ''),
                hardware_model=config.get('hardware_model', 'Unknown'),
                mac_address=config.get('mac_address', ''),
                chip_family=config.get('chip_family', ''),
                board_id=config.get('board_id', ''),
                online=False,
                last_seen=0,
            )
            self.state.adopted_nodes[node_id] = node

            peripherals_data = config.get('peripherals', {})
            if peripherals_data:
                node.peripheral_config = NodePeripheralConfig.from_dict(peripherals_data)

            if self.logger:
                self.logger.info(f"Loaded adopted node from config: {node_id} ({node.role})")

    # =========================================================================
    # Runtime State Methods
    # =========================================================================

    def get_or_create_runtime_state(self, node_id: str) -> Optional[NodeRuntimeState]:
        """Get or create runtime state for a node.

        Runtime state is populated lazily as firmware reports pin readings;
        we no longer pre-seed entries from a static pin config because
        peripherals own their pins now.
        """
        node = self.state.adopted_nodes.get(node_id)
        if not node:
            return None
        if not node.runtime_state:
            node.runtime_state = NodeRuntimeState(node_id=node_id)
        return node.runtime_state

    def update_pin_desired(self, node_id: str, gpio: int, value: float) -> bool:
        """Set a desired value for a pin.

        Called by the routing engine when a route resolves to a physical
        pin write. Creates the runtime-state entry on demand since we
        don't pre-seed any more.
        """
        runtime_state = self.get_or_create_runtime_state(node_id)
        if not runtime_state:
            return False
        if gpio not in runtime_state.pins:
            runtime_state.pins[gpio] = PinRuntimeState(gpio=gpio, mode='unknown')
        runtime_state.pins[gpio].desired_value = value
        return True

    def update_pin_actual(self, node_id: str,
                          pins_data: List[Dict[str, Any]],
                          channels_data: Optional[List[Dict[str, Any]]] = None) -> bool:
        """
        Update actual pin values from firmware feedback.

        Args:
            node_id: The node ID
            pins_data: legacy GPIO-keyed entries from the firmware
                state JSON (translated to channels via
                `_FIRMWARE_CHANNEL_MAP`).
            channels_data: peripheral-first records from the new
                `channels[]` field (direct `{peripheral_id, channel_id,
                value}` ingest). Optional; firmware drivers that
                haven't migrated yet emit nothing here. See
                docs/PERIPHERAL_FIRST_MIGRATION.md.

        Returns:
            True if update successful
        """
        runtime_state = self.get_or_create_runtime_state(node_id)
        if not runtime_state:
            return False

        channel_updates = runtime_state.update_from_firmware(pins_data)
        if channels_data:
            channel_updates.extend(
                runtime_state.update_channels_from_firmware(channels_data))

        # Hand the resolved channel values to the optional logger.
        # record() short-circuits cheaply when the peripheral isn't
        # in its enabled set, so calling unconditionally is fine.
        plog = self.peripheral_logger
        if plog is not None:
            for peripheral_id, channel_id, value in channel_updates:
                plog.record(node_id, peripheral_id, channel_id, value)

        # Feed the routing graph so sensor readings can drive wiring —
        # a limit switch tripping a pose, a BMS SOC gating a behaviour.
        # set_peripheral_channel_value short-circuits when nothing
        # references the channel or the value hasn't changed, so calling
        # it for every update is cheap. Wrapped because this runs on the
        # ROS callback thread: an exception here would otherwise kill
        # telemetry ingestion for the whole node.
        evaluator = getattr(self, "_routing_evaluator", None)
        if evaluator is not None:
            for peripheral_id, channel_id, value in channel_updates:
                try:
                    evaluator.set_peripheral_channel_value(
                        node_id, peripheral_id, channel_id, value)
                except Exception as e:
                    if self.logger:
                        self.logger.error(
                            f"Routing channel-source update failed for "
                            f"{node_id}/{peripheral_id}/{channel_id}: "
                            f"{type(e).__name__}: {e}", exc_info=True)
        return True

    def record_commanded_channel(self, node_id: str, peripheral_id: str,
                                 channel_id: str, value: float) -> None:
        """Note the value the server last COMMANDED for a channel.

        The State tab's sliders bind to `pin_state/<node_id>`, which is
        built from this runtime state — so a channel with no entry here
        renders a slider with no position, and the operator drags from
        wherever the control happened to be rather than from where the
        hardware actually is.

        Most actuator channels never report back. A Maestro's /state
        carries only `connected`, `error_flags` and `moving`; the 24
        servo channels are write-only on the wire (reading positions
        means polling the Maestro over USB, which is the transfer that
        used to wedge the Teensy). So for those channels the last
        commanded value IS the best available truth, and without it a
        pose can move a servo while its slider sits at zero.

        Firmware readings still win: a real reading for the same channel
        arriving on /state overwrites this via the same set_channel path.
        """
        runtime_state = self.get_or_create_runtime_state(node_id)
        if runtime_state is None:
            return
        try:
            runtime_state.set_channel(peripheral_id, channel_id, float(value))
        except (TypeError, ValueError):
            return

    def get_runtime_state(self, node_id: str) -> Optional[Dict[str, Any]]:
        """Get runtime state for a node as a dictionary."""
        runtime_state = self.get_or_create_runtime_state(node_id)
        if not runtime_state:
            return None

        return runtime_state.to_dict()

    # =========================================================================
    # Firmware Management Methods
    # =========================================================================

    def _get_firmware_dir(self) -> Optional[str]:
        """Locate the RP2040 firmware *source* directory (parent of build/)."""
        # Explicit override (same pattern as SAINT_INSTALL_PREFIX). Needed
        # where the server runs from a colcon *install* tree with no source
        # checkout above it — e.g. the Renode e2e container, where the
        # firmware is bind-mounted at /work/firmware/rp2040 but neither the
        # __file__-relative path below nor the ament-share dirname math
        # resolves to it, so sim-build discovery (and OTA Phase 5) failed.
        env_dir = os.environ.get("SAINT_FIRMWARE_RP2040_DIR")
        if env_dir and os.path.isdir(env_dir):
            return env_dir

        current_dir = os.path.dirname(__file__)
        firmware_dir = os.path.abspath(os.path.join(current_dir, '..', '..', 'firmware', 'rp2040'))
        if os.path.isdir(firmware_dir):
            return firmware_dir

        try:
            from ament_index_python.packages import get_package_share_directory
            package_dir = get_package_share_directory('saint_os')
            base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(package_dir))))
            firmware_dir = os.path.join(base_dir, 'firmware', 'rp2040')
            if os.path.isdir(firmware_dir):
                return firmware_dir
        except Exception:
            pass

        dev_paths = [
            os.path.expanduser('~/Projects/OpenSAINT/SaintOS/source/firmware/rp2040'),
            '/opt/saint-os/firmware/rp2040',
        ]
        for path in dev_paths:
            if os.path.isdir(path):
                return path

        return None

    def _firmware_staging_dir(self, fw_type: str) -> Optional[str]:
        """Resolve the on-disk staging directory for a firmware target.

        Returns the first existing path from the candidate list below, or
        None if none of them exist. Same resolution rules for every
        platform — used by get_firmware_info_for_type and friends so the
        per-platform code paths don't drift apart on path handling. The
        previous design had each platform branch hand-roll its own
        candidate list, which is how the Teensy ended up unable to find
        the staged version.h on the deployed Pi (its candidates missed
        the install-tree symlink that the RP2040 branch happened to hit).

        Candidate order (first match wins):
          1. /opt/saint-os/resources/firmware/<fw_type> via _INSTALL_PREFIX
          2. <here>/../../resources/firmware/<fw_type> — this is the
             install-tree symlink path that the dist creates at
             install/lib/python3.11/site-packages/resources/firmware ->
             /opt/saint-os/firmware
          3. ament_index_python share dir for the saint_os package
          4. Repo-relative dev path for local server runs
        """
        here = os.path.dirname(__file__)
        candidates = [
            str(_INSTALL_PREFIX / 'resources' / 'firmware' / fw_type),
            os.path.abspath(os.path.join(here, '..', '..', 'resources', 'firmware', fw_type)),
        ]
        try:
            from ament_index_python.packages import get_package_share_directory
            package_dir = get_package_share_directory('saint_os')
            candidates.append(os.path.join(package_dir, 'resources', 'firmware', fw_type))
        except Exception:
            pass
        # Repo-relative dev fallback for running the server out of source.
        candidates.append(
            os.path.abspath(os.path.join(here, '..', '..', '..', '..',
                                          'server', 'resources', 'firmware', fw_type))
        )
        for path in candidates:
            if os.path.isdir(path):
                return path
        return None

    def _read_version_h(self, staging_dir: str, result: Dict[str, Any]) -> None:
        """Populate `result` with version fields parsed from
        <staging_dir>/generated/version.h.

        Mutates `result` in place. Silently no-ops if the file is missing
        or the regexes don't match — caller's defaults stay. Fields
        populated when present:
          - version       (from FIRMWARE_VERSION_STRING)
          - version_full  (from FIRMWARE_VERSION_FULL — what the OTA
                           up-to-date check on the node parses for a
                           build timestamp)
          - git_hash      (from FIRMWARE_GIT_HASH)
          - build_date    (from FIRMWARE_BUILD_TIMESTAMP — preferred
                           over file mtime)

        Single implementation shared across get_server_firmware_info,
        get_firmware_build_info, and get_firmware_info_for_type so the
        regex/scope quirks that broke the Teensy's separate copy can't
        recur. Re-import re defensively in case the caller has shadowed
        the module-level binding.
        """
        version_h_path = os.path.join(staging_dir, 'generated', 'version.h')
        if not os.path.isfile(version_h_path):
            return
        try:
            with open(version_h_path, 'r') as f:
                content = f.read()
        except Exception:
            return
        import re as _re  # defensive against caller-side shadowing
        m = _re.search(r'FIRMWARE_VERSION_STRING\s+"([^"]+)"', content)
        if m:
            result["version"] = m.group(1)
        m_full = _re.search(r'FIRMWARE_VERSION_FULL\s+"([^"]+)"', content)
        if m_full:
            result["version_full"] = m_full.group(1)
        m_hash = _re.search(r'FIRMWARE_GIT_HASH\s+"([^"]+)"', content)
        if m_hash:
            result["git_hash"] = m_hash.group(1)
        m_ts = _re.search(r'FIRMWARE_BUILD_TIMESTAMP\s+"([^"]+)"', content)
        if m_ts:
            result["build_date"] = m_ts.group(1)

    def _populate_node_firmware_info(self, result: Dict[str, Any], fw_type: str,
                                      bin_filename: str) -> None:
        """Shared body for node-firmware (rp2040, teensy41) info reads.

        Fills `result` with: available, bin_path/size/crc32, build_date
        (from bin mtime, then overridden by version.h timestamp), plus
        version / version_full / git_hash via _read_version_h. Per-
        platform differences are just `fw_type` (staging dir name) and
        `bin_filename` (saint_node.bin / firmware.hex / saint_node.uf2).
        """
        staging = self._firmware_staging_dir(fw_type)
        if not staging:
            return
        bin_path = os.path.join(staging, bin_filename)
        if os.path.isfile(bin_path):
            result["available"]  = True
            result["bin_path"]   = bin_path
            result["bin_size"]   = os.path.getsize(bin_path)
            result["bin_crc32"]  = self._calculate_file_crc32(bin_path)
            result["build_date"] = time.strftime(
                '%Y-%m-%d %H:%M:%S', time.localtime(os.path.getmtime(bin_path)))
        self._read_version_h(staging, result)

    def _populate_package_firmware_info(self, result: Dict[str, Any], fw_type: str,
                                         file_glob_suffix: str,
                                         filename_contains: Optional[str] = None
                                         ) -> None:
        """Shared body for package-firmware (raspberrypi, controller) info reads.

        These don't ship a flash image — they ship a versioned archive
        (zip / AppImage) plus an info.json manifest. info.json carries the
        canonical version + checksum; the filename-scan fallback only
        runs when the manifest is missing (e.g. an older build).
        """
        staging = self._firmware_staging_dir(fw_type)
        if not staging:
            return
        info_file = os.path.join(staging, 'info.json')
        if os.path.isfile(info_file):
            try:
                with open(info_file, 'r') as f:
                    info = json.load(f)
                result["available"]   = True
                result["version"]     = info.get("latest_version", "0.0.0")
                result["filename"]    = info.get("latest_package")
                result["checksum"]    = info.get("latest_checksum")
                result["build_date"]  = info.get("updated")
                return
            except Exception:
                pass
        # info.json missing or unreadable — scan directory for matching files.
        import re as _re
        for f in os.listdir(staging):
            if not f.endswith(file_glob_suffix):
                continue
            if filename_contains and filename_contains not in f:
                continue
            result["available"] = True
            result["filename"]  = f
            m = _re.search(r'(\d+\.\d+\.\d+)', f)
            if m:
                result["version"] = m.group(1)
            break

    def _firmware_artifact_dirs(self) -> List[Tuple[str, str]]:
        """Candidate directories holding RP2040 .elf/.uf2 artifacts.

        Returns a list of (build_type, dir_path) tuples in priority order:
            - ``server/resources/firmware/rp2040`` — the production install
              location, populated by build-local-dist.sh / the CI tarball
              into ``/opt/saint-os/resources/firmware/rp2040``.
            - ``firmware/rp2040/build`` — what ``./build.sh hw`` writes in
              dev. (``build_hardware`` was an older path; kept for compat
              with anyone still using it.)
            - ``firmware/rp2040/build_sim`` — sim build output.
        """
        candidates: List[Tuple[str, str]] = []

        # Resource artifact paths (production-style and dev-fallback)
        here = os.path.dirname(__file__)
        resource_candidates = [
            str(_INSTALL_PREFIX / 'resources' / 'firmware' / 'rp2040'),
            os.path.abspath(os.path.join(here, '..', '..', 'resources', 'firmware', 'rp2040')),
        ]
        try:
            from ament_index_python.packages import get_package_share_directory
            package_dir = get_package_share_directory('saint_os')
            resource_candidates.append(os.path.join(package_dir, 'resources', 'firmware', 'rp2040'))
        except Exception:
            pass
        for path in resource_candidates:
            if os.path.isdir(path):
                candidates.append(('hardware', path))

        # Build-directory paths (dev)
        fw_src = self._get_firmware_dir()
        if fw_src:
            for sub, btype in [('build', 'hardware'),
                               ('build_hardware', 'hardware'),
                               ('build_sim', 'simulation'),
                               # Canonical sim *install* dir (`make
                               # install_sim`). build_sim is a dev-only
                               # output; install/simulation is what gets
                               # deployed AND what the Renode e2e container
                               # mounts — without it, force_firmware_update
                               # reported "No simulation firmware build
                               # found" in the e2e (Phase 5).
                               (os.path.join('install', 'simulation'), 'simulation')]:
                p = os.path.join(fw_src, sub)
                if os.path.isdir(p):
                    candidates.append((btype, p))
        return candidates

    def _parse_version(self, version_str: str) -> tuple:
        """Parse version string into comparable tuple."""
        if not version_str:
            return (0, 0, 0)
        # Handle versions like "1.1.0" or "1.1.0-abc123"
        match = re.match(r'^(\d+)\.(\d+)\.(\d+)', version_str)
        if match:
            return (int(match.group(1)), int(match.group(2)), int(match.group(3)))
        return (0, 0, 0)

    def _calculate_file_md5(self, file_path: str) -> Optional[str]:
        """Calculate MD5 hash of a file."""
        if not file_path or not os.path.isfile(file_path):
            return None
        try:
            md5_hash = hashlib.md5()
            with open(file_path, 'rb') as f:
                # Read in chunks to handle large files
                for chunk in iter(lambda: f.read(8192), b''):
                    md5_hash.update(chunk)
            return md5_hash.hexdigest()
        except Exception:
            return None

    def _calculate_file_crc32(self, file_path: str) -> Optional[int]:
        """Calculate CRC32 of a file (matches the OTA bootloader's CRC).

        Standard zlib polynomial — same as the firmware-side crc32_update().
        Returned as an unsigned 32-bit int so callers can hex-format it.
        """
        if not file_path or not os.path.isfile(file_path):
            return None
        try:
            import zlib
            crc = 0
            with open(file_path, 'rb') as f:
                for chunk in iter(lambda: f.read(65536), b''):
                    crc = zlib.crc32(chunk, crc)
            return crc & 0xFFFFFFFF
        except Exception:
            return None

    def get_server_firmware_info(self) -> Dict[str, Any]:
        """
        Get the latest server firmware version and build info.

        Returns dict with:
            - version: Version string (e.g., "1.1.0")
            - version_full: Full version with git hash (e.g., "1.1.0-abc123-dirty")
            - git_hash: Git commit hash
            - file_hash: MD5 hash of the firmware binary
            - build_date: Build date if available
            - elf_path: Path to ELF file (for simulation)
            - uf2_path: Path to UF2 file (for hardware)
            - available: Whether firmware files are available
        """
        result = {
            "version": "0.0.0",
            "version_full": None,
            "git_hash": None,
            "file_hash": None,
            "build_date": None,
            "elf_path": None,
            "uf2_path": None,
            "bin_path": None,
            "bin_size": None,
            "bin_crc32": None,
            "available": False,
        }

        candidates = self._firmware_artifact_dirs()
        firmware_dir = self._get_firmware_dir()  # used for CMakeLists fallback
        if not candidates:
            result["error"] = "Firmware artifacts not found"
            return result

        # Walk candidate dirs in priority order; first one with an
        # artifact wins. Production install (resources/firmware/rp2040)
        # ranks above local dev build dirs.
        for build_type, build_dir in candidates:
            elf_path = os.path.join(build_dir, 'saint_node.elf')
            uf2_path = os.path.join(build_dir, 'saint_node.uf2')
            bin_path = os.path.join(build_dir, 'saint_node.bin')

            if os.path.isfile(elf_path):
                result["elf_path"] = elf_path
                result["available"] = True
                result["build_type"] = build_type

                # Get build date from file modification time
                mtime = os.path.getmtime(elf_path)
                result["build_date"] = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(mtime))

                # Calculate MD5 hash of the firmware binary
                result["file_hash"] = self._calculate_file_md5(elf_path)

            if os.path.isfile(uf2_path):
                result["uf2_path"] = uf2_path
                result["available"] = True
                if not result.get("build_type"):
                    result["build_type"] = build_type
                # If we have UF2 but no ELF, hash and date come from UF2
                if not result["file_hash"]:
                    result["file_hash"] = self._calculate_file_md5(uf2_path)
                if not result["build_date"]:
                    mtime = os.path.getmtime(uf2_path)
                    result["build_date"] = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(mtime))

            # Raw .bin — what the OTA bootloader fetches over HTTP. Size +
            # CRC32 are the values the bootloader will verify against, so
            # they go into the firmware_update control message.
            if os.path.isfile(bin_path) and not result.get("bin_path"):
                result["bin_path"]  = bin_path
                result["bin_size"]  = os.path.getsize(bin_path)
                result["bin_crc32"] = self._calculate_file_crc32(bin_path)

            # Version metadata from generated version.h — same helper
            # the Teensy / per-platform info read uses, so a regex fix
            # in one place propagates everywhere.
            self._read_version_h(build_dir, result)

            # Fallback: Try to read version from CMakeLists.txt. Only
            # applies in dev where the firmware source tree is present;
            # in production firmware_dir is None and we already have a
            # version from version.h in the artifact dir.
            if result["version"] == "0.0.0" and firmware_dir:
                cmake_path = os.path.join(firmware_dir, 'CMakeLists.txt')
                if os.path.isfile(cmake_path):
                    try:
                        with open(cmake_path, 'r') as f:
                            content = f.read()
                        major = re.search(r'set\(FIRMWARE_VERSION_MAJOR\s+(\d+)\)', content)
                        minor = re.search(r'set\(FIRMWARE_VERSION_MINOR\s+(\d+)\)', content)
                        patch = re.search(r'set\(FIRMWARE_VERSION_PATCH\s+(\d+)\)', content)
                        if major and minor and patch:
                            result["version"] = f"{major.group(1)}.{minor.group(1)}.{patch.group(1)}"
                    except Exception:
                        pass

            if result["available"]:
                break

        return result

    def get_firmware_build_info(self, build_type: str) -> Dict[str, Any]:
        """
        Get firmware info for a specific build type.

        Args:
            build_type: 'simulation' or 'hardware'

        Returns dict with firmware info or available=False if not found.
        """
        result = {
            "version": "0.0.0",
            "version_full": None,
            "git_hash": None,
            "build_date": None,
            "elf_path": None,
            "uf2_path": None,
            "available": False,
            "build_type": build_type,
        }

        # Walk the same candidate list get_server_firmware_info uses,
        # but filter to the requested build type.
        candidates = [(bt, d) for (bt, d) in self._firmware_artifact_dirs()
                      if bt == build_type]
        if not candidates:
            return result

        for _bt, build_dir in candidates:
            elf_path = os.path.join(build_dir, 'saint_node.elf')
            uf2_path = os.path.join(build_dir, 'saint_node.uf2')

            if os.path.isfile(elf_path):
                result["elf_path"] = elf_path
                result["available"] = True
                mtime = os.path.getmtime(elf_path)
                result["build_date"] = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(mtime))

            if os.path.isfile(uf2_path):
                result["uf2_path"] = uf2_path
                result["available"] = True
                if not result["build_date"]:
                    mtime = os.path.getmtime(uf2_path)
                    result["build_date"] = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(mtime))

            # Version metadata from generated version.h — shared helper.
            self._read_version_h(build_dir, result)

            if result["available"]:
                break

        return result

    def get_all_firmware_builds(self) -> Dict[str, Any]:
        """Get info for all available firmware builds."""
        return {
            "simulation": self.get_firmware_build_info("simulation"),
            "hardware": self.get_firmware_build_info("hardware"),
            "teensy41": self.get_firmware_info_for_type("teensy41"),
            "raspberrypi": self.get_firmware_info_for_type("raspberrypi"),
            "controller": self.get_firmware_info_for_type("controller"),
        }

    def is_firmware_update_available(self, node_id: str) -> Dict[str, Any]:
        """
        Check if a firmware update is available for a node.

        Returns dict with:
            - available: Whether an update is available
            - current_version: Node's current firmware version
            - server_version: Server's firmware version
            - message: Human-readable status message
        """
        node = self.state.adopted_nodes.get(node_id) or self.state.unadopted_nodes.get(node_id)
        if not node:
            return {
                "available": False,
                "current_version": None,
                "server_version": None,
                "message": "Node not found",
            }

        # Suppress update-status computation for offline nodes. The node's
        # firmware_version stays at its last-known value (often the empty
        # "0.0.0" default when the node has never announced its build),
        # which would otherwise compare strictly-less-than the server's
        # real version and falsely flag "update available" — misleading
        # for nodes that are simply powered off. Surface it explicitly
        # so the UI can render the offline state instead of a stale
        # update prompt.
        if not node.online:
            return {
                "available": False,
                "current_version": node.firmware_version or None,
                "server_version": None,
                "message": "Node offline — update status unavailable",
            }

        # Dispatch by the node's chip family so a Teensy compares
        # against the Teensy's staged build (and an RP2040 against the
        # RP2040's). The earlier code always fetched RP2040 info, so
        # the "Update available: 1.2.0..." banner on a Teensy node
        # actually compared its 1.0.0 firmware against the RP2040's
        # 1.2.0 build — a category error that produced misleading UX
        # AND would have OTA'd the wrong .bin if the operator clicked
        # update without force. Fall back to the legacy
        # get_server_firmware_info path for nodes whose chip family
        # isn't reported yet (older firmware) — there the RP2040
        # build is the only thing we can compare against.
        chip = (node.chip_family or '').lower() or None
        if chip and chip in ('rp2040', 'teensy41', 'raspberrypi'):
            # Compare each chip family against ITS OWN staged firmware.
            # Without 'raspberrypi' in this list, a Pi node falls through
            # to get_server_firmware_info() — which walks the RP2040
            # build tree, so the Pi's 1.1.0 gets compared against an
            # RP2040 build's 0.5.0 and the UI shows a bogus "Update
            # available" badge on a Pi that's already current.
            server_fw = self.get_firmware_info_for_type(chip)
        else:
            server_fw = self.get_server_firmware_info()
        node_version = node.firmware_version or "0.0.0"
        server_version = server_fw.get("version", "0.0.0")
        # Use full version (with unix timestamp) for display if available
        server_version_display = server_fw.get("version_full") or server_version
        server_file_hash = server_fw.get("file_hash")

        node_tuple = self._parse_version(node_version)
        server_tuple = self._parse_version(server_version)

        update_available = False
        message = ""

        # Extract unix timestamp from version strings (format: "1.2.0-1738505432")
        def extract_build_timestamp(version_str: str) -> int:
            if not version_str or "-" not in version_str:
                return 0
            try:
                suffix = version_str.split("-", 1)[1]
                # Unix timestamp is all digits
                if suffix.isdigit():
                    return int(suffix)
            except (IndexError, ValueError):
                pass
            return 0

        node_build_ts = extract_build_timestamp(node_version)
        server_build_ts = extract_build_timestamp(server_version_display)

        if not server_fw["available"]:
            message = "No firmware build available on server"
        elif server_tuple > node_tuple:
            # Server has newer semantic version number
            update_available = True
            message = f"Update available: {node_version} → {server_version_display}"
        elif server_tuple == node_tuple:
            # Same semantic version - compare unix timestamps
            if server_build_ts > 0 and node_build_ts > 0:
                if server_build_ts > node_build_ts:
                    update_available = True
                    # Show file hash (truncated) if available for identification
                    hash_info = f" [md5:{server_file_hash[:8]}]" if server_file_hash else ""
                    message = f"Rebuild available: {node_version} → {server_version_display}{hash_info}"
                elif server_build_ts == node_build_ts:
                    message = "Firmware is up to date"
                else:
                    message = "Node firmware is newer than server"
            else:
                # Fallback to build date string comparison
                server_build = server_fw.get("build_date")
                node_build = node.firmware_build
                if server_build and node_build and server_build > node_build:
                    update_available = True
                    message = f"Rebuild available: {node_build} → {server_build}"
                else:
                    message = "Firmware is up to date"
        else:
            message = "Node firmware is newer than server"

        return {
            "available": update_available,
            "current_version": node_version,
            "server_version": server_version_display,
            "server_file_hash": server_file_hash,
            "server_build_date": server_fw.get("build_date"),
            "node_build_date": node.firmware_build if node else None,
            "message": message,
        }

    def get_firmware_info_for_type(self, fw_type: str) -> Dict[str, Any]:
        """
        Get firmware info for a specific platform type.

        Args:
            fw_type: 'rp2040', 'teensy41', 'raspberrypi', or 'controller'

        Returns dict with:
            - available: Whether firmware is available
            - version: Version string
            - filename: Package filename
            - checksum: SHA256 checksum
            - build_date: Build timestamp
        """
        result = {
            "available": False,
            "version": "0.0.0",
            "filename": None,
            "checksum": None,
            "build_date": None,
            "type": fw_type,
        }

        if fw_type == 'rp2040':
            # RP2040 has historically gone through get_server_firmware_info
            # which walks both the install staging dir AND dev-build
            # locations (firmware/rp2040/build/). Keep that path for ELF /
            # UF2 / dev fallbacks, but propagate version_full too — it
            # was previously missing here, which broke the up-to-date
            # check on the RP2040 branch of is_firmware_update_available
            # the same way it broke the Teensy.
            rp2040_info = self.get_server_firmware_info()
            result["available"]    = rp2040_info.get("available", False)
            result["version"]      = rp2040_info.get("version", "0.0.0")
            result["version_full"] = rp2040_info.get("version_full")
            result["build_date"]   = rp2040_info.get("build_date")
            result["elf_path"]     = rp2040_info.get("elf_path")
            result["uf2_path"]     = rp2040_info.get("uf2_path")
            result["bin_path"]     = rp2040_info.get("bin_path")
            result["bin_size"]     = rp2040_info.get("bin_size")
            result["bin_crc32"]    = rp2040_info.get("bin_crc32")
            return result

        elif fw_type == 'teensy41':
            # Teensy artifact = raw saint_node.bin streamed by the in-app
            # OTA. Path resolution + version.h read both come from the
            # shared helpers, so the regex bug that used to silently
            # produce version="0.0.0" can't recur from a copied-and-
            # drifted code path.
            self._populate_node_firmware_info(result, 'teensy41', 'saint_node.bin')
            return result

        elif fw_type == 'raspberrypi':
            # Pi 5 firmware is a zip package, not a flash image. info.json
            # is produced by the Pi 5 build script and carries the version
            # / checksum / package name. Fallback parses the filename if
            # info.json is missing.
            self._populate_package_firmware_info(
                result, 'raspberrypi', file_glob_suffix='.zip', filename_contains='raspberrypi')
            return result

        elif fw_type == 'controller':
            # Steam Deck controller .AppImage. Same info.json shape as
            # raspberrypi (controller/appimage/build-bundle.sh emits it).
            self._populate_package_firmware_info(
                result, 'controller', file_glob_suffix='.AppImage')

        return result
