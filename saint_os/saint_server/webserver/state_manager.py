"""
SAINT.OS State Manager

Manages system state and provides data for WebSocket clients.
"""

import hashlib
import json
import os
import re
import subprocess
import time
import yaml
import psutil
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
    SystemRouting,
    WidgetInstance,
    WidgetType,
    Wire,
    detect_pin_conflicts,
)
from saint_server.board_config import BoardConfigManager, derive_capabilities

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
        """Update actual values from firmware feedback.

        The firmware still publishes its peripheral channels on
        ``virtual GPIO`` slots (a transitional artifact of the
        peripheral-first refactor). We translate them here into the
        peripheral-first ``channels`` view that the routing graph
        and the UI consume. Once the firmware grows a channel-
        addressed publisher, this translation goes away.

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


class StateManager:
    """Manages system state and provides data for clients."""

    MAX_LOG_ENTRIES = 500          # Global activity-log ring buffer cap
    MAX_NODE_LOG_ENTRIES = 200     # Per-node log ring buffer cap

    def __init__(self, server_name: str = "SAINT-01", logger=None, config_dir: Optional[str] = None):
        self.state = SystemState(server_name=server_name)
        self.logger = logger
        self._activity_callback: Optional[Callable[[str, str], None]] = None
        # Optional callback fired when a node-scoped log entry is recorded.
        # Wired by server_node.py to broadcast on node_log/<id>.
        self._node_log_callback: Optional[Callable[[str, Dict[str, Any]], None]] = None
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
            # Dev fallback — repo's saint_os/config/ tree.
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

        # Optional routing graph evaluator. Set by server_node once the
        # ROS bridge is up. Whenever sheets change we call .reconcile()
        # on it so it can refresh its subscriptions.
        self._routing_evaluator = None

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

    def log_node_event(self, node_id: str, message: str, level: str = "info") -> None:
        """Public entry point for callers outside StateManager (e.g.
        server_node.py) that want to record a node-scoped event."""
        self._log_activity(message, level, node_id=node_id)

    def _log_activity(self, message: str, level: str = "info",
                      node_id: Optional[str] = None):
        """Log an activity event.

        If ``node_id`` is provided, the entry is also appended to that
        node's per-node ring buffer and broadcast on ``node_log/<id>``
        so the node-detail Logs tab sees it.
        """
        # Store in log buffer
        entry = {
            "time": time.time(),
            "text": message,
            "level": level,
        }
        self._log_entries.append(entry)

        # Trim to max size
        if len(self._log_entries) > self.MAX_LOG_ENTRIES:
            self._log_entries = self._log_entries[-self.MAX_LOG_ENTRIES:]

        # Broadcast to clients
        if self._activity_callback:
            self._activity_callback(message, level)

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
                node.peripheral_connected[pid] = connected

        # Clear any pending offline state and mark online
        node.online = True
        node.going_offline_at = None

        # Log reconnection
        if not prev_online:
            name = node.display_name or node.node_id
            self._log_activity(f"Node reconnected: {name}", "info", node_id=node_id)

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
        server_fw = self.get_server_firmware_info()

        for node in self.state.adopted_nodes.values():
            # Host controller isn't an RP2040/Teensy firmware target —
            # it has no .uf2/.hex update flow, so skip the comparison.
            if node.node_id == HOST_CONTROLLER_NODE_ID:
                fw_update_info = {
                    "available": False,
                    "message": "Host controller — runs from the server install",
                }
            else:
                fw_update_info = self.is_firmware_update_available(node.node_id)

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

    def adopt_node(self, node_id: str, role: str,
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
        node.role = role
        node.display_name = display_name or f"{role.title()} Node"
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

    def reset_node(self, node_id: str, factory_reset: bool = False) -> Dict[str, Any]:
        """Reset an adopted node."""
        if node_id not in self.state.adopted_nodes:
            return {"success": False, "message": f"Node {node_id} not found"}

        if factory_reset:
            node = self.state.adopted_nodes.pop(node_id)
            node.role = ""
            node.display_name = ""
            self.state.unadopted_nodes[node_id] = node
            return {"success": True, "message": "Node factory reset and unadopted"}
        else:
            return {"success": True, "message": "Node reset (soft reset)"}

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
        payload = {
            "action": "configure",
            "version": node.peripheral_config.version,
            "peripherals": [
                {
                    "id": p.id,
                    "type": p.type,
                    "pins": p.pins,
                    "params": p.params,
                }
                for p in node.peripheral_config.peripherals
                if not p.builtin
            ],
        }
        return json.dumps(payload)

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

    def add_routing_input(self, node_id: str, topic: str, field: str,
                          label: str = "",
                          position: Optional[List[int]] = None) -> Dict[str, Any]:
        if not topic:
            return {"success": False, "message": "Missing topic"}
        err = self._validate_sheet_owner(node_id)
        if err:
            return {"success": False, "message": err}
        pos = self._coerce_position(position)
        sheet = self.state.system_routing.get_sheet(node_id)
        node = sheet.add_input(topic=topic, field=field or "", label=label, position=pos)
        self.state.system_routing.bump_version()
        self._save_system_routing()
        return {"success": True, "input": node.to_dict()}

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
        """Mutate one sheet node in place (Input / Output / Operator / Widget).

        Supported keys depend on the node kind:
          - any node:        position, label
          - OperatorNode:    params, defaults
          - InputNode/OutputNode: topic, field
          - WidgetInstance:  params
        """
        err = self._validate_sheet_owner(node_id)
        if err:
            return {"success": False, "message": err}
        sheet = self.state.system_routing.get_sheet(node_id)
        target = (sheet.find_input(sheet_node_id)
                  or sheet.find_output(sheet_node_id)
                  or sheet.find_operator(sheet_node_id)
                  or sheet.find_widget(sheet_node_id))
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
        self.state.system_routing.bump_version()
        self._save_system_routing()
        return {"success": True}

    def remove_sheet_node(self, node_id: str, sheet_node_id: str) -> Dict[str, Any]:
        sheet = self.state.system_routing.sheets.get(node_id)
        if sheet is None:
            return {"success": False, "message": f"Sheet '{node_id}' not found"}
        removed = (sheet.remove_input(sheet_node_id)
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
                # Outputs are sinks only — a wire's source can never be an
                # output node (outputs don't produce values).
                if role == "source":
                    return "source: output nodes cannot be wire sources"
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
        widget = sheet.add_widget(
            type_id=type_id,
            label=label or self.widget_catalog[type_id].label,
            position=pos,
            params=params,
        )
        self.state.system_routing.bump_version()
        self._save_system_routing()
        self._log_activity(f"Added widget {widget.id} ({widget.type}) to {node_id}", "info")
        return {"success": True, "widget": widget.to_dict()}

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
            if self.logger:
                routing = self.state.system_routing
                wire_count = sum(len(s.wires) for s in routing.sheets.values())
                self.logger.info(
                    f"Loaded system routing: {len(routing.sheets)} sheets, "
                    f"{wire_count} wires, {len(routing.widgets)} widgets"
                )
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to load system routing: {e}")

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

    def update_host_controller_runtime(self) -> None:
        """Push current system metrics into the host node's runtime_state.

        Called from the broadcast loop. Values land on channel keys
        ``(system_monitor, cpu_usage)`` etc. so the dashboard's routing
        engine resolves them by the same peripheral/channel identifiers
        the routing graph uses — no virtual GPIO indirection.
        """
        node = self.state.adopted_nodes.get(HOST_CONTROLLER_NODE_ID)
        if not node:
            return
        if not node.runtime_state:
            node.runtime_state = NodeRuntimeState(node_id=HOST_CONTROLLER_NODE_ID)

        node.last_seen = time.time()

        try:
            cpu_usage = psutil.cpu_percent(interval=None)
            mem_usage = psutil.virtual_memory().percent
        except Exception:
            cpu_usage = 0.0
            mem_usage = 0.0
        cpu_temp = _read_cpu_temp()
        throttle = _read_throttle_status()
        uptime_s = float(int(time.time() - self.state.start_time))

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

    def update_pin_actual(self, node_id: str, pins_data: List[Dict[str, Any]]) -> bool:
        """
        Update actual pin values from firmware feedback.

        Args:
            node_id: The node ID
            pins_data: List of pin data from firmware

        Returns:
            True if update successful
        """
        runtime_state = self.get_or_create_runtime_state(node_id)
        if not runtime_state:
            return False

        channel_updates = runtime_state.update_from_firmware(pins_data)

        # Hand the resolved channel values to the optional logger.
        # record() short-circuits cheaply when the peripheral isn't
        # in its enabled set, so calling unconditionally is fine.
        plog = self.peripheral_logger
        if plog is not None:
            for peripheral_id, channel_id, value in channel_updates:
                plog.record(node_id, peripheral_id, channel_id, value)
        return True

    def get_runtime_state(self, node_id: str) -> Optional[Dict[str, Any]]:
        """Get runtime state for a node as a dictionary."""
        runtime_state = self.get_or_create_runtime_state(node_id)
        if not runtime_state:
            return None

        return runtime_state.to_dict()

    def get_node_by_role(self, role: str) -> Optional[NodeInfo]:
        """Look up an adopted node by role tag (still used for adoption flow)."""
        for node in self.state.adopted_nodes.values():
            if node.role == role:
                return node
        return None

    def get_nodes_by_role(self, role: str) -> List[NodeInfo]:
        return [node for node in self.state.adopted_nodes.values() if node.role == role]

    # =========================================================================
    # Firmware Management Methods
    # =========================================================================

    def _get_firmware_dir(self) -> Optional[str]:
        """Locate the RP2040 firmware *source* directory (parent of build/)."""
        current_dir = os.path.dirname(__file__)
        firmware_dir = os.path.abspath(os.path.join(current_dir, '..', '..', 'firmware', 'rp2040'))
        if os.path.isdir(firmware_dir):
            return firmware_dir

        try:
            from ament_index_python.packages import get_package_share_directory
            package_dir = get_package_share_directory('saint_os')
            base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(package_dir))))
            firmware_dir = os.path.join(base_dir, 'saint_os', 'firmware', 'rp2040')
            if os.path.isdir(firmware_dir):
                return firmware_dir
        except Exception:
            pass

        dev_paths = [
            os.path.expanduser('~/Projects/OpenSAINT/SaintOS/source/saint_os/firmware/rp2040'),
            '/opt/saint_os/firmware/rp2040',
        ]
        for path in dev_paths:
            if os.path.isdir(path):
                return path

        return None

    def _firmware_artifact_dirs(self) -> List[Tuple[str, str]]:
        """Candidate directories holding RP2040 .elf/.uf2 artifacts.

        Returns a list of (build_type, dir_path) tuples in priority order:
            - ``saint_os/resources/firmware/rp2040`` — the production install
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
                               ('build_sim', 'simulation')]:
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

            # Try to read version info from generated version.h
            version_h_path = os.path.join(build_dir, 'generated', 'version.h')
            if os.path.isfile(version_h_path):
                try:
                    with open(version_h_path, 'r') as f:
                        content = f.read()
                    # Extract version string
                    version_match = re.search(r'FIRMWARE_VERSION_STRING\s+"([^"]+)"', content)
                    if version_match:
                        result["version"] = version_match.group(1)
                    # Extract full version with git hash
                    full_match = re.search(r'FIRMWARE_VERSION_FULL\s+"([^"]+)"', content)
                    if full_match:
                        result["version_full"] = full_match.group(1)
                    # Extract git hash
                    hash_match = re.search(r'FIRMWARE_GIT_HASH\s+"([^"]+)"', content)
                    if hash_match:
                        result["git_hash"] = hash_match.group(1)
                    # Extract build timestamp from version.h (more accurate than file mtime)
                    build_match = re.search(r'FIRMWARE_BUILD_TIMESTAMP\s+"([^"]+)"', content)
                    if build_match:
                        result["build_date"] = build_match.group(1)
                except Exception:
                    pass

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

            # Read version from generated version.h (only present in build dirs)
            version_h_path = os.path.join(build_dir, 'generated', 'version.h')
            if os.path.isfile(version_h_path):
                try:
                    with open(version_h_path, 'r') as f:
                        content = f.read()
                    version_match = re.search(r'FIRMWARE_VERSION_STRING\s+"([^"]+)"', content)
                    if version_match:
                        result["version"] = version_match.group(1)
                    full_match = re.search(r'FIRMWARE_VERSION_FULL\s+"([^"]+)"', content)
                    if full_match:
                        result["version_full"] = full_match.group(1)
                    hash_match = re.search(r'FIRMWARE_GIT_HASH\s+"([^"]+)"', content)
                    if hash_match:
                        result["git_hash"] = hash_match.group(1)
                    build_match = re.search(r'FIRMWARE_BUILD_TIMESTAMP\s+"([^"]+)"', content)
                    if build_match:
                        result["build_date"] = build_match.group(1)
                except Exception:
                    pass

            if result["available"]:
                break

        return result

    def get_all_firmware_builds(self) -> Dict[str, Any]:
        """Get info for all available firmware builds."""
        return {
            "simulation": self.get_firmware_build_info("simulation"),
            "hardware": self.get_firmware_build_info("hardware"),
            "rpi5": self.get_firmware_info_for_type("rpi5"),
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
            fw_type: 'rp2040' or 'rpi5'

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
            # Use existing method for RP2040 — includes bin size + crc32
            # so the OTA update message has what it needs.
            rp2040_info = self.get_server_firmware_info()
            result["available"]   = rp2040_info.get("available", False)
            result["version"]     = rp2040_info.get("version", "0.0.0")
            result["build_date"]  = rp2040_info.get("build_date")
            result["elf_path"]    = rp2040_info.get("elf_path")
            result["uf2_path"]    = rp2040_info.get("uf2_path")
            result["bin_path"]    = rp2040_info.get("bin_path")
            result["bin_size"]    = rp2040_info.get("bin_size")
            result["bin_crc32"]   = rp2040_info.get("bin_crc32")
            return result

        elif fw_type == 'teensy41':
            # Teensy artifacts staged into resources/firmware/teensy41/.
            # We look for saint_node.bin (raw flash image — what the
            # in-app OTA downloads) and a version.h sidecar.
            here = os.path.dirname(__file__)
            candidates = [
                str(_INSTALL_PREFIX / 'resources' / 'firmware' / 'teensy41'),
                os.path.abspath(os.path.join(here, '..', '..', 'resources', 'firmware', 'teensy41')),
            ]
            fw_dir = None
            for path in candidates:
                if os.path.isdir(path):
                    fw_dir = path
                    break
            if not fw_dir:
                return result

            bin_path = os.path.join(fw_dir, 'saint_node.bin')
            if os.path.isfile(bin_path):
                result["available"]   = True
                result["bin_path"]    = bin_path
                result["bin_size"]    = os.path.getsize(bin_path)
                result["bin_crc32"]   = self._calculate_file_crc32(bin_path)
                mtime = os.path.getmtime(bin_path)
                result["build_date"]  = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(mtime))

            # Try to read version from generated version.h if staged.
            version_h_path = os.path.join(fw_dir, 'generated', 'version.h')
            if os.path.isfile(version_h_path):
                try:
                    with open(version_h_path, 'r') as f:
                        content = f.read()
                    m = re.search(r'FIRMWARE_VERSION_STRING\s+"([^"]+)"', content)
                    if m:
                        result["version"] = m.group(1)
                except Exception:
                    pass
            return result

        elif fw_type == 'rpi5':
            # Look for Pi 5 firmware in resources/firmware/rpi5
            current_dir = os.path.dirname(__file__)
            firmware_dir = os.path.abspath(
                os.path.join(current_dir, '..', '..', 'resources', 'firmware', 'rpi5')
            )

            if not os.path.isdir(firmware_dir):
                return result

            # Check for info.json
            info_file = os.path.join(firmware_dir, 'info.json')
            if os.path.isfile(info_file):
                try:
                    with open(info_file, 'r') as f:
                        info = json.load(f)
                        result["available"] = True
                        result["version"] = info.get("latest_version", "0.0.0")
                        result["filename"] = info.get("latest_package")
                        result["checksum"] = info.get("latest_checksum")
                        result["build_date"] = info.get("updated")
                        return result
                except Exception:
                    pass

            # Fallback: scan for zip files
            for f in os.listdir(firmware_dir):
                if f.endswith('.zip') and 'rpi5' in f:
                    result["available"] = True
                    result["filename"] = f
                    # Try to extract version from filename
                    import re
                    version_match = re.search(r'(\d+\.\d+\.\d+)', f)
                    if version_match:
                        result["version"] = version_match.group(1)
                    break

        return result
