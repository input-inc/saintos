"""
SAINT.OS State Manager

Manages system state and provides data for WebSocket clients.
"""

import json
import os
import re
import time
import yaml
import psutil
from dataclasses import dataclass, field
from typing import Dict, List, Any, Optional, Callable, Tuple

# Timeout for considering a node offline (no announcements received)
# Increased from 5.0 to 10.0 to handle network jitter and temporary disconnects
# Nodes announce at 1Hz, so 10s allows for multiple missed announcements
NODE_TIMEOUT_SECONDS = 10.0

# Grace period before marking a node as truly offline (prevents flapping)
NODE_OFFLINE_GRACE_SECONDS = 3.0


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
        }
        required_cap = mode_map.get(mode, mode)
        return required_cap in self.capabilities


@dataclass
class PinConfiguration:
    """Configuration for a single pin mapping."""
    gpio: int
    mode: str
    logical_name: str
    params: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        result = {
            "mode": self.mode,
            "logical_name": self.logical_name,
        }
        result.update(self.params)
        return result


@dataclass
class NodeCapabilities:
    """Complete pin capabilities for a node."""
    node_id: str
    pins: List[PinCapability] = field(default_factory=list)
    reserved_pins: List[int] = field(default_factory=list)
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
        }


@dataclass
class NodePinConfig:
    """Pin configuration state for a node."""
    version: int = 0
    pins: Dict[int, PinConfiguration] = field(default_factory=dict)  # gpio -> config
    sync_status: str = "unknown"  # unknown, pending, synced, error
    last_synced: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            "version": self.version,
            "pins": {str(gpio): cfg.to_dict() for gpio, cfg in self.pins.items()},
            "sync_status": self.sync_status,
            "last_synced": self.last_synced,
        }

    def to_firmware_json(self) -> Dict[str, Any]:
        """Convert to JSON format expected by firmware."""
        return {
            "action": "configure",
            "version": self.version,
            "pins": {str(gpio): cfg.to_dict() for gpio, cfg in self.pins.items()},
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
class NodeRuntimeState:
    """Runtime state for all pins on a node."""
    node_id: str
    pins: Dict[int, PinRuntimeState] = field(default_factory=dict)  # gpio -> state
    last_feedback: float = 0.0  # Timestamp of last state feedback from firmware

    def get_pin(self, gpio: int) -> Optional[PinRuntimeState]:
        """Get runtime state for a pin."""
        return self.pins.get(gpio)

    def set_pin_desired(self, gpio: int, value: float):
        """Set desired value for a pin."""
        if gpio in self.pins:
            self.pins[gpio].desired_value = value

    def update_from_firmware(self, pins_data: List[Dict[str, Any]]):
        """Update actual values from firmware feedback."""
        now = time.time()
        self.last_feedback = now

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

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            "node_id": self.node_id,
            "pins": [pin.to_dict() for pin in self.pins.values()],
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
    firmware_build: str = ""  # Build timestamp
    online: bool = True
    cpu_temp: float = 0.0
    cpu_usage: float = 0.0
    memory_usage: float = 0.0
    uptime_seconds: int = 0
    # Adopted node specific
    display_name: str = ""
    role: str = ""
    # Pin configuration
    capabilities: Optional[NodeCapabilities] = None
    pin_config: Optional[NodePinConfig] = None
    # Runtime state
    runtime_state: Optional[NodeRuntimeState] = None
    # Tracking
    last_seen: float = field(default_factory=time.time)
    # Grace period tracking - when we first detected potential disconnect
    going_offline_at: Optional[float] = None


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


class StateManager:
    """Manages system state and provides data for clients."""

    MAX_LOG_ENTRIES = 500  # Keep last 500 log entries

    def __init__(self, server_name: str = "SAINT-01", logger=None, config_dir: Optional[str] = None):
        self.state = SystemState(server_name=server_name)
        self.logger = logger
        self._activity_callback: Optional[Callable[[str, str], None]] = None
        self._log_entries: List[Dict[str, Any]] = []  # Circular buffer for logs

        # Config directory for node YAML persistence
        if config_dir is None:
            try:
                from ament_index_python.packages import get_package_share_directory
                config_dir = os.path.join(
                    get_package_share_directory('saint_os'), 'config'
                )
            except Exception:
                # Fallback for development
                config_dir = os.path.join(
                    os.path.dirname(__file__), '..', '..', 'config'
                )
        self.config_dir = os.path.abspath(config_dir)
        self.nodes_config_dir = os.path.join(self.config_dir, 'nodes')

        # Load adopted nodes from disk on startup
        self.load_all_node_configs()

    def set_activity_callback(self, callback: Callable[[str, str], None]):
        """Set callback for activity logging. Callback takes (message, level)."""
        self._activity_callback = callback

    def _log_activity(self, message: str, level: str = "info"):
        """Log an activity event."""
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
            self._log_activity(f"New node discovered: {node_id}", "info")
            is_new = True

        # Update node info from announcement (for both adopted and unadopted)
        node.mac_address = data.get("mac", node.mac_address)
        node.ip_address = data.get("ip", node.ip_address)
        node.hardware_model = data.get("hw", node.hardware_model)
        node.firmware_version = data.get("fw", node.firmware_version)
        node.firmware_build = data.get("fw_build", node.firmware_build)
        node.uptime_seconds = data.get("uptime", node.uptime_seconds)
        node.last_seen = time.time()

        # Clear any pending offline state and mark online
        was_offline = not node.online
        node.online = True
        node.going_offline_at = None

        # Log reconnection
        if was_offline:
            name = node.display_name or node.node_id
            self._log_activity(f"Node reconnected: {name}", "info")

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
                self._log_activity(f"Node offline: {node_id}", "warn")

        # Check adopted nodes
        for node_id, node in self.state.adopted_nodes.items():
            if check_node(node, is_adopted=True):
                timed_out.append(node_id)
                self._log_activity(
                    f"Adopted node offline: {node.display_name or node_id}", "warn"
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

        # Get server firmware info
        fw_info = self.get_server_firmware_info()

        return {
            "server_online": self.state.server_online,
            "server_name": self.state.server_name,
            "server_version": self.state.server_version,
            "uptime_seconds": int(time.time() - self.state.start_time),
            "cpu_usage": cpu_usage,
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
            # Use the proper firmware update check (includes build timestamp comparison)
            fw_update_info = self.is_firmware_update_available(node.node_id)

            node_data = {
                "node_id": node.node_id,
                "display_name": node.display_name,
                "role": node.role,
                "hardware_model": node.hardware_model,
                "ip_address": node.ip_address,
                "mac_address": node.mac_address,
                "firmware_version": node.firmware_version,
                "firmware_build": node.firmware_build,
                "online": node.online,
                "cpu_usage": node.cpu_usage,
                "memory_usage": node.memory_usage,
                "uptime_seconds": node.uptime_seconds,
                "has_capabilities": node.capabilities is not None,
                "pin_config_status": node.pin_config.sync_status if node.pin_config else "unconfigured",
                "firmware_update_available": fw_update_info["available"],
                "firmware_update_message": fw_update_info["message"],
                "server_firmware_version": server_fw.get("version"),
                "server_firmware_build": server_fw.get("build_date"),
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
                "firmware_build": node.firmware_build,
                "cpu_temp": node.cpu_temp,
                "cpu_usage": node.cpu_usage,
                "memory_usage": node.memory_usage,
                "uptime_seconds": node.uptime_seconds,
                "online": node.online,
            }
            for node in self.state.unadopted_nodes.values()
        ]

    def adopt_node(self, node_id: str, role: str, display_name: Optional[str] = None) -> Dict[str, Any]:
        """Adopt an unadopted node."""
        if node_id not in self.state.unadopted_nodes:
            return {"success": False, "message": f"Node {node_id} not found"}

        node = self.state.unadopted_nodes.pop(node_id)
        node.role = role
        node.display_name = display_name or f"{role.title()} Node"
        node.online = True

        self.state.adopted_nodes[node_id] = node

        # Load any existing configuration from disk
        self._load_node_config(node_id)

        # Save initial config
        self._save_node_config(node_id)

        topic_prefix = f"/saint/{role}"
        return {
            "success": True,
            "message": "Node adopted successfully",
            "assigned_topic_prefix": topic_prefix,
        }

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
            last_updated=time.time(),
        )

        if self.logger:
            self.logger.info(f"Stored {len(pins)} pin capabilities for node {node_id}")
        self._log_activity(f"Updated capabilities for node {node_id} ({len(pins)} pins)", "info")
        return True

    def get_node_capabilities(self, node_id: str) -> Optional[Dict[str, Any]]:
        """Get capabilities for a node."""
        node = self.state.adopted_nodes.get(node_id) or self.state.unadopted_nodes.get(node_id)
        if not node or not node.capabilities:
            return None
        return node.capabilities.to_dict()

    def get_node_pin_config(self, node_id: str) -> Optional[Dict[str, Any]]:
        """Get current pin configuration for a node."""
        node = self.state.adopted_nodes.get(node_id)
        if not node:
            return None

        # Load from file if not in memory
        if not node.pin_config:
            self._load_node_config(node_id)

        if node.pin_config:
            return node.pin_config.to_dict()
        return {"version": 0, "pins": {}, "sync_status": "unknown", "last_synced": None}

    def save_node_pin_config(
        self,
        node_id: str,
        pin_configs: Dict[str, Dict[str, Any]]
    ) -> Dict[str, Any]:
        """
        Save pin configuration for a node.

        Args:
            node_id: The node ID
            pin_configs: Dict mapping GPIO (as string) to config dict
                         e.g. {"5": {"mode": "pwm", "logical_name": "left_velocity"}}

        Returns:
            Result dict with success status and message
        """
        node = self.state.adopted_nodes.get(node_id)
        if not node:
            return {"success": False, "message": f"Node {node_id} not found or not adopted"}

        # Create or update pin config
        if not node.pin_config:
            node.pin_config = NodePinConfig()

        # Update version
        node.pin_config.version += 1

        # Parse configurations
        node.pin_config.pins.clear()
        for gpio_str, config in pin_configs.items():
            try:
                gpio = int(gpio_str)
            except ValueError:
                continue

            node.pin_config.pins[gpio] = PinConfiguration(
                gpio=gpio,
                mode=config.get('mode', 'unconfigured'),
                logical_name=config.get('logical_name', ''),
                params={k: v for k, v in config.items() if k not in ('mode', 'logical_name')},
            )

        node.pin_config.sync_status = "pending"

        # Save to YAML file
        self._save_node_config(node_id)

        self._log_activity(f"Saved pin configuration for {node.display_name or node_id}", "info")
        return {
            "success": True,
            "message": "Configuration saved",
            "version": node.pin_config.version,
        }

    def mark_node_synced(self, node_id: str, success: bool = True):
        """Mark node pin config as synced (or error)."""
        node = self.state.adopted_nodes.get(node_id)
        if node and node.pin_config:
            node.pin_config.sync_status = "synced" if success else "error"
            node.pin_config.last_synced = time.time() if success else None
            self._save_node_config(node_id)

    def get_firmware_config_json(self, node_id: str) -> Optional[str]:
        """Get pin configuration in firmware JSON format."""
        node = self.state.adopted_nodes.get(node_id)
        if not node or not node.pin_config:
            return None
        return json.dumps(node.pin_config.to_firmware_json())

    def validate_pin_config(
        self,
        node_id: str,
        pin_configs: Dict[str, Dict[str, Any]],
        role: str
    ) -> Dict[str, Any]:
        """
        Validate pin configuration against role requirements and node capabilities.

        Returns:
            Dict with 'valid' bool and 'errors' list
        """
        errors = []
        warnings = []

        node = self.state.adopted_nodes.get(node_id)
        if not node:
            return {"valid": False, "errors": ["Node not found"], "warnings": []}

        capabilities = node.capabilities
        if not capabilities:
            warnings.append("Node capabilities not available - cannot validate pin compatibility")

        # Load role definition
        try:
            from saint_server.roles import RoleManager
            role_mgr = RoleManager(logger=self.logger)
            role_def = role_mgr.get_role(role)
        except Exception as e:
            return {"valid": False, "errors": [f"Failed to load role: {e}"], "warnings": []}

        if not role_def:
            return {"valid": False, "errors": [f"Unknown role: {role}"], "warnings": []}

        # Check required functions are mapped
        assigned_functions = {
            cfg.get('logical_name')
            for cfg in pin_configs.values()
            if cfg.get('logical_name')
        }

        for func in role_def.get_required_functions():
            if func.name not in assigned_functions:
                errors.append(f"Required function '{func.display_name}' is not assigned to any pin")

        # Validate pin modes against capabilities
        if capabilities:
            for gpio_str, config in pin_configs.items():
                try:
                    gpio = int(gpio_str)
                except ValueError:
                    continue

                pin_cap = capabilities.get_pin(gpio)
                if not pin_cap:
                    errors.append(f"GPIO {gpio} is not available on this node")
                    continue

                mode = config.get('mode', '')
                if mode and not pin_cap.supports_mode(mode):
                    errors.append(
                        f"GPIO {gpio} ({pin_cap.name}) does not support mode '{mode}'"
                    )

        return {
            "valid": len(errors) == 0,
            "errors": errors,
            "warnings": warnings,
        }

    def _save_node_config(self, node_id: str):
        """Save node configuration to YAML file."""
        node = self.state.adopted_nodes.get(node_id)
        if not node:
            return

        # Ensure directory exists
        os.makedirs(self.nodes_config_dir, exist_ok=True)

        # Build config dict
        config = {
            "node_id": node.node_id,
            "display_name": node.display_name,
            "role": node.role,
            "hardware_model": node.hardware_model,
            "mac_address": node.mac_address,
        }

        if node.pin_config:
            config["pin_config"] = {
                "version": node.pin_config.version,
                "sync_status": node.pin_config.sync_status,
                "last_synced": node.pin_config.last_synced,
                "pins": {
                    str(gpio): {
                        "mode": cfg.mode,
                        "logical_name": cfg.logical_name,
                        **cfg.params,
                    }
                    for gpio, cfg in node.pin_config.pins.items()
                },
            }

        # Write YAML file
        filepath = os.path.join(self.nodes_config_dir, f"{node_id}.yaml")
        try:
            with open(filepath, 'w') as f:
                yaml.dump(config, f, default_flow_style=False, sort_keys=False)
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to save node config: {e}")

    def _load_node_config(self, node_id: str) -> bool:
        """Load node configuration from YAML file."""
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

        # Load pin configuration
        pin_config_data = config.get('pin_config', {})
        if pin_config_data:
            node.pin_config = NodePinConfig(
                version=pin_config_data.get('version', 0),
                sync_status=pin_config_data.get('sync_status', 'unknown'),
                last_synced=pin_config_data.get('last_synced'),
            )

            for gpio_str, cfg_data in pin_config_data.get('pins', {}).items():
                try:
                    gpio = int(gpio_str)
                except ValueError:
                    continue

                params = {k: v for k, v in cfg_data.items()
                          if k not in ('mode', 'logical_name')}

                node.pin_config.pins[gpio] = PinConfiguration(
                    gpio=gpio,
                    mode=cfg_data.get('mode', 'unconfigured'),
                    logical_name=cfg_data.get('logical_name', ''),
                    params=params,
                )

        return True

    def load_all_node_configs(self):
        """Load all adopted nodes from disk on startup."""
        if not os.path.isdir(self.nodes_config_dir):
            return

        for filename in os.listdir(self.nodes_config_dir):
            if not filename.endswith('.yaml'):
                continue

            node_id = filename[:-5]  # Remove .yaml extension
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

            # Create NodeInfo from saved config
            node = NodeInfo(
                node_id=config.get('node_id', node_id),
                display_name=config.get('display_name', ''),
                role=config.get('role', ''),
                hardware_model=config.get('hardware_model', 'Unknown'),
                mac_address=config.get('mac_address', ''),
                online=False,  # Will be updated when node announces
                last_seen=0,
            )

            # Add to adopted nodes
            self.state.adopted_nodes[node_id] = node

            # Load pin configuration
            pin_config_data = config.get('pin_config', {})
            if pin_config_data:
                node.pin_config = NodePinConfig(
                    version=pin_config_data.get('version', 0),
                    sync_status=pin_config_data.get('sync_status', 'unknown'),
                    last_synced=pin_config_data.get('last_synced'),
                )

                for gpio_str, cfg_data in pin_config_data.get('pins', {}).items():
                    try:
                        gpio = int(gpio_str)
                    except ValueError:
                        continue

                    params = {k: v for k, v in cfg_data.items()
                              if k not in ('mode', 'logical_name')}

                    node.pin_config.pins[gpio] = PinConfiguration(
                        gpio=gpio,
                        mode=cfg_data.get('mode', 'unconfigured'),
                        logical_name=cfg_data.get('logical_name', ''),
                        params=params,
                    )

            if self.logger:
                self.logger.info(f"Loaded adopted node from config: {node_id} ({node.role})")

    # =========================================================================
    # Runtime State Methods
    # =========================================================================

    def get_or_create_runtime_state(self, node_id: str) -> Optional[NodeRuntimeState]:
        """Get or create runtime state for a node."""
        node = self.state.adopted_nodes.get(node_id)
        if not node:
            return None

        if not node.runtime_state:
            node.runtime_state = NodeRuntimeState(node_id=node_id)

            # Initialize pins from pin_config if available
            if node.pin_config:
                for gpio, config in node.pin_config.pins.items():
                    if config.mode and config.mode != 'unconfigured':
                        node.runtime_state.pins[gpio] = PinRuntimeState(
                            gpio=gpio,
                            mode=config.mode,
                            logical_name=config.logical_name,
                        )

        return node.runtime_state

    def update_pin_desired(self, node_id: str, gpio: int, value: float) -> bool:
        """
        Update desired value for a pin.

        Args:
            node_id: The node ID
            gpio: GPIO number
            value: Desired value (interpretation depends on mode)

        Returns:
            True if update successful
        """
        runtime_state = self.get_or_create_runtime_state(node_id)
        if not runtime_state:
            return False

        if gpio not in runtime_state.pins:
            # Try to create from pin_config
            node = self.state.adopted_nodes.get(node_id)
            if node and node.pin_config and gpio in node.pin_config.pins:
                config = node.pin_config.pins[gpio]
                runtime_state.pins[gpio] = PinRuntimeState(
                    gpio=gpio,
                    mode=config.mode,
                    logical_name=config.logical_name,
                )
            else:
                return False

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

        runtime_state.update_from_firmware(pins_data)
        return True

    def get_runtime_state(self, node_id: str) -> Optional[Dict[str, Any]]:
        """Get runtime state for a node as a dictionary."""
        runtime_state = self.get_or_create_runtime_state(node_id)
        if not runtime_state:
            return None

        return runtime_state.to_dict()

    def get_controllable_pins(self, node_id: str) -> List[Dict[str, Any]]:
        """
        Get list of controllable pins for a node (PWM, servo, digital_out).

        Returns list of pin info dicts with gpio, mode, logical_name.
        """
        node = self.state.adopted_nodes.get(node_id)
        if not node or not node.pin_config:
            return []

        controllable_modes = {'pwm', 'servo', 'digital_out'}
        result = []

        for gpio, config in node.pin_config.pins.items():
            if config.mode in controllable_modes:
                result.append({
                    'gpio': gpio,
                    'mode': config.mode,
                    'logical_name': config.logical_name,
                })

        return result

    def get_readable_pins(self, node_id: str) -> List[Dict[str, Any]]:
        """
        Get list of readable pins for a node (digital_in, adc).

        Returns list of pin info dicts with gpio, mode, logical_name.
        """
        node = self.state.adopted_nodes.get(node_id)
        if not node or not node.pin_config:
            return []

        readable_modes = {'digital_in', 'adc'}
        result = []

        for gpio, config in node.pin_config.pins.items():
            if config.mode in readable_modes:
                result.append({
                    'gpio': gpio,
                    'mode': config.mode,
                    'logical_name': config.logical_name,
                })

        return result

    # =========================================================================
    # Role-based Lookup Methods
    # =========================================================================

    def get_node_by_role(self, role: str) -> Optional[NodeInfo]:
        """
        Find an adopted node by its role.

        Args:
            role: The role name (e.g., 'head', 'tracks')

        Returns:
            NodeInfo if found, None otherwise
        """
        for node in self.state.adopted_nodes.values():
            if node.role == role:
                return node
        return None

    def get_nodes_by_role(self, role: str) -> List[NodeInfo]:
        """
        Find all adopted nodes with a specific role.

        Args:
            role: The role name (e.g., 'head', 'tracks')

        Returns:
            List of NodeInfo objects with the specified role
        """
        return [node for node in self.state.adopted_nodes.values() if node.role == role]

    def get_gpio_for_function(self, node_id: str, logical_name: str) -> Optional[int]:
        """
        Find the GPIO pin number for a logical function on a node.

        Args:
            node_id: The node identifier
            logical_name: The logical function name (e.g., 'eye_left_lr', 'left_velocity')

        Returns:
            GPIO pin number if found, None otherwise
        """
        node = self.state.adopted_nodes.get(node_id)
        if not node or not node.pin_config:
            return None

        for gpio, config in node.pin_config.pins.items():
            if config.logical_name == logical_name:
                return gpio
        return None

    def resolve_function_to_pin(self, role: str, function: str) -> Optional[Tuple[str, int]]:
        """
        Resolve a role + function to node_id + GPIO.

        Args:
            role: The role name (e.g., 'head', 'tracks')
            function: The logical function name (e.g., 'eye_left_lr', 'left_velocity')

        Returns:
            Tuple of (node_id, gpio) if found, None otherwise
        """
        node = self.get_node_by_role(role)
        if not node:
            return None

        gpio = self.get_gpio_for_function(node.node_id, function)
        if gpio is None:
            return None

        return (node.node_id, gpio)

    # =========================================================================
    # Firmware Management Methods
    # =========================================================================

    def _get_firmware_dir(self) -> Optional[str]:
        """Get the firmware build directory path."""
        # Method 1: Try relative to this file (saint_server/webserver/state_manager.py)
        # Path: ../.. -> saint_os, then firmware/rp2040
        current_dir = os.path.dirname(__file__)
        firmware_dir = os.path.abspath(os.path.join(current_dir, '..', '..', 'firmware', 'rp2040'))
        if os.path.isdir(firmware_dir):
            return firmware_dir

        # Method 2: Try to find firmware directory relative to ROS2 package
        try:
            from ament_index_python.packages import get_package_share_directory
            package_dir = get_package_share_directory('saint_os')
            # install/saint_os/share/saint_os -> source/saint_os/firmware/rp2040
            base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(package_dir))))
            firmware_dir = os.path.join(base_dir, 'saint_os', 'firmware', 'rp2040')
            if os.path.isdir(firmware_dir):
                return firmware_dir
        except Exception:
            pass

        # Method 3: Check common development paths
        dev_paths = [
            os.path.expanduser('~/Projects/OpenSAINT/SaintOS/source/saint_os/firmware/rp2040'),
            '/opt/saint_os/firmware/rp2040',
        ]
        for path in dev_paths:
            if os.path.isdir(path):
                return path

        return None

    def _parse_version(self, version_str: str) -> tuple:
        """Parse version string into comparable tuple."""
        if not version_str:
            return (0, 0, 0)
        # Handle versions like "1.1.0" or "1.1.0-abc123"
        match = re.match(r'^(\d+)\.(\d+)\.(\d+)', version_str)
        if match:
            return (int(match.group(1)), int(match.group(2)), int(match.group(3)))
        return (0, 0, 0)

    def get_server_firmware_info(self) -> Dict[str, Any]:
        """
        Get the latest server firmware version and build info.

        Returns dict with:
            - version: Version string (e.g., "1.1.0")
            - version_full: Full version with git hash (e.g., "1.1.0-abc123-dirty")
            - git_hash: Git commit hash
            - build_date: Build date if available
            - elf_path: Path to ELF file (for simulation)
            - uf2_path: Path to UF2 file (for hardware)
            - available: Whether firmware files are available
        """
        result = {
            "version": "0.0.0",
            "version_full": None,
            "git_hash": None,
            "build_date": None,
            "elf_path": None,
            "uf2_path": None,
            "available": False,
        }

        firmware_dir = self._get_firmware_dir()
        if not firmware_dir:
            result["error"] = "Firmware directory not found"
            return result

        # Try hardware build first, then simulation build
        for build_type in ['build_hardware', 'build_sim']:
            build_dir = os.path.join(firmware_dir, build_type)
            if not os.path.isdir(build_dir):
                continue

            elf_path = os.path.join(build_dir, 'saint_node.elf')
            uf2_path = os.path.join(build_dir, 'saint_node.uf2')

            if os.path.isfile(elf_path):
                result["elf_path"] = elf_path
                result["available"] = True
                result["build_type"] = build_type

                # Get build date from file modification time
                mtime = os.path.getmtime(elf_path)
                result["build_date"] = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(mtime))

            if os.path.isfile(uf2_path):
                result["uf2_path"] = uf2_path

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

            # Fallback: Try to read version from CMakeLists.txt
            if result["version"] == "0.0.0":
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

        firmware_dir = self._get_firmware_dir()
        if not firmware_dir:
            return result

        build_dir_name = 'build_sim' if build_type == 'simulation' else 'build_hardware'
        build_dir = os.path.join(firmware_dir, build_dir_name)

        if not os.path.isdir(build_dir):
            return result

        elf_path = os.path.join(build_dir, 'saint_node.elf')
        uf2_path = os.path.join(build_dir, 'saint_node.uf2')

        if os.path.isfile(elf_path):
            result["elf_path"] = elf_path
            result["available"] = True

            mtime = os.path.getmtime(elf_path)
            result["build_date"] = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(mtime))

        if os.path.isfile(uf2_path):
            result["uf2_path"] = uf2_path

        # Read version from generated version.h
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

        node_tuple = self._parse_version(node_version)
        server_tuple = self._parse_version(server_version)

        update_available = False
        message = ""

        if not server_fw["available"]:
            message = "No firmware build available on server"
        elif server_tuple > node_tuple:
            # Server has newer version number
            update_available = True
            message = f"Update available: {node_version} → {server_version}"
        elif server_tuple == node_tuple:
            # Same version number - check if build is different
            # Compare build timestamps if available
            server_build = server_fw.get("build_date")
            node_build = node.firmware_build  # Build timestamp from node announcement

            if server_build and node_build:
                # Server build is newer if its timestamp is later
                # Node sends format: "2026-01-26 00:14:24"
                # Server has format: "2026-01-26 00:17:53"
                if server_build > node_build:
                    update_available = True
                    message = f"Rebuild available: {node_build} → {server_build}"
                else:
                    message = "Firmware is up to date"
            else:
                message = "Firmware is up to date"
        else:
            message = "Node firmware is newer than server"

        return {
            "available": update_available,
            "current_version": node_version,
            "server_version": server_version,
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
            # Use existing method for RP2040
            rp2040_info = self.get_server_firmware_info()
            result["available"] = rp2040_info.get("available", False)
            result["version"] = rp2040_info.get("version", "0.0.0")
            result["build_date"] = rp2040_info.get("build_date")
            result["elf_path"] = rp2040_info.get("elf_path")
            result["uf2_path"] = rp2040_info.get("uf2_path")
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
