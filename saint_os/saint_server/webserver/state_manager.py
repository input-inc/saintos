"""
SAINT.OS State Manager

Manages system state and provides data for WebSocket clients.
"""

import time
import psutil
from dataclasses import dataclass, field
from typing import Dict, List, Any, Optional


@dataclass
class NodeInfo:
    """Information about an adopted or unadopted node."""
    node_id: str
    hardware_model: str = "Raspberry Pi 4 Model B"
    mac_address: str = ""
    ip_address: str = ""
    firmware_version: str = "1.0.0"
    online: bool = True
    cpu_temp: float = 45.0
    cpu_usage: float = 15.0
    memory_usage: float = 35.0
    uptime_seconds: int = 0
    # Adopted node specific
    display_name: str = ""
    role: str = ""


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

    def __init__(self, server_name: str = "SAINT-01"):
        self.state = SystemState(server_name=server_name)
        self._init_mock_data()

    def _init_mock_data(self):
        """Initialize mock data for testing."""
        # Add a mock adopted node
        self.state.adopted_nodes["node_001"] = NodeInfo(
            node_id="node_001",
            display_name="Head Unit",
            role="head",
            hardware_model="Raspberry Pi 4 Model B",
            mac_address="dc:a6:32:aa:bb:cc",
            ip_address="192.168.10.11",
            online=True,
            cpu_usage=25.0,
            memory_usage=42.0,
            uptime_seconds=3600,
        )

        # Add a mock unadopted node
        self.state.unadopted_nodes["node_002"] = NodeInfo(
            node_id="node_002",
            hardware_model="Raspberry Pi 4 Model B",
            mac_address="dc:a6:32:dd:ee:ff",
            ip_address="192.168.10.50",
            firmware_version="1.0.0",
            cpu_temp=48.5,
            cpu_usage=10.0,
            memory_usage=28.0,
            uptime_seconds=120,
        )

    def get_system_status(self) -> Dict[str, Any]:
        """Get current system status for broadcasting."""
        try:
            cpu_usage = psutil.cpu_percent(interval=None)
            memory = psutil.virtual_memory()
            memory_usage = memory.percent
        except Exception:
            cpu_usage = 0.0
            memory_usage = 0.0

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
        }

    def get_adopted_nodes(self) -> List[Dict[str, Any]]:
        """Get list of adopted nodes."""
        return [
            {
                "node_id": node.node_id,
                "display_name": node.display_name,
                "role": node.role,
                "hardware_model": node.hardware_model,
                "online": node.online,
                "cpu_usage": node.cpu_usage,
                "memory_usage": node.memory_usage,
                "uptime_seconds": node.uptime_seconds,
            }
            for node in self.state.adopted_nodes.values()
        ]

    def get_unadopted_nodes(self) -> List[Dict[str, Any]]:
        """Get list of unadopted nodes."""
        return [
            {
                "node_id": node.node_id,
                "hardware_model": node.hardware_model,
                "mac_address": node.mac_address,
                "ip_address": node.ip_address,
                "firmware_version": node.firmware_version,
                "cpu_temp": node.cpu_temp,
                "cpu_usage": node.cpu_usage,
                "memory_usage": node.memory_usage,
                "uptime_seconds": node.uptime_seconds,
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

    def set_client_count(self, count: int):
        """Update WebSocket client count."""
        self.state.websocket_client_count = count
