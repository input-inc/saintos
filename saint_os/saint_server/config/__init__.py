"""
SAINT.OS Server Configuration Module

Loads and provides access to server configuration from server_config.yaml.
"""

import os
from typing import Any, Optional
from dataclasses import dataclass, field

import yaml


@dataclass
class WebSocketConfig:
    """WebSocket configuration."""
    password: Optional[str] = None
    auth_timeout: float = 10.0


@dataclass
class NetworkConfig:
    """Network configuration."""
    web_port: int = 80
    websocket_port: Optional[int] = None


@dataclass
class ROSBridgeConfig:
    """ROS Bridge configuration."""
    throttle_ms: int = 50


@dataclass
class LiveLinkConfig:
    """LiveLink configuration."""
    enabled: bool = True
    port: int = 11111


@dataclass
class ServerConfig:
    """Complete server configuration."""
    name: str = "SAINT-01"
    websocket: WebSocketConfig = field(default_factory=WebSocketConfig)
    network: NetworkConfig = field(default_factory=NetworkConfig)
    ros_bridge: ROSBridgeConfig = field(default_factory=ROSBridgeConfig)
    livelink: LiveLinkConfig = field(default_factory=LiveLinkConfig)


# Global config instance
_config: Optional[ServerConfig] = None


def load_config(config_path: Optional[str] = None) -> ServerConfig:
    """
    Load server configuration from YAML file.

    Args:
        config_path: Path to config file (default: server_config.yaml in this directory)

    Returns:
        ServerConfig instance
    """
    global _config

    if config_path is None:
        config_path = os.path.join(os.path.dirname(__file__), 'server_config.yaml')

    config = ServerConfig()

    if os.path.exists(config_path):
        with open(config_path, 'r') as f:
            data = yaml.safe_load(f) or {}

        # Server settings
        server_data = data.get('server', {})
        if server_data:
            config.name = server_data.get('name', config.name)

        # WebSocket settings
        ws_data = data.get('websocket', {})
        if ws_data:
            config.websocket.password = ws_data.get('password')
            config.websocket.auth_timeout = ws_data.get('auth_timeout', config.websocket.auth_timeout)

        # Network settings
        net_data = data.get('network', {})
        if net_data:
            config.network.web_port = net_data.get('web_port', config.network.web_port)
            config.network.websocket_port = net_data.get('websocket_port')

        # ROS Bridge settings
        ros_data = data.get('ros_bridge', {})
        if ros_data:
            config.ros_bridge.throttle_ms = ros_data.get('throttle_ms', config.ros_bridge.throttle_ms)

        # LiveLink settings
        ll_data = data.get('livelink', {})
        if ll_data:
            config.livelink.enabled = ll_data.get('enabled', config.livelink.enabled)
            config.livelink.port = ll_data.get('port', config.livelink.port)

    _config = config
    return config


def get_config() -> ServerConfig:
    """
    Get the current server configuration.

    Loads from file if not already loaded.
    """
    global _config
    if _config is None:
        load_config()
    return _config


__all__ = [
    'ServerConfig',
    'WebSocketConfig',
    'NetworkConfig',
    'ROSBridgeConfig',
    'LiveLinkConfig',
    'load_config',
    'get_config',
]
