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


def get_config_path() -> str:
    """Get the path to the config file."""
    return os.path.join(os.path.dirname(__file__), 'server_config.yaml')


def save_config(config: Optional[ServerConfig] = None) -> bool:
    """
    Save server configuration to YAML file.

    Args:
        config: ServerConfig to save (uses global if None)

    Returns:
        True if saved successfully
    """
    global _config

    if config is None:
        config = _config
    if config is None:
        return False

    config_path = get_config_path()

    # Build YAML structure
    data = {
        'server': {
            'name': config.name,
        },
        'websocket': {
            'password': config.websocket.password,
            'auth_timeout': config.websocket.auth_timeout,
        },
        'network': {
            'web_port': config.network.web_port,
        },
        'ros_bridge': {
            'throttle_ms': config.ros_bridge.throttle_ms,
        },
        'livelink': {
            'enabled': config.livelink.enabled,
            'port': config.livelink.port,
        },
    }

    # Only include websocket_port if set
    if config.network.websocket_port:
        data['network']['websocket_port'] = config.network.websocket_port

    try:
        with open(config_path, 'w') as f:
            # Add header comment
            f.write("# SAINT.OS Server Configuration\n")
            f.write("#\n")
            f.write("# This file configures the server's runtime behavior.\n")
            f.write("# Restart the server after making changes.\n\n")
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        return True
    except Exception:
        return False


def config_to_dict(config: Optional[ServerConfig] = None) -> dict:
    """
    Convert ServerConfig to dictionary for JSON serialization.

    Args:
        config: ServerConfig to convert (uses global if None)

    Returns:
        Dictionary representation of config
    """
    if config is None:
        config = get_config()

    return {
        'server': {
            'name': config.name,
        },
        'websocket': {
            'password': config.websocket.password,
            'auth_timeout': config.websocket.auth_timeout,
        },
        'network': {
            'web_port': config.network.web_port,
            'websocket_port': config.network.websocket_port,
        },
        'ros_bridge': {
            'throttle_ms': config.ros_bridge.throttle_ms,
        },
        'livelink': {
            'enabled': config.livelink.enabled,
            'port': config.livelink.port,
        },
    }


def update_config_from_dict(data: dict) -> ServerConfig:
    """
    Update the global config from a dictionary.

    Args:
        data: Dictionary with config values

    Returns:
        Updated ServerConfig
    """
    global _config

    if _config is None:
        _config = ServerConfig()

    # Server settings
    if 'server' in data:
        server = data['server']
        if 'name' in server:
            _config.name = server['name']

    # WebSocket settings
    if 'websocket' in data:
        ws = data['websocket']
        if 'password' in ws:
            _config.websocket.password = ws['password'] if ws['password'] else None
        if 'auth_timeout' in ws:
            _config.websocket.auth_timeout = float(ws['auth_timeout'])

    # Network settings
    if 'network' in data:
        net = data['network']
        if 'web_port' in net:
            _config.network.web_port = int(net['web_port'])
        if 'websocket_port' in net:
            _config.network.websocket_port = int(net['websocket_port']) if net['websocket_port'] else None

    # ROS Bridge settings
    if 'ros_bridge' in data:
        ros = data['ros_bridge']
        if 'throttle_ms' in ros:
            _config.ros_bridge.throttle_ms = int(ros['throttle_ms'])

    # LiveLink settings
    if 'livelink' in data:
        ll = data['livelink']
        if 'enabled' in ll:
            _config.livelink.enabled = bool(ll['enabled'])
        if 'port' in ll:
            _config.livelink.port = int(ll['port'])

    return _config


__all__ = [
    'ServerConfig',
    'WebSocketConfig',
    'NetworkConfig',
    'ROSBridgeConfig',
    'LiveLinkConfig',
    'load_config',
    'get_config',
    'get_config_path',
    'save_config',
    'config_to_dict',
    'update_config_from_dict',
]
