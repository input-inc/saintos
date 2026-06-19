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
    # Long-lived shared secret used by Console-kiosk Pis to skip the
    # operator password prompt. Generated on first server start and
    # persisted in server_config.yaml; rotateable from the Settings
    # panel. None means "no kiosk token configured" — kiosks then
    # only work when websocket.password is also None.
    kiosk_token: Optional[str] = None


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
class LoggingConfig:
    """Logging configuration.

    Default is WARNING so the per-tick INFO lines emitted by the
    routing evaluator and ROS bridge stay off the file handler at
    50 Hz. Bump to INFO (or DEBUG) from the Settings panel when
    diagnosing a binding or evaluator issue. Applies live — no
    server restart needed.
    """
    level: str = "WARNING"


@dataclass
class ServerConfig:
    """Complete server configuration."""
    name: str = "SAINT-01"
    websocket: WebSocketConfig = field(default_factory=WebSocketConfig)
    network: NetworkConfig = field(default_factory=NetworkConfig)
    ros_bridge: ROSBridgeConfig = field(default_factory=ROSBridgeConfig)
    livelink: LiveLinkConfig = field(default_factory=LiveLinkConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)


# Global config instance
_config: Optional[ServerConfig] = None


def load_config(config_path: Optional[str] = None) -> ServerConfig:
    """
    Load server configuration from YAML file.

    Args:
        config_path: Path to config file. If None, resolved via
            get_config_path() — which honors SAINT_CONFIG_DIR (set by
            the systemd unit to /etc/saint-os) and falls back to the
            in-package default for dev builds.

    Returns:
        ServerConfig instance
    """
    global _config

    if config_path is None:
        config_path = get_config_path()

    # First-run seeding: if SAINT_CONFIG_DIR points at /etc/saint-os
    # but that file doesn't exist yet (e.g. a fresh install where
    # packaging/install.sh seeded it but then something deleted it,
    # or a non-standard install where the seeding step was skipped),
    # copy the in-package default into place so we have a writable
    # target for the auto-generated kiosk_token below. Without this,
    # save_config() would try to write the in-package YAML (read-only
    # for the saint service user) and the token would never persist.
    if not os.path.exists(config_path):
        pkg_default = os.path.join(os.path.dirname(__file__), 'server_config.yaml')
        if pkg_default != config_path and os.path.exists(pkg_default):
            try:
                os.makedirs(os.path.dirname(config_path), exist_ok=True)
                import shutil as _shutil
                _shutil.copy2(pkg_default, config_path)
            except OSError:
                # Best-effort — load_config below still works with a
                # missing path (config stays at defaults).
                pass

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
            config.websocket.kiosk_token = ws_data.get('kiosk_token') or None

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

        # Logging settings
        log_data = data.get('logging', {})
        if log_data:
            level = log_data.get('level', config.logging.level)
            if isinstance(level, str) and level.strip():
                config.logging.level = level.strip().upper()

    # Auto-generate the kiosk token on first run so Console-kiosk Pis
    # have a stable shared secret to authenticate with — without
    # forcing the operator to set one by hand. We persist immediately
    # so the next read sees the same value; rotateable later.
    #
    # The persist used to silently swallow exceptions. That hid a real
    # bug: when config_path resolved to the in-package YAML (owned by
    # root, under /opt/saint-os/install/), the write threw PermissionError
    # every run, the token never made it to disk, and every restart
    # rolled a new in-memory token — Pi kiosks could never authenticate
    # across restarts. With get_config_path() now honoring
    # SAINT_CONFIG_DIR (/etc/saint-os/), the save should succeed; log
    # the failure path explicitly so the next time it breaks we see it
    # in journalctl instead of guessing.
    if config.websocket.kiosk_token is None:
        import secrets
        config.websocket.kiosk_token = secrets.token_urlsafe(32)
        _config = config
        if not save_config(config):
            import logging as _logging
            _logging.getLogger('saint_server.config').error(
                "Auto-generated kiosk_token but FAILED to persist to %s — "
                "kiosks won't authenticate across server restarts. Check "
                "write permission on the config directory.", config_path)

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
    """Get the path to the config file.

    Resolution order:

    1. ``SAINT_CONFIG_DIR`` env var (the systemd unit sets this to
       /etc/saint-os under a normal install). The file is then
       ``$SAINT_CONFIG_DIR/server_config.yaml``. This is the writable
       runtime path operators edit and the place auto-generated values
       (e.g. kiosk_token) need to land for them to survive a restart.
    2. The in-package YAML — `os.path.dirname(__file__) + /server_config.yaml`.
       This is the seed copy that ships in the dist tarball; on a fresh
       install packaging/install.sh seeds it into /etc/saint-os. It's
       also what dev / unit-test runs read from (no env var, no
       /etc/saint-os).

    Previously this always returned (2), which silently broke
    auto-token persistence under a real install because (2) is
    read-only after install. Honoring SAINT_CONFIG_DIR fixes that.
    """
    config_dir = os.environ.get('SAINT_CONFIG_DIR')
    if config_dir:
        return os.path.join(config_dir, 'server_config.yaml')
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
            'kiosk_token': config.websocket.kiosk_token,
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
        'logging': {
            'level': config.logging.level,
        },
    }

    # Only include websocket_port if set
    if config.network.websocket_port:
        data['network']['websocket_port'] = config.network.websocket_port

    try:
        # Make sure the dir exists. /etc/saint-os should always be there
        # under a normal install (packaging/install.sh creates it) but
        # not in dev / test runs where get_config_path() picks the
        # in-package path.
        os.makedirs(os.path.dirname(config_path), exist_ok=True)
        with open(config_path, 'w') as f:
            # Add header comment
            f.write("# SAINT.OS Server Configuration\n")
            f.write("#\n")
            f.write("# This file configures the server's runtime behavior.\n")
            f.write("# Restart the server after making changes.\n\n")
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        return True
    except OSError as e:
        # Log the path + errno so an operator chasing "my kiosk_token
        # isn't persisting" sees the cause instead of a silent return.
        # OSError covers PermissionError, IsADirectoryError, FileNotFound,
        # plus the rare disk-full case.
        import logging as _logging
        _logging.getLogger('saint_server.config').error(
            "save_config: failed to write %s: %s", config_path, e)
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
            'kiosk_token': config.websocket.kiosk_token,
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
        'logging': {
            'level': config.logging.level,
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
        if 'kiosk_token' in ws:
            _config.websocket.kiosk_token = ws['kiosk_token'] if ws['kiosk_token'] else None

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

    # Logging settings
    if 'logging' in data:
        log = data['logging']
        if 'level' in log:
            level = log['level']
            if isinstance(level, str) and level.strip():
                _config.logging.level = level.strip().upper()

    return _config


__all__ = [
    'ServerConfig',
    'WebSocketConfig',
    'NetworkConfig',
    'ROSBridgeConfig',
    'LiveLinkConfig',
    'LoggingConfig',
    'load_config',
    'get_config',
    'get_config_path',
    'save_config',
    'config_to_dict',
    'update_config_from_dict',
]
