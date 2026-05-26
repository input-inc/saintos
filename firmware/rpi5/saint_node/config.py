"""
Configuration Manager for Raspberry Pi 5 Node

Handles persistent storage of node configuration including:
- Node identity (ID, role, display name)
- Pin configurations
- Network settings

Configuration is stored in YAML format in /etc/saint-node/ or ~/.saint-node/
"""

import os
import yaml
from typing import Dict, Any, Optional
from pathlib import Path


class ConfigManager:
    """
    Manages persistent node configuration.

    Configuration files are stored in:
    - /etc/saint-node/ (system-wide, requires root)
    - ~/.saint-node/ (user-level fallback)
    """

    CONFIG_FILENAME = "config.yaml"

    def __init__(self, node_id: str, logger=None):
        self._node_id = node_id
        self._logger = logger
        self._config: Dict[str, Any] = {}
        self._config_dir = self._find_config_dir()
        self._config_path = self._config_dir / self.CONFIG_FILENAME

    def _log(self, level: str, msg: str):
        """Log a message."""
        if self._logger:
            getattr(self._logger, level)(msg)
        else:
            print(f"[{level.upper()}] {msg}")

    def _find_config_dir(self) -> Path:
        """Find or create configuration directory."""
        # Check for simulation/override directory
        override_dir = os.environ.get('SAINT_CONFIG_DIR')
        if override_dir:
            path = Path(override_dir)
            path.mkdir(parents=True, exist_ok=True)
            return path

        # Try system directory first
        system_dir = Path("/etc/saint-node")
        if system_dir.exists() or self._can_create_dir(system_dir):
            return system_dir

        # Fall back to user directory
        user_dir = Path.home() / ".saint-node"
        user_dir.mkdir(parents=True, exist_ok=True)
        return user_dir

    def _can_create_dir(self, path: Path) -> bool:
        """Check if we can create a directory."""
        try:
            path.mkdir(parents=True, exist_ok=True)
            return True
        except PermissionError:
            return False

    def load(self) -> bool:
        """Load configuration from file."""
        if not self._config_path.exists():
            self._log('info', f'No config file found at {self._config_path}')
            self._config = self._default_config()
            return False

        try:
            with open(self._config_path, 'r') as f:
                self._config = yaml.safe_load(f) or {}

            self._log('info', f'Loaded config from {self._config_path}')
            return True

        except Exception as e:
            self._log('error', f'Failed to load config: {e}')
            self._config = self._default_config()
            return False

    def save(self) -> bool:
        """Save configuration to file."""
        try:
            # Ensure directory exists
            self._config_dir.mkdir(parents=True, exist_ok=True)

            with open(self._config_path, 'w') as f:
                yaml.dump(self._config, f, default_flow_style=False, sort_keys=False)

            self._log('info', f'Saved config to {self._config_path}')
            return True

        except Exception as e:
            self._log('error', f'Failed to save config: {e}')
            return False

    def _default_config(self) -> Dict[str, Any]:
        """Return default configuration."""
        return {
            'node_id': self._node_id,
            'role': None,
            'display_name': '',
            'adopted': False,
            'pins': {},
            'network': {
                'agent_host': '192.168.1.1',
                'agent_port': 8888,
            },
        }

    def is_adopted(self) -> bool:
        """Check if node has been adopted."""
        return self._config.get('adopted', False)

    def get_role(self) -> Optional[str]:
        """Get assigned role."""
        return self._config.get('role')

    def set_role(self, role: str):
        """Set assigned role."""
        self._config['role'] = role
        self._config['adopted'] = True

    def get_display_name(self) -> str:
        """Get display name."""
        return self._config.get('display_name', '')

    def set_display_name(self, name: str):
        """Set display name."""
        self._config['display_name'] = name

    def get_pin_configs(self) -> Dict[str, Any]:
        """Get all pin configurations."""
        return self._config.get('pins', {})

    def set_pin_configs(self, pins: Dict[str, Any]):
        """Set all pin configurations."""
        self._config['pins'] = pins

    def get_pin_config(self, gpio: int) -> Optional[Dict[str, Any]]:
        """Get configuration for a specific pin."""
        return self._config.get('pins', {}).get(str(gpio))

    def set_pin_config(self, gpio: int, config: Dict[str, Any]):
        """Set configuration for a specific pin."""
        if 'pins' not in self._config:
            self._config['pins'] = {}
        self._config['pins'][str(gpio)] = config

    def get_network_config(self) -> Dict[str, Any]:
        """Get network configuration."""
        return self._config.get('network', {})

    def set_network_config(self, config: Dict[str, Any]):
        """Set network configuration."""
        self._config['network'] = config

    def factory_reset(self):
        """Reset to factory defaults."""
        self._log('warn', 'Performing factory reset')

        # Clear configuration
        self._config = self._default_config()

        # Remove config file
        if self._config_path.exists():
            try:
                self._config_path.unlink()
                self._log('info', f'Removed config file: {self._config_path}')
            except Exception as e:
                self._log('error', f'Failed to remove config file: {e}')

    def to_dict(self) -> Dict[str, Any]:
        """Return configuration as dictionary."""
        return self._config.copy()
