"""
SAINT.OS Role Definitions Manager

Manages role definitions that specify logical functions for different node types.
Each role defines what pins/functions the node needs (e.g., tracks need left_velocity, right_velocity).
"""

import os
import yaml
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any


@dataclass
class LogicalFunction:
    """Defines a logical function that can be mapped to a physical pin."""
    name: str
    display_name: str
    type: str  # pwm_output, servo_output, digital_input, digital_output, adc_input
    required: bool = False
    description: str = ""
    compatible_modes: List[str] = field(default_factory=list)
    default_params: Dict[str, Any] = field(default_factory=dict)


@dataclass
class RoleDefinition:
    """Complete definition of a node role."""
    role: str
    display_name: str
    description: str = ""
    logical_functions: List[LogicalFunction] = field(default_factory=list)

    def get_function(self, name: str) -> Optional[LogicalFunction]:
        """Get a logical function by name."""
        for func in self.logical_functions:
            if func.name == name:
                return func
        return None

    def get_required_functions(self) -> List[LogicalFunction]:
        """Get all required logical functions."""
        return [f for f in self.logical_functions if f.required]

    def get_optional_functions(self) -> List[LogicalFunction]:
        """Get all optional logical functions."""
        return [f for f in self.logical_functions if not f.required]

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            "role": self.role,
            "display_name": self.display_name,
            "description": self.description,
            "logical_functions": [
                {
                    "name": f.name,
                    "display_name": f.display_name,
                    "type": f.type,
                    "required": f.required,
                    "description": f.description,
                    "compatible_modes": f.compatible_modes,
                    "default_params": f.default_params,
                }
                for f in self.logical_functions
            ],
        }


class RoleManager:
    """Manages loading and accessing role definitions."""

    def __init__(self, roles_dir: Optional[str] = None, logger=None):
        self.logger = logger
        self.roles: Dict[str, RoleDefinition] = {}

        # Default roles directory
        if roles_dir is None:
            # Try to find config/roles relative to package
            try:
                from ament_index_python.packages import get_package_share_directory
                roles_dir = os.path.join(
                    get_package_share_directory('saint_os'), 'config', 'roles'
                )
            except Exception:
                # Fallback for development
                roles_dir = os.path.join(
                    os.path.dirname(__file__), '..', '..', 'config', 'roles'
                )

        self.roles_dir = os.path.abspath(roles_dir)
        self._load_roles()

    def _log(self, level: str, message: str):
        """Log a message if logger is available."""
        if self.logger:
            getattr(self.logger, level, self.logger.info)(message)

    def _load_roles(self):
        """Load all role definitions from YAML files."""
        if not os.path.isdir(self.roles_dir):
            self._log('warn', f"Roles directory not found: {self.roles_dir}")
            return

        for filename in os.listdir(self.roles_dir):
            if not filename.endswith('.yaml') and not filename.endswith('.yml'):
                continue

            filepath = os.path.join(self.roles_dir, filename)
            try:
                role = self._load_role_file(filepath)
                if role:
                    self.roles[role.role] = role
                    self._log('info', f"Loaded role definition: {role.role}")
            except Exception as e:
                self._log('error', f"Failed to load role file {filename}: {e}")

    def _load_role_file(self, filepath: str) -> Optional[RoleDefinition]:
        """Load a single role definition from a YAML file."""
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)

        if not data or 'role' not in data:
            return None

        # Parse logical functions
        functions = []
        for func_data in data.get('logical_functions', []):
            func = LogicalFunction(
                name=func_data.get('name', ''),
                display_name=func_data.get('display_name', func_data.get('name', '')),
                type=func_data.get('type', 'digital_output'),
                required=func_data.get('required', False),
                description=func_data.get('description', ''),
                compatible_modes=func_data.get('compatible_modes', []),
                default_params=func_data.get('default_params', {}),
            )
            functions.append(func)

        return RoleDefinition(
            role=data.get('role'),
            display_name=data.get('display_name', data.get('role', '')),
            description=data.get('description', ''),
            logical_functions=functions,
        )

    def get_role(self, role_name: str) -> Optional[RoleDefinition]:
        """Get a role definition by name."""
        return self.roles.get(role_name)

    def get_all_roles(self) -> List[RoleDefinition]:
        """Get all available role definitions."""
        return list(self.roles.values())

    def get_role_names(self) -> List[str]:
        """Get list of all role names."""
        return list(self.roles.keys())

    def reload(self):
        """Reload all role definitions from disk."""
        self.roles.clear()
        self._load_roles()

    def get_roles_summary(self) -> List[Dict[str, Any]]:
        """Get summary of all roles for UI display."""
        return [
            {
                "role": role.role,
                "display_name": role.display_name,
                "description": role.description,
                "required_count": len(role.get_required_functions()),
                "optional_count": len(role.get_optional_functions()),
            }
            for role in self.roles.values()
        ]
