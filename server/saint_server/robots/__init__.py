"""
SAINT.OS Robot Manifest Manager

A robot manifest (config/robots/<id>.yaml) describes one physical
platform: its identity (name/description/homepage) and the list of node
role names its builder adopts. This replaces the old per-role YAML files
(config/roles/*), whose logical_functions/default_params/compatible_modes
were loaded but never consumed — a role is just a human label for a node.

Drop a new manifest in the robots directory to run SAINT.OS on a
different robot; the active one is selected by the `active_robot` server
setting (defaults to the only/first manifest found).
"""

import os
import yaml
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any


@dataclass
class RobotManifest:
    """One robot platform definition."""
    id: str
    name: str
    description: str = ""
    homepage: str = ""
    roles: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "description": self.description,
            "homepage": self.homepage,
            "roles": list(self.roles),
        }


class RobotManager:
    """Loads robot manifests and tracks which one is active."""

    def __init__(self, robots_dir: Optional[str] = None,
                 active_id: Optional[str] = None, logger=None):
        self.logger = logger
        self.robots: Dict[str, RobotManifest] = {}
        self._active_id: Optional[str] = active_id

        if robots_dir is None:
            robots_dir = self._default_robots_dir()
        self.robots_dir = os.path.abspath(robots_dir)
        self._load()

    def _default_robots_dir(self) -> str:
        """Resolve the robots directory.

        Prefers a writable operator location so builders can drop in
        their own manifest without touching the read-only package:
          1. $SAINT_CONFIG_DIR/robots (set to /etc/saint-os on install)
          2. the installed package share (config/robots)
          3. the in-repo path for dev / unit tests
        """
        cfg_dir = os.environ.get('SAINT_CONFIG_DIR')
        if cfg_dir:
            cand = os.path.join(cfg_dir, 'robots')
            if os.path.isdir(cand):
                return cand
        try:
            from ament_index_python.packages import get_package_share_directory
            cand = os.path.join(
                get_package_share_directory('saint_os'), 'config', 'robots')
            if os.path.isdir(cand):
                return cand
        except Exception:
            pass
        return os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'robots')

    def _log(self, level: str, message: str):
        if self.logger:
            from saint_server.log_level import log_at
            log_at(self.logger, level, message)

    def _load(self):
        self.robots.clear()
        if not os.path.isdir(self.robots_dir):
            self._log('warn', f"Robots directory not found: {self.robots_dir}")
            return
        for filename in sorted(os.listdir(self.robots_dir)):
            # Skip macOS AppleDouble metadata shadows (._foo.yaml) that
            # ride along when a tarball is unpacked through Finder.
            if filename.startswith('._'):
                continue
            if not (filename.endswith('.yaml') or filename.endswith('.yml')):
                continue
            path = os.path.join(self.robots_dir, filename)
            try:
                manifest = self._load_file(path, filename)
                if manifest:
                    self.robots[manifest.id] = manifest
                    self._log('info', f"Loaded robot manifest: {manifest.id}")
            except Exception as e:
                self._log('error', f"Failed to load robot manifest {filename}: {e}")

    def _load_file(self, path: str, filename: str) -> Optional[RobotManifest]:
        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}
        if not isinstance(data, dict) or not (data.get('name') or data.get('id')):
            return None
        # id defaults to the filename stem so a manifest without an
        # explicit id still gets a stable, referenceable key.
        rid = str(data.get('id') or os.path.splitext(filename)[0])
        roles = data.get('roles') or []
        if not isinstance(roles, list):
            roles = []
        return RobotManifest(
            id=rid,
            name=str(data.get('name', rid)),
            description=str(data.get('description', '')),
            homepage=str(data.get('homepage', '')),
            roles=[str(r) for r in roles],
        )

    # ── access ──────────────────────────────────────────────────────

    def set_active(self, robot_id: Optional[str]) -> None:
        """Set the preferred active manifest id (None = auto/first)."""
        self._active_id = robot_id or None

    def get_active(self) -> Optional[RobotManifest]:
        """The active manifest: the configured id if present, else the
        first manifest found (deterministic — sorted by id), else None."""
        if self._active_id and self._active_id in self.robots:
            return self.robots[self._active_id]
        if not self.robots:
            return None
        return self.robots[sorted(self.robots.keys())[0]]

    def get(self, robot_id: str) -> Optional[RobotManifest]:
        return self.robots.get(robot_id)

    def list_manifests(self) -> List[RobotManifest]:
        return [self.robots[k] for k in sorted(self.robots.keys())]

    def active_roles(self) -> List[str]:
        active = self.get_active()
        return list(active.roles) if active else []

    def reload(self):
        self._load()
