"""File-backed persistence for animations and poses.

One JSON file per record, named ``<slug>.json``. The slug is the
record ID — operators pick the human-readable name, the server
derives the slug from it on first save.

Lives under the runtime config dir alongside ``nodes/`` and
``system_routing.yaml`` so it survives ROS package reinstalls.
"""

from __future__ import annotations

import json
import os
import re
import shutil
import threading
from typing import Dict, List, Optional

from saint_server.animation.models import Animation, Pose


_SLUG_RE = re.compile(r"[^a-z0-9_-]+")
_RESERVED_SLUGS = {"", ".", ".."}


def slugify(name: str) -> str:
    """Stable, filename-safe slug derived from a human name."""
    s = (name or "").strip().lower().replace(" ", "_")
    s = _SLUG_RE.sub("", s)
    s = re.sub(r"_+", "_", s).strip("_-")
    if s in _RESERVED_SLUGS:
        return "untitled"
    return s[:64]


class _JSONStore:
    """Shared loader/saver for `<dir>/<id>.json` files.

    Operations are guarded by an instance lock so concurrent WebSocket
    handlers don't trample each other on save. Reads aren't locked —
    a partial overwrite would surface as a JSONDecodeError which the
    caller handles.
    """

    def __init__(self, base_dir: str, logger=None):
        self.base_dir = base_dir
        self.logger = logger
        self._lock = threading.Lock()

    def _ensure_dir(self) -> None:
        os.makedirs(self.base_dir, exist_ok=True)

    def _path_for(self, item_id: str) -> str:
        # Defensive: never let an unsanitized id escape the base dir.
        safe_id = slugify(item_id)
        if not safe_id:
            raise ValueError(f"Invalid id: {item_id!r}")
        return os.path.join(self.base_dir, f"{safe_id}.json")

    def list_ids(self) -> List[str]:
        if not os.path.isdir(self.base_dir):
            return []
        out = []
        for name in os.listdir(self.base_dir):
            if name.endswith(".json"):
                out.append(name[:-5])
        return sorted(out)

    def read_raw(self, item_id: str) -> Optional[Dict]:
        path = self._path_for(item_id)
        if not os.path.isfile(path):
            return None
        try:
            with open(path, "r") as f:
                return json.load(f)
        except Exception as e:
            self._log("warning", f"Failed to read {path}: {e}")
            return None

    def write_raw(self, item_id: str, data: Dict) -> str:
        self._ensure_dir()
        path = self._path_for(item_id)
        tmp = path + ".tmp"
        with self._lock:
            with open(tmp, "w") as f:
                json.dump(data, f, indent=2)
            os.replace(tmp, path)
        return path

    def delete(self, item_id: str) -> bool:
        path = self._path_for(item_id)
        if os.path.isfile(path):
            os.unlink(path)
            return True
        return False

    def _log(self, level: str, msg: str) -> None:
        if self.logger:
            getattr(self.logger, level)(msg)


class AnimationStore:
    """JSON-backed animation library at ``{config_dir}/animations/``."""

    def __init__(self, config_dir: str, logger=None):
        self.config_dir = config_dir
        self.store = _JSONStore(os.path.join(config_dir, "animations"), logger=logger)
        self.logger = logger

    def list(self) -> List[Dict]:
        """Lightweight summary list for the sidebar."""
        out = []
        for aid in self.store.list_ids():
            raw = self.store.read_raw(aid)
            if not raw:
                continue
            out.append({
                "id": raw.get("id", aid),
                "name": raw.get("name", aid),
                "duration": raw.get("duration", 0.0),
                "fps": raw.get("fps", 60),
                "loop": raw.get("loop", False),
                "icon": raw.get("icon", ""),
                "group": raw.get("group", ""),
                "value_tracks": len(raw.get("value_tracks", [])),
                "trigger_tracks": len(raw.get("trigger_tracks", [])),
                "modified": raw.get("modified", ""),
            })
        return out

    def get(self, animation_id: str) -> Optional[Animation]:
        raw = self.store.read_raw(animation_id)
        if not raw:
            return None
        try:
            return Animation.from_dict(raw)
        except Exception as e:
            self._log("warning", f"Failed to parse animation {animation_id}: {e}")
            return None

    def save(self, anim: Animation) -> Animation:
        """Persist; assigns id if blank and stamps timestamps."""
        if not anim.id:
            anim.id = slugify(anim.name) or "untitled"
        else:
            anim.id = slugify(anim.id)
        anim.stamp()
        self.store.write_raw(anim.id, anim.to_dict())
        return anim

    def delete(self, animation_id: str) -> bool:
        return self.store.delete(animation_id)

    def _log(self, level: str, msg: str) -> None:
        if self.logger:
            getattr(self.logger, level)(msg)


class PoseStore:
    """JSON-backed pose library at ``{config_dir}/poses/``."""

    def __init__(self, config_dir: str, logger=None):
        self.config_dir = config_dir
        self.store = _JSONStore(os.path.join(config_dir, "poses"), logger=logger)
        self.logger = logger

    def list(self) -> List[Dict]:
        out = []
        for pid in self.store.list_ids():
            raw = self.store.read_raw(pid)
            if not raw:
                continue
            out.append({
                "id": raw.get("id", pid),
                "name": raw.get("name", pid),
                "icon": raw.get("icon", ""),
                "group": raw.get("group", ""),
                "description": raw.get("description", ""),
                "setpoint_count": len(raw.get("setpoints", [])),
                "modified": raw.get("modified", ""),
            })
        return out

    def get(self, pose_id: str) -> Optional[Pose]:
        raw = self.store.read_raw(pose_id)
        if not raw:
            return None
        try:
            return Pose.from_dict(raw)
        except Exception as e:
            self._log("warning", f"Failed to parse pose {pose_id}: {e}")
            return None

    def save(self, pose: Pose) -> Pose:
        if not pose.id:
            pose.id = slugify(pose.name) or "untitled"
        else:
            pose.id = slugify(pose.id)
        pose.stamp()
        self.store.write_raw(pose.id, pose.to_dict())
        return pose

    def delete(self, pose_id: str) -> bool:
        return self.store.delete(pose_id)

    def _log(self, level: str, msg: str) -> None:
        if self.logger:
            getattr(self.logger, level)(msg)
