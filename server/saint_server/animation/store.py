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

from saint_server.animation.audio_probe import probe_duration
from saint_server.animation.models import (
    PLAYLIST_KINDS, Animation, Playlist, Pose, Sound,
)


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
            # rcutils tracks log calls by file/line; getattr-dispatch
            # funnels every severity through one line and trips rclpy's
            # severity-change check. See saint_server.log_level.log_at.
            from saint_server.log_level import log_at
            log_at(self.logger, level, msg)


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
            # rcutils tracks log calls by file/line; getattr-dispatch
            # funnels every severity through one line and trips rclpy's
            # severity-change check. See saint_server.log_level.log_at.
            from saint_server.log_level import log_at
            log_at(self.logger, level, msg)


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
            setpoints = raw.get("setpoints", [])
            out.append({
                "id": raw.get("id", pid),
                "name": raw.get("name", pid),
                "icon": raw.get("icon", ""),
                "group": raw.get("group", ""),
                "description": raw.get("description", ""),
                "setpoint_count": len(setpoints),
                # Joint-addressed setpoints specifically. A pose can mix
                # joint and ws_input setpoints, and only the joint ones
                # matter to an animation pose track or a rig control
                # target — so the animation editor needs this count, not
                # the total.
                "joint_count": sum(
                    1 for s in setpoints
                    if s.get("target_kind") == "joint" and s.get("joint")),
                "source": raw.get("source", ""),
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
            # rcutils tracks log calls by file/line; getattr-dispatch
            # funnels every severity through one line and trips rclpy's
            # severity-change check. See saint_server.log_level.log_at.
            from saint_server.log_level import log_at
            log_at(self.logger, level, msg)


class SoundStore:
    """JSON-backed soundboard library at ``{config_dir}/sounds/``.

    Unlike animations/poses, sounds carry an explicit ``position`` so the
    operator can drag-order the flat "All Sounds" view; ``list()`` sorts
    by (position, name) and ``reorder()`` rewrites those positions.

    Order *inside* a playlist is a different thing and does not live
    here — the playlist owns it, because the same sound can sit at a
    different slot in each playlist it belongs to, which one int on the
    sound cannot express. See PlaylistStore.
    """

    def __init__(self, config_dir: str, logger=None):
        self.config_dir = config_dir
        self.store = _JSONStore(os.path.join(config_dir, "sounds"), logger=logger)
        self.logger = logger
        # Serializes read-modify-write of one entry, so the startup
        # duration backfill can't overwrite an operator save that lands
        # between its read and its write.
        self._write_lock = threading.Lock()

    def list(self) -> List[Dict]:
        out = []
        for sid in self.store.list_ids():
            raw = self.store.read_raw(sid)
            if not raw:
                continue
            out.append({
                "id": raw.get("id", sid),
                "name": raw.get("name", sid),
                "icon": raw.get("icon", ""),
                "group": raw.get("group", ""),
                "node_id": raw.get("node_id", ""),
                "file_path": raw.get("file_path", ""),
                "output_device": raw.get("output_device", ""),
                "volume": raw.get("volume", 1.0),
                "start_time": raw.get("start_time", 0.0),
                "loop": raw.get("loop", False),
                "loop_count": raw.get("loop_count", 0),
                "position": raw.get("position", 0),
                "duration": raw.get("duration", 0.0),
                "modified": raw.get("modified", ""),
            })
        # Stable order for the flat list: explicit position, then name
        # as a tiebreaker. Grouping used to lead this key; it moved to
        # playlists, which impose their own order on their members.
        out.sort(key=lambda s: (s["position"], s["name"].lower()))
        return out

    def get(self, sound_id: str) -> Optional[Sound]:
        raw = self.store.read_raw(sound_id)
        if not raw:
            return None
        try:
            return Sound.from_dict(raw)
        except Exception as e:
            self._log("warning", f"Failed to parse sound {sound_id}: {e}")
            return None

    def save(self, sound: Sound) -> Sound:
        if not sound.id:
            sound.id = slugify(sound.name) or "untitled"
        else:
            sound.id = slugify(sound.id)
        # New entries land at the end of the flat list unless a position
        # was set explicitly by the caller. (Where they land inside a
        # playlist is the playlist's business.)
        if sound.position == 0 and self.store.read_raw(sound.id) is None:
            peers = self.list()
            sound.position = max((s["position"] for s in peers), default=0) + 1
        # Measure the clip if we have not, or if the file changed under
        # an existing entry. Probing is cheap for the common .wav case
        # (a header read) and bounded for the rest; an unreadable file
        # just leaves duration at 0 = unknown.
        with self._write_lock:
            prior = self.store.read_raw(sound.id)
            path_changed = bool(prior) and prior.get("file_path") != sound.file_path
            if sound.file_path and (sound.duration <= 0 or path_changed):
                sound.duration = probe_duration(sound.file_path)
            sound.stamp()
            self.store.write_raw(sound.id, sound.to_dict())
        return sound

    def measure(self, sound_id: str) -> Optional[float]:
        """The clip's length in seconds, measuring and storing it first if
        the entry has none yet. None when there is no such sound; 0.0 when
        it can't be measured (file missing, or it lives on another node).

        Unlike reprobe(), a length already stored is returned as is and
        `modified` is not stamped: this is a lookup, not an edit.
        """
        raw = self.store.read_raw(sound_id)
        if not raw:
            return None
        if float(raw.get("duration") or 0) > 0:
            return float(raw["duration"])
        return self._fill_duration(sound_id, raw)

    def _fill_duration(self, sound_id: str, raw: Dict) -> float:
        """Probe an unmeasured entry and store the result. Returns the
        length, or 0.0 if it couldn't be measured or the entry changed
        while probing (that save measured its own file)."""
        path = raw.get("file_path") or ""
        if not path:
            return 0.0
        seconds = probe_duration(path)
        if seconds <= 0:
            return 0.0
        with self._write_lock:
            # Re-read: the operator may have saved or re-pointed this
            # sound while ffprobe ran. Only fill a duration that is still
            # missing for the file we actually measured.
            current = self.store.read_raw(sound_id)
            if (not current
                    or float(current.get("duration") or 0) > 0
                    or (current.get("file_path") or "") != path):
                return 0.0
            current["duration"] = seconds
            self.store.write_raw(sound_id, current)
        return seconds

    def backfill_durations(self) -> int:
        """Measure every entry that has no duration yet. Returns how many
        were filled in.

        save() only probes when a sound is written, so a library saved
        before probing existed stays unmeasured forever, and the timeline
        can only draw those clips as a start marker, not a length bar.
        Meant to run once at startup, off the main thread: an ffprobe
        per file adds up on a large library.

        An entry that still can't be measured (file missing, or it lives
        on another node) is left at 0 and simply tried again next start.
        `modified` is not stamped: measuring is not an operator edit.
        """
        filled = 0
        for sid in self.store.list_ids():
            raw = self.store.read_raw(sid)
            if not raw or float(raw.get("duration") or 0) > 0:
                continue
            if self._fill_duration(sid, raw) > 0:
                filled += 1
        if filled:
            self._log("info", f"Measured {filled} sound(s) with no stored duration")
        return filled

    def reprobe(self, sound_id: str) -> Optional[Sound]:
        """Re-measure a clip whose file was replaced on disk.

        Saving alone will not do it: save keeps a duration it already
        has, so that a normal edit does not re-read the file every time.
        """
        snd = self.get(sound_id)
        if snd is None:
            return None
        snd.duration = probe_duration(snd.file_path)
        snd.stamp()
        with self._write_lock:
            self.store.write_raw(snd.id, snd.to_dict())
        return snd

    def delete(self, sound_id: str) -> bool:
        return self.store.delete(sound_id)

    def reorder(self, ordered_ids: List[str]) -> List[Dict]:
        """Rewrite ``position`` to match ``ordered_ids`` order.

        Positions are assigned 1..N in the given sequence; ids not present
        in the store are skipped. Returns the refreshed summary list.
        """
        for idx, sid in enumerate(ordered_ids, start=1):
            snd = self.get(sid)
            if snd is None:
                continue
            snd.position = idx
            snd.stamp()
            self.store.write_raw(snd.id, snd.to_dict())
        return self.list()

    def _log(self, level: str, msg: str) -> None:
        if self.logger:
            # rcutils tracks log calls by file/line; getattr-dispatch
            # funnels every severity through one line and trips rclpy's
            # severity-change check. See saint_server.log_level.log_at.
            from saint_server.log_level import log_at
            log_at(self.logger, level, msg)


class PlaylistStore:
    """JSON-backed playlists at ``{config_dir}/playlists/``.

    A playlist is an ordered, named set of board items of one kind — the
    replacement for the single ``group`` string that used to sit on each
    animation, pose and sound. Because membership is many-to-many now,
    it lives here rather than on the item; see
    :class:`~saint_server.animation.models.Playlist` for why order lives
    here too.

    The store is deliberately dumb about whether its member ids still
    resolve. Item deletion calls :meth:`forget_item` to keep the lists
    tidy, but a member that slipped through (a hand-edited file, a
    restored backup) is filtered out at read time instead of being an
    error — a dangling id should never keep a board from rendering.
    """

    def __init__(self, config_dir: str, logger=None):
        self.config_dir = config_dir
        self.store = _JSONStore(os.path.join(config_dir, "playlists"), logger=logger)
        self.logger = logger

    # ── read ────────────────────────────────────────────────────────

    def list(self, kind: Optional[str] = None) -> List[Dict]:
        """Summaries, sidebar-ordered: (position, name).

        ``kind`` filters to one board section. Each summary carries the
        full ordered ``items`` list — it is a handful of ids, the UI
        needs it to render the section, and fetching each playlist
        separately to get it would be a request per row.
        """
        out = []
        for pid in self.store.list_ids():
            raw = self.store.read_raw(pid)
            if not raw:
                continue
            try:
                pl = Playlist.from_dict(raw)
            except (KeyError, ValueError) as e:
                self._log("warning", f"Skipping unreadable playlist {pid}: {e}")
                continue
            if kind is not None and pl.kind != kind:
                continue
            out.append({
                "id": pl.id,
                "name": pl.name,
                "kind": pl.kind,
                "icon": pl.icon,
                "items": list(pl.items),
                "count": len(pl.items),
                "position": pl.position,
                "modified": pl.modified,
            })
        out.sort(key=lambda p: (p["position"], p["name"].lower()))
        return out

    def get(self, playlist_id: str) -> Optional[Playlist]:
        raw = self.store.read_raw(playlist_id)
        if not raw:
            return None
        try:
            return Playlist.from_dict(raw)
        except (KeyError, ValueError) as e:
            self._log("warning", f"Failed to parse playlist {playlist_id}: {e}")
            return None

    def all(self) -> List[Playlist]:
        out = []
        for pid in self.store.list_ids():
            pl = self.get(pid)
            if pl is not None:
                out.append(pl)
        return out

    def memberships(self, kind: str) -> Dict[str, List[str]]:
        """``{item_id: [playlist_id, ...]}`` for one kind.

        The reverse index the item lists need to answer "which playlists
        is this row in" without every row scanning every playlist.
        Built once per list() call by the caller.
        """
        out: Dict[str, List[str]] = {}
        for pl in sorted(self.all(), key=lambda p: (p.position, p.name.lower())):
            if pl.kind != kind:
                continue
            for item_id in pl.items:
                out.setdefault(item_id, []).append(pl.id)
        return out

    # ── write ───────────────────────────────────────────────────────

    def save(self, playlist: Playlist) -> Playlist:
        if playlist.kind not in PLAYLIST_KINDS:
            raise ValueError(f"Unknown playlist kind: {playlist.kind!r}")
        if not playlist.id:
            # Namespace the slug by kind so an "animations" and a
            # "sounds" playlist can both be called "Greetings" without
            # colliding on one file — per-kind sections mean operators
            # WILL reuse names across kinds.
            base = slugify(playlist.name) or "untitled"
            playlist.id = self._unique_id(f"{playlist.kind[:4]}_{base}")
        else:
            playlist.id = slugify(playlist.id)
        if playlist.position == 0 and self.store.read_raw(playlist.id) is None:
            peers = [p for p in self.list(playlist.kind)]
            playlist.position = max((p["position"] for p in peers), default=0) + 1
        playlist.stamp()
        self.store.write_raw(playlist.id, playlist.to_dict())
        return playlist

    def delete(self, playlist_id: str) -> bool:
        """Delete the playlist only. Its members are untouched —
        removing a playlist must never delete the animations in it."""
        return self.store.delete(playlist_id)

    def add_item(self, playlist_id: str, item_id: str,
                 index: Optional[int] = None) -> Optional[Playlist]:
        pl = self.get(playlist_id)
        if pl is None:
            return None
        if pl.add(item_id, index):
            pl.stamp()
            self.store.write_raw(pl.id, pl.to_dict())
        return pl

    def remove_item(self, playlist_id: str, item_id: str) -> Optional[Playlist]:
        pl = self.get(playlist_id)
        if pl is None:
            return None
        if pl.remove(item_id):
            pl.stamp()
            self.store.write_raw(pl.id, pl.to_dict())
        return pl

    def reorder_items(self, playlist_id: str,
                      ordered_ids: List[str]) -> Optional[Playlist]:
        pl = self.get(playlist_id)
        if pl is None:
            return None
        pl.reorder(ordered_ids)
        pl.stamp()
        self.store.write_raw(pl.id, pl.to_dict())
        return pl

    def reorder(self, kind: str, ordered_ids: List[str]) -> List[Dict]:
        """Rewrite sidebar ``position`` for one kind's playlists."""
        for idx, pid in enumerate(ordered_ids, start=1):
            pl = self.get(pid)
            if pl is None or pl.kind != kind:
                continue
            pl.position = idx
            pl.stamp()
            self.store.write_raw(pl.id, pl.to_dict())
        return self.list(kind)

    def forget_item(self, kind: str, item_id: str) -> int:
        """Drop ``item_id`` from every playlist of ``kind``.

        Called when the item itself is deleted, so a later item that
        happens to slug to the same id doesn't silently inherit the dead
        one's playlist memberships. Returns how many playlists changed.
        """
        changed = 0
        for pl in self.all():
            if pl.kind != kind:
                continue
            if pl.remove(item_id):
                pl.stamp()
                self.store.write_raw(pl.id, pl.to_dict())
                changed += 1
        return changed

    def _unique_id(self, base: str) -> str:
        """``base``, or ``base_2``/``base_3``… if that file exists."""
        candidate = slugify(base) or "untitled"
        if self.store.read_raw(candidate) is None:
            return candidate
        for n in range(2, 1000):
            nxt = slugify(f"{candidate}_{n}")
            if self.store.read_raw(nxt) is None:
                return nxt
        return candidate

    # ── one-time migration off the legacy `group` string ────────────

    def migrate_legacy_groups(self, legacy: Dict[str, List[Dict]]) -> int:
        """Build playlists from the old per-item ``group`` values.

        ``legacy`` is ``{kind: [summary, ...]}`` using each store's own
        ``list()`` shape — every summary needs ``id`` and ``group``, and
        sounds additionally carry ``position``.

        Runs at most once per install: the presence of the playlists
        directory is the marker. That is deliberately coarse — an
        operator who deletes every playlist has an empty directory, not
        a missing one, so we will not resurrect groups they threw away.

        Order within each new playlist follows what the old UI showed:
        sounds by their explicit ``position`` then name, animations and
        poses alphabetically. Returns the number of playlists created.
        """
        if os.path.isdir(self.store.base_dir):
            return 0
        created = 0
        for kind in PLAYLIST_KINDS:
            items = legacy.get(kind) or []
            buckets: Dict[str, List[Dict]] = {}
            for raw in items:
                group = str(raw.get("group") or "").strip()
                if not group:
                    continue          # ungrouped stays ungrouped
                buckets.setdefault(group, []).append(raw)
            for position, group in enumerate(sorted(buckets, key=str.lower), start=1):
                members = buckets[group]
                if kind == "sounds":
                    members.sort(key=lambda s: (s.get("position", 0),
                                                str(s.get("name", "")).lower()))
                else:
                    members.sort(key=lambda s: str(s.get("name", "")).lower())
                pl = Playlist(
                    id="",
                    name=group,
                    kind=kind,
                    items=[str(m["id"]) for m in members if m.get("id")],
                    position=position,
                )
                self.save(pl)
                created += 1
        # Even with nothing to migrate, leave the directory behind so
        # this never runs again on a library that is simply ungrouped.
        self.store._ensure_dir()
        if created:
            self._log("info",
                      f"Playlists: migrated {created} legacy group(s) "
                      f"into playlists")
        return created

    def _log(self, level: str, msg: str) -> None:
        if self.logger:
            # rcutils tracks log calls by file/line; getattr-dispatch
            # funnels every severity through one line and trips rclpy's
            # severity-change check. See saint_server.log_level.log_at.
            from saint_server.log_level import log_at
            log_at(self.logger, level, msg)
