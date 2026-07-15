"""
Host-controller soundboard playback + filesystem/audio introspection.

The in-process (server-side) twin of
``firmware/raspberrypi/saint_node/soundboard.py``. When a soundboard entry
targets the synthetic ``host_controller`` node, playback happens *here* in
the server process — VLC → ALSA on the server's own host — rather than
being published to a firmware node over ROS. The three round-trips
(``list_directory``, ``list_audio_devices``, and ``SoundboardPlayer.play``)
mirror the firmware module exactly so the web/controller clients and the
``soundboard_fs|soundboard_devices|soundboard_result`` reply shapes are
identical for host and node targets.

The server and Pi firmware are separate Python installs (``saint_common``
is empty), so this is a deliberate copy of the firmware logic rather than
a shared import. ``vlc`` is imported lazily so a server host without
libVLC degrades to a clean error status instead of failing to start.
"""

from __future__ import annotations

import os
import subprocess
from typing import Any, Dict, List, Optional, Tuple

try:
    import vlc  # type: ignore
    _VLC_AVAILABLE = True
    _VLC_IMPORT_ERROR: Optional[BaseException] = None
except Exception as e:  # pragma: no cover - import error path
    vlc = None  # type: ignore
    _VLC_AVAILABLE = False
    _VLC_IMPORT_ERROR = e


# Extensions surfaced as "audio" to the file picker. The picker still
# lists everything; this just lets the UI grey-out non-audio files.
AUDIO_EXTENSIONS = (".wav", ".mp3", ".ogg", ".flac", ".aac", ".m4a", ".opus")

# VLC's ``input-repeat`` counts *additional* plays after the first. This
# sentinel stands in for "loop forever" (VLC treats it as effectively
# unbounded).
_INFINITE_REPEAT = 65535


def list_directory(path: str) -> Dict[str, Any]:
    """List one directory on the server host filesystem.

    Returns ``{status, path, parent, entries:[{name, is_dir, size,
    is_audio}]}``. Never raises — permission/other errors come back as
    ``{status: "error", message}`` so the picker can surface them.
    """
    abs_path = os.path.abspath(path or "/")
    if not os.path.isdir(abs_path):
        return {"status": "error", "message": f"Not a directory: {abs_path}",
                "path": abs_path}
    parent = os.path.dirname(abs_path.rstrip("/")) or "/"
    entries: List[Dict[str, Any]] = []
    try:
        names = sorted(os.listdir(abs_path))
    except Exception as e:
        return {"status": "error", "message": str(e), "path": abs_path}
    for name in names:
        full = os.path.join(abs_path, name)
        try:
            is_dir = os.path.isdir(full)
            size = 0 if is_dir else os.path.getsize(full)
        except OSError:
            # Broken symlink / vanished mid-listing — skip rather than
            # abort the whole listing.
            continue
        entries.append({
            "name": name,
            "is_dir": is_dir,
            "size": size,
            "is_audio": (not is_dir
                         and name.lower().endswith(AUDIO_EXTENSIONS)),
        })
    entries.sort(key=lambda e: (not e["is_dir"], e["name"].lower()))
    return {"status": "ok", "path": abs_path, "parent": parent,
            "entries": entries}


def list_audio_devices() -> Dict[str, Any]:
    """Enumerate ALSA playback devices as ``{id, name, is_default}``.

    ``id`` is a string suitable for VLC's ``--alsa-audio-device`` (e.g.
    ``"default"`` or ``"hw:0,0"``). Always includes a ``"default"`` entry.
    """
    devices: List[Dict[str, Any]] = [
        {"id": "default", "name": "Host default output", "is_default": True},
    ]
    try:
        out = subprocess.run(
            ["aplay", "-l"], capture_output=True, text=True, timeout=5.0)
        for line in out.stdout.splitlines():
            line = line.strip()
            if not line.startswith("card "):
                continue
            try:
                head, tail = line.split(",", 1)
                card_num = int(head.split(":", 1)[0].replace("card", "").strip())
                dev_num = int(tail.split(":", 1)[0].replace("device", "").strip())
            except (ValueError, IndexError):
                continue
            card_label = head.split("[", 1)[-1].rstrip("]").strip() if "[" in head else head
            dev_label = tail.split("[", 1)[-1].rstrip("]").strip() if "[" in tail else tail
            devices.append({
                "id": f"hw:{card_num},{dev_num}",
                "name": f"{card_label} — {dev_label}".strip(" —"),
                "is_default": False,
            })
    except Exception as e:
        return {"status": "ok", "devices": devices, "warning": str(e)}
    return {"status": "ok", "devices": devices}


class SoundboardPlayer:
    """Single-voice python-vlc player for host soundboard entries.

    Stop-and-replace: a new ``play()`` stops whatever is already sounding.
    A ``vlc.Instance`` is cached per ALSA device (the output device is an
    Instance-level argument); a fresh ``MediaPlayer`` is created per play so
    per-entry media options (start-time, loop) apply cleanly. Also reused
    by the host ``audio_player`` peripheral driver (see manager.py).
    """

    def __init__(self, logger=None):
        self._logger = logger
        self._instances: Dict[str, Any] = {}  # device id → vlc.Instance
        self._player = None

    def available(self) -> bool:
        return _VLC_AVAILABLE

    def _instance_for(self, device: str):
        device = device or "default"
        inst = self._instances.get(device)
        if inst is None:
            args = [
                "--no-video",
                "--quiet",
                "--aout=alsa",
                f"--alsa-audio-device={device}",
            ]
            inst = vlc.Instance(*args)
            self._instances[device] = inst
        return inst

    def play(self, path: str, device: str = "default", volume: float = 1.0,
             start_time_s: float = 0.0, loop: bool = False,
             loop_count: int = 0) -> Tuple[bool, str]:
        """Play ``path`` on ``device``. Returns ``(ok, message)``."""
        if not _VLC_AVAILABLE:
            return False, f"python-vlc not available: {_VLC_IMPORT_ERROR!r}"
        if not path or not os.path.isfile(path):
            return False, f"File not found: {path}"
        try:
            self.stop()
            inst = self._instance_for(device)
            media = inst.media_new(path)
            if start_time_s and start_time_s > 0:
                media.add_option(f"start-time={float(start_time_s)}")
            if loop:
                repeats = _INFINITE_REPEAT if loop_count <= 0 else max(0, loop_count - 1)
                media.add_option(f"input-repeat={repeats}")
            player = inst.media_player_new()
            player.set_media(media)
            player.audio_set_volume(int(max(0.0, min(1.0, volume)) * 100))
            rc = player.play()
            if rc == -1:
                return False, f"VLC refused to play {path}"
            self._player = player
            return True, "ok"
        except Exception as e:
            self._log("error", f"host soundboard play failed: {e}")
            return False, str(e)

    def set_volume(self, volume: float) -> None:
        """Adjust volume of the currently-playing clip (0.0–1.0)."""
        player = self._player
        if player is None:
            return
        try:
            player.audio_set_volume(int(max(0.0, min(1.0, volume)) * 100))
        except Exception:
            pass

    def stop(self) -> None:
        player, self._player = self._player, None
        if player is not None:
            try:
                player.stop()
                player.release()
            except Exception:
                pass

    def _log(self, level: str, msg: str) -> None:
        if self._logger is not None:
            getattr(self._logger, level, getattr(self._logger, "info", print))(msg)
