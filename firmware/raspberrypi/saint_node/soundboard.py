"""
SAINT.OS Pi node — soundboard playback + host introspection.

Node-level (not a peripheral) support for the server-managed Soundboard.
A soundboard entry names an audio file that lives on *this* node's own
storage and is played out one of *this* node's ALSA devices. The server
orchestrates three round-trips against a node (all correlated by
``request_id``, mirroring the BLE-scan pattern in ``node.py``):

  * ``list_directory(path)``      — browse the node filesystem when the
                                    operator picks a file,
  * ``list_audio_devices()``      — enumerate ALSA output devices for the
                                    per-sound output selector,
  * ``SoundboardPlayer.play(...)``— play a file with volume / start-time /
                                    loop options on a chosen device.

Playback reuses the same python-vlc → ALSA approach as the
``audio_player`` peripheral (see ``peripherals/audio_player.py``), but is
deliberately independent: one voice per node (stop-and-replace), no
library-path containment (operators browse the full filesystem by design),
and the ALSA output device is chosen per play.
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
    """List one directory on the node filesystem.

    Returns ``{status, path, parent, entries:[{name, is_dir, size,
    is_audio}]}``. Never raises — permission/te errors come back as
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
            # Broken symlink / vanished mid-listing — skip it rather than
            # abort the whole listing.
            continue
        entries.append({
            "name": name,
            "is_dir": is_dir,
            "size": size,
            "is_audio": (not is_dir
                         and name.lower().endswith(AUDIO_EXTENSIONS)),
        })
    # Directories first, then files; each alphabetical.
    entries.sort(key=lambda e: (not e["is_dir"], e["name"].lower()))
    return {"status": "ok", "path": abs_path, "parent": parent,
            "entries": entries}


def list_audio_devices() -> Dict[str, Any]:
    """Enumerate ALSA playback devices as ``{id, name, is_default}``.

    ``id`` is a string suitable for VLC's ``--alsa-audio-device`` (e.g.
    ``"default"`` or ``"hw:0,0"``). Parses ``aplay -l`` (the most reliable
    source of *output* devices); always includes a ``"default"`` entry so
    the operator can defer to the node's own default.
    """
    devices: List[Dict[str, Any]] = [
        {"id": "default", "name": "Node default output", "is_default": True},
    ]
    try:
        out = subprocess.run(
            ["aplay", "-l"], capture_output=True, text=True, timeout=5.0)
        for line in out.stdout.splitlines():
            # "card 0: b1 [bcm2835 HDMI 1], device 0: bcm2835 HDMI [..]"
            line = line.strip()
            if not line.startswith("card "):
                continue
            try:
                head, tail = line.split(",", 1)
                card_num = int(head.split(":", 1)[0].replace("card", "").strip())
                dev_num = int(tail.split(":", 1)[0].replace("device", "").strip())
            except (ValueError, IndexError):
                continue
            # Human label from the bracketed card + device names.
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
    """Single-voice python-vlc player for soundboard entries.

    Stop-and-replace: a new ``play()`` stops whatever is already sounding.
    A ``vlc.Instance`` is cached per ALSA device (the output device is an
    Instance-level argument), and a fresh ``MediaPlayer`` is created per
    play so per-entry media options (start-time, loop) apply cleanly.
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
                # Seek at load time — reliable across codecs, unlike a
                # post-play set_time() before the media is parsed.
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
            self._log("error", f"soundboard play failed: {e}")
            return False, str(e)

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
            getattr(self._logger, level, self._logger.info)(msg)
