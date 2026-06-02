"""
SAINT.OS Pi node — audio-player peripheral driver base.

Generic, backend-agnostic shell for the `audio_player` peripheral type.
The catalog declares one peripheral; concrete subclasses bind it to a
real audio backend (today: python-vlc on the Pi's ALSA stack;
tomorrow potentially a UART MP3-trigger module on an RP2040).

Subclasses implement the small `_backend_*` surface and the rest —
channel ↔ sub-channel mapping, trigger-edge detection, telemetry
publish — comes for free.
"""

from __future__ import annotations

import abc
import os
import time
from typing import Any, Dict, Optional

from .base import PeripheralDriver

# Virtual GPIO layout. The Pi-side router won't actually deliver values
# through vGPIO here (the peripheral is pinless and the server uses
# `set_channel` by id), but the base class contract requires a unique
# range so the manager's overlap-detection stays meaningful.
AUDIO_PLAYER_VIRTUAL_GPIO_BASE = 400
AUDIO_PLAYER_CHANNEL_COUNT     = 7

# Sub-channel indexes. Order must match SUB_CHANNEL_NAMES + the
# `channels` list on the server-side catalog entry.
AP_CH_PLAY       = 0
AP_CH_STOP       = 1
AP_CH_PAUSE      = 2
AP_CH_SEEK       = 3
AP_CH_VOLUME     = 4
AP_CH_IS_PLAYING = 5
AP_CH_POSITION   = 6


class AudioPlayerDriver(PeripheralDriver):
    """Base class for audio-player peripheral drivers.

    Subclasses override the `_backend_*` methods to bind to a real
    audio stack. This class handles:

      - parsing pins/params on apply_config,
      - mapping numeric channel writes to play / stop / pause / seek
        / volume calls,
      - rising-edge detection for the trigger channels (play / stop /
        pause): a value transitioning 0 → nonzero fires the action; a
        value sticking nonzero does NOT re-fire each tick,
      - the out-of-band `play_file` command (filename arg),
      - publishing telemetry (is_playing, position) via last_values.
    """

    TYPE_ID = "audio_player"
    MODE_STRING = "audio_player"
    VIRTUAL_GPIO_BASE = AUDIO_PLAYER_VIRTUAL_GPIO_BASE
    CHANNELS_PER_INSTANCE = AUDIO_PLAYER_CHANNEL_COUNT
    MAX_INSTANCES = 1
    SUB_CHANNEL_NAMES = [
        "play", "stop", "pause", "seek", "volume",
        "is_playing", "position",
    ]

    # ── PeripheralDriver overrides ────────────────────────────────

    def apply_config(self, instance_id: int, pins: Dict[str, int],
                     params: Dict[str, Any]) -> bool:
        inst = self._get_or_create_instance(instance_id)
        inst.pins = dict(pins or {})
        inst.params = dict(params or {})

        library_path = str(inst.params.get("library_path") or "").strip()
        if library_path and not os.path.isdir(library_path):
            # Don't fail the driver — operator may not have populated
            # the folder yet. Just warn so it's visible in logs.
            self._log("warn",
                f"audio_player#{instance_id}: library_path '{library_path}' "
                f"does not exist; play_file calls will fail until created")

        try:
            initial_volume = float(inst.params.get("initial_volume", 0.8))
        except (TypeError, ValueError):
            initial_volume = 0.8
        initial_volume = max(0.0, min(1.0, initial_volume))

        # Reset edge-detection latches.
        self._trigger_state(inst, AP_CH_PLAY,  0.0, fire=False)
        self._trigger_state(inst, AP_CH_STOP,  0.0, fire=False)
        self._trigger_state(inst, AP_CH_PAUSE, 0.0, fire=False)

        if not self._backend_open(inst):
            return False
        self._backend_set_volume(inst, initial_volume)
        inst.last_values[AP_CH_VOLUME] = initial_volume
        inst.last_values[AP_CH_IS_PLAYING] = 0.0
        inst.last_values[AP_CH_POSITION] = 0.0
        inst.connected = True
        return True

    def set_value(self, instance_id: int, sub_channel: int,
                  value: float) -> bool:
        inst = self._instances.get(instance_id)
        if inst is None:
            return False
        v = float(value)

        if sub_channel == AP_CH_PLAY:
            if self._trigger_state(inst, AP_CH_PLAY, v):
                self._backend_play(inst)
            return True
        if sub_channel == AP_CH_STOP:
            if self._trigger_state(inst, AP_CH_STOP, v):
                self._backend_stop(inst)
            return True
        if sub_channel == AP_CH_PAUSE:
            if self._trigger_state(inst, AP_CH_PAUSE, v):
                self._backend_toggle_pause(inst)
            return True
        if sub_channel == AP_CH_SEEK:
            self._backend_seek(inst, max(0.0, v))
            return True
        if sub_channel == AP_CH_VOLUME:
            vol = max(0.0, min(1.0, v))
            self._backend_set_volume(inst, vol)
            inst.last_values[AP_CH_VOLUME] = vol
            return True
        # is_playing / position are read-only.
        return False

    def get_value(self, instance_id: int, sub_channel: int) -> Optional[float]:
        inst = self._instances.get(instance_id)
        if inst is None:
            return None
        if sub_channel == AP_CH_IS_PLAYING:
            return 1.0 if self._backend_is_playing(inst) else 0.0
        if sub_channel == AP_CH_POSITION:
            return float(self._backend_position_s(inst))
        return inst.last_values.get(sub_channel)

    def update(self) -> None:
        for inst in self._instances.values():
            try:
                inst.last_values[AP_CH_IS_PLAYING] = (
                    1.0 if self._backend_is_playing(inst) else 0.0)
                inst.last_values[AP_CH_POSITION] = float(
                    self._backend_position_s(inst))
            except Exception as e:
                self._log("error",
                    f"audio_player#{inst.instance_id}: update raised: {e}")

    def estop(self) -> None:
        for inst in self._instances.values():
            try:
                self._backend_stop(inst)
            except Exception as e:
                self._log("error",
                    f"audio_player#{inst.instance_id}: estop stop raised: {e}")

    def reset(self) -> None:
        for inst in self._instances.values():
            try:
                self._backend_close(inst)
            except Exception:
                pass
        super().reset()

    def handle_command(self, instance_id: int, command: str,
                       args: Dict[str, Any]) -> bool:
        inst = self._instances.get(instance_id)
        if inst is None:
            return super().handle_command(instance_id, command, args)

        if command == "play_file":
            filename = str((args or {}).get("filename") or "").strip()
            if not filename:
                self._log("warn",
                    f"audio_player#{instance_id}: play_file missing filename")
                return False
            path = self._resolve_file(inst, filename)
            if path is None:
                return False
            return bool(self._backend_play_file(inst, path))
        if command == "stop":
            self._backend_stop(inst)
            return True
        if command == "pause":
            self._backend_toggle_pause(inst)
            return True
        return super().handle_command(instance_id, command, args)

    # ── Helpers ───────────────────────────────────────────────────

    def _resolve_file(self, inst, filename: str) -> Optional[str]:
        """Resolve `filename` against the configured library_path.

        Accepts either a relative path within the library folder or an
        absolute path that lies inside it. Refuses paths that escape the
        library (../) so a routing typo can't read /etc/shadow."""
        library = str(inst.params.get("library_path") or "").strip()
        if not library:
            self._log("warn",
                f"audio_player#{inst.instance_id}: no library_path configured")
            return None
        library_abs = os.path.abspath(library)

        if os.path.isabs(filename):
            candidate = os.path.abspath(filename)
        else:
            candidate = os.path.abspath(os.path.join(library_abs, filename))

        # Containment check — candidate must be inside library_abs.
        rel = os.path.relpath(candidate, library_abs)
        if rel.startswith("..") or os.path.isabs(rel):
            self._log("warn",
                f"audio_player#{inst.instance_id}: filename '{filename}' "
                f"resolves outside library_path '{library_abs}'")
            return None
        if not os.path.isfile(candidate):
            self._log("warn",
                f"audio_player#{inst.instance_id}: file not found: {candidate}")
            return None
        return candidate

    def _trigger_state(self, inst, sub_channel: int, value: float,
                       fire: bool = True) -> bool:
        """Update the latched value for a trigger channel and report
        whether this call corresponds to a rising edge (0 → nonzero).
        When `fire=False`, just store the value without claiming an
        edge — used during apply_config to seed the latch."""
        last = inst.last_values.get(sub_channel, 0.0)
        inst.last_values[sub_channel] = value
        if not fire:
            return False
        return abs(last) < 1e-6 and abs(value) > 1e-6

    # ── Backend surface — subclasses implement ────────────────────

    @abc.abstractmethod
    def _backend_open(self, inst) -> bool:
        """Construct any media player / DSP objects this backend needs.
        Returns False on hard failure (missing lib, device open error)."""

    @abc.abstractmethod
    def _backend_close(self, inst) -> None:
        """Release the backend resources on reset / shutdown."""

    @abc.abstractmethod
    def _backend_play_file(self, inst, absolute_path: str) -> bool:
        """Load and start playing the file at `absolute_path`. Returns
        False if the backend rejected the file."""

    @abc.abstractmethod
    def _backend_play(self, inst) -> None:
        """Resume the currently-loaded file (or no-op if none loaded)."""

    @abc.abstractmethod
    def _backend_stop(self, inst) -> None:
        """Stop playback. Idempotent."""

    @abc.abstractmethod
    def _backend_toggle_pause(self, inst) -> None:
        """Flip pause state for the currently-playing file."""

    @abc.abstractmethod
    def _backend_seek(self, inst, position_s: float) -> None:
        """Move the playhead to `position_s` seconds."""

    @abc.abstractmethod
    def _backend_set_volume(self, inst, volume_0_1: float) -> None:
        """Set output volume in 0..1."""

    @abc.abstractmethod
    def _backend_is_playing(self, inst) -> bool:
        """True iff the backend is actively producing audio right now."""

    @abc.abstractmethod
    def _backend_position_s(self, inst) -> float:
        """Current playhead position in seconds (0 if not playing)."""
