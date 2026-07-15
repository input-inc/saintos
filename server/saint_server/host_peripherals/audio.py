"""
In-process host_controller audio drivers.

The server-side twins of the Pi firmware ``audio_mixer`` / ``audio_player``
peripheral drivers, so the operator can control the SERVER host's system
volume (and play files) live from the web UI or the Steam Deck — the same
``set_channel`` / ``peripheral_command`` path a firmware node uses, but
routed in-process (see server_node.send_channel_command /
send_peripheral_command host branches → HostPeripheralManager).

Both bindings import lazily and degrade to a logged no-op when the native
library is missing, so a dev host without ALSA/VLC still starts cleanly.
"""

from __future__ import annotations

import os
from typing import Any, Callable, Dict, Optional

try:
    import alsaaudio  # type: ignore
    _ALSA_AVAILABLE = True
    _ALSA_ERR: Optional[BaseException] = None
except Exception as e:  # pragma: no cover - import error path
    alsaaudio = None  # type: ignore
    _ALSA_AVAILABLE = False
    _ALSA_ERR = e


StateCallback = Callable[[str, str, float], None]


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class HostAudioMixerDriver:
    """ALSA master volume / balance / mute for the server host, via
    pyalsaaudio. Ports firmware AlsaMixerDriver. Channel writes:
    ``volume`` (0–1), ``balance`` (−1…+1), ``mute`` (≥0.5 → muted).
    Emits ``left``/``right``/``muted`` read-back telemetry via state_cb."""

    def __init__(self, peripheral_id: str, params: Dict[str, Any],
                 state_cb: StateCallback,
                 log_cb: Optional[Callable[[str, str], None]] = None):
        self._pid = peripheral_id
        self._params = dict(params or {})
        self._state_cb = state_cb
        self._log_cb = log_cb
        self._mixer = None
        self._channels = 1
        self._vol = float(self._params.get("default_volume", 0.8))
        self._bal = float(self._params.get("default_balance", 0.0))
        self._muted = bool(self._params.get("default_mute", False))

    @property
    def is_connected(self) -> bool:
        return self._mixer is not None

    def start(self) -> None:
        if not _ALSA_AVAILABLE:
            self._log("error",
                f"{self._pid}: python3-alsaaudio not available "
                f"({_ALSA_ERR!r}); install it on the host")
            return
        control = str(self._params.get("mixer_control") or "Master")
        card = self._card_index(self._params.get("card"))
        try:
            self._mixer = alsaaudio.Mixer(control=control, cardindex=card)
        except Exception as e:
            self._log("error",
                f"{self._pid}: open mixer '{control}' on card {card} "
                f"failed: {e} — try `amixer -c {card} scontrols`")
            self._mixer = None
            return
        try:
            self._channels = len(self._mixer.getvolume())
        except Exception:
            self._channels = 1
        self._apply_levels()
        if self._muted:
            self._set_mute(True)
        self._emit_telemetry()

    def stop(self) -> None:
        m, self._mixer = self._mixer, None
        if m is not None:
            try:
                m.close()
            except Exception:
                pass

    def set_channel(self, channel_id: str, value: float) -> None:
        if channel_id == "volume":
            self._vol = _clamp(float(value), 0.0, 1.0)
            self._apply_levels()
        elif channel_id == "balance":
            self._bal = _clamp(float(value), -1.0, 1.0)
            self._apply_levels()
        elif channel_id == "mute":
            self._set_mute(float(value) >= 0.5)
        else:
            return  # read-only (left/right/muted) or unknown
        self._emit_telemetry()

    def handle_command(self, command: str, args: Dict[str, Any]) -> None:
        if command == "mute":
            # Trigger command toggles mute.
            self._set_mute(not self._muted)
            self._emit_telemetry()

    # ── ALSA plumbing (ported from firmware AlsaMixerDriver) ────────

    def _set_mute(self, muted: bool) -> None:
        self._muted = bool(muted)
        m = self._mixer
        if m is None:
            return
        try:
            m.setmute(1 if muted else 0)
            return
        except Exception:
            # No hardware mute switch — zero the level while muted;
            # _apply_levels restores the commanded volume on unmute.
            self._apply_levels()

    def _has_mute_switch(self) -> bool:
        m = self._mixer
        if m is None:
            return False
        try:
            m.getmute()
            return True
        except Exception:
            return False

    def _apply_levels(self) -> None:
        m = self._mixer
        if m is None:
            return
        vol = self._vol
        if self._muted and not self._has_mute_switch():
            vol = 0.0
        left, right = self._balance_to_gains(vol, self._bal)
        if self._channels >= 2:
            try:
                m.setvolume(int(round(left * 100.0)), 0)
                m.setvolume(int(round(right * 100.0)), 1)
                return
            except Exception as e:
                self._log("warn",
                    f"{self._pid}: per-channel setvolume failed ({e}); "
                    f"falling back to combined level")
        try:
            m.setvolume(int(round(vol * 100.0)))
        except Exception as e:
            self._log("error", f"{self._pid}: setvolume raised: {e}")

    def _emit_telemetry(self) -> None:
        m = self._mixer
        left = right = 0.0
        muted = self._muted
        if m is not None:
            try:
                vols = m.getvolume()
                if vols:
                    left = _clamp(vols[0] / 100.0, 0.0, 1.0)
                    right = _clamp(vols[1] / 100.0, 0.0, 1.0) if len(vols) > 1 else left
            except Exception:
                pass
            try:
                muted = any(bool(x) for x in m.getmute())
            except Exception:
                muted = self._muted
        self._emit("left", left)
        self._emit("right", right)
        self._emit("muted", 1.0 if muted else 0.0)

    def _emit(self, channel: str, value: float) -> None:
        try:
            self._state_cb(self._pid, channel, float(value))
        except Exception:
            pass

    @staticmethod
    def _balance_to_gains(vol: float, bal: float):
        vol = _clamp(vol, 0.0, 1.0)
        bal = _clamp(bal, -1.0, 1.0)
        return vol * (1.0 - max(0.0, bal)), vol * (1.0 - max(0.0, -bal))

    @staticmethod
    def _card_index(card) -> int:
        if card is None:
            return 0
        try:
            return int(card)
        except (TypeError, ValueError):
            return 0

    def _log(self, level: str, msg: str) -> None:
        if self._log_cb:
            try:
                self._log_cb(level, msg)
            except Exception:
                pass


class HostAudioPlayerDriver:
    """On-host file playback via the shared VLC SoundboardPlayer. Channel
    writes: ``volume`` (0–1), ``stop`` (trigger). ``play_file`` command
    plays a file resolved within ``library_path``. Emits ``is_playing``."""

    def __init__(self, peripheral_id: str, params: Dict[str, Any],
                 state_cb: StateCallback,
                 log_cb: Optional[Callable[[str, str], None]] = None):
        from . import soundboard as _sb
        self._pid = peripheral_id
        self._params = dict(params or {})
        self._state_cb = state_cb
        self._log_cb = log_cb
        self._player = _sb.SoundboardPlayer(logger=None)
        self._device = str(self._params.get("alsa_device", "default") or "default")
        self._library = str(self._params.get("library_path", "/var/lib/saint-os/audio"))
        self._volume = float(self._params.get("initial_volume", 0.8))

    @property
    def is_connected(self) -> bool:
        return self._player.available()

    def start(self) -> None:
        # Nothing to open until the first play.
        return

    def stop(self) -> None:
        self._player.stop()

    def set_channel(self, channel_id: str, value: float) -> None:
        if channel_id == "volume":
            self._volume = _clamp(float(value), 0.0, 1.0)
            self._player.set_volume(self._volume)
        elif channel_id == "stop" and float(value) >= 0.5:
            self._player.stop()
            self._emit("is_playing", 0.0)
        # play/pause/seek triggers need a loaded file — the play_file
        # command is the entry point for host playback.

    def handle_command(self, command: str, args: Dict[str, Any]) -> None:
        if command != "play_file":
            return
        filename = str(args.get("filename", "") or args.get("path", ""))
        path = self._resolve(filename)
        if path is None:
            self._log("warn", f"{self._pid}: file not in library: {filename!r}")
            self._emit("is_playing", 0.0)
            return
        ok, msg = self._player.play(
            path, device=self._device, volume=self._volume,
            loop=bool(args.get("loop", False)),
            loop_count=int(args.get("loop_count", 0)))
        self._emit("is_playing", 1.0 if ok else 0.0)
        if not ok:
            self._log("warn", f"{self._pid}: play failed: {msg}")

    def _resolve(self, filename: str) -> Optional[str]:
        """Resolve a filename within library_path (prevents ../ escape)."""
        if not filename:
            return None
        base = os.path.abspath(self._library)
        cand = filename if os.path.isabs(filename) else os.path.join(base, filename)
        cand = os.path.abspath(cand)
        if not (cand == base or cand.startswith(base + os.sep)):
            return None
        return cand if os.path.isfile(cand) else None

    def _emit(self, channel: str, value: float) -> None:
        try:
            self._state_cb(self._pid, channel, float(value))
        except Exception:
            pass

    def _log(self, level: str, msg: str) -> None:
        if self._log_cb:
            try:
                self._log_cb(level, msg)
            except Exception:
                pass


# Type id → driver class, for the manager's audio reconcile.
AUDIO_DRIVERS = {
    "audio_mixer": HostAudioMixerDriver,
    "audio_player": HostAudioPlayerDriver,
}
