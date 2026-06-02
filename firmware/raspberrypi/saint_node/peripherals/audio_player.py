"""
SAINT.OS Pi node — audio_player driver (python-vlc backend).

Concrete `AudioPlayerDriver` that plays audio files through libVLC
against the host's ALSA stack. Works for any output ALSA exposes —
HDMI, USB DAC, I²S HATs, the 3.5 mm jack on Pi 4 — by setting the
`alsa_device` peripheral param.

Why python-vlc and not pygame / aplay / mpv:

  - frame-accurate seek across MP3/FLAC/WAV/OGG (pygame's MP3 seek is
    unreliable; aplay can't seek mid-playback without restarting),
  - a single library handles every common container/codec the operator
    is likely to drop in (no per-format branching),
  - no need to manage an external subprocess + IPC socket the way
    mpv-IPC would require.

Trade-off: the runtime depends on libvlc being installed on the host
(`apt install vlc-bin libvlc-dev` on Raspberry Pi OS pulls in
everything needed). The driver imports `vlc` lazily so a missing
library only fails the driver, not the whole node.
"""

from __future__ import annotations

from typing import Any, Optional

from .audio_player_base import AudioPlayerDriver

try:
    import vlc  # type: ignore
    _VLC_AVAILABLE = True
    _VLC_IMPORT_ERROR: Optional[BaseException] = None
except Exception as e:  # pragma: no cover - import error path
    vlc = None  # type: ignore
    _VLC_AVAILABLE = False
    _VLC_IMPORT_ERROR = e


class PiAudioPlayerDriver(AudioPlayerDriver):
    """python-vlc–backed audio player. One VLC `MediaPlayer` per
    instance; volume is set on the player (0–100 in VLC units, mapped
    from the catalog's 0..1)."""

    def __init__(self, logger=None):
        super().__init__(logger=logger)
        # VLC's Instance is process-global-ish; one is enough for every
        # peripheral instance. Created lazily on first apply_config so
        # the import error path can short-circuit cleanly.
        self._vlc_instance = None

    # ── Backend overrides ─────────────────────────────────────────

    def _backend_open(self, inst) -> bool:
        if not _VLC_AVAILABLE:
            self._log("error",
                f"audio_player#{inst.instance_id}: python-vlc not "
                f"available ({_VLC_IMPORT_ERROR!r}); install vlc + "
                f"python3-vlc on the host")
            return False

        if self._vlc_instance is None:
            # `--aout=alsa` forces ALSA output; `--alsa-audio-device`
            # picks the PCM. `--no-video` keeps libvlc from trying to
            # initialize any video output on a headless Pi.
            alsa_device = str(inst.params.get("alsa_device") or "default")
            args = [
                "--no-video",
                "--quiet",
                "--aout=alsa",
                f"--alsa-audio-device={alsa_device}",
            ]
            try:
                self._vlc_instance = vlc.Instance(*args)
            except Exception as e:
                self._log("error",
                    f"audio_player#{inst.instance_id}: vlc.Instance failed: {e}")
                return False

        try:
            player = self._vlc_instance.media_player_new()
        except Exception as e:
            self._log("error",
                f"audio_player#{inst.instance_id}: media_player_new failed: {e}")
            return False

        # Stash the player on the instance via a dedicated attr — the
        # base PeripheralInstance dataclass doesn't have one, so we use
        # the params dict as a scratch space for runtime objects keyed
        # under a leading underscore.
        inst.params["_vlc_player"] = player
        inst.params["_vlc_media"] = None
        return True

    def _backend_close(self, inst) -> None:
        player = inst.params.pop("_vlc_player", None)
        inst.params.pop("_vlc_media", None)
        if player is not None:
            try:
                player.stop()
                player.release()
            except Exception:
                pass

    def _backend_play_file(self, inst, absolute_path: str) -> bool:
        player = inst.params.get("_vlc_player")
        if player is None or self._vlc_instance is None:
            return False
        try:
            media = self._vlc_instance.media_new(absolute_path)
        except Exception as e:
            self._log("error",
                f"audio_player#{inst.instance_id}: media_new('{absolute_path}') "
                f"failed: {e}")
            return False
        inst.params["_vlc_media"] = media
        try:
            player.set_media(media)
            rc = player.play()
        except Exception as e:
            self._log("error",
                f"audio_player#{inst.instance_id}: play raised: {e}")
            return False
        if rc == -1:
            self._log("warn",
                f"audio_player#{inst.instance_id}: play('{absolute_path}') "
                f"returned -1")
            return False
        return True

    def _backend_play(self, inst) -> None:
        player = inst.params.get("_vlc_player")
        if player is None:
            return
        try:
            player.play()
        except Exception as e:
            self._log("error",
                f"audio_player#{inst.instance_id}: play raised: {e}")

    def _backend_stop(self, inst) -> None:
        player = inst.params.get("_vlc_player")
        if player is None:
            return
        try:
            player.stop()
        except Exception as e:
            self._log("error",
                f"audio_player#{inst.instance_id}: stop raised: {e}")

    def _backend_toggle_pause(self, inst) -> None:
        player = inst.params.get("_vlc_player")
        if player is None:
            return
        try:
            player.pause()
        except Exception as e:
            self._log("error",
                f"audio_player#{inst.instance_id}: pause raised: {e}")

    def _backend_seek(self, inst, position_s: float) -> None:
        player = inst.params.get("_vlc_player")
        if player is None:
            return
        try:
            player.set_time(int(max(0.0, position_s) * 1000.0))
        except Exception as e:
            self._log("error",
                f"audio_player#{inst.instance_id}: seek raised: {e}")

    def _backend_set_volume(self, inst, volume_0_1: float) -> None:
        player = inst.params.get("_vlc_player")
        if player is None:
            return
        try:
            player.audio_set_volume(int(max(0.0, min(1.0, volume_0_1)) * 100))
        except Exception as e:
            self._log("error",
                f"audio_player#{inst.instance_id}: set_volume raised: {e}")

    def _backend_is_playing(self, inst) -> bool:
        player = inst.params.get("_vlc_player")
        if player is None:
            return False
        try:
            return bool(player.is_playing())
        except Exception:
            return False

    def _backend_position_s(self, inst) -> float:
        player = inst.params.get("_vlc_player")
        if player is None:
            return 0.0
        try:
            ms = int(player.get_time())
        except Exception:
            return 0.0
        if ms < 0:
            return 0.0
        return ms / 1000.0
