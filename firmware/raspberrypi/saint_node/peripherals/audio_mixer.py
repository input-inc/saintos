"""
SAINT.OS Pi node — audio_mixer driver (pyalsaaudio backend).

Concrete `AudioMixerDriver` that drives an ALSA mixer control (Master /
PCM / Digital / Headphone / …) on the host sound card. Setting volume +
balance translates to per-channel L/R playback levels on the chosen
control; mute uses the control's playback switch when it has one, and
falls back to zeroing the level when it doesn't.

Trade-off: the runtime depends on the `pyalsaaudio` Python module
(`apt install python3-alsaaudio` or `pip install pyalsaaudio` on
Raspberry Pi OS; ALSA itself ships with the OS). The driver imports
`alsaaudio` lazily so a missing library only fails this driver, not the
whole node.
"""

from __future__ import annotations

from typing import Optional, Tuple

from .audio_mixer_base import AudioMixerDriver

try:
    import alsaaudio  # type: ignore
    _ALSA_AVAILABLE = True
    _ALSA_IMPORT_ERROR: Optional[BaseException] = None
except Exception as e:  # pragma: no cover - import error path
    alsaaudio = None  # type: ignore
    _ALSA_AVAILABLE = False
    _ALSA_IMPORT_ERROR = e


class AlsaMixerDriver(AudioMixerDriver):
    """pyalsaaudio–backed audio-output mixer. One `alsaaudio.Mixer` per
    instance, opened against the configured control + card index."""

    # ── Backend overrides ─────────────────────────────────────────

    def _backend_open(self, inst) -> bool:
        if not _ALSA_AVAILABLE:
            self._log("error",
                f"audio_mixer#{inst.instance_id}: pyalsaaudio not "
                f"available ({_ALSA_IMPORT_ERROR!r}); install "
                f"python3-alsaaudio on the host")
            return False

        control = str(inst.params.get("mixer_control") or "Master")
        cardindex = self._card_index(inst.params.get("card"))

        try:
            mixer = alsaaudio.Mixer(control=control, cardindex=cardindex)
        except Exception as e:
            self._log("error",
                f"audio_mixer#{inst.instance_id}: could not open mixer "
                f"control '{control}' on card {cardindex}: {e} — run "
                f"`amixer -c {cardindex} scontrols` to list controls")
            return False

        inst.params["_alsa_mixer"] = mixer
        # Cache how many volume channels this control exposes so we know
        # whether per-channel L/R balance is even meaningful (mono = 1).
        try:
            inst.params["_mx_channels"] = len(mixer.getvolume())
        except Exception:
            inst.params["_mx_channels"] = 1
        inst.params["_mx_vol"] = 0.0
        inst.params["_mx_bal"] = 0.0
        inst.params["_mx_muted"] = False
        return True

    def _backend_close(self, inst) -> None:
        mixer = inst.params.pop("_alsa_mixer", None)
        inst.params.pop("_mx_channels", None)
        inst.params.pop("_mx_vol", None)
        inst.params.pop("_mx_bal", None)
        inst.params.pop("_mx_muted", None)
        if mixer is not None:
            try:
                mixer.close()
            except Exception:
                pass

    def _backend_set_volume(self, inst, volume_0_1: float) -> None:
        inst.params["_mx_vol"] = float(volume_0_1)
        self._apply_levels(inst)

    def _backend_set_balance(self, inst, balance_m1_1: float) -> None:
        inst.params["_mx_bal"] = float(balance_m1_1)
        self._apply_levels(inst)

    def _backend_set_mute(self, inst, muted: bool) -> None:
        mixer = inst.params.get("_alsa_mixer")
        if mixer is None:
            return
        inst.params["_mx_muted"] = bool(muted)
        # Preferred path: the control has a playback switch.
        try:
            mixer.setmute(1 if muted else 0)
            return
        except Exception:
            pass
        # Fallback: no mute switch — zero the level while muted and let
        # _apply_levels restore the commanded volume on unmute.
        self._apply_levels(inst)

    def _backend_get_levels(self, inst) -> Tuple[float, float]:
        mixer = inst.params.get("_alsa_mixer")
        if mixer is None:
            return 0.0, 0.0
        try:
            vols = mixer.getvolume()
        except Exception:
            return 0.0, 0.0
        if not vols:
            return 0.0, 0.0
        left = max(0.0, min(1.0, vols[0] / 100.0))
        right = max(0.0, min(1.0, vols[1] / 100.0)) if len(vols) > 1 else left
        return left, right

    def _backend_get_mute(self, inst) -> bool:
        mixer = inst.params.get("_alsa_mixer")
        if mixer is None:
            return False
        try:
            return any(bool(m) for m in mixer.getmute())
        except Exception:
            # Control has no mute switch — report the logical state we
            # track for the level-zeroing fallback.
            return bool(inst.params.get("_mx_muted", False))

    # ── Helpers ───────────────────────────────────────────────────

    def _apply_levels(self, inst) -> None:
        """Push the current (volume, balance, mute) state to the mixer as
        per-channel L/R percentages."""
        mixer = inst.params.get("_alsa_mixer")
        if mixer is None:
            return
        vol = float(inst.params.get("_mx_vol", 0.0))
        bal = float(inst.params.get("_mx_bal", 0.0))
        # When muted with no hardware switch, drive the level to zero.
        if inst.params.get("_mx_muted") and not self._has_mute_switch(inst):
            vol = 0.0
        left, right = self.balance_to_gains(vol, bal)
        lpct = int(round(left * 100.0))
        rpct = int(round(right * 100.0))

        channels = int(inst.params.get("_mx_channels", 1) or 1)
        if channels >= 2:
            try:
                mixer.setvolume(lpct, 0)   # left channel
                mixer.setvolume(rpct, 1)   # right channel
                return
            except Exception as e:
                self._log("warn",
                    f"audio_mixer#{inst.instance_id}: per-channel setvolume "
                    f"failed ({e}); falling back to combined level")
        # Mono control (or per-channel write failed): balance is
        # meaningless, so apply the master volume to every channel.
        try:
            mixer.setvolume(int(round(vol * 100.0)))
        except Exception as e:
            self._log("error",
                f"audio_mixer#{inst.instance_id}: setvolume raised: {e}")

    def _has_mute_switch(self, inst) -> bool:
        mixer = inst.params.get("_alsa_mixer")
        if mixer is None:
            return False
        try:
            mixer.getmute()
            return True
        except Exception:
            return False

    @staticmethod
    def _card_index(card) -> int:
        """Coerce the `card` param to an ALSA card index. Accepts an int,
        a numeric string, or 'default'/'' (→ card 0)."""
        if isinstance(card, bool):
            return 0
        if isinstance(card, int):
            return card
        s = str(card or "").strip()
        if s.isdigit():
            return int(s)
        return 0
