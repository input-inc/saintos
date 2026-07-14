"""
SAINT.OS Pi node — audio-mixer peripheral driver base.

Generic, backend-agnostic shell for the `audio_mixer` peripheral type.
Where `audio_player` controls one media player's *application* volume,
this peripheral controls the host's audio *output* — the system mixer's
master level and the left/right balance. It affects everything the card
emits (the player, system sounds, anything on that ALSA card).

The catalog declares one peripheral; concrete subclasses bind it to a
real mixer backend (today: pyalsaaudio against the Pi's ALSA stack;
tomorrow potentially PulseAudio/PipeWire via pactl/wpctl).

Subclasses implement the small `_backend_*` surface and the rest —
channel ↔ sub-channel mapping, volume/balance clamping, default-on-boot,
telemetry publish — comes for free.
"""

from __future__ import annotations

import abc
from typing import Any, Dict, Optional, Tuple

from .base import PeripheralDriver

# Virtual GPIO layout. The Pi-side router won't actually deliver values
# through vGPIO here (the peripheral is pinless and the server uses
# `set_channel` by id), but the base-class contract requires a unique
# range so the manager's overlap-detection stays meaningful. 410 sits in
# the gap between audio_player (400–406) and console_display (500).
AUDIO_MIXER_VIRTUAL_GPIO_BASE = 410
AUDIO_MIXER_CHANNEL_COUNT     = 6

# Sub-channel indexes. Order must match SUB_CHANNEL_NAMES + the
# `channels` list on the server-side catalog entry. The first three are
# operator-writable controls; the last three are read-back telemetry.
MX_CH_VOLUME  = 0   # out: master level 0..1
MX_CH_BALANCE = 1   # out: -1 full-left .. 0 center .. +1 full-right
MX_CH_MUTE    = 2   # out: 0/1
MX_CH_LEFT    = 3   # in:  read-back left channel level 0..1
MX_CH_RIGHT   = 4   # in:  read-back right channel level 0..1
MX_CH_MUTED   = 5   # in:  read-back mute state 0/1


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class AudioMixerDriver(PeripheralDriver):
    """Base class for audio-output mixer drivers.

    Subclasses override the `_backend_*` methods to bind to a real mixer.
    This class handles:

      - parsing pins/params on apply_config and applying the configured
        defaults (volume / balance / mute) on boot,
      - mapping numeric channel writes to set-volume / set-balance /
        set-mute backend calls,
      - publishing read-back telemetry (left, right, muted) via
        last_values, polled in update(),
      - muting on estop and restoring on clear_estop.
    """

    TYPE_ID = "audio_mixer"
    MODE_STRING = "audio_mixer"
    VIRTUAL_GPIO_BASE = AUDIO_MIXER_VIRTUAL_GPIO_BASE
    CHANNELS_PER_INSTANCE = AUDIO_MIXER_CHANNEL_COUNT
    MAX_INSTANCES = 1
    SUB_CHANNEL_NAMES = [
        "volume", "balance", "mute",
        "left", "right", "muted",
    ]

    # ── PeripheralDriver overrides ────────────────────────────────

    def apply_config(self, instance_id: int, pins: Dict[str, int],
                     params: Dict[str, Any]) -> bool:
        inst = self._get_or_create_instance(instance_id)
        inst.pins = dict(pins or {})
        inst.params = dict(params or {})

        default_volume = _clamp(self._as_float(
            inst.params.get("default_volume"), 0.8), 0.0, 1.0)
        default_balance = _clamp(self._as_float(
            inst.params.get("default_balance"), 0.0), -1.0, 1.0)
        default_mute = bool(inst.params.get("default_mute", False))

        if not self._backend_open(inst):
            return False

        # Apply the configured defaults on boot so the output starts at a
        # known level regardless of what the card powered up at.
        self._backend_set_volume(inst, default_volume)
        self._backend_set_balance(inst, default_balance)
        self._backend_set_mute(inst, default_mute)

        inst.last_values[MX_CH_VOLUME] = default_volume
        inst.last_values[MX_CH_BALANCE] = default_balance
        inst.last_values[MX_CH_MUTE] = 1.0 if default_mute else 0.0
        self._poll_telemetry(inst)
        inst.connected = True
        return True

    def set_value(self, instance_id: int, sub_channel: int,
                  value: float) -> bool:
        inst = self._instances.get(instance_id)
        if inst is None:
            return False
        v = float(value)

        if sub_channel == MX_CH_VOLUME:
            vol = _clamp(v, 0.0, 1.0)
            self._backend_set_volume(inst, vol)
            inst.last_values[MX_CH_VOLUME] = vol
            return True
        if sub_channel == MX_CH_BALANCE:
            bal = _clamp(v, -1.0, 1.0)
            self._backend_set_balance(inst, bal)
            inst.last_values[MX_CH_BALANCE] = bal
            return True
        if sub_channel == MX_CH_MUTE:
            muted = v >= 0.5
            self._backend_set_mute(inst, muted)
            inst.last_values[MX_CH_MUTE] = 1.0 if muted else 0.0
            return True
        # left / right / muted are read-only telemetry.
        return False

    def get_value(self, instance_id: int, sub_channel: int) -> Optional[float]:
        inst = self._instances.get(instance_id)
        if inst is None:
            return None
        if sub_channel in (MX_CH_LEFT, MX_CH_RIGHT):
            try:
                left, right = self._backend_get_levels(inst)
            except Exception:
                return inst.last_values.get(sub_channel)
            return left if sub_channel == MX_CH_LEFT else right
        if sub_channel == MX_CH_MUTED:
            try:
                return 1.0 if self._backend_get_mute(inst) else 0.0
            except Exception:
                return inst.last_values.get(sub_channel)
        return inst.last_values.get(sub_channel)

    def update(self) -> None:
        for inst in self._instances.values():
            try:
                self._poll_telemetry(inst)
            except Exception as e:
                self._log("error",
                    f"audio_mixer#{inst.instance_id}: update raised: {e}")

    def estop(self) -> None:
        # Silence the output on estop and remember the prior mute state so
        # clear_estop can restore it without a full config re-push.
        for inst in self._instances.values():
            try:
                prior = self._backend_get_mute(inst)
            except Exception:
                prior = bool(inst.last_values.get(MX_CH_MUTE, 0.0) >= 0.5)
            inst.params["_estop_prior_mute"] = prior
            try:
                self._backend_set_mute(inst, True)
                inst.last_values[MX_CH_MUTE] = 1.0
            except Exception as e:
                self._log("error",
                    f"audio_mixer#{inst.instance_id}: estop mute raised: {e}")

    def clear_estop(self) -> None:
        for inst in self._instances.values():
            prior = bool(inst.params.pop("_estop_prior_mute", False))
            try:
                self._backend_set_mute(inst, prior)
                inst.last_values[MX_CH_MUTE] = 1.0 if prior else 0.0
            except Exception as e:
                self._log("error",
                    f"audio_mixer#{inst.instance_id}: clear_estop raised: {e}")

    def reset(self) -> None:
        for inst in self._instances.values():
            try:
                self._backend_close(inst)
            except Exception:
                pass
        super().reset()

    # ── Helpers ───────────────────────────────────────────────────

    def _poll_telemetry(self, inst) -> None:
        left, right = self._backend_get_levels(inst)
        inst.last_values[MX_CH_LEFT] = float(left)
        inst.last_values[MX_CH_RIGHT] = float(right)
        inst.last_values[MX_CH_MUTED] = 1.0 if self._backend_get_mute(inst) else 0.0

    @staticmethod
    def _as_float(value: Any, fallback: float) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return fallback

    @staticmethod
    def balance_to_gains(volume_0_1: float, balance_m1_1: float) -> Tuple[float, float]:
        """Map a (master volume, balance) pair to per-channel L/R gains in
        0..1. Balance attenuates the opposite channel: panning right (+)
        pulls the left channel down and leaves the right at full master,
        and vice-versa. Center (0) leaves both at master."""
        vol = _clamp(volume_0_1, 0.0, 1.0)
        bal = _clamp(balance_m1_1, -1.0, 1.0)
        left = vol * (1.0 - max(0.0, bal))
        right = vol * (1.0 - max(0.0, -bal))
        return left, right

    # ── Backend surface — subclasses implement ────────────────────

    @abc.abstractmethod
    def _backend_open(self, inst) -> bool:
        """Open the mixer control this backend needs. Returns False on
        hard failure (missing lib, unknown control, card open error)."""

    @abc.abstractmethod
    def _backend_close(self, inst) -> None:
        """Release the backend resources on reset / shutdown."""

    @abc.abstractmethod
    def _backend_set_volume(self, inst, volume_0_1: float) -> None:
        """Set master output volume in 0..1 (applies with current balance)."""

    @abc.abstractmethod
    def _backend_set_balance(self, inst, balance_m1_1: float) -> None:
        """Set L/R balance in -1..1 (applies with current master volume)."""

    @abc.abstractmethod
    def _backend_set_mute(self, inst, muted: bool) -> None:
        """Mute (True) or unmute (False) the output."""

    @abc.abstractmethod
    def _backend_get_levels(self, inst) -> Tuple[float, float]:
        """Read back the current (left, right) channel levels in 0..1."""

    @abc.abstractmethod
    def _backend_get_mute(self, inst) -> bool:
        """Read back the current mute state."""
