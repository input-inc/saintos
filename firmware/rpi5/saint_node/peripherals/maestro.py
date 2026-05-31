"""
SAINT.OS Pi node — Pololu Maestro servo controller driver.

Port of firmware/shared/src/maestro_driver.c (UART path only — no
USB-host on Pi side for now). Maestro speaks the Pololu protocol over
TTL serial:

    [0xAA] [device_number] [command & 0x7F] [data bytes (MSB cleared)]

Up to 24 servo channels on a single Maestro. Operator-side scaling:
each channel has its own min_pulse_us / max_pulse_us / neutral_us;
set_value(channel, value) where value ∈ [-1.0, 1.0] maps to the
configured pulse range.

We don't poll positions back by default — the cost is high on shared
buses and the operator usually wants commanded value displayed (not
mechanically-stale chip readback). Add a `poll_positions` param flag
later if needed.
"""

from __future__ import annotations

from typing import Any, Dict, Optional

from .base import PeripheralDriver
from ..uart_transport import UartTransport


MAESTRO_DEFAULT_BAUD       = 9600
MAESTRO_DEFAULT_DEVICE_NUM = 12
MAESTRO_POLOLU_START_BYTE  = 0xAA
MAESTRO_CMD_SET_TARGET     = 0x04
MAESTRO_CMD_SET_SPEED      = 0x07
MAESTRO_CMD_SET_ACCEL      = 0x09
MAESTRO_CMD_GO_HOME        = 0x22
MAESTRO_VIRTUAL_GPIO_BASE  = 200
MAESTRO_MAX_CHANNELS       = 24

# Operator-facing pulse range defaults (microseconds).
MAESTRO_DEFAULT_MIN_PULSE_US = 1000
MAESTRO_DEFAULT_MAX_PULSE_US = 2000
MAESTRO_DEFAULT_NEUTRAL_US   = 1500


def _build_pololu_frame(device: int, cmd: int,
                        data: bytes = b"") -> bytes:
    """Pololu-protocol frame: 0xAA, device, (cmd & 0x7F), data."""
    return bytes([MAESTRO_POLOLU_START_BYTE, device & 0x7F, cmd & 0x7F]) + data


def _pulse_us_to_quarters(pulse_us: int) -> tuple[int, int]:
    """Maestro target is in quarter-microseconds. Split into the two
    7-bit bytes the protocol expects (low first, then high)."""
    quarters = max(0, min(16383, pulse_us * 4))
    return (quarters & 0x7F, (quarters >> 7) & 0x7F)


class MaestroDriver(PeripheralDriver):
    """Pololu Maestro UART driver.

    One instance == one Maestro chip (up to 24 servo channels). The
    "sub_channel" is the Maestro servo channel index. We expose
    CHANNELS_PER_INSTANCE = 24 and only one instance per node — the
    firmware caps the same way.
    """

    TYPE_ID = "maestro"
    MODE_STRING = "maestro_servo"
    VIRTUAL_GPIO_BASE = MAESTRO_VIRTUAL_GPIO_BASE
    CHANNELS_PER_INSTANCE = MAESTRO_MAX_CHANNELS
    MAX_INSTANCES = 1
    SUB_CHANNEL_NAMES = [f"ch{i}" for i in range(MAESTRO_MAX_CHANNELS)]

    _transport_cls = UartTransport

    def __init__(self, logger=None):
        super().__init__(logger=logger)
        self._uart: Optional[UartTransport] = None
        self._uart_pins: tuple[int, int, int] = (0, 0, MAESTRO_DEFAULT_BAUD)
        self._device_num: int = MAESTRO_DEFAULT_DEVICE_NUM
        # Per-servo-channel config; lives on the instance.
        # channel_idx -> (min_us, max_us, neutral_us)
        self._channel_cfg: Dict[int, tuple[int, int, int]] = {}

    # ── PeripheralDriver overrides ────────────────────────────────

    def apply_config(self, instance_id: int, pins: Dict[str, int],
                     params: Dict[str, Any]) -> bool:
        if instance_id != 0:
            self._log("warn", f"Maestro: only one instance supported")
            return False

        tx = int(pins.get("uart_tx", 0))
        rx = int(pins.get("uart_rx", 0))
        baud = int(params.get("baud", MAESTRO_DEFAULT_BAUD))
        device_num = int(params.get("device_number", MAESTRO_DEFAULT_DEVICE_NUM))
        channels = params.get("channels", {}) or {}

        if self._uart is None or self._uart_pins != (tx, rx, baud):
            if self._uart is not None:
                self._uart.close()
            self._uart = self._transport_cls(tx, rx, baud, logger=self._logger)
            if not self._uart.open():
                return False
            self._uart_pins = (tx, rx, baud)

        self._device_num = device_num
        inst = self._get_or_create_instance(0)
        inst.pins = dict(pins)
        inst.params = dict(params)
        inst.connected = True

        # Per-channel cfg comes as either a dict or a list keyed by ch
        # index. Normalize. Each channel cfg supports min_pulse_us,
        # max_pulse_us, neutral_us, speed, acceleration, home_us.
        if isinstance(channels, dict):
            ch_iter = channels.items()
        elif isinstance(channels, list):
            ch_iter = enumerate(channels)
        else:
            ch_iter = []

        for k, ch_cfg in ch_iter:
            try:
                ch = int(k)
            except (TypeError, ValueError):
                continue
            if not (0 <= ch < MAESTRO_MAX_CHANNELS):
                continue
            min_us = int(ch_cfg.get("min_pulse_us", MAESTRO_DEFAULT_MIN_PULSE_US))
            max_us = int(ch_cfg.get("max_pulse_us", MAESTRO_DEFAULT_MAX_PULSE_US))
            neu_us = int(ch_cfg.get("neutral_us", MAESTRO_DEFAULT_NEUTRAL_US))
            self._channel_cfg[ch] = (min_us, max_us, neu_us)

            # Speed + acceleration are optional but useful — push them
            # to the chip when the operator sets them so set_target
            # respects motion limits.
            speed = ch_cfg.get("speed")
            if speed is not None:
                self._send_set_speed(ch, int(speed))
            accel = ch_cfg.get("acceleration")
            if accel is not None:
                self._send_set_accel(ch, int(accel))
        return True

    def set_value(self, instance_id: int, sub_channel: int, value: float) -> bool:
        if instance_id != 0 or self._uart is None:
            return False
        if not (0 <= sub_channel < MAESTRO_MAX_CHANNELS):
            return False
        value = max(-1.0, min(1.0, value))

        min_us, max_us, neu_us = self._channel_cfg.get(
            sub_channel,
            (MAESTRO_DEFAULT_MIN_PULSE_US,
             MAESTRO_DEFAULT_MAX_PULSE_US,
             MAESTRO_DEFAULT_NEUTRAL_US))
        # Map [-1, 1] -> [min_us, max_us] anchored at neutral.
        if value >= 0:
            pulse_us = int(round(neu_us + value * (max_us - neu_us)))
        else:
            pulse_us = int(round(neu_us + value * (neu_us - min_us)))

        low7, high7 = _pulse_us_to_quarters(pulse_us)
        frame = _build_pololu_frame(
            self._device_num, MAESTRO_CMD_SET_TARGET,
            bytes([sub_channel & 0x7F, low7, high7]))
        if not self._uart.write(frame):
            return False
        inst = self._instances[0]
        inst.last_values[sub_channel] = value
        return True

    def get_value(self, instance_id: int, sub_channel: int) -> Optional[float]:
        if instance_id != 0:
            return None
        inst = self._instances.get(0)
        if inst is None:
            return None
        return inst.last_values.get(sub_channel)

    def estop(self) -> None:
        """Send Go Home (0x22) to release every channel to its
        configured home position. The Maestro's per-channel "home"
        setting in chip flash controls actual behavior (Off / Ignore /
        Go to X) — we trust the operator's chip-side setup."""
        if self._uart is None:
            return
        frame = _build_pololu_frame(self._device_num, MAESTRO_CMD_GO_HOME)
        self._uart.write(frame)
        self._log("warn", "Maestro: ESTOP — Go Home sent")

    def reset(self) -> None:
        if self._uart is not None:
            self._uart.close()
            self._uart = None
        self._uart_pins = (0, 0, MAESTRO_DEFAULT_BAUD)
        self._channel_cfg.clear()
        super().reset()

    # ── helpers ───────────────────────────────────────────────────

    def _send_set_speed(self, channel: int, speed: int) -> None:
        if self._uart is None:
            return
        speed = max(0, min(16383, speed))
        frame = _build_pololu_frame(
            self._device_num, MAESTRO_CMD_SET_SPEED,
            bytes([channel & 0x7F, speed & 0x7F, (speed >> 7) & 0x7F]))
        self._uart.write(frame)

    def _send_set_accel(self, channel: int, accel: int) -> None:
        if self._uart is None:
            return
        accel = max(0, min(255, accel))
        frame = _build_pololu_frame(
            self._device_num, MAESTRO_CMD_SET_ACCEL,
            bytes([channel & 0x7F, accel & 0x7F, (accel >> 7) & 0x7F]))
        self._uart.write(frame)
