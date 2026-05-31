"""
SAINT.OS Pi node — Pololu Tic stepper controller driver.

Port of firmware/shared/src/tic_driver.c. Tic does motion onboard —
the Pi just sends target_position / target_velocity / keepalive
commands over the Pololu binary protocol. Up to 8 Tics share one
UART via per-chip device IDs (1..127).

Sub-channels per unit (matches firmware):
  0  target_position    write, [-1,1] -> ±max_position
  1  target_velocity    write, [-1,1] -> ±max_speed_pps
  2  current_position   read
  3  current_velocity   read
  4  vin_voltage        read (volts)
  5  error_status       read (uint16 bitset as float)
"""

from __future__ import annotations

import time
from typing import Any, Dict, Optional

from .base import PeripheralDriver
from ..uart_transport import UartTransport


TIC_POLOLU_START_BYTE       = 0xAA
TIC_DEFAULT_DEVICE_ID       = 14
TIC_DEFAULT_BAUD            = 9600
TIC_VIRTUAL_GPIO_BASE       = 300
TIC_CHANNELS_PER_UNIT       = 6
TIC_MAX_UNITS               = 8
TIC_KEEPALIVE_S             = 0.4
TIC_VELOCITY_WIRE_SCALE     = 10000  # pulses per 10000 seconds

# Commands (MSB-cleared on Pololu wire)
TIC_CMD_SET_TARGET_POSITION    = 0xE0
TIC_CMD_SET_TARGET_VELOCITY    = 0xE3
TIC_CMD_HALT_AND_HOLD          = 0x89
TIC_CMD_ENERGIZE               = 0x85
TIC_CMD_DEENERGIZE             = 0x86
TIC_CMD_EXIT_SAFE_START        = 0x83
TIC_CMD_RESET_COMMAND_TIMEOUT  = 0x8C
TIC_CMD_GET_VARIABLE           = 0xA1

# Variable offsets
TIC_VAR_ERROR_STATUS     = 0x02
TIC_VAR_TARGET_POSITION  = 0x0A
TIC_VAR_CURRENT_POSITION = 0x22
TIC_VAR_CURRENT_VELOCITY = 0x26
TIC_VAR_VIN_VOLTAGE      = 0x33

# Sub-channel indexes
TIC_SUB_TARGET_POSITION  = 0
TIC_SUB_TARGET_VELOCITY  = 1
TIC_SUB_CURRENT_POSITION = 2
TIC_SUB_CURRENT_VELOCITY = 3
TIC_SUB_VIN_VOLTAGE      = 4
TIC_SUB_ERROR_STATUS     = 5


def encode_quick(device_id: int, cmd: int) -> bytes:
    return bytes([TIC_POLOLU_START_BYTE, device_id & 0x7F, cmd & 0x7F])


def encode_32bit(device_id: int, cmd: int, value: int) -> bytes:
    v = value & 0xFFFFFFFF
    msbs = 0
    if v & 0x80:        msbs |= 0x01
    if v & 0x8000:      msbs |= 0x02
    if v & 0x800000:    msbs |= 0x04
    if v & 0x80000000:  msbs |= 0x08
    return bytes([
        TIC_POLOLU_START_BYTE, device_id & 0x7F, cmd & 0x7F,
        msbs,
        v & 0x7F,
        (v >> 8) & 0x7F,
        (v >> 16) & 0x7F,
        (v >> 24) & 0x7F,
    ])


def encode_block_read(device_id: int, offset: int, length: int) -> bytes:
    return bytes([
        TIC_POLOLU_START_BYTE, device_id & 0x7F,
        TIC_CMD_GET_VARIABLE & 0x7F,
        offset & 0x7F, length & 0x7F,
    ])


def decode_i32_le(b: bytes) -> int:
    if len(b) < 4:
        return 0
    v = b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24)
    if v & 0x80000000:
        v -= 0x100000000
    return v


def decode_u16_le(b: bytes) -> int:
    if len(b) < 2:
        return 0
    return b[0] | (b[1] << 8)


class TicDriver(PeripheralDriver):
    TYPE_ID = "tic"
    MODE_STRING = "tic_stepper"
    VIRTUAL_GPIO_BASE = TIC_VIRTUAL_GPIO_BASE
    CHANNELS_PER_INSTANCE = TIC_CHANNELS_PER_UNIT
    MAX_INSTANCES = TIC_MAX_UNITS
    SUB_CHANNEL_NAMES = [
        "target_position", "target_velocity",
        "current_position", "current_velocity",
        "vin_voltage", "error_status",
    ]

    _transport_cls = UartTransport

    def __init__(self, logger=None):
        super().__init__(logger=logger)
        self._uart: Optional[UartTransport] = None
        self._uart_pins: tuple[int, int, int] = (0, 0, TIC_DEFAULT_BAUD)
        # instance_id -> device_id, max_position, max_speed_pps,
        # last_command_time, energized
        self._addresses: Dict[int, int] = {}
        self._scales: Dict[int, tuple[int, int]] = {}  # (max_pos, max_pps)
        self._last_cmd_time: Dict[int, float] = {}
        self._energized: Dict[int, bool] = {}
        self._poll_unit = 0
        self._poll_reg = 0
        # 0=current_pos, 1=current_vel, 2=vin, 3=error_status

    # ── PeripheralDriver overrides ────────────────────────────────

    def apply_config(self, instance_id: int, pins: Dict[str, int],
                     params: Dict[str, Any]) -> bool:
        if instance_id >= TIC_MAX_UNITS:
            return False
        tx = int(pins.get("uart_tx", 0))
        rx = int(pins.get("uart_rx", 0))
        baud = int(params.get("baud", TIC_DEFAULT_BAUD))
        device_id = int(params.get("address", TIC_DEFAULT_DEVICE_ID))
        max_pos = int(params.get("max_position", 10000))
        max_pps = int(params.get("max_speed_pps", 1000))

        if self._uart is None or self._uart_pins != (tx, rx, baud):
            if self._uart is not None:
                self._uart.close()
            self._uart = self._transport_cls(tx, rx, baud, logger=self._logger)
            if not self._uart.open():
                return False
            self._uart_pins = (tx, rx, baud)

        inst = self._get_or_create_instance(instance_id)
        inst.pins = dict(pins)
        inst.params = dict(params)
        self._addresses[instance_id] = device_id
        self._scales[instance_id] = (max_pos, max_pps)

        # Probe + auto-energize: same pattern as firmware.
        if self._probe(instance_id):
            inst.connected = True
            self._energize(instance_id)
        return True

    def update(self) -> None:
        """Send keepalive (Reset Command Timeout) for energized units,
        and round-robin one telemetry read per tick."""
        now = time.monotonic()
        # Keepalive — at most one packet per update.
        for inst_id, energized in self._energized.items():
            if not energized:
                continue
            last = self._last_cmd_time.get(inst_id, 0.0)
            if now - last < TIC_KEEPALIVE_S:
                continue
            self._send_quick(inst_id, TIC_CMD_RESET_COMMAND_TIMEOUT)
            self._last_cmd_time[inst_id] = now
            return  # one packet per tick keeps bus utilization bounded

        # Telemetry poll round-robin.
        if not self._addresses:
            return
        unit_ids = sorted(self._addresses)
        if self._poll_unit >= len(unit_ids):
            self._poll_unit = 0
        inst_id = unit_ids[self._poll_unit]
        if self._instances[inst_id].connected:
            self._poll_one_register(inst_id, self._poll_reg)
        self._poll_reg += 1
        if self._poll_reg > 3:
            self._poll_reg = 0
            self._poll_unit = (self._poll_unit + 1) % len(unit_ids)

    def set_value(self, instance_id: int, sub_channel: int, value: float) -> bool:
        if instance_id not in self._addresses or self._uart is None:
            return False
        max_pos, max_pps = self._scales[instance_id]
        value = max(-1.0, min(1.0, value))

        if sub_channel == TIC_SUB_TARGET_POSITION:
            position = int(round(value * max_pos))
            if not self._energized.get(instance_id):
                self._energize(instance_id)
            self._uart.write(encode_32bit(self._addresses[instance_id],
                                            TIC_CMD_SET_TARGET_POSITION,
                                            position))
            self._last_cmd_time[instance_id] = time.monotonic()
            self._instances[instance_id].last_values[sub_channel] = value
            return True

        if sub_channel == TIC_SUB_TARGET_VELOCITY:
            pps = int(round(value * max_pps))
            if not self._energized.get(instance_id):
                self._energize(instance_id)
            wire_vel = pps * TIC_VELOCITY_WIRE_SCALE
            # Clamp at int32 boundary.
            wire_vel = max(-2147483647, min(2147483647, wire_vel))
            self._uart.write(encode_32bit(self._addresses[instance_id],
                                            TIC_CMD_SET_TARGET_VELOCITY,
                                            wire_vel))
            self._last_cmd_time[instance_id] = time.monotonic()
            self._instances[instance_id].last_values[sub_channel] = value
            return True

        return False  # read-only sub-channels

    def get_value(self, instance_id: int, sub_channel: int) -> Optional[float]:
        inst = self._instances.get(instance_id)
        if inst is None:
            return None
        return inst.last_values.get(sub_channel)

    def estop(self) -> None:
        if self._uart is None:
            return
        for inst_id in list(self._addresses):
            self._uart.write(encode_quick(self._addresses[inst_id],
                                            TIC_CMD_HALT_AND_HOLD))
            self._uart.write(encode_quick(self._addresses[inst_id],
                                            TIC_CMD_DEENERGIZE))
            self._energized[inst_id] = False
        self._log("warn",
            f"Tic: ESTOP — deenergized {len(self._addresses)} unit(s)")

    def reset(self) -> None:
        if self._uart is not None:
            self._uart.close()
            self._uart = None
        self._uart_pins = (0, 0, TIC_DEFAULT_BAUD)
        self._addresses.clear()
        self._scales.clear()
        self._last_cmd_time.clear()
        self._energized.clear()
        self._poll_unit = 0
        self._poll_reg = 0
        super().reset()

    # ── helpers ───────────────────────────────────────────────────

    def _send_quick(self, instance_id: int, cmd: int) -> None:
        if self._uart is None or instance_id not in self._addresses:
            return
        self._uart.write(encode_quick(self._addresses[instance_id], cmd))

    def _energize(self, instance_id: int) -> None:
        self._send_quick(instance_id, TIC_CMD_ENERGIZE)
        self._send_quick(instance_id, TIC_CMD_EXIT_SAFE_START)
        self._energized[instance_id] = True
        self._last_cmd_time[instance_id] = time.monotonic()

    def _probe(self, instance_id: int) -> bool:
        """Read operation_state (offset 0, 1 byte) — link is alive if
        any reply comes back."""
        if self._uart is None:
            return False
        req = encode_block_read(self._addresses[instance_id], 0x00, 1)
        reply = self._uart.write_then_read(req, 1, drain_echo=False)
        return len(reply) >= 1

    def _poll_one_register(self, inst_id: int, reg: int) -> None:
        if self._uart is None:
            return
        addr = self._addresses[inst_id]
        inst = self._instances[inst_id]
        if reg == 0:
            data = self._uart.write_then_read(
                encode_block_read(addr, TIC_VAR_CURRENT_POSITION, 4), 4)
            if len(data) >= 4:
                inst.last_values[TIC_SUB_CURRENT_POSITION] = float(decode_i32_le(data))
        elif reg == 1:
            data = self._uart.write_then_read(
                encode_block_read(addr, TIC_VAR_CURRENT_VELOCITY, 4), 4)
            if len(data) >= 4:
                wire = decode_i32_le(data)
                inst.last_values[TIC_SUB_CURRENT_VELOCITY] = float(
                    wire // TIC_VELOCITY_WIRE_SCALE)
        elif reg == 2:
            data = self._uart.write_then_read(
                encode_block_read(addr, TIC_VAR_VIN_VOLTAGE, 2), 2)
            if len(data) >= 2:
                inst.last_values[TIC_SUB_VIN_VOLTAGE] = decode_u16_le(data) / 1000.0
        elif reg == 3:
            data = self._uart.write_then_read(
                encode_block_read(addr, TIC_VAR_ERROR_STATUS, 2), 2)
            if len(data) >= 2:
                inst.last_values[TIC_SUB_ERROR_STATUS] = float(decode_u16_le(data))
