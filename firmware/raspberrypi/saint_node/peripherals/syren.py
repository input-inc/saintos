"""
SAINT.OS Pi node — SyRen 50 motor controller driver.

Port of firmware/shared/src/syren_driver.c. SyRen is a write-only
serial motor controller using the Sabertooth/SyRen packetized
serial protocol:

    [address] [command] [data] [checksum]

Where checksum = (address + command + data) & 0x7F.

Up to 8 SyRens share one UART line via addresses 128..135. The chip
"autobaud" feature locks to whatever rate it sees an 0xAA byte at on
first power-on — we send 0xAA at open() to ensure the controller is
synced.
"""

from __future__ import annotations

from typing import Any, Dict, Optional

from .base import PeripheralDriver
from ..uart_transport import UartTransport


SYREN_DEFAULT_BAUD     = 9600
SYREN_AUTOBAUD_BYTE    = 0xAA
SYREN_ADDRESS_MIN      = 128
SYREN_ADDRESS_MAX      = 135
SYREN_CMD_MOTOR_FWD    = 0
SYREN_CMD_MOTOR_REV    = 1
SYREN_VIRTUAL_GPIO_BASE = 224  # matches firmware/shared/include/syren_protocol.h


def _build_packet(address: int, command: int, value: int) -> bytes:
    """Encode a single 4-byte SyRen frame. Caller is expected to clamp
    `value` to 0..127 — the SyRen protocol only uses 7 bits per byte
    so a non-conforming value will desync the checksum on the chip side."""
    value = value & 0x7F
    checksum = (address + command + value) & 0x7F
    return bytes([address, command, value, checksum])


class SyRenDriver(PeripheralDriver):
    """SyRen 50 driver. One sub-channel per instance:
        0: motor (writable, [-1.0, 1.0])
    """

    TYPE_ID = "syren"
    MODE_STRING = "syren_motor"
    VIRTUAL_GPIO_BASE = SYREN_VIRTUAL_GPIO_BASE
    CHANNELS_PER_INSTANCE = 1
    MAX_INSTANCES = 8
    SUB_CHANNEL_NAMES = ["motor"]

    # Make the transport class swappable for tests.
    _transport_cls = UartTransport

    def __init__(self, logger=None):
        super().__init__(logger=logger)
        # All SyRens on a node share one UART pair. The first apply_config
        # call binds it; subsequent ones with the same pins are no-ops.
        self._uart: Optional[UartTransport] = None
        self._uart_pins: tuple[int, int, int] = (0, 0, SYREN_DEFAULT_BAUD)
        self._addresses: Dict[int, int] = {}
        """instance_id -> 128..135"""

    # ── PeripheralDriver overrides ────────────────────────────────

    def apply_config(self, instance_id: int, pins: Dict[str, int],
                     params: Dict[str, Any]) -> bool:
        if instance_id >= self.MAX_INSTANCES:
            self._log("warn",
                f"SyRen: instance_id {instance_id} exceeds max "
                f"{self.MAX_INSTANCES}")
            return False

        address = int(params.get("address", SYREN_ADDRESS_MIN + instance_id))
        if not (SYREN_ADDRESS_MIN <= address <= SYREN_ADDRESS_MAX):
            self._log("warn",
                f"SyRen#{instance_id}: address {address} outside "
                f"{SYREN_ADDRESS_MIN}..{SYREN_ADDRESS_MAX}")
            return False

        tx = int(pins.get("uart_tx", 0))
        rx = int(pins.get("uart_rx", 0))
        baud = int(params.get("baud", SYREN_DEFAULT_BAUD))

        # Re-open the UART if the operator picked a different pair / baud
        # via this sync. First-call open also handled here.
        if self._uart is None or self._uart_pins != (tx, rx, baud):
            if self._uart is not None:
                self._uart.close()
            self._uart = self._transport_cls(tx, rx, baud, logger=self._logger)
            if not self._uart.open():
                return False
            self._uart_pins = (tx, rx, baud)
            # Autobaud byte. SyRen's first byte after power on sets its
            # rate; subsequent units sharing the bus rely on this being
            # sent before any addressed packet.
            self._uart.write(bytes([SYREN_AUTOBAUD_BYTE]))

        inst = self._get_or_create_instance(instance_id)
        inst.pins = dict(pins)
        inst.params = dict(params)
        inst.connected = True  # write-only — assume good until proven otherwise
        self._addresses[instance_id] = address
        return True

    def set_value(self, instance_id: int, sub_channel: int, value: float) -> bool:
        if sub_channel != 0:
            return False
        if self._uart is None or instance_id not in self._addresses:
            return False
        # Clamp to [-1, 1] then scale to SyRen's 7-bit range.
        value = max(-1.0, min(1.0, value))
        magnitude = int(round(abs(value) * 127))
        magnitude = min(magnitude, 127)
        command = SYREN_CMD_MOTOR_FWD if value >= 0 else SYREN_CMD_MOTOR_REV
        packet = _build_packet(self._addresses[instance_id], command, magnitude)
        if not self._uart.write(packet):
            return False
        inst = self._instances[instance_id]
        inst.last_values[0] = value
        return True

    def get_value(self, instance_id: int, sub_channel: int) -> Optional[float]:
        if sub_channel != 0:
            return None
        inst = self._instances.get(instance_id)
        if inst is None:
            return None
        return inst.last_values.get(0, 0.0)

    def estop(self) -> None:
        for instance_id in list(self._addresses):
            self.set_value(instance_id, 0, 0.0)
        self._log("warn",
            f"SyRen: ESTOP — set duty 0 on {len(self._addresses)} instance(s)")

    def reset(self) -> None:
        if self._uart is not None:
            self._uart.close()
            self._uart = None
        self._uart_pins = (0, 0, SYREN_DEFAULT_BAUD)
        self._addresses.clear()
        super().reset()
