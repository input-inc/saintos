"""
SAINT.OS Pi node — BasicMicro RoboClaw Solo 60A driver.

Port of firmware/shared/src/roboclaw_driver.c. Up to 8 RoboClaws on
one UART, addresses 0x80..0x87, CRC16 packet serial.

Per-unit sub-channels (matches firmware):
  0  motor    write,  [-1, 1] -> ±32767 duty
  1  encoder  read,   int32 position
  2  voltage  read,   battery V
  3  current  read,   motor amps
  4  temp     read,   °C

Duty keepalive: the RoboClaw has a built-in serial-timeout watchdog
(Motion Studio default 1s) that kills motion if no command arrives.
We resend M1DUTY every 400 ms for any unit with non-zero duty —
matches the firmware's maybe_send_duty_keepalive pattern.
"""

from __future__ import annotations

import os
import time
from typing import Any, Dict, Optional

from .base import PeripheralDriver
from ..uart_transport import UartTransport


# gpiod is only available on the Pi. On the Mac dev box we fall back to
# a no-op mock so apply_config doesn't have to special-case the missing
# dependency.
_SIMULATION_MODE = os.environ.get("SAINT_SIMULATION", "0") == "1"
_GPIOD_AVAILABLE = False
if not _SIMULATION_MODE:
    try:
        import gpiod
        from gpiod.line import Direction, Value
        _GPIOD_AVAILABLE = True
    except ImportError:
        pass


class _EstopPin:
    """One gpiod-backed output line for a RoboClaw's S3 (E-stop) input.

    Drives LOW = deasserted (motor enabled), HIGH = asserted (S3 latched
    → controller silently drops commands until reset). The driver lowers
    the line in apply_config so a floating boot signal can't trip the
    latch, and raises it on ESTOP.

    Falls back to a logging-only no-op when gpiod isn't available
    (Mac dev). Same shape as the SIMULATION mock in gpio_control.py."""

    # Pi 5 GPIO chip name. Mirrors GPIOController.GPIO_CHIP.
    DEFAULT_CHIP = "gpiochip4"

    def __init__(self, pin: int, logger=None, chip_name: Optional[str] = None):
        self._pin = pin
        self._logger = logger
        self._chip = None
        self._req = None
        self._state_high = False

        if not _GPIOD_AVAILABLE:
            self._log("info",
                f"RoboClaw estop_pin {pin}: gpiod unavailable — mock mode "
                f"(set/clear will only log)")
            return

        chip_name = chip_name or self.DEFAULT_CHIP
        try:
            self._chip = gpiod.Chip(chip_name)
            line_config = gpiod.LineSettings(
                direction=Direction.OUTPUT,
                output_value=Value.INACTIVE,
            )
            self._req = self._chip.request_lines(
                config={pin: line_config},
                consumer=f"saint_node_roboclaw_estop_gpio{pin}",
            )
            self._log("info",
                f"RoboClaw estop_pin {pin}: claimed on {chip_name}, "
                f"driving LOW (deasserted)")
        except Exception as e:
            self._log("error",
                f"RoboClaw estop_pin {pin}: failed to claim on "
                f"{chip_name} — {e}. Motor S3 latch protection is "
                f"NOT active for this unit.")
            self._req = None

    def set_high(self) -> None:
        self._state_high = True
        if self._req is None:
            self._log("warn",
                f"RoboClaw estop_pin {self._pin}: would drive HIGH "
                f"(mock — no real GPIO)")
            return
        try:
            self._req.set_value(self._pin, Value.ACTIVE)
        except Exception as e:
            self._log("error",
                f"RoboClaw estop_pin {self._pin}: set HIGH failed — {e}")

    def set_low(self) -> None:
        self._state_high = False
        if self._req is None:
            return
        try:
            self._req.set_value(self._pin, Value.INACTIVE)
        except Exception as e:
            self._log("error",
                f"RoboClaw estop_pin {self._pin}: set LOW failed — {e}")

    def is_high(self) -> bool:
        return self._state_high

    def release(self) -> None:
        if self._req is not None:
            try:
                self._req.release()
            except Exception:
                pass
            self._req = None
        if self._chip is not None:
            try:
                self._chip.close()
            except Exception:
                pass
            self._chip = None

    def _log(self, level: str, msg: str) -> None:
        if self._logger:
            getattr(self._logger, level)(msg)
        else:
            print(f"[{level.upper()}] {msg}")


ROBOCLAW_DEFAULT_BAUD          = 38400
ROBOCLAW_ADDRESS_MIN           = 0x80
ROBOCLAW_ADDRESS_MAX           = 0x87
ROBOCLAW_ACK_BYTE              = 0xFF
ROBOCLAW_VIRTUAL_GPIO_BASE     = 236
ROBOCLAW_CHANNELS_PER_UNIT     = 5
ROBOCLAW_MAX_UNITS             = 8
ROBOCLAW_DUTY_MAX              = 32767
ROBOCLAW_DUTY_KEEPALIVE_S      = 0.4
ROBOCLAW_RESPONSE_TIMEOUT_S    = 0.05

# Commands (Solo = M1 only)
ROBOCLAW_CMD_M1DUTY            = 32
ROBOCLAW_CMD_GETM1ENC          = 16
ROBOCLAW_CMD_GETVERSION        = 21
ROBOCLAW_CMD_GETMBATT          = 24
ROBOCLAW_CMD_GETCURRENTS       = 49
ROBOCLAW_CMD_GETTEMP           = 82

# Sub-channel indexes
ROBOCLAW_SUB_MOTOR             = 0
ROBOCLAW_SUB_ENCODER           = 1
ROBOCLAW_SUB_VOLTAGE           = 2
ROBOCLAW_SUB_CURRENT           = 3
ROBOCLAW_SUB_TEMP              = 4


def crc16_update(crc: int, byte: int) -> int:
    crc ^= (byte & 0xFF) << 8
    for _ in range(8):
        if crc & 0x8000:
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF
        else:
            crc = (crc << 1) & 0xFFFF
    return crc


def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = crc16_update(crc, b)
    return crc


def build_packet(address: int, command: int, data: bytes = b"") -> bytes:
    body = bytes([address, command]) + data
    c = crc16(body)
    return body + bytes([(c >> 8) & 0xFF, c & 0xFF])


class RoboClawDriver(PeripheralDriver):
    TYPE_ID = "roboclaw"
    MODE_STRING = "roboclaw_motor"
    VIRTUAL_GPIO_BASE = ROBOCLAW_VIRTUAL_GPIO_BASE
    CHANNELS_PER_INSTANCE = ROBOCLAW_CHANNELS_PER_UNIT
    MAX_INSTANCES = ROBOCLAW_MAX_UNITS
    SUB_CHANNEL_NAMES = ["motor", "encoder", "voltage", "current", "temp"]

    _transport_cls = UartTransport
    # Swappable for tests so we don't need real gpiod calls.
    _estop_pin_cls = _EstopPin

    def __init__(self, logger=None):
        super().__init__(logger=logger)
        self._uart: Optional[UartTransport] = None
        self._uart_pins: tuple[int, int, int] = (0, 0, ROBOCLAW_DEFAULT_BAUD)
        self._addresses: Dict[int, int] = {}
        self._duties: Dict[int, int] = {}        # signed int (-32767..+32767)
        self._invert: Dict[int, bool] = {}
        self._last_duty_send: Dict[int, float] = {}
        # estop_pin GPIO outputs, keyed by instance_id. Only populated
        # for units whose params specified an estop_pin.
        self._estop_pins: Dict[int, _EstopPin] = {}
        # Round-robin telemetry poll cursor
        self._poll_unit = 0
        self._poll_reg = 0  # 0=enc, 1=batt, 2=cur, 3=temp

    # ── PeripheralDriver overrides ────────────────────────────────

    def apply_config(self, instance_id: int, pins: Dict[str, int],
                     params: Dict[str, Any]) -> bool:
        if instance_id >= ROBOCLAW_MAX_UNITS:
            return False
        tx = int(pins.get("uart_tx", 0))
        rx = int(pins.get("uart_rx", 0))
        baud = int(params.get("baud", ROBOCLAW_DEFAULT_BAUD))
        address = int(params.get("address", ROBOCLAW_ADDRESS_MIN + instance_id))
        if not (ROBOCLAW_ADDRESS_MIN <= address <= ROBOCLAW_ADDRESS_MAX):
            self._log("warn",
                f"RoboClaw#{instance_id}: address 0x{address:02X} outside "
                f"0x{ROBOCLAW_ADDRESS_MIN:02X}..0x{ROBOCLAW_ADDRESS_MAX:02X}")
            return False

        if self._uart is None or self._uart_pins != (tx, rx, baud):
            if self._uart is not None:
                self._uart.close()
            self._uart = self._transport_cls(
                tx, rx, baud,
                timeout_s=ROBOCLAW_RESPONSE_TIMEOUT_S,
                logger=self._logger)
            if not self._uart.open():
                return False
            self._uart_pins = (tx, rx, baud)

        inst = self._get_or_create_instance(instance_id)
        inst.pins = dict(pins)
        inst.params = dict(params)
        self._addresses[instance_id] = address
        self._duties[instance_id] = 0
        self._invert[instance_id] = bool(params.get("invert_direction", False))
        self._last_duty_send[instance_id] = 0.0

        # uart_swap was a PCB-wiring workaround on the RP2040 (PIO-based
        # TX/RX swap). The Pi UART is silicon-fixed — no equivalent path.
        # Surface a warn so it's obvious from logs that the dashboard
        # toggle didn't change behavior; the operator must fix the PCB.
        if params.get("uart_swap"):
            self._log("warn",
                f"RoboClaw#{instance_id}: 'uart_swap' is set but not "
                f"supported on the Pi node — ignored. Fix PCB wiring "
                f"or use a node with the PIO-UART path (RP2040).")

        # estop_pin: GPIO output wired to the controller's S3 (E-stop
        # input). Drive LOW now so a floating boot signal can't trip
        # the RoboClaw's latching S3 mode before we even talk to it.
        # ESTOP raises this HIGH to trip S3 independently of UART comms.
        # Release any prior estop pin claim if the operator changed it.
        prior = self._estop_pins.pop(instance_id, None)
        if prior is not None:
            prior.release()
        estop_pin_num = int(params.get("estop_pin", 0))
        if estop_pin_num > 0:
            estop = self._estop_pin_cls(estop_pin_num, logger=self._logger)
            estop.set_low()
            self._estop_pins[instance_id] = estop

        # Probe via GETVERSION; if it answers, mark connected.
        version = self._probe(instance_id)
        if version is not None:
            inst.connected = True
            self._log("info",
                f"RoboClaw#{instance_id} (0x{address:02X}) connected: {version}")
        return True

    def update(self) -> None:
        """Duty keepalive (one packet per tick if due) + round-robin
        telemetry poll for connected units."""
        if not self._addresses or self._uart is None:
            return

        now = time.monotonic()
        # Keepalive first — the RoboClaw watchdog is the safety-
        # critical concern.
        for inst_id, duty in self._duties.items():
            if duty == 0:
                continue
            if now - self._last_duty_send.get(inst_id, 0.0) < ROBOCLAW_DUTY_KEEPALIVE_S:
                continue
            self._send_duty(inst_id, duty)
            return  # one packet per tick

        # Telemetry round-robin.
        unit_ids = sorted(uid for uid in self._addresses
                          if self._instances[uid].connected)
        if not unit_ids:
            return
        if self._poll_unit >= len(unit_ids):
            self._poll_unit = 0
        inst_id = unit_ids[self._poll_unit]
        self._poll_one(inst_id, self._poll_reg)
        self._poll_reg += 1
        if self._poll_reg > 3:
            self._poll_reg = 0
            self._poll_unit = (self._poll_unit + 1) % len(unit_ids)

    def set_value(self, instance_id: int, sub_channel: int, value: float) -> bool:
        if sub_channel != ROBOCLAW_SUB_MOTOR:
            return False
        if instance_id not in self._addresses or self._uart is None:
            return False
        value = max(-1.0, min(1.0, value))
        duty = int(round(value * ROBOCLAW_DUTY_MAX))
        duty = max(-ROBOCLAW_DUTY_MAX, min(ROBOCLAW_DUTY_MAX, duty))
        self._duties[instance_id] = duty
        self._send_duty(instance_id, duty)
        self._instances[instance_id].last_values[ROBOCLAW_SUB_MOTOR] = value
        return True

    def get_value(self, instance_id: int, sub_channel: int) -> Optional[float]:
        inst = self._instances.get(instance_id)
        if inst is None:
            return None
        return inst.last_values.get(sub_channel)

    def estop(self) -> None:
        # Defense in depth, same shape as the firmware driver:
        #   1) GPIO output: drive each unit's estop_pin HIGH so the
        #      RoboClaw's S3 latches into emergency-stop independent
        #      of anything the UART does next. Survives a wedged UART.
        #   2) UART: command duty=0 to every configured unit so the
        #      motor brakes from the controller side even on PCBs that
        #      don't route an estop_pin.
        for inst_id, estop in self._estop_pins.items():
            estop.set_high()
        for inst_id in list(self._addresses):
            self.set_value(inst_id, ROBOCLAW_SUB_MOTOR, 0.0)
        self._log("warn",
            f"RoboClaw: ESTOP — duty 0 on {len(self._addresses)} unit(s), "
            f"estop_pin HIGH on {len(self._estop_pins)}")

    def clear_estop(self) -> None:
        """Lower every unit's estop_pin so the controller's S3 latch
        releases on the next motor command. Motors stay at duty=0
        until the operator commands a new value — same convention as
        the firmware driver. No-op on units without an estop_pin."""
        for inst_id, estop in self._estop_pins.items():
            estop.set_low()
        if self._estop_pins:
            self._log("info",
                f"RoboClaw: estop_pin LOW on {len(self._estop_pins)} "
                f"unit(s) — motor commands will be honored again")

    def reset(self) -> None:
        if self._uart is not None:
            self._uart.close()
            self._uart = None
        self._uart_pins = (0, 0, ROBOCLAW_DEFAULT_BAUD)
        # Release every estop_pin's gpiod handle so the next
        # apply_config can re-claim the same pin cleanly.
        for estop in self._estop_pins.values():
            estop.release()
        self._estop_pins.clear()
        self._addresses.clear()
        self._duties.clear()
        self._invert.clear()
        self._last_duty_send.clear()
        self._poll_unit = 0
        self._poll_reg = 0
        super().reset()

    # ── wire I/O helpers ──────────────────────────────────────────

    def _send_duty(self, instance_id: int, duty: int) -> None:
        if self._uart is None:
            return
        address = self._addresses[instance_id]
        wire_duty = -duty if self._invert.get(instance_id) else duty
        wire_duty &= 0xFFFF  # twos-complement to uint16
        data = bytes([(wire_duty >> 8) & 0xFF, wire_duty & 0xFF])
        packet = build_packet(address, ROBOCLAW_CMD_M1DUTY, data)
        # M1DUTY expects an ACK byte 0xFF.
        reply = self._uart.write_then_read(packet, 1, drain_echo=False)
        ack_ok = len(reply) == 1 and reply[0] == ROBOCLAW_ACK_BYTE
        self._last_duty_send[instance_id] = time.monotonic()
        inst = self._instances.get(instance_id)
        if inst is not None:
            inst.connected = ack_ok or inst.connected  # one miss doesn't flip

    def _probe(self, instance_id: int) -> Optional[str]:
        """GETVERSION returns a null-terminated string + 2 CRC bytes."""
        if self._uart is None:
            return None
        address = self._addresses[instance_id]
        packet = build_packet(address, ROBOCLAW_CMD_GETVERSION)
        # Read up to 50 bytes; version strings are <30 chars usually.
        reply = self._uart.write_then_read(packet, 48, drain_echo=False)
        # Find null terminator + 2 CRC bytes.
        nul = reply.find(b"\x00")
        if nul < 0 or nul + 2 >= len(reply):
            return None
        version_bytes = reply[:nul]
        crc_bytes = reply[nul + 1: nul + 3]
        expected_crc = 0
        expected_crc = crc16_update(expected_crc, address)
        expected_crc = crc16_update(expected_crc, ROBOCLAW_CMD_GETVERSION)
        for b in reply[: nul + 1]:
            expected_crc = crc16_update(expected_crc, b)
        received_crc = (crc_bytes[0] << 8) | crc_bytes[1]
        if expected_crc != received_crc:
            return None
        try:
            return version_bytes.decode("ascii", errors="replace")
        except Exception:
            return "<unprintable>"

    def _read_value_with_crc(self, address: int, command: int,
                              expected_len: int) -> Optional[bytes]:
        """Send the read packet, expect `expected_len` payload bytes
        plus 2 CRC bytes. CRC covers [address, command, payload]."""
        if self._uart is None:
            return None
        packet = build_packet(address, command)
        reply = self._uart.write_then_read(packet, expected_len + 2,
                                              drain_echo=False)
        if len(reply) < expected_len + 2:
            return None
        crc = 0
        crc = crc16_update(crc, address)
        crc = crc16_update(crc, command)
        for b in reply[:expected_len]:
            crc = crc16_update(crc, b)
        received = (reply[expected_len] << 8) | reply[expected_len + 1]
        if crc != received:
            return None
        return reply[:expected_len]

    def _poll_one(self, inst_id: int, reg: int) -> None:
        address = self._addresses[inst_id]
        inst = self._instances[inst_id]
        if reg == 0:  # encoder
            payload = self._read_value_with_crc(address, ROBOCLAW_CMD_GETM1ENC, 5)
            if payload is None:
                return
            enc = (payload[0] << 24) | (payload[1] << 16) | (payload[2] << 8) | payload[3]
            if enc & 0x80000000:
                enc -= 0x100000000
            inst.last_values[ROBOCLAW_SUB_ENCODER] = float(enc)
        elif reg == 1:  # battery voltage (value/10 = volts)
            payload = self._read_value_with_crc(address, ROBOCLAW_CMD_GETMBATT, 2)
            if payload is None:
                return
            raw = (payload[0] << 8) | payload[1]
            inst.last_values[ROBOCLAW_SUB_VOLTAGE] = raw / 10.0
        elif reg == 2:  # motor current (value/100 = amps; M1 = first 2 bytes)
            payload = self._read_value_with_crc(address, ROBOCLAW_CMD_GETCURRENTS, 4)
            if payload is None:
                return
            raw = (payload[0] << 8) | payload[1]
            if raw & 0x8000:
                raw -= 0x10000
            inst.last_values[ROBOCLAW_SUB_CURRENT] = raw / 100.0
        elif reg == 3:  # temperature (value/10 = °C)
            payload = self._read_value_with_crc(address, ROBOCLAW_CMD_GETTEMP, 2)
            if payload is None:
                return
            raw = (payload[0] << 8) | payload[1]
            if raw & 0x8000:
                raw -= 0x10000
            inst.last_values[ROBOCLAW_SUB_TEMP] = raw / 10.0
