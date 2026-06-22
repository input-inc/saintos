"""
SAINT.OS Pi node — Dimension Engineering Kangaroo X2 motion controller.

Port of firmware/shared/src/kangaroo_driver.c. The Kangaroo is a
self-tuning closed-loop motion controller riding on a Sabertooth/SyRen
power stage. It speaks TWO serial framings, selectable per channel:

  * packet  — binary, CRC-14 framed, bit-packed numbers (default; what
              DE's own library uses, robust against line noise)
  * simple  — newline-terminated ASCII ("1,start" / "1,p100 s50" / ...)

One board = one address (128..135) and two motor channels named by a
single character: '1'/'2' (independent) or 'D'/'T' (mixed). Up to 8
channels share one UART. Each channel MUST be sent Start after power-up
before it honors motion commands.

Sub-channels per unit (matches firmware):
  0  target_position    write, [-1,1] -> ±max_position
  1  target_speed       write, [-1,1] -> ±max_speed
  2  current_position   read
  3  current_speed      read
  4  moving             read (1 = motion pending / "busy")
  5  error_status       read (last Kangaroo error code, 0 = OK)

crc14 / bitpack / bitunpack are verbatim ports of DE's Arduino/C#
library, identical to firmware/shared/include/kangaroo_protocol.h.
"""

from __future__ import annotations

from typing import Any, Dict, Optional, Tuple

from .base import PeripheralDriver
from ..uart_transport import UartTransport


KANGAROO_DEFAULT_BAUD    = 9600
KANGAROO_ADDRESS_MIN     = 128
KANGAROO_ADDRESS_MAX     = 135
KANGAROO_DEFAULT_ADDRESS = 128
KANGAROO_VIRTUAL_GPIO_BASE = 364   # matches firmware/shared/include/kangaroo_protocol.h
KANGAROO_CHANNELS_PER_UNIT = 6
KANGAROO_MAX_UNITS         = 8

# Packet command opcodes
KANGAROO_CMD_START   = 32
KANGAROO_CMD_HOME    = 34
KANGAROO_CMD_STATUS  = 35   # Get
KANGAROO_CMD_MOVE    = 36
KANGAROO_CMD_SYSTEM  = 37
KANGAROO_RC_STATUS   = 67   # Get/Status reply opcode

KANGAROO_MOVE_POSITION = 1
KANGAROO_MOVE_SPEED    = 2

KANGAROO_GET_POSITION  = 1
KANGAROO_GET_SPEED     = 2

KANGAROO_SYS_POWERDOWN = 0

# Get/Status reply flag bits
KANGAROO_STATUS_ERROR     = 0x01
KANGAROO_STATUS_BUSY      = 0x02
KANGAROO_STATUS_ECHO_CODE = 0x10
KANGAROO_STATUS_SEQUENCE  = 0x40

KANGAROO_ERR_NOT_STARTED = 1

# Sub-channel indexes
KANGAROO_SUB_TARGET_POSITION  = 0
KANGAROO_SUB_TARGET_SPEED     = 1
KANGAROO_SUB_CURRENT_POSITION = 2
KANGAROO_SUB_CURRENT_SPEED    = 3
KANGAROO_SUB_MOVING           = 4
KANGAROO_SUB_ERROR_STATUS     = 5

KANGAROO_DEFAULT_MAX_POSITION = 10000
KANGAROO_DEFAULT_MAX_SPEED    = 1000


# ── Wire primitives (verbatim port of DE crc14 / bitpackNumber) ──────

def crc14(data: bytes) -> int:
    crc = 0x3FFF
    for byte in data:
        crc ^= byte & 0x7F
        for _ in range(7):
            if crc & 1:
                crc = (crc >> 1) ^ 0x22F0
            else:
                crc >>= 1
    return crc ^ 0x3FFF


def bitpack(number: int) -> bytes:
    if number < 0:
        n = (-number << 1) | 1
    else:
        n = number << 1
    out = bytearray()
    for _ in range(5):
        out.append((n & 0x3F) | (0x40 if n >= 0x40 else 0x00))
        n >>= 6
        if n == 0:
            break
    return bytes(out)


def bitunpack(buf: bytes, idx: int) -> Tuple[int, int]:
    enc = 0
    shift = 0
    while idx < len(buf) and shift < 30:
        b = buf[idx]
        idx += 1
        enc |= (b & 0x3F) << shift
        shift += 6
        if not (b & 0x40):
            break
    value = -(enc >> 1) if (enc & 1) else (enc >> 1)
    return value, idx


def write_command(address: int, command: int, data: bytes) -> bytes:
    frame = bytearray([address & 0xFF, command & 0xFF, len(data)])
    frame.extend(data)
    crc = crc14(bytes(frame))
    frame.append(crc & 0x7F)
    frame.append((crc >> 7) & 0x7F)
    return bytes(frame)


def build_start(address: int, channel: str, home: bool = False) -> bytes:
    ch = ord(channel)
    out = write_command(address, KANGAROO_CMD_START, bytes([ch, 0]))
    if home:
        out += write_command(address, KANGAROO_CMD_HOME, bytes([ch, 0]))
    return out


def build_move_position(address: int, channel: str, position: int,
                        speed_limit: int = -1) -> bytes:
    data = bytearray([ord(channel), 0, KANGAROO_MOVE_POSITION])
    data += bitpack(position)
    if speed_limit >= 0:
        data.append(KANGAROO_MOVE_SPEED)
        data += bitpack(speed_limit)
    return write_command(address, KANGAROO_CMD_MOVE, bytes(data))


def build_move_speed(address: int, channel: str, speed: int) -> bytes:
    data = bytearray([ord(channel), 0, KANGAROO_MOVE_SPEED])
    data += bitpack(speed)
    return write_command(address, KANGAROO_CMD_MOVE, bytes(data))


def build_get(address: int, channel: str, param: int) -> bytes:
    return write_command(address, KANGAROO_CMD_STATUS, bytes([ord(channel), 0, param]))


def build_powerdown(address: int, channel: str) -> bytes:
    return write_command(address, KANGAROO_CMD_SYSTEM,
                         bytes([ord(channel), 0, KANGAROO_SYS_POWERDOWN]))


def parse_packet_reply(buf: bytes) -> Optional[Tuple[int, int, int]]:
    """Find and validate a command-67 reply in `buf`. Returns
    (flags, param, value) or None."""
    # Sync to an address byte (high bit set).
    start = 0
    while start < len(buf) and not (buf[start] & 0x80):
        start += 1
    if start + 3 > len(buf):
        return None
    length = buf[start + 2]
    end = start + 3 + length + 2
    if end > len(buf):
        return None
    if buf[start + 1] != KANGAROO_RC_STATUS:
        return None
    frame = buf[start:start + 3 + length]
    crc_lo = buf[start + 3 + length]
    crc_hi = buf[start + 3 + length + 1]
    want = crc14(frame)
    got = (crc_lo & 0x7F) | ((crc_hi & 0x7F) << 7)
    if want != got:
        return None

    data = buf[start + 3:start + 3 + length]
    idx = 1                       # skip channel
    if idx >= length:
        return None
    flags = data[idx]
    idx += 1
    if flags & KANGAROO_STATUS_ECHO_CODE:
        idx += 1
    if flags & KANGAROO_STATUS_SEQUENCE:
        idx += 1
    if idx >= length:
        return None
    param = data[idx]
    idx += 1
    value, _ = bitunpack(data, idx)
    return flags, param, value


def parse_simple_reply(line: str) -> Optional[Tuple[int, int]]:
    """Parse "<ch>,<L><number>" → (flags, value). The status letter L
    encodes done/busy/error; the caller knows which param it asked."""
    comma = line.find(",")
    if comma < 0 or comma + 1 >= len(line):
        return None
    letter = line[comma + 1]
    num = line[comma + 2:].strip()
    flags = 0
    if letter in ("E", "e"):
        flags |= KANGAROO_STATUS_ERROR
        if letter == "e":
            flags |= KANGAROO_STATUS_BUSY
    elif letter in ("p", "s"):
        flags |= KANGAROO_STATUS_BUSY
    elif letter not in ("P", "S"):
        return None
    try:
        value = int(num)
    except ValueError:
        value = 0
    return flags, value


class KangarooDriver(PeripheralDriver):
    TYPE_ID = "kangaroo"
    MODE_STRING = "kangaroo_motion"
    VIRTUAL_GPIO_BASE = KANGAROO_VIRTUAL_GPIO_BASE
    CHANNELS_PER_INSTANCE = KANGAROO_CHANNELS_PER_UNIT
    MAX_INSTANCES = KANGAROO_MAX_UNITS
    SUB_CHANNEL_NAMES = [
        "target_position", "target_speed",
        "current_position", "current_speed",
        "moving", "error_status",
    ]

    _transport_cls = UartTransport

    def __init__(self, logger=None):
        super().__init__(logger=logger)
        self._uart: Optional[UartTransport] = None
        self._uart_pins: Tuple[int, int, int] = (0, 0, KANGAROO_DEFAULT_BAUD)
        # instance_id -> config dict
        self._cfg: Dict[int, Dict[str, Any]] = {}
        self._started: Dict[int, bool] = {}
        self._poll_unit = 0
        self._poll_param = 0   # 0 = position, 1 = speed

    # ── PeripheralDriver overrides ────────────────────────────────

    def apply_config(self, instance_id: int, pins: Dict[str, int],
                     params: Dict[str, Any]) -> bool:
        if instance_id >= KANGAROO_MAX_UNITS:
            self._log("warn",
                f"Kangaroo: instance_id {instance_id} exceeds max "
                f"{KANGAROO_MAX_UNITS}")
            return False

        address = int(params.get("address", KANGAROO_DEFAULT_ADDRESS))
        if not (KANGAROO_ADDRESS_MIN <= address <= KANGAROO_ADDRESS_MAX):
            self._log("warn",
                f"Kangaroo#{instance_id}: address {address} outside "
                f"{KANGAROO_ADDRESS_MIN}..{KANGAROO_ADDRESS_MAX}")
            return False

        channel = str(params.get("channel", "1"))[:1] or "1"
        protocol = str(params.get("protocol", "packet")).lower()
        simple = protocol.startswith("s")
        home = bool(params.get("home_on_start", False))
        max_pos = int(params.get("max_position", KANGAROO_DEFAULT_MAX_POSITION))
        max_spd = int(params.get("max_speed", KANGAROO_DEFAULT_MAX_SPEED))

        tx = int(pins.get("uart_tx", 0))
        rx = int(pins.get("uart_rx", 0))
        baud = int(params.get("baud", KANGAROO_DEFAULT_BAUD))

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
        self._cfg[instance_id] = {
            "address": address, "channel": channel, "simple": simple,
            "home": home, "max_pos": max_pos, "max_spd": max_spd,
        }
        self._started[instance_id] = False

        # Start the channel so the first setpoint takes; probe it.
        self._send_start(instance_id)
        if self._query(instance_id, KANGAROO_GET_POSITION) is not None:
            inst.connected = True
        return True

    def update(self) -> None:
        """Round-robin one Get per tick, alternating position/speed,
        and refresh moving / error_status from the reply flags."""
        if not self._cfg:
            return
        ids = sorted(self._cfg)
        if self._poll_unit >= len(ids):
            self._poll_unit = 0
        inst_id = ids[self._poll_unit]
        param = KANGAROO_GET_POSITION if self._poll_param == 0 else KANGAROO_GET_SPEED
        reply = self._query(inst_id, param)
        inst = self._instances.get(inst_id)
        if reply is not None and inst is not None:
            flags, value = reply
            inst.connected = True
            inst.last_values[KANGAROO_SUB_MOVING] = \
                1.0 if (flags & KANGAROO_STATUS_BUSY) else 0.0
            if flags & KANGAROO_STATUS_ERROR:
                inst.last_values[KANGAROO_SUB_ERROR_STATUS] = float(value)
                if value == KANGAROO_ERR_NOT_STARTED:
                    self._started[inst_id] = False
                    self._send_start(inst_id)
            else:
                inst.last_values[KANGAROO_SUB_ERROR_STATUS] = 0.0
                if param == KANGAROO_GET_POSITION:
                    inst.last_values[KANGAROO_SUB_CURRENT_POSITION] = float(value)
                else:
                    inst.last_values[KANGAROO_SUB_CURRENT_SPEED] = float(value)

        self._poll_param ^= 1
        if self._poll_param == 0:
            self._poll_unit = (self._poll_unit + 1) % len(ids)

    def set_value(self, instance_id: int, sub_channel: int, value: float) -> bool:
        if instance_id not in self._cfg or self._uart is None:
            return False
        cfg = self._cfg[instance_id]
        value = max(-1.0, min(1.0, value))
        if not self._started.get(instance_id):
            self._send_start(instance_id)

        if sub_channel == KANGAROO_SUB_TARGET_POSITION:
            position = int(round(value * cfg["max_pos"]))
            limit = cfg["max_spd"] if cfg["max_spd"] > 0 else -1
            frame = self._build_position(cfg, position, limit)
        elif sub_channel == KANGAROO_SUB_TARGET_SPEED:
            speed = int(round(value * cfg["max_spd"]))
            frame = self._build_speed(cfg, speed)
        else:
            return False  # read-only sub-channels

        if not self._uart.write(frame):
            return False
        inst = self._instances[instance_id]
        inst.last_values[sub_channel] = value
        return True

    def get_value(self, instance_id: int, sub_channel: int) -> Optional[float]:
        inst = self._instances.get(instance_id)
        if inst is None:
            return None
        return inst.last_values.get(sub_channel)

    def estop(self) -> None:
        if self._uart is None:
            return
        for inst_id, cfg in self._cfg.items():
            self._uart.write(self._build_speed(cfg, 0))
            if cfg["simple"]:
                self._uart.write(f"{cfg['channel']},powerdown\n".encode("ascii"))
            else:
                self._uart.write(build_powerdown(cfg["address"], cfg["channel"]))
            self._started[inst_id] = False
        self._log("warn",
            f"Kangaroo: ESTOP — speed 0 + power down on {len(self._cfg)} channel(s)")

    def reset(self) -> None:
        if self._uart is not None:
            self._uart.close()
            self._uart = None
        self._uart_pins = (0, 0, KANGAROO_DEFAULT_BAUD)
        self._cfg.clear()
        self._started.clear()
        self._poll_unit = 0
        self._poll_param = 0
        super().reset()

    # ── helpers ───────────────────────────────────────────────────

    def _build_position(self, cfg: Dict[str, Any], position: int, limit: int) -> bytes:
        if cfg["simple"]:
            if limit >= 0:
                return f"{cfg['channel']},p{position} s{limit}\n".encode("ascii")
            return f"{cfg['channel']},p{position}\n".encode("ascii")
        return build_move_position(cfg["address"], cfg["channel"], position, limit)

    def _build_speed(self, cfg: Dict[str, Any], speed: int) -> bytes:
        if cfg["simple"]:
            return f"{cfg['channel']},s{speed}\n".encode("ascii")
        return build_move_speed(cfg["address"], cfg["channel"], speed)

    def _send_start(self, instance_id: int) -> None:
        if self._uart is None or instance_id not in self._cfg:
            return
        cfg = self._cfg[instance_id]
        if cfg["simple"]:
            self._uart.write(f"{cfg['channel']},start\n".encode("ascii"))
            if cfg["home"]:
                self._uart.write(f"{cfg['channel']},home\n".encode("ascii"))
        else:
            self._uart.write(build_start(cfg["address"], cfg["channel"], cfg["home"]))
        self._started[instance_id] = True

    def _query(self, instance_id: int, param: int) -> Optional[Tuple[int, int]]:
        """Issue a Get and parse the reply. Returns (flags, value) or None."""
        if self._uart is None or instance_id not in self._cfg:
            return None
        cfg = self._cfg[instance_id]
        self._uart.drain()
        if cfg["simple"]:
            q = "gets" if param == KANGAROO_GET_SPEED else "getp"
            self._uart.write(f"{cfg['channel']},{q}\n".encode("ascii"))
            raw = self._uart.read(24)
            if not raw:
                return None
            try:
                line = raw.decode("ascii", errors="ignore")
            except Exception:
                return None
            return parse_simple_reply(line)
        self._uart.write(build_get(cfg["address"], cfg["channel"], param))
        raw = self._uart.read(16)
        if not raw:
            return None
        result = parse_packet_reply(raw)
        if result is None:
            return None
        flags, _param, value = result
        return flags, value
