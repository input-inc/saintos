"""
SAINT.OS Pi node — Pathfinder / JBD-compatible BMS driver.

Originally a port of firmware/shared/src/pathfinder_bms_driver.c. JBD
protocol over UART (9600 baud, 8N1) OR over BLE (JBD service
0x0000FF00, write 0xFF02, notify 0xFF01) — same frame format either
way:

    [0xDD] [Action] [Register] [Length] [Data...] [CRC_hi] [CRC_lo] [0x77]

We read register 0x03 (basic info) periodically: pack voltage,
current, SOC, cycles, protection bits, NTC temps. Operator-facing
sub-channels are a subset of those.

Multi-instance — up to MAX_INSTANCES BMSes per Pi (mixed transports
allowed, e.g. one UART + two BLE). Each instance's `params["transport"]`
selects "uart" (default, backwards-compatible) or "ble". Channels:

  0  pack_voltage  (V)
  1  current       (A)
  2  soc           (%)
  3  remain_cap    (Ah)
  4  temp_1        (°C)
  5  temp_2        (°C)
  6  cycles        (count)
  7  protection    (uint16 bitset)
  8..23 cell_NN    (V each, up to 16 cells — reserved, not yet decoded)

Virtual GPIO base is 1024 on the Pi (Pi-side only). The
firmware-shared header still says 276 because RP2040/Teensy only run
one BMS each. On the Pi we need MAX_INSTANCES × 24 = 96 channels and
the 276..371 range would collide with Tic (300+) and TMC2208 (348+).
Server-side addressing is `(peripheral_id, channel_id)` per the
peripheral-first refactor — the vGPIO number is internal routing only.
"""

from __future__ import annotations

import time
from typing import Any, Dict, Optional

from .base import PeripheralDriver
from ..ble_transport import BleTransport
from ..uart_transport import UartTransport


JBD_BAUD                       = 9600
JBD_FRAME_START                = 0xDD
JBD_FRAME_END                  = 0x77
JBD_ACTION_READ                = 0xA5
JBD_ACTION_OK                  = 0x00
JBD_REG_BASIC_INFO             = 0x03
JBD_VIRTUAL_GPIO_BASE          = 1024
JBD_CHANNEL_COUNT              = 24
JBD_MAX_INSTANCES              = 4
JBD_DEFAULT_POLL_S             = 1.0
JBD_RESPONSE_MAX_DATA          = 64
JBD_READ_REQUEST_SIZE          = 7
JBD_UART_TIMEOUT_S             = 0.2
JBD_BLE_TIMEOUT_S              = 2.0

# Channel indexes
JBD_CH_PACK_VOLTAGE     = 0
JBD_CH_CURRENT          = 1
JBD_CH_SOC              = 2
JBD_CH_REMAIN_CAP       = 3
JBD_CH_TEMP_1           = 4
JBD_CH_TEMP_2           = 5
JBD_CH_CYCLES           = 6
JBD_CH_PROTECTION       = 7
JBD_CH_CELL_BASE        = 8

_SUB_CHANNEL_NAMES = (
    ["pack_voltage", "current", "soc", "remain_cap",
     "temp_1", "temp_2", "cycles", "protection"]
    + [f"cell_{i+1:02d}" for i in range(16)]
)


def jbd_checksum(data: bytes) -> int:
    """JBD checksum: 0 minus each byte, kept in a 16-bit window."""
    s = 0
    for b in data:
        s = (s - b) & 0xFFFF
    return s


def build_read_request(reg: int) -> bytes:
    """7-byte read request frame."""
    payload = bytes([reg, 0x00])
    c = jbd_checksum(payload)
    return bytes([JBD_FRAME_START, JBD_ACTION_READ]) \
         + payload \
         + bytes([(c >> 8) & 0xFF, c & 0xFF, JBD_FRAME_END])


def parse_basic_info_response(frame: bytes) -> Optional[Dict[str, float]]:
    """Validate a JBD basic-info response and return a dict of decoded
    fields. Returns None on framing / checksum / length errors."""
    if len(frame) < 4 + 3:
        return None
    if frame[0] != JBD_FRAME_START or frame[-1] != JBD_FRAME_END:
        return None
    if frame[1] != JBD_ACTION_OK:
        return None
    reg = frame[2]
    data_len = frame[3]
    if data_len < 23 or 4 + data_len + 3 > len(frame):
        return None
    data = frame[4: 4 + data_len]
    # Checksum covers [register, length, data]
    expected = jbd_checksum(bytes([reg, data_len]) + data)
    received = (frame[4 + data_len] << 8) | frame[4 + data_len + 1]
    if expected != received:
        return None

    # Decode the basic-info fields.
    def i16(off):
        v = (data[off] << 8) | data[off + 1]
        if v & 0x8000:
            v -= 0x10000
        return v

    def u16(off):
        return (data[off] << 8) | data[off + 1]

    pack_v = u16(0) * 0.01            # bytes 0-1, 10mV units
    current = i16(2) * 0.01           # bytes 2-3, signed 10mA
    remain  = u16(4) * 0.01           # bytes 4-5, 10mAh
    full    = u16(6) * 0.01           # bytes 6-7
    cycles  = u16(8)                  # bytes 8-9
    protection = u16(16)              # bytes 16-17
    soc     = data[19]                # byte 19, 0-100
    ntc_count = data[22] if len(data) > 22 else 0

    # NTC values start at byte 23, 2 bytes each, units of 0.1 K
    # (kelvin × 10). Convert to °C.
    temps = []
    base = 23
    for i in range(min(ntc_count, 4)):
        off = base + i * 2
        if off + 1 >= len(data):
            break
        raw = u16(off)
        celsius = (raw - 2731) / 10.0
        temps.append(celsius)

    return {
        "pack_voltage": pack_v,
        "current": current,
        "remain_cap": remain,
        "full_cap": full,
        "cycles": float(cycles),
        "soc": float(soc),
        "protection": float(protection),
        "temps": temps,
    }


class PathfinderBMSDriver(PeripheralDriver):
    TYPE_ID = "pathfinder_bms"
    MODE_STRING = "pathfinder_bms_sensor"
    VIRTUAL_GPIO_BASE = JBD_VIRTUAL_GPIO_BASE
    CHANNELS_PER_INSTANCE = JBD_CHANNEL_COUNT
    MAX_INSTANCES = JBD_MAX_INSTANCES
    SUB_CHANNEL_NAMES = _SUB_CHANNEL_NAMES

    # Transport factories — tests monkey-patch these to inject mocks.
    _uart_transport_cls = UartTransport
    _ble_transport_cls = BleTransport

    def __init__(self, logger=None):
        super().__init__(logger=logger)
        self._transports: Dict[int, Any] = {}
        self._poll_interval_s: Dict[int, float] = {}
        self._last_poll: Dict[int, float] = {}

    def apply_config(self, instance_id: int, pins: Dict[str, int],
                     params: Dict[str, Any]) -> bool:
        if instance_id < 0 or instance_id >= self.MAX_INSTANCES:
            return False
        transport_type = str(params.get("transport", "uart")).lower()
        poll_ms = int(params.get("poll_interval_ms",
                                  int(JBD_DEFAULT_POLL_S * 1000)))
        poll_s = max(0.1, poll_ms / 1000.0)

        # Drop any prior transport on this slot before swapping.
        self._teardown_instance(instance_id)

        transport = self._build_transport(
            instance_id, transport_type, pins, params)
        if transport is None:
            return False
        if not transport.open():
            try:
                transport.close()
            except Exception:
                pass
            return False

        self._transports[instance_id] = transport
        self._poll_interval_s[instance_id] = poll_s
        self._last_poll[instance_id] = 0.0

        inst = self._get_or_create_instance(instance_id)
        inst.pins = dict(pins)
        inst.params = dict(params)

        # Best-effort first poll. UART usually succeeds immediately;
        # BLE returns False until the async connect lands — update()
        # will keep retrying every poll interval.
        if self._poll_basic_info(instance_id):
            self._last_poll[instance_id] = time.monotonic()
        return True

    def update(self) -> None:
        now = time.monotonic()
        for instance_id in list(self._transports.keys()):
            interval = self._poll_interval_s.get(
                instance_id, JBD_DEFAULT_POLL_S)
            if now - self._last_poll.get(instance_id, 0.0) < interval:
                continue
            self._last_poll[instance_id] = now
            self._poll_basic_info(instance_id)

    def set_value(self, instance_id: int, sub_channel: int,
                  value: float) -> bool:
        return False  # read-only

    def get_value(self, instance_id: int, sub_channel: int) -> Optional[float]:
        inst = self._instances.get(instance_id)
        if inst is None:
            return None
        return inst.last_values.get(sub_channel)

    def reset(self) -> None:
        for instance_id in list(self._transports.keys()):
            self._teardown_instance(instance_id)
        super().reset()

    # ── internals ──────────────────────────────────────────────────

    def _build_transport(self, instance_id: int, transport_type: str,
                         pins: Dict[str, int],
                         params: Dict[str, Any]):
        if transport_type == "uart":
            tx = int(pins.get("uart_tx", 0))
            rx = int(pins.get("uart_rx", 0))
            baud = int(params.get("baud", JBD_BAUD))
            return self._uart_transport_cls(
                tx, rx, baud,
                timeout_s=JBD_UART_TIMEOUT_S, logger=self._logger)
        if transport_type == "ble":
            mac = str(params.get("mac", "")).strip()
            if not mac:
                self._log("error",
                    f"pathfinder_bms#{instance_id}: BLE transport "
                    f"requires 'mac' param")
                return None
            return self._ble_transport_cls(
                mac,
                timeout_s=JBD_BLE_TIMEOUT_S, logger=self._logger)
        self._log("error",
            f"pathfinder_bms#{instance_id}: unknown transport "
            f"'{transport_type}' (expected 'uart' or 'ble')")
        return None

    def _teardown_instance(self, instance_id: int) -> None:
        old = self._transports.pop(instance_id, None)
        self._poll_interval_s.pop(instance_id, None)
        self._last_poll.pop(instance_id, None)
        if old is None:
            return
        try:
            old.close()
        except Exception:
            pass

    def _poll_basic_info(self, instance_id: int) -> bool:
        transport = self._transports.get(instance_id)
        inst = self._instances.get(instance_id)
        if transport is None or inst is None:
            return False
        if not transport.is_open():
            # Normal for BLE before the async connect completes.
            inst.connected = False
            return False
        req = build_read_request(JBD_REG_BASIC_INFO)
        reply = transport.write_then_read(
            req, JBD_RESPONSE_MAX_DATA, drain_echo=False)
        decoded = parse_basic_info_response(reply)
        if decoded is None:
            inst.connected = False
            return False
        inst.connected = True
        inst.last_values[JBD_CH_PACK_VOLTAGE] = decoded["pack_voltage"]
        inst.last_values[JBD_CH_CURRENT]      = decoded["current"]
        inst.last_values[JBD_CH_SOC]          = decoded["soc"]
        inst.last_values[JBD_CH_REMAIN_CAP]   = decoded["remain_cap"]
        inst.last_values[JBD_CH_CYCLES]       = decoded["cycles"]
        inst.last_values[JBD_CH_PROTECTION]   = decoded["protection"]
        temps = decoded["temps"]
        if len(temps) >= 1:
            inst.last_values[JBD_CH_TEMP_1] = temps[0]
        if len(temps) >= 2:
            inst.last_values[JBD_CH_TEMP_2] = temps[1]
        return True
