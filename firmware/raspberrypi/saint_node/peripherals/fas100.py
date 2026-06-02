"""
SAINT.OS Pi node — FrSky FAS100 ADV current/voltage/temperature sensor.

Port of firmware/shared/src/fas100_driver.c. The FAS100 speaks two
protocols and decides which one at power-on based on what it sees on
the wire:

    S.Port:  57600 baud, 0x7E start-of-poll, 0x7D byte-stuffing,
             8-byte response frames.
    FBUS:    460800 baud, no byte stuffing, fixed 10-byte response
             frames with an explicit length byte.

Operators can't pick the mode — we auto-detect by alternating baud +
poll style between probes (~1.5 s each) and locking to whichever
responds. Same sensor IDs / data IDs / units in both modes, so channel
storage is shared.

Inversion: the FAS100 uses inverted half-duplex UART (idle LOW). The
RP2040 and Teensy drivers do this in the chip pad. On the Pi the
hardware UART doesn't expose inversion — production deployments use a
hardware inverter (74HC04 or equivalent) between the Pi UART pin and
the FAS100 signal wire, OR an FT232H USB-serial adapter with EEPROM-
configured TXINV/RXINV. From the driver's point of view the Pi UART
sees plain non-inverted bytes either way. Without an inverter on the
line the driver will simply fail to lock (no bytes match the parsers).

Half-duplex wiring loops TX → 1 kΩ → RX, so every byte we transmit
echoes into our own RX FIFO. The driver drains the echo synchronously
after each poll via write_then_read(drain_echo=True).
"""

from __future__ import annotations

import time
from typing import Any, Dict, Optional

from .base import PeripheralDriver
from ..uart_transport import UartTransport


# ── Protocol constants ───────────────────────────────────────────

SPORT_BAUD                   = 57600
FBUS_BAUD                    = 460800
SPORT_POLL_HEADER            = 0x7E
SPORT_DATA_HEADER            = 0x10
SPORT_EMPTY_HEADER           = 0x00
SPORT_STUFF_MARKER           = 0x7D
SPORT_STUFF_MASK             = 0x20
SPORT_FAS100_PHYSICAL_ID     = 0x22
SPORT_DATA_ID_CURRENT        = 0x0200
SPORT_DATA_ID_VOLTAGE        = 0x0210
SPORT_DATA_ID_TEMP1          = 0x0400
SPORT_DATA_ID_TEMP2          = 0x0410
SPORT_RESPONSE_FRAME_SIZE    = 8

FBUS_LEN_BYTE                = 0x08
FBUS_FRAME_ID_DATA           = 0x10
FBUS_FAS100_SENSOR_ID        = SPORT_FAS100_PHYSICAL_ID
FBUS_RESPONSE_FRAME_SIZE     = 10

# Auto-detect timing — same as firmware.
PROBE_S                      = 1.5
LOST_LINK_S                  = 4.0
RX_QUIET_S                   = 8.0
STICKY_CONNECTED_S           = 12.0
CONSEC_BAD_THRESH            = 16
DEFAULT_POLL_INTERVAL_MS     = 50

# Virtual GPIO layout (matches firmware sport_protocol.h).
FAS100_VIRTUAL_GPIO_BASE     = 232
FAS100_CHANNEL_COUNT         = 4
FAS100_CH_CURRENT            = 0
FAS100_CH_VOLTAGE            = 1
FAS100_CH_TEMP1              = 2
FAS100_CH_TEMP2              = 3


def sport_crc(data: bytes) -> int:
    """S.Port / FBUS share the same CRC. Sum-with-carry-fold, ones'
    complement at the end."""
    crc = 0
    for b in data:
        crc += b
        crc = (crc & 0xFF) + (crc >> 8)
    return 0xFF - (crc & 0xFF)


def _destuff(raw: int, in_stuff: bool) -> tuple[Optional[int], bool]:
    """S.Port byte-stuffing de-escape. Returns (decoded_byte_or_None,
    new_in_stuff_flag). If `raw` is the escape marker, returns (None,
    True) so the caller waits for the next byte."""
    if in_stuff:
        return (raw ^ SPORT_STUFF_MASK, False)
    if raw == SPORT_STUFF_MARKER:
        return (None, True)
    return (raw, False)


# ── Driver ───────────────────────────────────────────────────────


class FAS100Driver(PeripheralDriver):
    """FAS100 ADV current/voltage/temperature sensor. Single instance;
    4 read-only sub-channels (current, voltage, temp1, temp2)."""

    TYPE_ID = "fas100"
    MODE_STRING = "fas100_sensor"
    VIRTUAL_GPIO_BASE = FAS100_VIRTUAL_GPIO_BASE
    CHANNELS_PER_INSTANCE = FAS100_CHANNEL_COUNT
    MAX_INSTANCES = 1
    SUB_CHANNEL_NAMES = ["current", "voltage", "temp1", "temp2"]

    _transport_cls = UartTransport

    # Phase enum values — kept as ints so test assertions can use them
    # without exporting the enum.
    PHASE_PROBE_SPORT = 0
    PHASE_PROBE_FBUS  = 1
    PHASE_LOCKED      = 2

    def __init__(self, logger=None):
        super().__init__(logger=logger)
        self._uart: Optional[UartTransport] = None
        self._uart_pins: tuple[int, int] = (0, 0)
        self._poll_interval_s: float = DEFAULT_POLL_INTERVAL_MS / 1000.0
        self._last_poll_t: float = 0.0

        # Protocol state
        self._active_proto: int = 0   # 0=SPORT, 1=FBUS
        self._phase: int = self.PHASE_PROBE_SPORT
        self._phase_started_t: float = 0.0
        self._last_response_t: float = 0.0
        self._last_byte_t: float = 0.0
        self._last_soft_resync_t: float = 0.0

        # Frame accumulator
        self._rx: bytearray = bytearray()
        self._in_stuff: bool = False
        self._skip_addr: bool = False  # S.Port: drop the byte after 0x7E

        # Diagnostic streaks for soft-resync trigger.
        self._consecutive_bad: int = 0

        # Last decoded values + sticky-connected timestamp.
        self._values: Dict[int, float] = {}
        self._sensor_responded: bool = False

    # ── PeripheralDriver overrides ────────────────────────────────

    def apply_config(self, instance_id: int, pins: Dict[str, int],
                     params: Dict[str, Any]) -> bool:
        if instance_id != 0:
            return False
        tx = int(pins.get("uart_tx", 0))
        rx = int(pins.get("uart_rx", 0))
        poll_ms = int(params.get("poll_interval_ms", DEFAULT_POLL_INTERVAL_MS))
        self._poll_interval_s = max(0.005, poll_ms / 1000.0)

        if self._uart is None or self._uart_pins != (tx, rx):
            if self._uart is not None:
                self._uart.close()
            # S.Port baud first; auto-detect may switch to FBUS later.
            self._uart = self._transport_cls(
                tx, rx, SPORT_BAUD, timeout_s=0.02, logger=self._logger)
            if not self._uart.open():
                return False
            self._uart_pins = (tx, rx)

        inst = self._get_or_create_instance(0)
        inst.pins = dict(pins)
        inst.params = dict(params)

        # Start fresh in S.Port probe phase.
        self._enter_phase(self.PHASE_PROBE_SPORT, proto=0)
        self._log("info",
            f"FAS100: configured TX={tx} RX={rx} — auto-detecting "
            f"protocol (S.Port {SPORT_BAUD} / FBUS {FBUS_BAUD}, "
            f"{int(PROBE_S * 1000)} ms probe). Wire inversion is "
            f"expected to be handled upstream of the Pi UART.")
        return True

    def update(self) -> None:
        if self._uart is None:
            return
        now = time.monotonic()

        # 1) Pump RX through the active protocol parser.
        self._pump_rx(now)

        # 2) State-machine transitions.
        self._tick_state_machine(now)

        # 3) Poll cadence.
        if now - self._last_poll_t >= self._poll_interval_s:
            self._last_poll_t = now
            self._send_poll()

    def set_value(self, instance_id: int, sub_channel: int, value: float) -> bool:
        return False  # sensor is read-only

    def get_value(self, instance_id: int, sub_channel: int) -> Optional[float]:
        if instance_id != 0:
            return None
        return self._values.get(sub_channel)

    def reset(self) -> None:
        if self._uart is not None:
            self._uart.close()
            self._uart = None
        self._uart_pins = (0, 0)
        self._values.clear()
        self._sensor_responded = False
        self._last_response_t = 0.0
        self._rx.clear()
        self._in_stuff = False
        self._skip_addr = False
        self._consecutive_bad = 0
        super().reset()

    # PeripheralManager.collect_states reads driver._instances[i]
    # .connected to surface the link state. Mirror that here based on
    # the sticky-connected window.
    @property
    def _is_link_alive(self) -> bool:
        if self._sensor_responded:
            return True
        if self._last_response_t == 0.0:
            return False
        return (time.monotonic() - self._last_response_t) < STICKY_CONNECTED_S

    # ── Internal helpers ──────────────────────────────────────────

    def _enter_phase(self, new_phase: int, proto: int) -> None:
        if new_phase != self.PHASE_LOCKED:
            self._sensor_responded = False
        if proto != self._active_proto:
            self._active_proto = proto
            if self._uart is not None:
                self._uart.set_baud(FBUS_BAUD if proto == 1 else SPORT_BAUD)
                self._uart.drain()
        self._rx.clear()
        self._in_stuff = False
        self._skip_addr = False
        self._consecutive_bad = 0
        self._phase = new_phase
        self._phase_started_t = time.monotonic()
        if new_phase == self.PHASE_PROBE_SPORT:
            self._log("info", f"FAS100: probing S.Port @ {SPORT_BAUD} baud")
        elif new_phase == self.PHASE_PROBE_FBUS:
            self._log("info", f"FAS100: probing FBUS @ {FBUS_BAUD} baud")

    def _send_poll(self) -> None:
        if self._uart is None:
            return
        if self._active_proto == 0:
            poll = bytes([SPORT_POLL_HEADER, SPORT_FAS100_PHYSICAL_ID])
        else:
            poll = bytes([FBUS_LEN_BYTE, FBUS_FAS100_SENSOR_ID, FBUS_FRAME_ID_DATA])
        # Drain echo bytes synchronously so the parser doesn't see our
        # own transmit looping back through the half-duplex resistor.
        self._uart.write_then_read(poll, 0, drain_echo=True)

    def _pump_rx(self, now: float) -> None:
        if self._uart is None:
            return
        data = self._uart.read(64)
        if not data:
            return
        self._last_byte_t = now
        if self._active_proto == 0:
            for b in data:
                self._sport_feed_byte(b, now)
        else:
            for b in data:
                self._fbus_feed_byte(b, now)

    def _sport_feed_byte(self, raw: int, now: float) -> None:
        # 0x7E hard-syncs the parser: it only appears unescaped at the
        # start of a poll, so seeing one means we're at a known
        # boundary. The next byte is the physical-id of the polled
        # sensor — we drop it and start collecting on the byte after.
        if raw == SPORT_POLL_HEADER:
            self._rx.clear()
            self._in_stuff = False
            self._skip_addr = True
            return
        if self._skip_addr:
            self._skip_addr = False
            return

        decoded, self._in_stuff = _destuff(raw, self._in_stuff)
        if decoded is None:
            return
        self._rx.append(decoded)
        if len(self._rx) < SPORT_RESPONSE_FRAME_SIZE:
            return

        header = self._rx[0]
        if header in (SPORT_DATA_HEADER, SPORT_EMPTY_HEADER):
            expected_crc = sport_crc(bytes(self._rx[:7]))
            if self._rx[7] == expected_crc:
                if header == SPORT_DATA_HEADER:
                    data_id = self._rx[1] | (self._rx[2] << 8)
                    value = (self._rx[3]
                             | (self._rx[4] << 8)
                             | (self._rx[5] << 16)
                             | (self._rx[6] << 24))
                    self._store_telemetry(data_id, value, now)
                    self._consecutive_bad = 0
                else:
                    # Empty-header heartbeat: sensor is alive but had
                    # nothing new. Refresh last-response so the
                    # liveness check doesn't trip a re-probe.
                    self._sensor_responded = True
                    self._last_response_t = now
                self._rx.clear()
                self._in_stuff = False
                return
        # CRC mismatch / unknown header — shift one byte and retry.
        self._consecutive_bad += 1
        del self._rx[0]

    def _fbus_feed_byte(self, raw: int, now: float) -> None:
        if not self._rx:
            if raw == FBUS_LEN_BYTE:
                self._rx.append(raw)
            return
        if len(self._rx) == 1:
            if raw == FBUS_FAS100_SENSOR_ID:
                self._rx.append(raw)
            elif raw == FBUS_LEN_BYTE:
                self._rx[:] = [raw]
            else:
                self._rx.clear()
            return
        if len(self._rx) == 2:
            if raw == FBUS_FRAME_ID_DATA:
                self._rx.append(raw)
            elif raw == FBUS_LEN_BYTE:
                self._rx[:] = [raw]
            else:
                self._rx.clear()
            return
        self._rx.append(raw)
        if len(self._rx) < FBUS_RESPONSE_FRAME_SIZE:
            return

        expected_crc = sport_crc(bytes(self._rx[:9]))
        if self._rx[9] == expected_crc:
            data_id = self._rx[3] | (self._rx[4] << 8)
            value = (self._rx[5]
                     | (self._rx[6] << 8)
                     | (self._rx[7] << 16)
                     | (self._rx[8] << 24))
            self._store_telemetry(data_id, value, now)
            self._consecutive_bad = 0
        else:
            self._consecutive_bad += 1
        self._rx.clear()

    def _store_telemetry(self, data_id: int, raw_value: int, now: float) -> None:
        was_disconnected = not self._sensor_responded
        self._sensor_responded = True
        self._last_response_t = now
        # Sign-extend the value for temperature fields.
        signed = raw_value
        if signed & 0x80000000:
            signed -= 0x100000000

        if data_id == SPORT_DATA_ID_CURRENT:
            self._values[FAS100_CH_CURRENT] = raw_value / 10.0
        elif data_id == SPORT_DATA_ID_VOLTAGE:
            self._values[FAS100_CH_VOLTAGE] = raw_value / 100.0
        elif data_id == SPORT_DATA_ID_TEMP1:
            self._values[FAS100_CH_TEMP1] = float(signed)
        elif data_id == SPORT_DATA_ID_TEMP2:
            self._values[FAS100_CH_TEMP2] = float(signed)
        else:
            # Known frame format but unknown data_id — keep
            # sensor_responded set (link is alive) but treat as a bad-
            # alignment hint.
            self._consecutive_bad += 1
            return

        inst = self._instances.get(0)
        if inst is not None:
            inst.connected = True
            inst.last_values.update(self._values)
        if was_disconnected:
            self._log("info",
                f"FAS100: first {'FBUS' if self._active_proto else 'S.Port'} "
                f"response (data_id=0x{data_id:04x})")

    def _tick_state_machine(self, now: float) -> None:
        # Probe phases: lock if we got a response; switch protocols
        # after PROBE_S without one.
        if self._phase == self.PHASE_PROBE_SPORT:
            if self._sensor_responded:
                self._phase = self.PHASE_LOCKED
                self._log("info", f"FAS100: locked to S.Port @ {SPORT_BAUD}")
            elif now - self._phase_started_t >= PROBE_S:
                self._enter_phase(self.PHASE_PROBE_FBUS, proto=1)
            return
        if self._phase == self.PHASE_PROBE_FBUS:
            if self._sensor_responded:
                self._phase = self.PHASE_LOCKED
                self._log("info", f"FAS100: locked to FBUS @ {FBUS_BAUD}")
            elif now - self._phase_started_t >= PROBE_S:
                self._enter_phase(self.PHASE_PROBE_SPORT, proto=0)
            return

        # Locked: distinguish "bus silent" (switch protocols) from
        # "bytes flowing but CRC-failing" (soft-resync).
        if self._last_response_t == 0.0:
            return
        valid_silence = now - self._last_response_t
        if self._last_byte_t == 0.0:
            byte_silence = valid_silence
        else:
            byte_silence = now - self._last_byte_t

        if byte_silence >= RX_QUIET_S:
            other = 1 - self._active_proto
            target_phase = self.PHASE_PROBE_FBUS if other == 1 else self.PHASE_PROBE_SPORT
            self._log("warn",
                f"FAS100: bus silent for {byte_silence:.1f}s — "
                f"re-probing {'FBUS' if other else 'S.Port'}")
            self._enter_phase(target_phase, proto=other)
            return

        slow_resync_ready = (valid_silence >= LOST_LINK_S
                             and (now - self._last_soft_resync_t) >= LOST_LINK_S)
        fast_resync_ready = self._consecutive_bad >= CONSEC_BAD_THRESH
        if slow_resync_ready or fast_resync_ready:
            if self._uart is not None:
                self._uart.drain()
            self._rx.clear()
            self._in_stuff = False
            self._skip_addr = False
            bad_at_trigger = self._consecutive_bad
            self._consecutive_bad = 0
            self._last_soft_resync_t = now
            kind = "fast" if fast_resync_ready else "slow"
            self._log("info",
                f"FAS100: {kind} soft-resync "
                f"({'FBUS' if self._active_proto else 'S.Port'}, "
                f"bad_streak={bad_at_trigger}, "
                f"valid_silence={valid_silence:.1f}s)")
