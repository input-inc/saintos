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


# Pololu USB vendor request codes — see docs/MAESTRO_BRINGUP.md
# and Pololu/Maestro/protocol.h from pololu-usb-sdk on GitHub.
_REQ_GET_PARAMETER     = 0x81
_REQ_SET_PARAMETER     = 0x82
_REQ_GET_VARIABLES     = 0x83
_REQ_SET_SERVO_VARIABLE = 0x84
_REQ_SET_TARGET        = 0x85
_REQ_CLEAR_ERRORS      = 0x86
_REQ_SET_SCRIPT_DONE   = 0xA2


class _PyUsbVendorBackend:
    """Drop-in replacement for UartTransport that translates outgoing
    Pololu-protocol byte stream into USB vendor control transfers on
    the Maestro's endpoint 0.

    Same shape as UartTransport (open/close/write/read/is_open/
    device_path) so MaestroDriver doesn't need a different code path.
    The extra capability — `read_parameter` — is what enables EEPROM
    readback for the channel-edit UI's "(from Maestro)" defaults.

    Why parse the byte stream instead of taking high-level commands:
    keeps the upstream MaestroDriver unchanged. The frames it builds
    (0xAA device cmd [payload]) are simple enough to decode here.

    Vendor request mapping:
      Set Target (cmd 0x04) → REQUEST_SET_TARGET (0x85),
          wValue=target_qus, wIndex=channel
      Set Speed  (cmd 0x07) → REQUEST_SET_SERVO_VARIABLE (0x84),
          wValue=speed,       wIndex=channel
      Set Accel  (cmd 0x09) → REQUEST_SET_SERVO_VARIABLE (0x84),
          wValue=accel,       wIndex=channel | 0x80
      Go Home   (cmd 0x22)  → REQUEST_SET_SCRIPT_DONE (0xA2), value=0,
          (Compact-Protocol Go Home doesn't have a direct EP0 vendor
          equivalent; SET_SCRIPT_DONE=0 triggers script restart which
          on the Maestro re-applies HOME positions.)

    Open args mirror UartTransport so the driver-side constructor is
    polymorphic — the tx_pin / rx_pin / baud are unused on USB and
    ignored.
    """
    def __init__(self, tx_pin: int = 0, rx_pin: int = 0, baud: int = 0,
                 logger=None, vid: int = 0x1FFB, pid: int = 0x008C):
        self._logger = logger
        self._vid = vid
        self._pid = pid
        self._dev = None
        self._frame_buf = bytearray()
        self._device_num = 0   # tracked so we can validate frames

    def open(self) -> bool:
        try:
            import usb.core
        except ImportError:
            if self._logger:
                self._logger.warn("Maestro: pyusb not installed — usb_vendor transport unavailable")
            return False
        self._dev = usb.core.find(idVendor=self._vid, idProduct=self._pid)
        if self._dev is None:
            if self._logger:
                self._logger.warn(f"Maestro: no device with VID:PID {self._vid:04x}:{self._pid:04x}")
            return False
        # Detach kernel driver if necessary (Linux). Maestro doesn't
        # need a CDC kernel driver claim for vendor requests but if
        # ModemManager has it, our ctrl_transfer can sometimes get
        # blocked. Best-effort detach.
        try:
            for cfg in self._dev:
                for intf in cfg:
                    if self._dev.is_kernel_driver_active(intf.bInterfaceNumber):
                        try:
                            self._dev.detach_kernel_driver(intf.bInterfaceNumber)
                        except Exception:
                            pass
        except Exception:
            pass
        return True

    def close(self) -> None:
        # pyusb releases the device on garbage collection; nothing to
        # do explicitly here.
        self._dev = None
        self._frame_buf.clear()

    def is_open(self) -> bool:
        return self._dev is not None

    def device_path(self) -> Optional[str]:
        if self._dev is None:
            return None
        return f"usb:{self._vid:04x}:{self._pid:04x}"

    def write(self, data: bytes) -> bool:
        """Buffer + parse Pololu-protocol frames from `data`, translate
        each complete frame into a vendor control transfer."""
        if self._dev is None:
            return False
        self._frame_buf.extend(data)
        # Greedy parse: each frame is 0xAA device cmd [payload].
        # Payload length is implied by command. Commands we handle:
        #   0x04 Set Target  — 3-byte payload (ch, lo7, hi7)
        #   0x07 Set Speed   — 3-byte payload
        #   0x09 Set Accel   — 3-byte payload
        #   0x22 Go Home     — 0-byte payload
        # Anything else: best-effort skip the byte.
        i = 0
        while i < len(self._frame_buf):
            if self._frame_buf[i] != MAESTRO_POLOLU_START_BYTE:
                i += 1
                continue
            if i + 3 > len(self._frame_buf):
                break    # not enough bytes yet for header
            cmd = self._frame_buf[i + 2] & 0x7F
            if cmd in (MAESTRO_CMD_SET_TARGET, MAESTRO_CMD_SET_SPEED, MAESTRO_CMD_SET_ACCEL):
                if i + 6 > len(self._frame_buf):
                    break  # need 3 payload bytes
                ch = self._frame_buf[i + 3] & 0x7F
                v  = self._frame_buf[i + 4] | (self._frame_buf[i + 5] << 7)
                self._dispatch(cmd, ch, v)
                i += 6
            elif cmd == MAESTRO_CMD_GO_HOME:
                self._dispatch(cmd, 0, 0)
                i += 3
            else:
                i += 1   # unknown command — skip the byte to resync
        # Discard everything we consumed.
        del self._frame_buf[:i]
        return True

    def _dispatch(self, cmd: int, channel: int, value: int) -> None:
        if self._dev is None:
            return
        try:
            if cmd == MAESTRO_CMD_SET_TARGET:
                # Vendor request 0x85: wValue=target_qus, wIndex=channel
                self._dev.ctrl_transfer(0x40, _REQ_SET_TARGET, value, channel, 0, timeout=200)
            elif cmd == MAESTRO_CMD_SET_SPEED:
                self._dev.ctrl_transfer(0x40, _REQ_SET_SERVO_VARIABLE, value, channel, 0, timeout=200)
            elif cmd == MAESTRO_CMD_SET_ACCEL:
                self._dev.ctrl_transfer(0x40, _REQ_SET_SERVO_VARIABLE, value, channel | 0x80, 0, timeout=200)
            elif cmd == MAESTRO_CMD_GO_HOME:
                # No direct vendor equivalent; SCRIPT_DONE=0 triggers
                # script restart which re-applies HOME per channel.
                self._dev.ctrl_transfer(0x40, _REQ_SET_SCRIPT_DONE, 0, 0, 0, timeout=200)
        except Exception as e:
            if self._logger:
                self._logger.warn(f"Maestro: vendor xfer failed cmd=0x{cmd:02x} ch={channel}: {e}")

    def read(self, n: int) -> bytes:
        # Reading bytes off EP0 doesn't make sense in this protocol —
        # vendor reads are explicit per-request (GET_PARAMETER,
        # GET_VARIABLES). MaestroDriver doesn't currently use read on
        # the vendor backend; if it ever does, return empty.
        return b""

    def drain(self) -> int:
        return 0

    # ── Vendor-only extension ──────────────────────────────────────
    def read_parameter(self, param_id: int, length: int = 1) -> Optional[bytes]:
        """Pololu GET_PARAMETER (0x81). Returns `length` bytes from
        EEPROM parameter `param_id`, or None on transport error.
        Used by the channel-config-from-EEPROM readback path. Only
        available on the usb_vendor backend — other transports
        physically cannot do this. See docs/MAESTRO_BRINGUP.md."""
        if self._dev is None:
            return None
        try:
            data = bytes(self._dev.ctrl_transfer(0xC0, _REQ_GET_PARAMETER, 0, param_id, length, timeout=500))
            return data
        except Exception as e:
            if self._logger:
                self._logger.warn(f"Maestro: GET_PARAMETER({param_id}, {length}) failed: {e}")
            return None


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
        # channel_idx -> (min_us, max_us, neutral_us, home_us)
        self._channel_cfg: Dict[int, tuple[int, int, int, int]] = {}

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
        # Transport selection. "uart" uses the existing TTL UART path.
        # "usb_vendor" uses the pyusb EP0 control-transfer backend
        # which works in any Maestro Serial Mode and supports EEPROM
        # readback (read_parameter). The legacy "usb_host" string maps
        # to "usb_cdc" semantically, but until we add a pyserial-based
        # CDC backend, treat usb_cdc / usb_host the same as usb_vendor
        # — both enumerate over USB on the Pi; vendor is strictly more
        # capable and works for all USB connection scenarios.
        transport = str(params.get("transport", "uart")).lower()
        if transport in ("usb_vendor", "usb_cdc", "usb_host", "usb"):
            transport_cls = _PyUsbVendorBackend
        else:
            transport_cls = self._transport_cls   # UartTransport

        if self._uart is None or self._uart_pins != (tx, rx, baud) or not isinstance(self._uart, transport_cls):
            if self._uart is not None:
                self._uart.close()
            self._uart = transport_cls(tx, rx, baud, logger=self._logger)
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
            # home_us 0 = "unconfigured / don't auto-home". The
            # default-channel factory in peripheral_model populates
            # this with neutral, so configured channels do auto-home.
            home_us = int(ch_cfg.get("home_us", 0))
            self._channel_cfg[ch] = (min_us, max_us, neu_us, home_us)

            # Speed + acceleration are stored as the channel's default
            # (only applied on Pose transitions). They're NOT pushed to
            # the chip on config save — animation/control input should
            # snap to the target value at full speed, so the chip's
            # runtime limits stay at 0 (unlimited) unless a Pose
            # deliberately sets them. See TODO in
            # firmware/shared/src/maestro_driver.c
            # maestro_set_channel_config — same v1 policy on both
            # firmware platforms.
            # TODO(Pose-editor): apply ch_cfg["speed"]/["acceleration"]
            # via _send_set_speed/_send_set_accel BEFORE set_value from
            # the Pose-play code path, then reset to 0 after the
            # transition.

        # Send each configured channel to its home position via Set
        # Target so a freshly-connected (or re-connected) Maestro
        # lands at known positions instead of holding whatever it had
        # last. SaintOS-side home — the Maestro EEPROM HomeMode/HOME
        # is still required to be `Goto` with a non-zero HOME for PWM
        # to come up at all (see docs/MAESTRO_BRINGUP.md).
        for ch_idx, (mn, mx, _, home_us) in self._channel_cfg.items():
            if home_us == 0:
                continue
            # Clamp to per-channel [min, max] same as set_value so a
            # stale operator-typed home_us out of range doesn't drive
            # past safe extents.
            clamped = max(mn, min(mx, home_us))
            low7, high7 = _pulse_us_to_quarters(clamped)
            frame = _build_pololu_frame(
                self._device_num, MAESTRO_CMD_SET_TARGET,
                bytes([ch_idx & 0x7F, low7, high7]))
            self._uart.write(frame)
        return True

    def set_value(self, instance_id: int, sub_channel: int, value: float) -> bool:
        if instance_id != 0 or self._uart is None:
            return False
        if not (0 <= sub_channel < MAESTRO_MAX_CHANNELS):
            return False
        value = max(-1.0, min(1.0, value))

        min_us, max_us, neu_us, _home_us = self._channel_cfg.get(
            sub_channel,
            (MAESTRO_DEFAULT_MIN_PULSE_US,
             MAESTRO_DEFAULT_MAX_PULSE_US,
             MAESTRO_DEFAULT_NEUTRAL_US,
             0))
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
