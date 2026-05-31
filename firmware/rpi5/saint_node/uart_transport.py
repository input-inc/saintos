"""
SAINT.OS Pi node — UART transport.

Thin wrapper around pyserial that the peripheral drivers go through
instead of opening serial ports directly. Lets the test suite swap in
a mock transport without each driver knowing about pyserial.

The pin-pair → device-path resolution mirrors the firmware's
uart_pin_pair_lookup: an operator-supplied (tx_pin, rx_pin) gets
mapped to a real serial device. On the Pi 5, common pairs:

  GPIO 14 (TX) / 15 (RX)  → /dev/ttyAMA0 (primary UART)
  GPIO 0  (TX) / 1  (RX)  → /dev/ttyAMA2 (UART2 via dt-overlay)
  GPIO 4  (TX) / 5  (RX)  → /dev/ttyAMA3
  GPIO 8  (TX) / 9  (RX)  → /dev/ttyAMA4
  GPIO 12 (TX) / 13 (RX)  → /dev/ttyAMA5

Plus arbitrary USB-serial adapters: the operator can pass pseudo-pin
values 100..199 that map to /dev/ttyUSB0../dev/ttyUSB99 — keeps the
"pins are GPIOs" abstraction even when no actual GPIO is involved.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

try:
    import serial  # pyserial
    PYSERIAL_AVAILABLE = True
except ImportError:
    PYSERIAL_AVAILABLE = False


# Pi 5 GPIO pair → device path. Order matches the most common dt-overlay
# layouts; we don't try every alternate pin combination the chip can do.
_PIN_PAIR_TO_DEVICE = {
    (14, 15): "/dev/ttyAMA0",
    (0, 1):   "/dev/ttyAMA2",
    (4, 5):   "/dev/ttyAMA3",
    (8, 9):   "/dev/ttyAMA4",
    (12, 13): "/dev/ttyAMA5",
}


def resolve_pin_pair(tx_pin: int, rx_pin: int) -> Optional[str]:
    """Map (tx_pin, rx_pin) to a device path. Returns None if no
    standard mapping exists. USB pseudo-pins (100..199) map to
    /dev/ttyUSBn where n = tx_pin - 100."""
    if tx_pin >= 100 and tx_pin < 200:
        return f"/dev/ttyUSB{tx_pin - 100}"
    return _PIN_PAIR_TO_DEVICE.get((tx_pin, rx_pin))


class UartTransport:
    """One serial connection. Drivers create one per (tx, rx, baud)
    triple. Thread-safe write+read because a single Pi node may have
    multiple peripherals sharing the same UART (e.g. four RoboClaws
    on one bus) and the manager processes them serially but a
    background telemetry poll could race a foreground set_value."""

    def __init__(self, tx_pin: int, rx_pin: int, baud: int,
                 timeout_s: float = 0.05, logger=None):
        self._tx_pin = tx_pin
        self._rx_pin = rx_pin
        self._baud = baud
        self._timeout_s = timeout_s
        self._logger = logger
        self._port = None
        self._lock = threading.Lock()
        self._device = resolve_pin_pair(tx_pin, rx_pin)

    # ── lifecycle ──────────────────────────────────────────────────

    def open(self) -> bool:
        if self._port is not None:
            return True
        if not PYSERIAL_AVAILABLE:
            self._log("error",
                "pyserial not installed — UART unavailable. "
                "`pip install pyserial`")
            return False
        if self._device is None:
            self._log("error",
                f"UART: pin pair TX={self._tx_pin} RX={self._rx_pin} "
                f"has no known device mapping. Use one of "
                f"{list(_PIN_PAIR_TO_DEVICE)} or USB pseudo-pin 100..199")
            return False
        try:
            self._port = serial.Serial(
                self._device, self._baud, timeout=self._timeout_s,
                write_timeout=self._timeout_s,
            )
            self._log("info",
                f"UART: opened {self._device} @ {self._baud} baud "
                f"(TX={self._tx_pin}, RX={self._rx_pin})")
            return True
        except Exception as e:
            self._log("error", f"UART: failed to open {self._device}: {e}")
            self._port = None
            return False

    def close(self) -> None:
        with self._lock:
            if self._port is not None:
                try:
                    self._port.close()
                except Exception:
                    pass
                self._port = None

    def is_open(self) -> bool:
        return self._port is not None

    @property
    def device_path(self) -> Optional[str]:
        return self._device

    # ── I/O ────────────────────────────────────────────────────────

    def write(self, data: bytes) -> bool:
        with self._lock:
            if self._port is None:
                return False
            try:
                self._port.write(data)
                self._port.flush()
                return True
            except Exception as e:
                self._log("error", f"UART write failed: {e}")
                return False

    def read(self, n: int) -> bytes:
        """Read up to n bytes, returning whatever arrives within
        timeout_s. May return fewer than n bytes (or zero)."""
        with self._lock:
            if self._port is None:
                return b""
            try:
                return self._port.read(n)
            except Exception as e:
                self._log("error", f"UART read failed: {e}")
                return b""

    def drain(self) -> int:
        """Read and discard everything currently in RX. Returns count.
        Used before sending a request to clear stale echoes."""
        with self._lock:
            if self._port is None:
                return 0
            try:
                drained = 0
                while self._port.in_waiting > 0:
                    drained += len(self._port.read(self._port.in_waiting))
                return drained
            except Exception:
                return 0

    def write_then_read(self, tx: bytes, rx_len: int,
                        drain_echo: bool = False) -> bytes:
        """Send tx, optionally drain `len(tx)` echo bytes (half-duplex
        wires that loop TX→RX through a resistor), then collect up to
        rx_len reply bytes. Returns the reply (possibly short on
        timeout)."""
        with self._lock:
            if self._port is None:
                return b""
            try:
                # Clear stale RX so the echo drain doesn't consume
                # leftovers from a previous failed exchange.
                while self._port.in_waiting > 0:
                    self._port.read(self._port.in_waiting)
                self._port.write(tx)
                self._port.flush()
                if drain_echo:
                    drained = 0
                    deadline = time.monotonic() + self._timeout_s
                    while drained < len(tx) and time.monotonic() < deadline:
                        chunk = self._port.read(len(tx) - drained)
                        if not chunk:
                            break
                        drained += len(chunk)
                if rx_len == 0:
                    return b""
                return self._port.read(rx_len)
            except Exception as e:
                self._log("error", f"UART xfer failed: {e}")
                return b""

    # ── helpers ────────────────────────────────────────────────────

    def set_baud(self, baud: int) -> None:
        """Reopen with a new baud rate. Used by FAS100-style probes
        that switch baud during auto-detection."""
        with self._lock:
            if self._port is not None:
                try:
                    self._port.baudrate = baud
                    self._baud = baud
                except Exception as e:
                    self._log("error", f"UART set_baud failed: {e}")

    def _log(self, level: str, msg: str) -> None:
        if self._logger:
            getattr(self._logger, level)(msg)
        else:
            print(f"[{level.upper()}] {msg}")


# ── Mock transport for tests ────────────────────────────────────────

class MockUartTransport:
    """In-memory UART for unit tests. Drop-in replacement for
    UartTransport with the same write/read/write_then_read API.

    Tests pre-load `rx_queue` with bytes the "chip" will send back,
    and inspect `tx_log` afterward to assert what the driver
    transmitted."""

    def __init__(self, tx_pin: int = 0, rx_pin: int = 1, baud: int = 9600):
        self._tx_pin = tx_pin
        self._rx_pin = rx_pin
        self._baud = baud
        self._open = True
        self.tx_log: bytearray = bytearray()
        """Everything the driver has written, in order."""
        self.rx_queue: bytearray = bytearray()
        """Bytes the "chip" will return on the next read(). Tests
        prime this before calling the driver."""
        self.echo_writes: bool = False
        """If True, mock half-duplex wiring: every byte written is
        also enqueued onto rx_queue (for echo-drain tests)."""

    def open(self) -> bool: self._open = True; return True
    def close(self) -> None: self._open = False
    def is_open(self) -> bool: return self._open
    @property
    def device_path(self) -> str: return f"mock://{self._tx_pin},{self._rx_pin}"

    def write(self, data: bytes) -> bool:
        if not self._open: return False
        self.tx_log.extend(data)
        if self.echo_writes:
            self.rx_queue.extend(data)
        return True

    def read(self, n: int) -> bytes:
        if not self._open: return b""
        take = min(n, len(self.rx_queue))
        out = bytes(self.rx_queue[:take])
        del self.rx_queue[:take]
        return out

    def drain(self) -> int:
        n = len(self.rx_queue)
        self.rx_queue.clear()
        return n

    def write_then_read(self, tx: bytes, rx_len: int,
                        drain_echo: bool = False) -> bytes:
        if not self._open: return b""
        self.tx_log.extend(tx)
        if self.echo_writes:
            self.rx_queue.extend(tx)
        if drain_echo:
            drain_n = min(len(tx), len(self.rx_queue))
            del self.rx_queue[:drain_n]
        if rx_len == 0:
            return b""
        return self.read(rx_len)

    def set_baud(self, baud: int) -> None:
        self._baud = baud
