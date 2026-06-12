"""
SAINT.OS Pi node — BLE transport for JBD-protocol BMSes.

Sibling of uart_transport.py. Exposes the same write_then_read shape
so peripheral drivers can swap UART for BLE by config alone.

Protocol UUIDs are the JBD/Xiaoxiang/Overkill standard:

    service       0000ff00-0000-1000-8000-00805f9b34fb
    write (TX)    0000ff02-0000-1000-8000-00805f9b34fb   host → BMS
    notify (RX)   0000ff01-0000-1000-8000-00805f9b34fb   BMS  → host

Architecture: bleak is async, the peripheral manager's update() is sync.
One shared asyncio loop runs in a daemon thread and owns every
BleakClient on the node. write_then_read submits a coroutine to that
loop with run_coroutine_threadsafe and blocks the caller for the
duration.

Reconnect: BLE drops are routine (BMS power-cycle, RF interference,
out-of-range). On disconnect we re-enter the connect loop with
exponential backoff (1s, 5s, 30s, 60s, then 60s cap) and keep
retrying indefinitely. last_values stay sticky in the driver; the
instance's `connected` flag flips false until we're back.
"""

from __future__ import annotations

import asyncio
import threading
from typing import Dict, List, Optional

try:
    from bleak import BleakClient, BleakScanner
    BLEAK_AVAILABLE = True
except ImportError:
    BLEAK_AVAILABLE = False


JBD_BLE_SERVICE_UUID  = "0000ff00-0000-1000-8000-00805f9b34fb"
JBD_BLE_WRITE_UUID    = "0000ff02-0000-1000-8000-00805f9b34fb"
JBD_BLE_NOTIFY_UUID   = "0000ff01-0000-1000-8000-00805f9b34fb"

JBD_FRAME_START = 0xDD
JBD_FRAME_END   = 0x77

# Reconnect backoff in seconds. Last value caps subsequent retries.
_RECONNECT_BACKOFF_S = (1.0, 5.0, 30.0, 60.0)

# How long write_then_read waits for a complete JBD reply before
# giving up and returning whatever we have.
_DEFAULT_REPLY_TIMEOUT_S = 2.0

# Cap on the per-transport notification buffer. JBD basic-info replies
# are ~34 bytes; we keep room for a few back-to-back frames in case
# reads pile up.
_NOTIFY_BUF_CAP = 1024


# ── Shared event loop ───────────────────────────────────────────────

_ble_loop: Optional[asyncio.AbstractEventLoop] = None
_ble_loop_thread: Optional[threading.Thread] = None
_ble_loop_lock = threading.Lock()


def _ensure_ble_loop() -> asyncio.AbstractEventLoop:
    """Lazily start one daemon-thread event loop for all BLE work."""
    global _ble_loop, _ble_loop_thread
    with _ble_loop_lock:
        if _ble_loop is not None and _ble_loop.is_running():
            return _ble_loop
        loop = asyncio.new_event_loop()

        def runner():
            asyncio.set_event_loop(loop)
            loop.run_forever()

        _ble_loop = loop
        _ble_loop_thread = threading.Thread(
            target=runner, name="saint-ble-loop", daemon=True)
        _ble_loop_thread.start()
        return loop


def _has_complete_jbd_frame(buf: bytes) -> bool:
    """A complete reply starts 0xDD and ends 0x77. The end byte can
    legitimately appear inside the data section, so we let the driver's
    parser do the real validation — this is just the "stop waiting"
    signal."""
    if len(buf) < 4:
        return False
    return buf[0] == JBD_FRAME_START and JBD_FRAME_END in buf[2:]


# ── Transport ──────────────────────────────────────────────────────

class BleTransport:
    """One BLE central connection. Drivers create one per (MAC) target.

    Mirrors the UartTransport surface so the BMS driver doesn't care
    which it has: open / close / is_open / write_then_read.
    """

    def __init__(self, mac: str, timeout_s: float = _DEFAULT_REPLY_TIMEOUT_S,
                 logger=None):
        self._mac = mac.upper().strip()
        self._timeout_s = timeout_s
        self._logger = logger
        self._client: Optional["BleakClient"] = None
        self._notify_buf = bytearray()
        self._notify_lock = threading.Lock()
        self._connected = False
        self._closing = False
        self._attempt = 0
        self._loop = _ensure_ble_loop() if BLEAK_AVAILABLE else None
        self._reconnect_task: Optional[asyncio.Future] = None

    @property
    def device_path(self) -> str:
        return f"ble://{self._mac}"

    @property
    def mac(self) -> str:
        return self._mac

    # ── lifecycle ──────────────────────────────────────────────────

    def open(self) -> bool:
        """Kick off the connect-and-keep-connected loop. Returns True
        if the asyncio runner accepted the task. Actual BLE link comes
        up asynchronously; callers should treat is_open() / connected
        as the source of truth, and write_then_read returns b"" until
        we're up."""
        if not BLEAK_AVAILABLE:
            self._log("error",
                "bleak not installed — BLE unavailable. `pip install bleak`")
            return False
        if self._loop is None:
            return False
        self._closing = False
        self._attempt = 0
        try:
            self._reconnect_task = asyncio.run_coroutine_threadsafe(
                self._connect_with_retry(), self._loop)
        except Exception as e:
            self._log("error", f"BLE: failed to schedule connect to {self._mac}: {e}")
            return False
        self._log("info", f"BLE: connecting to {self._mac} (async)")
        return True

    def close(self) -> None:
        self._closing = True
        if self._loop is None:
            return
        fut = asyncio.run_coroutine_threadsafe(self._disconnect(), self._loop)
        try:
            fut.result(timeout=3.0)
        except Exception:
            pass

    def is_open(self) -> bool:
        return self._connected

    # ── I/O ────────────────────────────────────────────────────────

    def write_then_read(self, tx: bytes, rx_len: int,
                        drain_echo: bool = False) -> bytes:
        """Write `tx` to 0xFF02, collect notification bytes from 0xFF01
        until we have a complete JBD frame (or `rx_len` bytes, or
        timeout). `drain_echo` is a UART half-duplex concept and is
        ignored here."""
        del drain_echo  # not applicable to BLE
        if not self._connected or self._loop is None:
            return b""
        try:
            fut = asyncio.run_coroutine_threadsafe(
                self._write_then_read_async(tx, rx_len), self._loop)
            # Give the async side its own deadline plus a small grace
            # window so we don't race the timeout.
            return fut.result(timeout=self._timeout_s + 0.5)
        except Exception as e:
            self._log("error", f"BLE xfer to {self._mac} failed: {e}")
            return b""

    # ── async internals (run on the shared loop) ──────────────────

    async def _connect_with_retry(self) -> None:
        while not self._closing:
            try:
                client = BleakClient(
                    self._mac,
                    timeout=10.0,
                    disconnected_callback=self._on_disconnect,
                )
                await client.connect()
                await client.start_notify(JBD_BLE_NOTIFY_UUID, self._on_notify)
                self._client = client
                self._connected = True
                self._attempt = 0
                self._log("info", f"BLE: connected to {self._mac}")
                return
            except Exception as e:
                self._connected = False
                self._client = None
                delay = _RECONNECT_BACKOFF_S[
                    min(self._attempt, len(_RECONNECT_BACKOFF_S) - 1)]
                self._attempt += 1
                self._log("warn",
                    f"BLE: connect to {self._mac} failed ({e}); "
                    f"retrying in {delay:.0f}s")
                try:
                    await asyncio.sleep(delay)
                except asyncio.CancelledError:
                    return

    def _on_disconnect(self, _client) -> None:
        # Fires on the asyncio loop thread.
        if self._closing:
            return
        self._log("warn", f"BLE: {self._mac} disconnected; reconnecting")
        self._connected = False
        self._client = None
        # Re-enter the retry loop. asyncio.ensure_future is loop-safe
        # because we're already on the loop thread.
        try:
            asyncio.ensure_future(self._connect_with_retry(), loop=self._loop)
        except Exception as e:
            self._log("error", f"BLE: failed to schedule reconnect: {e}")

    def _on_notify(self, _char, data: bytearray) -> None:
        with self._notify_lock:
            self._notify_buf.extend(data)
            if len(self._notify_buf) > _NOTIFY_BUF_CAP:
                # Discard the oldest half — keeps the tail, which is
                # where any in-flight reply lives.
                del self._notify_buf[: _NOTIFY_BUF_CAP // 2]

    async def _write_then_read_async(self, tx: bytes, rx_len: int) -> bytes:
        client = self._client
        if client is None or not self._connected:
            return b""
        # Clear any leftover notifications from the previous exchange
        # so we don't return a stale frame.
        with self._notify_lock:
            self._notify_buf.clear()
        try:
            await client.write_gatt_char(JBD_BLE_WRITE_UUID, tx, response=False)
        except Exception as e:
            self._log("error", f"BLE write to {self._mac} failed: {e}")
            return b""

        # Poll the notify buffer until a complete JBD frame arrives
        # (the driver's parser does final validation), or we hit rx_len,
        # or we time out. 25 ms polls keep latency low without burning
        # CPU.
        deadline = self._loop.time() + self._timeout_s
        while True:
            with self._notify_lock:
                buf = bytes(self._notify_buf)
            if _has_complete_jbd_frame(buf):
                return buf
            if rx_len > 0 and len(buf) >= rx_len:
                return buf
            if self._loop.time() >= deadline:
                return buf
            await asyncio.sleep(0.025)

    async def _disconnect(self) -> None:
        client, self._client = self._client, None
        self._connected = False
        if client is None:
            return
        try:
            await client.stop_notify(JBD_BLE_NOTIFY_UUID)
        except Exception:
            pass
        try:
            await client.disconnect()
        except Exception:
            pass

    # ── helpers ────────────────────────────────────────────────────

    def _log(self, level: str, msg: str) -> None:
        if self._logger:
            getattr(self._logger, level)(msg)
        else:
            print(f"[{level.upper()}] {msg}")


# ── Discovery ──────────────────────────────────────────────────────

def scan_jbd_devices(duration_s: float = 8.0,
                     filter_service: bool = True,
                     logger=None) -> List[Dict[str, object]]:
    """Synchronous wrapper: scan for nearby BLE devices and return any
    that advertise the JBD service UUID (or all devices, if
    `filter_service=False`).

    Returns a list of {"mac": str, "name": str, "rssi": int}. The
    caller (node.py's scan action handler) runs this on a worker
    thread so the main loop keeps ticking.

    Raises RuntimeError if bleak isn't installed — the caller surfaces
    that to the operator instead of silently returning [].
    """
    if not BLEAK_AVAILABLE:
        raise RuntimeError(
            "bleak not installed — BLE scan unavailable. `pip install bleak`")
    loop = _ensure_ble_loop()
    fut = asyncio.run_coroutine_threadsafe(
        _scan_jbd_async(duration_s, filter_service, logger), loop)
    # Worst case: bleak's internal scan can hang ~2× the requested
    # duration on a busy BlueZ. Generous grace window keeps us from
    # killing a live scan.
    return fut.result(timeout=duration_s * 2 + 5.0)


async def _scan_jbd_async(duration_s: float, filter_service: bool,
                          logger) -> List[Dict[str, object]]:
    found: List[Dict[str, object]] = []
    seen: set = set()

    def _on_advert(device, advert_data):
        if device.address in seen:
            return
        if filter_service:
            uuids = [u.lower() for u in (advert_data.service_uuids or [])]
            if JBD_BLE_SERVICE_UUID.lower() not in uuids:
                return
        seen.add(device.address)
        rssi = getattr(advert_data, "rssi", None)
        if rssi is None:
            rssi = getattr(device, "rssi", 0)
        found.append({
            "mac": device.address,
            "name": device.name or "",
            "rssi": int(rssi or 0),
        })

    scanner = BleakScanner(detection_callback=_on_advert)
    try:
        await scanner.start()
        await asyncio.sleep(duration_s)
    finally:
        try:
            await scanner.stop()
        except Exception:
            pass
    if logger is not None:
        logger.info(
            f"BLE scan: {len(found)} device(s) "
            f"({'JBD-filtered' if filter_service else 'unfiltered'}) "
            f"in {duration_s:.1f}s")
    # Strongest-signal-first is the most useful sort for the UI.
    found.sort(key=lambda d: d["rssi"], reverse=True)
    return found
