"""
HostPeripheralManager — owns the in-process BLE drivers backing the
host_controller virtual node.

Threading model: everything BLE-related runs on the saint-server's
shared asyncio loop (`server_node._async_loop`). The manager exposes
two threadsafe entry points:

  reconcile(peripherals)
      Called from any thread when host_controller's peripheral_config
      changes (initial load on startup, then each upsert / remove).
      Diffs against the running driver set, starts new ones, stops
      stale ones, hot-updates poll intervals. Returns immediately;
      driver lifecycle changes happen on the loop.

  scan(...)
      Returns a coroutine the caller schedules on the loop. Resolves
      to a list of nearby JBD devices.

The state-update callback handed to each driver writes into the
NodeRuntimeState directly. State broadcast to the UI happens on the
existing WebSocket loop tick (websocket_handler.py:2535 picks up
whatever's in runtime_state).
"""

from __future__ import annotations

import asyncio
import subprocess
from typing import Any, Callable, Dict, List, Optional

from .jbd_bms import JBD_BLE_SERVICE_UUID, JbdBleDriver
from .scanner import scan_jbd_devices


# Catalog type ids the manager knows how to drive. Anything else in
# the host_controller peripheral list is ignored (e.g. system_monitor,
# which the state_manager handles directly).
HANDLED_TYPES = frozenset({"pathfinder_bms"})

# Used by flush_stale_connections to identify JBD radios among the
# BlueZ-Connected device set. Short form is what `bluetoothctl info`
# prints in its UUID listing.
_JBD_SERVICE_UUID_HEX_PREFIX = "0000ff00"


StateCallback = Callable[[str, str, float], None]


class HostPeripheralManager:
    """One per server process. Holds the asyncio loop reference so
    callers can dispatch from any thread."""

    def __init__(self,
                 loop: asyncio.AbstractEventLoop,
                 state_cb: StateCallback,
                 log_cb: Optional[Callable[[str, str], None]] = None):
        self._loop = loop
        self._state_cb = state_cb
        self._log_cb = log_cb
        # Drivers keyed by peripheral_id (operator-visible name).
        self._drivers: Dict[str, JbdBleDriver] = {}
        # Snapshot of the last applied config keyed by peripheral_id
        # so reconcile can detect param-only edits (e.g. MAC change,
        # poll interval bump) without thrashing the connection.
        self._configured: Dict[str, Dict[str, Any]] = {}
        # Serializes concurrent reconciles (initial startup + rapid
        # operator edits) so two _reconcile_async coroutines don't
        # both try to stop the same driver.
        self._reconcile_lock: Optional[asyncio.Lock] = None

    # ── threadsafe entry points ─────────────────────────────────

    def reconcile(self, peripherals: List[Dict[str, Any]]) -> None:
        """Diff `peripherals` against running drivers; start/stop/
        update to match. Safe to call from any thread.

        Each entry follows the standard peripheral dict shape:
            {"id": str, "type": str, "pins": {...}, "params": {...}}
        """
        if not self._loop.is_running():
            self._log("warn", "reconcile called but async loop isn't running")
            return
        try:
            asyncio.run_coroutine_threadsafe(
                self._reconcile_async(list(peripherals)), self._loop)
        except RuntimeError as e:
            self._log("warn", f"reconcile scheduling failed: {e}")

    async def flush_stale_connections(self) -> None:
        """Disconnect any JBD-advertising device that BlueZ still
        believes is connected. Catches stale links left by a previous
        saint-os process that didn't get to clean up (SIGKILL, OOM,
        hard reboot, systemctl restart racing the asyncio teardown).

        JBD-clone BMSes only accept ONE central connection at a time
        — without this flush, restarting saint-os while a link from
        the previous process is still pinned in BlueZ will make every
        connect attempt fail until BlueZ times out the dead link
        (minutes).

        Awaitable on purpose: callers should await this BEFORE
        triggering reconcile, otherwise a driver connect can race
        with the flush and fail. Runs subprocess work on the executor
        so the event loop isn't blocked."""
        await self._flush_stale_connections_async()

    def scan(self, duration_s: float = 8.0,
             filter_jbd: bool = True
             ) -> "asyncio.Future[List[Dict[str, object]]]":
        """Schedule a BLE scan and return a Future the caller can await
        (or attach a `.add_done_callback` to). Safe to call from any
        thread."""
        return asyncio.run_coroutine_threadsafe(
            scan_jbd_devices(duration_s=duration_s,
                             filter_service=filter_jbd),
            self._loop)

    def shutdown(self) -> None:
        """Stop every driver. Called from server_node.py during
        teardown. Synchronous from the caller's perspective; the loop
        does the actual work."""
        if not self._loop.is_running():
            return
        try:
            fut = asyncio.run_coroutine_threadsafe(
                self._shutdown_async(), self._loop)
            fut.result(timeout=5.0)
        except Exception as e:
            self._log("warn", f"shutdown failed: {e}")

    # ── status queries ─────────────────────────────────────────

    def connected_peripheral_ids(self) -> List[str]:
        return [pid for pid, drv in self._drivers.items() if drv.is_connected]

    # ── async internals (run on the shared loop) ─────────────────

    async def _reconcile_async(self,
                               peripherals: List[Dict[str, Any]]) -> None:
        # Lock is created lazily on the loop so __init__ doesn't need
        # an active loop. Subsequent calls reuse the same lock.
        if self._reconcile_lock is None:
            self._reconcile_lock = asyncio.Lock()
        async with self._reconcile_lock:
            await self._reconcile_async_locked(peripherals)

    async def _reconcile_async_locked(self,
                                      peripherals: List[Dict[str, Any]]) -> None:
        # Index by peripheral_id, filter to types we drive.
        desired: Dict[str, Dict[str, Any]] = {}
        for entry in peripherals:
            type_id = entry.get("type", "")
            if type_id not in HANDLED_TYPES:
                continue
            pid = entry.get("id", "")
            params = entry.get("params") or {}
            transport = str(params.get("transport", "uart")).lower()
            if transport != "ble":
                # UART BMSes don't run on the host (no UART hardware).
                # Skip silently — the operator picked the wrong host.
                continue
            mac = str(params.get("mac", "")).strip()
            if not mac:
                self._log("warn",
                    f"{pid}: transport=ble but no MAC — skipping")
                continue
            desired[pid] = {
                "mac": mac,
                "poll_interval_s": max(
                    0.1, float(params.get("poll_interval_ms", 1000)) / 1000.0),
            }

        # Stop drivers no longer wanted (or whose MAC changed — the
        # MAC swap can't hot-update because the BLE target address is
        # baked into the connection).
        to_remove: List[str] = []
        to_recreate: List[str] = []
        for pid, drv in list(self._drivers.items()):
            want = desired.get(pid)
            if want is None:
                to_remove.append(pid)
            elif want["mac"] != drv.mac:
                to_recreate.append(pid)
        loop = asyncio.get_running_loop()
        for pid in to_remove + to_recreate:
            drv = self._drivers.get(pid)
            mac = drv.mac if drv is not None else None
            self._log("info", f"stopping driver for {pid}")
            try:
                if drv is not None:
                    await drv.stop()
            except Exception as e:
                self._log("warn", f"stopping {pid} raised: {e}")
            self._drivers.pop(pid, None)
            self._configured.pop(pid, None)
            # Bleak's disconnect doesn't always make BlueZ release
            # the link — the BMS then thinks someone is still
            # connected and refuses new central connections. Force
            # the drop via bluetoothctl so a subsequent re-add or
            # re-pair can succeed immediately.
            if mac:
                self._log("info", f"flushing BlueZ link for {mac}")
                try:
                    await loop.run_in_executor(
                        None, _bluetoothctl_disconnect, mac)
                except Exception as e:
                    self._log("warn", f"flush {mac} failed: {e}")

        # Start / hot-update drivers.
        for pid, want in desired.items():
            existing = self._drivers.get(pid)
            if existing is not None:
                # Same MAC — just refresh the poll cadence if it changed.
                existing.update_poll_interval(want["poll_interval_s"])
                self._configured[pid] = want
                continue
            self._log("info",
                f"starting driver for {pid} (mac={want['mac']})")
            driver = JbdBleDriver(
                peripheral_id=pid,
                mac=want["mac"],
                poll_interval_s=want["poll_interval_s"],
                state_cb=self._state_cb,
                log_cb=self._log_cb,
            )
            await driver.start()
            self._drivers[pid] = driver
            self._configured[pid] = want

    async def _flush_stale_connections_async(self) -> None:
        # bluetoothctl is shell-cheap and reliable. dbus-fast would be
        # more Pythonic but adds 60+ lines of code to do the same job.
        # Each call goes through run_in_executor so the asyncio loop
        # keeps serving other work while bluetoothctl is in flight.
        loop = asyncio.get_running_loop()
        try:
            connected = await loop.run_in_executor(
                None, _bluetoothctl_connected_devices)
        except FileNotFoundError:
            self._log("info",
                "bluetoothctl not installed — skipping stale-link flush")
            return
        if not connected:
            return
        for mac, name in connected:
            try:
                is_jbd = await loop.run_in_executor(
                    None, _bluetoothctl_device_is_jbd, mac)
            except Exception as e:
                self._log("warn", f"info {mac} failed: {e}")
                continue
            if not is_jbd:
                continue
            self._log("info",
                f"flushing stale JBD link {mac} ({name or 'unnamed'})")
            try:
                await loop.run_in_executor(
                    None, _bluetoothctl_disconnect, mac)
            except Exception as e:
                self._log("warn", f"disconnect {mac} failed: {e}")

    async def _shutdown_async(self) -> None:
        drivers = list(self._drivers.items())
        self._drivers.clear()
        self._configured.clear()
        for pid, drv in drivers:
            try:
                await drv.stop()
            except Exception as e:
                self._log("warn", f"shutdown of {pid} raised: {e}")

    def _log(self, level: str, msg: str) -> None:
        if self._log_cb is not None:
            try:
                self._log_cb(level, f"host_peripherals: {msg}")
            except Exception:
                pass


# ── bluetoothctl wrappers ──────────────────────────────────────────
#
# Thin shells around the CLI; live at module scope so the manager can
# punt them onto the loop's default executor without method-binding
# weirdness. All three swallow nothing — caller decides whether to
# log the exception. Timeouts are conservative because BlueZ on a
# busy adapter can take several seconds to answer.

def _bluetoothctl_connected_devices() -> List[tuple]:
    """List BLE devices BlueZ believes are currently connected.
    Returns [(mac, name), ...]. Empty list when no devices match."""
    out = subprocess.run(
        ["bluetoothctl", "devices", "Connected"],
        capture_output=True, text=True, timeout=8,
        check=False,
    ).stdout
    result: List[tuple] = []
    for line in out.splitlines():
        parts = line.split(maxsplit=2)
        if len(parts) >= 2 and parts[0] == "Device":
            mac = parts[1]
            name = parts[2] if len(parts) > 2 else ""
            result.append((mac, name))
    return result


def _bluetoothctl_device_is_jbd(mac: str) -> bool:
    """True iff bluetoothctl info reports the JBD service UUID for
    this device. BlueZ caches advertised UUIDs from prior scans so
    this works even if the device isn't actively advertising."""
    out = subprocess.run(
        ["bluetoothctl", "info", mac],
        capture_output=True, text=True, timeout=8,
        check=False,
    ).stdout.lower()
    return _JBD_SERVICE_UUID_HEX_PREFIX in out


def _bluetoothctl_disconnect(mac: str) -> None:
    """Tell BlueZ to drop any active link to this MAC. Best-effort —
    bluetoothctl prints a confirmation line on stdout we ignore."""
    subprocess.run(
        ["bluetoothctl", "disconnect", mac],
        capture_output=True, timeout=10,
        check=False,
    )
