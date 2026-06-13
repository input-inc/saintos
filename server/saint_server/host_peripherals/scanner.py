"""
BLE scan helper for the host_controller node.

Mirrors firmware/raspberrypi/saint_node/ble_transport.scan_jbd_devices,
but exposed as a coroutine (no asyncio.run_coroutine_threadsafe wrap
needed — the caller is already on the server's shared async loop).
"""

from __future__ import annotations

import asyncio
from typing import Callable, Dict, List, Optional

try:
    from bleak import BleakScanner
    BLEAK_AVAILABLE = True
except ImportError:
    BLEAK_AVAILABLE = False


JBD_BLE_SERVICE_UUID = "0000ff00-0000-1000-8000-00805f9b34fb"

# Callback shape for progressive scan results. Receives the current
# best-known device list (sorted strongest-first) every time a new
# MAC enters the set.
ProgressCallback = Callable[[List[Dict[str, object]]], None]


async def scan_jbd_devices(duration_s: float = 8.0,
                           filter_service: bool = True,
                           on_progress: Optional[ProgressCallback] = None,
                           ) -> List[Dict[str, object]]:
    """Scan for BLE devices for `duration_s` and return a list of
    {"mac": str, "name": str, "rssi": int} sorted strongest-first.

    When `filter_service` is True (default), only devices that
    advertise the JBD service UUID (0xFF00) are returned. Set False
    when the operator wants to see every nearby radio (debugging an
    install).

    `on_progress`, if provided, is called every time a NEW MAC
    enters the result set — operators see candidates appear in the
    UI as soon as they advertise instead of waiting for the full
    scan window. The callback receives the current sorted list. It
    runs on the scan's event loop; do non-trivial work via
    loop.call_soon / loop.create_task so detection callbacks aren't
    blocked.

    Raises RuntimeError if bleak isn't installed — the caller surfaces
    that instead of silently returning [].
    """
    if not BLEAK_AVAILABLE:
        raise RuntimeError(
            "bleak not installed — BLE scan unavailable. `pip install bleak`")

    found: Dict[str, Dict[str, object]] = {}

    def _emit_progress() -> None:
        if on_progress is None:
            return
        snapshot = sorted(found.values(),
                          key=lambda d: d["rssi"], reverse=True)
        try:
            on_progress(snapshot)
        except Exception:
            # Don't let a UI broadcast issue kill the scan.
            pass

    def _on_advert(device, advert_data):
        mac = device.address
        if filter_service:
            uuids = [u.lower() for u in (advert_data.service_uuids or [])]
            if JBD_BLE_SERVICE_UUID.lower() not in uuids:
                return
        rssi = getattr(advert_data, "rssi", None)
        if rssi is None:
            rssi = getattr(device, "rssi", 0)
        rssi = int(rssi or 0)
        existing = found.get(mac)
        is_new = existing is None
        # Always refresh name + rssi: BMSes sometimes change their
        # local_name after a brief moment, and RSSI drifts.
        found[mac] = {
            "mac": mac,
            "name": device.name or advert_data.local_name or
                    (existing["name"] if existing else ""),
            "rssi": rssi if (existing is None or rssi > existing["rssi"])
                    else existing["rssi"],
        }
        # Fire on_progress only when the set grows. RSSI-only updates
        # don't change what the operator can pick, so suppressing
        # them keeps the WS broadcast rate manageable.
        if is_new:
            _emit_progress()

    scanner = BleakScanner(detection_callback=_on_advert)
    try:
        await scanner.start()
        await asyncio.sleep(duration_s)
    finally:
        try:
            await scanner.stop()
        except Exception:
            pass

    devices = sorted(found.values(),
                     key=lambda d: d["rssi"], reverse=True)
    return devices
