"""
SAINT.OS server-side host peripherals.

The `host_controller` virtual node owns a small set of in-process
peripherals that talk to the host hardware directly (no ROS / micro-
ROS hop). `system_monitor` was the first; this package adds BLE
peripherals such as JBD-protocol BMSes that need a Linux BlueZ stack
to talk to their hardware.

Layout:
  jbd_bms.py — JBD/Xiaoxiang protocol decoder + BleakClient-backed
               polling driver (one instance per configured BMS).
  scanner.py — BleakScanner wrapper used by the operator-triggered
               discovery flow.
  manager.py — `HostPeripheralManager` reconciles configured BMSes
               against running driver instances and routes scan
               requests to the scanner. Lives on the server's shared
               asyncio loop.
"""

from .manager import HostPeripheralManager  # noqa: F401
