"""
SAINT.OS Pi node — peripheral driver base class.

Mirrors the firmware-side peripheral_driver_t (firmware/shared/include/
peripheral_driver.h). Each peripheral type (Maestro, SyRen, RoboClaw,
Tic, Pathfinder BMS) subclasses PeripheralDriver and implements:
  - apply_config(instance, pins, params): wire-up + push config to chip
  - set_value(instance, sub_channel, value): operator → wire
  - get_value(instance, sub_channel): wire → operator
  - update(): periodic poll (telemetry round-robin, keepalive, etc.)
  - estop(): safety stop all instances of this peripheral

Class-level metadata (TYPE_ID, MODE_STRING, VIRTUAL_GPIO_BASE,
CHANNELS_PER_INSTANCE, MAX_INSTANCES) lets the registry dispatch
virtual GPIO numbers to the right driver + sub-channel without having
to ask each driver.
"""

from __future__ import annotations

import abc
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class PeripheralInstance:
    """One configured instance of a peripheral type — e.g. one SyRen
    controller at address 128 on UART pair (0, 1)."""

    instance_id: int
    """Slot 0..MAX_INSTANCES-1. Same as RoboClaw's `unit_idx`."""

    logical_name: str = ""
    """Operator-supplied id (the `id` field of the peripheral JSON)."""

    pins: Dict[str, int] = field(default_factory=dict)
    """Pin assignments (e.g. {"uart_tx": 14, "uart_rx": 15})."""

    params: Dict[str, Any] = field(default_factory=dict)
    """Operator-supplied params (e.g. {"address": 128, "deadband": 0})."""

    # Runtime state — drivers fill these as they go. Kept on the instance
    # so the manager can publish state without going through the driver.
    last_values: Dict[int, float] = field(default_factory=dict)
    """Most recent (sub_channel -> value) read by get_value."""

    connected: bool = False
    """True after the driver has confirmed two-way comms (e.g. a probe
    response, ACK, etc.)."""


class PeripheralDriver(abc.ABC):
    """Base class for SAINT.OS Pi-side peripheral drivers.

    Subclasses provide class-level metadata to register with the
    PeripheralRegistry, and implement the methods below.

    Concurrency: the manager calls driver methods from the node's main
    loop thread. Long-running operations (e.g. UART reads with
    multi-millisecond timeouts) block the loop — keep them short.
    Drivers that need persistent background work (BMS polling at 1 Hz,
    RoboClaw duty keepalive) should use update() which the manager
    calls every loop tick (10 Hz).
    """

    # ── Required class-level metadata ──────────────────────────────

    TYPE_ID: str = ""
    """Matches the peripheral JSON's "type" field — "syren", "tic", etc."""

    MODE_STRING: str = ""
    """Pin mode string the server uses for capability checks —
    "syren_motor", "tic_stepper", etc."""

    VIRTUAL_GPIO_BASE: int = 0
    """First virtual GPIO number this peripheral owns. Matches the
    firmware-side base from <type>_protocol.h."""

    CHANNELS_PER_INSTANCE: int = 1
    """Number of sub-channels each instance exposes (e.g. RoboClaw is
    5: motor, encoder, voltage, current, temp)."""

    MAX_INSTANCES: int = 1
    """Maximum number of instances this peripheral supports per node.
    Total virtual GPIO slots claimed = MAX_INSTANCES * CHANNELS_PER_INSTANCE."""

    SUB_CHANNEL_NAMES: List[str] = []
    """Per-sub-channel labels for logging/diagnostics. Indexed 0..
    CHANNELS_PER_INSTANCE-1. Optional but encouraged."""

    # ── Construction ───────────────────────────────────────────────

    def __init__(self, logger=None):
        self._logger = logger
        self._instances: Dict[int, PeripheralInstance] = {}

    # ── Helpers a driver can override but doesn't have to ──────────

    def update(self) -> None:
        """Called from the node's main loop tick (default 10 Hz).
        Override for periodic telemetry polling, keepalives, etc.
        No-op by default — write-only drivers (SyRen) don't need it."""

    def estop(self) -> None:
        """Safety stop on every configured instance. Drivers override
        with whatever wire-level command kills motion / opens contacts
        / etc. Default: log and no-op."""
        self._log("warn", f"{self.TYPE_ID}: ESTOP (no driver-specific action)")

    def clear_estop(self) -> None:
        """Counterpart to estop(): release any latched safety state so
        the operator can resume control without re-syncing the whole
        config. Only meaningful for drivers that latch on estop (e.g.
        RoboClaw's S3 input). Default: no-op."""

    # ── Required by every driver ──────────────────────────────────

    @abc.abstractmethod
    def apply_config(self, instance_id: int, pins: Dict[str, int],
                     params: Dict[str, Any]) -> bool:
        """Add or update an instance. Wire up UART/GPIO, push initial
        chip config. Returns False on hard failure (e.g. invalid pins).
        Idempotent — called again with the same params is a no-op."""

    @abc.abstractmethod
    def set_value(self, instance_id: int, sub_channel: int, value: float) -> bool:
        """Operator → wire. Sub-channel meaning is per-driver. Returns
        False if the sub-channel is read-only or instance unknown."""

    @abc.abstractmethod
    def get_value(self, instance_id: int, sub_channel: int) -> Optional[float]:
        """Read most recent value. Returns None if no value available
        yet (e.g. telemetry hasn't polled this register)."""

    # ── Optional: respond to factory reset / cleanup ───────────────

    def reset(self) -> None:
        """Wipe all instances + release UART/GPIO. Called from factory
        reset and shutdown. Drivers should detach IntervalTimers,
        close serial ports, etc."""
        self._instances.clear()

    # ── Convenience for instance bookkeeping ───────────────────────

    def _get_or_create_instance(self, instance_id: int) -> PeripheralInstance:
        inst = self._instances.get(instance_id)
        if inst is None:
            if instance_id >= self.MAX_INSTANCES:
                raise ValueError(
                    f"{self.TYPE_ID}: instance_id {instance_id} >= "
                    f"MAX_INSTANCES {self.MAX_INSTANCES}"
                )
            inst = PeripheralInstance(instance_id=instance_id)
            self._instances[instance_id] = inst
        return inst

    # ── Logging shim — matches gpio_control.py's pattern ───────────

    def _log(self, level: str, msg: str) -> None:
        if self._logger:
            getattr(self._logger, level)(msg)
        else:
            print(f"[{level.upper()}] {msg}")

    # ── Introspection used by the manager ──────────────────────────

    @classmethod
    def total_channel_count(cls) -> int:
        return cls.MAX_INSTANCES * cls.CHANNELS_PER_INSTANCE

    @classmethod
    def virtual_gpio_range(cls) -> range:
        """Half-open range of virtual GPIO numbers this peripheral owns."""
        return range(cls.VIRTUAL_GPIO_BASE,
                     cls.VIRTUAL_GPIO_BASE + cls.total_channel_count())

    @classmethod
    def split_channel(cls, vgpio: int) -> tuple[int, int]:
        """Split a virtual gpio number into (instance_id, sub_channel).
        Raises ValueError if vgpio isn't in this peripheral's range."""
        if vgpio not in cls.virtual_gpio_range():
            raise ValueError(
                f"{cls.TYPE_ID}: vgpio {vgpio} not in "
                f"{cls.VIRTUAL_GPIO_BASE}..{cls.VIRTUAL_GPIO_BASE + cls.total_channel_count() - 1}"
            )
        offset = vgpio - cls.VIRTUAL_GPIO_BASE
        return offset // cls.CHANNELS_PER_INSTANCE, offset % cls.CHANNELS_PER_INSTANCE
