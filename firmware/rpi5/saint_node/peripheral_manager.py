"""
SAINT.OS Pi node — peripheral manager + registry.

Singleton-ish manager that owns one instance of each registered
PeripheralDriver subclass. node.py calls into the manager to:
  - apply a peripheral config (from a "configure" message)
  - dispatch set_pin control messages by virtual GPIO number
  - call update() each tick for drivers that need it (BMS polls, etc.)
  - estop everything on a safety stop
  - tear down on factory reset / shutdown

Virtual GPIO routing: each driver class declares VIRTUAL_GPIO_BASE +
CHANNELS_PER_INSTANCE + MAX_INSTANCES. The manager picks the right
driver by walking those ranges. Drivers that overlap their virtual
gpio ranges with each other are a bug — we log a warning on register.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Type

from .peripherals.base import PeripheralDriver, PeripheralInstance


class PeripheralManager:
    """Owns the peripheral driver registry + per-driver instance state."""

    def __init__(self, logger=None):
        self._logger = logger
        self._drivers: Dict[str, PeripheralDriver] = {}
        """type_id -> driver instance"""
        self._driver_by_vgpio_range: List[tuple[range, PeripheralDriver]] = []
        """Cache for fast routing of set_value(gpio, ...) → driver."""

    # ── Registration ───────────────────────────────────────────────

    def register(self, driver_cls: Type[PeripheralDriver]) -> PeripheralDriver:
        """Instantiate `driver_cls` and add it to the registry. Returns
        the instance (mostly useful in tests)."""
        if not driver_cls.TYPE_ID:
            raise ValueError(f"{driver_cls.__name__}: TYPE_ID must be set")
        if driver_cls.TYPE_ID in self._drivers:
            self._log("warn",
                f"PeripheralManager: '{driver_cls.TYPE_ID}' already "
                f"registered — overwriting")
        # Detect overlapping virtual-GPIO ranges before instantiating.
        new_range = driver_cls.virtual_gpio_range()
        for existing_range, existing in self._driver_by_vgpio_range:
            if (new_range.start < existing_range.stop and
                    new_range.stop > existing_range.start):
                self._log("warn",
                    f"PeripheralManager: '{driver_cls.TYPE_ID}' "
                    f"vGPIO {new_range.start}..{new_range.stop - 1} "
                    f"overlaps existing '{existing.TYPE_ID}' "
                    f"{existing_range.start}..{existing_range.stop - 1} "
                    f"— set_pin routing will pick the first match")
        driver = driver_cls(logger=self._logger)
        self._drivers[driver_cls.TYPE_ID] = driver
        self._driver_by_vgpio_range.append((new_range, driver))
        # Keep sorted so iteration order is stable + we can short-circuit
        # the range walk in route_vgpio().
        self._driver_by_vgpio_range.sort(key=lambda pair: pair[0].start)
        self._log("info",
            f"PeripheralManager: registered '{driver_cls.TYPE_ID}' "
            f"(vGPIO {new_range.start}..{new_range.stop - 1}, "
            f"{driver_cls.MAX_INSTANCES} × {driver_cls.CHANNELS_PER_INSTANCE} ch)")
        return driver

    def get_driver(self, type_id: str) -> Optional[PeripheralDriver]:
        return self._drivers.get(type_id)

    def all_drivers(self) -> List[PeripheralDriver]:
        return list(self._drivers.values())

    # ── Lookup: virtual GPIO → driver ─────────────────────────────

    def route_vgpio(self, vgpio: int) -> Optional[tuple[PeripheralDriver, int, int]]:
        """Map a virtual GPIO to (driver, instance_id, sub_channel).
        Returns None when the GPIO isn't in any peripheral's range."""
        for rng, driver in self._driver_by_vgpio_range:
            if vgpio in rng:
                instance_id, sub_channel = type(driver).split_channel(vgpio)
                return driver, instance_id, sub_channel
        return None

    def is_peripheral_vgpio(self, gpio: int) -> bool:
        """True if `gpio` falls inside any registered peripheral's range.
        The node uses this to decide whether a set_pin message routes
        to the peripheral manager or to gpio_control."""
        return any(gpio in rng for rng, _ in self._driver_by_vgpio_range)

    # ── Config application ────────────────────────────────────────

    def apply_peripherals(self, peripherals: List[Dict[str, Any]]) -> None:
        """Apply a "peripherals":[...] array from a configure message.

        Each entry is {"id": str, "type": str, "pins": {...}, "params": {...}}.
        Multiple entries with the same type stack onto consecutive
        instance slots (same pattern as the firmware).
        """
        # First pass: tear down every driver so a fresh sync starts clean
        # (matches the firmware-side pin_config_reset before apply_peripherals_json).
        for driver in self.all_drivers():
            driver.reset()

        # Group by type so each driver sees instance_id 0..N in order.
        per_type_instances: Dict[str, List[Dict[str, Any]]] = {}
        for entry in peripherals:
            type_id = entry.get("type", "")
            per_type_instances.setdefault(type_id, []).append(entry)

        for type_id, entries in per_type_instances.items():
            driver = self._drivers.get(type_id)
            if driver is None:
                self._log("warn",
                    f"PeripheralManager: no driver for type '{type_id}' "
                    f"— skipping {len(entries)} instance(s)")
                continue
            if len(entries) > type(driver).MAX_INSTANCES:
                self._log("warn",
                    f"PeripheralManager: type '{type_id}' got "
                    f"{len(entries)} instances but MAX is "
                    f"{type(driver).MAX_INSTANCES} — truncating")
                entries = entries[: type(driver).MAX_INSTANCES]
            for inst_idx, entry in enumerate(entries):
                pins = entry.get("pins", {}) or {}
                params = entry.get("params", {}) or {}
                logical = entry.get("id", "")
                ok = False
                try:
                    ok = driver.apply_config(inst_idx, pins, params)
                except Exception as e:
                    self._log("error",
                        f"PeripheralManager: {type_id}#{inst_idx} "
                        f"apply_config raised: {e}")
                if ok:
                    inst = driver._instances.get(inst_idx)
                    if inst is not None:
                        inst.logical_name = logical
                    self._log("info",
                        f"PeripheralManager: configured {type_id}#{inst_idx} "
                        f"({logical}) pins={pins} params={params}")
                else:
                    self._log("warn",
                        f"PeripheralManager: {type_id}#{inst_idx} "
                        f"apply_config returned False")

    # ── Per-loop tick + safety stop ───────────────────────────────

    def update(self) -> None:
        """Fan out to every driver. Called once per node main-loop
        tick. Exceptions in one driver don't bring down the others."""
        for driver in self.all_drivers():
            try:
                driver.update()
            except Exception as e:
                self._log("error",
                    f"PeripheralManager: {driver.TYPE_ID}.update() raised: {e}")

    def estop(self) -> None:
        for driver in self.all_drivers():
            try:
                driver.estop()
            except Exception as e:
                self._log("error",
                    f"PeripheralManager: {driver.TYPE_ID}.estop() raised: {e}")

    def clear_estop(self) -> None:
        for driver in self.all_drivers():
            try:
                driver.clear_estop()
            except Exception as e:
                self._log("error",
                    f"PeripheralManager: {driver.TYPE_ID}.clear_estop() raised: {e}")

    def reset(self) -> None:
        """Wipe instance state on every driver. Used for factory reset
        and shutdown. The drivers stay registered."""
        for driver in self.all_drivers():
            try:
                driver.reset()
            except Exception as e:
                self._log("error",
                    f"PeripheralManager: {driver.TYPE_ID}.reset() raised: {e}")

    # ── Control dispatch ──────────────────────────────────────────

    def set_value(self, vgpio: int, value: float) -> bool:
        routed = self.route_vgpio(vgpio)
        if routed is None:
            return False
        driver, instance_id, sub_channel = routed
        try:
            return driver.set_value(instance_id, sub_channel, value)
        except Exception as e:
            self._log("error",
                f"PeripheralManager: {driver.TYPE_ID}#{instance_id}"
                f".set_value({sub_channel}, {value}) raised: {e}")
            return False

    def get_value(self, vgpio: int) -> Optional[float]:
        routed = self.route_vgpio(vgpio)
        if routed is None:
            return None
        driver, instance_id, sub_channel = routed
        try:
            return driver.get_value(instance_id, sub_channel)
        except Exception as e:
            self._log("error",
                f"PeripheralManager: {driver.TYPE_ID}#{instance_id}"
                f".get_value({sub_channel}) raised: {e}")
            return None

    # ── State publishing ──────────────────────────────────────────

    def collect_states(self) -> List[Dict[str, Any]]:
        """Snapshot of every instance's last_values, in the same shape
        as gpio_control.get_all_states() — one entry per (driver,
        instance, sub_channel) so the server can render telemetry."""
        out: List[Dict[str, Any]] = []
        for driver in self.all_drivers():
            cls = type(driver)
            for inst_id, inst in driver._instances.items():
                for sub in range(cls.CHANNELS_PER_INSTANCE):
                    vgpio = cls.VIRTUAL_GPIO_BASE + inst_id * cls.CHANNELS_PER_INSTANCE + sub
                    value = inst.last_values.get(sub)
                    if value is None:
                        # Try a live read so the dashboard shows
                        # something even before update() has polled.
                        value = driver.get_value(inst_id, sub) or 0.0
                    sub_name = (cls.SUB_CHANNEL_NAMES[sub]
                                if sub < len(cls.SUB_CHANNEL_NAMES) else f"ch{sub}")
                    out.append({
                        "gpio": vgpio,
                        "mode": cls.MODE_STRING,
                        "value": float(value),
                        "name": f"{inst.logical_name}/{sub_name}" if inst.logical_name else sub_name,
                        "connected": inst.connected,
                    })
        return out

    # ── helpers ───────────────────────────────────────────────────

    def _log(self, level: str, msg: str) -> None:
        if self._logger:
            getattr(self._logger, level)(msg)
        else:
            print(f"[{level.upper()}] {msg}")
