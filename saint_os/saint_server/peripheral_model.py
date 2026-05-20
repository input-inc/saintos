"""SAINT.OS peripheral / route / widget data model.

Replaces the old pin-centric `pin_config` abstraction. Peripherals are
the primary concept now: everything attached to a node — from a single-
pin button to a multi-channel sensor — is a peripheral that owns one or
more physical pins and exposes a list of named channels.

System-wide, Routes connect peripheral channels to logical signals and
to dashboard Widgets. Routes and Widgets live outside any individual
node since they cross node boundaries.

The catalog of *peripheral types* (FAS100, Button, LED, NeoPixel, …) is
defined here too. Channel definitions live with the type so the rest of
the server can ask "what does an instance of this peripheral expose?"
without consulting the firmware. (When a node reports its capabilities
the firmware can declare additional/built-in peripheral types; those
get merged into this catalog at runtime.)
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional, Tuple

# ─────────────────────────────────────────────────────────────────────────────
# Peripheral catalog
# ─────────────────────────────────────────────────────────────────────────────
#
# pin_kind values:
#   "gpio"     — one GPIO pin (in or out, depending on direction)
#   "pwm"      — one PWM-capable pin
#   "adc"      — one ADC-capable pin
#   "uart"     — a UART pin pair (tx, rx)
#   "i2c"      — an I2C pin pair (sda, scl)
#   "builtin"  — the peripheral is hardwired on the board; pins are fixed
#
# channel direction:
#   "in"  — peripheral produces this value (sensor reading)
#   "out" — peripheral consumes this value (motor command, LED state)
#
# capability:
#   short type tag used by routes to verify that a source-and-sink pair
#   are compatible. Compared verbatim today; could grow into a richer
#   type system later (analog/digital, value ranges, etc.).


@dataclass
class PeripheralChannel:
    """One named input or output of a peripheral type."""
    id: str
    display: str
    dir: str           # "in" | "out"
    cap: str           # capability tag

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class PeripheralTypeParam:
    """A configurable parameter on a peripheral type."""
    id: str
    label: str
    type: str          # "bool" | "int" | "float" | "string" | "gpio"
                       # "gpio" tells the UI to render this param as a
                       # dropdown of the controller's GPIO pins, filtered
                       # to those not already claimed by another
                       # peripheral. Stored as an integer on the wire,
                       # so the firmware-side parser stays unchanged.
    default: Any
    min: Optional[float] = None
    max: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        # Drop unset min/max so the JSON is compact.
        if d["min"] is None:
            d.pop("min")
        if d["max"] is None:
            d.pop("max")
        return d


@dataclass
class PeripheralType:
    """Static metadata about a kind of peripheral."""
    id: str
    label: str
    description: str
    pin_kind: str
    channels: List[PeripheralChannel] = field(default_factory=list)
    params: List[PeripheralTypeParam] = field(default_factory=list)
    # Built-in types are automatically instantiated when a node is adopted
    # and can't be removed by the operator. Examples: the Feather's
    # onboard NeoPixel.
    builtin_only: bool = False

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "label": self.label,
            "description": self.description,
            "pin_kind": self.pin_kind,
            "channels": [c.to_dict() for c in self.channels],
            "params": [p.to_dict() for p in self.params],
            "builtin_only": self.builtin_only,
        }


# Default catalog. Mirrors the firmware's runtime drivers + adds generic
# single-pin peripherals (Button, LED, AnalogInput, Servo) plus built-in
# board peripherals (NeoPixel).
DEFAULT_CATALOG: Dict[str, PeripheralType] = {
    "button": PeripheralType(
        id="button", label="Button",
        description="A momentary or latching switch on a GPIO pin.",
        pin_kind="gpio",
        channels=[PeripheralChannel("pressed", "Pressed", "in", "digital_in")],
        params=[
            PeripheralTypeParam("pull_up",     "Pull-up resistor",      "bool", True),
            PeripheralTypeParam("active_low",  "Active low",            "bool", True),
            PeripheralTypeParam("debounce_ms", "Debounce (ms)",         "int",  20, min=0, max=500),
        ],
    ),
    "led": PeripheralType(
        id="led", label="LED",
        description="A digital output on a GPIO pin.",
        pin_kind="gpio",
        channels=[PeripheralChannel("on", "On", "out", "digital_out")],
        params=[
            PeripheralTypeParam("active_low", "Active low", "bool", False),
            PeripheralTypeParam("initial_on", "Initial on", "bool", False),
        ],
    ),
    "analog_in": PeripheralType(
        id="analog_in", label="Analog Input",
        description="Voltage reading on an ADC pin.",
        pin_kind="adc",
        channels=[PeripheralChannel("voltage", "Voltage", "in", "analog")],
        params=[
            PeripheralTypeParam("divider_ratio", "Voltage divider ratio", "float", 1.0),
        ],
    ),
    "servo": PeripheralType(
        id="servo", label="Servo (PWM)",
        description="Hobby servo on a PWM-capable pin.",
        pin_kind="pwm",
        channels=[PeripheralChannel("angle", "Angle", "out", "analog")],
        params=[
            PeripheralTypeParam("min_pulse_us", "Min pulse (µs)", "int",  500, min=100, max=3000),
            PeripheralTypeParam("max_pulse_us", "Max pulse (µs)", "int", 2500, min=100, max=3000),
        ],
    ),
    "neopixel": PeripheralType(
        id="neopixel", label="NeoPixel (RGB)",
        description="WS2812 RGB LED, typically hardwired on the board.",
        pin_kind="builtin",
        channels=[
            PeripheralChannel("color",      "Color (RGB)", "out", "rgb"),
            PeripheralChannel("brightness", "Brightness",  "out", "analog"),
        ],
        params=[
            PeripheralTypeParam("default_color", "Idle color (hex)", "string", "#1e293b"),
        ],
        builtin_only=True,
    ),
    "fas100": PeripheralType(
        id="fas100", label="FAS100",
        description="FrSky FAS100 ADV current/voltage/temperature sensor over S.Port UART.",
        pin_kind="uart",
        channels=[
            PeripheralChannel("amps",  "Current (A)", "in", "analog"),
            PeripheralChannel("volts", "Voltage (V)", "in", "analog"),
            PeripheralChannel("temp1", "Temp 1 (°C)", "in", "analog"),
            PeripheralChannel("temp2", "Temp 2 (°C)", "in", "analog"),
        ],
        params=[
            PeripheralTypeParam("poll_interval_ms", "Poll interval (ms)", "int", 50, min=20, max=1000),
        ],
    ),
    "syren": PeripheralType(
        id="syren", label="SyRen Motor",
        description="Sabertooth SyRen motor controller over packetized serial.",
        pin_kind="uart",
        channels=[PeripheralChannel("motor", "Motor power", "out", "analog")],
        params=[
            PeripheralTypeParam("address",  "Address",  "int", 128, min=128, max=135),
            PeripheralTypeParam("deadband", "Deadband", "int",   3, min=0,   max=127),
        ],
    ),
    "roboclaw": PeripheralType(
        id="roboclaw", label="RoboClaw Motor",
        description="RoboClaw Solo 60A motor controller with telemetry.",
        pin_kind="uart",
        channels=[
            PeripheralChannel("motor",   "Motor duty",     "out", "analog"),
            PeripheralChannel("encoder", "Encoder",        "in",  "analog"),
            PeripheralChannel("voltage", "Bus voltage",    "in",  "analog"),
            PeripheralChannel("current", "Motor current",  "in",  "analog"),
            PeripheralChannel("temp",    "Temperature",    "in",  "analog"),
        ],
        params=[
            PeripheralTypeParam("address",        "Address",         "int", 128, min=128, max=135),
            PeripheralTypeParam("deadband",       "Deadband",        "int",   0, min=0,   max=127),
            PeripheralTypeParam("max_current_ma", "Max current (mA)","int",   0, min=0,   max=60000),
            # Optional GPIO the PCB wires from this MCU to the
            # RoboClaw's S3 (E-stop) input. When set (1..29), the
            # firmware drives the pin LOW (deasserted) the moment
            # the peripheral config is applied or restored from
            # flash, so a floating boot-time signal can't trip the
            # controller's latching S3 E-stop input. Leave at 0
            # (the default) on PCBs that don't route an E-stop wire
            # — in that case the operator must configure S3 to a
            # non-latching mode on the RoboClaw side or the
            # controller will silently lock out motor commands the
            # first time the MCU resets.
            PeripheralTypeParam("estop_pin", "E-stop pin", "gpio", 0, min=0, max=29),
            # When True, firmware uses a PIO-based UART so the TX/RX
            # GPIOs can be any pin (not the silicon-fixed UART0
            # GP0/GP1 pair). Set this on custom PCBs where the wires
            # to the controller's S1/S2 are routed such that the
            # MCU's expected TX pin is physically wired to the
            # controller's TX (and vice versa) — the hardware UART
            # can't swap directions; PIO can. Default False: use the
            # hardware UART exactly as before.
            PeripheralTypeParam("uart_swap", "Use PIO UART (swap TX/RX)", "bool", False),
        ],
    ),
    "pathfinder_bms": PeripheralType(
        id="pathfinder_bms", label="Pathfinder BMS",
        description="JBD-compatible battery management system.",
        pin_kind="uart",
        channels=[
            PeripheralChannel("pack_voltage", "Pack voltage",   "in", "analog"),
            PeripheralChannel("current",      "Current",        "in", "analog"),
            PeripheralChannel("soc",          "State of charge","in", "analog"),
            PeripheralChannel("temp_1",       "Temp 1",         "in", "analog"),
            PeripheralChannel("temp_2",       "Temp 2",         "in", "analog"),
        ],
        params=[
            PeripheralTypeParam("poll_interval_ms", "Poll interval (ms)", "int", 1000, min=100, max=10000),
        ],
    ),
    "system_monitor": PeripheralType(
        id="system_monitor", label="System Monitor",
        description="Host controller telemetry: CPU/memory usage, temperature, throttle status, uptime.",
        pin_kind="builtin",
        channels=[
            PeripheralChannel("cpu_usage", "CPU usage (%)",    "in", "analog"),
            PeripheralChannel("cpu_temp",  "CPU temp (°C)",    "in", "analog"),
            PeripheralChannel("mem_usage", "Memory usage (%)", "in", "analog"),
            PeripheralChannel("throttle",  "Throttled",        "in", "digital_in"),
            PeripheralChannel("uptime",    "Uptime (s)",       "in", "analog"),
        ],
        params=[],
        builtin_only=True,
    ),
}


# ─────────────────────────────────────────────────────────────────────────────
# Per-node state
# ─────────────────────────────────────────────────────────────────────────────


@dataclass
class PeripheralInstance:
    """One peripheral attached to a specific node."""
    id: str
    type: str
    label: str
    pins: Dict[str, int] = field(default_factory=dict)
    params: Dict[str, Any] = field(default_factory=dict)
    builtin: bool = False

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "PeripheralInstance":
        return cls(
            id=d["id"],
            type=d["type"],
            label=d.get("label", d["id"]),
            pins=dict(d.get("pins", {})),
            params=dict(d.get("params", {})),
            builtin=bool(d.get("builtin", False)),
        )

    def channels(self, catalog: Dict[str, PeripheralType]) -> List[PeripheralChannel]:
        t = catalog.get(self.type)
        return list(t.channels) if t else []


@dataclass
class NodePeripheralConfig:
    """Per-node config: just the list of peripherals attached.

    Version increments on any change, used to detect drift between the
    server's view and what the node has running. sync_status reports
    whether the firmware is currently running the same config.
    """
    version: int = 0
    peripherals: List[PeripheralInstance] = field(default_factory=list)
    sync_status: str = "unknown"   # unknown | pending | synced | error
    last_synced: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "version": self.version,
            "peripherals": [p.to_dict() for p in self.peripherals],
            "sync_status": self.sync_status,
            "last_synced": self.last_synced,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "NodePeripheralConfig":
        return cls(
            version=int(d.get("version", 0)),
            peripherals=[PeripheralInstance.from_dict(p) for p in d.get("peripherals", [])],
            sync_status=d.get("sync_status", "unknown"),
            last_synced=d.get("last_synced"),
        )

    def get(self, peripheral_id: str) -> Optional[PeripheralInstance]:
        for p in self.peripherals:
            if p.id == peripheral_id:
                return p
        return None

    def upsert(self, peripheral: PeripheralInstance) -> None:
        for i, existing in enumerate(self.peripherals):
            if existing.id == peripheral.id:
                self.peripherals[i] = peripheral
                self.version += 1
                self.sync_status = "pending"
                return
        self.peripherals.append(peripheral)
        self.version += 1
        self.sync_status = "pending"

    def remove(self, peripheral_id: str) -> bool:
        before = len(self.peripherals)
        self.peripherals = [p for p in self.peripherals
                            if not (p.id == peripheral_id and not p.builtin)]
        if len(self.peripherals) < before:
            self.version += 1
            self.sync_status = "pending"
            return True
        return False


# ─────────────────────────────────────────────────────────────────────────────
# System-wide state (routes + widgets)
# ─────────────────────────────────────────────────────────────────────────────


@dataclass
class RouteEndpoint:
    """One end of a route.

    kind == "peripheral": parts = [node_id, peripheral_id, channel_id]
    kind == "signal":     parts = [signal_path]
    kind == "widget":     parts = [widget_id, input_id]
    """
    kind: str
    parts: List[str]

    def to_dict(self) -> Dict[str, Any]:
        return {"kind": self.kind, "parts": list(self.parts)}

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "RouteEndpoint":
        return cls(kind=d["kind"], parts=list(d.get("parts", [])))

    def describe(self) -> str:
        return f"{self.kind}:{'/'.join(self.parts)}"


@dataclass
class Route:
    id: str
    source: RouteEndpoint
    sink: RouteEndpoint

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "source": self.source.to_dict(),
            "sink": self.sink.to_dict(),
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "Route":
        return cls(
            id=d["id"],
            source=RouteEndpoint.from_dict(d["source"]),
            sink=RouteEndpoint.from_dict(d["sink"]),
        )


@dataclass
class WidgetInstance:
    """A dashboard widget — a sink for one or more peripheral channels."""
    id: str
    type: str
    label: str
    position: Tuple[int, int] = (0, 0)
    params: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "type": self.type,
            "label": self.label,
            "position": list(self.position),
            "params": dict(self.params),
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "WidgetInstance":
        pos = d.get("position") or [0, 0]
        return cls(
            id=d["id"],
            type=d["type"],
            label=d.get("label", d["id"]),
            position=(int(pos[0]) if len(pos) > 0 else 0,
                      int(pos[1]) if len(pos) > 1 else 0),
            params=dict(d.get("params", {})),
        )


# ─────────────────────────────────────────────────────────────────────────────
# Widget catalog
# ─────────────────────────────────────────────────────────────────────────────


@dataclass
class WidgetInputSpec:
    id: str
    display: str
    cap: str          # which channel capability tags can route into this input

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class WidgetType:
    id: str
    label: str
    description: str
    inputs: List[WidgetInputSpec] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "label": self.label,
            "description": self.description,
            "inputs": [i.to_dict() for i in self.inputs],
        }


DEFAULT_WIDGET_CATALOG: Dict[str, WidgetType] = {
    "battery_monitor": WidgetType(
        id="battery_monitor", label="Battery Monitor",
        description="Current, voltage, and up to two temperature readings.",
        inputs=[
            WidgetInputSpec("current", "Current", "analog"),
            WidgetInputSpec("voltage", "Voltage", "analog"),
            WidgetInputSpec("temp1",   "Temp 1",  "analog"),
            WidgetInputSpec("temp2",   "Temp 2",  "analog"),
        ],
    ),
    "estop_indicator": WidgetType(
        id="estop_indicator", label="E-Stop Indicator",
        description="Lights up when an e-stop signal goes high.",
        inputs=[WidgetInputSpec("triggered", "Triggered", "digital_in")],
    ),
    "single_gauge": WidgetType(
        id="single_gauge", label="Single-value Gauge",
        description="One numeric reading with a meter.",
        inputs=[WidgetInputSpec("value", "Value", "analog")],
    ),
    "status_led_indicator": WidgetType(
        id="status_led_indicator", label="Status Indicator",
        description="Reflects a digital signal as an on-screen LED.",
        inputs=[WidgetInputSpec("state", "State", "digital_in")],
    ),
}


# ─────────────────────────────────────────────────────────────────────────────
# System routing aggregate (routes + widgets + version)
# ─────────────────────────────────────────────────────────────────────────────


@dataclass
class SystemRouting:
    version: int = 0
    routes: List[Route] = field(default_factory=list)
    widgets: List[WidgetInstance] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "version": self.version,
            "routes":  [r.to_dict() for r in self.routes],
            "widgets": [w.to_dict() for w in self.widgets],
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "SystemRouting":
        return cls(
            version=int(d.get("version", 0)),
            routes=[Route.from_dict(r) for r in d.get("routes", [])],
            widgets=[WidgetInstance.from_dict(w) for w in d.get("widgets", [])],
        )

    # ── Routes ────────────────────────────────────────────────────────

    def _next_route_id(self) -> str:
        existing = {r.id for r in self.routes}
        n = len(self.routes) + 1
        while f"r{n}" in existing:
            n += 1
        return f"r{n}"

    def add_route(self, source: RouteEndpoint, sink: RouteEndpoint,
                  route_id: Optional[str] = None) -> Route:
        rid = route_id or self._next_route_id()
        route = Route(id=rid, source=source, sink=sink)
        self.routes.append(route)
        self.version += 1
        return route

    def update_route(self, route_id: str, source: RouteEndpoint,
                     sink: RouteEndpoint) -> Optional[Route]:
        for r in self.routes:
            if r.id == route_id:
                r.source = source
                r.sink = sink
                self.version += 1
                return r
        return None

    def remove_route(self, route_id: str) -> bool:
        before = len(self.routes)
        self.routes = [r for r in self.routes if r.id != route_id]
        if len(self.routes) < before:
            self.version += 1
            return True
        return False

    def routes_touching_peripheral(self, node_id: str, peripheral_id: str) -> List[Route]:
        out = []
        for r in self.routes:
            for ep in (r.source, r.sink):
                if ep.kind == "peripheral" and len(ep.parts) >= 2 \
                        and ep.parts[0] == node_id and ep.parts[1] == peripheral_id:
                    out.append(r)
                    break
        return out

    def routes_touching_widget(self, widget_id: str) -> List[Route]:
        out = []
        for r in self.routes:
            for ep in (r.source, r.sink):
                if ep.kind == "widget" and ep.parts and ep.parts[0] == widget_id:
                    out.append(r)
                    break
        return out

    # ── Widgets ───────────────────────────────────────────────────────

    def _next_widget_id(self, type_id: str) -> str:
        existing = {w.id for w in self.widgets}
        n = sum(1 for w in self.widgets if w.type == type_id) + 1
        while f"{type_id}-{n}" in existing:
            n += 1
        return f"{type_id}-{n}"

    def add_widget(self, type_id: str, label: str,
                   position: Tuple[int, int] = (0, 0),
                   widget_id: Optional[str] = None,
                   params: Optional[Dict[str, Any]] = None) -> WidgetInstance:
        wid = widget_id or self._next_widget_id(type_id)
        w = WidgetInstance(id=wid, type=type_id, label=label,
                           position=position, params=dict(params or {}))
        self.widgets.append(w)
        self.version += 1
        return w

    def update_widget(self, widget_id: str, **changes) -> Optional[WidgetInstance]:
        for w in self.widgets:
            if w.id == widget_id:
                for k, v in changes.items():
                    if hasattr(w, k):
                        setattr(w, k, v)
                self.version += 1
                return w
        return None

    def remove_widget(self, widget_id: str) -> bool:
        before = len(self.widgets)
        # Cascade: drop routes touching this widget
        self.routes = [r for r in self.routes
                       if not any(ep.kind == "widget" and ep.parts and ep.parts[0] == widget_id
                                  for ep in (r.source, r.sink))]
        self.widgets = [w for w in self.widgets if w.id != widget_id]
        if len(self.widgets) < before:
            self.version += 1
            return True
        return False


# ─────────────────────────────────────────────────────────────────────────────
# Validation
# ─────────────────────────────────────────────────────────────────────────────


def detect_pin_conflicts(config: NodePeripheralConfig,
                         uart_pairs: Optional[List[Dict[str, int]]] = None
                         ) -> List[str]:
    """Return a list of human-readable conflict messages, or empty."""
    errors: List[str] = []
    by_pin: Dict[int, PeripheralInstance] = {}
    for p in config.peripherals:
        for pin in p.pins.values():
            if pin in by_pin:
                other = by_pin[pin]
                errors.append(
                    f"Pin GP{pin}: claimed by both '{other.label}' and '{p.label}'"
                )
            else:
                by_pin[pin] = p

    # UART conflicts: two peripherals on the same UART instance.
    if uart_pairs:
        # Map (tx, rx) -> uart instance
        pair_index = {(u["tx"], u["rx"]): u["uart"] for u in uart_pairs}
        seen: Dict[int, PeripheralInstance] = {}
        for p in config.peripherals:
            tx = p.pins.get("uart_tx")
            rx = p.pins.get("uart_rx")
            if tx is None or rx is None:
                continue
            uart_instance = pair_index.get((tx, rx))
            if uart_instance is None:
                continue
            if uart_instance in seen and seen[uart_instance].id != p.id:
                other = seen[uart_instance]
                errors.append(
                    f"UART{uart_instance}: '{other.label}' and '{p.label}' both claim it"
                )
            else:
                seen[uart_instance] = p
    return errors


def validate_route_compatibility(source_cap: str, sink_cap: str) -> bool:
    """Return True if a value of `source_cap` can flow into a `sink_cap`.

    Currently a permissive exact-match-or-analog check. Easy to harden
    later (range checks, unit conversion, etc.).
    """
    if source_cap == sink_cap:
        return True
    # 'any' is the wildcard used by logical-signal endpoints
    if source_cap == "any" or sink_cap == "any":
        return True
    # Analog signals can drive analog sinks regardless of finer subtype
    if source_cap == "analog" and sink_cap == "analog":
        return True
    return False
