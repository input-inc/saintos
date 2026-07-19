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
from typing import Any, Dict, List, Optional, Set, Tuple

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
    # When set, the modal renders this param as a dropdown instead of
    # a freeform input. Each entry is either a scalar (used as both
    # value and label) or a {value, label} dict. Works for any base
    # `type` — the value gets coerced through the type's normal path.
    choices: Optional[List[Any]] = None
    # Inline help text rendered under the input in the Add/Edit
    # Peripheral modal. Keep it to 1–2 sentences — anything longer
    # belongs in a docs page.
    help: Optional[str] = None
    # Conditional visibility predicate. `{other_param_id: value, ...}`
    # — the field is hidden in the modal unless every named param has
    # the matching value. Example: a `mac` field with
    # `visible_when={"transport": "ble"}` only appears when the
    # operator has picked the BLE transport. Used by the BMS catalog
    # entry to swap UART pins for a MAC field. The hidden field's
    # value is also stripped from the saved payload so it doesn't
    # confuse driver-side config parsers.
    visible_when: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        # Drop unset optional fields so the JSON stays compact.
        if d["min"] is None:
            d.pop("min")
        if d["max"] is None:
            d.pop("max")
        if d["choices"] is None:
            d.pop("choices")
        if d["help"] is None:
            d.pop("help")
        if d["visible_when"] is None:
            d.pop("visible_when")
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
    # Key the single-pin picker writes into the instance's `pins` map.
    # Defaults to "gpio" (Button, LED, Servo, …). NeoPixel uses "data"
    # so the WS2812 data line is named correctly and matches the slot
    # the board YAML seeds the onboard one with — added and onboard
    # NeoPixels then carry the SAME pin key, so a driver reads one slot.
    pin_slot: str = "gpio"

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "label": self.label,
            "description": self.description,
            "pin_kind": self.pin_kind,
            "pin_slot": self.pin_slot,
            "channels": [c.to_dict() for c in self.channels],
            "params": [p.to_dict() for p in self.params],
            "builtin_only": self.builtin_only,
        }


# ── Pololu Maestro ───────────────────────────────────────────────────
#
# One catalog entry, one driver. The `channel_count` param picks
# which of the four Pololu SKUs the operator wired up (6 / 12 / 18 /
# 24). The catalog exposes all 24 channels statically — UI surfaces
# (Peripherals modal, Routing canvas, Live view) filter the list down
# to `params.channel_count` at render time. See visible_channels()
# below — driver-side code should iterate it instead of the raw type
# channel list.
#
# See server/docs/MAESTRO_PROTOCOL.md for wire-format details. The
# protocol is identical across SKUs; only Mini Maestro 12/18/24
# support Set Multiple Targets (0x9F) for bulk updates.

_MAESTRO_MAX_CHANNELS = 24


def visible_maestro_channels(peripheral) -> int:
    """How many of the type's static 24 channels are 'live' on this
    Maestro instance. Drivers / UI consumers iterate the first N
    entries of ``type.channels``. Falls back to 6 for forward-compat
    if the param is unset.
    """
    n = int(peripheral.params.get("channel_count", 6) or 6)
    if n not in (6, 12, 18, 24):
        n = 6
    return n


# Per-channel config schema lives in params["channels"] as a list of
# dicts (length == channel_count). Keeping it in params keeps the
# storage layer simple — no schema migration, no new top-level
# PeripheralInstance field — and lets the existing
# `save_node_peripheral` WebSocket carry channel edits without a new
# message type. The downside is the channels list isn't described by
# PeripheralTypeParam, so validation lives here instead of the generic
# param validator. See _maestro_normalize_channels below.
#
# Field names match the firmware-side maestro_channel_config_t
# (firmware/shared/include/maestro_driver.h) and the Python Pi driver
# (firmware/raspberrypi/saint_node/peripherals/maestro.py) so the
# server-to-firmware push needs no key translation. The "speed" and
# "acceleration" fields are conceptually *defaults* applied on Pose
# transitions only — see TODO in maestro_driver.c — but stored under
# the bare firmware names.
_MAESTRO_CHANNEL_KEYS = (
    "min_pulse_us", "max_pulse_us", "neutral_us", "home_us",
    "speed", "acceleration", "idle_disengage_ms",
)
# Channel fields the dashboard stores but the firmware never reads —
# kept in the in-memory model + on-disk YAML for display only, never on
# the wire. The 2 KB XRCE-DDS reassembly cap is real (memory:
# feedback_xrce_wire_size_budget) and an operator with all 24 channels
# named "Right Top Flap Rotation"-style burns ~600 bytes on labels
# alone — enough on its own to push the Maestro config over the cap
# and crash the firmware mid-apply. "icon" is also stored on the model
# but already excluded from _MAESTRO_CHANNEL_KEYS for the same reason.
_MAESTRO_CHANNEL_DISPLAY_ONLY_KEYS = ("label", "icon")


def _maestro_default_channel(idx: int, peripheral_params: Dict[str, Any]) -> Dict[str, Any]:
    """Build a single default channel entry, sourcing min/max/neutral
    from the peripheral-level Advanced fallbacks so a brand-new Maestro
    inherits whatever the operator set in the main edit modal."""
    min_pulse_us = int(peripheral_params.get("min_pulse_us", 1000))
    max_pulse_us = int(peripheral_params.get("max_pulse_us", 2000))
    # Guard against operator-flipped order; the UI clamps but be safe.
    if min_pulse_us > max_pulse_us:
        min_pulse_us, max_pulse_us = max_pulse_us, min_pulse_us
    neutral_us = (min_pulse_us + max_pulse_us) // 2
    return {
        "label": f"Ch {idx}",
        # Material-icons ligature name (e.g. "open_with", "pan_tool") for
        # the State view so channels are visually distinguishable.
        # Display-only — deliberately NOT in _MAESTRO_CHANNEL_KEYS, so it
        # never goes on the firmware wire. Empty = no icon.
        "icon": "",
        "min_pulse_us": min_pulse_us,
        "max_pulse_us": max_pulse_us,
        "neutral_us": neutral_us,
        # home_us defaults to neutral; operator can change in the
        # channel-edit modal. Conceptually distinct from the Maestro
        # EEPROM HomeMode/HOME — see docs/MAESTRO_BRINGUP.md. 0 here
        # is the firmware sentinel for "unconfigured, skip auto-home"
        # (see maestro_apply_home_positions); we default to neutral so
        # configured channels do auto-home.
        "home_us": neutral_us,
        # 0 = unlimited. Only applied on Pose transitions (deferred to
        # Pose editor work); animation input snaps at full speed.
        "speed": 0,
        "acceleration": 0,
        # 0 = always engaged. >0 = after N ms of unchanged target, the
        # firmware writes SET_TARGET=0 (kills PWM) so the servo goes
        # limp and stops the idle-correction whine. The next control
        # value restores PWM. Set per-channel because load-bearing
        # joints (arms, anything fighting a spring) must stay armed,
        # but cosmetic ones (eyes, eyebrows, mouth flaps) can safely
        # release.
        "idle_disengage_ms": 0,
    }


def _maestro_sanitize_channel(
    raw: Dict[str, Any], idx: int, peripheral_params: Dict[str, Any]
) -> Dict[str, Any]:
    """Clamp + sanitize one channel dict from operator/firmware input.
    Unknown keys are stripped; missing keys fall back to defaults. The
    pulse-width fields are clamped to [64, 3200] to match the
    peripheral-level Advanced params' range."""
    default = _maestro_default_channel(idx, peripheral_params)
    out: Dict[str, Any] = {}
    label = raw.get("label", default["label"])
    out["label"] = str(label)[:32] if label else default["label"]
    # Material-icons ligature name — restrict to the charset those names
    # use (lowercase ascii + digits + underscore) so a stray value can't
    # inject markup into the State view. Empty = no icon.
    _icon_ok = set("abcdefghijklmnopqrstuvwxyz0123456789_")
    raw_icon = str(raw.get("icon", default.get("icon", "")) or "").lower()
    out["icon"] = "".join(c for c in raw_icon if c in _icon_ok)[:40]
    for key in ("min_pulse_us", "max_pulse_us", "neutral_us", "home_us"):
        try:
            v = int(raw.get(key, default[key]))
        except (TypeError, ValueError):
            v = default[key]
        out[key] = max(64, min(3200, v))
    # Defense in depth: keep min ≤ max even if operator typed them in
    # the wrong order; UI clamps too but server should be the source
    # of truth.
    if out["min_pulse_us"] > out["max_pulse_us"]:
        out["min_pulse_us"], out["max_pulse_us"] = out["max_pulse_us"], out["min_pulse_us"]
    # Clamp neutral / home into the per-channel range so a runtime
    # SET_TARGET = home_us never triggers the Maestro's MIN/MAX clamp.
    out["neutral_us"] = max(out["min_pulse_us"], min(out["max_pulse_us"], out["neutral_us"]))
    out["home_us"]    = max(out["min_pulse_us"], min(out["max_pulse_us"], out["home_us"]))
    try:
        out["speed"] = max(0, min(65_535, int(raw.get("speed", 0))))
    except (TypeError, ValueError):
        out["speed"] = 0
    try:
        out["acceleration"] = max(0, min(255, int(raw.get("acceleration", 0))))
    except (TypeError, ValueError):
        out["acceleration"] = 0
    # Cap at 10 minutes — past that the firmware is functionally "never
    # disengage" anyway, and capping keeps the JSON-wire field a tight
    # uint32 in practice. Lower bound 0 = disabled (always engaged).
    try:
        out["idle_disengage_ms"] = max(0, min(600_000, int(raw.get("idle_disengage_ms", 0))))
    except (TypeError, ValueError):
        out["idle_disengage_ms"] = 0
    return out


def maestro_normalize_channels(params: Dict[str, Any]) -> None:
    """Ensure params["channels"] is a list whose length matches
    params["channel_count"], with sanitized entries throughout.

    Called from state_manager.upsert_node_peripheral right after the
    PeripheralInstance is built — so saving a Maestro with no channels
    array (legacy configs, or a fresh add) populates defaults, and an
    operator who changed channel_count from 6 → 24 gets the extra 18
    channels filled in. Modifies the dict in-place.
    """
    count = int(params.get("channel_count", 6) or 6)
    if count not in (6, 12, 18, 24):
        count = 6
    raw_channels = params.get("channels")
    if not isinstance(raw_channels, list):
        raw_channels = []
    normalized: List[Dict[str, Any]] = []
    for i in range(count):
        src = raw_channels[i] if i < len(raw_channels) and isinstance(raw_channels[i], dict) else {}
        normalized.append(_maestro_sanitize_channel(src, i, params))
    params["channels"] = normalized


def maestro_slim_channels_for_wire(params: Dict[str, Any]) -> Dict[str, Any]:
    """Return a copy of `params` where every channel that exactly
    matches its default gets replaced with an empty `{}`. The full
    Maestro 24-channel config serializes to ~3500 bytes if every
    channel is emitted in full — past both the firmware's
    config_buffer AND the XRCE-DDS UDP MTU fragmentation budget
    (4 × 512 ≈ 2K reassembly cap with default RMW_UXRCE_MAX_HISTORY).
    The receiving Teensy crashes when the reassembled message
    overruns those caps.

    Trick: a channel that's identical to what
    `_maestro_default_channel(idx, params)` would produce can be
    emitted as `{}` on the wire. The firmware parser walks the
    channels array by brace-counting; an empty `{}` still occupies
    the channel's slot, and the per-channel field lookups find
    nothing → channel inherits peripheral-level defaults (which is
    exactly what "all default" means anyway).

    Typical Maestro just-added (24 defaulted channels): drops from
    ~3500 → ~600 bytes. Operator with 3 customized channels: ~1000
    bytes. Operator with all 24 customized: still ~3500 — at that
    point bump config_buffer / rebuild libmicroros with larger
    history.

    Pure function: does not mutate `params`. Returns a new dict
    with the same keys; `channels` is a fresh list of either {} or
    the full channel dict.
    """
    if "channels" not in params or not isinstance(params["channels"], list):
        return dict(params)
    out = dict(params)
    slim: List[Dict[str, Any]] = []
    for i, ch in enumerate(params["channels"]):
        if not isinstance(ch, dict):
            slim.append({})
            continue
        default = _maestro_default_channel(i, params)
        # Per-field diff: emit only the fields that differ from the
        # default. A channel where every field matches default
        # becomes `{}` (just a placeholder for parser index counting).
        # A channel with one customized home_us becomes
        # `{"home_us": <val>}`. This keeps even worst-case payloads
        # under the XRCE-DDS reassembly cap. The firmware parser
        # falls back to peripheral-level defaults for any field not
        # present in the channel object.
        diff: Dict[str, Any] = {}
        for k in _MAESTRO_CHANNEL_KEYS:
            # ch.get(k) returns None for fields the saved YAML predates
            # (e.g. idle_disengage_ms on a config saved before that
            # field shipped). None != 0 would emit `"key":null` on the
            # wire for every channel — pure waste (the firmware parses
            # "null" as 0 anyway, and 24×26 bytes of "null" alone is
            # 624 bytes, enough to bust the XRCE-DDS reassembly cap on
            # its own). Treat missing == default.
            v = ch.get(k)
            if v is None:
                continue
            if v != default[k]:
                diff[k] = v
        slim.append(diff)
    out["channels"] = slim
    return out


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
        description=(
            "Hobby servo on a PWM-capable pin. The operator-facing "
            "channel `angle` accepts a normalized −1..+1 signal from the "
            "routing sheet; the servo peripheral itself maps that to a "
            "pulse width using the four extents below. Start and End "
            "define what −1 and +1 map to; Center is where 0 lands "
            "(allowing asymmetric mechanical limits); Home is the "
            "startup / safe-reset pulse driven before any command "
            "arrives. All values are pulse-width microseconds at the "
            "standard 50 Hz servo frame rate."
        ),
        pin_kind="pwm",
        channels=[PeripheralChannel("angle", "Angle (−1..+1)", "out", "analog")],
        params=[
            PeripheralTypeParam(
                "start_us",  "Start (−1) pulse (µs)",  "int", 1000,
                min=500, max=2500,
                help="Pulse width sent when the routed signal is −1."),
            PeripheralTypeParam(
                "end_us",    "End (+1) pulse (µs)",    "int", 2000,
                min=500, max=2500,
                help="Pulse width sent when the routed signal is +1."),
            PeripheralTypeParam(
                "center_us", "Center (0) pulse (µs)",  "int", 1500,
                min=500, max=2500,
                help="Pulse width sent when the routed signal is 0. "
                     "Need not be the midpoint of Start/End — "
                     "asymmetric mechanical ranges are supported."),
            PeripheralTypeParam(
                "home_us",   "Startup / home pulse (µs)", "int", 1500,
                min=500, max=2500,
                help="Pulse the servo is driven to immediately after "
                     "config apply and on safe-reset, before any "
                     "operator command arrives."),
        ],
    ),
    "pwm": PeripheralType(
        id="pwm", label="PWM Output",
        description=(
            "Generic PWM output on a PWM-capable pin. Independent "
            "frequency and duty cycle — handy for driving LED dimmers, "
            "ESCs, fan controllers, or any device that takes a raw PWM "
            "signal. The `duty` channel accepts 0.0–1.0; map a routing "
            "source into it via a Map Range operator if your source's "
            "natural range is different."
        ),
        pin_kind="pwm",
        channels=[PeripheralChannel("duty", "Duty cycle (0–1)", "out", "analog")],
        params=[
            PeripheralTypeParam("frequency_hz", "Frequency (Hz)",  "int",  1000, min=1,  max=200_000),
            PeripheralTypeParam("initial_duty", "Initial duty (0–1)", "float", 0.0, min=0.0, max=1.0),
            # Invert is rare for PWM but matches the Servo + RoboClaw
            # ergonomics in this catalog — useful for active-low PWM
            # consumers (some MOSFET drivers).
            PeripheralTypeParam("invert",       "Invert output",     "bool", False),
        ],
    ),
    "neopixel": PeripheralType(
        id="neopixel", label="NeoPixel (RGB)",
        description=(
            "WS2812 / NeoPixel RGB LED(s) on a single data pin. Boards "
            "with an onboard NeoPixel seed one automatically as a "
            "built-in; operators can also add external NeoPixels (single "
            "LED or a strip) on any free GPIO. The `color` and "
            "`brightness` channels drive every pixel in the instance."
        ),
        # Data line is a single digital-output GPIO — surfaced through
        # the same single-pin picker as Button/LED, but stored under the
        # "data" slot (see pin_slot) to match the onboard seed.
        pin_kind="gpio",
        pin_slot="data",
        channels=[
            PeripheralChannel("color",      "Color (RGB)", "out", "rgb"),
            PeripheralChannel("brightness", "Brightness",  "out", "analog"),
        ],
        params=[
            PeripheralTypeParam(
                "pixel_count", "Pixel count", "int", 1, min=1, max=300,
                help="Number of WS2812 LEDs on this data line. The "
                     "onboard NeoPixel is 1; a strip can be longer."),
            PeripheralTypeParam("default_color", "Idle color (hex)", "string", "#1e293b"),
        ],
    ),
    # Single-color onboard LED, typically hardwired on the board (e.g.
    # Teensy 4.1 pin 13). Distinct from `neopixel` because the LED has
    # no hue — exposing a color picker would just confuse operators
    # (any non-black collapses to "on"). Brightness IS preserved as a
    # channel: on PWM-capable pins the firmware drives analogWrite,
    # giving real dimming; on plain digital pins, brightness >0 maps to
    # HIGH and brightness 0 maps to LOW. The pin's PWM capability is
    # part of the platform, not the catalog metadata, so this type
    # works for both cases.
    "mono_led": PeripheralType(
        id="mono_led", label="Onboard LED (single color)",
        description=(
            "Single-color LED hardwired on the board. State + brightness "
            "control only — no color picker, because the LED is one fixed "
            "hue. On PWM-capable pins (Teensy 4.1 pin 13, RP2040 PWM "
            "slices) brightness produces real dimming; otherwise it's "
            "binary on/off."
        ),
        pin_kind="builtin",
        channels=[
            PeripheralChannel("state",      "On",          "out", "digital_out"),
            PeripheralChannel("brightness", "Brightness",  "out", "analog"),
        ],
        params=[],
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
    # Pololu Maestro servo controller. One peripheral type; the
    # ``channel_count`` param picks Micro / Mini Maestro size (6 /
    # 12 / 18 / 24). UI consumers and the firmware driver clip the
    # static 24-channel list to the operator-selected count via
    # ``visible_maestro_channels(p)`` above. Wire protocol is in
    # server/docs/MAESTRO_PROTOCOL.md.
    "maestro": PeripheralType(
        id="maestro", label="Pololu Maestro",
        description=(
            "Pololu Maestro USB / TTL servo controller. Pick the channel "
            "count to match your hardware (6 Micro, 12 / 18 / 24 Mini) "
            "and the transport — UART for TTL serial, or USB Host for "
            "MCUs with native USB host support (Teensy 4.1)."
        ),
        pin_kind="uart",
        channels=[
            PeripheralChannel(f"ch{i}", f"Channel {i}", "out", "analog")
            for i in range(_MAESTRO_MAX_CHANNELS)
        ] + [
            # Status channels the firmware-side driver
            # (firmware/shared/src/maestro_driver.c) polls at ~2 Hz and
            # emits via state_emit_channels. Drive the dashboard's
            # Live-Readings MaestroCard:
            #
            #   connected   — 1 if the Maestro is currently answering
            #                 GET_ERRORS / GET_MOVING_STATE round-trips;
            #                 0 if not (USB unplugged, UART silent,
            #                 wrong Serial Mode, …).
            #   error_flags — 16-bit Pololu error bitmap (see Pololu
            #                 Maestro User's Guide §6.4). Each bit
            #                 means a distinct fault; the UI decodes
            #                 into human-readable badges.
            #   moving      — 0 if every servo is at its target,
            #                 1 if at least one is still moving toward
            #                 it. Useful for confirming a Pose
            #                 transition is in flight vs. silently
            #                 dropped.
            PeripheralChannel("connected",   "Connected",   "in", "digital_in"),
            PeripheralChannel("error_flags", "Error flags", "in", "analog"),
            PeripheralChannel("moving",      "Moving",      "in", "digital_in"),
        ],
        params=[
            # Transport first — gates which sub-params and pin pickers
            # are relevant. UART is the default to preserve the
            # behavior pre-USB-Host configs expect. The Peripherals
            # modal's `pinsVisible` computed hides the UART pair
            # picker automatically when transport != "uart"; see
            # the same pattern on the BMS catalog entry. Firmware
            # parses these string values directly — keep "uart" /
            # "usb_host" in sync with maestro_driver.c parse_json_params.
            PeripheralTypeParam(
                "transport", "Transport", "string", "uart",
                choices=[
                    {"value": "uart",       "label": "UART (TTL serial)"},
                    {"value": "usb_cdc",    "label": "USB CDC (Compact Protocol)"},
                    {"value": "usb_vendor", "label": "USB Vendor (full features incl. EEPROM read)"},
                    # Legacy alias — older saves wrote "usb_host" before
                    # the rename to usb_cdc. Kept so existing configs
                    # still load; new UIs won't surface it.
                    {"value": "usb_host", "label": "USB Host (legacy — same as USB CDC)"},
                ],
                help=(
                    "UART speaks Pololu protocols over a TTL serial pair "
                    "(pick the pins below). USB CDC sends Compact Protocol "
                    "over the Maestro's CDC ACM endpoint — requires the "
                    "Maestro's Serial Mode = 'USB Dual Port' in MCC. USB "
                    "Vendor uses EP0 control transfers, works in any "
                    "Maestro Serial Mode, and supports reading per-channel "
                    "settings back from the Maestro's EEPROM. No pin pair "
                    "is needed for any USB transport."
                ),
            ),
            PeripheralTypeParam(
                "channel_count", "Channel count", "int", 6,
                choices=[
                    {"value":  6, "label": "Micro Maestro · 6 ch"},
                    {"value": 12, "label": "Mini Maestro · 12 ch"},
                    {"value": 18, "label": "Mini Maestro · 18 ch"},
                    {"value": 24, "label": "Mini Maestro · 24 ch"},
                ],
            ),
            # Pololu protocol device number — used when more than one
            # Maestro shares the same UART bus. 0 = Compact protocol.
            PeripheralTypeParam("device_number", "Device number (0–127)",
                                "int", 12, min=0, max=127),
            PeripheralTypeParam(
                "baud_rate", "Baud rate", "int", 9600,
                choices=[
                    {"value":   1200, "label":   "1200 bps"},
                    {"value":   2400, "label":   "2400 bps"},
                    {"value":   4800, "label":   "4800 bps"},
                    {"value":   9600, "label":   "9600 bps (Maestro default)"},
                    {"value":  19200, "label":  "19200 bps"},
                    {"value":  38400, "label":  "38400 bps"},
                    {"value":  57600, "label":  "57600 bps"},
                    {"value": 115200, "label": "115200 bps (auto-detect max)"},
                    {"value": 200000, "label": "200000 bps (fixed-baud max)"},
                ],
                # Baud is meaningless on USB Host — the connection is a
                # USB CDC pipe, not a UART. Hide it so the operator
                # doesn't think they need to set it.
                visible_when={"transport": "uart"},
                help=(
                    "Must match the rate configured in the Maestro Control "
                    "Center. The Maestro's auto-detect mode locks onto the "
                    "first 0xAA byte and supports up to 115200; fixed-baud "
                    "mode goes up to 200000."
                ),
            ),
            # Pulse-range maps. Operator-facing channel values are
            # 0.0–1.0; firmware maps into [min_pulse_us, max_pulse_us]
            # and sends to the Maestro × 4 (quarter-microsecond units).
            PeripheralTypeParam("min_pulse_us",  "Min pulse (µs)",
                                "int", 1000, min=64, max=3200),
            PeripheralTypeParam("max_pulse_us",  "Max pulse (µs)",
                                "int", 2000, min=64, max=3200),
            # 0.0 → min_pulse_us, 1.0 → max_pulse_us. Commanded on each
            # channel at init so unwired channels don't go limp.
            PeripheralTypeParam("idle_value",    "Idle value (0–1)",
                                "float", 0.5, min=0.0, max=1.0),
            # 0.25-µs / 10-ms units (Maestro's native). 0 = unlimited.
            PeripheralTypeParam("speed_limit",   "Speed limit (0 = off)",
                                "int", 0, min=0, max=65_535),
            # 0.25-µs / 10-ms / 80-ms units. 0 = unlimited.
            PeripheralTypeParam("accel_limit",   "Accel limit (0 = off)",
                                "int", 0, min=0, max=255),
            # Poll Get Position each tick to surface live setpoints
            # in the UI. Off by default — extra UART traffic the
            # operator rarely needs.
            PeripheralTypeParam(
                "poll_positions", "Poll live positions", "bool", False,
                help=(
                    "When on, the driver issues a Get Position query for "
                    "each channel every tick so the UI can show what the "
                    "Maestro is actually doing (servos honoring speed / "
                    "accel limits land here gradually, not instantly). "
                    "Off by default: at 9600 baud each query costs ≈3 ms, "
                    "so polling 24 channels is too expensive for a 100 Hz "
                    "control loop. Leave it off unless you're debugging."
                ),
            ),
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
            # Status telemetry (firmware Read Status, cmd 90), emitted on
            # the peripheral-first state channel — mirrors the Maestro:
            #   connected   — 1 while the unit answers polls.
            #   error_flags — canonical ROBOCLAW_FAULT_* bitmask the UI
            #                 decodes into fault badges (over-current,
            #                 over-temp, over/under-voltage, E-Stop,
            #                 driver fault, ...). 0 = no faults.
            PeripheralChannel("connected",   "Connected",   "in", "digital_in"),
            PeripheralChannel("error_flags", "Error flags", "in", "analog"),
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
            # When true, positive duty commands drive the motor in
            # reverse (and vice versa). Use when the motor wiring or
            # mechanical mounting inverts the operator-intended
            # direction and physically re-wiring is inconvenient.
            # Inversion happens at the wire level in the firmware;
            # encoder readings are NOT auto-inverted.
            PeripheralTypeParam("invert_direction", "Invert motor direction", "bool", False),
        ],
    ),
    "tic": PeripheralType(
        id="tic", label="Tic Stepper",
        description=(
            "Pololu Tic stepper motor controller (T834/T825/T249/36v4) "
            "over TTL serial with per-unit device ID. Up to 8 units share "
            "one UART. Step mode and current limit must be pre-configured "
            "via Pololu Tic Control Center over USB."
        ),
        pin_kind="uart",
        channels=[
            PeripheralChannel("target_position",  "Target position",  "out", "analog"),
            PeripheralChannel("target_velocity",  "Target velocity",  "out", "analog"),
            PeripheralChannel("current_position", "Current position", "in",  "analog"),
            PeripheralChannel("current_velocity", "Current velocity", "in",  "analog"),
            PeripheralChannel("vin_voltage",      "VIN voltage",      "in",  "analog"),
            PeripheralChannel("error_status",     "Error status",     "in",  "analog"),
        ],
        params=[
            # Pololu device number. Factory default 14; operators
            # change it via Tic Control Center to allow multi-drop.
            PeripheralTypeParam("address",        "Device ID",            "int", 14,    min=1, max=127),
            # Operator-facing scaling. The dashboard slider sends
            # [-1.0, 1.0]; firmware multiplies by these to get the
            # wire value sent to the Tic.
            PeripheralTypeParam("max_position",   "Max position (steps)", "int", 10000, min=1, max=2000000000),
            PeripheralTypeParam("max_speed_pps",  "Max speed (pulses/s)", "int", 1000,  min=1, max=50000),
        ],
    ),
    "tmc2208": PeripheralType(
        id="tmc2208", label="TMC2208 Stepper",
        description=(
            "Trinamic TMC2208 stepper amplifier with UART configuration "
            "and MCU-generated STEP/DIR pulses. Up to 4 axes per node, "
            "each with its own slave address (PCB MS1/MS2 strapping), "
            "STEP and DIR GPIOs, and Rsense. Motion is constant-velocity "
            "between target_position commands — no acceleration ramp."
        ),
        pin_kind="uart",
        channels=[
            PeripheralChannel("target_position",  "Target position",  "out", "analog"),
            PeripheralChannel("target_velocity",  "Target velocity",  "out", "analog"),
            PeripheralChannel("current_position", "Current position", "in",  "analog"),
            PeripheralChannel("error_flags",      "Error flags",      "in",  "analog"),
        ],
        params=[
            # PCB-side slave address strap (MS1/MS2). 0-3 supported.
            PeripheralTypeParam("address",         "Slave address",        "int", 0,     min=0, max=3),
            # GPIOs the MCU wires to the chip's STEP and DIR inputs.
            PeripheralTypeParam("step_pin",        "STEP pin",             "gpio", 0),
            PeripheralTypeParam("dir_pin",         "DIR pin",              "gpio", 0),
            # PCB-specific sense resistor. 110 = 0.11Ω (most boards),
            # 75 = 0.075Ω (high-current variants).
            PeripheralTypeParam("rsense_milliohm", "Rsense (mΩ)",          "int", 110,   min=20, max=500),
            # Chip-side motion characteristics.
            PeripheralTypeParam("microsteps",      "Microsteps",           "int", 16,    min=1, max=256),
            PeripheralTypeParam("run_current_ma",  "Run current (mA)",     "int", 500,   min=0, max=2000),
            PeripheralTypeParam("hold_current_ma", "Hold current (mA)",    "int", 200,   min=0, max=2000),
            PeripheralTypeParam("stealth_chop",    "StealthChop",          "bool", True),
            # Operator-facing scaling for set_value [-1,1].
            PeripheralTypeParam("max_position",    "Max position (steps)", "int", 10000, min=1, max=2000000000),
            PeripheralTypeParam("max_speed_pps",   "Max speed (pulses/s)", "int", 1000,  min=1, max=12500),
        ],
    ),
    "kangaroo": PeripheralType(
        id="kangaroo", label="Kangaroo X2",
        description=(
            "Dimension Engineering Kangaroo X2 self-tuning closed-loop "
            "motion controller (rides on a Sabertooth / SyRen power "
            "stage). One peripheral = one Kangaroo motor channel; add a "
            "second for the board's other channel. Speaks packet serial "
            "(binary, CRC-14) or simplified serial (ASCII). The channel "
            "must be tuned in DEScribe first — this driver does motion, "
            "not tuning."
        ),
        pin_kind="uart",
        channels=[
            PeripheralChannel("target_position",  "Target position",  "out", "analog"),
            PeripheralChannel("target_speed",     "Target speed",     "out", "analog"),
            PeripheralChannel("current_position", "Current position", "in",  "analog"),
            PeripheralChannel("current_speed",    "Current speed",    "in",  "analog"),
            PeripheralChannel("moving",           "Moving",           "in",  "digital_in"),
            PeripheralChannel("error_status",     "Error status",     "in",  "analog"),
        ],
        params=[
            # Board address (high bit set on the wire). Two Kangaroo
            # channels share one address; `channel` picks which.
            PeripheralTypeParam("address", "Address", "int", 128, min=128, max=135),
            PeripheralTypeParam(
                "channel", "Channel", "string", "1",
                choices=[
                    {"value": "1", "label": "1 — Motor 1 (independent)"},
                    {"value": "2", "label": "2 — Motor 2 (independent)"},
                    {"value": "D", "label": "D — Drive (mixed/differential)"},
                    {"value": "T", "label": "T — Turn (mixed/differential)"},
                ],
                help="Use 1/2 for independent mode, D/T for mixed "
                     "(tank-drive) mode. Must match how the Kangaroo was "
                     "tuned in DEScribe.",
            ),
            PeripheralTypeParam(
                "protocol", "Protocol", "string", "packet",
                choices=[
                    {"value": "packet", "label": "Packet serial (binary, CRC-14)"},
                    {"value": "simple", "label": "Simplified serial (ASCII)"},
                ],
                help="Packet is more robust and is the default. Simplified "
                     "serial is plain ASCII — easier to debug with a serial "
                     "terminal but unframed. Set the matching mode in "
                     "DEScribe / via the DIP switches.",
            ),
            PeripheralTypeParam(
                "home_on_start", "Home on start", "bool", False,
                help="Send a Home command after Start. Needed for "
                     "quadrature / limit-switch rigs before absolute moves "
                     "are accepted; not meaningful in mixed (D/T) mode.",
            ),
            # Operator-facing scaling. The dashboard sends [-1, 1]; the
            # firmware multiplies by these to get the value on the wire
            # in the Kangaroo's machine units (set via DEScribe `units`).
            PeripheralTypeParam("max_position", "Max position (units)", "int", 10000,
                                min=1, max=536870911),
            PeripheralTypeParam("max_speed",    "Max speed (units/s)",  "int", 1000,
                                min=1, max=536870911),
            PeripheralTypeParam(
                "baud", "Baud rate", "int", 9600,
                choices=[
                    {"value":   9600, "label": "9600 bps (default)"},
                    {"value":  19200, "label": "19200 bps"},
                    {"value":  38400, "label": "38400 bps"},
                    {"value": 115200, "label": "115200 bps"},
                ],
                help="Must match the rate set in DEScribe. The Kangaroo "
                     "has no autobaud — it listens at exactly this rate.",
            ),
        ],
    ),
    "pathfinder_bms": PeripheralType(
        id="pathfinder_bms", label="Pathfinder BMS",
        description="JBD-compatible battery management system. "
                    "Connect via UART (one BMS per UART pair) or "
                    "Bluetooth Low Energy (multiple BMSes per Pi).",
        # pin_kind="uart" so the modal renders the UART pin pair
        # selector. We hide those pin widgets via visible_when on the
        # frontend when transport=ble — the driver ignores them in
        # that case.
        pin_kind="uart",
        channels=[
            PeripheralChannel("pack_voltage", "Pack voltage",   "in", "analog"),
            PeripheralChannel("current",      "Current",        "in", "analog"),
            PeripheralChannel("soc",          "State of charge","in", "analog"),
            PeripheralChannel("temp_1",       "Temp 1",         "in", "analog"),
            PeripheralChannel("temp_2",       "Temp 2",         "in", "analog"),
            PeripheralChannel("protection",   "Protection bits","in", "analog"),
            PeripheralChannel("fet_status",   "FET status",     "in", "analog"),
            PeripheralChannel("remain_cap",   "Remaining cap",  "in", "analog"),
            PeripheralChannel("cycles",       "Cycle count",    "in", "analog"),
            # Number of series cells the BMS reports it has wired.
            # Drives cell-bar visibility in the BMS card (cells inside
            # this count are always shown, even at 0 V; cells beyond
            # are treated as unused slots and hidden).
            PeripheralChannel("cell_count",   "Cell count",     "in", "analog"),
        ] + [
            # JBD reports up to 16 series cells in register 0x04. For
            # smaller packs (4S, 8S, …) the unused slots stay at 0;
            # the BMS card hides any channel reading 0 V so they
            # don't clutter the display.
            PeripheralChannel(f"cell_{i:02d}", f"Cell {i}", "in", "analog")
            for i in range(1, 17)
        ],
        params=[
            PeripheralTypeParam(
                "transport", "Transport", "string", "uart",
                choices=[
                    {"value": "uart", "label": "UART (serial)"},
                    {"value": "ble",  "label": "Bluetooth (BLE)"},
                ],
                help="UART is more robust; BLE lets one Pi host multiple "
                     "BMSes without wiring."),
            PeripheralTypeParam(
                "mac", "BLE MAC address", "string", "",
                visible_when={"transport": "ble"},
                help="The BMS's Bluetooth MAC. Use the Scan button to "
                     "discover nearby JBD devices."),
            PeripheralTypeParam(
                "poll_interval_ms", "Poll interval (ms)", "int", 1000,
                min=100, max=10000),
        ],
    ),
    # On-host audio file playback. The catalog entry is platform-agnostic;
    # `backend` picks the concrete implementation the node-side driver
    # uses. Today only "pi_alsa" (python-vlc against the host's ALSA stack)
    # ships, and the raspberrypi board YAML auto-attaches one instance per Pi.
    # A future RP2040 + UART MP3-trigger module would register as
    # backend="<module_id>" with pin_kind/pins covering the UART pair —
    # the catalog stays unchanged.
    #
    # Control surface:
    #   - `play` / `stop` / `pause` (rising-edge triggers): wire a routing
    #     source or controller binding into these to drive transport.
    #   - `seek` (seconds): scrub the playhead to a timecode while playing.
    #   - `volume` (0..1): playback gain.
    #   - `is_playing` + `position` are read-only telemetry channels.
    #
    # The filename to play is selected via the out-of-band peripheral-
    # command path (action="peripheral_command", command="play_file",
    # args={"filename": "..."}), not a channel — channels carry numeric
    # values only, and we don't want to limit the operator to a pre-
    # enumerated file list.
    "audio_player": PeripheralType(
        id="audio_player", label="Audio Player",
        description=(
            "On-host audio file playback. The Raspberry Pi backend uses "
            "python-vlc against the host's ALSA stack (HDMI, USB DAC, or "
            "I²S HAT — pick the ALSA device). Route signals into "
            "play/stop/pause/seek/volume, or send a play_file peripheral "
            "command to pick an arbitrary file from the library folder."
        ),
        pin_kind="builtin",
        channels=[
            PeripheralChannel("play",       "Play (trigger)",  "out", "digital_out"),
            PeripheralChannel("stop",       "Stop (trigger)",  "out", "digital_out"),
            PeripheralChannel("pause",      "Pause (trigger)", "out", "digital_out"),
            PeripheralChannel("seek",       "Seek (s)",        "out", "analog"),
            PeripheralChannel("volume",     "Volume (0–1)",    "out", "analog"),
            PeripheralChannel("is_playing", "Playing",         "in",  "digital_in"),
            PeripheralChannel("position",   "Position (s)",    "in",  "analog"),
        ],
        params=[
            PeripheralTypeParam(
                "library_path", "Library path", "string",
                "/var/lib/saint-os/audio",
                help=(
                    "Folder on this node holding the audio files. The "
                    "driver does not pre-enumerate or restrict file types — "
                    "send play_file with whatever filename you placed there."
                ),
            ),
            PeripheralTypeParam(
                "backend", "Backend", "string", "pi_alsa",
                choices=[
                    {"value": "pi_alsa", "label": "Raspberry Pi (python-vlc / ALSA)"},
                ],
                help=(
                    "Audio backend implementation. Today only the Pi ALSA "
                    "backend ships; future MP3-trigger modules will appear "
                    "here without a catalog change."
                ),
            ),
            PeripheralTypeParam(
                "alsa_device", "ALSA device", "string", "default",
                help=(
                    "ALSA PCM device name. Use 'default' for the system "
                    "default sink, or run `aplay -l` on the host to find "
                    "specific cards (e.g. 'hw:0,0', 'plughw:Headphones')."
                ),
            ),
            PeripheralTypeParam(
                "initial_volume", "Initial volume (0–1)", "float", 0.8,
                min=0.0, max=1.0,
            ),
        ],
    ),
    # Audio output mixer — controls the host sound card's *output*
    # (master volume + L/R balance + mute), unlike audio_player which
    # sets one media player's application volume. The Raspberry Pi
    # backend drives an ALSA mixer control via pyalsaaudio. Pinless,
    # one instance per node; auto-seeded as a built-in on Pi hosts.
    "audio_mixer": PeripheralType(
        id="audio_mixer", label="Audio Output (Volume / Mix)",
        description=(
            "System audio-output control for the host sound card. Route "
            "signals into volume (master 0–1), balance (−1 full-left … "
            "+1 full-right), or mute; the left/right/muted read-back "
            "channels report the mixer's live state. The Raspberry Pi "
            "backend drives an ALSA mixer control (Master/PCM/Digital/…) "
            "via pyalsaaudio and affects everything the card emits — the "
            "audio player, system sounds, all of it."
        ),
        pin_kind="builtin",
        channels=[
            PeripheralChannel("volume",  "Volume (0–1)",      "out", "analog"),
            PeripheralChannel("balance", "Balance (−1…+1)",   "out", "analog"),
            PeripheralChannel("mute",    "Mute (trigger)",    "out", "digital_out"),
            PeripheralChannel("left",    "Left level (0–1)",  "in",  "analog"),
            PeripheralChannel("right",   "Right level (0–1)", "in",  "analog"),
            PeripheralChannel("muted",   "Muted",             "in",  "digital_in"),
        ],
        params=[
            PeripheralTypeParam(
                "backend", "Backend", "string", "alsa",
                choices=[
                    {"value": "alsa", "label": "Raspberry Pi (pyalsaaudio / ALSA)"},
                ],
                help=(
                    "Mixer backend implementation. Today only the Pi ALSA "
                    "backend ships; future PulseAudio/PipeWire backends "
                    "will appear here without a catalog change."
                ),
            ),
            PeripheralTypeParam(
                "card", "ALSA card index", "int", 0, min=0,
                help=(
                    "ALSA card index the mixer control lives on. Run "
                    "`aplay -l` on the host to list cards; 0 is the "
                    "default output on most Pis."
                ),
            ),
            PeripheralTypeParam(
                "mixer_control", "Mixer control", "string", "Master",
                help=(
                    "Name of the ALSA mixer element to drive. Run "
                    "`amixer -c <card> scontrols` to list them; common "
                    "choices are Master, PCM, Digital, Headphone, Speaker."
                ),
            ),
            PeripheralTypeParam(
                "default_volume", "Default volume (0–1)", "float", 0.8,
                min=0.0, max=1.0,
                help="Master level applied on boot / config apply.",
            ),
            PeripheralTypeParam(
                "default_balance", "Default balance (−1…+1)", "float", 0.0,
                min=-1.0, max=1.0,
                help="L/R balance applied on boot. 0 = centered.",
            ),
            PeripheralTypeParam(
                "default_mute", "Start muted", "bool", False,
                help="When true, the output starts muted on boot.",
            ),
        ],
    ),
    # Console display — configures a Pi node as an HDMI kiosk pointing
    # at a Console view URL on this server. The peripheral has no I/O
    # channels: applying it makes the Pi's saint_node write a Chromium
    # launch script + autostart entry, then (if running an X/Wayland
    # session) signal a relaunch. The kiosk URL embeds the server's
    # kiosk token so the browser auto-authenticates against any
    # password gate — see WebSocketConfig.kiosk_token.
    "console_display": PeripheralType(
        id="console_display", label="Console Display",
        description=(
            "Configures this Pi node as an HDMI kiosk that shows one of "
            "the SAINT.OS Console views (red LED dot-matrix UI). On "
            "apply, the driver writes a Chromium launch script and an "
            "autostart desktop entry; on the next login the Pi opens "
            "the configured view fullscreen with passwordless access. "
            "Useful for purpose-built console boxes attached to a robot."
        ),
        pin_kind="builtin",
        channels=[],
        params=[
            PeripheralTypeParam(
                "view", "Console view", "string", "batteries",
                choices=[
                    {"value": "batteries", "label": "Batteries overview (all packs)"},
                    {"value": "battery",   "label": "Single battery (requires target IDs)"},
                ],
                help="Which Console view to display. 'Batteries overview' "
                     "auto-discovers every adopted BMS — no targeting "
                     "needed. 'Single battery' zooms into one pack and "
                     "requires the target node + peripheral IDs below.",
            ),
            PeripheralTypeParam(
                "target_node_id", "Target node ID", "string", "",
                help="(Only used when view = Single battery.) ID of the "
                     "node whose peripheral feeds this view, e.g. the "
                     "controller node that owns the BMS. Leave blank "
                     "for the overview view."),
            PeripheralTypeParam(
                "target_peripheral_id", "Target peripheral ID", "string", "",
                help="(Only used when view = Single battery.) Peripheral "
                     "ID on the target node, e.g. the BMS instance "
                     "label. Leave blank for the overview view."),
            PeripheralTypeParam(
                "server_url", "Server URL", "string", "",
                help="Base URL of the SAINT.OS server the kiosk browser "
                     "loads. Leave blank to auto-detect the server's own "
                     "address (the server fills it in on config push). "
                     "Set only to override, e.g. http://opensaint.local."),
            PeripheralTypeParam(
                "rotation", "Display rotation", "int", 0,
                choices=[0, 90, 180, 270],
                help="HDMI display rotation. Applied via xrandr / "
                     "wlr-randr at kiosk launch."),
            PeripheralTypeParam(
                "autostart", "Autostart on login", "bool", True,
                help="When true, the driver writes ~/.config/autostart/"
                     "saint-console-kiosk.desktop so Chromium relaunches "
                     "on every desktop-session login. Disable for manual "
                     "control."),
        ],
    ),
    "system_monitor": PeripheralType(
        id="system_monitor", label="System Monitor",
        description="Host controller telemetry: CPU/memory usage, temperature, throttle status, uptime, WiFi link health.",
        pin_kind="builtin",
        channels=[
            PeripheralChannel("cpu_usage", "CPU usage (%)",    "in", "analog"),
            PeripheralChannel("cpu_temp",  "CPU temp (°C)",    "in", "analog"),
            PeripheralChannel("mem_usage", "Memory usage (%)", "in", "analog"),
            PeripheralChannel("throttle",  "Throttled",        "in", "digital_in"),
            PeripheralChannel("uptime",    "Uptime (s)",       "in", "analog"),
            # WiFi link health — populated from `iw dev` + /proc/net/wireless
            # when the host has a wireless interface. See wifi_stats.py
            # for the data sources and what each number means.
            PeripheralChannel("wifi_signal",    "WiFi signal (dBm)",     "in", "analog"),
            PeripheralChannel("wifi_retry_pct", "WiFi tx retry (%)",     "in", "analog"),
            PeripheralChannel("wifi_noise",     "WiFi noise floor (dBm)", "in", "analog"),
            PeripheralChannel("wifi_bitrate",   "WiFi tx bitrate (Mbps)", "in", "analog"),
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
    log_enabled: bool = False

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
            log_enabled=bool(d.get("log_enabled", False)),
        )

    def channels(self, catalog: Dict[str, PeripheralType]) -> List[PeripheralChannel]:
        t = catalog.get(self.type)
        return list(t.channels) if t else []

    def claimed_gpios(self, catalog: Dict[str, PeripheralType]) -> Dict[str, int]:
        """All GPIO numbers this peripheral claims, by source label.

        Combines `pins` (UART tx/rx, single-gpio peripherals, etc.) with
        any `params` whose schema declares type == "gpio" — those are
        configurable pin assignments stored alongside the peripheral's
        other parameters (e.g. RoboClaw's `estop_pin`). A param value of
        0 is the "no pin assigned" sentinel and is excluded.

        Both `detect_pin_conflicts` (here) and the dashboard's pin-
        availability widget need to see GPIO-typed params or they'd let
        the operator double-claim the same pin (e.g. RoboClaw's E-stop +
        a Button on the same GPIO) without warning.
        """
        out: Dict[str, int] = {}
        for k, v in self.pins.items():
            if isinstance(v, int) and v > 0:
                out[k] = v
        t = catalog.get(self.type)
        if t is None:
            return out
        for p in t.params:
            if p.type != "gpio":
                continue
            v = self.params.get(p.id)
            if isinstance(v, int) and v > 0:
                out[p.id] = v
        return out


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
# System-wide routing: per-node sheets + dashboard widgets
# ─────────────────────────────────────────────────────────────────────────────
#
# A routing sheet is owned by a controller node and contains everything
# needed to compute that node's peripheral inputs from external sources.
# Each sheet holds:
#
#   - InputNodes      ROS topic sources (topic + scalar field)
#   - OperatorNodes   server-evaluated transforms (Max/Min/Clamp/Lerp/…)
#   - Wires           edges from (input|operator) output → (operator pin |
#                                                            peripheral channel |
#                                                            widget input)
#
# Sheets are hard-scoped to one controller node: if the same topic feeds
# two controllers, it appears as an InputNode on each sheet. The special
# pseudo-sheet "dashboard" hosts the widget sinks. Endpoint kinds are:
#
#   kind == "input":      parts = [input_id]            (ROS-topic input; pin always "out")
#   kind == "ws_input":   parts = [ws_input_id]         (controller-driven input; pin always "out")
#   kind == "operator":   parts = [op_id, pin]          (pin = "out" or input id)
#   kind == "output":     parts = [output_id]           (ROS-topic output; valid as sink and tap source)
#   kind == "peripheral": parts = [node_id, peripheral_id, channel_id]
#   kind == "widget":     parts = [widget_id, input_id]
#   kind == "signal":     parts = [signal_name]         (cross-sheet named float; pin "in" sink, "out" source)
#
# Signals are first-class named floats that live above the sheet
# layer — a SignalNode on Sheet A wired to its `in` pin writes a
# value into the global signal table; a SignalNode on Sheet B with
# the same name reads that value out of `out`. The evaluator's
# RoutingEvaluator owns the table (Dict[str, float]); see
# _evaluate_sheet's pass-0 and _resolve_source.


@dataclass
class RouteEndpoint:
    """One end of a wire."""
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
class InputNode:
    """A source node on a sheet — currently either a ROS topic or a
    URDF joint driven by the animation player.

    Discriminated by `kind`:
      - ``"topic"`` (default): ``topic`` is the canonical endpoint path
        (e.g. ``"/joy"``, ``"/cmd_vel"``), ``field`` is a flattened scalar
        path into the message (e.g. ``"axes[0]"``, ``"linear.x"``). The
        evaluator subscribes once per ``(topic, field)`` pair across all
        sheets that reference it.
      - ``"urdf_joint"``: ``joint`` names a URDF joint. Whenever an
        animation plays and one of its value tracks matches this joint
        name, the sampled value flows through this input as the joint's
        live setpoint.
    """
    id: str
    topic: str
    field: str
    label: str = ""
    position: Tuple[int, int] = (0, 0)
    kind: str = "topic"
    joint: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "topic": self.topic,
            "field": self.field,
            "label": self.label,
            "position": list(self.position),
            "kind": self.kind,
            "joint": self.joint,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "InputNode":
        pos = d.get("position") or [0, 0]
        return cls(
            id=d["id"],
            topic=d.get("topic", ""),
            field=d.get("field", "value"),
            label=d.get("label", ""),
            position=(int(pos[0]) if len(pos) > 0 else 0,
                      int(pos[1]) if len(pos) > 1 else 0),
            kind=d.get("kind", "topic"),
            joint=d.get("joint", ""),
        )


@dataclass
class WebSocketInputNode:
    """A controller-driven input on a sheet (no ROS topic involved).

    Distinct from InputNode (which subscribes to a real ROS topic). A
    controller binding addresses this node by (sheet_id, input_id) over
    the websocket and the value flows straight into the routing
    evaluator's source cache. Used for joystick/gamepad axes whose only
    purpose is to drive operators and peripheral channels — no need to
    round-trip through ROS or pretend they're state topics.

    The ``kind`` field distinguishes two flavors of WS input:

      * ``"command"`` — a target a controller binding writes TO. Default
        for operator-added inputs; what the binding picker should
        offer. These are the actual "drive this motor" slots.
      * ``"state"`` — an echo node, produced by the migration of
        state-only ROS endpoints (``endpoints.yaml`` entries with
        ``state_type`` and no ``command_type``). These exist so wires
        downstream can still tap the value, but binding a CONTROLLER
        to one doesn't make sense — controllers don't produce state,
        sensors do. The binding picker filters these out.

    Default ``"command"`` so anything added in code paths that don't
    set it explicitly behaves as before (operator-added inputs).
    """
    id: str
    label: str = ""
    position: Tuple[int, int] = (0, 0)
    kind: str = "command"

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "label": self.label,
            "position": list(self.position),
            "kind": self.kind,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "WebSocketInputNode":
        pos = d.get("position") or [0, 0]
        return cls(
            id=d["id"],
            label=d.get("label", ""),
            position=(int(pos[0]) if len(pos) > 0 else 0,
                      int(pos[1]) if len(pos) > 1 else 0),
            # Default "command" preserves the meaning of existing
            # persisted sheets that pre-date the kind field.
            kind=d.get("kind", "command"),
        )


@dataclass
class OutputNode:
    """A ROS topic sink that also taps the value as a wire source.

    A wire feeding this node's `value` sink gets published onto
    `topic.field` via the bridge's set_topic_channel buffer. The same
    value is exposed as a wire *source* (kind="output", parts=[id]) so
    the sheet can fan the same value onward to a peripheral — letting a
    single sheet describe controller-input → math → ROS-publish →
    peripheral end to end.
    """
    id: str
    topic: str
    field: str
    label: str = ""
    position: Tuple[int, int] = (0, 0)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "topic": self.topic,
            "field": self.field,
            "label": self.label,
            "position": list(self.position),
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "OutputNode":
        pos = d.get("position") or [0, 0]
        return cls(
            id=d["id"],
            topic=d["topic"],
            field=d.get("field", "value"),
            label=d.get("label", ""),
            position=(int(pos[0]) if len(pos) > 0 else 0,
                      int(pos[1]) if len(pos) > 1 else 0),
        )


@dataclass
class OperatorNode:
    """A server-evaluated transform on a sheet.

    `op` selects an entry from OPERATOR_CATALOG. `params` are static
    config (e.g. the curve exponent). `defaults` are per-input fallback
    constants used when no wire is connected to that input pin — this is
    how the operator lets the operator type in literal min/max values
    directly on the node card.
    """
    id: str
    op: str
    label: str = ""
    params: Dict[str, Any] = field(default_factory=dict)
    defaults: Dict[str, float] = field(default_factory=dict)
    position: Tuple[int, int] = (0, 0)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "op": self.op,
            "label": self.label,
            "params": dict(self.params),
            "defaults": {k: float(v) for k, v in self.defaults.items()},
            "position": list(self.position),
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "OperatorNode":
        pos = d.get("position") or [0, 0]
        return cls(
            id=d["id"],
            op=d["op"],
            label=d.get("label", ""),
            params=dict(d.get("params", {})),
            defaults={k: float(v) for k, v in (d.get("defaults") or {}).items()},
            position=(int(pos[0]) if len(pos) > 0 else 0,
                      int(pos[1]) if len(pos) > 1 else 0),
        )


@dataclass
class SignalNode:
    """A named global float that crosses sheet boundaries.

    The same `name` resolves to the same value table entry regardless
    of which sheet the SignalNode lives on, so the routing graph can
    span sheets — wire a boolean computed on sheet A into a signal
    named "shoulder_locked", then on sheet B wire that signal into an
    IF gate's `cond` to inhibit a downstream servo command.

    Per-sheet placement is purely UI bookkeeping. Two sheets each
    holding a SignalNode named "foo" reference the same global value;
    if both have wires writing to it the evaluator's sheet-iteration
    order determines last-writer-wins for the tick.
    """
    id: str
    name: str
    label: str = ""
    position: Tuple[int, int] = (0, 0)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "label": self.label,
            "position": list(self.position),
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "SignalNode":
        pos = d.get("position") or [0, 0]
        return cls(
            id=d["id"],
            name=str(d.get("name") or "").strip(),
            label=d.get("label", ""),
            position=(int(pos[0]) if len(pos) > 0 else 0,
                      int(pos[1]) if len(pos) > 1 else 0),
        )


@dataclass
class Wire:
    """One edge on a sheet — a value flowing from source to sink."""
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
    def from_dict(cls, d: Dict[str, Any]) -> "Wire":
        return cls(
            id=d["id"],
            source=RouteEndpoint.from_dict(d["source"]),
            sink=RouteEndpoint.from_dict(d["sink"]),
        )


# Sentinel used by the legacy widgets-only sheet; kept so old yaml files
# loaded by from_dict can be migrated (widgets moved onto a real sheet),
# but no longer exposed in the routing UI.
DASHBOARD_SHEET_ID = "_dashboard"


@dataclass
class NodeSheet:
    """All routing state owned by one controller node.

    A sheet contains the ROS topic Inputs, Operator transforms, Output
    topic publishers, Widget sinks, and the Wires connecting them. Each
    sheet is hard-scoped to one controller node — the peripherals
    visible on the canvas are exactly that node's peripherals, and
    widgets here are surfaced on that controller's dashboard view.
    """
    node_id: str
    inputs: List[InputNode] = field(default_factory=list)
    ws_inputs: List[WebSocketInputNode] = field(default_factory=list)
    outputs: List["OutputNode"] = field(default_factory=list)
    operators: List[OperatorNode] = field(default_factory=list)
    widgets: List["WidgetInstance"] = field(default_factory=list)
    signals: List["SignalNode"] = field(default_factory=list)
    wires: List[Wire] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "node_id": self.node_id,
            "inputs": [n.to_dict() for n in self.inputs],
            "ws_inputs": [n.to_dict() for n in self.ws_inputs],
            "outputs": [n.to_dict() for n in self.outputs],
            "operators": [n.to_dict() for n in self.operators],
            "widgets": [w.to_dict() for w in self.widgets],
            "signals": [s.to_dict() for s in self.signals],
            "wires": [w.to_dict() for w in self.wires],
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "NodeSheet":
        # Forward reference: WidgetInstance is declared below.
        # Legacy "animations" list (AnimationInputNode entries) — silently
        # dropped on load; animations now drive routing through URDF-joint
        # InputNodes, so any persisted animation nodes are obsolete.
        return cls(
            node_id=d["node_id"],
            inputs=[InputNode.from_dict(x) for x in d.get("inputs", [])],
            ws_inputs=[WebSocketInputNode.from_dict(x) for x in d.get("ws_inputs", [])],
            outputs=[OutputNode.from_dict(x) for x in d.get("outputs", [])],
            operators=[OperatorNode.from_dict(x) for x in d.get("operators", [])],
            widgets=[WidgetInstance.from_dict(x) for x in d.get("widgets", [])],
            signals=[SignalNode.from_dict(x) for x in d.get("signals", [])],
            wires=[Wire.from_dict(x) for x in d.get("wires", [])],
        )

    def _next_id(self, prefix: str, existing_ids) -> str:
        n = 1
        while f"{prefix}{n}" in existing_ids:
            n += 1
        return f"{prefix}{n}"

    def add_input(self, topic: str, field: str, label: str = "",
                  position: Tuple[int, int] = (0, 0),
                  kind: str = "topic", joint: str = "") -> InputNode:
        existing = {n.id for n in self.inputs}
        if kind == "urdf_joint":
            default_label = joint or "joint"
        else:
            default_label = f"{topic}{('.' + field) if field else ''}"
        node = InputNode(
            id=self._next_id("in", existing),
            topic=topic, field=field,
            label=label or default_label,
            position=position,
            kind=kind,
            joint=joint,
        )
        self.inputs.append(node)
        return node

    def add_ws_input(self, label: str = "",
                     position: Tuple[int, int] = (0, 0)) -> WebSocketInputNode:
        existing = {n.id for n in self.ws_inputs}
        node = WebSocketInputNode(
            id=self._next_id("wsin", existing),
            label=label,
            position=position,
        )
        self.ws_inputs.append(node)
        return node

    def add_output(self, topic: str, field: str, label: str = "",
                   position: Tuple[int, int] = (0, 0)) -> OutputNode:
        existing = {n.id for n in self.outputs}
        node = OutputNode(
            id=self._next_id("out", existing),
            topic=topic, field=field,
            label=label or f"{topic}{('.' + field) if field else ''}",
            position=position,
        )
        self.outputs.append(node)
        return node

    def add_operator(self, op: str, label: str = "",
                     params: Optional[Dict[str, Any]] = None,
                     defaults: Optional[Dict[str, float]] = None,
                     position: Tuple[int, int] = (0, 0)) -> OperatorNode:
        existing = {n.id for n in self.operators}
        node = OperatorNode(
            id=self._next_id("op", existing),
            op=op, label=label,
            params=dict(params or {}),
            defaults=dict(defaults or {}),
            position=position,
        )
        self.operators.append(node)
        return node

    def add_signal(self, name: str, label: str = "",
                   position: Tuple[int, int] = (0, 0)) -> "SignalNode":
        existing = {n.id for n in self.signals}
        node = SignalNode(
            id=self._next_id("sig", existing),
            name=name.strip(),
            label=label or name.strip(),
            position=position,
        )
        self.signals.append(node)
        return node

    def add_widget(self, type_id: str, label: str,
                   position: Tuple[int, int] = (0, 0),
                   params: Optional[Dict[str, Any]] = None,
                   extra_existing_ids: Optional[Set[str]] = None) -> "WidgetInstance":
        # Widget IDs must be unique GLOBALLY across all sheets, not just
        # within this sheet — the dashboard's WidgetsDashboard keys its
        # DOM elements, _lastValues map, and history map by bare widget
        # ID, and its wire-lookup helper (`_findWidgetSheet`) treats the
        # first sheet containing a given widget ID as authoritative.
        # Two sheets each adding their first roboclaw_monitor would
        # otherwise both pick "roboclaw_monitor-1" and render onto the
        # same DOM nodes. State manager passes the set of widget IDs
        # already in use elsewhere via extra_existing_ids.
        existing = {w.id for w in self.widgets}
        if extra_existing_ids:
            existing = existing | extra_existing_ids
        n = sum(1 for w in self.widgets if w.type == type_id) + 1
        while f"{type_id}-{n}" in existing:
            n += 1
        widget = WidgetInstance(id=f"{type_id}-{n}", type=type_id,
                                label=label or type_id,
                                position=position, params=dict(params or {}))
        self.widgets.append(widget)
        return widget

    def add_wire(self, source: RouteEndpoint, sink: RouteEndpoint) -> Wire:
        existing = {w.id for w in self.wires}
        # Replace any existing wire feeding the same sink — sinks accept
        # one value, last writer wins.
        self.wires = [w for w in self.wires
                      if not (w.sink.kind == sink.kind and w.sink.parts == sink.parts)]
        wire = Wire(id=self._next_id("w", existing), source=source, sink=sink)
        self.wires.append(wire)
        return wire

    def remove_input(self, input_id: str) -> bool:
        before = len(self.inputs)
        self.inputs = [n for n in self.inputs if n.id != input_id]
        if len(self.inputs) == before:
            return False
        self.wires = [w for w in self.wires
                      if not (w.source.kind == "input" and w.source.parts and w.source.parts[0] == input_id)]
        return True

    def remove_ws_input(self, input_id: str) -> bool:
        before = len(self.ws_inputs)
        self.ws_inputs = [n for n in self.ws_inputs if n.id != input_id]
        if len(self.ws_inputs) == before:
            return False
        self.wires = [w for w in self.wires
                      if not (w.source.kind == "ws_input"
                              and w.source.parts and w.source.parts[0] == input_id)]
        return True

    def remove_output(self, output_id: str) -> bool:
        before = len(self.outputs)
        self.outputs = [n for n in self.outputs if n.id != output_id]
        if len(self.outputs) == before:
            return False
        # Output participates as both sink (publish) and source (tap);
        # drop wires that reference it on either end.
        self.wires = [
            w for w in self.wires
            if not ((w.sink.kind == "output" and w.sink.parts and w.sink.parts[0] == output_id)
                    or (w.source.kind == "output" and w.source.parts and w.source.parts[0] == output_id))
        ]
        return True

    def remove_operator(self, op_id: str) -> bool:
        before = len(self.operators)
        self.operators = [n for n in self.operators if n.id != op_id]
        if len(self.operators) == before:
            return False
        self.wires = [
            w for w in self.wires
            if not ((w.source.kind == "operator" and w.source.parts and w.source.parts[0] == op_id)
                    or (w.sink.kind == "operator" and w.sink.parts and w.sink.parts[0] == op_id))
        ]
        return True

    def remove_signal(self, signal_id: str) -> bool:
        before = len(self.signals)
        self.signals = [n for n in self.signals if n.id != signal_id]
        if len(self.signals) == before:
            return False
        # Wires reference signals by name (not signal_id), but a delete
        # of the local SignalNode only severs THIS sheet's view of the
        # global signal — other sheets' SignalNodes (and their wires)
        # keep working against the same name. So we only drop wires
        # whose endpoint name matches this node's name AND whose
        # endpoint actually lives on this sheet's wires list.
        gone = next((n for n in self.signals if n.id == signal_id), None)
        # No-op: the removed signal is no longer in self.signals so we
        # can't look it up. Instead just leave wires alone; an orphan
        # wire targeting kind="signal" with a name nothing on this
        # sheet writes/reads is harmless (resolves to None).
        return True

    def remove_widget(self, widget_id: str) -> bool:
        before = len(self.widgets)
        self.widgets = [w for w in self.widgets if w.id != widget_id]
        if len(self.widgets) == before:
            return False
        self.wires = [
            w for w in self.wires
            if not (w.sink.kind == "widget" and w.sink.parts and w.sink.parts[0] == widget_id)
        ]
        return True

    def remove_wire(self, wire_id: str) -> bool:
        before = len(self.wires)
        self.wires = [w for w in self.wires if w.id != wire_id]
        return len(self.wires) < before

    def find_input(self, input_id: str) -> Optional[InputNode]:
        return next((n for n in self.inputs if n.id == input_id), None)

    def find_ws_input(self, input_id: str) -> Optional[WebSocketInputNode]:
        return next((n for n in self.ws_inputs if n.id == input_id), None)

    def find_output(self, output_id: str) -> Optional[OutputNode]:
        return next((n for n in self.outputs if n.id == output_id), None)

    def find_operator(self, op_id: str) -> Optional[OperatorNode]:
        return next((n for n in self.operators if n.id == op_id), None)

    def find_widget(self, widget_id: str) -> Optional["WidgetInstance"]:
        return next((w for w in self.widgets if w.id == widget_id), None)

    def find_signal(self, signal_id: str) -> Optional["SignalNode"]:
        return next((n for n in self.signals if n.id == signal_id), None)


# ── Operator catalog ──────────────────────────────────────────────────


@dataclass
class OperatorInputSpec:
    id: str
    label: str
    default: float = 0.0


@dataclass
class OperatorParamSpec:
    id: str
    label: str
    type: str          # "float" | "int" | "bool"
    default: Any


@dataclass
class OperatorType:
    id: str
    label: str
    description: str
    inputs: List[OperatorInputSpec]
    params: List[OperatorParamSpec] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "label": self.label,
            "description": self.description,
            "inputs": [asdict(i) for i in self.inputs],
            "params": [asdict(p) for p in self.params],
        }


# Catalog of pure float→float operators. The evaluator computes each
# operator's single output value from its input pins; inputs without a
# wired source fall back to the OperatorNode.defaults map, or to the
# spec default if absent.
OPERATOR_CATALOG: Dict[str, OperatorType] = {
    "add": OperatorType(
        "add", "Add", "out = a + b",
        inputs=[OperatorInputSpec("a", "A"), OperatorInputSpec("b", "B")],
    ),
    "subtract": OperatorType(
        "subtract", "Subtract", "out = a − b",
        inputs=[OperatorInputSpec("a", "A"), OperatorInputSpec("b", "B")],
    ),
    "multiply": OperatorType(
        "multiply", "Multiply", "out = a × b",
        inputs=[OperatorInputSpec("a", "A", 1.0), OperatorInputSpec("b", "B", 1.0)],
    ),
    "scale": OperatorType(
        "scale", "Scale", "out = value × factor",
        inputs=[OperatorInputSpec("value", "Value"), OperatorInputSpec("factor", "Factor", 1.0)],
    ),
    "max": OperatorType(
        "max", "Max", "out = max(a, b)",
        inputs=[OperatorInputSpec("a", "A"), OperatorInputSpec("b", "B")],
    ),
    "min": OperatorType(
        "min", "Min", "out = min(a, b)",
        inputs=[OperatorInputSpec("a", "A"), OperatorInputSpec("b", "B")],
    ),
    "clamp": OperatorType(
        "clamp", "Clamp", "Clamp value into [min, max].",
        inputs=[
            OperatorInputSpec("value", "Value"),
            OperatorInputSpec("min",   "Min", -1.0),
            OperatorInputSpec("max",   "Max",  1.0),
        ],
    ),
    "map_range": OperatorType(
        "map_range", "Map Range",
        "Linearly remap value from [in_min, in_max] to [out_min, out_max]. "
        "When 'Clamp input' is checked, values outside the input range are "
        "first pinned to the nearest endpoint, so the output is guaranteed "
        "to stay inside [out_min, out_max]. With it off the mapping extends "
        "linearly past either end (useful when you want to amplify beyond "
        "the nominal range).",
        inputs=[
            OperatorInputSpec("value",   "Value"),
            OperatorInputSpec("in_min",  "In Min", -1.0),
            OperatorInputSpec("in_max",  "In Max",  1.0),
            OperatorInputSpec("out_min", "Out Min", 0.0),
            OperatorInputSpec("out_max", "Out Max", 1.0),
        ],
        params=[OperatorParamSpec("clamp", "Clamp input to range", "bool", True)],
    ),
    "lerp": OperatorType(
        "lerp", "Lerp", "out = a + (b − a) × t",
        inputs=[
            OperatorInputSpec("a", "A"),
            OperatorInputSpec("b", "B"),
            OperatorInputSpec("t", "T", 0.5),
        ],
    ),
    "invert": OperatorType(
        "invert", "Invert", "out = −value",
        inputs=[OperatorInputSpec("value", "Value")],
    ),
    "abs": OperatorType(
        "abs", "Absolute", "out = |value|",
        inputs=[OperatorInputSpec("value", "Value")],
    ),
    "deadband": OperatorType(
        "deadband", "Deadband",
        "Pass through value if |value| > threshold; else 0.",
        inputs=[
            OperatorInputSpec("value",     "Value"),
            OperatorInputSpec("threshold", "Threshold", 0.05),
        ],
    ),
    "curve": OperatorType(
        "curve", "Curve (Expo)",
        "out = sign(value) × |value|^exponent — common joystick expo curve.",
        inputs=[
            OperatorInputSpec("value",    "Value"),
            OperatorInputSpec("exponent", "Exponent", 2.0),
        ],
    ),
    "select": OperatorType(
        "select", "Select Input",
        "Pick one of two sources. The preferred input wins whenever it has "
        "a non-zero value; otherwise the other input passes through. Common "
        "use: wire a joystick to the preferred pin and an animation to the "
        "other so manual control overrides playback only while the operator "
        "is actively pushing the stick.",
        inputs=[
            OperatorInputSpec("a", "A"),
            OperatorInputSpec("b", "B"),
        ],
        params=[OperatorParamSpec("prefer_a", "Prefer A", "bool", False)],
    ),

    # ── Logic operators ─────────────────────────────────────────────
    # All values on the routing graph are floats. The logic ops below
    # treat any input ≥ 0.5 as true, otherwise false; their outputs are
    # always 1.0 or 0.0. That keeps boolean values composable with the
    # numeric operators above (eg. multiply by an AND gate to mask a
    # signal off, route through a Map Range to scale a 0/1 enable into
    # whatever the consumer expects).
    "and": OperatorType(
        "and", "AND",
        "out = 1 if both a and b are true (≥ 0.5), else 0.",
        inputs=[OperatorInputSpec("a", "A"), OperatorInputSpec("b", "B")],
    ),
    "or": OperatorType(
        "or", "OR",
        "out = 1 if either a or b is true (≥ 0.5), else 0.",
        inputs=[OperatorInputSpec("a", "A"), OperatorInputSpec("b", "B")],
    ),
    "not": OperatorType(
        "not", "NOT",
        "out = 1 if value is false (< 0.5), else 0. Flips a boolean.",
        inputs=[OperatorInputSpec("value", "Value")],
    ),
    "in_range": OperatorType(
        "in_range", "In Range",
        "out = 1 when min ≤ value ≤ max, else 0. Useful as the safety "
        "condition feeding an IF gate — e.g. allow shoulder rotation "
        "only when elbow is within a safe window.",
        inputs=[
            OperatorInputSpec("value", "Value"),
            OperatorInputSpec("min",   "Min", -1.0),
            OperatorInputSpec("max",   "Max",  1.0),
        ],
    ),
    "if": OperatorType(
        "if", "IF",
        "out = then-value when cond is true (≥ 0.5); else else-value. "
        "Combined with AND / OR / NOT / In Range this is the gating "
        "primitive: route the desired servo command into `then`, a safe "
        "fallback (often 0 or the current position) into `else`, and a "
        "boolean expression into `cond`.",
        inputs=[
            OperatorInputSpec("cond", "Cond"),
            OperatorInputSpec("then", "Then"),
            OperatorInputSpec("else", "Else"),
        ],
    ),
}


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
        id="battery_monitor", label="FAS100 Power Monitor",
        description="FrSky FAS100 battery telemetry: current, voltage, and "
                    "two temperature sensors. Rendered as the dashboard's "
                    "amber-accented power card.",
        inputs=[
            WidgetInputSpec("current", "Current Draw",    "analog"),
            WidgetInputSpec("voltage", "Battery Voltage", "analog"),
            WidgetInputSpec("temp1",   "Temp 1",          "analog"),
            WidgetInputSpec("temp2",   "Temp 2",          "analog"),
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
    "roboclaw_monitor": WidgetType(
        id="roboclaw_monitor", label="RoboClaw Motor Monitor",
        description="RoboClaw motor controller telemetry: commanded duty, "
                    "encoder, bus voltage, motor current, controller "
                    "temperature, and fault status. Rendered as the "
                    "dashboard's violet-accented motor card with per-row "
                    "sparklines and a fault badge strip.",
        inputs=[
            WidgetInputSpec("motor",   "Motor Duty",     "analog"),
            WidgetInputSpec("encoder", "Encoder",        "analog"),
            WidgetInputSpec("voltage", "Bus Voltage",    "analog"),
            WidgetInputSpec("current", "Motor Current",  "analog"),
            WidgetInputSpec("temp",    "Temperature",    "analog"),
            # Canonical ROBOCLAW_FAULT_* bitmask — the widget decodes it
            # into a red fault-badge row (overcurrent, over-temp, E-Stop,
            # driver fault, ...). 0 = healthy.
            WidgetInputSpec("error_flags", "Fault Status", "analog"),
        ],
    ),
    "bms_monitor": WidgetType(
        id="bms_monitor", label="BMS Monitor",
        description="Pack-level battery summary: SOC bar, voltage / current / "
                    "temperature, CHG/DSG FET state, and a fault panel that "
                    "surfaces any asserted protection bits. Lighter than the "
                    "per-cell BMS card on the node Live tab.",
        inputs=[
            WidgetInputSpec("soc",        "State of Charge",  "analog"),
            WidgetInputSpec("voltage",    "Pack Voltage",     "analog"),
            WidgetInputSpec("current",    "Pack Current",     "analog"),
            WidgetInputSpec("temp",       "Temperature",      "analog"),
            WidgetInputSpec("protection", "Protection Bits",  "analog"),
            WidgetInputSpec("fet_status", "FET Status",       "analog"),
        ],
    ),
}


# ─────────────────────────────────────────────────────────────────────────────
# System routing aggregate (routes + widgets + version)
# ─────────────────────────────────────────────────────────────────────────────


@dataclass
class SystemRouting:
    version: int = 0
    # Sheets keyed by owning controller node_id. Each sheet owns its
    # own inputs / outputs / operators / widgets / wires.
    sheets: Dict[str, NodeSheet] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "version": self.version,
            "sheets":  {nid: s.to_dict() for nid, s in self.sheets.items()},
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "SystemRouting":
        sheets_in = d.get("sheets") or {}
        sheets: Dict[str, NodeSheet] = {}
        for nid, sd in sheets_in.items():
            # Stored sheet dicts may omit node_id; trust the key.
            sd = dict(sd)
            sd.setdefault("node_id", nid)
            sheets[nid] = NodeSheet.from_dict(sd)
        # Migration: legacy yamls held a top-level `widgets` list (the
        # old global dashboard). Stash any survivors on the legacy
        # DASHBOARD_SHEET_ID sheet so the operator can re-place them
        # rather than silently losing them.
        legacy_widgets = d.get("widgets") or []
        if legacy_widgets:
            legacy = sheets.setdefault(DASHBOARD_SHEET_ID, NodeSheet(node_id=DASHBOARD_SHEET_ID))
            for w in legacy_widgets:
                legacy.widgets.append(WidgetInstance.from_dict(w))

        # Migration: resolve widget-ID collisions across sheets. The
        # pre-fix NodeSheet.add_widget scoped its uniqueness check to
        # the current sheet only, so two sheets that each added their
        # first widget of a given type would collide on "<type>-1".
        # The dashboard's WidgetsDashboard uses widget IDs as DOM keys
        # and as keys into its live-value map; collisions cause one
        # widget's values to silently overwrite the other's. We rename
        # the *second* (and any further) collisions to a fresh
        # globally-unique ID and rewrite the wires in the affected
        # sheet so the binding is preserved.
        seen_widget_ids: Set[str] = set()
        for sheet_id, sheet in sheets.items():
            for widget in sheet.widgets:
                if widget.id not in seen_widget_ids:
                    seen_widget_ids.add(widget.id)
                    continue
                # Pick a fresh suffix that's globally unique. Start at
                # 2 so the first survivor keeps "<type>-1".
                base = widget.type
                n = 2
                while f"{base}-{n}" in seen_widget_ids:
                    n += 1
                new_id = f"{base}-{n}"
                old_id = widget.id
                widget.id = new_id
                # Update wires in this sheet that reference the old
                # widget. Widget endpoints only appear as wire sinks
                # (you can't wire FROM a widget), so we only check
                # `sink`.
                for wire in sheet.wires:
                    if (wire.sink.kind == "widget"
                            and wire.sink.parts
                            and wire.sink.parts[0] == old_id):
                        wire.sink.parts[0] = new_id
                seen_widget_ids.add(new_id)
        return cls(
            version=int(d.get("version", 0)),
            sheets=sheets,
        )

    # ── Sheet access ──────────────────────────────────────────────────

    def get_sheet(self, node_id: str) -> NodeSheet:
        """Return the sheet for `node_id`, creating an empty one if needed."""
        sheet = self.sheets.get(node_id)
        if sheet is None:
            sheet = NodeSheet(node_id=node_id)
            self.sheets[node_id] = sheet
        return sheet

    def drop_sheet(self, node_id: str) -> bool:
        if node_id in self.sheets:
            del self.sheets[node_id]
            self.version += 1
            return True
        return False

    def bump_version(self) -> None:
        self.version += 1

    # ── Cascade helpers ──────────────────────────────────────────────

    def wires_touching_peripheral(self, node_id: str, peripheral_id: str) -> List[Wire]:
        """All wires across all sheets that reference (node_id, peripheral_id)."""
        out: List[Wire] = []
        for sheet in self.sheets.values():
            for w in sheet.wires:
                for ep in (w.source, w.sink):
                    if ep.kind == "peripheral" and len(ep.parts) >= 2 \
                            and ep.parts[0] == node_id and ep.parts[1] == peripheral_id:
                        out.append(w)
                        break
        return out

    def wires_touching_widget(self, widget_id: str) -> List[Wire]:
        out: List[Wire] = []
        for sheet in self.sheets.values():
            for w in sheet.wires:
                for ep in (w.source, w.sink):
                    if ep.kind == "widget" and ep.parts and ep.parts[0] == widget_id:
                        out.append(w)
                        break
        return out

    def drop_wires_touching_peripheral(self, node_id: str, peripheral_id: str) -> int:
        """Remove every wire that mentions the given peripheral. Returns count dropped."""
        dropped = 0
        for sheet in self.sheets.values():
            before = len(sheet.wires)
            sheet.wires = [
                w for w in sheet.wires
                if not any(
                    ep.kind == "peripheral" and len(ep.parts) >= 2
                    and ep.parts[0] == node_id and ep.parts[1] == peripheral_id
                    for ep in (w.source, w.sink)
                )
            ]
            dropped += before - len(sheet.wires)
        if dropped:
            self.version += 1
        return dropped

    # ── Widget lookup across sheets (used by `wires_touching_widget`) ──

    def find_widget(self, widget_id: str) -> Optional[Tuple[NodeSheet, WidgetInstance]]:
        """Locate which sheet owns the widget, if any."""
        for sheet in self.sheets.values():
            w = sheet.find_widget(widget_id)
            if w is not None:
                return sheet, w
        return None


# ─────────────────────────────────────────────────────────────────────────────
# Validation
# ─────────────────────────────────────────────────────────────────────────────


def detect_pin_conflicts(config: NodePeripheralConfig,
                         uart_pairs: Optional[List[Dict[str, int]]] = None,
                         catalog: Optional[Dict[str, PeripheralType]] = None,
                         ) -> List[str]:
    """Return a list of human-readable conflict messages, or empty.

    Catalog is needed so the conflict check can also see GPIO-typed
    `params` (e.g. RoboClaw's `estop_pin`), not just `pins`. Without it,
    we'd let the operator double-claim the same GPIO between a
    RoboClaw's E-stop and another peripheral's primary pin. The
    parameter is Optional for backwards compatibility with callers
    that haven't been updated yet; in that case we fall back to the
    pins-only check and silently miss GPIO-typed params.
    """
    errors: List[str] = []
    by_pin: Dict[int, PeripheralInstance] = {}
    cat = catalog or {}
    for p in config.peripherals:
        claimed = p.claimed_gpios(cat) if cat else dict(p.pins)
        for pin in claimed.values():
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
