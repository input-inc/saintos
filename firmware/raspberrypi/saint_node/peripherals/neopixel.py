"""NeoPixel (WS2812) strip driver for the Raspberry Pi node.

Parity port of the Teensy (`neopixel_strip.cpp`, Adafruit bit-bang) and
RP2040 (`neopixel_strip.c`, PIO) external-strip drivers: operator-added
WS2812 strips on a data pin, catalog type "neopixel", channels
`color` (packed 0xRRGGBB int carried as a float) and `brightness`
(0.0-1.0 float). Strips start DARK on config apply — the operator's
routing turns them on. ESTOP drives every strip dark but keeps the
stored color so a fresh write after clear restores state.

Hardware backends (auto-selected, lazily imported so a missing system
lib disables just this driver — same convention as gpiod/vlc/alsa):

  * ``rpi_ws281x`` — PWM/DMA, arbitrary PWM-capable pin. Works on
    Pi 3/4; does NOT work on Pi 5 (RP1 has no equivalent DMA path).
  * ``spidev``     — WS2812-over-SPI (each WS2812 bit encoded as 3 SPI
    bits at 2.4 MHz). Works on every Pi INCLUDING Pi 5, but the data
    pin must be GPIO 10 (SPI0 MOSI).
  * logging stub   — neither lib present; config/control flow still
    works so the rest of the node keeps running.

Tests swap ``NeoPixelDriver._strip_cls`` (same pattern as RoboClaw's
``_transport_cls``) so no hardware lib is needed.
"""

from __future__ import annotations

from typing import Any, Dict, Optional

from .base import PeripheralDriver

# Virtual GPIO map (must not overlap other drivers — see
# peripheral_manager.register): maestro 200-223, syren 224-231,
# fas100 232-235, roboclaw 236-275, THIS 280-287, tic 300+.
NEOPIXEL_VIRTUAL_GPIO_BASE = 280
NEOPIXEL_CHANNELS_PER_INSTANCE = 2      # color, brightness
NEOPIXEL_MAX_INSTANCES = 4
NEOPIXEL_MAX_PIXELS = 300               # matches the catalog + Teensy/RP2040 cap

SUB_COLOR = 0
SUB_BRIGHTNESS = 1

# ── lazy hardware imports ─────────────────────────────────────────────

_WS281X_AVAILABLE = False
try:                                    # Pi 3/4 — PWM/DMA
    from rpi_ws281x import PixelStrip as _PixelStrip, Color as _Ws281xColor
    _WS281X_AVAILABLE = True
except Exception:                       # ImportError or runtime .so issues
    pass

_SPIDEV_AVAILABLE = False
try:                                    # any Pi incl. Pi 5 — SPI0 MOSI only
    import spidev as _spidev
    _SPIDEV_AVAILABLE = True
except Exception:
    pass


class _Ws281xStrip:
    """rpi_ws281x backend — arbitrary PWM-capable data pin (Pi 3/4)."""

    def __init__(self, pin: int, count: int, logger=None):
        self._count = count
        # LED_FREQ 800kHz, DMA 10, no invert, channel 0 — library defaults
        # used by virtually every wiring guide. Brightness is applied per
        # show() by scaling the color, matching the firmware drivers.
        self._px = _PixelStrip(count, pin, 800000, 10, False, 255, 0)
        self._px.begin()

    def show(self, r: int, g: int, b: int, brightness: int) -> None:
        c = _Ws281xColor(r * brightness // 255,
                         g * brightness // 255,
                         b * brightness // 255)
        for i in range(self._count):
            self._px.setPixelColor(i, c)
        self._px.show()

    def off(self) -> None:
        self.show(0, 0, 0, 0)

    def release(self) -> None:
        try:
            self.off()
        except Exception:
            pass


class _SpiStrip:
    """WS2812-over-SPI backend (Pi 5-safe). Data pin must be GPIO 10
    (SPI0 MOSI): each WS2812 bit becomes 3 SPI bits at 2.4 MHz
    (1 → 110, 0 → 100), plus >50 µs of zeros to latch."""

    SPI_HZ = 2_400_000
    LATCH_BYTES = 30    # 30 bytes * 8 bits / 2.4 MHz = 100 µs low

    def __init__(self, pin: int, count: int, logger=None):
        if pin != 10:
            raise ValueError(
                f"SPI NeoPixel backend requires data on GPIO 10 (SPI0 "
                f"MOSI); got GPIO {pin}. Rewire the strip or use a "
                f"Pi 3/4 with rpi_ws281x for arbitrary pins.")
        self._count = count
        self._spi = _spidev.SpiDev()
        self._spi.open(0, 0)
        self._spi.max_speed_hz = self.SPI_HZ
        self._spi.mode = 0

    @staticmethod
    def _encode_byte(value: int) -> bytes:
        """8 WS2812 bits → 24 SPI bits (3 per bit, MSB first)."""
        bits = 0
        for i in range(8):
            bits = (bits << 3) | (0b110 if (value >> (7 - i)) & 1 else 0b100)
        return bits.to_bytes(3, "big")

    def show(self, r: int, g: int, b: int, brightness: int) -> None:
        rs = r * brightness // 255
        gs = g * brightness // 255
        bs = b * brightness // 255
        # WS2812 wire order is GRB.
        pixel = (self._encode_byte(gs) + self._encode_byte(rs) +
                 self._encode_byte(bs))
        frame = pixel * self._count + b"\x00" * self.LATCH_BYTES
        self._spi.writebytes2(frame)

    def off(self) -> None:
        self.show(0, 0, 0, 0)

    def release(self) -> None:
        try:
            self.off()
            self._spi.close()
        except Exception:
            pass


class _LoggingStrip:
    """No WS2812 library available — keeps the config/control flow
    alive and visible in the logs, drives nothing."""

    def __init__(self, pin: int, count: int, logger=None):
        self._logger = logger
        self._pin = pin
        self._count = count
        self._say(f"NeoPixel [stub]: would drive {count} px on GPIO {pin} "
                  f"(install rpi_ws281x or spidev)")

    def _say(self, msg: str) -> None:
        if self._logger:
            self._logger.info(msg)
        else:
            print(msg)

    def show(self, r: int, g: int, b: int, brightness: int) -> None:
        self._say(f"NeoPixel [stub] GPIO {self._pin}: "
                  f"rgb=({r},{g},{b}) brightness={brightness}")

    def off(self) -> None:
        self._say(f"NeoPixel [stub] GPIO {self._pin}: off")

    def release(self) -> None:
        pass


def _default_strip_factory(pin: int, count: int, logger=None):
    """Pick the best available backend for this host."""
    if _WS281X_AVAILABLE:
        return _Ws281xStrip(pin, count, logger)
    if _SPIDEV_AVAILABLE:
        return _SpiStrip(pin, count, logger)
    return _LoggingStrip(pin, count, logger)


class NeoPixelDriver(PeripheralDriver):
    TYPE_ID = "neopixel"
    MODE_STRING = "neopixel"
    VIRTUAL_GPIO_BASE = NEOPIXEL_VIRTUAL_GPIO_BASE
    CHANNELS_PER_INSTANCE = NEOPIXEL_CHANNELS_PER_INSTANCE
    MAX_INSTANCES = NEOPIXEL_MAX_INSTANCES
    SUB_CHANNEL_NAMES = ["color", "brightness"]

    # Swappable for tests (RoboClaw _transport_cls pattern) and for a
    # forced backend choice if a deployment ever needs one.
    _strip_cls = staticmethod(_default_strip_factory)

    def __init__(self, logger=None):
        super().__init__(logger=logger)
        self._strips: Dict[int, Any] = {}
        # Last commanded color/brightness per instance (pre-scale),
        # kept OUTSIDE last_values so estop can re-render without
        # perturbing what get_value reports.
        self._rgb: Dict[int, tuple] = {}
        self._brightness: Dict[int, int] = {}

    # ── lifecycle ────────────────────────────────────────────────────

    def apply_config(self, instance_id: int, pins: Dict[str, int],
                     params: Dict[str, Any]) -> bool:
        pin = pins.get("data")
        if pin is None:
            self._log("warning",
                      f"NeoPixel {instance_id}: no 'data' pin in config — skipping")
            return False
        try:
            count = int(params.get("pixel_count", 1) or 1)
        except (TypeError, ValueError):
            count = 1
        count = max(1, min(NEOPIXEL_MAX_PIXELS, count))

        inst = self._get_or_create_instance(instance_id)
        if inst is None:
            return False

        # Re-point: tear down the old backend when pin/count moved
        # (mirrors the firmware drivers' re-create-on-change).
        old = self._strips.get(instance_id)
        if old is not None and (inst.pins.get("data") != pin
                                or inst.params.get("pixel_count") != count):
            old.release()
            old = None
            self._strips.pop(instance_id, None)

        if old is None:
            try:
                strip = self._strip_cls(int(pin), count, self._logger)
            except Exception as e:
                self._log("error",
                          f"NeoPixel {instance_id}: backend init failed on "
                          f"GPIO {pin}: {e}")
                inst.connected = False
                return False
            self._strips[instance_id] = strip

        inst.pins = {"data": int(pin)}
        inst.params = {"pixel_count": count}
        inst.connected = True
        self._rgb[instance_id] = (0, 0, 0)
        self._brightness[instance_id] = 255
        # Start dark — operator routing turns it on (firmware parity).
        self._strips[instance_id].show(0, 0, 0, 255)
        self._log("info",
                  f"NeoPixel: strip {instance_id} = {count} px on GPIO {pin} "
                  f"({type(self._strips[instance_id]).__name__})")
        return True

    def set_value(self, instance_id: int, sub_channel: int, value: float) -> bool:
        strip = self._strips.get(instance_id)
        inst = self._instances.get(instance_id)
        if strip is None or inst is None:
            return False

        if sub_channel == SUB_COLOR:
            # Packed 0xRRGGBB int carried as a float (identical to the
            # Teensy/RP2040 decode: (uint32_t)value then shift/mask).
            packed = int(value) & 0xFFFFFF
            r = (packed >> 16) & 0xFF
            g = (packed >> 8) & 0xFF
            b = packed & 0xFF
            self._rgb[instance_id] = (r, g, b)
        elif sub_channel == SUB_BRIGHTNESS:
            v = max(0.0, min(1.0, float(value)))
            self._brightness[instance_id] = int(v * 255.0 + 0.5)
        else:
            return False

        r, g, b = self._rgb.get(instance_id, (0, 0, 0))
        strip.show(r, g, b, self._brightness.get(instance_id, 255))
        inst.last_values[sub_channel] = float(value)
        return True

    def get_value(self, instance_id: int, sub_channel: int) -> Optional[float]:
        inst = self._instances.get(instance_id)
        if inst is None:
            return None
        return inst.last_values.get(sub_channel)

    def estop(self) -> None:
        # Dark, but keep the stored color — clear_estop plus a fresh
        # write restores operator state (firmware parity).
        for inst_id, strip in self._strips.items():
            try:
                strip.off()
            except Exception as e:
                self._log("warning", f"NeoPixel {inst_id}: estop off failed: {e}")
        self._log("warning", f"NeoPixel: ESTOP — {len(self._strips)} strip(s) dark")

    def clear_estop(self) -> None:
        # Strips stay dark until the operator commands a new value —
        # same convention as the motor drivers.
        pass

    def reset(self) -> None:
        for strip in self._strips.values():
            strip.release()
        self._strips.clear()
        self._rgb.clear()
        self._brightness.clear()
        super().reset()
