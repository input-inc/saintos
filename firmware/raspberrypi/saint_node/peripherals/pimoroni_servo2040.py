"""
SAINT.OS Pi node — Pimoroni Servo 2040 driver (I2C master).

Host-side driver for the Pimoroni Servo 2040 servo controller, driven
over I2C on the Pi's Qwiic/I2C bus. Mirrors the shared C driver
(firmware/shared/src/pimoroni_servo2040_driver.c) and the same wire
contract (firmware/shared/include/pimoroni_servo2040_protocol.h): the
board is an I2C target running the fixed SAINT.OS image
(firmware/pimoroni_servo2040/).

Feature parity with the Maestro Pi driver:
  - servo EXTENTS live host-side (this driver maps −1..+1 → pulse and
    clamps); the board only receives absolute pulse µs.
  - HOME pulses are pushed to the board (SERVO_HOME + COMMIT) and
    persisted to its flash so servos come up homed on power-on.

Channels: sub-channels 0..17 = servos "ch0".."ch17", 18..23 = onboard
RGB LEDs "led0".."led5", 24..26 = telemetry ("connected", "current_a",
"error_flags").
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

from .base import PeripheralDriver

# python-smbus2 is optional at import time — a node with no Servo 2040
# wired shouldn't fail to boot just because the lib isn't installed. The
# driver logs and stays inert if it's missing (mirrors the BLE/serial
# optional-dep pattern used elsewhere in this package).
try:
    from smbus2 import SMBus
    _SMBUS_OK = True
except Exception:  # pragma: no cover - import guard
    SMBus = None  # type: ignore
    _SMBUS_OK = False


# ── Wire contract — keep in sync with
#    firmware/shared/include/pimoroni_servo2040_protocol.h ─────────────
I2C_ADDR = 0x30
WHOAMI_MAGIC = 0x53
NUM_SERVOS = 18
NUM_LEDS = 6
VGPIO_BASE = 168

HARD_MIN_US = 400
HARD_MAX_US = 2600

REG_WHOAMI = 0x00
REG_STATUS = 0x02
REG_CURRENT_MA = 0x04
REG_SERVO_TARGET_BASE = 0x10
REG_SERVO_HOME_BASE = 0x40
REG_LED_BASE = 0x70
REG_BRIGHTNESS = 0x90
REG_COMMIT = 0xF0
REG_HEARTBEAT = 0xF1
REG_ESTOP = 0xF2

PING_MS = 250
POLL_MS = 200


def _u16_le(v: int) -> List[int]:
    v &= 0xFFFF
    return [v & 0xFF, (v >> 8) & 0xFF]


class PimoroniServo2040Driver(PeripheralDriver):
    """One Servo 2040 board on the Pi's I2C bus. One instance per node."""

    TYPE_ID = "pimoroni_servo2040"
    MODE_STRING = "pimoroni_servo"
    VIRTUAL_GPIO_BASE = VGPIO_BASE
    # 18 servos + 6 LEDs + 3 telemetry sub-channels.
    CHANNELS_PER_INSTANCE = NUM_SERVOS + NUM_LEDS + 3
    MAX_INSTANCES = 1
    SUB_CHANNEL_NAMES = (
        [f"ch{i}" for i in range(NUM_SERVOS)]
        + [f"led{i}" for i in range(NUM_LEDS)]
        + ["connected", "current_a", "error_flags"]
    )

    # Telemetry sub-channel indices.
    _SUB_CONNECTED = NUM_SERVOS + NUM_LEDS + 0
    _SUB_CURRENT = NUM_SERVOS + NUM_LEDS + 1
    _SUB_FLAGS = NUM_SERVOS + NUM_LEDS + 2

    def __init__(self, logger=None):
        super().__init__(logger=logger)
        self._bus: Optional["SMBus"] = None
        self._bus_num: int = 1
        # channel_idx -> (start_us, end_us, center_us, home_us)
        self._channel_cfg: Dict[int, tuple[int, int, int, int]] = {}
        self._led_brightness: int = 255
        self._present: bool = False
        self._current_a: float = 0.0
        self._flags: int = 0
        self._last_ping_ms: int = 0
        self._last_poll_ms: int = 0

    # ── helpers ────────────────────────────────────────────────────

    @staticmethod
    def _now_ms() -> int:
        import time
        return int(time.monotonic() * 1000)

    def _write_reg(self, reg: int, data: List[int]) -> bool:
        if self._bus is None:
            return False
        try:
            self._bus.write_i2c_block_data(I2C_ADDR, reg, data)
            return True
        except OSError:
            return False

    def _read_reg(self, reg: int, length: int) -> Optional[List[int]]:
        if self._bus is None:
            return None
        try:
            return self._bus.read_i2c_block_data(I2C_ADDR, reg, length)
        except OSError:
            return None

    def _normalized_to_pulse(self, sub_channel: int, value: float) -> int:
        start, end, center, _home = self._channel_cfg.get(
            sub_channel, (1000, 2000, 1500, 1500))
        value = max(-1.0, min(1.0, value))
        if value <= 0.0:
            pulse = center + value * (center - start)
        else:
            pulse = center + value * (end - center)
        return int(round(max(HARD_MIN_US, min(HARD_MAX_US, pulse))))

    def _provision_home(self) -> None:
        """Persist every channel's home pulse to the board's flash so it
        homes on power-on (Maestro EEPROM HomeMode=Goto parity)."""
        if not self._present:
            return
        for ch in range(NUM_SERVOS):
            _s, _e, _c, home = self._channel_cfg.get(ch, (1000, 2000, 1500, 1500))
            self._write_reg(REG_SERVO_HOME_BASE + ch * 2, _u16_le(home))
        self._write_reg(REG_COMMIT, [1])

    def _apply_connect_state(self) -> None:
        for ch in range(NUM_SERVOS):
            _s, _e, _c, home = self._channel_cfg.get(ch, (1000, 2000, 1500, 1500))
            if home:
                self._write_reg(REG_SERVO_TARGET_BASE + ch * 2, _u16_le(home))
        self._write_reg(REG_BRIGHTNESS, [self._led_brightness & 0xFF])
        self._provision_home()

    # ── PeripheralDriver overrides ─────────────────────────────────

    def apply_config(self, instance_id: int, pins: Dict[str, int],
                     params: Dict[str, Any]) -> bool:
        if instance_id != 0:
            self._log("warn", "Servo2040: only one instance supported")
            return False
        if not _SMBUS_OK:
            self._log("error",
                      "Servo2040: smbus2 not installed — I2C unavailable")
            return False

        # The Pi's Qwiic / 40-pin I2C is bus 1 (GPIO2 SDA / GPIO3 SCL); an
        # operator can override with an explicit i2c_bus param for HAT
        # multiplexers or bus 0.
        bus_num = int(params.get("i2c_bus", 1))
        self._led_brightness = max(0, min(255, int(params.get("led_brightness", 255))))

        if self._bus is None or self._bus_num != bus_num:
            try:
                if self._bus is not None:
                    self._bus.close()
                self._bus = SMBus(bus_num)
                self._bus_num = bus_num
            except Exception as e:  # pragma: no cover - hardware path
                self._log("error", f"Servo2040: opening /dev/i2c-{bus_num} failed: {e}")
                self._bus = None
                return False

        # Per-channel extents (list keyed by channel index, matching the
        # server's pimoroni_normalize_channels output).
        channels = params.get("channels", []) or []
        self._channel_cfg.clear()
        if isinstance(channels, dict):
            ch_iter = channels.items()
        elif isinstance(channels, list):
            ch_iter = enumerate(channels)
        else:
            ch_iter = []
        for k, cfg in ch_iter:
            try:
                ch = int(k)
            except (TypeError, ValueError):
                continue
            if not (0 <= ch < NUM_SERVOS) or not isinstance(cfg, dict):
                continue
            self._channel_cfg[ch] = (
                int(cfg.get("start_us", 1000)),
                int(cfg.get("end_us", 2000)),
                int(cfg.get("center_us", 1500)),
                int(cfg.get("home_us", 1500)),
            )

        inst = self._get_or_create_instance(0)
        inst.pins = dict(pins)
        inst.params = dict(params)
        inst.connected = False   # confirmed by the WHOAMI poll in update()

        # Push initial state if the board is already answering.
        who = self._read_reg(REG_WHOAMI, 1)
        self._present = bool(who and who[0] == WHOAMI_MAGIC)
        if self._present:
            self._apply_connect_state()
        return True

    def set_value(self, instance_id: int, sub_channel: int, value: float) -> bool:
        if instance_id != 0 or self._bus is None:
            return False
        if 0 <= sub_channel < NUM_SERVOS:
            pulse = self._normalized_to_pulse(sub_channel, value)
            ok = self._write_reg(REG_SERVO_TARGET_BASE + sub_channel * 2, _u16_le(pulse))
        elif NUM_SERVOS <= sub_channel < NUM_SERVOS + NUM_LEDS:
            idx = sub_channel - NUM_SERVOS
            rgb = int(value) & 0xFFFFFF
            ok = self._write_reg(
                REG_LED_BASE + idx * 3,
                [(rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF])
        else:
            return False   # telemetry sub-channels are read-only
        if ok:
            inst = self._instances.get(0)
            if inst is not None:
                inst.last_values[sub_channel] = value
        return ok

    def get_value(self, instance_id: int, sub_channel: int) -> Optional[float]:
        if instance_id != 0:
            return None
        if sub_channel == self._SUB_CONNECTED:
            return 1.0 if self._present else 0.0
        if sub_channel == self._SUB_CURRENT:
            return self._current_a
        if sub_channel == self._SUB_FLAGS:
            return float(self._flags)
        inst = self._instances.get(0)
        return inst.last_values.get(sub_channel) if inst else None

    def update(self) -> None:
        if self._bus is None:
            return
        now = self._now_ms()

        if now - self._last_poll_ms >= POLL_MS:
            self._last_poll_ms = now
            who = self._read_reg(REG_WHOAMI, 1)
            present = bool(who and who[0] == WHOAMI_MAGIC)
            if present and not self._present:
                self._present = True
                self._log("info", "Servo2040: board detected on I2C")
                self._apply_connect_state()
            elif not present and self._present:
                self._present = False
                self._log("warn", "Servo2040: board lost on I2C")
            if self._present:
                cur = self._read_reg(REG_CURRENT_MA, 2)
                if cur:
                    self._current_a = ((cur[0] | (cur[1] << 8)) / 1000.0)
                st = self._read_reg(REG_STATUS, 1)
                if st:
                    self._flags = st[0]
            inst = self._instances.get(0)
            if inst is not None:
                inst.connected = self._present

        if self._present and now - self._last_ping_ms >= PING_MS:
            self._last_ping_ms = now
            self._write_reg(REG_HEARTBEAT, [1])

    def estop(self) -> None:
        self._write_reg(REG_ESTOP, [1])

    def reset(self) -> None:
        if self._bus is not None:
            try:
                self._bus.close()
            except Exception:  # pragma: no cover
                pass
        self._bus = None
        self._present = False
        super().reset()
