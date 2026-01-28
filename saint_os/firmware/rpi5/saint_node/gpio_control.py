"""
GPIO Controller for Raspberry Pi 5

Provides GPIO control using libgpiod (gpiod) for digital I/O
and hardware PWM for servo/PWM outputs.

Raspberry Pi 5 GPIO differences from earlier models:
- Uses RP1 chip for GPIO (not BCM2835/BCM2711)
- libgpiod is the recommended interface (not RPi.GPIO)
- Hardware PWM available on GPIO 12, 13, 18, 19
"""

import os
import time
from typing import Dict, List, Any, Optional
from dataclasses import dataclass, field
from enum import Enum

# Check for simulation mode (set via environment variable)
import os
SIMULATION_MODE = os.environ.get('SAINT_SIMULATION', '0') == '1'

# Try to import gpiod, fall back to mock for development/simulation
GPIOD_AVAILABLE = False
if not SIMULATION_MODE:
    try:
        import gpiod
        from gpiod.line import Direction, Value, Bias
        GPIOD_AVAILABLE = True
    except ImportError:
        pass

# Mock classes for simulation
if not GPIOD_AVAILABLE:
    class Value:
        ACTIVE = 1
        INACTIVE = 0

    class Direction:
        INPUT = 0
        OUTPUT = 1

    class Bias:
        DISABLED = 0
        PULL_UP = 1
        PULL_DOWN = 2


class PinMode(Enum):
    """Supported pin modes."""
    UNCONFIGURED = "unconfigured"
    DIGITAL_IN = "digital_in"
    DIGITAL_OUT = "digital_out"
    PWM = "pwm"
    SERVO = "servo"
    # I2C and SPI are typically handled by kernel drivers
    I2C_SDA = "i2c_sda"
    I2C_SCL = "i2c_scl"
    SPI_MOSI = "spi_mosi"
    SPI_MISO = "spi_miso"
    SPI_CLK = "spi_clk"
    SPI_CS = "spi_cs"


@dataclass
class PinConfig:
    """Configuration for a single pin."""
    gpio: int
    mode: PinMode = PinMode.UNCONFIGURED
    logical_name: str = ""
    value: float = 0.0
    frequency: int = 1000  # For PWM
    pull: str = "none"  # none, up, down


@dataclass
class PinCapability:
    """Capabilities of a single pin."""
    gpio: int
    name: str
    capabilities: List[str] = field(default_factory=list)


class GPIOController:
    """
    GPIO controller for Raspberry Pi 5.

    Uses libgpiod for digital I/O and software PWM for PWM outputs.
    Hardware PWM is available on specific pins if needed.
    """

    # GPIO chip name for Pi 5 (RP1)
    GPIO_CHIP = "gpiochip4"  # Pi 5 uses gpiochip4 for the main GPIO header

    # Reserved pins (typically used for system functions)
    RESERVED_PINS = [
        0, 1,    # I2C0 (ID EEPROM)
        14, 15,  # UART0 (console)
    ]

    # Pins with hardware PWM capability
    PWM_PINS = [12, 13, 18, 19]

    # All available GPIO pins on the 40-pin header
    # Excluding reserved pins
    AVAILABLE_GPIOS = [
        2, 3,    # I2C1
        4,       # GPCLK0
        5, 6,    # General GPIO
        7, 8,    # SPI0 CE1, CE0
        9, 10, 11,  # SPI0 MISO, MOSI, SCLK
        12, 13,  # PWM0, PWM1
        16, 17,  # General GPIO
        18, 19,  # PCM CLK, FS (also PWM)
        20, 21,  # PCM DIN, DOUT
        22, 23, 24, 25, 26, 27,  # General GPIO
    ]

    def __init__(self, logger=None):
        self._logger = logger
        self._pins: Dict[int, PinConfig] = {}
        self._chip = None
        self._lines: Dict[int, Any] = {}  # gpio -> line request

        # Software PWM state
        self._pwm_running: Dict[int, bool] = {}
        self._pwm_duty: Dict[int, float] = {}

        self._init_gpio()

    def _log(self, level: str, msg: str):
        """Log a message."""
        if self._logger:
            getattr(self._logger, level)(msg)
        else:
            print(f"[{level.upper()}] {msg}")

    def _init_gpio(self):
        """Initialize GPIO chip."""
        if SIMULATION_MODE:
            self._log('info', 'Running in SIMULATION mode - GPIO operations are mocked')
            self._mock_values: Dict[int, float] = {}  # Simulated pin values
            return

        if not GPIOD_AVAILABLE:
            self._log('warn', 'gpiod not available - running in mock mode')
            self._mock_values: Dict[int, float] = {}
            return

        try:
            self._chip = gpiod.Chip(self.GPIO_CHIP)
            self._log('info', f'Opened GPIO chip: {self.GPIO_CHIP}')
        except Exception as e:
            self._log('error', f'Failed to open GPIO chip: {e}')
            # Try alternative chip names
            for chip_name in ['gpiochip0', 'gpiochip4', '/dev/gpiochip4']:
                try:
                    self._chip = gpiod.Chip(chip_name)
                    self._log('info', f'Opened GPIO chip: {chip_name}')
                    break
                except Exception:
                    continue

    def get_capabilities(self) -> Dict[str, Any]:
        """Get node GPIO capabilities."""
        available_pins = []

        for gpio in self.AVAILABLE_GPIOS:
            if gpio in self.RESERVED_PINS:
                continue

            caps = ['digital_io']

            # PWM capability
            if gpio in self.PWM_PINS:
                caps.append('pwm')
                caps.append('servo')
            else:
                # Software PWM available on all pins
                caps.append('pwm_soft')
                caps.append('servo')

            # I2C pins
            if gpio in [2, 3]:
                caps.append('i2c')

            # SPI pins
            if gpio in [7, 8, 9, 10, 11]:
                caps.append('spi')

            available_pins.append({
                'gpio': gpio,
                'name': f'GPIO{gpio}',
                'capabilities': caps,
            })

        return {
            'available_pins': available_pins,
            'reserved_pins': self.RESERVED_PINS,
            'features': ['digital_io', 'pwm', 'servo', 'i2c', 'spi'],
        }

    def configure_pin(self, gpio: int, mode: str, logical_name: str = "",
                     frequency: int = 1000, pull: str = "none", **kwargs):
        """
        Configure a GPIO pin.

        Args:
            gpio: GPIO number
            mode: Pin mode (digital_in, digital_out, pwm, servo)
            logical_name: Human-readable name
            frequency: PWM frequency in Hz
            pull: Pull resistor (none, up, down)
        """
        if gpio in self.RESERVED_PINS:
            raise ValueError(f'GPIO {gpio} is reserved')

        if gpio not in self.AVAILABLE_GPIOS:
            raise ValueError(f'GPIO {gpio} is not available')

        # Release existing configuration
        self._release_pin(gpio)

        # Create pin config
        pin_mode = PinMode(mode) if mode else PinMode.UNCONFIGURED
        config = PinConfig(
            gpio=gpio,
            mode=pin_mode,
            logical_name=logical_name,
            frequency=frequency,
            pull=pull,
        )

        self._pins[gpio] = config

        # Apply hardware configuration
        self._apply_pin_config(config)

        self._log('info', f'Configured GPIO {gpio} as {mode} ({logical_name})')

    def _apply_pin_config(self, config: PinConfig):
        """Apply hardware configuration for a pin."""
        gpio = config.gpio

        # In simulation/mock mode, just track the pin
        if SIMULATION_MODE or not GPIOD_AVAILABLE or not self._chip:
            if hasattr(self, '_mock_values'):
                self._mock_values[gpio] = 0.0
            self._log('info', f'[SIM] Configured GPIO {gpio} as {config.mode.value}')
            return

        try:
            if config.mode == PinMode.DIGITAL_IN:
                # Configure as input
                line_config = gpiod.LineSettings(
                    direction=Direction.INPUT,
                    bias=self._get_bias(config.pull),
                )
                req = self._chip.request_lines(
                    config={gpio: line_config},
                    consumer=f"saint_node_gpio{gpio}"
                )
                self._lines[gpio] = req

            elif config.mode == PinMode.DIGITAL_OUT:
                # Configure as output
                line_config = gpiod.LineSettings(
                    direction=Direction.OUTPUT,
                    output_value=Value.INACTIVE,
                )
                req = self._chip.request_lines(
                    config={gpio: line_config},
                    consumer=f"saint_node_gpio{gpio}"
                )
                self._lines[gpio] = req

            elif config.mode in [PinMode.PWM, PinMode.SERVO]:
                # For PWM/Servo, configure as output and use software PWM
                line_config = gpiod.LineSettings(
                    direction=Direction.OUTPUT,
                    output_value=Value.INACTIVE,
                )
                req = self._chip.request_lines(
                    config={gpio: line_config},
                    consumer=f"saint_node_gpio{gpio}"
                )
                self._lines[gpio] = req

                # Initialize PWM state
                self._pwm_duty[gpio] = 0.0
                self._pwm_running[gpio] = False

        except Exception as e:
            self._log('error', f'Failed to configure GPIO {gpio}: {e}')
            raise

    def _get_bias(self, pull: str):
        """Convert pull string to gpiod bias."""
        if not GPIOD_AVAILABLE:
            return None
        if pull == 'up':
            return Bias.PULL_UP
        elif pull == 'down':
            return Bias.PULL_DOWN
        else:
            return Bias.DISABLED

    def _release_pin(self, gpio: int):
        """Release a pin's resources."""
        if gpio in self._lines:
            try:
                self._lines[gpio].release()
            except Exception:
                pass
            del self._lines[gpio]

        if gpio in self._pwm_running:
            self._pwm_running[gpio] = False

        if gpio in self._pins:
            del self._pins[gpio]

    def set_value(self, gpio: int, value: float):
        """
        Set pin value.

        Args:
            gpio: GPIO number
            value: Value to set
                - Digital: 0 or 1
                - PWM: 0-100 (duty cycle percentage)
                - Servo: 0-180 (degrees)
        """
        if gpio not in self._pins:
            self._log('warn', f'GPIO {gpio} not configured')
            return

        config = self._pins[gpio]
        config.value = value

        # In simulation mode, just store the value
        if SIMULATION_MODE or (not GPIOD_AVAILABLE and hasattr(self, '_mock_values')):
            self._mock_values[gpio] = value
            self._log('debug', f'[SIM] GPIO {gpio} = {value}')
            return

        if config.mode == PinMode.DIGITAL_OUT:
            self._set_digital(gpio, value > 0.5)

        elif config.mode == PinMode.PWM:
            self._set_pwm_duty(gpio, value)

        elif config.mode == PinMode.SERVO:
            self._set_servo_angle(gpio, value)

    def _set_digital(self, gpio: int, high: bool):
        """Set digital output value."""
        if gpio not in self._lines:
            return

        try:
            val = Value.ACTIVE if high else Value.INACTIVE
            self._lines[gpio].set_value(gpio, val)
        except Exception as e:
            self._log('error', f'Failed to set GPIO {gpio}: {e}')

    def _set_pwm_duty(self, gpio: int, duty: float):
        """
        Set PWM duty cycle (0-100%).

        Note: This is a simplified implementation. For precise PWM,
        consider using pigpio or hardware PWM.
        """
        duty = max(0.0, min(100.0, duty))
        self._pwm_duty[gpio] = duty

        # For now, use simple threshold (on if >50%, off otherwise)
        # A proper implementation would use a separate thread or timer
        if gpio in self._lines:
            try:
                val = Value.ACTIVE if duty > 50 else Value.INACTIVE
                self._lines[gpio].set_value(gpio, val)
            except Exception as e:
                self._log('error', f'Failed to set PWM on GPIO {gpio}: {e}')

    def _set_servo_angle(self, gpio: int, angle: float):
        """
        Set servo angle (0-180 degrees).

        Standard servo pulse width: 500us (0°) to 2500us (180°)
        Period: 20ms (50Hz)
        """
        angle = max(0.0, min(180.0, angle))

        # Convert angle to duty cycle
        # 50Hz = 20ms period
        # 0° = 0.5ms = 2.5% duty
        # 180° = 2.5ms = 12.5% duty
        duty = 2.5 + (angle / 180.0) * 10.0

        self._pwm_duty[gpio] = duty

        # Simplified: just store the value, actual servo control
        # would need precise timing (consider pigpio for production)
        if gpio in self._lines:
            try:
                # For testing, just set high if angle > 90
                val = Value.ACTIVE if angle > 90 else Value.INACTIVE
                self._lines[gpio].set_value(gpio, val)
            except Exception as e:
                self._log('error', f'Failed to set servo on GPIO {gpio}: {e}')

    def get_value(self, gpio: int) -> float:
        """Get current pin value."""
        if gpio not in self._pins:
            return 0.0

        config = self._pins[gpio]

        # In simulation mode, return mock value
        if SIMULATION_MODE or (not GPIOD_AVAILABLE and hasattr(self, '_mock_values')):
            return self._mock_values.get(gpio, 0.0)

        if config.mode == PinMode.DIGITAL_IN:
            return self._read_digital(gpio)
        elif config.mode in [PinMode.PWM, PinMode.SERVO]:
            return self._pwm_duty.get(gpio, 0.0)
        else:
            return config.value

    def _read_digital(self, gpio: int) -> float:
        """Read digital input value."""
        if gpio not in self._lines:
            return 0.0

        try:
            val = self._lines[gpio].get_value(gpio)
            return 1.0 if val == Value.ACTIVE else 0.0
        except Exception as e:
            self._log('error', f'Failed to read GPIO {gpio}: {e}')
            return 0.0

    def get_all_states(self) -> List[Dict[str, Any]]:
        """Get state of all configured pins."""
        states = []

        for gpio, config in self._pins.items():
            states.append({
                'gpio': gpio,
                'mode': config.mode.value,
                'value': self.get_value(gpio),
                'name': config.logical_name,
            })

        return states

    def reset_all(self):
        """Reset all pins to default (unconfigured) state."""
        for gpio in list(self._pins.keys()):
            self._release_pin(gpio)

    def cleanup(self):
        """Clean up all GPIO resources."""
        self._log('info', 'Cleaning up GPIO resources')

        # Release all lines
        for gpio in list(self._lines.keys()):
            self._release_pin(gpio)

        # Close chip
        if self._chip:
            try:
                self._chip.close()
            except Exception:
                pass
            self._chip = None
