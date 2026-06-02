#!/usr/bin/env python3
"""
SAINT.OS Raspberry Pi Node

Main ROS2 node implementation for Raspberry Pi 3 / 4 / 5 GPIO control
and on-host peripherals (audio playback, future I²S/I²C/SPI peripherals).
Communicates with SAINT.OS server using the same protocol as the
microcontroller node firmwares (RP2040, Teensy 4.1).

Model-specific behavior (which GPIO chip libgpiod opens, what the
announcement reports under ``hw``) is detected at startup from
``/proc/device-tree/model`` so a single firmware build runs on any
supported Pi without operator configuration.
"""

import json
import time
import socket
import hashlib
from enum import Enum
from typing import Optional, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

from .gpio_control import GPIOController
from .config import ConfigManager
from .updater import FirmwareUpdater
from .peripheral_manager import PeripheralManager
from .peripherals.syren import SyRenDriver
from .peripherals.maestro import MaestroDriver
from .peripherals.roboclaw import RoboClawDriver
from .peripherals.tic import TicDriver
from .peripherals.pathfinder_bms import PathfinderBMSDriver
from .peripherals.fas100 import FAS100Driver
from .peripherals.audio_player import PiAudioPlayerDriver
from .pi_model import detect_pi_model


class NodeState(Enum):
    """Node lifecycle states."""
    BOOT = "BOOT"
    CONNECTING = "CONNECTING"
    UNADOPTED = "UNADOPTED"
    ADOPTING = "ADOPTING"
    ACTIVE = "ACTIVE"
    UPDATING = "UPDATING"
    ERROR = "ERROR"


class SaintNode(Node):
    """
    SAINT.OS Raspberry Pi Node — runs on Pi 3 / Pi 4 / Pi 5.

    Implements the same communication protocol as RP2040 nodes:
    - Publishes announcements to /saint/nodes/announce
    - Publishes capabilities to /saint/nodes/<id>/capabilities
    - Publishes state to /saint/nodes/<id>/state
    - Subscribes to /saint/nodes/<id>/config for configuration
    - Subscribes to /saint/nodes/<id>/control for runtime control
    """

    # Server-side board catalog key. Must match a directory under
    # server/config/boards/; the server uses this to assign a default
    # board (and its built-in peripherals — onboard_audio, etc.) on
    # adoption. One catalog entry serves every Pi generation — model-
    # specific bits (audio jack, GPIO chip name) are runtime concerns.
    CHIP_FAMILY = "raspberrypi"

    def __init__(self):
        # Detect actual Pi model from /proc/device-tree/model so the
        # announcement carries "Raspberry Pi 4 Model B" / "Raspberry
        # Pi 5" / etc. rather than a hardcoded constant. Falls back to
        # plain "Raspberry Pi" off-Pi (dev host) so the firmware boots
        # in CI / on a Mac without crashing.
        self.HW_TYPE = detect_pi_model()

        # Generate node ID before calling super().__init__
        self._node_id = self._generate_node_id()

        super().__init__(f'saint_node_{self._node_id}')

        self.get_logger().info(f'Starting SAINT Node: {self._node_id}')

        # Initialize state
        self._state = NodeState.BOOT
        self._start_time = time.time()
        self._last_announce = 0.0
        self._last_state_publish = 0.0

        # Configuration manager
        self._config = ConfigManager(self._node_id, self.get_logger())
        self._config.load()

        # GPIO controller
        self._gpio = GPIOController(self.get_logger())

        # Peripheral manager: owns one instance of each peripheral
        # driver (SyRen, Maestro, RoboClaw, Tic, Pathfinder BMS). Each
        # driver claims a virtual GPIO range (200+); set_pin messages
        # that target a virtual GPIO get dispatched here instead of
        # going to gpio_control.
        self._peripherals = PeripheralManager(self.get_logger())
        self._peripherals.register(SyRenDriver)
        self._peripherals.register(MaestroDriver)
        self._peripherals.register(RoboClawDriver)
        self._peripherals.register(TicDriver)
        self._peripherals.register(PathfinderBMSDriver)
        self._peripherals.register(FAS100Driver)
        # Built-in audio playback: lives on every Pi-host saint-node,
        # auto-seeded by the raspberrypi board YAML's builtin_peripherals
        # entry. libVLC is loaded lazily — a missing system package
        # logs an error but doesn't crash the node.
        self._peripherals.register(PiAudioPlayerDriver)

        # Firmware updater
        self._updater = FirmwareUpdater(self.get_logger())
        self._updater.set_progress_callback(self._on_update_progress)

        # Apply saved configuration if adopted
        if self._config.is_adopted():
            self._apply_saved_config()
            self._state = NodeState.ACTIVE
        else:
            self._state = NodeState.UNADOPTED

        # QoS profiles
        self._qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        # Streaming control commands use BEST_EFFORT + KEEP_LAST(1) so a
        # dropped UDP packet doesn't queue subsequent commands behind
        # retransmissions — the deadstick (return-to-zero) needs to be
        # the freshest thing at the head of the queue, not stuck behind
        # nine stale non-zero values. Must match the server-side
        # publisher's QoS (see server_node.py CONTROL_QOS).
        self._qos_control = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # Publishers
        self._pub_announce = self.create_publisher(
            String, '/saint/nodes/announce', self._qos_reliable)
        self._pub_capabilities = self.create_publisher(
            String, f'/saint/nodes/{self._node_id}/capabilities', self._qos_reliable)
        self._pub_state = self.create_publisher(
            String, f'/saint/nodes/{self._node_id}/state', self._qos_reliable)
        self._pub_update_progress = self.create_publisher(
            String, f'/saint/nodes/{self._node_id}/update_progress', self._qos_reliable)

        # Subscribers
        self._sub_config = self.create_subscription(
            String, f'/saint/nodes/{self._node_id}/config',
            self._on_config_message, self._qos_reliable)
        self._sub_control = self.create_subscription(
            String, f'/saint/nodes/{self._node_id}/control',
            self._on_control_message, self._qos_control)
        # One-shot operator actions (firmware_update, factory_reset,
        # reboot, check_update). RELIABLE so a dropped UDP packet
        # doesn't silently lose an OTA trigger — see server_node.py
        # CONTROL_QOS/COMMAND_QOS for the rationale. Same callback as
        # /control: the action match-on-string is the actual router,
        # the topic split is purely for QoS.
        self._sub_command = self.create_subscription(
            String, f'/saint/nodes/{self._node_id}/command',
            self._on_control_message, self._qos_reliable)

        # Main loop timer (10 Hz)
        self._timer = self.create_timer(0.1, self._main_loop)

        self.get_logger().info(f'Node initialized in state: {self._state.value}')

    def _generate_node_id(self) -> str:
        """Generate unique node ID from hardware identifiers."""
        import os

        # Check for override (used in simulation)
        override_id = os.environ.get('SAINT_NODE_ID')
        if override_id:
            return override_id

        try:
            # Try to get CPU serial from /proc/cpuinfo
            with open('/proc/cpuinfo', 'r') as f:
                for line in f:
                    if line.startswith('Serial'):
                        serial = line.split(':')[1].strip()
                        # Use last 12 characters of hash
                        hash_id = hashlib.md5(serial.encode()).hexdigest()[:12]
                        return f'raspberrypi_{hash_id}'
        except Exception:
            pass

        # Fallback: use MAC address
        try:
            mac = self._get_mac_address()
            if mac:
                hash_id = hashlib.md5(mac.encode()).hexdigest()[:12]
                return f'raspberrypi_{hash_id}'
        except Exception:
            pass

        # Last resort: use hostname
        hostname = socket.gethostname()
        hash_id = hashlib.md5(hostname.encode()).hexdigest()[:12]
        return f'raspberrypi_{hash_id}'

    def _get_mac_address(self) -> Optional[str]:
        """Get MAC address of primary network interface."""
        try:
            # Try common interface names
            for iface in ['eth0', 'wlan0', 'end0', 'wlan1']:
                try:
                    with open(f'/sys/class/net/{iface}/address', 'r') as f:
                        return f.read().strip()
                except FileNotFoundError:
                    continue
        except Exception:
            pass
        return None

    def _get_ip_address(self) -> str:
        """Get current IP address."""
        try:
            # Create a socket to determine the outgoing IP
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(('8.8.8.8', 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return '0.0.0.0'

    def _apply_saved_config(self):
        """Apply saved pin configuration."""
        pin_configs = self._config.get_pin_configs()
        for gpio, config in pin_configs.items():
            try:
                self._gpio.configure_pin(
                    gpio=int(gpio),
                    mode=config.get('mode', 'unconfigured'),
                    logical_name=config.get('logical_name', ''),
                    **{k: v for k, v in config.items() if k not in ['mode', 'logical_name']}
                )
            except Exception as e:
                self.get_logger().error(f'Failed to configure GPIO {gpio}: {e}')

    def _main_loop(self):
        """Main processing loop (10 Hz)."""
        now = time.time()

        # Tick every peripheral driver (keepalives, telemetry polls,
        # etc.). Cheap when no peripherals are configured; required
        # for things like RoboClaw duty keepalive even when the
        # operator isn't sliding a control.
        self._peripherals.update()

        # Publish announcements (1 Hz)
        if now - self._last_announce >= 1.0:
            self._publish_announcement()
            self._last_announce = now

        # Publish state when active (10 Hz)
        if self._state == NodeState.ACTIVE:
            if now - self._last_state_publish >= 0.1:
                self._publish_state()
                self._last_state_publish = now

    def _publish_announcement(self):
        """Publish node announcement message."""
        from . import __version__

        msg_data = {
            'node_id': self._node_id,
            'mac': self._get_mac_address() or 'unknown',
            'ip': self._get_ip_address(),
            'hw': self.HW_TYPE,
            'chip_family': self.CHIP_FAMILY,
            'fw': __version__,
            'fw_build': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            'state': self._state.value,
            'uptime': int(time.time() - self._start_time),
            'role': self._config.get_role() or 'none',
            'display_name': self._config.get_display_name() or '',
        }

        msg = String()
        msg.data = json.dumps(msg_data)
        self._pub_announce.publish(msg)

    def _publish_capabilities(self):
        """Publish node capabilities — physical GPIO pins from
        gpio_control plus the peripheral types this node has registered
        drivers for. The server uses peripheral_types to decide which
        catalog entries are syncable to this node."""
        capabilities = self._gpio.get_capabilities()

        peripheral_types = [
            driver.TYPE_ID for driver in self._peripherals.all_drivers()
        ]

        msg_data = {
            'node_id': self._node_id,
            'available_pins': capabilities['available_pins'],
            'reserved_pins': capabilities['reserved_pins'],
            'total_pins': len(capabilities['available_pins']),
            'features': capabilities.get('features', []),
            'peripheral_types': peripheral_types,
        }

        msg = String()
        msg.data = json.dumps(msg_data)
        self._pub_capabilities.publish(msg)
        self.get_logger().info(
            f'Published capabilities ({len(peripheral_types)} peripheral types)')

    def _publish_state(self):
        """Publish current pin state — physical GPIOs + peripheral
        virtual channels in the same `pins` array. The server filters
        by channel range to surface what the dashboard cares about."""
        pin_states = self._gpio.get_all_states()
        pin_states += self._peripherals.collect_states()

        msg_data = {
            'node_id': self._node_id,
            'timestamp': int(time.time() * 1000),
            'pins': pin_states,
        }

        msg = String()
        msg.data = json.dumps(msg_data)
        self._pub_state.publish(msg)

    def _on_config_message(self, msg: String):
        """Handle configuration message from server."""
        try:
            data = json.loads(msg.data)
            action = data.get('action', '')

            self.get_logger().info(f'Received config action: {action}')

            if action == 'request_capabilities':
                self._publish_capabilities()

            elif action == 'configure':
                # Peripheral-first format takes precedence over the
                # legacy "pins" format. Server emits one or the other,
                # not both.
                if 'peripherals' in data:
                    self._peripherals.apply_peripherals(
                        data.get('peripherals', []))
                    self._state = NodeState.ACTIVE
                    self.get_logger().info(
                        'Configuration applied (peripherals), now ACTIVE')
                else:
                    self._handle_configure(data)

            elif action == 'adopt':
                self._handle_adopt(data)

            else:
                self.get_logger().warn(f'Unknown config action: {action}')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid config JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Config handling error: {e}')

    def _handle_adopt(self, data: Dict[str, Any]):
        """Handle node adoption."""
        role = data.get('role', 'none')
        display_name = data.get('display_name', '')

        self._config.set_role(role)
        self._config.set_display_name(display_name)
        self._config.save()

        self._state = NodeState.ADOPTING
        self.get_logger().info(f'Adopted with role: {role}')

        # Publish capabilities after adoption
        self._publish_capabilities()

    def _handle_configure(self, data: Dict[str, Any]):
        """Handle pin configuration."""
        pins_config = data.get('pins', {})

        for gpio_str, config in pins_config.items():
            try:
                gpio = int(gpio_str)
                mode = config.get('mode', 'unconfigured')
                logical_name = config.get('logical_name', '')

                self._gpio.configure_pin(
                    gpio=gpio,
                    mode=mode,
                    logical_name=logical_name,
                    **{k: v for k, v in config.items() if k not in ['mode', 'logical_name']}
                )

                self.get_logger().info(f'Configured GPIO {gpio} as {mode}')

            except Exception as e:
                self.get_logger().error(f'Failed to configure GPIO {gpio_str}: {e}')

        # Save configuration
        self._config.set_pin_configs(pins_config)
        self._config.save()

        # Transition to active
        self._state = NodeState.ACTIVE
        self.get_logger().info('Configuration applied, now ACTIVE')

    def _on_control_message(self, msg: String):
        """Handle control message from server."""
        try:
            data = json.loads(msg.data)
            action = data.get('action', '')

            if action == 'set_pin':
                gpio = data.get('gpio')
                value = data.get('value')

                if gpio is None or value is None:
                    self.get_logger().warn('set_pin missing gpio or value')
                else:
                    gpio_i = int(gpio)
                    # Peripheral virtual GPIOs (≥200) go through the
                    # peripheral manager; everything else goes to the
                    # physical GPIO controller.
                    if self._peripherals.is_peripheral_vgpio(gpio_i):
                        self._peripherals.set_value(gpio_i, float(value))
                    else:
                        self._gpio.set_value(gpio_i, float(value))

            elif action == 'set_channel':
                # Peripheral-first addressing: server sends a
                # (peripheral_id, channel_id, value) triple and the
                # manager resolves to (driver, instance, sub_channel)
                # by logical name. Used for pinless built-ins like
                # the on-board audio player.
                peripheral_id = data.get('peripheral')
                channel_id = data.get('channel')
                value = data.get('value')
                if not peripheral_id or not channel_id or value is None:
                    self.get_logger().warn(
                        'set_channel missing peripheral/channel/value')
                else:
                    self._peripherals.set_channel_value(
                        str(peripheral_id), str(channel_id), float(value))

            elif action == 'peripheral_command':
                # Out-of-band command for peripherals that take
                # non-numeric args — audio_player's `play_file`
                # (filename), future display drivers' text payloads,
                # etc.
                peripheral_id = data.get('peripheral')
                command = data.get('command')
                args = data.get('args') or {}
                if not peripheral_id or not command:
                    self.get_logger().warn(
                        'peripheral_command missing peripheral or command')
                else:
                    self._peripherals.dispatch_command(
                        str(peripheral_id), str(command), dict(args))

            elif action == 'estop':
                self._handle_estop()

            elif action == 'clear_estop':
                self._handle_clear_estop()

            elif action == 'factory_reset':
                self._handle_factory_reset()

            elif action == 'reboot':
                self._handle_reboot()

            elif action == 'firmware_update':
                self._handle_firmware_update(data)

            elif action == 'check_update':
                self._handle_check_update(data)

            else:
                self.get_logger().warn(f'Unknown control action: {action}')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid control JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Control handling error: {e}')

    def _handle_factory_reset(self):
        """Perform factory reset."""
        self.get_logger().warn('Factory reset requested')

        # Reset all GPIO to default
        self._gpio.reset_all()

        # Wipe every peripheral driver's state too (closes serial
        # ports, drops instance config).
        self._peripherals.reset()

        # Clear configuration
        self._config.factory_reset()

        # Return to unadopted state
        self._state = NodeState.UNADOPTED

        self.get_logger().info('Factory reset complete')

    def _handle_estop(self):
        """Operator-triggered emergency stop: fan out to every
        peripheral driver so write-side motion is halted as fast as
        possible. Doesn't affect physical GPIO state — operator can
        re-arm by sending fresh values."""
        self.get_logger().warn('ESTOP requested')
        self._peripherals.estop()

    def _handle_clear_estop(self):
        """Release latched estop state on every driver that supports
        it (e.g. RoboClaw's S3 estop_pin). Motor commands stay where
        they are — operator must send a fresh value to start moving."""
        self.get_logger().info('Clear ESTOP requested')
        self._peripherals.clear_estop()

    def _handle_reboot(self):
        """Handle reboot request."""
        self.get_logger().warn('Reboot requested')

        # Clean shutdown
        self._gpio.cleanup()

        # Request system reboot (requires sudo privileges)
        import subprocess
        subprocess.run(['sudo', 'reboot'], check=False)

    def _handle_check_update(self, data: Dict[str, Any]):
        """Handle update check request."""
        new_version = data.get('version', '')

        is_available = self._updater.check_update_available(new_version)

        # Publish result
        result = {
            'node_id': self._node_id,
            'current_version': self._updater.current_version,
            'new_version': new_version,
            'update_available': is_available,
        }

        msg = String()
        msg.data = json.dumps(result)
        self._pub_update_progress.publish(msg)

        self.get_logger().info(f'Update check: {self._updater.current_version} -> {new_version}, available: {is_available}')

    def _handle_firmware_update(self, data: Dict[str, Any]):
        """Handle firmware update request."""
        url = data.get('url')
        checksum = data.get('checksum')
        version = data.get('version')

        if not url:
            self.get_logger().error('firmware_update missing url')
            self._publish_update_result(False, 'Missing update URL')
            return

        self.get_logger().info(f'Firmware update requested: {url}')

        # Set state to updating
        previous_state = self._state
        self._state = NodeState.UPDATING

        # Perform update in background (this may not return if successful)
        import threading
        def do_update():
            result = self._updater.perform_update(url, checksum, version)
            if not result['success']:
                # Update failed, restore state
                self._state = previous_state
                self._publish_update_result(False, result['message'])

        update_thread = threading.Thread(target=do_update, daemon=True)
        update_thread.start()

    def _on_update_progress(self, stage: str, percent: int):
        """Callback for update progress."""
        msg_data = {
            'node_id': self._node_id,
            'stage': stage,
            'percent': percent,
            'status': 'in_progress',
        }

        msg = String()
        msg.data = json.dumps(msg_data)
        self._pub_update_progress.publish(msg)

    def _publish_update_result(self, success: bool, message: str):
        """Publish update result."""
        msg_data = {
            'node_id': self._node_id,
            'status': 'complete' if success else 'failed',
            'success': success,
            'message': message,
        }

        msg = String()
        msg.data = json.dumps(msg_data)
        self._pub_update_progress.publish(msg)

    def cleanup(self):
        """Clean up resources on shutdown."""
        self.get_logger().info('Cleaning up...')
        self._peripherals.reset()
        self._gpio.cleanup()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = SaintNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
