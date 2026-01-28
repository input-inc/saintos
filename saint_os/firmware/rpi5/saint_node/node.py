#!/usr/bin/env python3
"""
SAINT.OS Raspberry Pi 5 Node

Main ROS2 node implementation for Raspberry Pi 5 GPIO control.
Communicates with SAINT.OS server using the same protocol as RP2040 nodes.
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
    SAINT.OS Raspberry Pi 5 Node.

    Implements the same communication protocol as RP2040 nodes:
    - Publishes announcements to /saint/nodes/announce
    - Publishes capabilities to /saint/nodes/<id>/capabilities
    - Publishes state to /saint/nodes/<id>/state
    - Subscribes to /saint/nodes/<id>/config for configuration
    - Subscribes to /saint/nodes/<id>/control for runtime control
    """

    # Hardware identification
    HW_TYPE = "Raspberry Pi 5"

    def __init__(self):
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
                        return f'rpi5_{hash_id}'
        except Exception:
            pass

        # Fallback: use MAC address
        try:
            mac = self._get_mac_address()
            if mac:
                hash_id = hashlib.md5(mac.encode()).hexdigest()[:12]
                return f'rpi5_{hash_id}'
        except Exception:
            pass

        # Last resort: use hostname
        hostname = socket.gethostname()
        hash_id = hashlib.md5(hostname.encode()).hexdigest()[:12]
        return f'rpi5_{hash_id}'

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
        """Publish node capabilities."""
        capabilities = self._gpio.get_capabilities()

        msg_data = {
            'node_id': self._node_id,
            'available_pins': capabilities['available_pins'],
            'reserved_pins': capabilities['reserved_pins'],
            'total_pins': len(capabilities['available_pins']),
            'features': capabilities.get('features', []),
        }

        msg = String()
        msg.data = json.dumps(msg_data)
        self._pub_capabilities.publish(msg)
        self.get_logger().info('Published capabilities')

    def _publish_state(self):
        """Publish current pin state."""
        pin_states = self._gpio.get_all_states()

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

                if gpio is not None and value is not None:
                    self._gpio.set_value(int(gpio), float(value))
                else:
                    self.get_logger().warn('set_pin missing gpio or value')

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

        # Clear configuration
        self._config.factory_reset()

        # Return to unadopted state
        self._state = NodeState.UNADOPTED

        self.get_logger().info('Factory reset complete')

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
