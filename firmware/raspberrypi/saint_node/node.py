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
import os
import time
import socket
import hashlib
import threading
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
from .peripherals.kangaroo import KangarooDriver
from .peripherals.pathfinder_bms import PathfinderBMSDriver
from .peripherals.fas100 import FAS100Driver
from .peripherals.audio_player import PiAudioPlayerDriver
from .peripherals.audio_mixer import AlsaMixerDriver
from .peripherals.console_display import ConsoleDisplayDriver
from .pi_model import detect_pi_model
from . import ble_transport


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

        # Capture the firmware-build timestamp ONCE at startup so every
        # announce reports the same value. Previously the announce path
        # called time.gmtime() inline, which made fw_build the wall-clock
        # time of the announce — it ticked forward by one second per
        # broadcast and made the operator UI's "Built: …" line look like
        # an ever-fresh build. We use the mtime of saint_node/__init__.py
        # because OTA updates rewrite that file (and copytree preserves
        # the source mtime so install.sh and OTA agree on what "build
        # time" means for a given firmware drop).
        from . import __file__ as _pkg_init
        try:
            self._fw_build = time.strftime(
                '%Y-%m-%dT%H:%M:%SZ', time.gmtime(os.path.getmtime(_pkg_init)))
        except OSError:
            # Couldn't stat — fall back to process-start so it's at least
            # stable for the lifetime of this run.
            self._fw_build = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())

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
        self._peripherals.register(KangarooDriver)
        self._peripherals.register(PathfinderBMSDriver)
        self._peripherals.register(FAS100Driver)
        # Built-in audio playback: lives on every Pi-host saint-node,
        # auto-seeded by the raspberrypi board YAML's builtin_peripherals
        # entry. libVLC is loaded lazily — a missing system package
        # logs an error but doesn't crash the node.
        self._peripherals.register(PiAudioPlayerDriver)
        # Built-in audio output mixer: controls the host card's master
        # volume + L/R balance + mute (the whole output, not one player).
        # Auto-seeded by the board YAML; pyalsaaudio is loaded lazily.
        self._peripherals.register(AlsaMixerDriver)
        # Console kiosk: configures the Pi as an HDMI dashboard
        # appliance pointing at a Console view URL on the server.
        # Not a built-in — operator opts a given Pi in by adopting
        # the console_display peripheral on its Peripherals tab.
        self._peripherals.register(ConsoleDisplayDriver)

        # Firmware updater
        self._updater = FirmwareUpdater(self.get_logger())
        self._updater.set_progress_callback(self._on_update_progress)
        # Terminal result — published on success from inside
        # perform_update (just before the service restart kills us) and
        # on failure from do_update below.
        self._updater.set_result_callback(self._publish_update_result)

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
        # /log: BEST_EFFORT with depth=20 to match the server's LOG_QOS
        # (server_node.py LOG_QOS). RELIABLE here used to wedge the
        # micro-XRCE-DDS writer on RP2040/Teensy after a 4+ line burst;
        # the Pi doesn't have that constraint but matching keeps QoS-
        # incompatibility headaches off the table.
        self._qos_log = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=20
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
        # /log: forwards arbitrary {level, text, uptime_ms, peripheral?}
        # frames to the server. Without this publisher, only RP2040 /
        # Teensy micro-ROS nodes show up in the server-side activity
        # log; the Pi was silent because get_logger().info() only writes
        # to the local journal. server_node._on_node_log reads this
        # topic and forwards via state_manager.log_node_event to the
        # operator's activity feed.
        self._pub_log = self.create_publisher(
            String, f'/saint/nodes/{self._node_id}/log', self._qos_log)
        # Async scan results (BLE BMS discovery). Operator-triggered,
        # one frame per scan completion. Reliable QoS so a dropped
        # packet doesn't strand the UI's progress spinner.
        self._pub_ble_scan = self.create_publisher(
            String, f'/saint/nodes/{self._node_id}/ble_scan_results',
            self._qos_reliable)
        # Re-entry guard: one scan at a time. BlueZ doesn't like
        # parallel scans on the same adapter.
        self._ble_scan_thread: Optional[threading.Thread] = None

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
        """Apply saved pin + peripheral configuration on boot.

        Two layers — legacy GPIO pin configs (pre-peripheral-first
        addressing era), then peripherals. Peripheral replay is the
        important one: PeripheralManager state is purely in-memory,
        so without it every Pi reboot would silently come up with
        zero peripherals attached and the operator would have to
        re-sync from the UI to recover console_display, audio_player,
        Maestro, etc. The server only auto-re-pushes when we announce
        UNADOPTED (see _maybe_reconcile_adopted_unadopted), and we
        already claim ACTIVE on boot — so without local replay the
        peripherals never come back.
        """
        # Layer 1: legacy GPIO pin configs.
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

        # Layer 2: peripherals (peripheral-first addressing).
        peripherals = self._config.get_peripherals()
        if peripherals:
            try:
                self._peripherals.apply_peripherals(peripherals)
                # Use the journal (not _publish_log) here — this runs
                # in __init__ before DDS discovery completes, so the
                # /log publisher's subscriber may not be matched yet
                # and the message would drop (BEST_EFFORT QoS). Still
                # captured in journalctl -u saint-node either way.
                self.get_logger().info(
                    f'Replayed {len(peripherals)} saved peripheral(s) '
                    f'from {self._config._config_path}')
            except Exception as e:
                self.get_logger().error(
                    f'Failed to replay saved peripherals: {e}')

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

    def _publish_log(self, level: str, text: str, peripheral: str = '') -> None:
        """Publish a log frame to the server's per-node /log topic.

        Mirrors the firmware-side format expected by server_node._on_node_log:
        {"level": "info|warn|error", "text": "...", "uptime_ms": N,
         "peripheral": optional}. Also writes to the local journal via
        rclpy's logger so a Pi without ROS connectivity still has the
        line on disk for post-mortem (journalctl -u saint-node).

        Use this for operator-visible state transitions (firmware update
        started/finished, restart received, config applied) — NOT for
        every internal tick. /log topic uses BEST_EFFORT and a UDP
        path so a burst of 100 lines/sec will silently drop frames.
        """
        # Local journal first — guaranteed durable even if ROS is down.
        lvl_fn = {
            'error': self.get_logger().error,
            'warn':  self.get_logger().warn,
            'warning': self.get_logger().warn,
            'info':  self.get_logger().info,
            'debug': self.get_logger().debug,
        }.get(level, self.get_logger().info)
        lvl_fn(text)
        # Then the ROS-side broadcast.
        try:
            payload = {
                'level': level if level in ('info', 'warn', 'error') else 'info',
                'text': text,
                'uptime_ms': int((time.time() - self._start_time) * 1000),
            }
            if peripheral:
                payload['peripheral'] = peripheral
            msg = String()
            msg.data = json.dumps(payload)
            self._pub_log.publish(msg)
        except Exception:
            # Don't let a /log publish failure crash the caller —
            # the journal line above is the durable record.
            pass

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
            'fw_build': self._fw_build,
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
                    peripherals = data.get('peripherals', []) or []
                    self._peripherals.apply_peripherals(peripherals)
                    self._state = NodeState.ACTIVE
                    # Persist the peripherals to /etc/saint-node/config.yaml
                    # so we can replay them on next boot without waiting
                    # for the server to re-push. Includes any private
                    # params the state_manager injected (e.g.
                    # _kiosk_token for console_display) — replay is
                    # byte-for-byte what the server told us.
                    self._config.set_peripherals(peripherals)
                    save_ok = self._config.save()
                    # Two log lines, intentionally:
                    #   1. "Config applied" — operator-friendly summary
                    #      ("here's what we did").
                    #   2. "Config saved to disk" / "Config save failed"
                    #      — the sync-ACK marker the server's
                    #      _maybe_handle_sync_ack pattern-matches to
                    #      flip peripheral_sync_status. The RP2040 /
                    #      Teensy firmware emits "Config saved to flash"
                    #      after pin_config_save(); the Pi's equivalent
                    #      is the YAML save, mirrored with "to disk".
                    #      The "success" marker is gated on the save
                    #      actually succeeding — a config that lives
                    #      only in RAM would be lost on next reboot
                    #      and shouldn't read as synced.
                    self._publish_log('info',
                        f'Config applied: {len(peripherals)} peripheral(s) — '
                        f'now ACTIVE')
                    if save_ok:
                        self._publish_log('info', 'Config saved to disk')
                    else:
                        self._publish_log('error', 'Config save failed')
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

            elif action == 'reboot' or action == 'restart':
                # The server's "Restart Node" UI sends action="restart"
                # (server_node.send_restart_command); accept both names
                # so future renames in either direction don't silently
                # break this command.
                self._handle_reboot()

            elif action == 'firmware_update':
                self._handle_firmware_update(data)

            elif action == 'check_update':
                self._handle_check_update(data)

            elif action == 'ble_scan':
                self._handle_ble_scan(data)

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

    def _handle_ble_scan(self, data: Dict[str, Any]):
        """Operator-triggered BLE scan for JBD-protocol BMSes.

        Request: {action: "ble_scan", duration_s: 8, filter_jbd: true,
                  request_id: "..."}
        Reply (on /ble_scan_results topic):
          {status: "ok", devices: [{mac, name, rssi}], duration_s, ...}
          {status: "busy"|"error", message: "..."}

        Runs the actual scan in a worker thread so the main ROS loop
        keeps ticking. BlueZ rejects parallel scans on the same
        adapter, so we guard against re-entry.
        """
        duration_s = float(data.get('duration_s', 8.0))
        duration_s = max(1.0, min(30.0, duration_s))
        filter_jbd = bool(data.get('filter_jbd', True))
        request_id = str(data.get('request_id', ''))

        existing = self._ble_scan_thread
        if existing is not None and existing.is_alive():
            self._publish_ble_scan_result({
                "status": "busy",
                "message": "Another scan is already running",
                "request_id": request_id,
            })
            return

        def worker():
            started_at = time.time()
            try:
                devices = ble_transport.scan_jbd_devices(
                    duration_s=duration_s,
                    filter_service=filter_jbd,
                    logger=self.get_logger())
                payload = {
                    "status": "ok",
                    "devices": devices,
                    "duration_s": duration_s,
                    "filter_jbd": filter_jbd,
                    "started_at": started_at,
                    "finished_at": time.time(),
                    "request_id": request_id,
                }
            except Exception as e:
                self.get_logger().error(f"BLE scan failed: {e}")
                payload = {
                    "status": "error",
                    "message": str(e),
                    "request_id": request_id,
                }
            self._publish_ble_scan_result(payload)

        thread = threading.Thread(
            target=worker, name="saint-ble-scan", daemon=True)
        self._ble_scan_thread = thread
        thread.start()
        self.get_logger().info(
            f"BLE scan started ({duration_s:.1f}s, "
            f"filter_jbd={filter_jbd}, request_id={request_id!r})")

    def _publish_ble_scan_result(self, payload: Dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        self._pub_ble_scan.publish(msg)

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
        """Handle reboot request from server (action 'reboot' or
        'restart'). Logs an explicit "Reboot requested" line so an
        operator chasing "Pi didn't reboot when I clicked Restart"
        can confirm the message at least reached the handler:
            journalctl -u saint-node -n 100 | grep -i reboot
        If the line is absent, the control message never arrived
        (server-side dispatch / WS bridge problem). If present but
        no actual reboot, the systemctl/reboot binaries didn't fire.
        """
        self._publish_log('warn', 'Reboot requested — shutting down GPIO and rebooting')

        # Clean shutdown
        self._gpio.cleanup()

        # saint-node.service runs as User=root so neither `sudo` nor
        # special caps are needed. Try systemctl first (clean teardown
        # of other units); fall back to /sbin/reboot if that fails
        # (e.g. a minimal image where systemctl reboot needs a polkit
        # rule that's missing, or a logind socket that's not running).
        import subprocess
        try:
            r = subprocess.run(['systemctl', 'reboot'],
                               check=False, capture_output=True,
                               text=True, timeout=5)
            if r.returncode != 0:
                self.get_logger().error(
                    f'systemctl reboot failed (rc={r.returncode}): '
                    f'{(r.stderr or "").strip()} — falling back to /sbin/reboot')
                subprocess.run(['/sbin/reboot'], check=False)
        except (OSError, subprocess.SubprocessError) as e:
            self.get_logger().error(
                f'systemctl reboot raised: {e} — falling back to /sbin/reboot')
            subprocess.run(['/sbin/reboot'], check=False)

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
            self._publish_log('error', 'firmware_update missing url')
            self._publish_update_result(False, 'Missing update URL')
            return

        self._publish_log('info',
            f'Firmware update requested: v{version or "?"} from {url}')

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
        """Callback for update progress.

        Publishes two things per call:
          1. A frame on /update_progress for the UI's progress bar.
          2. ON STAGE TRANSITION ONLY, a /log line so the server-side
             activity feed gets a narrative ("Downloading…",
             "Verifying checksum…", "Installing…") without being
             flooded by per-percent ticks during the long download.
        """
        msg_data = {
            'node_id': self._node_id,
            'stage': stage,
            'percent': percent,
            'status': 'in_progress',
        }

        msg = String()
        msg.data = json.dumps(msg_data)
        self._pub_update_progress.publish(msg)

        # Emit a /log line only when the stage NAME changes — otherwise
        # a 1000-line "downloading 47%, 48%…" burst would saturate the
        # activity feed and (because /log is BEST_EFFORT) drop frames
        # the operator actually cares about (start/end transitions).
        if stage != getattr(self, '_last_update_stage', None):
            self._last_update_stage = stage
            STAGE_TEXT = {
                'downloading': 'Downloading firmware',
                'verifying':   'Verifying checksum',
                'extracting':  'Extracting package',
                'validating':  'Validating package contents',
                'backup':      'Backing up previous firmware',
                'installing':  'Installing new firmware',
                'restarting':  'Restarting saint-node service',
            }
            self._publish_log('info',
                f'Firmware update: {STAGE_TEXT.get(stage, stage)}')

    def _publish_update_result(self, success: bool, message: str):
        """Publish update result on both /update_progress and /log so
        the UI's progress bar AND the server's activity feed see it.
        Wired as the updater's result callback: success is published
        from inside perform_update just before the service restart (with
        a flush delay), and failure is published from do_update. Either
        way the UI gets a terminal frame instead of hanging on the last
        progress stage.
        """
        msg_data = {
            'node_id': self._node_id,
            'status': 'complete' if success else 'failed',
            'success': success,
            'message': message,
        }

        msg = String()
        msg.data = json.dumps(msg_data)
        self._pub_update_progress.publish(msg)

        self._publish_log(
            'info' if success else 'error',
            f'Firmware update {"complete" if success else "failed"}: {message}')

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
