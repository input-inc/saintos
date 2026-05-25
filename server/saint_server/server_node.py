"""
SAINT.OS Main Server Node

The central ROS2 node that coordinates all server functionality:
- Node adoption and management
- Firmware distribution
- WebSocket gateway
- LiveLink receiver
- RC receiver
- Input/Output routing
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# QoS for streaming control commands (joystick → motor). Joystick-style
# data is fire-and-forget: a packet lost in transit must NOT stall the
# queue behind it while DDS retransmits, because that's how a deadstick
# ends up arriving a full second late behind nine stale non-zero values.
# BEST_EFFORT + KEEP_LAST(1) gives us "always the freshest command, drop
# anything older" — the standard pattern for /cmd_vel-class topics.
#
# Trade-off: one-shot commands sent on the same topic (factory_reset,
# firmware_update, identify, estop) become slightly less reliable too.
# In practice this hasn't been a problem — those are operator-driven
# clicks and a re-click costs nothing, while a stuck deadstick costs the
# robot crashing into a wall.
CONTROL_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE,
)

# Use std_msgs/String for node announcements (matches firmware)
from std_msgs.msg import String

import asyncio
import os
import signal
import threading
from typing import Optional, Dict, Any, List

from saint_server.webserver.state_manager import StateManager
from saint_server.peripheral_logger import PeripheralLogger
from saint_server.roles import RoleManager
from saint_server.livelink import LiveLinkReceiver, LiveLinkRouter
from saint_server.livelink.receiver import LiveLinkConfig
from saint_server.livelink.router import LiveLinkRoute, OutputMapping, MappingType
from saint_server.discovery import DiscoveryService


class SaintServerNode(Node):
    """Main SAINT.OS server node."""

    def __init__(self):
        super().__init__('saint_server')

        # Declare parameters
        self.declare_parameter('server_name', 'SAINT-01')
        self.declare_parameter('web_port', 80)
        self.declare_parameter('websocket_port', 9090)
        self.declare_parameter('livelink_enabled', True)
        self.declare_parameter('livelink_discovery_port', 54321)
        self.declare_parameter('livelink_data_port', 54322)
        self.declare_parameter('rc_enabled', False)
        self.declare_parameter('rc_protocol', 'sbus')
        self.declare_parameter('discovery_enabled', True)
        self.declare_parameter('discovery_port', 8889)
        self.declare_parameter('agent_port', 8888)

        # Get parameters
        self.server_name = self.get_parameter('server_name').value
        self.web_port = self.get_parameter('web_port').value
        self.websocket_port = self.get_parameter('websocket_port').value

        # Callback group for async operations
        self.callback_group = ReentrantCallbackGroup()

        # Async infrastructure
        self._async_loop: Optional[asyncio.AbstractEventLoop] = None
        self._async_thread: Optional[threading.Thread] = None
        self._shutdown_event: Optional[asyncio.Event] = None

        # State manager - shared with web server
        self.state_manager = StateManager(
            server_name=self.server_name,
            logger=self.get_logger()
        )

        # File-backed logging tree under $SAINT_LOG_DIR (default
        # /var/log/saint-os):
        #   saint-server.log                      — every activity event
        #   nodes/<node_id>.log                   — per-node activity events
        #   peripherals/<node_id>__<periph>.log   — per-peripheral telemetry (opt-in)
        #
        # SAINT_LOG_DIR is set by the systemd unit and pre-created +
        # chowned to the service user by install.sh, so production
        # paths are writable. We test the target directory itself
        # (after ensuring it exists) — earlier versions checked the
        # PARENT's writability, which is /var/log for the production
        # path and is root-owned, so the check always failed and the
        # logger silently fell through to /tmp/saint-os (which is
        # isolated by PrivateTmp=true and thus invisible from outside
        # the service).
        #
        # If the configured dir isn't writable we fall back to
        # /tmp/saint-os so the server still starts; the journal log
        # is the audit trail when the file sinks are unavailable.
        base_dir = os.environ.get("SAINT_LOG_DIR") or "/var/log/saint-os"
        try:
            os.makedirs(base_dir, exist_ok=True)
            if not os.access(base_dir, os.W_OK):
                raise PermissionError(f"{base_dir} not writable")
        except (OSError, PermissionError) as e:
            fallback = "/tmp/saint-os"
            self.get_logger().warn(
                f"saint-os log dir: {base_dir} unusable ({e}); "
                f"falling back to {fallback}"
            )
            os.makedirs(fallback, exist_ok=True)
            base_dir = fallback

        # Server-wide activity file — one line per _log_activity call,
        # rotating daily at midnight. Plain-text format so an operator
        # can tail it without a parser.
        import logging as _logging
        from saint_server.file_log import make_daily_handler, PerKeyLogger
        _activity_fmt = _logging.Formatter(
            "%(asctime)s %(message)s", datefmt="%Y-%m-%d %H:%M:%S"
        )
        self._server_activity_logger = _logging.getLogger("saint_os.activity")
        self._server_activity_logger.setLevel(_logging.DEBUG)
        self._server_activity_logger.propagate = False
        # Idempotent in case __init__ runs twice (tests, reloads).
        for _h in list(self._server_activity_logger.handlers):
            self._server_activity_logger.removeHandler(_h)
        self._server_activity_logger.addHandler(make_daily_handler(
            os.path.join(base_dir, "saint-server.log"),
            formatter=_activity_fmt,
        ))

        # Per-node activity files. PerKeyLogger lazily opens
        # nodes/<node_id>.log on the first event for that node, each
        # with its own daily-rotating handler (with rotate-on-restart).
        self._per_node_activity_logger = PerKeyLogger(
            base_dir=os.path.join(base_dir, "nodes"),
            logger_name_prefix="saint_os.nodes",
            formatter=_activity_fmt,
        )

        # Hook them into the state manager so every _log_activity call
        # also writes to the file sinks.
        self.state_manager.set_activity_file_logger(
            server_logger=self._server_activity_logger,
            per_node_logger=self._per_node_activity_logger,
        )

        # Per-peripheral telemetry logger — opt-in per peripheral,
        # writes NDJSON to ${SAINT_LOG_DIR}/peripherals/<node>__<id>.log
        # with daily rotation.
        peripherals_dir = os.path.join(base_dir, "peripherals")
        os.makedirs(peripherals_dir, exist_ok=True)
        self.peripheral_logger = PeripheralLogger(
            log_dir=peripherals_dir,
            logger=self.get_logger(),
        )
        # Hand it to the state manager so update_pin_actual can feed
        # samples and so the WS log_enabled toggle finds it.
        self.state_manager.set_peripheral_logger(self.peripheral_logger)

        # Web server components (set during start_async_services)
        self.web_server = None

        # Role manager
        self.role_manager = RoleManager(logger=self.get_logger())

        # LiveLink components
        self.livelink_receiver: Optional[LiveLinkReceiver] = None
        self.livelink_router: Optional[LiveLinkRouter] = None
        self._livelink_enabled = self.get_parameter('livelink_enabled').value

        # Discovery service (helps nodes find the server)
        self.discovery_service: Optional[DiscoveryService] = None
        self._discovery_enabled = self.get_parameter('discovery_enabled').value
        self._discovery_port = self.get_parameter('discovery_port').value
        self._agent_port = self.get_parameter('agent_port').value

        # ROS Bridge for WebSocket-ROS communication
        self.ros_bridge = None

        # Dynamic publishers/subscribers for node communication
        self._node_config_pubs: Dict[str, Any] = {}  # node_id -> publisher
        self._node_caps_subs: Dict[str, Any] = {}    # node_id -> subscription
        self._node_control_pubs: Dict[str, Any] = {} # node_id -> control publisher
        self._node_state_subs: Dict[str, Any] = {}   # node_id -> state subscription
        self._node_log_subs: Dict[str, Any] = {}     # node_id -> log subscription

        # Initialize components
        self._init_publishers()
        self._init_subscribers()
        self._init_services()
        self._init_timers()

        self.get_logger().info(f'SAINT.OS Server "{self.server_name}" initialized')

    def _init_publishers(self):
        """Initialize ROS2 publishers."""
        # System status publisher
        # self.status_pub = self.create_publisher(SystemStatus, '/saint/system/status', 10)
        pass

    def _init_subscribers(self):
        """Initialize ROS2 subscribers."""
        # Node announcements (using std_msgs/String with JSON payload)
        self.announcement_sub = self.create_subscription(
            String,
            '/saint/nodes/announce',
            self._on_node_announcement,
            10,
            callback_group=self.callback_group
        )
        self.get_logger().info('Subscribed to /saint/nodes/announce')

    def _init_services(self):
        """Initialize ROS2 services."""
        # Firmware services
        # self.firmware_check_srv = self.create_service(
        #     FirmwareCheck,
        #     '/saint/firmware/check',
        #     self._handle_firmware_check
        # )
        pass

    def _init_timers(self):
        """Initialize periodic timers."""
        # Status broadcast timer (also checks for node timeouts)
        self.status_timer = self.create_timer(
            1.0,  # 1 Hz
            self._periodic_update,
            callback_group=self.callback_group
        )

        # Cleanup timer for stale offline nodes (every 30 seconds)
        self.cleanup_timer = self.create_timer(
            30.0,
            self._cleanup_stale_nodes,
            callback_group=self.callback_group
        )

    def _on_node_announcement(self, msg: String):
        """Handle node announcement from micro-ROS nodes."""
        raw = msg.data
        try:
            # Log announcement for debugging
            self.get_logger().debug(f'Received announcement ({len(raw)} bytes): {raw[:100]}...')
            is_new = self.state_manager.update_node_from_announcement(raw)
            # Parse node_id once; used for both the log line below and
            # to register the per-node log subscriber lazily on first
            # contact (so firmware boot lines from a fresh node get
            # captured even before adoption).
            announced_node_id = None
            try:
                import json
                announced_node_id = json.loads(raw).get('node_id')
            except Exception:
                pass
            if is_new:
                self.get_logger().info(f'New node discovered via announcement')
            else:
                if announced_node_id:
                    self.get_logger().debug(f'Updated node {announced_node_id}')
            if announced_node_id:
                # Subscribe to everything the firmware publishes for
                # this node, so data flows whether or not the operator
                # has triggered a control / config push yet. State at
                # 10 Hz feeds the Live Readings tab + widget dashboard;
                # log lines feed the per-node Logs tab; capabilities
                # are kept for legacy nodes that still publish them.
                self._ensure_node_log_subscriber(announced_node_id)
                self._ensure_node_state_subscriber(announced_node_id)
                self._ensure_node_capabilities_subscriber(announced_node_id)
        except Exception as e:
            # Dump the full raw payload so operators can see exactly what
            # arrived — the bare error message ("unterminated string at
            # column 251") tells you the symptom but not the cause.
            preview_len = len(raw) if raw is not None else 0
            self.get_logger().error(
                f'Error processing announcement ({preview_len} bytes): {e}\n'
                f'  RAW: {raw!r}'
            )

    def _periodic_update(self):
        """Periodic update - check timeouts, publish status."""
        # Check for nodes that have timed out
        timed_out = self.state_manager.check_node_timeouts()
        if timed_out:
            self.get_logger().info(f'Nodes timed out: {timed_out}')

    def _cleanup_stale_nodes(self):
        """Remove unadopted nodes that have been offline too long."""
        removed = self.state_manager.remove_offline_unadopted_nodes(max_offline_seconds=60.0)
        if removed > 0:
            self.get_logger().debug(f'Removed {removed} stale unadopted nodes')

    # =========================================================================
    # Pin Configuration Communication
    # =========================================================================

    def _sanitize_topic_name(self, name: str) -> str:
        """Sanitize a string for use in ROS2 topic names."""
        # Replace invalid characters with underscores
        return ''.join(c if c.isalnum() or c == '_' else '_' for c in name)

    def _ensure_node_config_publisher(self, node_id: str):
        """Ensure a config publisher exists for a node."""
        if node_id in self._node_config_pubs:
            return self._node_config_pubs[node_id]

        sanitized_id = self._sanitize_topic_name(node_id)
        topic = f'/saint/nodes/{sanitized_id}/config'

        pub = self.create_publisher(
            String,
            topic,
            10,
            callback_group=self.callback_group
        )
        self._node_config_pubs[node_id] = pub
        self.get_logger().info(f'Created config publisher: {topic}')
        return pub

    def _ensure_node_capabilities_subscriber(self, node_id: str):
        """Ensure a capabilities subscriber exists for a node."""
        if node_id in self._node_caps_subs:
            return

        sanitized_id = self._sanitize_topic_name(node_id)
        topic = f'/saint/nodes/{sanitized_id}/capabilities'

        def callback(msg: String):
            self._on_node_capabilities(node_id, msg)

        sub = self.create_subscription(
            String,
            topic,
            callback,
            10,
            callback_group=self.callback_group
        )
        self._node_caps_subs[node_id] = sub
        self.get_logger().info(f'Subscribed to capabilities: {topic}')

    def _on_node_capabilities(self, node_id: str, msg: String):
        """Handle capabilities message from a node."""
        try:
            self.get_logger().info(f'Received capabilities from {node_id}: {msg.data[:200]}...' if len(msg.data) > 200 else f'Received capabilities from {node_id}: {msg.data}')
            success = self.state_manager.update_node_capabilities(node_id, msg.data)
            if success:
                self.get_logger().info(f'Capabilities stored successfully for {node_id}')
                # Mark node as synced (config confirmed by node publishing capabilities)
                self.state_manager.mark_node_synced(node_id, success=True)
                # Broadcast update to WebSocket clients
                self._broadcast_node_capabilities(node_id)
                self._broadcast_sync_status(node_id)
            else:
                self.get_logger().warn(f'Failed to store capabilities for {node_id}')
        except Exception as e:
            self.get_logger().error(f'Error processing capabilities from {node_id}: {e}')

    def _broadcast_node_capabilities(self, node_id: str):
        """Broadcast capabilities update to WebSocket clients."""
        if self.web_server and self.web_server.ws_handler:
            if self._async_loop:
                caps = self.state_manager.get_node_capabilities(node_id)
                if caps:
                    # Include peripheral sync_status in the capabilities payload
                    peripherals = self.state_manager.get_node_peripherals(node_id)
                    if peripherals:
                        caps["sync_status"] = peripherals.get("sync_status", "unknown")
                        caps["last_synced"] = peripherals.get("last_synced")
                    caps_copy = dict(caps)
                    topic = f'node_capabilities/{node_id}'
                    self._async_loop.call_soon_threadsafe(
                        lambda c=caps_copy, t=topic: asyncio.create_task(
                            self.web_server.ws_handler.broadcast_state(t, c)
                        )
                    )

    def _broadcast_sync_status(self, node_id: str):
        """Broadcast peripheral sync status to WebSocket clients."""
        if self.web_server and self.web_server.ws_handler:
            if self._async_loop:
                peripherals = self.state_manager.get_node_peripherals(node_id)
                if peripherals:
                    status_data = {
                        "node_id": node_id,
                        "sync_status": peripherals.get("sync_status", "unknown"),
                        "last_synced": peripherals.get("last_synced"),
                    }
                    topic = f'sync_status/{node_id}'
                    self._async_loop.call_soon_threadsafe(
                        lambda d=status_data, t=topic: asyncio.create_task(
                            self.web_server.ws_handler.broadcast_state(t, d)
                        )
                    )

    def send_config_to_node(self, node_id: str, config_json: str):
        """Send pin configuration to a node via ROS2."""
        pub = self._ensure_node_config_publisher(node_id)
        self._ensure_node_capabilities_subscriber(node_id)

        msg = String()
        msg.data = config_json

        pub.publish(msg)
        self.get_logger().info(f'Sent config to node {node_id}')
        self.state_manager.log_node_event(
            node_id, "Pushed peripheral config (Sync to Node)", "info")

        # Mark as syncing (not yet confirmed)
        # The node will publish capabilities after applying config

    def request_node_capabilities(self, node_id: str):
        """Request capabilities from a node."""
        pub = self._ensure_node_config_publisher(node_id)
        self._ensure_node_capabilities_subscriber(node_id)

        # Send capabilities request
        msg = String()
        msg.data = '{"action":"request_capabilities"}'

        pub.publish(msg)
        self.get_logger().info(f'Requested capabilities from node {node_id}')
        self.state_manager.log_node_event(
            node_id, "Requested capabilities refresh", "info")

    # =========================================================================
    # Pin Control Communication
    # =========================================================================

    def _ensure_node_control_publisher(self, node_id: str):
        """Ensure a control publisher exists for a node."""
        if node_id in self._node_control_pubs:
            return self._node_control_pubs[node_id]

        sanitized_id = self._sanitize_topic_name(node_id)
        topic = f'/saint/nodes/{sanitized_id}/control'

        pub = self.create_publisher(
            String,
            topic,
            CONTROL_QOS,
            callback_group=self.callback_group
        )
        self._node_control_pubs[node_id] = pub
        self.get_logger().info(f'Created control publisher: {topic} (best-effort, depth=1)')
        return pub

    def _ensure_node_state_subscriber(self, node_id: str):
        """Ensure a state subscriber exists for a node."""
        if node_id in self._node_state_subs:
            return

        sanitized_id = self._sanitize_topic_name(node_id)
        topic = f'/saint/nodes/{sanitized_id}/state'

        def callback(msg: String):
            self._on_node_state(node_id, msg)

        sub = self.create_subscription(
            String,
            topic,
            callback,
            10,
            callback_group=self.callback_group
        )
        self._node_state_subs[node_id] = sub
        self.get_logger().info(f'Subscribed to state: {topic}')

    def _ensure_node_log_subscriber(self, node_id: str):
        """Ensure a log subscriber exists for a node.

        The firmware publishes JSON-encoded lines to
        /saint/nodes/<id>/log; we route each into the per-node
        log buffer so they appear in the node-detail Logs tab.
        """
        if node_id in self._node_log_subs:
            return

        sanitized_id = self._sanitize_topic_name(node_id)
        topic = f'/saint/nodes/{sanitized_id}/log'

        def callback(msg: String):
            self._on_node_log(node_id, msg)

        # depth=20 since log bursts during config-apply can be a handful
        # in quick succession; we'd rather buffer than drop.
        sub = self.create_subscription(
            String,
            topic,
            callback,
            20,
            callback_group=self.callback_group
        )
        self._node_log_subs[node_id] = sub
        self.get_logger().info(f'Subscribed to log: {topic}')

    def _on_node_log(self, node_id: str, msg: String):
        """Receive a firmware log line and feed it into the per-node buffer.

        Firmware emits {"level": "info|warn|error", "text": "…", "uptime_ms": N}.
        Anything malformed gets stored as a raw-payload "warn" so we never lose
        a hint that the firmware tried to say something.

        We also use specific log texts as sync acks. Since the firmware
        no longer publishes a capabilities message after applying
        config (stripped in 1b42bd6), the pre-existing sync_status
        flow had no signal to flip from "pending" to "synced". The
        "Config saved to flash" log line is now that signal — it fires
        from main.c's config_subscription_callback after pin_config_save()
        returns true.
        """
        try:
            import json
            data = json.loads(msg.data)
            level = str(data.get('level', 'info'))
            text = str(data.get('text', ''))
            up = data.get('uptime_ms')
            if up is not None:
                text = f'[+{int(up) / 1000.0:.3f}s] {text}'
            if not text:
                return
            self.state_manager.log_node_event(node_id, text, level)
            self._maybe_handle_sync_ack(node_id, text)
        except Exception as e:
            self.state_manager.log_node_event(
                node_id, f'(malformed log frame: {e}) {msg.data[:120]!r}', 'warn')

    def _maybe_handle_sync_ack(self, node_id: str, text: str) -> None:
        """If the firmware reported it finished applying + saving config,
        flip the node's sync_status to ``synced`` and broadcast the
        update so the UI's "Sync to Node" pill updates."""
        if "Config saved to flash" in text:
            self.state_manager.mark_node_synced(node_id, success=True)
            self._broadcast_sync_status(node_id)
        elif "Config apply failed" in text or "Flash save failed" in text:
            self.state_manager.mark_node_synced(node_id, success=False)
            self._broadcast_sync_status(node_id)

    def _on_node_state(self, node_id: str, msg: String):
        """Handle state message from a node."""
        try:
            import json
            data = json.loads(msg.data)
            pins_data = data.get('pins', [])

            # Update state manager
            self.state_manager.update_pin_actual(node_id, pins_data)

            # Broadcast to WebSocket clients
            self._broadcast_pin_state(node_id)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid state JSON from {node_id}: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing state from {node_id}: {e}')

    def _broadcast_pin_state(self, node_id: str):
        """Broadcast pin state update to WebSocket clients."""
        if self.web_server and self.web_server.ws_handler:
            if self._async_loop:
                state = self.state_manager.get_runtime_state(node_id)
                if state:
                    topic = f'pin_state/{node_id}'
                    state_copy = state
                    self._async_loop.call_soon_threadsafe(
                        lambda s=state_copy, t=topic: asyncio.create_task(
                            self.web_server.ws_handler.broadcast_state(t, s)
                        )
                    )

    def send_control_command(self, node_id: str, gpio: int, value: float):
        """Send pin control command to a node via ROS2."""
        import json

        pub = self._ensure_node_control_publisher(node_id)
        self._ensure_node_state_subscriber(node_id)

        control_data = {
            "action": "set_pin",
            "gpio": gpio,
            "value": value
        }

        msg = String()
        msg.data = json.dumps(control_data)

        pub.publish(msg)
        self.get_logger().debug(f'Sent control to {node_id}: GPIO {gpio} = {value}')

    def send_channel_command(self, node_id: str, peripheral_id: str,
                             channel_id: str, value: float,
                             peripheral_type: str = ""):
        """Send channel-addressed control to a node via ROS2.

        Operator-visible names go on the wire — the firmware does its
        own (peripheral_id, channel_id) → driver dispatch. The server
        does no GPIO translation; that indirection is being deleted.

        `peripheral_type` is the catalog type (e.g. "neopixel",
        "roboclaw") — empty string if unknown. Firmware uses it to
        route writes for peripherals that don't live in pin_config (the
        built-in status NeoPixel is the canonical case — its instance
        id is "onboard_neopixel" but the firmware only knows it as
        type="neopixel").
        """
        import json

        pub = self._ensure_node_control_publisher(node_id)
        self._ensure_node_state_subscriber(node_id)

        control_data = {
            "action": "set_channel",
            "peripheral": peripheral_id,
            "channel": channel_id,
            "value": value,
        }
        if peripheral_type:
            control_data["type"] = peripheral_type

        msg = String()
        msg.data = json.dumps(control_data)

        pub.publish(msg)
        self.get_logger().debug(
            f'Sent channel control to {node_id}: '
            f'{peripheral_type}/{peripheral_id}/{channel_id} = {value}')

    def send_factory_reset_command(self, node_id: str):
        """Send factory reset command to a node via ROS2.

        This tells the node to:
        1. Clear its saved pin configuration
        2. Transition back to UNADOPTED state
        """
        import json

        pub = self._ensure_node_control_publisher(node_id)

        control_data = {
            "action": "factory_reset"
        }

        msg = String()
        msg.data = json.dumps(control_data)

        pub.publish(msg)
        self.get_logger().info(f'Sent factory reset command to {node_id}')
        self.state_manager.log_node_event(
            node_id, "Sent factory-reset command", "warn")

    def send_restart_command(self, node_id: str):
        """Send restart command to a node via ROS2.

        This tells the node to reboot itself.
        """
        import json

        pub = self._ensure_node_control_publisher(node_id)

        control_data = {
            "action": "restart"
        }

        msg = String()
        msg.data = json.dumps(control_data)

        pub.publish(msg)
        self.get_logger().info(f'Sent restart command to {node_id}')
        self.state_manager.log_node_event(node_id, "Sent restart command", "info")

    def send_identify_command(self, node_id: str):
        """Send identify command to a node via ROS2.

        This tells the node to blink its LED to help identify it physically.
        """
        import json

        pub = self._ensure_node_control_publisher(node_id)

        control_data = {
            "action": "identify"
        }

        msg = String()
        msg.data = json.dumps(control_data)

        pub.publish(msg)
        self.get_logger().info(f'Sent identify command to {node_id}')
        self.state_manager.log_node_event(node_id, "Sent identify (blink LED) command", "info")

    def send_estop_command(self, node_id: str, action: str = "engage"):
        """Send emergency stop command to a node via ROS2.

        Args:
            node_id: target node id
            action: "engage" to assert e-stop (motors → 0, peripheral
                e-stop hooks fire, e.g. RoboClaw drives its estop_pin
                HIGH), or "release" to clear the latch and return
                peripherals to their normal control path.
        """
        import json

        pub = self._ensure_node_control_publisher(node_id)

        # JSON wire format kept backward-compatible: the legacy
        # firmware path matches on "action":"estop" for the engage
        # case. New firmware also matches "clear_estop" for release.
        # We emit BOTH the legacy action and an explicit state field
        # so old controllers keep stopping while new ones can also
        # release.
        if action == "release":
            wire_action = "clear_estop"
        else:
            wire_action = "estop"
        control_data = {
            "action": wire_action,
            "state": "engaged" if action == "engage" else "released",
        }

        msg = String()
        msg.data = json.dumps(control_data)

        pub.publish(msg)
        verb = "engage" if action == "engage" else "release"
        self.get_logger().info(f'Sent emergency stop {verb} to {node_id}')
        self.state_manager.log_node_event(
            node_id, f"Sent emergency stop {verb}",
            "warn" if action == "engage" else "info")

    def send_roboclaw_debug_command(self, node_id: str, payload: dict):
        """Forward a RoboClaw wire-level debug payload to the node.

        `payload` is the {op, op-specific keys} dict; we wrap it in
        the control envelope ({"action":"roboclaw_debug", ...}) and
        publish. The firmware replies asynchronously via the normal
        log stream with a 'roboclaw_dbg:' prefix.
        """
        import json

        pub = self._ensure_node_control_publisher(node_id)
        control_data = {"action": "roboclaw_debug", **payload}
        msg = String()
        msg.data = json.dumps(control_data)
        pub.publish(msg)
        self.get_logger().info(
            f'Sent roboclaw_debug {payload.get("op")} to {node_id}')
        self.state_manager.log_node_event(
            node_id, f"roboclaw_debug {payload.get('op')} dispatched", "info")

    def send_firmware_update_command(self, node_id: str, simulation: bool = False, force: bool = False):
        """Send firmware update command to a node via ROS2.

        This tells the node to:
        1. For RP2040 simulation: Restart and load new firmware ELF
        2. For RP2040 hardware: Enter bootloader mode for UF2 update
        3. For Pi 5: Download and install update package from server

        The actual update mechanism depends on the node type.

        Args:
            node_id: Target node ID
            simulation: Whether this is a simulation build
            force: If True, skip version comparison on node side
        """
        import json

        # Determine node type from state manager
        node_info = self.state_manager.get_node(node_id)
        hw_type = node_info.get('hw', '') if node_info else ''

        pub = self._ensure_node_control_publisher(node_id)

        if 'Raspberry Pi' in hw_type or 'rpi5' in hw_type.lower():
            # Pi 5 node - send download URL
            self._send_rpi5_firmware_update(pub, node_id, force)
        elif 'Teensy' in hw_type or 'teensy' in node_id.lower():
            # Teensy 4.1 — same OTA wire format as RP2040 (size + crc32),
            # different resource path so the server serves the right .bin.
            self._send_teensy41_firmware_update(pub, node_id, force)
        else:
            # RP2040 node
            self._send_rp2040_firmware_update(pub, node_id, simulation, force)

    def _send_rpi5_firmware_update(self, pub, node_id: str, force: bool = False):
        """Send firmware update command to a Pi 5 node."""
        import json

        # Get Pi 5 firmware info
        fw_info = self.state_manager.get_firmware_info_for_type('rpi5')

        if not fw_info or not fw_info.get('available'):
            self.get_logger().error(f'No Pi 5 firmware available for update')
            return

        # Construct download URL using server's address
        server_port = getattr(self.web_server, 'port', 80) if hasattr(self, 'web_server') else 80
        # Use the node's last known IP to determine which interface the server should use
        # For now, use a relative path that assumes the node can reach the server
        download_url = f'http://{{server_ip}}:{server_port}/api/firmware/rpi5/{fw_info["filename"]}'

        control_data = {
            "action": "firmware_update",
            "version": fw_info.get("version", "0.0.0"),
            "url": download_url,
            "checksum": fw_info.get("checksum"),
            "build_date": fw_info.get("build_date"),
            "force": force,
        }

        msg = String()
        msg.data = json.dumps(control_data)

        pub.publish(msg)
        self.get_logger().info(f'Sent Pi 5 firmware update command to {node_id} (version: {fw_info.get("version")}, force: {force})')

    def _send_teensy41_firmware_update(self, pub, node_id: str, force: bool = False):
        """Send firmware update command to a Teensy 4.1 node.

        Teensy reads `size` + `crc32` from this message and downloads
        /api/firmware/teensy41/saint_node.bin over HTTP, then uses
        FlashTxx to swap it in. Without those fields the node falls
        back to a no-op SCB_AIRCR restart.
        """
        import json

        fw_info = self.state_manager.get_firmware_info_for_type('teensy41')
        if not fw_info or not fw_info.get('available'):
            self.get_logger().error('No Teensy 4.1 firmware available for update')
            return

        control_data = {
            "action": "firmware_update",
            "version": fw_info.get("version") or "0.0.0",
            "build_date": fw_info.get("build_date"),
            "force": force,
        }

        if fw_info.get("bin_size") and fw_info.get("bin_crc32") is not None:
            control_data["size"]  = fw_info["bin_size"]
            control_data["crc32"] = f"0x{fw_info['bin_crc32']:08x}"
            control_data["url"]   = "/api/firmware/teensy41/saint_node.bin"

        msg = String()
        msg.data = json.dumps(control_data)
        pub.publish(msg)
        self.get_logger().info(
            f'Sent Teensy 4.1 firmware update command to {node_id} '
            f'(size: {fw_info.get("bin_size")}, crc32: {control_data.get("crc32")}, '
            f'force: {force})'
        )

    def _send_rp2040_firmware_update(self, pub, node_id: str, simulation: bool, force: bool = False):
        """Send firmware update command to an RP2040 node.

        With the OTA bootloader present, the node uses `size` + `crc32`
        from this message to fetch /api/firmware/rp2040/saint_node.bin
        from the server (DHCP-gateway) and verify the download. Without
        a bootloader the node falls back to a no-op reboot.
        """
        import json

        # Get firmware info from state manager (includes bin size + crc32)
        fw_info = self.state_manager.get_server_firmware_info()

        control_data = {
            "action": "firmware_update",
            "version": fw_info.get("version_full") or fw_info.get("version", "0.0.0"),
            "elf_path": fw_info.get("elf_path"),
            "build_date": fw_info.get("build_date"),
            "force": force,
        }

        # OTA metadata — present only if the build pipeline produced the
        # raw .bin (i.e. the SAINT_OS_OTA_BOOTLOADER build path). When
        # missing, the node sees no size/crc32 and falls back to legacy
        # reboot behavior.
        if fw_info.get("bin_size") and fw_info.get("bin_crc32") is not None:
            control_data["size"] = fw_info["bin_size"]
            control_data["crc32"] = f"0x{fw_info['bin_crc32']:08x}"
            control_data["url"]  = f"/api/firmware/rp2040/saint_node.bin"

        msg = String()
        msg.data = json.dumps(control_data)

        pub.publish(msg)
        self.get_logger().info(
            f'Sent RP2040 firmware update command to {node_id} '
            f'(version: {fw_info.get("version_full") or fw_info.get("version")}, '
            f'size: {fw_info.get("bin_size")}, crc32: {control_data.get("crc32")}, '
            f'force: {force})'
        )
        self.state_manager.log_node_event(
            node_id,
            f"Firmware update triggered → {fw_info.get('version_full') or fw_info.get('version')}"
            + (" (force)" if force else ""),
            "info"
        )

        # For simulation nodes, also trigger the node manager to install and restart
        if simulation:
            self._update_simulation_node(node_id)

    def _update_simulation_node(self, node_id: str):
        """Update a simulation node by calling the node manager.

        This:
        1. Copies firmware from build_sim to install directory
        2. Restarts the Renode simulation
        """
        import subprocess
        import os

        # Path to node manager script
        node_manager_path = os.path.join(
            os.path.dirname(__file__),
            '..', 'firmware', 'rp2040', 'simulation', 'saint_node_manager.py'
        )

        if not os.path.exists(node_manager_path):
            # Try alternate path (when running from install)
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_dir = get_package_share_directory('saint_os')
                node_manager_path = os.path.join(
                    pkg_dir, '..', '..', '..', '..', 'src', 'saint_os',
                    'firmware', 'rp2040', 'simulation', 'saint_node_manager.py'
                )
            except Exception:
                pass

        if not os.path.exists(node_manager_path):
            self.get_logger().error(f'Node manager not found at {node_manager_path}')
            return

        try:
            # Run update command (installs firmware and restarts node)
            result = subprocess.run(
                ['python3', node_manager_path, 'update', node_id],
                capture_output=True,
                text=True,
                timeout=30
            )

            if result.returncode == 0:
                self.get_logger().info(f'Simulation node {node_id} updated and restarted')
                if result.stdout:
                    self.get_logger().info(f'Node manager output: {result.stdout}')
            else:
                self.get_logger().error(f'Failed to update simulation node: {result.stderr}')

        except subprocess.TimeoutExpired:
            self.get_logger().error(f'Timeout updating simulation node {node_id}')
        except Exception as e:
            self.get_logger().error(f'Error updating simulation node {node_id}: {e}')

    # =========================================================================
    # LiveLink Integration
    # =========================================================================

    async def _start_livelink(self):
        """Start the LiveLink receiver and router."""
        try:
            # Create router with output callback
            self.livelink_router = LiveLinkRouter(logger=self.get_logger())
            self.livelink_router.set_output_callback(self._livelink_output_callback)

            # Create receiver
            config = LiveLinkConfig(
                port=11111,  # Default Live Link Face port
                bind_address="0.0.0.0",
                timeout=5.0
            )
            self.livelink_receiver = LiveLinkReceiver(config=config, logger=self.get_logger())

            # Connect receiver to router
            self.livelink_receiver.add_callback(self.livelink_router.process)

            # Also broadcast blend shapes to WebSocket clients for visualization
            self.livelink_receiver.add_callback(self._broadcast_blend_shapes)

            # Add connection event handlers for activity logging
            self.livelink_receiver.on_connect(self._on_livelink_connect)
            self.livelink_receiver.on_disconnect(self._on_livelink_disconnect)

            # Start receiver
            await self.livelink_receiver.start(loop=self._async_loop)

            self.get_logger().info(f'LiveLink receiver started on port {config.port}')

            # Set up WebSocket handlers for LiveLink
            if self.web_server and self.web_server.ws_handler:
                self.web_server.ws_handler.set_livelink_callbacks(
                    get_status=self.get_livelink_status,
                    get_routes=self.get_livelink_routes,
                    set_routes=self.set_livelink_routes,
                    enable_route=self.enable_livelink_route,
                    create_default_route=self.create_livelink_default_route,
                )

        except Exception as e:
            self.get_logger().error(f'Failed to start LiveLink: {e}')

    async def _stop_livelink(self):
        """Stop the LiveLink receiver."""
        if self.livelink_receiver:
            await self.livelink_receiver.stop()
            self.get_logger().info('LiveLink receiver stopped')

    def _on_livelink_connect(self, source_addr: str, subject_name: str):
        """Handle LiveLink connection event."""
        self._broadcast_activity(
            f'LiveLink connected: {subject_name} from {source_addr}',
            'info'
        )

    def _on_livelink_disconnect(self, source_addr: str):
        """Handle LiveLink disconnection event."""
        self._broadcast_activity(
            f'LiveLink disconnected: {source_addr}',
            'warn'
        )

    _last_blend_broadcast = 0.0  # Throttle blend shape broadcasts

    def _broadcast_blend_shapes(self, blend_shapes):
        """Broadcast blend shape data to WebSocket clients for visualization."""
        import time
        import math

        # Throttle to ~10 Hz
        now = time.time()
        if now - self._last_blend_broadcast < 0.1:
            return
        self._last_blend_broadcast = now

        if self.web_server and self.web_server.ws_handler and self._async_loop:
            # Helper to sanitize float values (NaN/Inf are not valid JSON)
            def sanitize(value):
                if math.isnan(value) or math.isinf(value):
                    return 0.0
                return value

            # Convert to dict for serialization
            data = blend_shapes.to_dict()

            # Sanitize all float values
            for key, value in data.items():
                if isinstance(value, float):
                    data[key] = sanitize(value)

            data['subject_name'] = blend_shapes.subject_name
            data['timestamp'] = blend_shapes.timestamp

            # Add computed properties (sanitized)
            data['eye_look_horizontal_left'] = sanitize(blend_shapes.eye_look_horizontal_left)
            data['eye_look_horizontal_right'] = sanitize(blend_shapes.eye_look_horizontal_right)
            data['eye_look_vertical_left'] = sanitize(blend_shapes.eye_look_vertical_left)
            data['eye_look_vertical_right'] = sanitize(blend_shapes.eye_look_vertical_right)
            data['brow_left'] = sanitize(blend_shapes.brow_left)
            data['brow_right'] = sanitize(blend_shapes.brow_right)

            self._async_loop.call_soon_threadsafe(
                lambda d=data: asyncio.create_task(
                    self.web_server.ws_handler.broadcast_state('livelink/blend_shapes', d)
                )
            )

    def _livelink_output_callback(self, node_id: str, output_name: str, value: float):
        """Callback from LiveLink router to send an output to a node.

        TODO: rewire through the system routing graph. The old code did
        a GPIO lookup via `pin_config.logical_name`; with peripherals owning
        their pins, the right replacement is to find a Route whose source
        is `livelink:<output_name>` (signal kind) and whose sink is a
        peripheral channel, then dispatch accordingly. Disabled until the
        routing engine ships.
        """
        self.get_logger().debug(
            f'LiveLink output {output_name} for {node_id} ignored — routing engine pending'
        )

    def get_livelink_status(self) -> Dict[str, Any]:
        """Get LiveLink status for WebSocket API."""
        result = {
            "enabled": self._livelink_enabled,
            "receiver": None,
            "router": None
        }

        if self.livelink_receiver:
            result["receiver"] = self.livelink_receiver.stats

        if self.livelink_router:
            result["router"] = self.livelink_router.get_status()

        return result

    def get_livelink_routes(self) -> List[Dict[str, Any]]:
        """Get all LiveLink routes."""
        if self.livelink_router:
            return self.livelink_router.get_routes_config()
        return []

    def set_livelink_routes(self, routes_data: List[Dict[str, Any]]):
        """Set LiveLink routes from config data."""
        if self.livelink_router:
            self.livelink_router.load_routes_config(routes_data)
            self.get_logger().info(f'Loaded {len(routes_data)} LiveLink routes')

    def enable_livelink_route(self, route_name: str, enabled: bool):
        """Enable or disable a LiveLink route."""
        if self.livelink_router:
            self.livelink_router.enable_route(route_name, enabled)
            self.get_logger().info(f'LiveLink route "{route_name}" {"enabled" if enabled else "disabled"}')

    def create_livelink_default_route(self, node_id: str) -> Dict[str, Any]:
        """Create and add a default head route for a node."""
        if self.livelink_router:
            route = self.livelink_router.create_default_head_route(node_id)
            self.livelink_router.add_route(route)
            return route.to_dict()
        return {}

    def _run_async_loop(self):
        """Run the asyncio event loop in a background thread."""
        self._async_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._async_loop)
        self._shutdown_event = asyncio.Event()

        try:
            self._async_loop.run_until_complete(self._async_main())
        except Exception as e:
            self.get_logger().error(f'Async loop error: {e}')
        finally:
            self._async_loop.close()

    async def _async_main(self):
        """Main async coroutine managing all async services."""
        await self.start_async_services()

        # Wait for shutdown signal
        await self._shutdown_event.wait()

        await self.stop_async_services()

    async def start_async_services(self):
        """Start async services (WebSocket, HTTP server, etc.)."""
        self.get_logger().info('Starting async services...')

        try:
            # Import here to avoid circular imports
            from saint_server.webserver import WebServer

            # Get web root path from package share directory
            try:
                from ament_index_python.packages import get_package_share_directory
                web_root = os.path.join(get_package_share_directory('saint_os'), 'web')
            except Exception:
                # Fallback for development
                web_root = os.path.join(os.path.dirname(__file__), '..', 'web')

            # Create and start web server with shared state manager
            self.web_server = WebServer(
                web_root=web_root,
                port=self.web_port,
                server_name=self.server_name,
                logger=self.get_logger(),
                state_manager=self.state_manager
            )

            # Set up activity callback to broadcast to WebSocket clients
            self.state_manager.set_activity_callback(
                lambda msg, level: self._broadcast_activity(msg, level)
            )

            # Per-node log callback — feeds the Logs tab on node-detail.
            self.state_manager.set_node_log_callback(
                lambda node_id, entry: self._broadcast_node_log(node_id, entry)
            )

            # Set up pin config sync callback
            if self.web_server.ws_handler:
                self.web_server.ws_handler.set_sync_config_callback(
                    lambda node_id, config_json: self.send_config_to_node(node_id, config_json)
                )
                self.web_server.ws_handler.set_request_capabilities_callback(
                    lambda node_id: self.request_node_capabilities(node_id)
                )
                self.web_server.ws_handler.set_send_control_callback(
                    lambda node_id, gpio, value: self.send_control_command(node_id, gpio, value)
                )
                self.web_server.ws_handler.set_send_channel_callback(
                    lambda node_id, peripheral_id, channel_id, value, peripheral_type:
                        self.send_channel_command(node_id, peripheral_id,
                                                  channel_id, value,
                                                  peripheral_type)
                )
                self.web_server.ws_handler.set_firmware_update_callback(
                    lambda node_id, simulation, force: self.send_firmware_update_command(node_id, simulation, force)
                )
                self.web_server.ws_handler.set_factory_reset_callback(
                    lambda node_id: self.send_factory_reset_command(node_id)
                )
                self.web_server.ws_handler.set_restart_node_callback(
                    lambda node_id: self.send_restart_command(node_id)
                )
                self.web_server.ws_handler.set_identify_node_callback(
                    lambda node_id: self.send_identify_command(node_id)
                )
                self.web_server.ws_handler.set_estop_node_callback(
                    lambda node_id, action: self.send_estop_command(node_id, action)
                )
                self.web_server.ws_handler.set_roboclaw_debug_callback(
                    lambda node_id, payload: self.send_roboclaw_debug_command(node_id, payload)
                )

                # Initialize ROS Bridge for WebSocket-ROS communication
                try:
                    from saint_server.ros_bridge import ROSBridge
                    self.ros_bridge = ROSBridge(self, logger=self.get_logger())
                    self.ros_bridge.initialize()
                    self.web_server.ws_handler.set_ros_bridge(self.ros_bridge)
                    # Override the callback to use thread-safe async broadcast
                    self.ros_bridge.set_message_callback(self._broadcast_ros_message)
                    self.get_logger().info('ROS Bridge initialized and connected to WebSocket handler')

                    # Routing graph evaluator — drives peripheral channels
                    # from ROS topic inputs through user-defined sheets.
                    try:
                        from saint_server.router.routing_evaluator import RoutingEvaluator
                        self.routing_evaluator = RoutingEvaluator(
                            ros_bridge=self.ros_bridge,
                            send_channel=self.send_channel_command,
                            peripheral_type_lookup=self.state_manager.lookup_peripheral_type,
                            logger=self.get_logger(),
                            on_values_changed=self._broadcast_routing_values,
                        )
                        self.ros_bridge.add_routing_listener(
                            self.routing_evaluator.on_topic_message
                        )
                        self.state_manager.set_routing_evaluator(self.routing_evaluator)
                        self.get_logger().info('Routing graph evaluator initialized')
                    except Exception as e:
                        self.get_logger().warn(f'Routing evaluator not available: {e}')
                except Exception as e:
                    self.get_logger().warn(f'ROS Bridge not available: {e}')

            await self.web_server.start()

            self.get_logger().info(f'Web server started on http://localhost:{self.web_port}/')

            # Update manager: GitHub release polling + USB scan + apply.
            # Wired to the WebSocket handler so state changes broadcast to
            # clients on the 'update' topic.
            try:
                from saint_server.update_manager import UpdateManager
                self.update_manager = UpdateManager(logger=self.get_logger())
                if self.web_server.ws_handler:
                    self.web_server.ws_handler.set_update_manager(self.update_manager)
                # Fire a non-blocking startup check. Offline robots will
                # land on 'no_network' status quickly (5s timeout) and the
                # UI will surface a "Check for Updates" button.
                asyncio.create_task(self.update_manager.check_github())
                self.get_logger().info('Update manager initialized')
            except Exception as e:
                self.get_logger().warn(f'Update manager not available: {e}')

        except ImportError as e:
            self.get_logger().warn(f'Web server not available: {e}')
        except Exception as e:
            self.get_logger().error(f'Failed to start web server: {e}')

        # Start LiveLink receiver if enabled
        if self._livelink_enabled:
            await self._start_livelink()

        # Start discovery service if enabled
        if self._discovery_enabled:
            await self._start_discovery()

    async def _start_discovery(self):
        """Start the node discovery service."""
        try:
            self.discovery_service = DiscoveryService(
                agent_port=self._agent_port,
                discovery_port=self._discovery_port,
                logger=self.get_logger()
            )
            await self.discovery_service.start()
            self.get_logger().info(
                f'Discovery service started on UDP port {self._discovery_port} '
                f'(agent port: {self._agent_port})'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to start discovery service: {e}')

    async def _stop_discovery(self):
        """Stop the discovery service."""
        if self.discovery_service:
            await self.discovery_service.stop()
            self.discovery_service = None

    def _broadcast_activity(self, message: str, level: str):
        """Broadcast activity message to WebSocket clients."""
        if self.web_server and self.web_server.ws_handler:
            # Queue the activity broadcast in the async loop
            if self._async_loop:
                self._async_loop.call_soon_threadsafe(
                    lambda: asyncio.create_task(
                        self.web_server.ws_handler.broadcast_activity(message, level)
                    )
                )

    def _broadcast_node_log(self, node_id: str, entry: Dict[str, Any]):
        """Push a per-node log entry to subscribers (thread-safe)."""
        if self.web_server and self.web_server.ws_handler:
            if self._async_loop:
                self._async_loop.call_soon_threadsafe(
                    lambda nid=node_id, e=entry: asyncio.create_task(
                        self.web_server.ws_handler.broadcast_node_log(nid, e)
                    )
                )

    def _broadcast_ros_message(self, topic: str, data: Dict[str, Any]):
        """Broadcast ROS message to WebSocket clients (thread-safe)."""
        if self.web_server and self.web_server.ws_handler:
            if self._async_loop:
                # Use call_soon_threadsafe to schedule broadcast in async loop
                self._async_loop.call_soon_threadsafe(
                    lambda t=topic, d=data: asyncio.create_task(
                        self.web_server.ws_handler.broadcast_ros_state(t, d)
                    )
                )

    # Throttle state for the routing-evaluator value broadcast. We want
    # live values on the routing UI but a Joy stream at 30Hz drags too
    # much over the WebSocket — clamp to 20Hz / 50ms with a trailing
    # edge so the final snapshot in any burst always reaches the UI.
    _ROUTING_VALUES_INTERVAL_S = 0.05
    _last_routing_values_emit: float = 0.0
    _pending_routing_snapshot: Optional[Dict[str, Any]] = None
    _routing_trail_scheduled: bool = False

    def _broadcast_routing_values(self, snapshot: Dict[str, Any]):
        """Forward the routing evaluator's value snapshot to subscribers.

        Called from the ROS callback thread on every sheet evaluation.
        Leading-edge broadcast every 50ms; intra-window updates are
        coalesced into `_pending_routing_snapshot` and emitted by a
        single deferred trailing-edge fire. Without the trail, the
        last snapshot in a burst (e.g. the all-zero state when the
        operator releases the joystick) silently disappears and the
        UI sticks on whichever intermediate value was the leading-edge
        broadcast.
        """
        if not (self.web_server and self.web_server.ws_handler and self._async_loop):
            return
        import time as _time
        now = _time.monotonic()
        elapsed = now - self._last_routing_values_emit
        if elapsed >= self._ROUTING_VALUES_INTERVAL_S:
            # Leading edge: emit immediately, clear any pending trail
            # so we don't double-send the same snapshot.
            self._last_routing_values_emit = now
            self._pending_routing_snapshot = None
            self._emit_routing_snapshot(snapshot)
            return
        # Inside the throttle window — stash the latest snapshot and
        # schedule a single deferred fire if one isn't already pending.
        self._pending_routing_snapshot = snapshot
        if self._routing_trail_scheduled:
            return
        self._routing_trail_scheduled = True
        delay = max(0.0, self._ROUTING_VALUES_INTERVAL_S - elapsed)
        self._async_loop.call_soon_threadsafe(
            lambda d=delay: self._async_loop.call_later(d, self._flush_routing_trail)
        )

    def _flush_routing_trail(self):
        """Trailing-edge fire of the most-recent pending snapshot."""
        self._routing_trail_scheduled = False
        snap = self._pending_routing_snapshot
        self._pending_routing_snapshot = None
        if snap is None:
            return
        import time as _time
        self._last_routing_values_emit = _time.monotonic()
        self._emit_routing_snapshot(snap)

    def _emit_routing_snapshot(self, snapshot: Dict[str, Any]):
        """Schedule the actual websocket broadcast."""
        handler = self.web_server.ws_handler
        self._async_loop.call_soon_threadsafe(
            lambda s=snapshot: asyncio.create_task(
                handler.broadcast_state('routing_values', s)
            )
        )

    async def stop_async_services(self):
        """Stop async services gracefully."""
        self.get_logger().info('Stopping async services...')

        # Stop discovery service
        if self.discovery_service:
            await self._stop_discovery()

        # Stop LiveLink
        if self.livelink_receiver:
            await self._stop_livelink()

        if self.web_server:
            await self.web_server.stop()

    def start(self):
        """Start the async services in a background thread."""
        self._async_thread = threading.Thread(
            target=self._run_async_loop,
            name='saint-async',
            daemon=True
        )
        self._async_thread.start()
        self.get_logger().info('Async services thread started')

    def shutdown(self):
        """Signal async services to shut down and wait for thread."""
        if self._async_loop and self._shutdown_event:
            # Signal the async loop to stop
            self._async_loop.call_soon_threadsafe(self._shutdown_event.set)

        if self._async_thread and self._async_thread.is_alive():
            self._async_thread.join(timeout=5.0)
            if self._async_thread.is_alive():
                self.get_logger().warn('Async thread did not stop in time')

        # Flush the peripheral logger's disk queue so we don't lose the
        # last second of samples on a clean exit. Daemon thread would
        # die with the process anyway; shutdown() drains the queue first.
        try:
            self.peripheral_logger.shutdown()
        except Exception as e:
            self.get_logger().warn(f'PeripheralLogger shutdown failed: {e}')


def main(args=None):
    """Main entry point for the SAINT.OS server."""
    rclpy.init(args=args)

    node = SaintServerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Use threading Event for cooperative shutdown
    shutdown_requested = threading.Event()

    def signal_handler(sig, frame):
        node.get_logger().info('Shutdown signal received')
        shutdown_requested.set()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        node.get_logger().info('SAINT.OS Server starting...')
        node.start()  # Start async services thread

        # Spin with timeout to allow checking shutdown flag
        while not shutdown_requested.is_set():
            executor.spin_once(timeout_sec=0.1)

    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down...')
        node.shutdown()  # Signal async services to stop
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
