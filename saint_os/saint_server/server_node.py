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

# Use std_msgs/String for node announcements (matches firmware)
from std_msgs.msg import String

import asyncio
import os
import signal
import threading
from typing import Optional, Dict, Any, List

from saint_server.webserver.state_manager import StateManager
from saint_server.roles import RoleManager
from saint_server.livelink import LiveLinkReceiver, LiveLinkRouter
from saint_server.livelink.receiver import LiveLinkConfig
from saint_server.livelink.router import LiveLinkRoute, OutputMapping, MappingType


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

        # Web server components (set during start_async_services)
        self.web_server = None

        # Role manager
        self.role_manager = RoleManager(logger=self.get_logger())

        # LiveLink components
        self.livelink_receiver: Optional[LiveLinkReceiver] = None
        self.livelink_router: Optional[LiveLinkRouter] = None
        self._livelink_enabled = self.get_parameter('livelink_enabled').value

        # Dynamic publishers/subscribers for node communication
        self._node_config_pubs: Dict[str, Any] = {}  # node_id -> publisher
        self._node_caps_subs: Dict[str, Any] = {}    # node_id -> subscription
        self._node_control_pubs: Dict[str, Any] = {} # node_id -> control publisher
        self._node_state_subs: Dict[str, Any] = {}   # node_id -> state subscription

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
        try:
            # Log announcement for debugging
            self.get_logger().debug(f'Received announcement: {msg.data[:100]}...')
            is_new = self.state_manager.update_node_from_announcement(msg.data)
            if is_new:
                self.get_logger().info(f'New node discovered via announcement')
            else:
                # Parse to get node_id for logging
                import json
                try:
                    data = json.loads(msg.data)
                    node_id = data.get('node_id', 'unknown')
                    fw = data.get('fw', '?')
                    self.get_logger().debug(f'Updated node {node_id} (fw: {fw})')
                except:
                    pass
        except Exception as e:
            self.get_logger().error(f'Error processing announcement: {e}')

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
                    # Include sync_status in capabilities broadcast
                    pin_config = self.state_manager.get_node_pin_config(node_id)
                    if pin_config:
                        caps["sync_status"] = pin_config.get("sync_status", "unknown")
                        caps["last_synced"] = pin_config.get("last_synced")
                    # Make a copy to ensure data isn't modified before async send
                    caps_copy = dict(caps)
                    topic = f'node_capabilities/{node_id}'
                    self._async_loop.call_soon_threadsafe(
                        lambda c=caps_copy, t=topic: asyncio.create_task(
                            self.web_server.ws_handler.broadcast_state(t, c)
                        )
                    )

    def _broadcast_sync_status(self, node_id: str):
        """Broadcast sync status update to WebSocket clients."""
        if self.web_server and self.web_server.ws_handler:
            if self._async_loop:
                pin_config = self.state_manager.get_node_pin_config(node_id)
                if pin_config:
                    status_data = {
                        "node_id": node_id,
                        "sync_status": pin_config.get("sync_status", "unknown"),
                        "last_synced": pin_config.get("last_synced"),
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
            10,
            callback_group=self.callback_group
        )
        self._node_control_pubs[node_id] = pub
        self.get_logger().info(f'Created control publisher: {topic}')
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

    def send_firmware_update_command(self, node_id: str, simulation: bool = False):
        """Send firmware update command to a node via ROS2.

        This tells the node to:
        1. For simulation: Restart and load new firmware ELF
        2. For hardware: Enter bootloader mode for UF2 update

        The actual update mechanism depends on the node type.
        """
        import json

        pub = self._ensure_node_control_publisher(node_id)

        # Get firmware info from state manager
        fw_info = self.state_manager.get_server_firmware_info()

        control_data = {
            "action": "firmware_update",
            "version": fw_info.get("version", "0.0.0"),
            "elf_path": fw_info.get("elf_path"),
            "build_date": fw_info.get("build_date"),
        }

        msg = String()
        msg.data = json.dumps(control_data)

        pub.publish(msg)
        self.get_logger().info(f'Sent firmware update command to {node_id} (version: {fw_info.get("version")})')

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
        """
        Callback from LiveLink router to send output to a node.

        This looks up the GPIO pin for the logical output name and sends
        the control command to the node.
        """
        # Get the node's pin config to find the GPIO for this output_name
        pin_config = self.state_manager.get_node_pin_config(node_id)
        if not pin_config:
            return

        # Find the pin with this logical name
        for pin in pin_config.get('pins', []):
            if pin.get('logical_name') == output_name:
                gpio = pin.get('gpio')
                if gpio is not None:
                    self.send_control_command(node_id, gpio, value)
                    return

        # Log if output not found (but don't spam)
        self.get_logger().debug(f'LiveLink output {output_name} not found on node {node_id}')

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
                self.web_server.ws_handler.set_firmware_update_callback(
                    lambda node_id, simulation: self.send_firmware_update_command(node_id, simulation)
                )

            await self.web_server.start()

            self.get_logger().info(f'Web server started on http://localhost:{self.web_port}/')

        except ImportError as e:
            self.get_logger().warn(f'Web server not available: {e}')
        except Exception as e:
            self.get_logger().error(f'Failed to start web server: {e}')

        # Start LiveLink receiver if enabled
        if self._livelink_enabled:
            await self._start_livelink()

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

    async def stop_async_services(self):
        """Stop async services gracefully."""
        self.get_logger().info('Stopping async services...')

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
