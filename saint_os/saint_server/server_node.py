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
from typing import Optional, Dict, Any

from saint_server.webserver.state_manager import StateManager
from saint_server.roles import RoleManager


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
            is_new = self.state_manager.update_node_from_announcement(msg.data)
            if is_new:
                self.get_logger().info(f'New node discovered via announcement')
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
                # Broadcast update to WebSocket clients
                self._broadcast_node_capabilities(node_id)
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
                    self.get_logger().info(f'Broadcasting capabilities for {node_id} to WebSocket clients (pins: {len(caps.get("pins", []))})')
                    # Capture caps in local scope to avoid closure issues
                    caps_copy = caps
                    topic = f'node_capabilities/{node_id}'
                    self._async_loop.call_soon_threadsafe(
                        lambda c=caps_copy, t=topic: asyncio.create_task(
                            self.web_server.ws_handler.broadcast_state(t, c)
                        )
                    )
                else:
                    self.get_logger().warn(f'No capabilities found to broadcast for {node_id}')

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

            await self.web_server.start()

            self.get_logger().info(f'Web server started on http://localhost:{self.web_port}/')

        except ImportError as e:
            self.get_logger().warn(f'Web server not available: {e}')
        except Exception as e:
            self.get_logger().error(f'Failed to start web server: {e}')

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
