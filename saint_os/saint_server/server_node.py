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

import asyncio
import os
import signal
import threading
from typing import Optional


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

        # Web server components (set during start_async_services)
        self.web_server = None

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
        # Unadopted node announcements
        # self.unadopted_sub = self.create_subscription(
        #     NodeAnnouncement,
        #     '/saint/nodes/unadopted',
        #     self._on_node_announcement,
        #     10
        # )
        pass

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
        # Status broadcast timer
        self.status_timer = self.create_timer(
            1.0,  # 1 Hz
            self._publish_status,
            callback_group=self.callback_group
        )

    def _publish_status(self):
        """Publish system status periodically."""
        # TODO: Implement status publishing
        pass

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

            # Create and start web server
            self.web_server = WebServer(
                web_root=web_root,
                port=self.web_port,
                server_name=self.server_name,
                logger=self.get_logger()
            )
            await self.web_server.start()

            self.get_logger().info(f'Web server started on http://localhost:{self.web_port}/')

        except ImportError as e:
            self.get_logger().warn(f'Web server not available: {e}')
        except Exception as e:
            self.get_logger().error(f'Failed to start web server: {e}')

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
