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
import signal
import sys
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

        # Initialize components (will be implemented)
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

    async def start_async_services(self):
        """Start async services (WebSocket, LiveLink, etc.)."""
        self.get_logger().info('Starting async services...')
        # TODO: Start WebSocket server
        # TODO: Start LiveLink receiver if enabled
        # TODO: Start RC receiver if enabled

    async def stop_async_services(self):
        """Stop async services gracefully."""
        self.get_logger().info('Stopping async services...')
        # TODO: Stop all async services


def main(args=None):
    """Main entry point for the SAINT.OS server."""
    rclpy.init(args=args)

    node = SaintServerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Setup signal handlers for graceful shutdown
    def signal_handler(sig, frame):
        node.get_logger().info('Shutdown signal received')
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        node.get_logger().info('SAINT.OS Server starting...')
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
