"""
SAINT.OS Server Launch File

Launches the main SAINT.OS server node alongside the micro-ROS agent
that microcontroller nodes connect to. The agent runs as a separate
process so its lifecycle is tied to ros2 launch (and therefore to the
systemd unit), not embedded in the Python server.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('saint_os')

    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=os.path.join(pkg_share, 'config', 'server.yaml'),
        description='Path to the server configuration file'
    )

    server_name_arg = DeclareLaunchArgument(
        'server_name',
        default_value='SAINT-01',
        description='Name of this SAINT.OS server'
    )

    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value='80',
        description='HTTP port for web administration interface'
    )

    websocket_port_arg = DeclareLaunchArgument(
        'websocket_port',
        default_value='9090',
        description='WebSocket port for client connections'
    )

    agent_port_arg = DeclareLaunchArgument(
        'agent_port',
        default_value='8888',
        description='UDP port for the micro-ROS agent — must match the port '
                    'advertised by the discovery service'
    )

    livelink_enabled_arg = DeclareLaunchArgument(
        'livelink_enabled',
        default_value='true',
        description='Enable LiveLink receiver'
    )

    rc_enabled_arg = DeclareLaunchArgument(
        'rc_enabled',
        default_value='false',
        description='Enable RC receiver'
    )

    # Create the server node.
    # NOTE: `config/server.yaml` is a generic application config (nested
    # keys), NOT a ROS2 parameters file — the saint_server code reads it
    # directly via saint_server/config/__init__.py. We pass only the
    # explicit per-launch overrides through ROS2 parameters.
    server_node = Node(
        package='saint_os',
        executable='saint_server',
        name='saint_server',
        output='screen',
        parameters=[
            {
                'server_name': LaunchConfiguration('server_name'),
                'web_port': LaunchConfiguration('web_port'),
                'websocket_port': LaunchConfiguration('websocket_port'),
                'livelink_enabled': LaunchConfiguration('livelink_enabled'),
                'rc_enabled': LaunchConfiguration('rc_enabled'),
            }
        ],
    )

    # micro-ROS agent — XRCE-DDS bridge listening on UDP. Without this,
    # node controllers complete the discovery handshake but can't actually
    # publish their announcement, so they never appear in the UI list.
    micro_ros_agent = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'udp4', '--port', LaunchConfiguration('agent_port'),
            '-v4',
        ],
        name='micro_ros_agent',
        output='screen',
    )

    agent_info = LogInfo(
        msg=['micro-ROS agent listening on UDP port ',
             LaunchConfiguration('agent_port')],
    )

    return LaunchDescription([
        config_arg,
        server_name_arg,
        web_port_arg,
        websocket_port_arg,
        agent_port_arg,
        livelink_enabled_arg,
        rc_enabled_arg,
        agent_info,
        server_node,
        micro_ros_agent,
    ])
