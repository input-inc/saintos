"""
SAINT.OS Server Launch File

Launches the main SAINT.OS server node with configurable parameters.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

    # Create the server node
    server_node = Node(
        package='saint_os',
        executable='saint_server',
        name='saint_server',
        output='screen',
        parameters=[
            LaunchConfiguration('config'),
            {
                'server_name': LaunchConfiguration('server_name'),
                'web_port': LaunchConfiguration('web_port'),
                'websocket_port': LaunchConfiguration('websocket_port'),
                'livelink_enabled': LaunchConfiguration('livelink_enabled'),
                'rc_enabled': LaunchConfiguration('rc_enabled'),
            }
        ],
    )

    return LaunchDescription([
        config_arg,
        server_name_arg,
        web_port_arg,
        websocket_port_arg,
        livelink_enabled_arg,
        rc_enabled_arg,
        server_node,
    ])
