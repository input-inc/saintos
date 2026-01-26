"""
SAINT.OS Server Launch File

Launches the SAINT.OS server along with the micro-ROS agent for
microcontroller node support.

Usage:
    ros2 launch saint_os saint_server.launch.py

Options:
    server_name:=NAME       Server name (default: SAINT-01)
    web_port:=PORT          Web interface port (default: 80)
    agent_port:=PORT        micro-ROS agent UDP port (default: 8888)
    enable_agent:=BOOL      Enable micro-ROS agent (default: true on Linux, false on macOS)

Platform Notes:
    - Linux (including Raspberry Pi): micro-ROS agent runs natively
    - macOS: Run agent separately via Docker (see docs/DEVELOPMENT.md)
"""

import platform
import shutil

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for SAINT.OS server."""

    # Detect platform and agent availability
    is_linux = platform.system() == 'Linux'
    is_macos = platform.system() == 'Darwin'
    agent_available = shutil.which('micro_ros_agent') is not None

    # Default: enable agent on Linux if available, disable on macOS
    default_enable_agent = 'true' if (is_linux and agent_available) else 'false'

    # Declare launch arguments
    server_name_arg = DeclareLaunchArgument(
        'server_name',
        default_value='SAINT-01',
        description='Name of this SAINT.OS server'
    )

    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value='80',
        description='Port for web administration interface'
    )

    agent_port_arg = DeclareLaunchArgument(
        'agent_port',
        default_value='8888',
        description='UDP port for micro-ROS agent'
    )

    enable_agent_arg = DeclareLaunchArgument(
        'enable_agent',
        default_value=default_enable_agent,
        description='Enable micro-ROS agent (auto-detected based on platform)'
    )

    # Get launch configuration values
    server_name = LaunchConfiguration('server_name')
    web_port = LaunchConfiguration('web_port')
    agent_port = LaunchConfiguration('agent_port')
    enable_agent = LaunchConfiguration('enable_agent')

    # SAINT.OS Server Node
    saint_server_node = Node(
        package='saint_os',
        executable='saint_server',
        name='saint_server',
        parameters=[{
            'server_name': server_name,
            'web_port': web_port,
        }],
        output='screen',
        emulate_tty=True,
    )

    # micro-ROS Agent (for microcontroller nodes)
    # Uses UDP transport on specified port
    # Only runs on Linux where the agent is available
    micro_ros_agent = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'udp4', '--port', agent_port,
            '-v4'  # Verbosity level
        ],
        name='micro_ros_agent',
        output='screen',
        condition=IfCondition(enable_agent),
    )

    # Log startup info
    startup_info = LogInfo(
        msg=['Starting SAINT.OS Server "', server_name, '" with web interface on port ', web_port]
    )

    agent_info = LogInfo(
        msg=['micro-ROS agent listening on UDP port ', agent_port],
        condition=IfCondition(enable_agent),
    )

    # Warning for macOS users if agent is needed but not running
    macos_agent_warning = LogInfo(
        msg=[
            '\n',
            '=' * 70, '\n',
            'NOTE: micro-ROS agent not started (macOS detected)\n',
            'To connect microcontroller nodes, run the agent via Docker:\n',
            '\n',
            '  docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888\n',
            '\n',
            'See docs/DEVELOPMENT.md for details.\n',
            '=' * 70,
        ],
    ) if is_macos else LogInfo(msg=[''])

    # Warning if agent requested but not available
    agent_not_found_warning = LogInfo(
        msg=[
            '\n',
            'WARNING: micro-ROS agent not found. Install with:\n',
            '  sudo apt install ros-humble-micro-ros-agent\n',
        ],
    ) if (is_linux and not agent_available) else LogInfo(msg=[''])

    return LaunchDescription([
        # Arguments
        server_name_arg,
        web_port_arg,
        agent_port_arg,
        enable_agent_arg,

        # Info
        startup_info,
        agent_info,
        macos_agent_warning,
        agent_not_found_warning,

        # Nodes
        saint_server_node,
        micro_ros_agent,
    ])
