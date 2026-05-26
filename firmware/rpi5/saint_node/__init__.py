"""
SAINT.OS Raspberry Pi 5 Node Firmware

A ROS2 node that runs on Raspberry Pi 5, providing GPIO control
and sensor reading capabilities for the SAINT robotics platform.

Unlike the RP2040 microcontroller firmware, this runs as a standard
ROS2 node on Linux with direct DDS communication to the server.
"""

__version__ = "1.1.0"
