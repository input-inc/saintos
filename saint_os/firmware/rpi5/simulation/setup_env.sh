#!/bin/bash
#
# Setup environment for Pi 5 node simulation
#
# Source this script before running simulated nodes:
#   source setup_env.sh
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FIRMWARE_DIR="$(dirname "$SCRIPT_DIR")"

# Add saint_node to Python path
export PYTHONPATH="${FIRMWARE_DIR}:${PYTHONPATH}"

# Enable simulation mode
export SAINT_SIMULATION=1

# Source ROS2 if available
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo "Sourced ROS2 Jazzy"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "Sourced ROS2 Humble"
else
    echo "Warning: ROS2 not found - nodes will fail to start"
fi

echo "Environment configured for Pi 5 simulation"
echo "  PYTHONPATH includes: ${FIRMWARE_DIR}"
echo "  SAINT_SIMULATION=1"
echo ""
echo "Usage:"
echo "  ./rpi5_node_manager.py create test_node --role head"
echo "  ./rpi5_node_manager.py start test_node"
echo "  ./rpi5_node_manager.py list"
