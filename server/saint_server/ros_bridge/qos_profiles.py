"""
QoS Profile definitions for ROS Bridge topics.
"""

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


# QoS for state topics (reliable, depth 1, volatile)
# Used for HeadState, ArmState, GripperState, TrackState
STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

# QoS for command topics (reliable, depth 10, volatile)
# Used for HeadCommand, ArmCommand, GripperCommand
COMMAND_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Dictionary of named QoS profiles
QOS_PROFILES = {
    'state': STATE_QOS,
    'command': COMMAND_QOS,
}
