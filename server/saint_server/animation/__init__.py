"""Animation builder support: robot model storage, pose/animation models, playback.

The robot model is three files — URDF (structure), SRDF (groups + named
poses), and the rig file (controls). See docs/RIG_SCHEMA.md.
"""

from saint_server.animation.robot_model_store import (
    RobotModelError,
    RobotModelStore,
)

__all__ = ["RobotModelStore", "RobotModelError"]
