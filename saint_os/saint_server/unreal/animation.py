"""
Animation data structures for Unreal Engine animation sequences.

These structures represent parsed animation data in a format suitable
for robot motion playback.
"""

from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
from enum import IntEnum


class CurveInterpolation(IntEnum):
    """Curve interpolation mode."""
    CONSTANT = 0
    LINEAR = 1
    CUBIC = 2


@dataclass
class CurveKey:
    """A single keyframe in an animation curve."""
    time: float  # Time in seconds
    value: float  # Value at this keyframe
    interp: CurveInterpolation = CurveInterpolation.LINEAR
    # Tangent data for cubic interpolation
    arrive_tangent: float = 0.0
    leave_tangent: float = 0.0


@dataclass
class AnimationCurve:
    """A single animation curve (e.g., one component of rotation)."""
    name: str  # Curve name (e.g., "head_pan", "joint_1_rotation_x")
    keys: List[CurveKey] = field(default_factory=list)

    def get_value_at_time(self, time: float) -> float:
        """
        Evaluate the curve at a given time.

        Args:
            time: Time in seconds

        Returns:
            Interpolated value at the given time
        """
        if not self.keys:
            return 0.0

        # Before first key
        if time <= self.keys[0].time:
            return self.keys[0].value

        # After last key
        if time >= self.keys[-1].time:
            return self.keys[-1].value

        # Find surrounding keys
        for i in range(len(self.keys) - 1):
            k0 = self.keys[i]
            k1 = self.keys[i + 1]

            if k0.time <= time <= k1.time:
                # Interpolate between k0 and k1
                t = (time - k0.time) / (k1.time - k0.time) if k1.time != k0.time else 0.0

                if k0.interp == CurveInterpolation.CONSTANT:
                    return k0.value
                elif k0.interp == CurveInterpolation.LINEAR:
                    return k0.value + t * (k1.value - k0.value)
                elif k0.interp == CurveInterpolation.CUBIC:
                    # Hermite interpolation
                    t2 = t * t
                    t3 = t2 * t
                    h1 = 2 * t3 - 3 * t2 + 1
                    h2 = -2 * t3 + 3 * t2
                    h3 = t3 - 2 * t2 + t
                    h4 = t3 - t2
                    dt = k1.time - k0.time
                    return (h1 * k0.value + h2 * k1.value +
                            h3 * k0.leave_tangent * dt +
                            h4 * k1.arrive_tangent * dt)

        return self.keys[-1].value

    @property
    def duration(self) -> float:
        """Get the duration of this curve."""
        if not self.keys:
            return 0.0
        return self.keys[-1].time - self.keys[0].time

    @property
    def start_time(self) -> float:
        """Get the start time of this curve."""
        return self.keys[0].time if self.keys else 0.0

    @property
    def end_time(self) -> float:
        """Get the end time of this curve."""
        return self.keys[-1].time if self.keys else 0.0


@dataclass
class Vector3:
    """3D vector for position/scale."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def to_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def to_dict(self) -> Dict[str, float]:
        return {'x': self.x, 'y': self.y, 'z': self.z}


@dataclass
class Quaternion:
    """Quaternion for rotation."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0

    def to_tuple(self) -> Tuple[float, float, float, float]:
        return (self.x, self.y, self.z, self.w)

    def to_dict(self) -> Dict[str, float]:
        return {'x': self.x, 'y': self.y, 'z': self.z, 'w': self.w}

    def to_euler_degrees(self) -> Tuple[float, float, float]:
        """Convert quaternion to Euler angles (roll, pitch, yaw) in degrees."""
        import math

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (self.w * self.x + self.y * self.z)
        cosr_cosp = 1 - 2 * (self.x * self.x + self.y * self.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (self.w * self.y - self.z * self.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (self.w * self.z + self.x * self.y)
        cosy_cosp = 1 - 2 * (self.y * self.y + self.z * self.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Convert to degrees
        return (
            math.degrees(roll),
            math.degrees(pitch),
            math.degrees(yaw)
        )


@dataclass
class BoneTransform:
    """Transform for a single bone at a single frame."""
    position: Vector3 = field(default_factory=Vector3)
    rotation: Quaternion = field(default_factory=Quaternion)
    scale: Vector3 = field(default_factory=lambda: Vector3(1.0, 1.0, 1.0))

    def to_dict(self) -> Dict:
        return {
            'position': self.position.to_dict(),
            'rotation': self.rotation.to_dict(),
            'scale': self.scale.to_dict(),
        }


@dataclass
class BoneTrack:
    """Animation track for a single bone."""
    bone_name: str
    position_keys: List[Tuple[float, Vector3]] = field(default_factory=list)  # (time, pos)
    rotation_keys: List[Tuple[float, Quaternion]] = field(default_factory=list)  # (time, rot)
    scale_keys: List[Tuple[float, Vector3]] = field(default_factory=list)  # (time, scale)

    def get_transform_at_time(self, time: float) -> BoneTransform:
        """Get interpolated bone transform at given time."""
        transform = BoneTransform()

        # Interpolate position
        if self.position_keys:
            transform.position = self._interpolate_vector(self.position_keys, time)

        # Interpolate rotation (simple linear - should use slerp for production)
        if self.rotation_keys:
            transform.rotation = self._interpolate_quat(self.rotation_keys, time)

        # Interpolate scale
        if self.scale_keys:
            transform.scale = self._interpolate_vector(self.scale_keys, time)

        return transform

    def _interpolate_vector(self, keys: List[Tuple[float, Vector3]], time: float) -> Vector3:
        """Linear interpolation for Vector3 keys."""
        if not keys:
            return Vector3()

        if time <= keys[0][0]:
            return keys[0][1]
        if time >= keys[-1][0]:
            return keys[-1][1]

        for i in range(len(keys) - 1):
            t0, v0 = keys[i]
            t1, v1 = keys[i + 1]
            if t0 <= time <= t1:
                alpha = (time - t0) / (t1 - t0) if t1 != t0 else 0.0
                return Vector3(
                    v0.x + alpha * (v1.x - v0.x),
                    v0.y + alpha * (v1.y - v0.y),
                    v0.z + alpha * (v1.z - v0.z),
                )

        return keys[-1][1]

    def _interpolate_quat(self, keys: List[Tuple[float, Quaternion]], time: float) -> Quaternion:
        """Linear interpolation for Quaternion keys (should use slerp)."""
        if not keys:
            return Quaternion()

        if time <= keys[0][0]:
            return keys[0][1]
        if time >= keys[-1][0]:
            return keys[-1][1]

        for i in range(len(keys) - 1):
            t0, q0 = keys[i]
            t1, q1 = keys[i + 1]
            if t0 <= time <= t1:
                alpha = (time - t0) / (t1 - t0) if t1 != t0 else 0.0
                # Simple lerp (not slerp - good enough for small angles)
                return Quaternion(
                    q0.x + alpha * (q1.x - q0.x),
                    q0.y + alpha * (q1.y - q0.y),
                    q0.z + alpha * (q1.z - q0.z),
                    q0.w + alpha * (q1.w - q0.w),
                )

        return keys[-1][1]


@dataclass
class AnimationSequence:
    """
    A complete animation sequence parsed from Unreal.

    Contains both generic curves (for blend shapes, custom properties)
    and bone tracks (for skeletal animation).
    """
    name: str
    duration: float  # Total duration in seconds
    frame_rate: float = 30.0  # Frames per second
    num_frames: int = 0

    # Generic float curves (blend shapes, custom curves)
    curves: Dict[str, AnimationCurve] = field(default_factory=dict)

    # Bone animation tracks
    bone_tracks: Dict[str, BoneTrack] = field(default_factory=dict)

    # Metadata
    source_file: Optional[str] = None
    unreal_version: Optional[str] = None

    def get_curve(self, name: str) -> Optional[AnimationCurve]:
        """Get a curve by name."""
        return self.curves.get(name)

    def get_bone_track(self, bone_name: str) -> Optional[BoneTrack]:
        """Get a bone track by bone name."""
        return self.bone_tracks.get(bone_name)

    def sample_at_time(self, time: float) -> Dict[str, float]:
        """
        Sample all curves at a given time.

        Returns:
            Dict mapping curve name to interpolated value
        """
        return {
            name: curve.get_value_at_time(time)
            for name, curve in self.curves.items()
        }

    def sample_bones_at_time(self, time: float) -> Dict[str, BoneTransform]:
        """
        Sample all bone transforms at a given time.

        Returns:
            Dict mapping bone name to interpolated transform
        """
        return {
            name: track.get_transform_at_time(time)
            for name, track in self.bone_tracks.items()
        }

    def to_dict(self) -> Dict:
        """Convert to dictionary for JSON serialization."""
        return {
            'name': self.name,
            'duration': self.duration,
            'frame_rate': self.frame_rate,
            'num_frames': self.num_frames,
            'curves': {
                name: {
                    'name': curve.name,
                    'keys': [
                        {
                            'time': k.time,
                            'value': k.value,
                            'interp': k.interp.name,
                        }
                        for k in curve.keys
                    ]
                }
                for name, curve in self.curves.items()
            },
            'bone_tracks': list(self.bone_tracks.keys()),
            'source_file': self.source_file,
        }
