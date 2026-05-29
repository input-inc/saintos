"""
Animation data structures for Unreal Engine animation sequences.

These structures represent parsed animation data in a format suitable
for robot motion playback.
"""

from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
from enum import IntEnum


class CurveInterpolation(IntEnum):
    """Curve interpolation mode — codes match the Vue catalog in
    src/composables/easings.js. Legacy codes 0/1/2 are the original
    Unreal enum; 3-6 are the CSS basics; 10-39 are the easings.net
    palette (Sine, Quad, Cubic, Quart, Quint, Expo, Circ, Back,
    Elastic, Bounce — each with In, Out, InOut)."""
    CONSTANT     = 0
    LINEAR       = 1
    CUBIC        = 2
    EASE         = 3
    EASE_IN      = 4
    EASE_OUT     = 5
    EASE_IN_OUT  = 6
    # ── easings.net ──────────────────────────────────────────────
    EASE_IN_SINE     = 10
    EASE_OUT_SINE    = 11
    EASE_IN_OUT_SINE = 12
    EASE_IN_QUAD     = 13
    EASE_OUT_QUAD    = 14
    EASE_IN_OUT_QUAD = 15
    EASE_IN_CUBIC    = 16
    EASE_OUT_CUBIC   = 17
    EASE_IN_OUT_CUBIC = 18
    EASE_IN_QUART    = 19
    EASE_OUT_QUART   = 20
    EASE_IN_OUT_QUART = 21
    EASE_IN_QUINT    = 22
    EASE_OUT_QUINT   = 23
    EASE_IN_OUT_QUINT = 24
    EASE_IN_EXPO     = 25
    EASE_OUT_EXPO    = 26
    EASE_IN_OUT_EXPO = 27
    EASE_IN_CIRC     = 28
    EASE_OUT_CIRC    = 29
    EASE_IN_OUT_CIRC = 30
    EASE_IN_BACK     = 31
    EASE_OUT_BACK    = 32
    EASE_IN_OUT_BACK = 33
    EASE_IN_ELASTIC  = 34
    EASE_OUT_ELASTIC = 35
    EASE_IN_OUT_ELASTIC = 36
    EASE_IN_BOUNCE   = 37
    EASE_OUT_BOUNCE  = 38
    EASE_IN_OUT_BOUNCE = 39


# CSS easing → bezier control points. Mirrors EASING_BEZIERS on the
# Vue side so playback matches what the operator drew in the timeline.
_EASING_BEZIERS = {
    CurveInterpolation.LINEAR:      (0.0,  0.0,  1.0,   1.0),
    CurveInterpolation.EASE:        (0.25, 0.1,  0.25,  1.0),
    CurveInterpolation.EASE_IN:     (0.42, 0.0,  1.0,   1.0),
    CurveInterpolation.EASE_OUT:    (0.0,  0.0,  0.58,  1.0),
    CurveInterpolation.EASE_IN_OUT: (0.42, 0.0,  0.58,  1.0),
}


def _cubic_bezier_at_time(t: float, c1x: float, c1y: float,
                          c2x: float, c2y: float) -> float:
    """CSS-spec cubic-bezier easing: solve for parametric s where
    x(s) == t, return y(s). Newton-Raphson with 8 iterations matches
    browser-engine behavior and is plenty for 60 fps playback."""
    if t <= 0.0:
        return 0.0
    if t >= 1.0:
        return 1.0
    s = t
    for _ in range(8):
        one_s = 1.0 - s
        x = (3 * one_s * one_s * s * c1x
             + 3 * one_s * s * s * c2x
             + s * s * s)
        dx = (3 * one_s * one_s * c1x
              + 6 * one_s * s * (c2x - c1x)
              + 3 * s * s * (1.0 - c2x))
        if abs(x - t) < 1e-5 or abs(dx) < 1e-6:
            break
        s -= (x - t) / dx
        if s < 0.0:
            s = 0.0
        elif s > 1.0:
            s = 1.0
    one_s = 1.0 - s
    return (3 * one_s * one_s * s * c1y
            + 3 * one_s * s * s * c2y
            + s * s * s)


# ── easings.net palette ─────────────────────────────────────────────
#
# Formulas ported verbatim from https://easings.net/ which credits
# Robert Penner's original easing equations. Keep the Vue-side
# easings.js and these in sync — any divergence shows up as a
# preview-vs-playback mismatch.

import math

_C1 = 1.70158
_C2 = _C1 * 1.525
_C3 = _C1 + 1
_C4 = (2 * math.pi) / 3
_C5 = (2 * math.pi) / 4.5
_N1 = 7.5625
_D1 = 2.75


def _bounce_out(x: float) -> float:
    if x < 1 / _D1:
        return _N1 * x * x
    if x < 2 / _D1:
        x -= 1.5 / _D1
        return _N1 * x * x + 0.75
    if x < 2.5 / _D1:
        x -= 2.25 / _D1
        return _N1 * x * x + 0.9375
    x -= 2.625 / _D1
    return _N1 * x * x + 0.984375


def _easings_net_at(code: int, t: float) -> float:
    """Sample one of the easings.net functions at t in [0, 1]. Returns
    the linear identity if the code isn't recognized — keeps playback
    forward-compatible with future catalog entries."""
    if t <= 0.0:
        return 0.0
    if t >= 1.0:
        return 1.0
    # Sine
    if code == 10: return 1 - math.cos((t * math.pi) / 2)
    if code == 11: return math.sin((t * math.pi) / 2)
    if code == 12: return -(math.cos(math.pi * t) - 1) / 2
    # Quad
    if code == 13: return t * t
    if code == 14: return 1 - (1 - t) * (1 - t)
    if code == 15: return 2 * t * t if t < 0.5 else 1 - pow(-2 * t + 2, 2) / 2
    # Cubic (easings.net — distinct from CurveInterpolation.CUBIC Hermite)
    if code == 16: return t ** 3
    if code == 17: return 1 - pow(1 - t, 3)
    if code == 18: return 4 * t ** 3 if t < 0.5 else 1 - pow(-2 * t + 2, 3) / 2
    # Quart
    if code == 19: return t ** 4
    if code == 20: return 1 - pow(1 - t, 4)
    if code == 21: return 8 * t ** 4 if t < 0.5 else 1 - pow(-2 * t + 2, 4) / 2
    # Quint
    if code == 22: return t ** 5
    if code == 23: return 1 - pow(1 - t, 5)
    if code == 24: return 16 * t ** 5 if t < 0.5 else 1 - pow(-2 * t + 2, 5) / 2
    # Expo
    if code == 25: return 0.0 if t == 0 else pow(2, 10 * t - 10)
    if code == 26: return 1.0 if t == 1 else 1 - pow(2, -10 * t)
    if code == 27:
        if t == 0: return 0.0
        if t == 1: return 1.0
        return pow(2, 20 * t - 10) / 2 if t < 0.5 else (2 - pow(2, -20 * t + 10)) / 2
    # Circ
    if code == 28: return 1 - math.sqrt(1 - t * t)
    if code == 29: return math.sqrt(1 - (t - 1) ** 2)
    if code == 30:
        return ((1 - math.sqrt(1 - (2 * t) ** 2)) / 2 if t < 0.5
                else (math.sqrt(1 - (-2 * t + 2) ** 2) + 1) / 2)
    # Back
    if code == 31: return _C3 * t ** 3 - _C1 * t * t
    if code == 32: return 1 + _C3 * (t - 1) ** 3 + _C1 * (t - 1) ** 2
    if code == 33:
        return ((2 * t) ** 2 * ((_C2 + 1) * 2 * t - _C2) / 2 if t < 0.5
                else ((2 * t - 2) ** 2 * ((_C2 + 1) * (t * 2 - 2) + _C2) + 2) / 2)
    # Elastic
    if code == 34:
        if t == 0: return 0.0
        if t == 1: return 1.0
        return -pow(2, 10 * t - 10) * math.sin((t * 10 - 10.75) * _C4)
    if code == 35:
        if t == 0: return 0.0
        if t == 1: return 1.0
        return pow(2, -10 * t) * math.sin((t * 10 - 0.75) * _C4) + 1
    if code == 36:
        if t == 0: return 0.0
        if t == 1: return 1.0
        if t < 0.5:
            return -(pow(2, 20 * t - 10) * math.sin((20 * t - 11.125) * _C5)) / 2
        return (pow(2, -20 * t + 10) * math.sin((20 * t - 11.125) * _C5)) / 2 + 1
    # Bounce
    if code == 37: return 1 - _bounce_out(1 - t)
    if code == 38: return _bounce_out(t)
    if code == 39:
        return ((1 - _bounce_out(1 - 2 * t)) / 2 if t < 0.5
                else (1 + _bounce_out(2 * t - 1)) / 2)
    return t


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
                if k0.interp == CurveInterpolation.LINEAR:
                    return k0.value + t * (k1.value - k0.value)
                if k0.interp == CurveInterpolation.CUBIC:
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
                # CSS easing presets — cubic-bezier evaluated at the
                # normalized segment progress `t`, scaled into the
                # segment's value span.
                cp = _EASING_BEZIERS.get(k0.interp)
                if cp is not None:
                    y = _cubic_bezier_at_time(t, cp[0], cp[1], cp[2], cp[3])
                    return k0.value + y * (k1.value - k0.value)
                # easings.net catalog (codes 10-39).
                code = int(k0.interp)
                if 10 <= code <= 39:
                    y = _easings_net_at(code, t)
                    return k0.value + y * (k1.value - k0.value)
                # Unknown interp — fall back to linear so playback
                # never silently jumps to zero on legacy data.
                return k0.value + t * (k1.value - k0.value)

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
