"""
ARKit Blend Shape Definitions

Defines the 52 ARKit blend shapes used by Live Link Face app.
Values range from 0.0 to 1.0.
"""

from dataclasses import dataclass, field
from typing import Dict, List
from enum import IntEnum


class BlendShapeIndex(IntEnum):
    """ARKit blend shape indices (order matters for parsing)."""
    # Eyes
    EyeBlinkLeft = 0
    EyeLookDownLeft = 1
    EyeLookInLeft = 2
    EyeLookOutLeft = 3
    EyeLookUpLeft = 4
    EyeSquintLeft = 5
    EyeWideLeft = 6
    EyeBlinkRight = 7
    EyeLookDownRight = 8
    EyeLookInRight = 9
    EyeLookOutRight = 10
    EyeLookUpRight = 11
    EyeSquintRight = 12
    EyeWideRight = 13

    # Jaw
    JawForward = 14
    JawLeft = 15
    JawRight = 16
    JawOpen = 17

    # Mouth
    MouthClose = 18
    MouthFunnel = 19
    MouthPucker = 20
    MouthLeft = 21
    MouthRight = 22
    MouthSmileLeft = 23
    MouthSmileRight = 24
    MouthFrownLeft = 25
    MouthFrownRight = 26
    MouthDimpleLeft = 27
    MouthDimpleRight = 28
    MouthStretchLeft = 29
    MouthStretchRight = 30
    MouthRollLower = 31
    MouthRollUpper = 32
    MouthShrugLower = 33
    MouthShrugUpper = 34
    MouthPressLeft = 35
    MouthPressRight = 36
    MouthLowerDownLeft = 37
    MouthLowerDownRight = 38
    MouthUpperUpLeft = 39
    MouthUpperUpRight = 40

    # Brows
    BrowDownLeft = 41
    BrowDownRight = 42
    BrowInnerUp = 43
    BrowOuterUpLeft = 44
    BrowOuterUpRight = 45

    # Cheeks
    CheekPuff = 46
    CheekSquintLeft = 47
    CheekSquintRight = 48

    # Nose
    NoseSneerLeft = 49
    NoseSneerRight = 50

    # Tongue
    TongueOut = 51

    # Head rotation (Live Link Face extended data)
    HeadYaw = 52
    HeadPitch = 53
    HeadRoll = 54

    # Eye rotation (Live Link Face extended data)
    LeftEyeYaw = 55
    LeftEyePitch = 56
    LeftEyeRoll = 57
    RightEyeYaw = 58
    RightEyePitch = 59
    RightEyeRoll = 60


# Blend shape names in order (for reference and logging)
BLEND_SHAPE_NAMES: List[str] = [
    "EyeBlinkLeft", "EyeLookDownLeft", "EyeLookInLeft", "EyeLookOutLeft",
    "EyeLookUpLeft", "EyeSquintLeft", "EyeWideLeft",
    "EyeBlinkRight", "EyeLookDownRight", "EyeLookInRight", "EyeLookOutRight",
    "EyeLookUpRight", "EyeSquintRight", "EyeWideRight",
    "JawForward", "JawLeft", "JawRight", "JawOpen",
    "MouthClose", "MouthFunnel", "MouthPucker", "MouthLeft", "MouthRight",
    "MouthSmileLeft", "MouthSmileRight", "MouthFrownLeft", "MouthFrownRight",
    "MouthDimpleLeft", "MouthDimpleRight", "MouthStretchLeft", "MouthStretchRight",
    "MouthRollLower", "MouthRollUpper", "MouthShrugLower", "MouthShrugUpper",
    "MouthPressLeft", "MouthPressRight", "MouthLowerDownLeft", "MouthLowerDownRight",
    "MouthUpperUpLeft", "MouthUpperUpRight",
    "BrowDownLeft", "BrowDownRight", "BrowInnerUp", "BrowOuterUpLeft", "BrowOuterUpRight",
    "CheekPuff", "CheekSquintLeft", "CheekSquintRight",
    "NoseSneerLeft", "NoseSneerRight",
    "TongueOut",
    # Head rotation (Live Link Face extended)
    "HeadYaw", "HeadPitch", "HeadRoll",
    # Eye rotation (Live Link Face extended)
    "LeftEyeYaw", "LeftEyePitch", "LeftEyeRoll",
    "RightEyeYaw", "RightEyePitch", "RightEyeRoll",
]

# Total number of standard ARKit blend shapes
BLEND_SHAPE_COUNT = 52

# Extended count including head/eye rotation (Live Link Face)
EXTENDED_BLEND_SHAPE_COUNT = 61


@dataclass
class BlendShapes:
    """
    Container for ARKit blend shape values.

    All values are floats from 0.0 to 1.0.
    """
    # Eyes - Left
    eye_blink_left: float = 0.0
    eye_look_down_left: float = 0.0
    eye_look_in_left: float = 0.0
    eye_look_out_left: float = 0.0
    eye_look_up_left: float = 0.0
    eye_squint_left: float = 0.0
    eye_wide_left: float = 0.0

    # Eyes - Right
    eye_blink_right: float = 0.0
    eye_look_down_right: float = 0.0
    eye_look_in_right: float = 0.0
    eye_look_out_right: float = 0.0
    eye_look_up_right: float = 0.0
    eye_squint_right: float = 0.0
    eye_wide_right: float = 0.0

    # Jaw
    jaw_forward: float = 0.0
    jaw_left: float = 0.0
    jaw_right: float = 0.0
    jaw_open: float = 0.0

    # Mouth
    mouth_close: float = 0.0
    mouth_funnel: float = 0.0
    mouth_pucker: float = 0.0
    mouth_left: float = 0.0
    mouth_right: float = 0.0
    mouth_smile_left: float = 0.0
    mouth_smile_right: float = 0.0
    mouth_frown_left: float = 0.0
    mouth_frown_right: float = 0.0
    mouth_dimple_left: float = 0.0
    mouth_dimple_right: float = 0.0
    mouth_stretch_left: float = 0.0
    mouth_stretch_right: float = 0.0
    mouth_roll_lower: float = 0.0
    mouth_roll_upper: float = 0.0
    mouth_shrug_lower: float = 0.0
    mouth_shrug_upper: float = 0.0
    mouth_press_left: float = 0.0
    mouth_press_right: float = 0.0
    mouth_lower_down_left: float = 0.0
    mouth_lower_down_right: float = 0.0
    mouth_upper_up_left: float = 0.0
    mouth_upper_up_right: float = 0.0

    # Brows
    brow_down_left: float = 0.0
    brow_down_right: float = 0.0
    brow_inner_up: float = 0.0
    brow_outer_up_left: float = 0.0
    brow_outer_up_right: float = 0.0

    # Cheeks
    cheek_puff: float = 0.0
    cheek_squint_left: float = 0.0
    cheek_squint_right: float = 0.0

    # Nose
    nose_sneer_left: float = 0.0
    nose_sneer_right: float = 0.0

    # Tongue
    tongue_out: float = 0.0

    # Head rotation (Live Link Face extended data)
    # Values are in radians, typically range -1 to 1
    head_yaw: float = 0.0      # Turning left/right (negative = left, positive = right)
    head_pitch: float = 0.0    # Tilting up/down (negative = down, positive = up)
    head_roll: float = 0.0     # Tilting side to side (negative = left, positive = right)

    # Eye rotation (Live Link Face extended data)
    left_eye_yaw: float = 0.0
    left_eye_pitch: float = 0.0
    left_eye_roll: float = 0.0
    right_eye_yaw: float = 0.0
    right_eye_pitch: float = 0.0
    right_eye_roll: float = 0.0

    # Timestamp
    timestamp: float = 0.0

    # Subject name (from Live Link Face app)
    subject_name: str = ""

    @classmethod
    def from_array(cls, values: List[float], subject_name: str = "", timestamp: float = 0.0) -> 'BlendShapes':
        """Create BlendShapes from an array of up to 61 float values."""
        if len(values) < EXTENDED_BLEND_SHAPE_COUNT:
            # Pad with zeros if needed
            values = list(values) + [0.0] * (EXTENDED_BLEND_SHAPE_COUNT - len(values))

        return cls(
            # Eyes - Left
            eye_blink_left=values[0],
            eye_look_down_left=values[1],
            eye_look_in_left=values[2],
            eye_look_out_left=values[3],
            eye_look_up_left=values[4],
            eye_squint_left=values[5],
            eye_wide_left=values[6],
            # Eyes - Right
            eye_blink_right=values[7],
            eye_look_down_right=values[8],
            eye_look_in_right=values[9],
            eye_look_out_right=values[10],
            eye_look_up_right=values[11],
            eye_squint_right=values[12],
            eye_wide_right=values[13],
            # Jaw
            jaw_forward=values[14],
            jaw_left=values[15],
            jaw_right=values[16],
            jaw_open=values[17],
            # Mouth
            mouth_close=values[18],
            mouth_funnel=values[19],
            mouth_pucker=values[20],
            mouth_left=values[21],
            mouth_right=values[22],
            mouth_smile_left=values[23],
            mouth_smile_right=values[24],
            mouth_frown_left=values[25],
            mouth_frown_right=values[26],
            mouth_dimple_left=values[27],
            mouth_dimple_right=values[28],
            mouth_stretch_left=values[29],
            mouth_stretch_right=values[30],
            mouth_roll_lower=values[31],
            mouth_roll_upper=values[32],
            mouth_shrug_lower=values[33],
            mouth_shrug_upper=values[34],
            mouth_press_left=values[35],
            mouth_press_right=values[36],
            mouth_lower_down_left=values[37],
            mouth_lower_down_right=values[38],
            mouth_upper_up_left=values[39],
            mouth_upper_up_right=values[40],
            # Brows
            brow_down_left=values[41],
            brow_down_right=values[42],
            brow_inner_up=values[43],
            brow_outer_up_left=values[44],
            brow_outer_up_right=values[45],
            # Cheeks
            cheek_puff=values[46],
            cheek_squint_left=values[47],
            cheek_squint_right=values[48],
            # Nose
            nose_sneer_left=values[49],
            nose_sneer_right=values[50],
            # Tongue
            tongue_out=values[51],
            # Head rotation (Live Link Face extended)
            head_yaw=values[52],
            head_pitch=values[53],
            head_roll=values[54],
            # Eye rotation (Live Link Face extended)
            left_eye_yaw=values[55],
            left_eye_pitch=values[56],
            left_eye_roll=values[57],
            right_eye_yaw=values[58],
            right_eye_pitch=values[59],
            right_eye_roll=values[60],
            # Metadata
            timestamp=timestamp,
            subject_name=subject_name,
        )

    def to_dict(self) -> Dict[str, float]:
        """Convert to dictionary with blend shape names as keys."""
        return {
            "EyeBlinkLeft": self.eye_blink_left,
            "EyeLookDownLeft": self.eye_look_down_left,
            "EyeLookInLeft": self.eye_look_in_left,
            "EyeLookOutLeft": self.eye_look_out_left,
            "EyeLookUpLeft": self.eye_look_up_left,
            "EyeSquintLeft": self.eye_squint_left,
            "EyeWideLeft": self.eye_wide_left,
            "EyeBlinkRight": self.eye_blink_right,
            "EyeLookDownRight": self.eye_look_down_right,
            "EyeLookInRight": self.eye_look_in_right,
            "EyeLookOutRight": self.eye_look_out_right,
            "EyeLookUpRight": self.eye_look_up_right,
            "EyeSquintRight": self.eye_squint_right,
            "EyeWideRight": self.eye_wide_right,
            "JawForward": self.jaw_forward,
            "JawLeft": self.jaw_left,
            "JawRight": self.jaw_right,
            "JawOpen": self.jaw_open,
            "MouthClose": self.mouth_close,
            "MouthFunnel": self.mouth_funnel,
            "MouthPucker": self.mouth_pucker,
            "MouthLeft": self.mouth_left,
            "MouthRight": self.mouth_right,
            "MouthSmileLeft": self.mouth_smile_left,
            "MouthSmileRight": self.mouth_smile_right,
            "MouthFrownLeft": self.mouth_frown_left,
            "MouthFrownRight": self.mouth_frown_right,
            "MouthDimpleLeft": self.mouth_dimple_left,
            "MouthDimpleRight": self.mouth_dimple_right,
            "MouthStretchLeft": self.mouth_stretch_left,
            "MouthStretchRight": self.mouth_stretch_right,
            "MouthRollLower": self.mouth_roll_lower,
            "MouthRollUpper": self.mouth_roll_upper,
            "MouthShrugLower": self.mouth_shrug_lower,
            "MouthShrugUpper": self.mouth_shrug_upper,
            "MouthPressLeft": self.mouth_press_left,
            "MouthPressRight": self.mouth_press_right,
            "MouthLowerDownLeft": self.mouth_lower_down_left,
            "MouthLowerDownRight": self.mouth_lower_down_right,
            "MouthUpperUpLeft": self.mouth_upper_up_left,
            "MouthUpperUpRight": self.mouth_upper_up_right,
            "BrowDownLeft": self.brow_down_left,
            "BrowDownRight": self.brow_down_right,
            "BrowInnerUp": self.brow_inner_up,
            "BrowOuterUpLeft": self.brow_outer_up_left,
            "BrowOuterUpRight": self.brow_outer_up_right,
            "CheekPuff": self.cheek_puff,
            "CheekSquintLeft": self.cheek_squint_left,
            "CheekSquintRight": self.cheek_squint_right,
            "NoseSneerLeft": self.nose_sneer_left,
            "NoseSneerRight": self.nose_sneer_right,
            "TongueOut": self.tongue_out,
            # Head rotation
            "HeadYaw": self.head_yaw,
            "HeadPitch": self.head_pitch,
            "HeadRoll": self.head_roll,
            # Eye rotation
            "LeftEyeYaw": self.left_eye_yaw,
            "LeftEyePitch": self.left_eye_pitch,
            "LeftEyeRoll": self.left_eye_roll,
            "RightEyeYaw": self.right_eye_yaw,
            "RightEyePitch": self.right_eye_pitch,
            "RightEyeRoll": self.right_eye_roll,
        }

    def get_value(self, name: str) -> float:
        """Get blend shape value by name."""
        return self.to_dict().get(name, 0.0)

    # Computed properties for common use cases

    @property
    def eye_look_horizontal_left(self) -> float:
        """Left eye horizontal look: -1 (left) to 1 (right)."""
        return self.eye_look_out_left - self.eye_look_in_left

    @property
    def eye_look_horizontal_right(self) -> float:
        """Right eye horizontal look: -1 (left) to 1 (right)."""
        return self.eye_look_in_right - self.eye_look_out_right

    @property
    def eye_look_vertical_left(self) -> float:
        """Left eye vertical look: -1 (down) to 1 (up)."""
        return self.eye_look_up_left - self.eye_look_down_left

    @property
    def eye_look_vertical_right(self) -> float:
        """Right eye vertical look: -1 (down) to 1 (up)."""
        return self.eye_look_up_right - self.eye_look_down_right

    @property
    def brow_left(self) -> float:
        """Left brow position: -1 (down) to 1 (up)."""
        return self.brow_outer_up_left - self.brow_down_left

    @property
    def brow_right(self) -> float:
        """Right brow position: -1 (down) to 1 (up)."""
        return self.brow_outer_up_right - self.brow_down_right
