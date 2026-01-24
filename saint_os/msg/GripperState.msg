# GripperState.msg
# Current gripper state

builtin_interfaces/Time timestamp

string arm_id               # "left" or "right"
float32 position            # 0.0 (closed) to 1.0 (open)
float32 force               # Current grip force
bool object_detected        # Object detected in gripper
bool in_motion              # Currently moving
