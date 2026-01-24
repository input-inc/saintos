# GripperCommand.msg
# Gripper command

string arm_id               # "left" or "right"
float32 position            # 0.0 (closed) to 1.0 (open)
float32 force               # Maximum grip force
