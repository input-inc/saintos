# ArmCommand.msg
# Arm movement command

string arm_id               # "left" or "right"
float32[] target_positions  # Target joint positions
float32 speed_factor        # Movement speed (0.0 to 1.0)
bool immediate              # Override current motion
