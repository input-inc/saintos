# ArmState.msg
# Current arm position and status

builtin_interfaces/Time timestamp

string arm_id               # "left" or "right"
float32[] joint_positions   # Array of joint angles
float32[] joint_velocities  # Array of joint velocities
float32[] joint_efforts     # Array of joint efforts/torques

bool in_motion              # Currently moving
bool error_state            # Error condition
string error_message        # Error description if error_state
