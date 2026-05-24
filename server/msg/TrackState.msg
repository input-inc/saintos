# TrackState.msg
# Current track drive state

builtin_interfaces/Time timestamp

float32 left_velocity       # Left track velocity
float32 right_velocity      # Right track velocity
float32 left_current        # Left motor current
float32 right_current       # Right motor current

bool estop_active           # Emergency stop active
bool motors_enabled         # Motors are enabled

nav_msgs/Odometry odometry  # Current odometry
