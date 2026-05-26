# HeadState.msg
# Current head position and status

builtin_interfaces/Time timestamp

float32 pan                 # -180 to 180 degrees
float32 tilt                # -90 to 90 degrees
float32 roll                # -45 to 45 degrees

bool camera_active          # Camera is streaming
bool in_motion              # Currently moving
bool error_state            # Error condition
string error_message        # Error description if error_state
