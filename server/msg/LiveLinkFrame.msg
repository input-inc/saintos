# LiveLinkFrame.msg
# Incoming LiveLink animation frame

builtin_interfaces/Time timestamp

string subject_name         # e.g., "FaceCapture", "UE_Character"
string subject_type         # "animation", "transform", "face", "camera"
uint64 frame_number         # Frame sequence number

string[] property_names     # BlendShape names or bone names
float32[] property_values   # Corresponding values

geometry_msgs/Transform transform  # For transform-based subjects
