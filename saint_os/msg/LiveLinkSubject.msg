# LiveLinkSubject.msg
# LiveLink subject information

string subject_name         # Subject identifier
string subject_type         # "animation", "transform", "face", "camera"
string source_ip            # Source IP address
float32 frame_rate          # Current frame rate
uint64 last_frame_number    # Last received frame
builtin_interfaces/Time last_update

string[] property_names     # Available properties
