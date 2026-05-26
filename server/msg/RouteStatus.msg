# RouteStatus.msg
# Current routing state

builtin_interfaces/Time timestamp

string route_id             # Route identifier
bool enabled                # Route is active
uint16 priority             # Route priority

string input_source         # "websocket", "livelink", "rc", "autonomous"
string input_subject        # Subject/channel identifier
string output_target        # Target node/property

float32 last_input_value    # Last input value received
float32 last_output_value   # Last output value sent
builtin_interfaces/Time last_activity
