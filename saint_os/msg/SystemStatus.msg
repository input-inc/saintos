# SystemStatus.msg
# Overall system status message

builtin_interfaces/Time timestamp

string server_name
string server_version
bool server_online

uint32 uptime_seconds
float32 cpu_usage
float32 memory_usage

uint8 adopted_node_count
uint8 unadopted_node_count
uint8 online_node_count

bool livelink_enabled
uint8 livelink_source_count

bool rc_enabled
bool rc_connected

uint8 websocket_client_count
