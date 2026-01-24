# NodeAnnouncement.msg
# Announcement from an unadopted node

builtin_interfaces/Time timestamp

string node_id              # Unique identifier (Pi serial)
string mac_address          # Primary MAC address
string hardware_model       # e.g., "Raspberry Pi 4 Model B"
uint32 hardware_ram_mb      # RAM in megabytes
string firmware_version     # Current firmware version
string ip_address           # Current IP address

float32 cpu_temp            # CPU temperature in Celsius
float32 cpu_usage           # CPU usage percentage
float32 memory_usage        # Memory usage percentage
uint32 uptime_seconds       # Seconds since boot
