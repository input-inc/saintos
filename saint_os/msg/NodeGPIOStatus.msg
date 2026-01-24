# NodeGPIOStatus.msg
# Full GPIO status for a node

builtin_interfaces/Time timestamp

string node_id              # Unique identifier
string hardware_rev         # Hardware revision
string firmware_version     # Current firmware version

GPIOPin[] pins              # Array of GPIO pin states
PeripheralInfo[] peripherals # Detected peripherals

float32 cpu_temp            # CPU temperature in Celsius
float32 cpu_usage           # CPU usage percentage
float32 memory_usage        # Memory usage percentage
uint32 uptime_seconds       # Seconds since boot
