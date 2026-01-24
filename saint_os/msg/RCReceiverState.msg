# RCReceiverState.msg
# RC receiver status

builtin_interfaces/Time timestamp

string protocol             # "pwm", "ppm", "sbus", "ibus", "crsf"
bool enabled                # Receiver enabled
bool connected              # Receiver communication active
bool failsafe               # Transmitter signal lost
uint8 channel_count         # Number of active channels
RCChannelState[] channels   # All channel states
float32 rssi                # Signal strength (if available, dBm)
uint32 frame_rate_hz        # Update rate
