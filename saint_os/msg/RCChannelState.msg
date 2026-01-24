# RCChannelState.msg
# Individual RC channel state

uint8 channel_number        # 1-16
string channel_name         # User-assigned name
uint16 raw_value            # Raw receiver value (typically 1000-2000 or 0-2047)
float32 normalized          # -1.0 to 1.0 (centered) or 0.0 to 1.0
bool failsafe               # True if channel in failsafe
builtin_interfaces/Time last_update
