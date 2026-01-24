# GPIOPin.msg
# Individual GPIO pin state

uint8 pin_number            # BCM pin number
string pin_name             # BCM name (e.g., "GPIO17")
string direction            # "input", "output", "pwm", "i2c", "spi", "uart"
string current_state        # "high", "low", "pwm_50", etc.
bool in_use                 # Currently claimed by a driver
string used_by              # Driver name if in_use
