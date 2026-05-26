# ConsoleInput.msg
# User input from console

builtin_interfaces/Time timestamp

string input_type           # "button", "encoder", "touch"
string input_id             # Button/encoder identifier
float32 value               # Input value
bool pressed                # For buttons: currently pressed
