# State.msg
# This message represents the data structure of a state in a state machine.

string name                # Name of the state (e.g., "POWER_OFF")
int32 num                  # Numerical identifier of the state
int32[] valid_sig          # List of integers representing valid transition signals
