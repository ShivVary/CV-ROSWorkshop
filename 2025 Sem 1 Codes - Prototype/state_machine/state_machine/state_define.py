'''
Author: Thomas
This file defines the state enumeration and state transitions.
Use this file for convenience so you don't have to define your own state classes.
'''

from dataclasses import dataclass
from typing import List, Optional


MAX_SIM_TIME = 120
MAX_ALLOWABLE_RUNTIME = 130
QUE_SIZE = 15

# Class of transition signals with its members being unmodifiable
@dataclass(frozen=True)
class TRANSITION_SIG:
    """Class representing possible transition signals."""
    NORM_OP     = 1
    RES_LOW     = 2
    RE_EVAL     = 3
    EMERGENCY   = 4

# Data class representing a state
@dataclass(frozen=True)
class State:
    """Class representing a state in a state machine."""
    name: str               # State name (e.g., "POWER_OFF")
    num: int                # Numerical identifier for the state
    valid_sig: List[int]    # List of valid transition signals

# Metaclass to prevent modification of state attributes
class ImmutableMeta(type):
    """Metaclass to prevent modification of class attributes."""
    def __setattr__(cls, key, value):
        if key in cls.__dict__:  # Prevent modification of existing attributes
            raise AttributeError(f"Cannot modify immutable state: {key}")
        
        super().__setattr__(key, value)

# State enumeration class
class StateEnum(metaclass=ImmutableMeta):
    """Class representing all possible states."""
    ts = TRANSITION_SIG

    # Define states with valid transition signals
    IDLE             = State("idle", 1, [ts.NORM_OP])
    POWER_OFF        = State("power_off", 2, [ts.NORM_OP])
    BRING_UP         = State("bring_up", 3, [ts.NORM_OP])
    RECHARGE_BATTERY = State("recharge_battery", 4, [ts.NORM_OP, ts.RES_LOW])
    REFILL_RESOURCE  = State("refill_resource", 5, [ts.NORM_OP, ts.RES_LOW])
    STOP             = State("stop", 6, [ts.NORM_OP])
    CAPTURE_VISUALS  = State("capture_visuals", 7, [ts.NORM_OP, ts.RE_EVAL])
    ASSESS_ENV       = State("assess_environment", 8, [ts.NORM_OP, ts.RE_EVAL])
    PLAN_ROUTE       = State("plan_route", 9, [ts.NORM_OP])
    DRIVE            = State("drive", 10, [ts.NORM_OP])
    SPRAY_WATER      = State("spray_water", 11, [ts.NORM_OP])
    PLACE_SAND_BAG   = State("place_sand_bag", 12, [ts.NORM_OP])
    UPDATE_MAP       = State("update_map", 13, [ts.NORM_OP, ts.RE_EVAL])  
    SOS              = State("SOS", 14, [ts.EMERGENCY])

    #DEFINE ALL AES SEQUENCES HERE
    RFR = (PLAN_ROUTE,DRIVE)
    RRB =( PLAN_ROUTE,DRIVE)

    OPM = (CAPTURE_VISUALS,UPDATE_MAP,PLAN_ROUTE)
    SCM = (STOP,CAPTURE_VISUALS,UPDATE_MAP,PLAN_ROUTE)

    RIU = (ASSESS_ENV,PLAN_ROUTE)
    FDSU = (STOP,ASSESS_ENV,PLAN_ROUTE)

    @classmethod
    def get_all_states(cls):
        """Returns all defined states as a list."""
        return [value for key, value in cls.__dict__.items() if isinstance(value, State)]

    @classmethod
    def is_valid_state(cls, state_name: str) -> bool:
        """Check if a state with the given name exists."""
        return any(state.name == state_name for state in cls.get_all_states())

    @classmethod
    def get_state_by_name(cls, state_name: str) -> Optional[State]:
        """Retrieve a state object by its name. Returns None if not found."""
        for state in cls.get_all_states():
            if state.name == state_name:
                return state
            
        print(f"Warning: State '{state_name}' not found.")
        return None

    @classmethod
    def get_state_by_num(cls, state_num: int) -> Optional[State]:
        """Retrieve a state object by its numerical identifier. Returns None if not found."""
        for state in cls.get_all_states():
            if state.num == state_num:
                return state
            
        print(f"Warning: State with number '{state_num}' not found.")
        return None

# Example usage
if __name__ == "__main__":
    # List all defined states
    for state in StateEnum.get_all_states():
        print(f"{state.name} (Num: {state.num}) - Valid transitions: {state.valid_sig}")

    # # Retrieve state by number
    # state = State_Enum.get_state_by_num(1)
    # if state:
    #     print(f"\nRetrieved state: {state.name} - Valid transitions: {state.valid_sig}")
    
    # # Retrieve state by name
    # state = State_Enum.get_state_by_name("stop")
    # if state:
    #     print(f"\nRetrieved state: {state.name} - Valid transitions: {state.valid_sig}")
    
    # # Attempt to modify a state (this will raise an error)
    # try:
    #     State_Enum.POWER_OFF = State("stop", 99, [])
    # except AttributeError as e:
    #     print(e)
