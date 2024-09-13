'''
Author: Thomas 
This file defines the state enumeration and state transitions.
Use this file for convenience so you dont have to define your own state classes

Brief overview of classes:
    1. TRANSITION_SIG: Constant class of transition signals 
    2. State: class represents a state
    3. State_Enum: class that contains all instantiations of states with their 
                    valid transition signals as its members. 
'''

from dataclasses import dataclass
from typing import List

# Class of transition signals with its members being unmodifiable
# DO NOT MODIFY THIS CLASS
@dataclass(frozen=True)
class TRANSITION_SIG:
    """Class representing possible transition signals."""
    NORM_OP     = 1
    RES_LOW     = 2
    RE_EVAL     = 3
    EMERGENCY   = 4

# Data class represent a state with name, integers and valid transition signals
@dataclass
class State:
    """Class representing a state in a state machine."""
    name: str               # Name of the state (e.g., "POWER_OFF")
    num: int                # Numerical identifier for the state
    valid_sig: List[int]    # List of integers representing valid transition signals

# Class for preventing modifiying class attributes
class ImmutableMeta(type):
    """Metaclass to prevent modification of class attributes."""
    def __setattr__(cls, key, value):
        if key in cls._immutable_states:
            raise AttributeError(f"Cannot modify immutable state: {key}")
        super().__setattr__(key, value)

# Enum calss for all states and their defined valid signals
# You can add new class methods here but do not change the members values
class State_Enum(metaclass=ImmutableMeta):
    """Class representing all possible states."""
    ts = TRANSITION_SIG

    # Define states as class attributes with correct signal references
    POWER_OFF               = State("power_off", 1, [ts.NORM_OP])
    POWER_ON                = State("power_on", 2, [ts.NORM_OP])
    BRING_UP                = State("bring_up", 3, [ts.NORM_OP])
    RECHARGE_BATTERY        = State("recharge_battery", 4, [ts.NORM_OP, ts.RES_LOW])
    REFILL_RESOURCE         = State("refill_resource", 5, [ts.NORM_OP, ts.RES_LOW])
    STOP                    = State("stop", 6, [ts.NORM_OP, ts.RE_EVAL, ts.EMERGENCY])
    LIDAR_SCAN              = State("lidar_scan", 7, [ts.NORM_OP, ts.RE_EVAL, ts.EMERGENCY])
    CAPTURE_VISUALS         = State("capture_visuals", 8, [ts.NORM_OP, ts.RE_EVAL])
    ASSESS_ENV              = State("assess_env", 9, [ts.NORM_OP, ts.RE_EVAL])
    UPDATE_MAP              = State("update_map", 10, [ts.NORM_OP, ts.RE_EVAL])
    PLAN_ROUTE              = State("plan_route", 11, [ts.NORM_OP])
    DRIVE                   = State("drive", 12, [ts.NORM_OP])
    SPRAY                   = State("spray", 13, [ts.NORM_OP])
    PLACE_BAG               = State("place_bag", 14, [ts.NORM_OP])
    SOS                     = State("SOS", 15, [ts.NORM_OP, ts.EMERGENCY])

    # Prevent modification of class attributes
    _immutable_states = {
        'POWER_OFF', 'POWER_ON', 'BRING_UP', 'RECHARGE_BATTERY',
        'REFILL_RESOURCE', 'STOP', 'LIDAR_SCAN', 'CAPTURE_VISUALS',
        'ASSESS_ENV', 'UPDATE_MAP', 'PLAN_ROUTE', 'DRIVE',
        'SPRAY', 'PLACE_BAG', 'SOS'
    }

    @classmethod
    def is_valid_state(cls, state_name: str) -> bool:
        """Check if a state with the given name exists."""
        return any(state.name == state_name for state in cls.get_all_states())

    
    @classmethod
    def get_all_states(cls):
        """Returns all defined states as a list."""
        return [
            cls.POWER_OFF, cls.POWER_ON, cls.BRING_UP, cls.RECHARGE_BATTERY,
            cls.REFILL_RESOURCE, cls.STOP, cls.LIDAR_SCAN, cls.CAPTURE_VISUALS,
            cls.ASSESS_ENV, cls.UPDATE_MAP, cls.PLAN_ROUTE, cls.DRIVE,
            cls.SPRAY, cls.PLACE_BAG, cls.SOS
        ]

    @classmethod
    def is_valid_state(cls, state_name: str) -> bool:
        """Check if a state with the given name exists."""
        return any(state.name == state_name for state in cls.get_all_states())

    # Retrieve state by its name
    @classmethod
    def get_state_by_name(cls, state_name: str) -> State:
        """Retrieve a state object by its name."""
        for state in cls.get_all_states():
            if state.name == state_name:
                return state
        
        raise ValueError(f"State '{state_name}' is not defined in State_Enum.")

    # Retrieve state by its number
    @classmethod
    def get_state_by_num(cls, state_num: int) -> State:
        """Retrieve a state object by its numerical identifier."""
        for state in cls.get_all_states():
            if state.num == state_num:
                return state
        raise ValueError(f"State with number '{state_num}' is not defined in StateEnum.")
    
    # Example of adding a new class method
    # @classmethod
    # def add_new_method(cls):
    #     """A new class method added without altering existing state definitions."""
    #     print("This is a new class method!")

# Example usage
if __name__ == "__main__":
    # List all defined states
    for state in State_Enum.get_all_states():
        print(f"{state.name} (Num: {state.num}) - Valid transitions: {state.valid_sig}")

     # Example of retrieving state by number
    try:
        state = State_Enum.get_state_by_num(1)
        print(f"\nRetrieved state by number: {state.name} - Valid transitions: {state.valid_sig}")

    except ValueError as e:
        print(e)
    
    # Attempt to modify a state (this will raise an error)
    try:
        State_Enum.POWER_OFF = State("stop", 99, [])

    except AttributeError as e:
        print(e)

    # Example of getting states by name
    try:
        state = State_Enum.get_state_by_name("stop")
        print(f"\nRetrieved state: {state.name} - Valid transitions: {state.valid_sig}")

    except ValueError as e:
        print(e)
    