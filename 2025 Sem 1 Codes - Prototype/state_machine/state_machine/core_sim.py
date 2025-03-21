"""
Author: Thomas 

-------------------------------------------------------------------------------------------------------
DO NOT ATTEMPT TO MODIFY THESE CLASSES AND CODE AS IT IS IMPORTANT TO SETUP THE SIMULATION.
This file contains all the necessary setup for the simulation. 

Class TranSeqData: Contain all info on a transition sequence performed during simulation

Class SimSequence: Generate randomised sequences with randomised weightings and duration for each
                    transition sequence. 

Class DDRSMSim: ROS Node that performs the actual simulation sending off sequences to Stateclient
                using msg SimState and waits for Client Node to publish successful transition message
                TranSeq back. The entire publishing and receiving is timed to measure the time elapsed
                compared to the assigned duration of each transition.

-------------------------------------------------------------------------------------------------------
"""

import rclpy
from rclpy.node import Node
from state_define import *
#from state_machine.msg import StateData,SimState,TranSeq
import random
import time  

class TransSeqData:
    def __init__(self,seq_order,old_state,new_state,sig,duration,time_elapsed = (0,0)):
        self.seq = seq_order
        self.old_state = old_state
        self.new_state = new_state
        self.tran_sig = sig
        self.duration = duration
        self.start_time = time_elapsed[0]
        self.end_time = time_elapsed[1]
        self.elasped = self.end_time - self.start_time

class SimSequence:
    def __init__(self):
        # Get all states
        self.all_states = StateEnum.get_all_states()
        self.state_nums = [state.num for state in self.all_states]
        ''' 
        Assign base weights for preferred states
        ALL weightings for states with AES hav weightings from 6-12
        ALL weightings for other state range from 1 -5
        Assign randomised weights 6-12 for AES Entry states
        '''
        
        # Randomise weights for AES Entry states
        aes_entry_states = {num: random.randint(6, 12) for num in [4, 5, 8, 13]}
        
        # Assign random weights for other states
        other_states = {num: random.randint(1, 5) for num in self.state_nums if num not in aes_entry_states}

        # Merge both into base_weights
        self.base_weights = {**aes_entry_states, **other_states}

        # Generate state transition sequence
        self.sequence, self.signals, self.durations, self.total_time = self.generate_sequence()
        self.trans_seq = self.trans_seq_reorg()

    def generate_weighted_list(self):
        """Generate a weighted list of states with some randomised variation."""
        weighted_choices = []

        for state in self.all_states:
            base_weight = self.base_weights[state.num]  
            weighted_choices.extend([state] * base_weight)  

        return weighted_choices
    
    def trans_seq_reorg(self):
        '''
        using the generate sequence, create a list of 
        '''
        sm_seq_list = self.get_sequence()

        list_sq_trans = []

        for i in range(len(sm_seq_list)-1):
            new_tr_data = TransSeqData(seq_order=i+1,
                                        old_state=sm_seq_list[i][0],
                                       new_state=sm_seq_list[i+1][0],
                                       sig=sm_seq_list[i+1][1],
                                       duration=sm_seq_list[i+1][2])

            list_sq_trans.append(new_tr_data)

        return list_sq_trans

    def generate_sequence(self):
        """
        Generate a random sequence of state transitions with durations and signals.
        Duration randomised between 1- 5 seconds, No fractional durations.
        MAXIMUM Total duration of the sequence is 120 secconds
        IDLE is always the INITIAL STATE.
        """
        # Get all weightings for the states
        weighted_states = self.generate_weighted_list() 

        # Start with SLEEP
        sequence = [StateEnum.IDLE]  
        signals = [TRANSITION_SIG.NORM_OP]  
        durations = [0]  

        # Begin generate sequence with their transition signals and durations
        total_time = durations[0]
        while total_time < MAX_SIM_TIME:
            # Select next state with weightings
            selected_state = random.choice(weighted_states)

            # Determine valid transition signals based on next state's valid signals
            valid_signals = selected_state.valid_sig
            transition_signal = random.choice(valid_signals)

            # SOS State can only be requested by transition signal 4
            if selected_state.name == "SOS":
                transition_signal =  TRANSITION_SIG.EMERGENCY
 
            # Random duration (1s to 6s)
            duration = random.randint(1, 6)  

            if selected_state.name == "SOS":
                duration = 0

            # Ensure we don't exceed 120s
            if total_time + duration > MAX_SIM_TIME:
                break  # Adjust to fit exactly

            # update the sequence
            sequence.append(selected_state)
            signals.append(transition_signal)
            durations.append(duration)
            total_time += duration

        return sequence, signals, durations, total_time

    def get_sequence(self):
        """
        Return the generated state sequence with transition signals and durations.
        """
        return [(state.name, signal, duration) for state, signal, duration in zip(self.sequence, self.signals, self.durations)]


# Example usage
if __name__ == "__main__":
    sim_seq = SimSequence()
    print("Generated State Transition Sequence:")
    for state, signal,duration in sim_seq.get_sequence():
        print(f"State: {state}, Signal {signal} ,Duration: {duration}s")
    print(f"Total Duration: {sim_seq.total_time}s")
    
    print(len(sim_seq.get_sequence()),len(sim_seq.trans_seq_reorg()))


# class DDRSMSim(Node):
#     def __init__(self):
#         super().__init__('DDR_node')
#         # Create timer for sim time.
#         self.timer = self.create_timer(float(MAX_ALLOWABLE_RUNTIME), self.shutdown_node)

#         # SIMULATION SEQUENCE 
#         sim_seq = SimSequence()
#         self.state_seq,self.sig_seq,self.time_seq = sim_seq.generate_sequence()
#         self.state_index = 0
#         self.current_state = self.state_seq[self.state_index]
        
#         # State publisher
#         self.pub_ = self.create_publisher(SimState, 'SimState', QUE_SIZE)
#         self.sub_ = self.create_subscription(TranSeq,'report_state',self.client_report,QUE_SIZE)

#         '''
#         Record actual time elaspe for each transition
#         '''
#         self.recorded_seq = []


#         pass

#     def make_state(self):
#         """
#         Funciton crafts SimMsg for publisher to publish message to client
#         Invoked within client_report() upon receiving sucessful transitions 
#         from state client 
#         """
#         new_state = StateEnum.get_state_by_name(self.state_seq[self.state_index])
#         next_state_msg = SimState()
#         if not (self.state_index > len(self.state_seq)):
#         # Get actual information of new state using the state name
#             next_state_msg.name = new_state.name
#             next_state_msg.num =  new_state.num
#             next_state_msg.sig =  self.sig_seq[self.state_index]
#             next_state_msg.duration = self.time_seq[self.state_index]

#             # update counter
#             self.state_index += 1
        
#         else:
#             next_state_msg = None

#         return next_state_msg

#     def client_report(self,RxNewState):
#         '''
#         Call back function updates successful state transition. 
#         Invoke log transition to log state transitions for evaluation
#         Send out the next state sequence
#         '''
#         # Craft next state 
#         next_state_msg = self.make_state()

#         # Check if seqeunce has been completed
#         if not (next_state_msg == None):
#             self.pub_.publish(next_state_msg)

#         else:
#             self.get_logger().info("Simulation run completed. Await for evaluation...")

#     def log_transition(self):
#         pass

#     def shutdown_node(self):
#         self.get_logger().info("Simulated sequence completed - Shutting down node.")
#         rclpy.shutdown()

# def main(args=None):
#     rclpy.init(args=args)
#     node = DDRSMSim()
#     rclpy.spin(node)

# if __name__ == '__main__':
#     main()
    

    
