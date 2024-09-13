import sys
import rclpy
from rclpy.node import Node

from sm_srv_msg.srv import StateData  
from sm_srv_msg.msg import StateTrans 
from state_machine.state_define import *
class State_client(Node):

    def __init__(self):
        super().__init__('sm_client')
        pass

    def send_request(self, new_state, tran_sig):
        pass


def main():
    rclpy.init()

    state_machine_client = State_client()
    new_state = state_machine_client.send_request(int(sys.argv[1]), int(sys.argv[2]))

    rclpy.spin_until_future_complete(state_machine_client, new_state)

    response = new_state.result()
 
    state_machine_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()