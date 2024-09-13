from sm_srv_msg.srv import StateData  
from sm_srv_msg.msg import StateTrans 

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from state_machine.state_define import *

# Template subscriber node 
class State_Sub(Node):

    def __init__(self):
        super().__init__('state_sub')

    def state_callback(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)

    state_storage = State_Sub()

    rclpy.spin(state_storage)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    state_storage.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()