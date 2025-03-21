
import rclpy
from rclpy.node import Node
from state_machine.srv import PowerCont 
from state_machine.msg import StateData,SimState
from state_machine.action import StateTrans
from state_define import *

class StateClient(Node):

    def __init__(self):
        super().__init__('sm_client')

        self.pub_ = self.create_publisher(StateData, 'report_state', 10)
        self.sub_ = self.create_subscription(SimState,'SimState',self.client_report,10)

        pass

    def request_state_change(self, state_order):
        pass

    def sos(self):
        pass


def main():
    rclpy.init()

    state_machine_client = StateClient()

 
    state_machine_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()