import rclpy
from rclpy.node import Node
from state_machine.srv import PowerCont 
from state_machine.msg import StateData 
from state_machine.action import StateTrans
from state_define import *

class StateServer(Node):
    def __init__(self):
        super().__init__('state_machine_server')
        #TODO: define Action And Serice server
        #      Define publisher and topics

    def sm_action(self, request, response):

        pass

    def sm_service(self, request, response):

        pass

    def state_transition(self):
        pass


def main():
    rclpy.init()

    SM_Server = StateServer()

    rclpy.spin(SM_Server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()


