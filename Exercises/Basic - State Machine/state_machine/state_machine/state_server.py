import rclpy
from rclpy.node import Node
from sm_srv_msg.srv import StateData  
from sm_srv_msg.msg import StateTrans 
from state_machine.state_define import *

class State_server(Node):

    def __init__(self):
        super().__init__('state_machine_server')
        #TODO: define service server
        #      Define publishr and topics

    def state_machine(self, request, response):
        # response.sum = request.a + request.b
        # self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        pass

    def state_change_parse():
        # TODO: process state change transition here

        pass


def main():
    rclpy.init()

    SM_Server = State_server()

    rclpy.spin(SM_Server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()