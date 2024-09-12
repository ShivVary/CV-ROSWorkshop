import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class UserInputPublisher(Node):
    def __init__(self):
        super().__init__('user_input_publisher')
        self.publisher_ = self.create_publisher(String, 'user_input_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback) # Timer to

        self.user_input = ""
    def timer_callback(self):
        self.user_input = input("Enter a message to publish: ") # Taking input
        msg = String()
        msg.data = self.user_input
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = UserInputPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Node stopped cleanly.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
