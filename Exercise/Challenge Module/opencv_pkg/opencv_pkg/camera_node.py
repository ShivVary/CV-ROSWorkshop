import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge

class CCamera(Node):
    def __init__(self,rate=50):
        super().__init__('camera_node')
        self.get_logger().info('CameraNode started')
        self.bridge = CvBridge()

        # Sub to camera
        self.sub_image = self.create_subscription(Image,'/camera/image_raw',
            self.vision_callback,rate)
        
        # Sub to LiDAR
        self.sub_lidar = self.create_subscription(LaserScan,
            '/scan',
            self.lidar_callback,
            rate)

    def lidar_callback(self, lidar_msg):
        pass


    def vision_callback(self, ros_image):
        '''
        Camera topic callback 
        '''
        # Decode and display feed
        raw_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
        
        # Display row feed
        cv2.imshow("Feed", raw_image)
        cv2.waitKey(3)

    def on_shutdown(self):
        self.get_logger().info('Camera node shutting down')
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    camera_node = CCamera()
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.on_shutdown()
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()