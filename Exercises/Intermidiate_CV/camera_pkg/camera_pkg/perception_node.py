#!/usr/bin/env python3
"""
Camera node - main processing file for ROS 2 camera node

Authors: Thomas, Alvin
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from camera_pkg.CCompVis import CCompVis

class CCamera(Node):
    def __init__(self, rate=50):
        super().__init__('camera_node')

        self.get_logger().info('CameraNode started')

        self.rate = rate
        self.bridge = CvBridge()

        # Initialize subscribers
        self.sub_image = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.vision_callback,
            10
        )

        self.sub_lidar = self.create_subscription(LaserScan,
            '/scan',
            self.get_lidar_in_pov,
            10
        )


        # Set camera feed window name
        self.window_feed_name = self.get_name()

        # Store raw image frame
        self.image = None

        # CompVision module for computer vision processing
        self.comp_vis = CCompVis()

        # All lidar measurements under 1.5m from 140 degrees to 220 degrees.
        self.back_fov = []

        # Projected lidar pixels and their associated ranges in x, y (m)
        self.projected_lidar = []
        self.projected_range = []

    def get_lidar_in_pov(self, lidar_msg):
        '''
        Callback to /scan to get lidar scans from LDS.
        Filters points between 300° (right rear) and 60° (left front).
        '''
        # Convert desired angles to radians
        min_angle = np.deg2rad(270)  # Equivalent to -60° or 5.24 rad
        max_angle = np.deg2rad(90)   # Equivalent to 60° or 1.05 rad

        increment = lidar_msg.angle_increment
        bearing = lidar_msg.angle_min  # Starting angle of the scan
        back_view = []

        # Adjust angles for wrap-around (i.e., crossing 0°)
        for i in range(len(lidar_msg.ranges)):
            # Handle wrap-around: Consider the segment from 300° to 360° and from 0° to 60°
            # Convert angles to a positive range by adding 2π where necessary
            adjusted_bearing = bearing if bearing >= 0 else bearing + 2 * math.pi

            if (adjusted_bearing >= min_angle or adjusted_bearing <= max_angle):
                if lidar_msg.ranges[i] <= 1.5:  # Filter ranges within 1.5 meters
                    x = lidar_msg.ranges[i] * math.sin(bearing)
                    y = lidar_msg.ranges[i] * math.cos(bearing)
                    back_view.append((x, y))

            # Increment bearing angle
            bearing += increment

        # Store the filtered back FOV data
        self.back_fov = back_view

    def project_lidar_to_cam(self):
        '''
        Project the lidar points onto the camera frame using intrinsics and extrinsics.
        '''
        K = np.array([
            [494.714, 0, 307.381],
            [0, 492.364, 226.292],
            [0, 0, 1]
        ])
        K = np.hstack((K, np.array([[0], [0], [0]])))

        # Physical parameters from A2
        theta = 0.004
        x = 0.0026
        y = 0.0095

        R = np.array([
            [math.cos(theta), math.sin(theta), 0],
            [math.sin(theta), -math.cos(theta), 0],
            [0, 0, 1]
        ])

        translation = np.array([[x], [y], [0]])
        extrinsic = np.hstack((R, translation))
        extrinsic = np.vstack((extrinsic, np.array([[0, 0, 0, 1]])))

        self.projected_lidar = []
        self.projected_range = []

        for l in self.back_fov:
            lidar = np.array([l[0], 0.0, -l[1], 1.0])
            lidar_to_cam_frame = extrinsic @ lidar.T

            if lidar_to_cam_frame[2] != 0.0:
                u = ((K[0][0] * lidar_to_cam_frame[0]) / lidar_to_cam_frame[2]) + K[0][2]
                v = ((K[1][1] * lidar_to_cam_frame[1]) / lidar_to_cam_frame[2]) + K[1][2]

                pixel = (int(u), int(v))
                self.projected_lidar.append(pixel)
                self.projected_range.append((l[0], -l[1]))

                cv2.circle(self.image, pixel, 3, (255, 255, 255), -1)

    def associate_range(self, bounding_box):
        '''
        Output x, y lidar ranges that lie within the bounding box of detected objects.
        '''
        x, y = [], []

        for i in range(len(self.projected_lidar)):
            within_x = bounding_box[0][0] <= self.projected_lidar[i][0] <= bounding_box[0][1]
            within_y = bounding_box[1][0] <= self.projected_lidar[i][1] <= bounding_box[1][1]

            if within_x and within_y:
                x.append(self.projected_range[i][0])
                y.append(self.projected_range[i][1])

        return x, y

    def vision_callback(self, ros_image):
        '''
        Camera topic callback where main computer vision processing is performed.
        '''
        try:
            # array = np.frombuffer(ros_image.data, np.uint8)
            # self.image = cv2.imdecode(array, cv2.IMREAD_COLOR)
            # Convert the ROS image message to OpenCV format
            self.image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
            self.comp_vis.mImage = self.image

            # TODO: Perform Cylinder detection here with self.image

            # TODO: Printout which cylinders are detected in current frame

            # TODO: EXTENSION: USing projected lidar range, acquire relative ranges of the cylinders LIVE

        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        self.project_lidar_to_cam()
        cv2.imshow(self.window_feed_name, self.image)
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
