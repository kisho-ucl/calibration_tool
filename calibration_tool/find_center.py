import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from realsense2_camera_msgs.msg import RGBD
import os
import open3d as o3d

class RsSub(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(RGBD, '/camera/camera/rgbd', self.listener_callback, 10)
        self.timer = self.create_timer(1.0, self.save_images)  # Timer to save images every 1 second
        self.rgb_image = None
        self.depth_image = None
        self.counter = 0
        self.save_path = './data/'
        self.fxy = 1.0  # Scaling factor, set as needed
        self.depth_range_max = 5000  # Maximum depth range for truncation

        # Create directory if it does not exist
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)

    def listener_callback(self, msg):
        #self.rgb_image = self.bridge.imgmsg_to_cv2(msg.rgb).astype(np.uint8)
        self.rgb_image = cv2.cvtColor(CvBridge().imgmsg_to_cv2(msg.rgb), cv2.COLOR_BGR2RGB).astype(np.uint8)
        self.depth_image = self.bridge.imgmsg_to_cv2(msg.depth, "passthrough")
        
        self.rgb_image = cv2.resize(self.rgb_image, dsize=None, fx=self.fxy, fy=self.fxy, interpolation=cv2.INTER_NEAREST)
        self.depth_image = cv2.resize(self.depth_image, dsize=None, fx=self.fxy, fy=self.fxy, interpolation=cv2.INTER_NEAREST)

        fx_d = msg.rgb_camera_info.k[0] * self.fxy
        fy_d = msg.rgb_camera_info.k[4] * self.fxy
        cx_d = msg.rgb_camera_info.k[2] * self.fxy
        cy_d = msg.rgb_camera_info.k[5] * self.fxy
        height, width = self.depth_image.shape
        
        self.intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width, height, fx_d, fy_d, cx_d, cy_d
        )
        
        cv2.imshow("RGB Image", self.rgb_image)
        cv2.imshow("Depth Image", self.depth_image)
        cv2.waitKey(1)

    def save_images(self):
        if self.rgb_image is not None and self.depth_image is not None:
            rgb_filename = os.path.join(self.save_path, f'rgb_{self.counter}.png')
            depth_filename = os.path.join(self.save_path, f'depth_{self.counter}.png')

            cv2.imwrite(rgb_filename, self.rgb_image)
            cv2.imwrite(depth_filename, self.depth_image)
            
            self.get_logger().info(f'Saved images {rgb_filename} and {depth_filename}')
            self.counter += 1

rclpy.init(args=None)
rclpy.spin(RsSub())
rclpy.shutdown()
