import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from realsense2_camera_msgs.msg import RGBD
import os
import open3d as o3d
import time
from geometry_msgs.msg import TransformStamped
import tf2_ros

class Find(Node):
    def __init__(self):
        super().__init__('Find_table_location')
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        #self.subscription = self.create_subscription(RGBD, '/camera/camera/rgbd', self.listener_callback, 10)
        self.timer = self.create_timer(1.0, self.find_table)  # Timer to save images every 1 second
        self.flag = True

    def find_table(self):
        if self.flag == True:
            try:
                trans = self.tf_buffer.lookup_transform('map', 'marker', rclpy.time.Time())
                tx,ty,tz = trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z
                self.x_table = tx   
                self.y_table = ty   
                self.z_table = tz
                with open('/home/realsense/test_ws/src/calibration_tool/calibration_tool/info/table_location.txt', 'w') as f:
                    f.write(f"{tx},{ty},{tz}\n")
                self.get_logger().info(f'Table location was found.')
                self.get_logger().info(f'x:{tx}. y:{ty}, z:{tz}')
                self.flag = False

            except Exception as e:
                self.get_logger().info(f'Error: {e}')

            """
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'camera_color_optical_frame'
            transform.child_frame_id = 'marker'

            # Set translation
            transform.transform.translation.x = self.x_camera
            transform.transform.translation.y = -self.y_camera
            transform.transform.translation.z = -self.z_camera

            # Set rotation (in radians)
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 1.0

            # Publish the transform
            self.tf_broadcaster.sendTransform(transform)
            """



rclpy.init(args=None)
rclpy.spin(Find())
rclpy.shutdown()
