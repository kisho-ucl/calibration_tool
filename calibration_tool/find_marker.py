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
import pyrealsense2 as rs

class Detector(Node):
    def __init__(self):
        super().__init__('Marker_detector')
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.subscription = self.create_subscription(RGBD, '/camera/camera/rgbd', self.listener_callback, 10)
        #self.camera_info_subscription = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)
        
        #self.timer = self.create_timer(1.0, self.publish_marker_tf)  # Timer to save images every 1 second
        self.rgb_image = None
        self.depth_image = None
        self.counter = 0
        self.save_path = './data/'
        self.fxy = 1.0  # Scaling factor, set as needed
        self.depth_range_max = 5000  # Maximum depth range for truncation
        self.flag_once = True
        self.flag_received_pose = False
        self.x_camera = 0.0
        self.y_camera = 0.0
        self.z_camera = 0.0
        self.camera_intrinsics =  o3d.camera.PinholeCameraIntrinsic(
            1280, 720, 908.600, 908.537, 643.698, 382.494
        )

        # Create directory if it does not exist
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)

    def listener_callback(self, msg):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'end_effector', rclpy.time.Time())
            self.flag_wait = True

        except Exception as e:
            self.get_logger().info(f'Error : robot pose not found')
            #time.sleep(3)
            self.flag_wait = False


        if self.flag_once == True and self.flag_wait == True: 
            rgb_image = self.bridge.imgmsg_to_cv2(msg.rgb).astype(np.uint8)
            rgb_image_cv2 = cv2.cvtColor(CvBridge().imgmsg_to_cv2(msg.rgb), cv2.COLOR_BGR2RGB).astype(np.uint8)
            depth_image = self.bridge.imgmsg_to_cv2(msg.depth, "passthrough")
            
            rgb_image = cv2.resize(rgb_image, dsize=None, fx=self.fxy, fy=self.fxy, interpolation=cv2.INTER_NEAREST)
            rgb_image_cv2 = cv2.resize(rgb_image_cv2, dsize=None, fx=self.fxy, fy=self.fxy, interpolation=cv2.INTER_NEAREST)
            depth_image = cv2.resize(depth_image, dsize=None, fx=self.fxy, fy=self.fxy, interpolation=cv2.INTER_NEAREST)
            #cv2.imshow("Depth Image", depth_image)
            #cv2.imshow("RGB Image", rgb_image_cv2)
            #cv2.waitKey(2000)
            #time.sleep(0.5)
            #cv2.destroyAllWindows()


            color = o3d.geometry.Image(rgb_image)
            depth = o3d.geometry.Image(depth_image)

            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                            color, depth, depth_trunc=5.0, convert_rgb_to_intensity=False)

            rgb_height, rgb_width, rgb_channels = rgb_image.shape
            depth_height, depth_width = depth_image.shape

               # Print the dimensions
            print(f"RGB Image - Width: {rgb_width}, Height: {rgb_height}, Channels: {rgb_channels}")

            time.sleep(1)

            #self.find_marker(rgbd)
            self.find_aruco(rgb_image, depth_image)
            self.publish_marker_tf()

    def find_aruco(self, rgb_image, depth_image):
        # ArUco Marker Detection
        aruco = cv2.aruco
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_image, dictionary)
        rs_camera_intrinsics = self.convert_intrinsics(self.camera_intrinsics)

        if len(corners) != 0:
            top_left_corner = corners[0][0][0]
            x, y = top_left_corner
            depth = depth_image[int(y), int(x)]
            if depth != 0:
                point3d = rs.rs2_deproject_pixel_to_point(rs_camera_intrinsics, [x, y], depth)
                self.x_camera = point3d[0] / 1000
                self.y_camera = point3d[1] / 1000
                self.z_camera = point3d[2] / 1000
                self.get_logger().info(f"Top-left corner 3D point: x={self.x_camera}, y={self.y_camera}, z={self.z_camera}")
                self.flag_once = False 
            """
            for corner in corners:
                for point in corner[0]:
                    x, y = point
                    depth = depth_image[int(y), int(x)]  # Get depth value at (x, y)
                    if depth != 0:
                        x_real, y_real, z_real = rs.rs2_deproject_pixel_to_point(rs_camera_intrinsics, [x, y], depth)
                        self.get_logger().info(f"Marker position: x={x_real}, y={y_real}, z={z_real}")
                        #self.get_logger().info('Publishing static transform from camera to marker')
            self.flag_once = False 
            """


        aruco.drawDetectedMarkers(rgb_image, corners, ids, (0,255,0))
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        rgb_image_cv2 = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        images = np.hstack((rgb_image_cv2, depth_colormap))
        cv2.imshow("Depth Image", depth_image)
        cv2.imshow("RGB Image", rgb_image_cv2)
        cv2.waitKey(2000)
        time.sleep(1.5)
        cv2.destroyAllWindows()
                        
    def convert_intrinsics(self, o3d_intrinsics):
        intrinsics = rs.intrinsics()
        intrinsics.width = o3d_intrinsics.width
        intrinsics.height = o3d_intrinsics.height
        intrinsics.fx = o3d_intrinsics.intrinsic_matrix[0, 0]
        intrinsics.fy = o3d_intrinsics.intrinsic_matrix[1, 1]
        intrinsics.ppx = o3d_intrinsics.intrinsic_matrix[0, 2]
        intrinsics.ppy = o3d_intrinsics.intrinsic_matrix[1, 2]
        intrinsics.model = rs.distortion.none
        intrinsics.coeffs = [0, 0, 0, 0, 0]  # Assuming no distortion
        return intrinsics

    def camera_info_callback(self, msg):
        # Extract intrinsic parameters from CameraInfo message
        if self.camera_intrinsics is None:
            self.camera_intrinsics = msg
            #self.get_logger.info('Camera intrinsics received.')

    def publish_marker_tf(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'camera_color_optical_frame'
        transform.child_frame_id = 'marker'

        # Set translation
        transform.transform.translation.x = self.x_camera
        transform.transform.translation.y = self.y_camera
        transform.transform.translation.z = self.z_camera

        # Set rotation (in radians)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        # Publish the transform
        self.tf_broadcaster.sendTransform(transform)
        #self.get_logger().info('Publishing static transform from camera to marker')
        
rclpy.init(args=None)
rclpy.spin(Detector())
rclpy.shutdown()
