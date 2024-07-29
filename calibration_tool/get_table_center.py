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

class RsSub(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        #self.tf_buffer = tf2_ros.Buffer()
        #self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.subscription = self.create_subscription(RGBD, '/camera/camera/rgbd', self.listener_callback, 10)
        self.timer = self.create_timer(1.0, self.publish_table_tf)  # Timer to save images every 1 second
        self.rgb_image = None
        self.depth_image = None
        self.counter = 0
        self.save_path = './data/'
        self.fxy = 1.0  # Scaling factor, set as needed
        self.depth_range_max = 5000  # Maximum depth range for truncation
        self.flag = True
        self.x_camera = 0.0
        self.y_camera = 0.0
        self.z_camera = 0.0

        # Create directory if it does not exist
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)

    def listener_callback(self, msg):
        if self.flag == True: 
            rgb_image = self.bridge.imgmsg_to_cv2(msg.rgb).astype(np.uint8)
            rgb_image_cv2 = cv2.cvtColor(CvBridge().imgmsg_to_cv2(msg.rgb), cv2.COLOR_BGR2RGB).astype(np.uint8)
            depth_image = self.bridge.imgmsg_to_cv2(msg.depth, "passthrough")
            
            rgb_image = cv2.resize(rgb_image, dsize=None, fx=self.fxy, fy=self.fxy, interpolation=cv2.INTER_NEAREST)
            rgb_image_cv2 = cv2.resize(rgb_image_cv2, dsize=None, fx=self.fxy, fy=self.fxy, interpolation=cv2.INTER_NEAREST)
            depth_image = cv2.resize(depth_image, dsize=None, fx=self.fxy, fy=self.fxy, interpolation=cv2.INTER_NEAREST)
            cv2.imshow("Depth Image", depth_image)
            cv2.imshow("RGB Image", rgb_image_cv2)
            cv2.waitKey(2000)
            time.sleep(0.5)
            cv2.destroyAllWindows()


            color = o3d.geometry.Image(rgb_image)
            depth = o3d.geometry.Image(depth_image)

            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                            color, depth, depth_trunc=5.0, convert_rgb_to_intensity=False)

            rgb_height, rgb_width, rgb_channels = rgb_image.shape
            depth_height, depth_width = depth_image.shape

               # Print the dimensions
            print(f"RGB Image - Width: {rgb_width}, Height: {rgb_height}, Channels: {rgb_channels}")

            self.find_marker(rgbd)

            self.flag = False


    def find_marker(self, rgbd):

        pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            1280, 720, 908.600, 908.537, 643.698, 382.494
        )

        point_cloud = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd, pinhole_camera_intrinsic
        )

        # ポイントクラウドの向きを調整（必要に応じて）
        point_cloud.transform([[1, 0, 0, 0],
                            [0, -1, 0, 0],
                            [0, 0, -1, 0],
                            [0, 0, 0, 1]])

        # ポイントクラウドを表示
        o3d.visualization.draw_geometries([point_cloud])
        points = np.asarray(point_cloud.points)
        colors = np.asarray(point_cloud.colors)

        lower_bound = np.array([0.4, 0.0, 0.0])  # 赤色の下限
        upper_bound = np.array([1.0, 0.2, 0.2])  # 赤色の上限

        

        # 色が範囲内のポイントをフィルタリング
        mask = np.all((colors >= lower_bound) & (colors <= upper_bound), axis=1)
        filtered_points = np.asarray(point_cloud.points)[mask]
        filtered_colors = colors[mask]

        print(f"Number of filtered points: {len(filtered_points)}")  # フィルタリング後のポイント数を表示

        # フィルタリングされたポイントクラウドを作成
        filtered_point_cloud = o3d.geometry.PointCloud()
        filtered_point_cloud.points = o3d.utility.Vector3dVector(filtered_points)
        filtered_point_cloud.colors = o3d.utility.Vector3dVector(filtered_colors)

        # フィルタリングされたポイントクラウドを表示
        #o3d.visualization.draw_geometries([filtered_point_cloud])


        #1000 mm = 1 m 以内の距離を設定
        distance_threshold = 1.0  # メートル単位
        distances = np.linalg.norm(points, axis=1)

        # 赤色と距離の条件を満たすマスクを作成
        color_mask = np.all((colors >= lower_bound) & (colors <= upper_bound), axis=1)
        distance_mask = distances <= distance_threshold
        combined_mask = color_mask & distance_mask

        # 条件を満たすポイントを抽出
        marker_points = points[combined_mask]
        marker_colors = colors[combined_mask]

        print(f"Number of filtered points: {len(marker_points)}")  # フィルタリング後のポイント数を表示

        # フィルタリングされたポイントクラウドを作成
        marker_point_cloud = o3d.geometry.PointCloud()
        marker_point_cloud.points = o3d.utility.Vector3dVector(marker_points)
        marker_point_cloud.colors = o3d.utility.Vector3dVector(marker_colors)
        o3d.visualization.draw_geometries([marker_point_cloud])

        # 赤色のポイントデータをテキストファイルに保存
        with open('marker_points_data.txt', 'w') as f:
            for i in range(len(marker_points)):
                x, y, z = marker_points[i]
                r, g, b = marker_colors[i]
                f.write(f"{x:.6f} {y:.6f} {z:.6f} {r:.6f} {g:.6f} {b:.6f}\n")

        mean_point = np.mean(points, axis=0)
        min_point = np.min(points, axis=0)
        max_point = np.max(points, axis=0)

        print(f"座標の平均値: x = {mean_point[0]:.6f}, y = {mean_point[1]:.6f}, z = {mean_point[2]:.6f}")
        print(f"座標の最小値: x = {min_point[0]:.6f}, y = {min_point[1]:.6f}, z = {min_point[2]:.6f}")
        print(f"座標の最大値: x = {max_point[0]:.6f}, y = {max_point[1]:.6f}, z = {max_point[2]:.6f}")

        self.x_camera = mean_point[0]
        self.y_camera = mean_point[1]
        self.z_camera = mean_point[2]

    def publish_table_tf(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'camera_color_optical_frame'
        transform.header.frame_id = 'camera__optical_frame'
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
        self.get_logger().info('Publishing static transform from camera to marker')
        


    




rclpy.init(args=None)
rclpy.spin(RsSub())
rclpy.shutdown()
