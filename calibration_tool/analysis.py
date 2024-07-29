import open3d as o3d
import cv2
import numpy as np

# Load the images
def load_rgbd_images(rgb_path, depth_path):
    rgb_image = cv2.imread(rgb_path).astype(np.uint8)
    depth_image = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED).astype(np.float32)
    return rgb_image, depth_image

# Define paths
rgb_path = './data/rgb_9.png'  # Replace with the actual file path
depth_path = './data/depth_9.png'  # Replace with the actual file path

# Load images
rgb_image, depth_image = load_rgbd_images(rgb_path, depth_path)

# Convert to Open3D format
color = o3d.geometry.Image(rgb_image)
depth = o3d.geometry.Image(depth_image)

rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    color, depth, depth_trunc=5000, convert_rgb_to_intensity=False
)

# Camera intrinsic parameters
fx_d = 525.0  # Example value, replace with actual
fy_d = 525.0  # Example value, replace with actual
cx_d = 319.5  # Example value, replace with actual
cy_d = 239.5  # Example value, replace with actual
height, width = depth_image.shape
pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
    width, height, fx_d, fy_d, cx_d, cy_d
)

# Create point cloud
point_cloud = o3d.geometry.PointCloud.create_from_rgbd_image(
    rgbd_image, pinhole_camera_intrinsic
)

# Flip the point cloud (optional, for correct orientation)
point_cloud.transform([[1, 0, 0, 0],
                       [0, -1, 0, 0],
                       [0, 0, -1, 0],
                       [0, 0, 0, 1]])

# Visualize the point cloud
o3d.visualization.draw_geometries([point_cloud])
