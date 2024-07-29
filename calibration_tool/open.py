import cv2
import numpy as np
import open3d as o3d

# Define paths to the images
rgb_path = './data/rgb_0.png'  # Replace with your actual RGB image path
depth_path = './data/depth_0.png'  # Replace with your actual depth image path

# Load the images
rgb_image = cv2.imread(rgb_path)
rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
depth_image = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)  # Load depth image as unchanged

# Get the dimensions of the images
rgb_height, rgb_width, rgb_channels = rgb_image.shape
depth_height, depth_width = depth_image.shape

# Print the dimensions
print(f"RGB Image - Width: {rgb_width}, Height: {rgb_height}, Channels: {rgb_channels}")
print(f"Depth Image - Width: {depth_width}, Height: {depth_height}")


# Normalize depth image for better visualization
# Depth image normalization to 8-bit for display purposes
#depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
#depth_image_normalized = np.uint8(depth_image_normalized)

#color = o3d.geometry.Image(input_rgb)
#depth = o3d.geometry.Image(input_d)#.astype(np.uint16))

color = o3d.geometry.Image(rgb_image)
depth = o3d.geometry.Image(depth_image)

rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color, depth, depth_trunc=5.0, convert_rgb_to_intensity=False)


pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
    #640, 480, 618.335, 618.521, 311.006, 239.008
    1280, 720, 908.600, 908.537, 643.698, 382.494
)

# ポイントクラウドを作成
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

colors = np.asarray(point_cloud.colors)
#colors = colors[:, ::-1]
print("Color data sample:", colors[:5]) 

# 赤色の範囲を定義（浮動小数点数形式）
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
o3d.visualization.draw_geometries([filtered_point_cloud])


points = np.asarray(point_cloud.points)
colors = np.asarray(point_cloud.colors)

# ポイントと色をテキストファイルに保存
with open('point_cloud_data.txt', 'w') as f:
    for i in range(len(points)):
        x, y, z = points[i]
        r, g, b = colors[i]
        f.write(f"{x:.6f} {y:.6f} {z:.6f} {r:.6f} {g:.6f} {b:.6f}\n")


filtered_points = points[mask]
filtered_colors = colors[mask]

# 赤色のポイントデータをテキストファイルに保存
with open('red_points_data.txt', 'w') as f:
    for i in range(len(filtered_points)):
        x, y, z = filtered_points[i]
        r, g, b = filtered_colors[i]
        f.write(f"{x:.6f} {y:.6f} {z:.6f} {r:.6f} {g:.6f} {b:.6f}\n")



# 1000 mm = 1 m 以内の距離を設定
distance_threshold = 1.0  # メートル単位
distances = np.linalg.norm(points, axis=1)

# 赤色と距離の条件を満たすマスクを作成
color_mask = np.all((colors >= lower_bound) & (colors <= upper_bound), axis=1)
distance_mask = distances <= distance_threshold
combined_mask = color_mask & distance_mask

# 条件を満たすポイントを抽出
filtered_points = points[combined_mask]
filtered_colors = colors[combined_mask]

print(f"Number of filtered points: {len(filtered_points)}")  # フィルタリング後のポイント数を表示

# フィルタリングされたポイントクラウドを作成
filtered_point_cloud = o3d.geometry.PointCloud()
filtered_point_cloud.points = o3d.utility.Vector3dVector(filtered_points)
filtered_point_cloud.colors = o3d.utility.Vector3dVector(filtered_colors)
o3d.visualization.draw_geometries([filtered_point_cloud])

points = np.asarray(filtered_point_cloud.points)
colors = np.asarray(filtered_point_cloud.colors)

with open('1m_cloud_data.txt', 'w') as f:
    for i in range(len(points)):
        x, y, z = points[i]
        r, g, b = colors[i]
        f.write(f"{x:.6f} {y:.6f} {z:.6f} {r:.6f} {g:.6f} {b:.6f}\n")

mean_point = np.mean(points, axis=0)
min_point = np.min(points, axis=0)
max_point = np.max(points, axis=0)

# 結果を表示
print(f"座標の平均値: x = {mean_point[0]:.6f}, y = {mean_point[1]:.6f}, z = {mean_point[2]:.6f}")
print(f"座標の最小値: x = {min_point[0]:.6f}, y = {min_point[1]:.6f}, z = {min_point[2]:.6f}")
print(f"座標の最大値: x = {max_point[0]:.6f}, y = {max_point[1]:.6f}, z = {max_point[2]:.6f}")

with open('coordinates_statistics.txt', 'w') as f:
    f.write(f"座標の平均値: x = {mean_point[0]:.6f}, y = {mean_point[1]:.6f}, z = {mean_point[2]:.6f}\n")
    f.write(f"座標の最小値: x = {min_point[0]:.6f}, y = {min_point[1]:.6f}, z = {min_point[2]:.6f}\n")
    f.write(f"座標の最大値: x = {max_point[0]:.6f}, y = {max_point[1]:.6f}, z = {max_point[2]:.6f}\n")
    f.write(f"transform.transform.translation.x = {mean_point[0]:.6f}\n")
    f.write(f"transform.transform.translation.y = {-mean_point[1]:.6f}\n")
    f.write(f"transform.transform.translation.z = {-mean_point[2]:.6f}\n")
"""
# Display the images
cv2.imshow("RGB Image", rgb_image)
cv2.imshow("Depth Image", depth_image_normalized)

# Wait until a key is pressed and close windows
cv2.waitKey(0)
cv2.destroyAllWindows()
"""