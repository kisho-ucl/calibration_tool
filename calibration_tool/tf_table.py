import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros

class StaticTransformPublisher(Node):
    def __init__(self):
        super().__init__('static_transform_publisher')

        # Create a static transform broadcaster
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.transform_and_broadcast)

    def transform_and_broadcast(self):
        # Define the static transform
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'camera_depth_optical_frame'
        transform.child_frame_id = 'center'

        # Set translation
        transform.transform.translation.x = 0.103822
        transform.transform.translation.y = 0.076542
        transform.transform.translation.z = 0.422882

        
        # Set rotation (in radians)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        # Publish the transform
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info('Publishing static transform from camera_link to table')

def main(args=None):
    rclpy.init(args=args)
    node = StaticTransformPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
