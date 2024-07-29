import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs

class StaticTransformPublisher(Node):
    def __init__(self):
        super().__init__('static_transform_publisher')

        # Create a tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Create a static transform broadcaster
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.timer = self.create_timer(1.0, self.transform_and_publish)

    def transform_and_publish(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'center', rclpy.time.Time())
            
            tx,ty,tz = trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z
            # Define the static transform from map to table
            transform = geometry_msgs.msg.TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'map'
            transform.child_frame_id = 'table'

            # Set translation
            transform.transform.translation.x = tx
            transform.transform.translation.y = ty
            transform.transform.translation.z = tz

            # Set rotation (identity quaternion)
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 1.0

            # Publish the transform
            self.tf_broadcaster.sendTransform(transform)
            self.get_logger().info('Publishing static transform from map to table')

        except Exception as e:
            self.get_logger().info(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = StaticTransformPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
