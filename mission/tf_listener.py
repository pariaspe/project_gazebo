import rclpy
from rclpy.node import Node
from tf2_geometry_msgs import PoseStamped
from sensor_msgs.msg import LaserScan
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_ros


class TFListener(Node):

    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscription = self.create_subscription(
            LaserScan,
            '/drone0/sensor_measurements/point_lidar/scan',
            self.scan_callback,
            10)

    def scan_callback(self, msg: LaserScan):
        try:
            p1 = PoseStamped()
            p1.header.frame_id = 'drone0/gb/_0/_1/_2/point_lidar/point_lidar/gpu_ray'
            p1.header.stamp = msg.header.stamp
            p1.pose.position.x = msg.ranges[0]

            p2 = self.tf_buffer.transform(p1, 'earth', new_type=PoseStamped)
            self.get_logger().info(
                f"{p2.pose.position.x}, {p2.pose.position.y}, {p2.pose.position.z}")
        except (tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Exception occurred: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    tf_listener = TFListener()
    rclpy.spin(tf_listener)
    tf_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
