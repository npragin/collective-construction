
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs



class TransformTesting(Node):

    def __init__(self):
        super().__init__('test_tf')

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.p = PoseStamped()

        self.p.header.frame_id = 'sierra/odom'

        self.p.pose.position.x = 1.0
        self.p.pose.position.y = 1.0
        self.p.pose.position.z = 1.0

        self.p.pose.orientation.x = 0.0
        self.p.pose.orientation.y = 0.0
        self.p.pose.orientation.z = 0.0
        self.p.pose.orientation.w = 1.0

        self.timer = self.create_timer(1.0, self.test_transform)

    def test_transform(self):
        try:
            # self.p.header.stamp = self.get_clock().now().to_msg()
            self.get_logger().info(
                f'Prior tf:\n\tx={self.p.pose.position.x}\n\ty={self.p.pose.position.y}\n'
            )
    
            pose_map = self.tf_buffer.transform(
                self.p,
                'map'
            )
    
            self.get_logger().info(
                f'Post tf:\n\tx={pose_map.pose.position.x}\n\ty={pose_map.pose.position.y}\n'
            )
    
        except Exception as e:
            self.get_logger().error(str(e))

def main(args=None):
    rclpy.init(args=args)

    node = TransformTesting()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":

    main()



