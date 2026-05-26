import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class Odom2Base(Node):

    def __init__(self):
        super().__init__('odom2base')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/sierra/odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg: Odometry):

        t = TransformStamped()

        # Time
        t.header.stamp = msg.header.stamp
        t.header.frame_id = '/sierra/odom'      
        t.child_frame_id = "/sierra/base_link"      

        # Translation
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Rotation (already quaternion in odom)
        t.transform.rotation = msg.pose.pose.orientation

        # Broadcast TF
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    node = Odom2Base()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()