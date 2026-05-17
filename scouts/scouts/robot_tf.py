import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class WorldOdomBroadcaster(Node):

    def __init__(self):
        super().__init__('world_odom_broadcaster')
        self.br = StaticTransformBroadcaster(self)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'odom'
        t.transform.rotation.w = 1.0  # identity rotation

        self.br.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = WorldOdomBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
