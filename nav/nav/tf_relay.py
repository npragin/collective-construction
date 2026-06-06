"""Relay selected camera TF frames from the global /tf onto the robot <ns>/tf.

The overhead camera (cc_localization) publishes ``world -> build`` (and other
frames) on the GLOBAL ``/tf``, but the robot's tree
(``world -> odom -> base_link -> ... -> arm_0_base_link``) lives on the
namespaced ``/<robot_ns>/tf`` that ``ekf_world`` writes to. The two ``world``
subtrees are on different topics, so robot-side nodes can't transform e.g.
``build`` against the arm.

This node copies the configured frames from the global ``/tf`` onto the robot's
``/tf`` (their shared ``world`` glues them together).

Implementation note: tf2_ros.TransformBroadcaster publishes to the ABSOLUTE
``/tf``, so it can't target the namespaced topic. We therefore use a plain
publisher to the explicit ``/<robot_ns>/tf`` and a plain subscription to the
absolute ``/tf`` -- both absolute, so they neither need remaps nor collide.
"""

from tf2_msgs.msg import TFMessage

import rclpy
from rclpy.node import Node


class TfRelay(Node):
    """Copy configured camera frames from the global /tf to the robot /tf."""

    def __init__(self):
        super().__init__('tf_relay')

        # Camera child frames to relay (their parent `world` is already on the
        # robot tf via ekf_world).
        self.declare_parameter('frames', ['build'])
        # Robot namespace whose /tf we publish onto.
        self.declare_parameter('robot_namespace', 'j100_0897')
        self.frames = set(self.get_parameter('frames').value)
        robot_ns = self.get_parameter('robot_namespace').value

        # Subscribe to the GLOBAL /tf (camera tree).
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        # Publish to the robot's NAMESPACED /tf via an explicit absolute name.
        self.pub = self.create_publisher(TFMessage, f'/{robot_ns}/tf', 100)

        self.get_logger().info(
            f'Relaying camera frames {sorted(self.frames)} from /tf onto '
            f'/{robot_ns}/tf')

    def on_tf(self, msg):
        """Re-publish the configured frames onto the robot /tf."""
        relayed = []
        for t in msg.transforms:
            if t.child_frame_id in self.frames:
                # Restamp with now(): these frames are (near-)static, keeping
                # them fresh in the robot clock domain avoids extrapolation.
                t.header.stamp = self.get_clock().now().to_msg()
                relayed.append(t)
        if relayed:
            out = TFMessage()
            out.transforms = relayed
            self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TfRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
