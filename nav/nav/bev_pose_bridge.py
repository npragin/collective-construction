"""Bridge the overhead BEV ArUco TF into a base_link pose for the global EKF.

The overhead camera (cc_localization) broadcasts ``world -> aruco_<tag>`` for the
marker mounted on this robot. robot_localization cannot consume a raw TF edge as
an absolute reference, so this node looks that transform up, shifts it from the
marker to ``base_link`` using the known mounting offset, and republishes it as a
``geometry_msgs/PoseWithCovarianceStamped`` in the ``world`` frame on
``bev_pose``. The global EKF fuses that pose with wheel odometry and IMU to
publish ``world -> odom``.

The transform is flattened to 2D (yaw only), matching cc_localization (which
already zeroes z and keeps yaw only) and the EKF's ``two_d_mode``.
"""

import math

from geometry_msgs.msg import PoseWithCovarianceStamped

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros

from tf_transformations import quaternion_from_euler


class BevPoseBridge(Node):
    """Republish the overhead-camera marker TF as a base_link pose."""

    def __init__(self):
        super().__init__('bev_pose_bridge')

        self.declare_parameter('tag_id', 15)
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('marker_frame', '')
        self.declare_parameter('marker_offset_x', 0.0)
        self.declare_parameter('marker_offset_y', 0.0)
        self.declare_parameter('marker_offset_yaw', 0.0)
        self.declare_parameter('output_topic', 'bev_pose')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('pose_covariance', [0.1, 0.1, 0.05])
        self.declare_parameter('max_jump_speed', 1.0)  # m/s

        self.world_frame = self.get_parameter('world_frame').value
        self.base_link_frame = self.get_parameter('base_link_frame').value
        tag_id = self.get_parameter('tag_id').value
        marker_frame = self.get_parameter('marker_frame').value
        self.marker_frame = marker_frame or f'aruco_{tag_id}'
        self.off_x = self.get_parameter('marker_offset_x').value
        self.off_y = self.get_parameter('marker_offset_y').value
        self.off_yaw = self.get_parameter('marker_offset_yaw').value
        self.cov = self.get_parameter('pose_covariance').value
        self.max_jump_speed = self.get_parameter('max_jump_speed').value
        output_topic = self.get_parameter('output_topic').value

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, output_topic, 10)

        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self._last_stamp = None
        self._last_accepted = None  # (x, y, sec_float) of last published fix

        self.get_logger().info(
            f'BEV pose bridge: {self.world_frame} -> {self.marker_frame} '
            f'-> {self.base_link_frame}, publishing on {output_topic}')

    def on_timer(self):
        """Look up the latest marker TF and republish the base_link pose."""
        try:
            tf = self.buffer.lookup_transform(
                self.world_frame, self.marker_frame, Time())
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(
                f'No transform {self.world_frame} -> {self.marker_frame}: '
                f'{exc}', throttle_duration_sec=2.0)
            return


        stamp = tf.header.stamp
        key = (stamp.sec, stamp.nanosec)
        if key == self._last_stamp:
            return
        self._last_stamp = key

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw_marker = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # world->base_link = (world->marker) composed with (base_link->marker)^-1.
        yaw_bl = yaw_marker - self.off_yaw
        x_bl = t.x - (math.cos(yaw_bl) * self.off_x
                      - math.sin(yaw_bl) * self.off_y)
        y_bl = t.y - (math.sin(yaw_bl) * self.off_x
                      + math.cos(yaw_bl) * self.off_y)

        # Drop physically impossible jumps from the last accepted fix: a bad
        # overhead detection (ArUco ambiguity, or a cc_localization world-frame
        # switch) would otherwise teleport the EKF and never recover.
        now = stamp.sec + stamp.nanosec * 1e-9
        if self._last_accepted is not None and self.max_jump_speed > 0.0:
            lx, ly, lt = self._last_accepted
            dt = now - lt
            dist = math.hypot(x_bl - lx, y_bl - ly)
            if dt > 0.0 and dist > self.max_jump_speed * dt:
                self.get_logger().warn(
                    f'Rejecting BEV fix: {dist:.1f} m jump in {dt:.2f} s '
                    f'({dist / dt:.1f} m/s > {self.max_jump_speed} m/s)',
                    throttle_duration_sec=2.0)
                return
        self._last_accepted = (x_bl, y_bl, now)

        msg = PoseWithCovarianceStamped()

        msg.header.stamp = stamp
        msg.header.frame_id = self.world_frame
        msg.pose.pose.position.x = x_bl
        msg.pose.pose.position.y = y_bl
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw_bl)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        cov = [0.0] * 36
        cov[0] = self.cov[0]    # x
        cov[7] = self.cov[1]    # y
        cov[35] = self.cov[2]   # yaw
        # Large variance on unobserved DoF so the EKF ignores them.
        cov[14] = 1e6           # z
        cov[21] = 1e6           # roll
        cov[28] = 1e6           # pitch
        msg.pose.covariance = cov

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = BevPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
