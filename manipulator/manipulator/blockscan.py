import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import numpy as np


class BlockScan(Node):
    """Camera-based block search exposed as a service.

    On a ``/blockscan/scan`` (std_srvs/Trigger) request, the robot rotates and
    drives until the centroid of the largest orange blob reaches a target point
    in the image (horizontal middle, vertical lower third), then stops and
    returns success. Returns failure if the target is not reached before the
    timeout. The base is always commanded to stop before the service returns.
    """

    # Orange block colour gate (HSV).
    HSV_LOWER = np.array([0.0, 128.0, 200.0])
    HSV_UPPER = np.array([15.0, 255.0, 255.0])
    MIN_AREA = 500          # px, ignore smaller orange blobs as noise

    # Target image point for the orange centroid: horizontal middle, vertical
    # lower third (y grows downward, so 2/3 of the height is the lower third).
    TARGET_X_FRAC = 0.5
    TARGET_Y_FRAC = 0.55    # slightly higher than the lower third (y grows downward)

    # Horizontal centring via rotation. Proportional on pixel error so the turn
    # eases off near target instead of overshooting.
    CENTER_TOL = 20         # px horizontal error allowed before we stop turning
    SEARCH_ANGULAR = 0.4    # rad/s, slow in-place sweep while searching
    ANGULAR_GAIN = 0.0015   # rad/s per px of horizontal error
    MAX_ANGULAR = 0.3       # rad/s cap on the centring turn

    # Vertical placement via forward/back motion. Driving forward makes a block
    # on the ground appear lower in the frame, so we close the vertical pixel
    # error by moving along x. Proportional and capped to avoid overshoot.
    VERTICAL_TOL = 20       # px vertical error allowed before we stop driving
    LINEAR_GAIN = 0.0004    # m/s per px of vertical error
    MAX_LINEAR = 0.06       # m/s cap on the approach

    # Move-stop-settle control cycle. Each cycle measures on a stationary robot,
    # pulses motion for PULSE_S, then stops and waits SETTLE_S for motion blur
    # to clear before the next measurement. This keeps motion slow relative to
    # the camera frame rate so the servo converges instead of oscillating.
    PULSE_S = 0.2           # s of motion commanded per cycle
    SETTLE_S = 0.3          # s stopped before re-measuring

    def __init__(self):
        super().__init__('blockscan')

        self.declare_parameter('namespace', 'j100_0897')
        ns = self.get_parameter('namespace').value

        self.declare_parameter('timeout', 20.0)
        self.timeout = self.get_parameter('timeout').value

        # Sensors run in a reentrant group so their callbacks keep firing while
        # the service callback blocks in its servo loop (MultiThreadedExecutor).
        sensor_group = ReentrantCallbackGroup()
        self.sub_color = self.create_subscription(
            Image, f'/{ns}/sensors/camera_0/color/image',
            self.camera_callback, 10, callback_group=sensor_group)

        self.pub = self.create_publisher(TwistStamped, f'/{ns}/cmd_vel', 10)

        self.scan_srv = self.create_service(
            Trigger, '/blockscan/scan', self.scan_callback,
            callback_group=MutuallyExclusiveCallbackGroup())

        self.bridge = CvBridge()
        self.color_image = None
        # Bumped on every color frame so the servo loop can wait for a fresh
        # image instead of acting on a stale one.
        self.color_seq = 0

        self.get_logger().info('BlockScan service ready on /blockscan/scan')

    def camera_callback(self, msg):
        self.color_image = msg
        self.color_seq += 1

    def make_twist(self, linear=0.0, angular=0.0):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = linear
        twist.twist.angular.z = angular
        return twist

    def stop(self):
        self.pub.publish(self.make_twist())

    def compute_command(self):
        """Return (twist, on_target) from the latest color frame.

        ``on_target`` is True once the orange centroid sits within tolerance of
        the target point (horizontal middle, vertical lower third). With no
        usable blob the default is a slow in-place search rotation.
        """
        if self.color_image is None:
            return self.make_twist(angular=self.SEARCH_ANGULAR), False

        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.color_image, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().info(f'Error converting image: {e}')
            return self.make_twist(angular=self.SEARCH_ANGULAR), False

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.HSV_LOWER, self.HSV_UPPER)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        height, width, _ = cv_image.shape

        # No orange at all: sweep in the fixed search direction to find some.
        if not contours:
            return self.make_twist(angular=self.SEARCH_ANGULAR), False

        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)

        # Degenerate centroid: nothing usable, fall back to the fixed sweep.
        if M['m00'] <= 0:
            return self.make_twist(angular=self.SEARCH_ANGULAR), False

        cX = int(M['m10'] / M['m00'])
        cY = int(M['m01'] / M['m00'])

        # Horizontal pixel error from the target (+ve: centroid left of target).
        error_x = width * self.TARGET_X_FRAC - cX

        # Blob too small to place precisely (far / partially in view): turn
        # toward the largest orange contour rather than sweeping blindly, so we
        # rotate in whichever direction homes in on it.
        if cv2.contourArea(largest_contour) <= self.MIN_AREA:
            direction = 1.0 if error_x >= 0 else -1.0
            return self.make_twist(angular=direction * self.SEARCH_ANGULAR), False

        # Vertical pixel error from the target (+ve: centroid above lower third).
        error_y = height * self.TARGET_Y_FRAC - cY

        on_target_x = abs(error_x) <= self.CENTER_TOL
        on_target_y = abs(error_y) <= self.VERTICAL_TOL

        # On target on both axes: centroid is at middle / lower third. Done.
        if on_target_x and on_target_y:
            self.get_logger().info(
                f'Block on target (err x {error_x:.0f}, y {error_y:.0f} px)')
            return self.make_twist(), True

        # Rotate to centre horizontally; drive forward/back to reach the lower
        # third. Proportional and capped on each axis to ease off near target.
        # Zero an axis once it is within tolerance so it doesn't jitter while
        # the other axis is still converging.
        angular = 0.0 if on_target_x else error_x * self.ANGULAR_GAIN
        linear = 0.0 if on_target_y else error_y * self.LINEAR_GAIN
        angular = max(min(angular, self.MAX_ANGULAR), -self.MAX_ANGULAR)
        linear = max(min(linear, self.MAX_LINEAR), -self.MAX_LINEAR)
        return self.make_twist(linear=linear, angular=angular), False

    def _wait_for_new_frame(self, last_seq, deadline):
        """Block until a color frame newer than last_seq arrives (or timeout).

        Returns the new sequence number, or None if the deadline passed first.
        """
        while self.color_seq == last_seq:
            if time.time() >= deadline:
                return None
            time.sleep(0.02)
        return self.color_seq

    def scan_callback(self, request, response):
        self.get_logger().info('BlockScan: scan requested')
        deadline = time.time() + self.timeout
        last_seq = self.color_seq

        try:
            while time.time() < deadline:
                # Measure on a stationary robot: wait for a frame captured after
                # the previous pulse settled, so motion blur has cleared and the
                # command reflects the current pose, not a stale one.
                seq = self._wait_for_new_frame(last_seq, deadline)
                if seq is None:
                    break
                last_seq = seq

                twist, on_target = self.compute_command()

                if on_target:
                    response.success = True
                    response.message = 'Block at middle / lower third'
                    self.get_logger().info(response.message)
                    return response

                # Pulse the motion briefly, then stop and let the camera settle
                # before the next measurement. Keeps motion slow relative to the
                # frame rate so the servo converges instead of oscillating.
                self.pub.publish(twist)
                time.sleep(self.PULSE_S)
                self.stop()
                time.sleep(self.SETTLE_S)

            response.success = False
            response.message = 'Timed out searching for block'
            self.get_logger().warn(response.message)
            return response
        finally:
            self.stop()


def main(args=None):
    rclpy.init(args=args)
    scanner = BlockScan()
    executor = MultiThreadedExecutor()
    executor.add_node(scanner)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        scanner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
