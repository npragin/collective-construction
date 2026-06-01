import math
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

from cc_interfaces.srv import CheckPlacement
from cc_interfaces.action import CorrectionTask


class PlacementAccuracyChecker(Node):
    def __init__(self):
        super().__init__('placement_accuracy_checker')

        # Parameters
        self.declare_parameter('x_tolerance', 0.10)
        self.declare_parameter('y_tolerance', 0.10)
        self.declare_parameter('theta_tolerance_deg', 10.0)
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('enable_correction', True)
        self.declare_parameter('correction_action_name', '/manipulator/correction_task')

        self.x_tolerance = float(self.get_parameter('x_tolerance').value)
        self.y_tolerance = float(self.get_parameter('y_tolerance').value)
        self.theta_tolerance = math.radians(
            float(self.get_parameter('theta_tolerance_deg').value)
        )
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.enable_correction = bool(self.get_parameter('enable_correction').value)
        self.correction_action_name = str(self.get_parameter('correction_action_name').value)

        # Latest ArUco perception data
        self.latest_marker_ids: List[int] = []
        self.latest_world_poses: List[Pose] = []

        # Subscribers from your ArUco perception node
        self.marker_id_sub = self.create_subscription(
            Int32MultiArray,
            '/aruco/marker_ids',
            self.marker_id_callback,
            10
        )

        self.world_pose_sub = self.create_subscription(
            PoseArray,
            '/aruco/poses/world_frame',
            self.world_pose_callback,
            10
        )

        # Service called by planner
        self.check_service = self.create_service(
            CheckPlacement,
            '/placement_checker/check_placement',
            self.check_placement_callback
        )

        # RViz marker output
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/placement_checker/marker_array',
            10
        )

        # Correction action client
        self.correction_client = ActionClient(
            self,
            CorrectionTask,
            self.correction_action_name
        )

        self.get_logger().info('Placement accuracy checker is ready.')
        self.get_logger().info(
            f'Tolerances: x={self.x_tolerance:.2f} m, '
            f'y={self.y_tolerance:.2f} m, '
            f'theta={math.degrees(self.theta_tolerance):.1f} deg'
        )
        self.get_logger().info(f'Correction enabled: {self.enable_correction}')
        self.get_logger().info(f'Correction action: {self.correction_action_name}')

    def marker_id_callback(self, msg: Int32MultiArray):
        self.latest_marker_ids = list(msg.data)

    def world_pose_callback(self, msg: PoseArray):
        self.latest_world_poses = list(msg.poses)

    def check_placement_callback(self, request, response):
        """
        Planner calls this service after a block has been marked successfully placed.

        Request:
            block_ids
            aruco_ids
            desired_poses

        Response:
            properly_placed_ids
            misplaced_ids
            unseen_ids

        If a block is seen but displaced, this node sends a correction action:
            current perceived pose -> desired planner pose
        """

        perceived_blocks = self.build_perceived_block_dict()

        response.properly_placed_ids = []
        response.misplaced_ids = []
        response.unseen_ids = []

        misplaced_block_ids = []
        misplaced_aruco_ids = []
        misplaced_current_poses = []
        misplaced_desired_poses = []

        marker_array = MarkerArray()

        if len(request.block_ids) != len(request.aruco_ids):
            response.success = False
            response.message = 'block_ids and aruco_ids lengths do not match.'
            self.get_logger().error(response.message)
            return response

        if len(request.block_ids) != len(request.desired_poses):
            response.success = False
            response.message = 'block_ids and desired_poses lengths do not match.'
            self.get_logger().error(response.message)
            return response

        if len(perceived_blocks) == 0:
            self.get_logger().warn('No ArUco blocks currently perceived.')

        for i in range(len(request.block_ids)):
            block_id = str(request.block_ids[i])
            aruco_id = int(request.aruco_ids[i])
            desired_pose_stamped = request.desired_poses[i]
            desired_pose = desired_pose_stamped.pose

            # Case 1: block was expected but not seen
            if aruco_id not in perceived_blocks:
                response.unseen_ids.append(block_id)
                self.get_logger().warn(
                    f'{block_id} with ArUco ID {aruco_id} was expected but not seen.'
                )
                continue

            perceived_pose = perceived_blocks[aruco_id]

            x_error = abs(desired_pose.position.x - perceived_pose.position.x)
            y_error = abs(desired_pose.position.y - perceived_pose.position.y)

            desired_theta = self.yaw_from_quaternion(desired_pose.orientation)
            perceived_theta = self.yaw_from_quaternion(perceived_pose.orientation)
            theta_error = abs(self.wrap_angle(desired_theta - perceived_theta))

            is_good = (
                x_error <= self.x_tolerance and
                y_error <= self.y_tolerance and
                theta_error <= self.theta_tolerance
            )

            # Case 2: block is seen and properly placed
            if is_good:
                response.properly_placed_ids.append(block_id)

                marker = self.make_box_marker(
                    marker_id=i,
                    pose=perceived_pose,
                    color='green'
                )
                marker_array.markers.append(marker)

                self.get_logger().info(
                    f'{block_id} OK | '
                    f'aruco={aruco_id}, '
                    f'x_err={x_error:.3f} m, '
                    f'y_err={y_error:.3f} m, '
                    f'theta_err={math.degrees(theta_error):.2f} deg'
                )

            # Case 3: block is seen but displaced
            else:
                response.misplaced_ids.append(block_id)

                current_pose_stamped = self.pose_to_posestamped(perceived_pose)

                misplaced_block_ids.append(block_id)
                misplaced_aruco_ids.append(aruco_id)
                misplaced_current_poses.append(current_pose_stamped)
                misplaced_desired_poses.append(desired_pose_stamped)

                marker = self.make_box_marker(
                    marker_id=i,
                    pose=perceived_pose,
                    color='red'
                )
                marker_array.markers.append(marker)

                self.get_logger().warn(
                    f'{block_id} MISPLACED | '
                    f'aruco={aruco_id}, '
                    f'x_err={x_error:.3f} m, '
                    f'y_err={y_error:.3f} m, '
                    f'theta_err={math.degrees(theta_error):.2f} deg'
                )

        # Publish green/red boxes to RViz
        self.marker_pub.publish(marker_array)

        # Send correction only for seen + misplaced blocks
        if misplaced_block_ids:
            self.send_correction_task(
                misplaced_block_ids,
                misplaced_aruco_ids,
                misplaced_current_poses,
                misplaced_desired_poses
            )

        response.success = True
        response.message = (
            f'Placement check complete. '
            f'good={list(response.properly_placed_ids)}, '
            f'misplaced={list(response.misplaced_ids)}, '
            f'unseen={list(response.unseen_ids)}'
        )

        self.get_logger().info(response.message)
        return response

    def build_perceived_block_dict(self) -> Dict[int, Pose]:
        """
        Build dictionary:
            aruco_id -> perceived Pose

        Assumes /aruco/marker_ids and /aruco/poses/world_frame have matching order.
        """
        perceived_blocks = {}

        count = min(len(self.latest_marker_ids), len(self.latest_world_poses))

        for i in range(count):
            aruco_id = int(self.latest_marker_ids[i])
            pose = self.latest_world_poses[i]
            perceived_blocks[aruco_id] = pose

        return perceived_blocks

    def send_correction_task(
        self,
        block_ids,
        aruco_ids,
        current_poses,
        desired_poses
    ):
        """
        Send correction task to manipulation team.

        This only happens when:
            block is seen + block is misplaced

        It does not correct unseen blocks.
        """

        if not self.enable_correction:
            self.get_logger().warn(
                f'Correction disabled. Misplaced blocks: {block_ids}'
            )
            return

        if not self.correction_client.server_is_ready():
            self.get_logger().warn(
                f'Correction action server not ready: {self.correction_action_name}. '
                f'Misplaced blocks waiting for future correction: {block_ids}'
            )
            return

        goal_msg = CorrectionTask.Goal()
        goal_msg.block_ids = block_ids
        goal_msg.aruco_ids = aruco_ids
        goal_msg.current_poses = current_poses
        goal_msg.desired_poses = desired_poses

        self.get_logger().warn(
            f'Sending correction task for misplaced blocks: {block_ids}'
        )

        send_goal_future = self.correction_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.correction_goal_response_callback)

    def correction_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Correction goal was rejected.')
            return

        self.get_logger().info('Correction goal accepted.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.correction_result_callback)

    def correction_result_callback(self, future):
        result = future.result().result

        if result.success:
            self.get_logger().info(
                f'Correction succeeded. Corrected IDs: {list(result.corrected_ids)}'
            )
        else:
            self.get_logger().warn(
                f'Correction failed. Failed IDs: {list(result.failed_ids)}. '
                f'Message: {result.message}'
            )

    def make_box_marker(self, marker_id: int, pose: Pose, color: str) -> Marker:
        marker = Marker()

        marker.header.frame_id = self.world_frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'placement_accuracy_checker'
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose = pose

        # Adjust these to your actual block dimensions
        marker.scale.x = 0.10
        marker.scale.y = 0.10
        marker.scale.z = 0.03

        if color == 'green':
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

        marker.color.a = 0.75

        return marker

    def pose_to_posestamped(self, pose: Pose) -> PoseStamped:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.world_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose
        return pose_stamped

    def yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def wrap_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)

    node = PlacementAccuracyChecker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
