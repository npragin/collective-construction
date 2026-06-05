import copy

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped

from cc_interfaces.action import CorrectionTask
from manipulator_interface.action import AbsoluteMove


class CorrectionTaskServer(Node):
    """
    Correction server for misplaced blocks.

    This server is called by the placement accuracy checker when a block is:
        - seen by perception
        - outside the allowed placement tolerance

    It receives:
        current_poses[i] = where the block is now
        desired_poses[i] = where the block should be

    It then performs:
        pick from current pose -> place at desired pose
    """

    def __init__(self):
        super().__init__('correction_task_server')

        self.callback_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('namespace', 'j100_0897')
        self.declare_parameter('correction_action_name', '/manipulator/correction_task')

        self.declare_parameter('target_frame', 'arm_0_end_effector_link')
        self.declare_parameter('planner', 'ompl')
        self.declare_parameter('pilz_planner', 'PTP')

        # These offsets need tuning for the real block/gripper geometry
        self.declare_parameter('approach_height', 0.20)
        self.declare_parameter('pick_z_offset', 0.10)
        self.declare_parameter('place_z_offset', 0.10)

        # Use a fixed end-effector orientation similar to the existing pick test pose
        self.declare_parameter(
            'fixed_gripper_orientation',
            [-0.055763, 0.9973, 0.0073431, 0.04721]
        )

        self.namespace = self.get_parameter('namespace').value
        self.correction_action_name = self.get_parameter('correction_action_name').value

        self.target_frame = self.get_parameter('target_frame').value
        self.planner = self.get_parameter('planner').value
        self.pilz_planner = self.get_parameter('pilz_planner').value

        self.approach_height = float(self.get_parameter('approach_height').value)
        self.pick_z_offset = float(self.get_parameter('pick_z_offset').value)
        self.place_z_offset = float(self.get_parameter('place_z_offset').value)

        self.fixed_gripper_orientation = list(
            self.get_parameter('fixed_gripper_orientation').value
        )

        self.get_logger().info(f'Using namespace: {self.namespace}')
        self.get_logger().info(f'Correction action: {self.correction_action_name}')
        self.get_logger().info(f'Target frame: {self.target_frame}')

        # Action server called by the checker
        self.correction_server = ActionServer(
            self,
            CorrectionTask,
            self.correction_action_name,
            self.execute_correction_task,
            callback_group=self.callback_group
        )

        # Existing AbsoluteMove action client
        self.absolute_move_client = ActionClient(
            self,
            AbsoluteMove,
            'absolute_move',
            callback_group=self.callback_group
        )

        self.get_logger().info('Waiting for AbsoluteMove action server...')
        self.absolute_move_client.wait_for_server()
        self.get_logger().info('AbsoluteMove action server connected.')

        # Existing gripper action client
        gripper_action = f'/{self.namespace}/arm_0_gripper_controller/gripper_cmd'
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            gripper_action,
            callback_group=self.callback_group
        )

        self.get_logger().info(f'Waiting for gripper action server: {gripper_action}')
        self.gripper_client.wait_for_server()
        self.get_logger().info('Gripper action server connected.')

        self.get_logger().info('Correction task server is ready.')

    async def execute_correction_task(self, goal_handle):
        request = goal_handle.request

        result = CorrectionTask.Result()
        result.corrected_ids = []
        result.failed_ids = []

        if not self._valid_request(request):
            result.success = False
            result.message = (
                'Invalid correction request. block_ids, aruco_ids, '
                'current_poses, and desired_poses must have the same length.'
            )
            self.get_logger().error(result.message)
            goal_handle.succeed()
            return result

        self.get_logger().warn(
            f'Received correction task for blocks: {list(request.block_ids)}'
        )

        for i in range(len(request.block_ids)):
            block_id = str(request.block_ids[i])
            aruco_id = int(request.aruco_ids[i])
            current_pose = request.current_poses[i]
            desired_pose = request.desired_poses[i]

            feedback = CorrectionTask.Feedback()
            feedback.status = f'correcting {block_id}'
            goal_handle.publish_feedback(feedback)

            self.get_logger().warn(
                f'Starting correction for {block_id}, ArUco ID {aruco_id}'
            )

            success = await self.correct_one_block(
                block_id,
                current_pose,
                desired_pose,
                goal_handle
            )

            if success:
                result.corrected_ids.append(block_id)
                self.get_logger().info(f'Correction completed for {block_id}')
            else:
                result.failed_ids.append(block_id)
                self.get_logger().error(f'Correction failed for {block_id}')

        if len(result.failed_ids) == 0:
            result.success = True
            result.message = 'All misplaced blocks corrected successfully.'
        else:
            result.success = False
            result.message = (
                f'Correction completed with failures. '
                f'corrected={list(result.corrected_ids)}, '
                f'failed={list(result.failed_ids)}'
            )

        goal_handle.succeed()
        return result

    def _valid_request(self, request) -> bool:
        n = len(request.block_ids)
        return (
            len(request.aruco_ids) == n and
            len(request.current_poses) == n and
            len(request.desired_poses) == n
        )

    async def correct_one_block(
        self,
        block_id: str,
        current_pose: PoseStamped,
        desired_pose: PoseStamped,
        goal_handle
    ) -> bool:
        """
        Correct one misplaced block.

        current_pose: block pose from perception
        desired_pose: target pose from planner
        """

        # 1. Move above current block
        self.publish_status(goal_handle, f'{block_id}: moving above current pose')
        current_approach = self.make_ee_pose_from_block_pose(
            current_pose,
            z_offset=self.approach_height
        )
        if not await self.move_to_absolute_pose(current_approach):
            return False

        # 2. Move down to pick pose
        self.publish_status(goal_handle, f'{block_id}: moving to pick pose')
        current_pick = self.make_ee_pose_from_block_pose(
            current_pose,
            z_offset=self.pick_z_offset
        )
        if not await self.move_to_absolute_pose(current_pick):
            return False

        # 3. Close gripper
        self.publish_status(goal_handle, f'{block_id}: closing gripper')
        if not await self.move_gripper('close'):
            return False

        # 4. Lift block
        self.publish_status(goal_handle, f'{block_id}: lifting block')
        if not await self.move_to_absolute_pose(current_approach):
            return False

        # 5. Move above desired pose
        self.publish_status(goal_handle, f'{block_id}: moving above desired pose')
        desired_approach = self.make_ee_pose_from_block_pose(
            desired_pose,
            z_offset=self.approach_height
        )
        if not await self.move_to_absolute_pose(desired_approach):
            return False

        # 6. Move down to place pose
        self.publish_status(goal_handle, f'{block_id}: moving to place pose')
        desired_place = self.make_ee_pose_from_block_pose(
            desired_pose,
            z_offset=self.place_z_offset
        )
        if not await self.move_to_absolute_pose(desired_place):
            return False

        # 7. Open gripper
        self.publish_status(goal_handle, f'{block_id}: opening gripper')
        if not await self.move_gripper('open'):
            return False

        # 8. Retreat
        self.publish_status(goal_handle, f'{block_id}: retreating after placement')
        if not await self.move_to_absolute_pose(desired_approach):
            return False

        return True

    def publish_status(self, goal_handle, status: str):
        feedback = CorrectionTask.Feedback()
        feedback.status = status
        goal_handle.publish_feedback(feedback)
        self.get_logger().info(status)

    def make_ee_pose_from_block_pose(
        self,
        block_pose: PoseStamped,
        z_offset: float
    ) -> PoseStamped:
        """
        Convert a block pose into an end-effector pose.

        The block pose comes from perception/planner.
        The end-effector pose is created by:
            - copying x and y
            - adding a z offset
            - using a fixed gripper orientation

        Tune z_offset and fixed_gripper_orientation for your actual gripper.
        """

        ee_pose = PoseStamped()
        ee_pose.header = copy.deepcopy(block_pose.header)

        # If frame is empty, fall back to world
        if ee_pose.header.frame_id == '':
            ee_pose.header.frame_id = 'world'

        ee_pose.pose.position.x = block_pose.pose.position.x
        ee_pose.pose.position.y = block_pose.pose.position.y
        ee_pose.pose.position.z = block_pose.pose.position.z + z_offset

        ee_pose.pose.orientation.x = self.fixed_gripper_orientation[0]
        ee_pose.pose.orientation.y = self.fixed_gripper_orientation[1]
        ee_pose.pose.orientation.z = self.fixed_gripper_orientation[2]
        ee_pose.pose.orientation.w = self.fixed_gripper_orientation[3]

        return ee_pose

    async def move_to_absolute_pose(
        self,
        pose: PoseStamped
    ) -> bool:
        """
        Send a pose goal to the AbsoluteMove action server.
        """

        goal_msg = AbsoluteMove.Goal()
        goal_msg.pose = pose
        goal_msg.planner = self.planner
        goal_msg.pilz_planner = self.pilz_planner
        goal_msg.target_frame = self.target_frame

        self.get_logger().info(
            f'Sending AbsoluteMove goal: '
            f'frame={pose.header.frame_id}, '
            f'x={pose.pose.position.x:.3f}, '
            f'y={pose.pose.position.y:.3f}, '
            f'z={pose.pose.position.z:.3f}'
        )

        send_goal_future = self.absolute_move_client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future

        if not goal_handle.accepted:
            self.get_logger().error('AbsoluteMove goal rejected.')
            return False

        result_future = goal_handle.get_result_async()
        result = await result_future

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('AbsoluteMove succeeded.')
            return True

        self.get_logger().error(f'AbsoluteMove failed with status: {result.status}')
        return False

    async def move_gripper(self, direction: str = 'open') -> bool:
        """
        Send open/close command to gripper.
        """

        goal_msg = GripperCommand.Goal()

        if direction == 'close':
            goal_msg.command.position = 0.85
        else:
            goal_msg.command.position = 0.0

        self.get_logger().info(f'Sending gripper command: {direction}')

        send_goal_future = self.gripper_client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future

        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected.')
            return False

        result_future = goal_handle.get_result_async()
        result = await result_future

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Gripper command succeeded: {direction}')
            return True

        self.get_logger().error(f'Gripper command failed with status: {result.status}')
        return False


def main(args=None):
    rclpy.init(args=args)

    node = CorrectionTaskServer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
