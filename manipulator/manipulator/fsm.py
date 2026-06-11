import time
import math

import numpy
from enum import Enum, auto
import asyncio
import copy

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.task import Future

from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool, Int32

from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401  registers PoseStamped for tf_buffer.transform

from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from manipulator_interface.action import TransportBlock
from manipulator_interface.action import AbsoluteMove
from control_msgs.action import GripperCommand
from manipulator_interface.srv import DetectMarkers
from std_srvs.srv import Trigger
from cc_interfaces.action import ManipulationTask


class State(Enum):

    IDLE = auto()
    NAVIGATE_TO_PICKUP = auto()
    DETECT_BLOCK = auto()
    PICK_BLOCK = auto()
    NAVIGATE_TO_DROPOFF = auto()
    PLACE_BLOCK = auto()
    COMPLETE = auto()
    ERROR = auto()

# manipulation (Jared's pick code edited)

class Manipulator:

    def __init__(self, node, namespace="j100_0897/manipulators"):
        
        self.node = node
        self.namespace = namespace
        self.target_frame = "arm_0_end_effector_link"
        self.reference_frame = "arm_0_base_link"

        self.absolute_move_client = ActionClient(node, AbsoluteMove,'absolute_move')

        self.gripper_client = ActionClient(node,GripperCommand,f'/{self.namespace}/arm_0_gripper_controller/gripper_cmd')

        node.get_logger().info('Waiting for manipulation action servers...')

        self.absolute_move_client.wait_for_server()
        self.gripper_client.wait_for_server()

        node.get_logger().info('Manipulator connected!')

        self.pick_pose = [0.46217,-0.030112,-0.1223,-0.055763,0.9973,0.0073431,0.04721]
        self.place_pose = [0.46217,-0.030112, -0.1200,-0.055763,0.9973,0.0073431,0.04721]


        self.stow_pose = [0.21218,-0.075709,0.41342,0.72917,0.016382,0.68135,-0.061708]

    def make_posestamped(self, pose_arr):

        pose = PoseStamped()
        pose.header.frame_id = self.reference_frame
        pose.pose.position.x = pose_arr[0]
        pose.pose.position.y = pose_arr[1]
        pose.pose.position.z = pose_arr[2]
        pose.pose.orientation.x = pose_arr[3]
        pose.pose.orientation.y = pose_arr[4]
        pose.pose.orientation.z = pose_arr[5]
        pose.pose.orientation.w = pose_arr[6]

        return pose

    async def move_to_pose(self,pose,planner="ompl",pilz_planner="PTP"):

        self.node.get_logger().info('Sending arm goal')

        goal_msg = AbsoluteMove.Goal()

        goal_msg.pose = pose
        goal_msg.planner = planner
        goal_msg.pilz_planner = pilz_planner
        goal_msg.target_frame = self.target_frame

        goal_future = self.absolute_move_client.send_goal_async(goal_msg)
        goal_handle = await goal_future

        if not goal_handle.accepted:
            self.node.get_logger().error('Arm goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        result = await result_future

        if result.status == 4:
            self.node.get_logger().info('Arm motion succeeded')
            return True

        self.node.get_logger().error('Arm motion failed')

        return False

    async def move_gripper(self, direction):

        self.node.get_logger().info(f'Gripper command: {direction}')

        goal_msg = GripperCommand.Goal()

        if direction == "close":
            goal_msg.command.position = 0.85
        else:
            goal_msg.command.position = 0.0

        goal_future = self.gripper_client.send_goal_async(goal_msg)
        goal_handle = await goal_future

        if not goal_handle.accepted:
            self.node.get_logger().error('Gripper goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        result = await result_future

        if result.status == 4:
            self.node.get_logger().info('Gripper motion succeeded')
            return True

        self.node.get_logger().error('Gripper motion failed')

        return False

#FSM code

class RobotFSM(Node):

    def __init__(self):

        super().__init__('robot_fsm')
        self.declare_parameter('namespace', "j100_0897")
        self.namespace = self.get_parameter("namespace").value
        self.get_logger().info(f"Using namespace: {self.namespace}")

        self.declare_parameter('manipulator_namespace', "j100_0897/manipulators")
        self.manipulator_namespace = self.get_parameter("manipulator_namespace").value
        self.get_logger().info(f"Using manipulator namespace: {self.manipulator_namespace}")

        self.declare_parameter('task_namespace', 'manipulator_1')
        task_ns = self.get_parameter('task_namespace').value
        self.get_logger().info(f'Using task namespace: {task_ns}')

        self.navigator = BasicNavigator(namespace=self.namespace)
        self.get_logger().info('Waiting for Nav2...')
        self.navigator.waitUntilNav2Active(localizer="bt_navigator")

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, f'/{self.namespace}/navigate_to_pose')
        self.nav_to_pose_client.wait_for_server()

        self.block_detection = self.create_client(DetectMarkers, '/aruco/detect_markers')
        while not self.block_detection.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.block_scan = self.create_client(Trigger, '/blockscan/scan')
        while not self.block_scan.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('blockscan service not available, waiting again...')

        self.manipulator = Manipulator(self, namespace=self.manipulator_namespace)
        self.state = State.IDLE

        # TF, used to convert dropoff poses from the world frame into the arm base frame.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.action_server = ActionServer(self,TransportBlock,'transport_block',execute_callback=self.execute_callback)

        self.action_server = ActionServer(
            self, ManipulationTask, f'/{task_ns}/manipulation_task',
            execute_callback=self.execute_callback_2)
        self.get_logger().info(
            f'ManipulationTask server ready on /{task_ns}/manipulation_task')

        self.get_logger().info('Robot FSM Ready')

        self.idle_pub = self.create_publisher(Bool, 'manipulator_idle_time', 10)
        self.placed_blocks_pub = self.create_publisher(Int32, 'manipulator_blocks_placed', 10)

        # Direct base velocity command, used to fine-tune heading after Nav2 stops.
        # nav2.yaml has enable_stamped_cmd_vel: true, so the base expects TwistStamped.
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, f'/{self.namespace}/cmd_vel', 10)

        self.timer = self.create_timer(1.0, self.timer_callback)


        self.x_offset = -0.45 #meters
        self.y_offset = 0.01 
        self.blocks_placed = 0


    def timer_callback(self):
        #send status feed back
        if self.state == State.IDLE:
            self.idle_pub.publish(Bool(data=True))
        else:
            self.idle_pub.publish(Bool(data=False))
        self.placed_blocks_pub.publish(Int32(data=self.blocks_placed))



    async def detect_blocks(self):
        attempts = 5   # initial try + 4 retries
        for attempt in range(attempts):
            # Always scan first: drive the base so an orange block sits at the
            # target image point (middle / lower third) before running marker
            # detection, so the marker is framed for the detector each attempt.
            self.get_logger().info(
                f'Scanning for block before detection '
                f'(attempt {attempt + 1}/{attempts})'
            )
            scan = await self.block_scan.call_async(Trigger.Request())
            self.get_logger().info(f'blockscan result: {scan.message}')

            block_request = DetectMarkers.Request()
            block_request.target_id = -1

            # A service call_async future resolves directly to the response;
            # there is no goal-handle / get_result_async (those are for actions).
            result = await self.block_detection.call_async(block_request)

            if result.success:
                # Keep only markers that resolved to a valid arm-frame pose
                # (empty frame_id means the TF lookup failed for that marker).
                arm_poses = [p for p in result.poses_arm if p.header.frame_id]
                if arm_poses:
                    self.get_logger().info(
                        f'Detected {len(arm_poses)} block(s); ids={list(result.ids)}'
                    )
                    return arm_poses
                self.get_logger().warn('No blocks with a valid arm-frame pose detected')
            else:
                self.get_logger().error(f'Block detection failed: {result.message}')

        self.get_logger().error(f'No block detected after {attempts} attempts')
        return None

    async def _sleep(self, seconds):
        # Cooperative sleep: lets the rclpy executor keep spinning while waiting
        # (asyncio.sleep would not be driven under rclpy.spin).
        future = Future()
        timer = self.create_timer(
            seconds, lambda: future.done() or future.set_result(None)
        )
        try:
            await future
        finally:
            self.destroy_timer(timer)

    def offset_pose(self, pose, dx=0.0, dy=0.0):
        new_pose = copy.deepcopy(pose)
        new_pose.pose.position.x += dx
        new_pose.pose.position.y += dy

        return new_pose

    async def execute_callback_2(self, goal_handle):
        """Navigate to the stockpile centroid and report success."""
        stockpile = goal_handle.request.stockpile
        points = stockpile.polygon.points

        result = ManipulationTask.Result()
        if not points:
            self.get_logger().error('Stockpile polygon has no points')
            goal_handle.abort()
            result.success = False
            return result

        if self.state != State.IDLE:
            goal_handle.reject()
            result.success = False
            return result

        cx = sum(p.x for p in points) / len(points)
        cy = sum(p.y for p in points) / len(points)
        frame = stockpile.header.frame_id or 'world'

        # self.get_logger().info(
        #     f'Stockpile centroid ({cx:.3f}, {cy:.3f}) [{frame}]; '
        #     f'parking at ({gx:.3f}, {cy:.3f}) with {self.stockpile_standoff:.2f} m '
        #     f'-x standoff, facing +x')

        goal = PoseStamped()
        goal.header.frame_id = frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = cx
        goal.pose.position.y = cy
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0   # yaw 0 -> facing world +x, toward stockpile

        pickup_pose = goal
        dropoff_pose = goal_handle.request.block.pose

        success = await self.execute_mission(pickup_pose,dropoff_pose)
        result.success = success
        self.get_logger().info(f'Mission execution result: {success}')

        if success:
            self.state = State.IDLE
            self.blocks_placed += 1
            goal_handle.succeed()
        else:
            self.state = State.IDLE
            goal_handle.abort()
        return result


    # action server callback

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Received transport mission')

        result = TransportBlock.Result()

        if self.state != State.IDLE:
            goal_handle.reject()
            result.success = False
            result.message = 'Robot is currently busy with another mission'
            return result

        pickup_pose = goal_handle.request.pickup_pose
        dropoff_pose = goal_handle.request.dropoff_pose

        success = await self.execute_mission(pickup_pose,dropoff_pose)
        result.success = success
        self.get_logger().info(f'Mission execution result: {success}')

        if success:
            result.message = 'Mission complete'
            self.state = State.IDLE
            goal_handle.succeed()
        else:
            result.message = 'Mission failed'
            self.state = State.IDLE
            goal_handle.abort()

        return result

    # main fsm mission execution 

    async def execute_mission(self, pickup_pose, dropoff_pose):

        # navigating to the block

        self.state = State.NAVIGATE_TO_PICKUP
        pickup_offset = self.offset_pose(pickup_pose, -self.x_offset, self.y_offset)

        success = await self.navigate_to_pose(pickup_offset)
        self.get_logger().info(f'Arrived at pickup location: {success}')
        if not success:
            success = await self.navigate_to_pose(pickup_pose)

        # Nav2 stops within its (coarse) goal tolerance; fine-tune the heading
        # in place via cmd_vel so the base faces the pickup centroid.
        await self.orient_to(pickup_pose)

        time.sleep(2)
        
        # Detect block
        self.state = State.DETECT_BLOCK
        arm_poses = await self.detect_blocks()

        if not arm_poses:
            return False

        # picking up the block (use the first detected block's arm-frame pose)
        self.state = State.PICK_BLOCK
        success = await self.pick_block(arm_poses[0])

        if not success:
            return False

        # navigate to drop off

        self.state = State.NAVIGATE_TO_DROPOFF
        dropoff_offset = self.offset_pose(dropoff_pose, self.x_offset, 0.0)
        dropoff_offset.pose.orientation.x = 0.0
        dropoff_offset.pose.orientation.y = 0.0
        dropoff_offset.pose.orientation.z = 0.0
        dropoff_offset.pose.orientation.w = 1.0
        success = await self.navigate_to_pose(dropoff_offset)

        if not success:
            success = await self.navigate_to_pose(dropoff_pose)
        if not success:
            return False

        # Nav2 stops within its (coarse) goal tolerance; fine-tune the heading
        # in place via cmd_vel so the base faces the block before placing.
        await self.orient_to(dropoff_pose)

        # place block

        self.state = State.PLACE_BLOCK
        success = await self.place_block(dropoff_pose)

        if not success:
            return False

        self.state = State.COMPLETE
        self.get_logger().info('Mission completed successfully')

        return True

    def _publish_cmd_vel(self, wz):
        """Publish an in-place rotation command on /<ns>/cmd_vel."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.angular.z = wz
        self.cmd_vel_pub.publish(msg)

    async def orient_to(self, target_pose, tol=0.03, kp=0.5,
                        max_wz=0.25, min_wz=0.05, max_dwz=0.04, timeout=20.0):
        """Rotate the base in place (via cmd_vel) to face target_pose.

        Closed loop on the world->base_link TF: turns until the base x-axis
        points at the target. Runs after Nav2 has stopped, so it owns the base.
        """
        # Resolve the target into the world frame.
        tgt = copy.deepcopy(target_pose)
        tgt.header.stamp.sec = 0
        tgt.header.stamp.nanosec = 0
        try:
            tgt = self.tf_buffer.transform(
                tgt, 'world', timeout=Duration(seconds=1.0))
        except TransformException as exc:
            self.get_logger().warn(
                f'orient_to: could not transform target to world: {exc}')
            return False
        tx, ty = tgt.pose.position.x, tgt.pose.position.y

        elapsed, dt, wz_cmd = 0.0, 0.1, 0.0
        while elapsed < timeout:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'world', 'base_link', rclpy.time.Time(),
                    timeout=Duration(seconds=1.0))
            except TransformException as exc:
                self.get_logger().warn(f'orient_to: TF lookup failed: {exc}')
                break

            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            desired = math.atan2(ty - tf.transform.translation.y,
                                 tx - tf.transform.translation.x)
            err = math.atan2(math.sin(desired - yaw), math.cos(desired - yaw))

            if abs(err) <= tol:
                self.get_logger().info(
                    f'orient_to: aligned (err {err:+.3f} rad)')
                break

            wz = max(-max_wz, min(max_wz, kp * err))
            if abs(wz) < min_wz:          # overcome stiction on tiny errors
                wz = math.copysign(min_wz, wz)
            # Slew-rate limit so the turn ramps smoothly instead of snapping.
            wz_cmd += max(-max_dwz, min(max_dwz, wz - wz_cmd))
            self._publish_cmd_vel(wz_cmd)
            await self._sleep(dt)
            elapsed += dt
        else:
            self.get_logger().warn('orient_to: timed out before aligning')

        self._publish_cmd_vel(0.0)        # always leave the base stopped
        return True

    async def navigate_to_pose(self, pose):

        pose.header.stamp.sec = 0
        pose.header.stamp.nanosec = 0

        self.get_logger().info(f'Navigating to: 'f'({pose.pose.position.x}, 'f'{pose.pose.position.y})')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        goal_handle = await self.nav_to_pose_client.send_goal_async(goal_msg)

        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return False

        result = await goal_handle.get_result_async()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
            return True

        self.get_logger().error('Navigation failed')

        return False

    async def pick_block(self, pick_pose):

        self.get_logger().info('Starting pick sequence')

        # open gripper

        success = await self.manipulator.move_gripper("open")

        if not success:
            return False

        # move to the detected block pose (arm_0_base_link frame, gripper down)
        pick_pose_above = copy.deepcopy(pick_pose)
        pick_pose_above.pose.position.z = 0.0

        success = await self.manipulator.move_to_pose(pick_pose_above)

        if not success:
            return False
        
        # move to the detected block pose (arm_0_base_link frame, gripper down)
        pick_pose_grasp = copy.deepcopy(pick_pose)
        pick_pose_grasp.pose.position.z = -0.13

        success = await self.manipulator.move_to_pose(pick_pose_grasp)

        if not success:
            return False

        # close the gripper

        success = await self.manipulator.move_gripper("close")

        if not success:
            return False

        pick_pose_lift = copy.deepcopy(pick_pose)
        pick_pose_lift.pose.position.z = 0.0

        success = await self.manipulator.move_to_pose(pick_pose_lift)

        if not success:
            return False


        # stow the arm

        stow_pose = self.manipulator.make_posestamped(self.manipulator.stow_pose)
        success = await self.manipulator.move_to_pose(stow_pose)

        return success

    async def place_block(self, place_pose):

        self.get_logger().info('Starting place sequence')

        # dropoff_pose arrives in the world frame; convert it to the arm base
        # frame before sending to the manipulator (which plans in arm_0_base_link).
        try:
            place_pose.header.stamp.sec = 0
            place_pose.header.stamp.nanosec = 0
            place_pose = self.tf_buffer.transform(
                place_pose,
                self.manipulator.reference_frame,
                timeout=Duration(seconds=1.0)
            )
        except TransformException as exc:
            self.get_logger().error(
                f"Could not transform dropoff pose from "
                f"'{place_pose.header.frame_id}' to "
                f"'{self.manipulator.reference_frame}': {exc}"
            )
            return False

        # Clip the place target into the arm's reachable box (in place).
        px = place_pose.pose.position.x
        py = place_pose.pose.position.y
        place_pose.pose.position.x = min(max(px, 0.35), 0.45)
        place_pose.pose.position.y = min(max(py, -0.10), 0.10)
        if (place_pose.pose.position.x, place_pose.pose.position.y) != (px, py):
            self.get_logger().warn(
                f'Clipped place target ({px:.3f}, {py:.3f}) -> '
                f'({place_pose.pose.position.x:.3f}, '
                f'{place_pose.pose.position.y:.3f})')

        # Remap the orientation
        o = place_pose.pose.orientation
        o.x, o.y, o.z, o.w = o.w, o.z, -o.y, -o.x

        # place block

        place_pose_above = copy.deepcopy(place_pose)
        place_pose_above.pose.position.z = 0.0
        success = await self.manipulator.move_to_pose(place_pose_above)

        if not success:
            return False


        place_pose_drop = copy.deepcopy(place_pose)
        place_pose_drop.pose.position.z = -0.10
        success = await self.manipulator.move_to_pose(place_pose_drop)

        if not success:
            return False

        # open gripper

        success = await self.manipulator.move_gripper("open")

        if not success:
            return False
        
        place_pose_lift = copy.deepcopy(place_pose)
        place_pose_lift.pose.position.z = 0.0
        success = await self.manipulator.move_to_pose(place_pose_lift)

        if not success:
            return False

        # stow the arm

        stow_pose = self.manipulator.make_posestamped(self.manipulator.stow_pose)
        success = await self.manipulator.move_to_pose(stow_pose)

        return success

def main(args=None):
    rclpy.init(args=args)
    node = RobotFSM()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()