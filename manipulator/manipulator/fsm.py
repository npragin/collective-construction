import numpy
from enum import Enum, auto
import asyncio
import copy

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient

from geometry_msgs.msg import PoseStamped

from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from manipulator_interface.action import TransportBlock
from manipulator_interface.action import AbsoluteMove
from control_msgs.action import GripperCommand

class State(Enum):

    IDLE = auto()
    NAVIGATE_TO_PICKUP = auto()
    PICK_BLOCK = auto()
    NAVIGATE_TO_DROPOFF = auto()
    PLACE_BLOCK = auto()
    COMPLETE = auto()
    ERROR = auto()

# manipulation (Jared's pick code edited)

class Manipulator:

    def __init__(self, node):
        
        self.node = node
        self.namespace = "j100_0897"
        self.target_frame = "arm_0_end_effector_link"
        self.reference_frame = "arm_0_base_link"

        self.absolute_move_client = ActionClient(node, AbsoluteMove,'absolute_move')

        self.gripper_client = ActionClient(node,GripperCommand,f'/{self.namespace}/manipulators/arm_0_gripper_controller/gripper_cmd')

        node.get_logger().info('Waiting for manipulation action servers...')

        self.absolute_move_client.wait_for_server()
        self.gripper_client.wait_for_server()

        node.get_logger().info('Manipulator connected!')

        self.pick_pose = [0.46217,-0.030112,0.13923,-0.055763,0.9973,0.0073431,0.04721]

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


        self.navigator = BasicNavigator(namespace=self.namespace)
        self.get_logger().info('Waiting for Nav2...')
        self.navigator.waitUntilNav2Active(localizer="bt_navigator")

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, f'/{self.namespace}/navigate_to_pose')
        self.nav_to_pose_client.wait_for_server()

        self.manipulator = Manipulator(self)
        self.manipulator.namespace = self.manipulator_namespace
        self.state = State.IDLE

        self.action_server = ActionServer(self,TransportBlock,'transport_block',execute_callback=self.execute_callback)
        self.get_logger().info('Robot FSM Ready')

        self.x_offset = 1.0 #meters
        self.y_offset = 1.0 #meters

    def offset_pose(self, pose, dx=0.0, dy=0.0):
        new_pose = copy.deepcopy(pose)
        new_pose.pose.position.x += dx
        new_pose.pose.position.y += dy

        return new_pose

    # action server callback

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Received transport mission')

        if self.state != State.IDLE:
            goal_handle.reject()
            return TransportBlock.Result()

        pickup_pose = goal_handle.request.pickup_pose
        dropoff_pose = goal_handle.request.dropoff_pose

        success = await self.execute_mission(pickup_pose,dropoff_pose)
        result = TransportBlock.Result()
        result.success = success

        if success:
            result.message = 'Mission complete'
            goal_handle.succeed()

        else:
            result.message = 'Mission failed'
            goal_handle.abort()

        return result

    # main fsm mission execution 

    async def execute_mission(self, pickup_pose, dropoff_pose):

        # navigating to the block

        self.state = State.NAVIGATE_TO_PICKUP
        success = await self.navigate_to_pose(pickup_pose)

        if not success:
            return False
        
        # picking up the block

        self.state = State.PICK_BLOCK
        success = await self.pick_block()

        if not success:
            return False

        # navigate to drop off

        self.state = State.NAVIGATE_TO_DROPOFF
        dropoff_offset = self.offset_pose(dropoff_pose, self.x_offset, self.y_offset)
        success = await self.navigate_to_pose(dropoff_offset)

        if not success:
            return False

        # place block

        self.state = State.PLACE_BLOCK
        success = await self.place_block()

        if not success:
            return False

        self.state = State.COMPLETE
        self.get_logger().info('Mission completed successfully')

        return True

    async def navigate_to_pose(self, pose):

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

    async def pick_block(self):

        self.get_logger().info('Starting pick sequence')

        # open gripper

        success = await self.manipulator.move_gripper("open")

        if not success:
            return False

        # move to pick pose

        pick_pose = self.manipulator.make_posestamped(self.manipulator.pick_pose)
        success = await self.manipulator.move_to_pose(pick_pose)

        if not success:
            return False

        # close the gripper

        success = await self.manipulator.move_gripper("close")

        if not success:
            return False

        # stow the arm

        stow_pose = self.manipulator.make_posestamped(self.manipulator.stow_pose)
        success = await self.manipulator.move_to_pose(stow_pose)

        return success

    async def place_block(self):

        self.get_logger().info('Starting place sequence')

        # place block

        pick_pose = self.manipulator.make_posestamped(self.manipulator.pick_pose)
        success = await self.manipulator.move_to_pose(pick_pose)

        if not success:
            return False

        # open gripper

        success = await self.manipulator.move_gripper("open")

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