import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient, ActionServer


from manipulator_interface.action import AbsoluteMove
from geometry_msgs.msg import TwistStamped, Twist, PointStamped, Quaternion, Pose, PoseStamped

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from control_msgs.action import GripperCommand


class Pick(Node):

    def __init__(self):
        super().__init__('pick')

        self.declare_parameter('namespace', "j100_0897")
        self.namespace = self.get_parameter("namespace").value
        self.get_logger().info(f"Using namespace: {self.namespace}")

        self.absolute_move_client = ActionClient(self, AbsoluteMove, 'absolute_move')
        self.get_logger().info('Waiting for AbsoluteMove action server...')
        self.absolute_move_client.wait_for_server()
        self.get_logger().info('AbsoluteMove action server connected!')

        self.get_logger().info(f"Gripper action:  /{self.namespace}/arm_0_gripper_controller/gripper_cmd")

        self.gripper_client = ActionClient(self, GripperCommand, f'/{self.namespace}/arm_0_gripper_controller/gripper_cmd')
        self.get_logger().info('Waiting for GripperCommand action server...')
        self.gripper_client.wait_for_server()
        self.get_logger().info('GripperCommand action server connected!')
        
        self.time = self.create_timer(0.03, self.timer_callback)

        #state params
        self.state = "start"
        self.latest_joint_state = None
        self.move_success = False
        self.sent_goal = False
        self.servo = True

        self.target_frame = "arm_0_end_effector_link"
        self.reference_frame = "arm_0_base_link"


        self.gripper_offset = 0.335
        self.test_pose = [0.46217, -0.030112, 0.13923, -0.055763, 0.9973, 0.0073431, 0.04721]
        self.stow_pose = [0.21218, -0.075709, 0.41342, 0.72917, 0.016382, 0.68135, -0.061708]



    def timer_callback(self):
        if self.state == "start":
            if not self.sent_goal:
                self.move_gripper("open")
            if self.move_success:
                self.state = "Test"
                self.get_logger().info("Attemped to open gripper")
                self.sent_goal = False

        if self.state == "Test":
            if not self.sent_goal:
                move_pose = self.make_posestamped(self.test_pose)
                self.move_to_absolute_pose(move_pose, "ompl")
            if self.move_success:
                self.state = "close"
                self.get_logger().info("Tested the command above")
                self.sent_goal = False
        if self.state == "close":
            if not self.sent_goal:
                self.move_gripper("close")
            if self.move_success:
                self.state = "stow"
                self.get_logger().info("Attemped to close gripper")
                self.sent_goal = False

        if self.state == "stow":
            if not self.sent_goal:
                move_pose = self.make_posestamped(self.stow_pose)
                self.move_to_absolute_pose(move_pose, "ompl")
            if self.move_success:
                self.state = "open"
                self.get_logger().info("Tested the command above")
                self.sent_goal = False
        if self.state == "open":
            if not self.sent_goal:
                self.move_gripper("open")
            if self.move_success:
                self.state = "stall"
                self.get_logger().info("Attemped to open gripper")
                self.sent_goal = False


    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected!')
            return
            
        self.get_logger().info('Goal accepted!')
        
        # Get the result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status        
        if status == 4:  # Succeeded
            self.get_logger().info('Motion execution succeeded!')
            self.move_success = True
        else:
            self.get_logger().error(f'Motion execution failed with status: {status}')
            self.move_success = False
    
    def move_to_absolute_pose(self, pose, planner = "pilz", pilz_planner = "PTP"):
        """Sends a goal to absolute move action server.

        Parameters
        ----------
        pose : PoseStamped
            The pose to move the EE in the base frame.
        """
        self.move_success = False

        self.get_logger().info(f"sent pose")
        goal_msg = AbsoluteMove.Goal()
        goal_msg.pose = pose
        goal_msg.planner = planner
        goal_msg.pilz_planner = pilz_planner
        goal_msg.target_frame = self.target_frame
        send_goal_future = self.absolute_move_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.sent_goal = True

    def move_gripper(self, direction = "open"):
        """Sends a goal to the gripper control action server.

        Parameters
        ----------
        direction : {'open', 'close'}
            The direction to move given as ['open', 'close'].
        """
        
        self.move_success = False
        self.get_logger().info(f"sent Gripper command: {direction}")
        goal_msg = GripperCommand.Goal()
        if direction == "close":
            goal_msg.command.position = 0.85
        else: 
            goal_msg.command.position = 0.0
        send_goal_future = self.gripper_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.sent_goal = True


        
    def make_posestamped(self, pose_arr):
        """Build a pose stamped message from an array.

        Parameters
        ----------
        pose_arr : list
            A list of pose values [x, y, z, qx, qy, qz, qw].

        Returns
        -------
        pose : PoseStamped
            The constructed PoseStamped message.
        
        Notes
        -----
        A header frame id should be added as a parameter.
        """
        pose = PoseStamped()
        pose.header.frame_id = self.reference_frame
        #set array to x,y,z pose positions
        pose.pose.position.x = pose_arr[0]
        pose.pose.position.y = pose_arr[1]
        pose.pose.position.z = pose_arr[2]
        #Set pose at 90 degree rotation along x
        pose.pose.orientation.x = pose_arr[3]
        pose.pose.orientation.y = pose_arr[4]
        pose.pose.orientation.z = pose_arr[5]
        pose.pose.orientation.w = pose_arr[6]  # Neutral orientation
        return pose


def main(args=None):
    rclpy.init(args=args)

    pick = Pick()

    rclpy.spin(pick)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pick.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()