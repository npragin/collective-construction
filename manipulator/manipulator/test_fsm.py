import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import asyncio

from manipulator_interface.action import TransportBlock



class TestFSM(Node):

    def __init__(self):
        super().__init__('test_FSM')
        self.fsm_action = ActionClient(self, TransportBlock, 'transport_block')
        self.get_logger().info('Waiting for transport_block action server...')
        self.fsm_action.wait_for_server()


        self.pick = PoseStamped()
        self.pick.header.frame_id = 'odom'
        self.pick.header.stamp = self.get_clock().now().to_msg()
        self.pick.pose.position.x = 1.0
        self.pick.pose.position.y = 0.0
        self.pick.pose.orientation.w = 1.0  # Facing forward

        self.place = PoseStamped()
        self.place.header.frame_id = 'odom'
        self.place.header.stamp = self.get_clock().now().to_msg()
        self.place.pose.position.x = 0.0
        self.place.pose.position.y = 1.0
        self.place.pose.orientation.w = 1.0  # Facing forward

        self.send_test_goal()

    def send_test_goal(self):
        goal_msg = TransportBlock.Goal()
        # Fill in the goal message as needed
        self.get_logger().info('Sending test goal...')
        self.pick.header.stamp = self.get_clock().now().to_msg()
        self.place.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pickup_pose = self.pick
        goal_msg.dropoff_pose = self.place
        send_goal_future = self.fsm_action.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == 4:  # SUCCEEDED
            self.get_logger().info('Action succeeded')
            return

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.current_state}')

    

def main(args=None):
    rclpy.init(args=args)

    test_FSM = TestFSM()

    rclpy.spin(test_FSM)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test_FSM.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()