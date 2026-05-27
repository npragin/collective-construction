# Manipulator Node for Testing
# Simulates manipulator robot accepting and completing placement tasks

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from cc_interfaces.action import RetrieverTask
import time


class ManipulatorStubNode(Node):
    def __init__(self, robot_id='manipulator'):
        super().__init__(f'{robot_id}_stub')
        self.robot_id = robot_id
        self.get_logger().info(f'Manipulator Node {robot_id} has been started.')

        # Create action server for receiving tasks
        self.action_server = ActionServer(
            self,
            RetrieverTask,
            f'{robot_id}/manipulator_task',
            self.execute_placement_task
        )

        self.current_task_id = None

    async def execute_placement_task(self, goal_handle):
        """Execute a placement task."""
        request = goal_handle.request
        self.current_task_id = request.task_id

        self.get_logger().info(f'{self.robot_id} accepted task: {request.task_id}')

        # Simulate task execution with periodic feedback
        result = RetrieverTask.Result()

        try:
            # Simulate navigating to stockpile
            self.get_logger().info(f'{self.robot_id} navigating to stockpile...')
            feedback_msg = RetrieverTask.Feedback()
            feedback_msg.status = 'navigating_to_stockpile'
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

            # Simulate picking block
            self.get_logger().info(f'{self.robot_id} picking block...')
            feedback_msg.status = 'picking'
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

            # Simulate navigating to build site
            self.get_logger().info(f'{self.robot_id} navigating to build site...')
            feedback_msg.status = 'navigating_to_build_site'
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

            # Simulate placing block
            self.get_logger().info(f'{self.robot_id} placing block...')
            feedback_msg.status = 'placing'
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

            # Task completed
            result.success = True
            result.message = f'Successfully placed block from task {request.task_id}'
            self.get_logger().info(f'{self.robot_id} task completed: {request.task_id}')

        except Exception as e:
            result.success = False
            result.message = f'Error: {str(e)}'
            self.get_logger().error(f'{self.robot_id} task failed: {e}')

        goal_handle.succeed()
        return result


def main(args=None):
    import sys
    robot_id = sys.argv[1] if len(sys.argv) > 1 else 'manipulator'

    rclpy.init(args=args)
    manipulator_stub = ManipulatorStubNode(robot_id)
    rclpy.spin(manipulator_stub)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
