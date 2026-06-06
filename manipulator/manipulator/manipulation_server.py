"""ManipulationTask action server that navigates the base to the goal waypoint."""

from action_msgs.msg import GoalStatus

from cc_interfaces.action import ManipulationTask

from geometry_msgs.msg import PoseStamped

from nav2_msgs.action import NavigateToPose

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node


class ManipulationTaskServer(Node):
    """Serve ManipulationTask by navigating the base to the stockpile."""

    def __init__(self):
        super().__init__('manipulation_task_server')

        # Namespace of the robot whose Nav2 we drive.
        self.declare_parameter('robot_namespace', 'j100_0897')
        # Namespace the action is advertised under (matches the planner's
        # per-robot identifier, e.g. manipulator_1).
        self.declare_parameter('task_namespace', 'manipulator_1')
        self.declare_parameter('stockpile_standoff', 0.5)
        robot_ns = self.get_parameter('robot_namespace').value
        task_ns = self.get_parameter('task_namespace').value
        self.stockpile_standoff = self.get_parameter('stockpile_standoff').value

        self.nav_client = ActionClient(
            self, NavigateToPose, f'/{robot_ns}/navigate_to_pose')
        self.get_logger().info(
            f'Waiting for Nav2 at /{robot_ns}/navigate_to_pose ...')
        self.nav_client.wait_for_server()

        self.action_server = ActionServer(
            self, ManipulationTask, f'/{task_ns}/manipulation_task',
            execute_callback=self.execute_callback)
        self.get_logger().info(
            f'ManipulationTask server ready on /{task_ns}/manipulation_task')

    async def execute_callback(self, goal_handle):
        """Navigate to the stockpile centroid and report success."""
        stockpile = goal_handle.request.stockpile
        points = stockpile.polygon.points

        result = ManipulationTask.Result()
        if not points:
            self.get_logger().error('Stockpile polygon has no points')
            goal_handle.abort()
            result.success = False
            return result

        cx = sum(p.x for p in points) / len(points)
        cy = sum(p.y for p in points) / len(points)
        frame = stockpile.header.frame_id or 'world'

        # Park in -x of the centroid, facing +x toward the stockpile.
        gx = cx - self.stockpile_standoff
        self.get_logger().info(
            f'Stockpile centroid ({cx:.3f}, {cy:.3f}) [{frame}]; '
            f'parking at ({gx:.3f}, {cy:.3f}) with {self.stockpile_standoff:.2f} m '
            f'-x standoff, facing +x')

        goal = PoseStamped()
        goal.header.frame_id = frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = gx
        goal.pose.position.y = cy
        goal.pose.orientation.w = 1.0  # yaw 0 -> facing world +x, toward stockpile

        success = await self.navigate_to(goal)
        result.success = success
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    async def navigate_to(self, pose):
        """Send a NavigateToPose goal and await the result."""
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = pose

        goal_handle = await self.nav_client.send_goal_async(nav_goal)
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return False

        result = await goal_handle.get_result_async()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
            return True

        self.get_logger().error(f'Navigation failed (status {result.status})')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = ManipulationTaskServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
