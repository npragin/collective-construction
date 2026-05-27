from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy

def main():
    rclpy.init()
    navigator = BasicNavigator('basic_navigator')

    # Wait for Nav2 to be fully active
    navigator.waitUntilNav2Active()

    # Define your waypoints as PoseStamped messages
    waypoint_1 = PoseStamped()
    waypoint_1.header.frame_id = 'odom'
    waypoint_1.header.stamp = navigator.get_clock().now().to_msg()
    waypoint_1.pose.position.x = 2.0
    waypoint_1.pose.position.y = 3.0
    waypoint_1.pose.orientation.w = 1.0  # Facing forward

    waypoint_2 = PoseStamped()
    waypoint_2.header.frame_id = 'odom'
    waypoint_2.header.stamp = navigator.get_clock().now().to_msg()
    waypoint_2.pose.position.x = 4.0
    waypoint_2.pose.position.y = 1.0
    waypoint_2.pose.orientation.w = 1.0

    waypoints = [waypoint_1]

    # Send waypoints to the navigator
    navigator.followWaypoints(waypoints)

    # Monitor the progress
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        print(f'Executing waypoint: {feedback.current_waypoint}')

    # Handle the final result
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Waypoint following complete!')
    elif result == TaskResult.CANCELED:
        print('Waypoint following was canceled.')
    elif result == TaskResult.FAILED:
        print('Waypoint following failed.')

    navigator.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()