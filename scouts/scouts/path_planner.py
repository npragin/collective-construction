import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, TransformStamped, Pose, PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints

import numpy as np

class Nav2ActionClient(Node):

    def __init__(self):
        super().__init__('nav2_action_client')

        

        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.send_goal()

    def send_goal(self):
        goal_msg = FollowWaypoints.Goal()
        
        waypoint = PoseStamped()
        waypoint.header.frame_id = 'map'
        waypoint.pose.position.x = 2.0
        waypoint.pose.position.y = 10.0
        waypoint.pose.orientation.w = 1.0
        goal_msg.poses = [waypoint]
        
        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        return future

    def feedback_callback(self, feedback_msg):
            self.get_logger().info(f'Received feedback: {feedback_msg.feedback}')

def main() -> None:
    """Spin the path_planner until interrupted."""
    rclpy.init()
    node = Nav2ActionClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
