import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints

class Nav2ActionClient(Node):
    def __init__(self):
        super().__init__('nav2_action_client')
        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        # self.marker_pub = self.create_publisher(MarkerArray, 'waypoint_markers', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'waypoints', 10)


        self.points = [
            (2.0, 7.0), (5.0, 7.0), (5.0, 18.0),
            (8.0, 18.0), (8.0, 7.0),
            (11.0, 7.0), (11.0, 18.0),
            (14.0, 18.0), (14.0, 7.0),
            (17.0, 2.0), (17.0, 18.0),
            (20.0, 18.0), (20.0, 2.0),
        ]
        self.last_waypoint_index = -1

        self.publish_markers(set(range(len(self.points))))
        self.send_goal()

    def publish_markers(self, active_indices):
        marker_array = MarkerArray()
        for i, (x, y) in enumerate(self.points):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            # this is how we remove old markers
            marker.action = Marker.ADD if i in active_indices else Marker.DELETE
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 0.4
            marker.color.r = 1.0
            marker.color.g = 0.4
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def send_goal(self):
        goal_msg = FollowWaypoints.Goal()
        for x, y in self.points:
            waypoint = PoseStamped()
            waypoint.header.frame_id = 'map'
            waypoint.pose.position.x = x
            waypoint.pose.position.y = y
            waypoint.pose.orientation.w = 1.0
            goal_msg.poses.append(waypoint)

        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        current = feedback_msg.feedback.current_waypoint
        if current != self.last_waypoint_index:
            self.get_logger().info(f'Reached waypoint {current}')
            self.last_waypoint_index = current
            # removes already visited waypoints
            active = set(range(current, len(self.points)))
            self.publish_markers(active)

    def result_callback(self, future):
        self.get_logger().info('All waypoints completed, clearing markers')
        self.publish_markers(set())  # Delete all remaining


def main() -> None:
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