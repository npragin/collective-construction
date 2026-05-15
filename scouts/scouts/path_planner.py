import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints

class Nav2ActionClient(Node):

    def __init__(self):
        super().__init__('nav2_action_client')
        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        
    

