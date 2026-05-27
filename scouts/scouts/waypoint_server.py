#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

from tf_transformations import euler_from_quaternion

import numpy as np

from geometry_msgs.msg import Twist

class WaypointServer(Node):

    def __init__(self):
        super().__init__('waypoint_server')

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.timer = self.create_timer(
            1.0,
            self.lookup_transform
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            'sierra/cmd_vel',
            10,
        )

        self.robot = [None, None, None] # x, y, theta

        self.goal([5.0, 5.0, np.pi])



    def goal(self, goal_pose):

        distance = float('inf')
        heading_error = np.pi

        while distance > 0.1 or heading_error > 0.5:

            dy = goal_pose[1] - self.robot[1]
            dx = goal_pose[0] - self.robot[0]

            distance = np.sqrt(dx*dx + dy*dy)

            desired_heading = np.atan2(dy, dx)

            heading_error = desired_heading - self.robot[2]

            heading_error = np.atan2(np.sin(heading_error, np.cos(heading_error)))
            self.get_logger().info(f'heading_error: {heading_error}')

            msg = Twist()
            msg.linear.x = min(0.3, distance)
            msg.angular.z = heading_error

            self.cmd_pub.publish(msg)
        
        self.get_logger().info(f'Goal {goal_pose} reached!')





    def lookup_transform(self):

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',        # target frame
                'sierra/base_link',  # source frame
                rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            # self.get_logger().info(
            #     f"x={x:.2f}, y={y:.2f}, yaw quaternion=({qx:.2f}, {qy:.2f}, {qz:.2f}, {qw:.2f})"
            # )

            self.robot[0] = x
            self.robot[1] = y
            self.robot[2] = euler_from_quaternion([qx, qy, qz, qw])[2]

        except TransformException as ex:
            self.get_logger().warn(f"Could not transform: {ex}")



def main():
    rclpy.init()

    node = WaypointServer()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()