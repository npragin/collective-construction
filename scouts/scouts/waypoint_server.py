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
from rclpy.executors import MultiThreadedExecutor

class WaypointServer(Node):

    def __init__(self):
        super().__init__('waypoint_server')

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.timer = self.create_timer(
            0.1,
            self.lookup_transform
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10,
        )

        self.robot = [None, None, None] # x, y, theta


        self.goal_list = [(0.2, 3.0),
                          (0.8, 3.0),
                          (0.8, 0.4),
        ]
                        #   (0.6, 0.25),
                        #   (1.0, 0.25),
                        #   (1.0, 3.0),
                        #   (1.4, 3.0)]

        self.goal_idx = 0
        self.goal = self.goal_list[self.goal_idx]

        self.control_timer = self.create_timer(
            0.1,
            self.control_loop
        )

        

    def control_loop(self):
    
        if None in self.robot:
            return
        
        self.get_logger().info(f'goal: {self.goal}')
        self.get_logger().info(f'robot: [{self.robot[0]}, {self.robot[1]}, {self.robot[2]}')
        dx = self.goal[0] - self.robot[0]
        dy = self.goal[1] - self.robot[1]
    
        distance = np.hypot(dx, dy)
        self.get_logger().info(f'distance: {distance}')
    
        desired_heading = np.arctan2(dy, dx)
        self.get_logger().info(f'desired_heading: {desired_heading}')

        heading_error = desired_heading - self.robot[2]
    
        heading_error = np.arctan2(
            np.sin(heading_error),
            np.cos(heading_error)
        )
    
        msg = Twist()
    
        msg.linear.x = min(0.1, distance) * 1
        msg.angular.z = heading_error * 0.8
    
        self.cmd_pub.publish(msg)
        self.get_logger().info('---------------------')

        if distance < 0.15:
            msg = Twist()  # all zeros
            self.cmd_pub.publish(msg)

            self.get_logger().info(f'Goal {self.goal} reached!')
            self.goal_idx += 1

            if self.goal_idx >= len(self.goal_list):
                self.control_timer.cancel()
                self.get_logger().info(f'All goals are completed!')
                return

            self.goal = self.goal_list[self.goal_idx]

            





    def lookup_transform(self):

        try:
            self.get_logger().info("Getting robot pose ...")
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

            self.get_logger().info(
                f"x={x:.2f}, y={y:.2f}, yaw quaternion=({qx:.2f}, {qy:.2f}, {qz:.2f}, {qw:.2f})"
            )

            self.robot[0] = x
            self.robot[1] = y
            self.robot[2] = euler_from_quaternion([qx, qy, qz, qw])[2]

        except TransformException as ex:
            self.get_logger().warn(f"Could not transform: {ex}")



def main():
    rclpy.init()
    node = WaypointServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()