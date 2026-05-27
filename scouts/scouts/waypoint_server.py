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


        self.goal_list = [(0.2, 1.5),
                          (0.4, 1.5),
                          (0.4, 0.75)]

        self.goal_pose = (None, None)



        self.control_timer = self.create_timer(
            0.1,
            self.control_loop
        )
            



    def control_loop(self):
    
        if None in self.robot:
            return
    
        distance = float('inf')
        for goal in self.goal_list:

            self.goal_pose = goal

            while distance > 0.05:
                self.get_logger().info(f'goal: {goal}')
                self.get_logger().info(f'robot: [{self.robot[0]}, {self.robot[1]}, {self.robot[2]}')
                dx = self.goal_pose[0] - self.robot[0]
                dy = self.goal_pose[1] - self.robot[1]
            
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
            
                msg.linear.x = min(0.3, distance)
                msg.angular.z = heading_error * 0.5
            
                self.cmd_pub.publish(msg)
                self.get_logger().info('---------------------')

            msg = Twist()  # all zeros
            self.cmd_pub.publish(msg)

            # self.control_timer.cancel()
            
            self.get_logger().info(f'Goal {goal} reached!')
            





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

    executer = MultiThreadedExecutor(num_threads=4)
    executer.add_node(node)
    executer.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()