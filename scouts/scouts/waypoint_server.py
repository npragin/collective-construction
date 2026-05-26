#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer


from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException


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


    def lookup_transform(self):

        try:
            transform = self.tf_buffer.lookup_transform(
                'sierra/map',        # target frame
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

        except TransformException as ex:
            self.get_logger().warn(f"Could not transform: {ex}")



def main():
    rclpy.init()

    node = WaypointServer()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()