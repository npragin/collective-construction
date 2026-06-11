#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from cc_interfaces.srv import LogPickedTag


class PickedTagLogger(Node):
    def __init__(self):
        super().__init__('picked_tag_logger')

        self.tag_history = {}

        self.service = self.create_service(
            LogPickedTag,
            '/log_picked_tag',
            self.log_picked_tag_callback
        )

        self.get_logger().info('Picked tag logger started.')
        self.get_logger().info('Service available: /log_picked_tag')

    def log_picked_tag_callback(self, request, response):
        aruco_id = request.aruco_id
        dropoff_pose = request.dropoff_pose

        self.tag_history[aruco_id] = dropoff_pose

        self.get_logger().info(
            f'Logged AprilTag ID {aruco_id} with dropoff pose: '
            f'frame={dropoff_pose.header.frame_id}, '
            f'x={dropoff_pose.pose.position.x:.3f}, '
            f'y={dropoff_pose.pose.position.y:.3f}, '
            f'z={dropoff_pose.pose.position.z:.3f}'
        )

        response.success = True
        response.message = f'Logged AprilTag ID {aruco_id}.'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PickedTagLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
