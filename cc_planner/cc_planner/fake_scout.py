# Scout Node for Testing
# Simulates scout robots discovering blocks and reporting their locations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
import time
import random


class ScoutStubNode(Node):
    def __init__(self):
        super().__init__('scout_stub')
        self.get_logger().info('Scout Stub Node has been started.')

        # Create publisher for block discoveries
        self.block_discovery_pub = self.create_publisher(String, 'scout/block_discovered', 10)

        # Simulated blocks to discover
        self.blocks_to_discover = [
            {'block_type': 'small', 'x': 1.0, 'y': 2.0, 'z': 0.0},
            {'block_type': 'medium', 'x': 2.0, 'y': 2.0, 'z': 0.0},
            {'block_type': 'large', 'x': 3.0, 'y': 2.0, 'z': 0.0},
            {'block_type': 'small', 'x': 1.0, 'y': 3.0, 'z': 0.0},
            {'block_type': 'medium', 'x': 2.0, 'y': 3.0, 'z': 0.0},
        ]

        self.discovered_index = 0

        # Create timer to discover blocks periodically
        self.timer = self.create_timer(2.0, self.discover_block)

    def discover_block(self):
        """Simulate discovering a block."""
        if self.discovered_index >= len(self.blocks_to_discover):
            self.get_logger().info('All blocks discovered!')
            self.timer.cancel()
            return

        block = self.blocks_to_discover[self.discovered_index]
        self.discovered_index += 1

        # Create and publish discovery message
        msg = String()
        msg.data = (
            f"block_type:{block['block_type']}|"
            f"x:{block['x']}|"
            f"y:{block['y']}|"
            f"z:{block['z']}"
        )

        self.block_discovery_pub.publish(msg)
        self.get_logger().info(f'Discovered block: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    scout_stub = ScoutStubNode()
    rclpy.spin(scout_stub)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
