"""Central planner node for the collective-construction system."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from cc_interfaces.msg import Block
from rclpy.node import Node


class PlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("planner")

        self.declare_parameter("scout_report_topic", "scout_report")

        scout_report_topic = self.get_parameter("scout_report_topic").get_parameter_value().string_value

        config_path = Path(get_package_share_directory("cc_planner")) / "config" / "robots.yaml"
        with config_path.open() as f:
            robots = yaml.safe_load(f)["robots"]

        self.reported_blocks: list[Block] = []
        self._scout_subs = []
        for robot in robots:
            if robot["capability"] != "scout":
                continue
            topic = f"{robot['identifier']}/{scout_report_topic}"
            sub = self.create_subscription(Block, topic, self._on_scout_report, 10)
            self._scout_subs.append(sub)
            self.get_logger().info(f"Subscribed to {topic}")

    def _on_scout_report(self, msg: Block) -> None:
        self.reported_blocks.append(msg)


def main() -> None:
    rclpy.init()
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
