"""
Fake retriever action server: auto-accepts and immediately succeeds RetrievalTask goals.

Reads robots.yaml from cc_planner's share directory and stands up one action server
per retriever-capability robot at `<robot_id>/retrieval_task`. Useful for smoke
testing the planner's retrieval allocation path without real robots.
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from cc_interfaces.action import RetrievalTask
from rclpy.action import ActionServer
from rclpy.node import Node


class FakeRetrieverNode(Node):
    def __init__(self) -> None:
        super().__init__("fake_retriever")

        self.declare_parameter("retrieval_action_name", "retrieval_task")
        action_base = self.get_parameter("retrieval_action_name").get_parameter_value().string_value

        config_path = Path(get_package_share_directory("cc_planner")) / "config" / "robots.yaml"
        with config_path.open() as f:
            robots = yaml.safe_load(f)["robots"]

        self._servers: list[ActionServer] = []
        for robot in robots:
            if robot["capability"] != "retriever":
                continue
            robot_id = robot["identifier"]
            action_name = f"{robot_id}/{action_base}"
            server = ActionServer(
                self,
                RetrievalTask,
                action_name,
                execute_callback=self._make_execute_callback(robot_id),
            )
            self._servers.append(server)
            self.get_logger().info(f"Fake retriever ready: {action_name}")

    def _make_execute_callback(self, robot_id: str):
        def execute(goal_handle):
            block = goal_handle.request.block
            self.get_logger().info(
                f"{robot_id} received RetrievalTask: type={block.type} -> "
                f"stockpile_frame={goal_handle.request.stockpile.header.frame_id}; auto-succeeding"
            )
            goal_handle.succeed()
            result = RetrievalTask.Result()
            result.success = True
            return result

        return execute


def main() -> None:
    rclpy.init()
    node = FakeRetrieverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
