"""Central planner node for the collective-construction system."""

from enum import Enum
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from cc_interfaces.action import ManipulationTask, RetrievalTask
from cc_interfaces.msg import Block
from geometry_msgs.msg import PolygonStamped
from rclpy.action import ActionClient
from rclpy.node import Node


class RobotStatus(Enum):
    IDLE = "idle"
    TASKED = "tasked"


class PlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("planner")

        self.declare_parameter("scout_report_topic", "scout_report")
        self.declare_parameter("retrieval_action_name", "retrieval_task")
        self.declare_parameter("manipulation_action_name", "manipulation_task")
        self.declare_parameter("action_server_timeout", 5.0)

        scout_report_topic = self.get_parameter("scout_report_topic").get_parameter_value().string_value
        self._retrieval_action_name = self.get_parameter("retrieval_action_name").get_parameter_value().string_value
        self._manipulation_action_name = (
            self.get_parameter("manipulation_action_name").get_parameter_value().string_value
        )
        self._action_server_timeout = self.get_parameter("action_server_timeout").get_parameter_value().double_value

        config_path = Path(get_package_share_directory("cc_planner")) / "config" / "robots.yaml"
        with config_path.open() as f:
            robots = yaml.safe_load(f)["robots"]

        self.robot_status: dict[str, RobotStatus] = {robot["identifier"]: RobotStatus.IDLE for robot in robots}

        self.reported_blocks: list[Block] = []
        self._scout_subs = []
        self._action_clients: dict[str, ActionClient] = {}
        for robot in robots:
            robot_id = robot["identifier"]
            capability = robot["capability"]
            if capability == "scout":
                topic = f"{robot_id}/{scout_report_topic}"
                sub = self.create_subscription(Block, topic, self._on_scout_report, 10)
                self._scout_subs.append(sub)
                self.get_logger().info(f"Subscribed to {topic}")
            elif capability == "retriever":
                action_name = f"{robot_id}/{self._retrieval_action_name}"
                self._action_clients[robot_id] = ActionClient(self, RetrievalTask, action_name)
            elif capability == "manipulator":
                action_name = f"{robot_id}/{self._manipulation_action_name}"
                self._action_clients[robot_id] = ActionClient(self, ManipulationTask, action_name)

    def _on_scout_report(self, msg: Block) -> None:
        self.reported_blocks.append(msg)

    def send_retrieval_task(self, robot_id: str, block: Block, stockpile: PolygonStamped) -> None:
        client = self._action_clients[robot_id]
        if not client.wait_for_server(timeout_sec=self._action_server_timeout):
            self.get_logger().error(f"Retrieval action server for {robot_id} unavailable")
            return

        goal = RetrievalTask.Goal()
        goal.block = block
        goal.stockpile = stockpile

        send_future = client.send_goal_async(goal)
        send_future.add_done_callback(lambda f: self._on_goal_response(f, robot_id))
        self.robot_status[robot_id] = RobotStatus.TASKED

    def send_manipulation_task(self, robot_id: str, block: Block, stockpile: PolygonStamped) -> None:
        client = self._action_clients[robot_id]
        if not client.wait_for_server(timeout_sec=self._action_server_timeout):
            self.get_logger().error(f"Manipulation action server for {robot_id} unavailable")
            return

        goal = ManipulationTask.Goal()
        goal.block = block
        goal.stockpile = stockpile

        send_future = client.send_goal_async(goal)
        send_future.add_done_callback(lambda f: self._on_goal_response(f, robot_id))
        self.robot_status[robot_id] = RobotStatus.TASKED

    def _on_goal_response(self, future, robot_id: str) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"Goal rejected by {robot_id}")
            self.robot_status[robot_id] = RobotStatus.IDLE
            return
        goal_handle.get_result_async().add_done_callback(lambda f: self._on_task_result(f, robot_id))

    def _on_task_result(self, future, robot_id: str) -> None:
        result = future.result().result
        self.get_logger().info(f"{robot_id} task complete: success={result.success}")
        self.robot_status[robot_id] = RobotStatus.IDLE


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
