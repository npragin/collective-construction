"""Central planner node for the collective-construction system."""

from enum import Enum
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from cc_interfaces.action import ManipulationTask, RetrievalTask
from cc_interfaces.msg import Block, Stockpiles
from geometry_msgs.msg import Polygon, PolygonStamped
from rclpy.action import ActionClient
from rclpy.node import Node


def _polygon_centroid_xy(polygon: Polygon) -> tuple[float, float]:
    """Arithmetic mean of polygon points' xy. Caller guarantees points is non-empty."""
    n = len(polygon.points)
    sx = sum(p.x for p in polygon.points)
    sy = sum(p.y for p in polygon.points)
    return (sx / n, sy / n)


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
        self.robot_capabilities: dict[str, str] = {robot["identifier"]: robot["capability"] for robot in robots}

        # Every stockpile the planner has ever seen, keyed by aruco tag id.
        # Polygon updates in place when re-detected; entries are never removed.
        self.stockpiles: dict[int, PolygonStamped] = {}
        # Block type (Block.TYPE_*) -> tag id of the bound stockpile. Permanent once set.
        self.type_to_stockpile: dict[int, int] = {}
        # Reported blocks whose type is bound but which haven't been retrieved yet.
        self.reported_blocks: list[Block] = []
        # Reported blocks whose type couldn't be bound (no unassigned stockpile).
        self.pending_unbound: list[Block] = []
        self._stockpiles_sub = self.create_subscription(Stockpiles, "stockpile_polygons", self._on_stockpiles, 10)
        self._scout_subs = []
        self._action_clients: dict[str, ActionClient] = {}
        for robot_id, capability in self.robot_capabilities.items():
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

    def _on_stockpiles(self, msg: Stockpiles) -> None:
        """Upsert detected stockpiles by tag id; drain pending blocks if any new ids appeared."""
        new_ids = False
        for tag_id, polygon in zip(msg.ids, msg.polygons, strict=True):
            tid = int(tag_id)
            if tid not in self.stockpiles:
                new_ids = True
            self.stockpiles[tid] = PolygonStamped(header=msg.header, polygon=polygon)
        if new_ids and self.pending_unbound:
            self._drain_pending_unbound()

    def _drain_pending_unbound(self) -> None:
        """
        Retry binding for blocks queued earlier when no stockpile was free.

        Preserves arrival order: the first queued block of a given type drives that
        type's binding. Blocks that still cannot bind remain queued.
        """
        remaining: list[Block] = []
        for block in self.pending_unbound:
            if block.type in self.type_to_stockpile:
                self.reported_blocks.append(block)
                continue
            tag_id = self._choose_closest_unassigned_stockpile(block.pose.pose.position.x, block.pose.pose.position.y)
            if tag_id is None:
                remaining.append(block)
                continue
            self.type_to_stockpile[block.type] = tag_id
            self.get_logger().info(f"(drain) Bound block type {block.type} -> stockpile tag {tag_id}")
            self.reported_blocks.append(block)
        self.pending_unbound = remaining

    def _choose_closest_unassigned_stockpile(self, x: float, y: float) -> int | None:
        """
        Return the tag id of the unassigned stockpile whose centroid is closest to (x, y).

        Returns None when every known stockpile is already bound to a block type.
        Distance is squared Euclidean in xy.
        """
        candidates = set(self.stockpiles) - set(self.type_to_stockpile.values())
        if not candidates:
            return None

        def dist2(tag_id: int) -> float:
            cx, cy = _polygon_centroid_xy(self.stockpiles[tag_id].polygon)
            dx, dy = cx - x, cy - y
            return dx * dx + dy * dy

        return min(candidates, key=dist2)

    def _on_scout_report(self, msg: Block) -> None:
        """Record a scout-reported block; bind its type to a stockpile if not yet bound."""
        block = msg
        if block.type not in self.type_to_stockpile:
            tag_id = self._choose_closest_unassigned_stockpile(
                block.pose.pose.position.x,
                block.pose.pose.position.y,
            )
            if tag_id is None:
                self.get_logger().error(
                    f"No unassigned stockpile available for block type {block.type} "
                    f"(known stockpiles: {len(self.stockpiles)}, "
                    f"bindings: {self.type_to_stockpile}); queueing block."
                )
                self.pending_unbound.append(block)
                return
            self.type_to_stockpile[block.type] = tag_id
            self.get_logger().info(f"Bound block type {block.type} -> stockpile tag {tag_id}")
        self.reported_blocks.append(block)

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
