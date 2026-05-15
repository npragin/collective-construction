"""Central planner node for the collective-construction system."""

import copy
from collections import deque
from enum import Enum
from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
import rclpy.duration
import rclpy.time
import tf2_ros
from cc_interfaces.action import ManipulationTask, RetrievalTask
from cc_interfaces.msg import Block, Stockpiles, StructurePlan
from geometry_msgs.msg import Polygon, PolygonStamped, PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import TransformException


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

        scout_report_topic = self.get_parameter("scout_report_topic").get_parameter_value().string_value
        self._retrieval_action_name = self.get_parameter("retrieval_action_name").get_parameter_value().string_value
        self._manipulation_action_name = (
            self.get_parameter("manipulation_action_name").get_parameter_value().string_value
        )

        self.declare_parameter("world_frame", "world")
        self.declare_parameter("build_frame", "build")
        self.declare_parameter("tf_lookup_timeout", 0.1)
        self.declare_parameter("robot_frame_prefix", "aruco_")

        self._world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        self._build_frame = self.get_parameter("build_frame").get_parameter_value().string_value
        self._tf_lookup_timeout = self.get_parameter("tf_lookup_timeout").get_parameter_value().double_value
        self._robot_frame_prefix = self.get_parameter("robot_frame_prefix").get_parameter_value().string_value

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        config_path = Path(get_package_share_directory("cc_planner")) / "config" / "robots.yaml"
        with config_path.open() as f:
            robots = yaml.safe_load(f)["robots"]

        self.robot_status: dict[str, RobotStatus] = {robot["identifier"]: RobotStatus.IDLE for robot in robots}
        self.robot_capabilities: dict[str, str] = {robot["identifier"]: robot["capability"] for robot in robots}
        self.robot_aruco_ids: dict[str, int] = {robot["identifier"]: int(robot["aruco_tag_id"]) for robot in robots}

        # Aruco tag id -> polygon
        # Polygon updates in place when re-detected; entries are never removed.
        self.stockpiles: dict[int, PolygonStamped] = {}
        # Block type (Block.TYPE_*) -> tag id of the bound stockpile. Permanent once set.
        self.type_to_stockpile: dict[int, int] = {}
        # Reported blocks whose type is bound but which haven't been retrieved yet.
        self.reported_blocks: list[Block] = []
        # Reported blocks whose type couldn't be bound (no unassigned stockpile).
        self.pending_unbound: list[Block] = []
        # robot_id -> Block currently being retrieved
        self.retrieval_in_flight: dict[str, Block] = {}
        # stockpile tag id -> delivered block count; initialized in _on_stockpiles
        self.stockpile_counts: dict[int, int] = {}
        # RW: robot_id -> dependency-graph block id currently being placed
        self.manipulation_in_flight: dict[str, str] = {}

        # RW: Placement dependency graph for manipulator scheduling.
        self.dependency_graph: dict[str, dict[str, Any]] = {}
        # Block type -> queue of dependency-ready block ids. Entries are added
        # lazily as block types are discovered in the structure plan.
        self.ready_blocks_by_type: dict[int, deque[str]] = {}

        self._stockpiles_sub = self.create_subscription(Stockpiles, "stockpile_polygons", self._on_stockpiles, 10)
        self._structure_plan_sub = self.create_subscription(
            StructurePlan, "structure_plan", self._on_structure_plan, 10
        )
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

    # ----- Retrieval pipeline -----

    def _on_stockpiles(self, msg: Stockpiles) -> None:
        """Upsert detected stockpiles by tag id; drain pending blocks if any new ids appeared."""
        new_ids = False
        for tag_id, polygon in zip(msg.ids, msg.polygons, strict=True):
            tid = int(tag_id)
            if tid not in self.stockpiles:
                new_ids = True
                self.stockpile_counts[tid] = 0
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
        self._try_assign_retrievers()

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
        self._try_assign_retrievers()

    def _try_assign_retrievers(self) -> None:
        """
        Pair queued blocks with idle retrievers.

        Block order is FIFO over reported_blocks. For each block, the idle retriever
        with the smallest squared-Euclidean xy distance is chosen. Retrievers whose
        TF pose cannot be resolved this round are skipped and reconsidered on the
        next trigger.
        """
        idle = [
            r
            for r, s in self.robot_status.items()
            if s is RobotStatus.IDLE
            and self.robot_capabilities[r] == "retriever"
            and self._action_clients[r].server_is_ready()
        ]
        if not idle or not self.reported_blocks:
            return

        poses: dict[str, tuple[float, float]] = {}
        for robot_id in idle:
            xy = self._lookup_robot_xy(robot_id)
            if xy is not None:
                poses[robot_id] = xy

        remaining: list[Block] = []
        for block in self.reported_blocks:
            if not poses:
                remaining.append(block)
                continue
            bx = block.pose.pose.position.x
            by = block.pose.pose.position.y
            robot_id = min(
                poses,
                key=lambda r: (poses[r][0] - bx) ** 2 + (poses[r][1] - by) ** 2,
            )
            stockpile = self.stockpiles[self.type_to_stockpile[block.type]]
            if not self._send_retrieval_task(robot_id, block, stockpile):
                remaining.append(block)
            del poses[robot_id]
        self.reported_blocks = remaining

    def _lookup_robot_xy(self, robot_id: str) -> tuple[float, float] | None:
        """
        Resolve a robot's xy in the world frame via TF.

        Returns None when the lookup fails (frame missing, transform stale, etc.);
        callers treat that robot as unavailable for the current pass.
        """
        aruco_id = self.robot_aruco_ids[robot_id]
        source_frame = f"{self._robot_frame_prefix}{aruco_id}"
        try:
            tf = self._tf_buffer.lookup_transform(
                self._world_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self._tf_lookup_timeout),
            )
        except TransformException as exc:
            self.get_logger().debug(f"TF lookup {self._world_frame}<-{source_frame} failed: {exc}")
            return None
        t = tf.transform.translation
        return (t.x, t.y)

    def _send_retrieval_task(self, robot_id: str, block: Block, stockpile: PolygonStamped) -> bool:
        """Dispatch a retrieval goal. Returns True on send, False if the server is not ready."""
        client = self._action_clients[robot_id]
        if not client.server_is_ready():
            self.get_logger().error(f"Retrieval action server for {robot_id} unavailable")
            return False

        goal = RetrievalTask.Goal()
        goal.block = block
        goal.stockpile = stockpile

        send_future = client.send_goal_async(goal)
        send_future.add_done_callback(lambda f: self._on_retrieval_goal_response(f, robot_id))
        self.retrieval_in_flight[robot_id] = block
        self.robot_status[robot_id] = RobotStatus.TASKED
        return True

    def _on_retrieval_goal_response(self, future, robot_id: str) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"Retrieval goal rejected by {robot_id}; requeueing block")
            block = self.retrieval_in_flight.pop(robot_id)
            self.reported_blocks.append(block)
            self.robot_status[robot_id] = RobotStatus.IDLE
            self._try_assign_retrievers()
            return
        goal_handle.get_result_async().add_done_callback(lambda f: self._on_retrieval_task_result(f, robot_id))

    def _on_retrieval_task_result(self, future, robot_id: str) -> None:
        result = future.result().result
        block = self.retrieval_in_flight.pop(robot_id)
        if result.success:
            tid = self.type_to_stockpile[block.type]
            self.stockpile_counts[tid] += 1
            self.get_logger().info(
                f"{robot_id} delivered block type {block.type} -> stockpile {tid} (count={self.stockpile_counts[tid]})"
            )
        else:
            self.get_logger().warn(f"{robot_id} retrieval failed; requeueing block")
            self.reported_blocks.append(block)

        self.robot_status[robot_id] = RobotStatus.IDLE

        self.allocate_manipulator_tasks()
        self._try_assign_retrievers()

    # ----- Manipulation pipeline -----

    def _on_structure_plan(self, msg: StructurePlan) -> None:
        """
        Load the planned structure from rasterizer output.

        Assumptions:
        - The plan is its own frameless coordinate system whose origin
          coincides with the build-site origin, so block positions are kept
          unchanged and only relabeled into the build frame.
        """
        self.dependency_graph = {}
        self.ready_blocks_by_type = {}

        for index, block in enumerate(msg.blocks):
            block_id = f"block_{index}"

            pose = PoseStamped()
            pose.header.stamp = block.pose.header.stamp
            pose.header.frame_id = self._build_frame
            pose.pose = copy.deepcopy(block.pose.pose)

            self.dependency_graph[block_id] = {
                "block_type": int(block.type),
                "pose": pose,
                "parent_ids": [],
                "child_ids": [],
                "placed": False,
                "in_progress": False,
            }

        # TODO: Construct dependency graph here.

        for block_id, node in self.dependency_graph.items():
            if self._parents_placed(node):
                block_type = node["block_type"]
                self.ready_blocks_by_type.setdefault(block_type, deque[str]()).append(block_id)

        self.get_logger().info(
            f"Loaded structure plan with {len(self.dependency_graph)} blocks in frame "
            f"'{self._build_frame}' ({sum(len(q) for q in self.ready_blocks_by_type.values())} initially ready)"
        )
        self.allocate_manipulator_tasks()

    def _parents_placed(self, node: dict[str, Any]) -> bool:
        """Return True when every parent of the given graph node has been placed."""
        return all(self.dependency_graph[parent_id]["placed"] for parent_id in node["parent_ids"])

    def allocate_manipulator_tasks(self) -> None:
        """Assign one ready planned block to each idle manipulator if stock is available."""
        if not self.dependency_graph:
            return

        idle_manipulators = [
            robot_id
            for robot_id, status in self.robot_status.items()
            if status is RobotStatus.IDLE
            and self.robot_capabilities[robot_id] == "manipulator"
            and self._action_clients[robot_id].server_is_ready()
        ]
        if not idle_manipulators:
            return

        for robot_id in idle_manipulators:
            block_id = self._reserve_placeable_block()
            if block_id is None:
                self.get_logger().debug(f"No manipulator task available for {robot_id}")
                return

            block_node = self.dependency_graph[block_id]
            block_type = block_node["block_type"]
            stockpile = self.stockpiles[self.type_to_stockpile[block_type]]

            block = Block()
            block.type = block_type
            block.pose = block_node["pose"]

            self.manipulation_in_flight[robot_id] = block_id

            if not self.send_manipulation_task(robot_id, block, stockpile):
                self.manipulation_in_flight.pop(robot_id)
                self._release_placeable_block(block_id)
                continue

    def _reserve_placeable_block(self) -> str | None:
        """
        Reserve one dependency-ready block whose type is stocked.

        Pops the block from its ready queue, marks it in-progress, and consumes
        one unit of its stockpile's stock. Returns None when no block is both
        dependency-ready and stocked. Stale queue entries are skipped lazily.
        """
        for block_type in self.ready_blocks_by_type:
            stockpile_tag = self.type_to_stockpile.get(block_type)
            if stockpile_tag is None or self.stockpile_counts.get(stockpile_tag, 0) <= 0:
                continue

            queue = self.ready_blocks_by_type[block_type]
            while queue:
                block_id = queue.popleft()
                node = self.dependency_graph[block_id]
                if node["placed"] or node["in_progress"] or not self._parents_placed(node):
                    continue
                node["in_progress"] = True
                self.stockpile_counts[stockpile_tag] -= 1
                return block_id

        return None

    def _release_placeable_block(self, block_id: str) -> None:
        """
        Inverse of _reserve_placeable_block.

        Returns the block to the front of its ready queue, clears its
        in-progress flag, and restores the stock unit consumed at reservation.
        """
        node = self.dependency_graph[block_id]
        node["in_progress"] = False
        block_type = node["block_type"]
        self.ready_blocks_by_type[block_type].appendleft(block_id)
        stockpile_tag = self.type_to_stockpile.get(block_type)
        if stockpile_tag is not None:
            self.stockpile_counts[stockpile_tag] += 1

    def send_manipulation_task(
        self,
        robot_id: str,
        block: Block,
        stockpile: PolygonStamped,
    ) -> bool:
        """Dispatch a manipulation goal. Returns True on send, False if the server is not ready."""
        client = self._action_clients[robot_id]
        if not client.server_is_ready():
            self.get_logger().error(f"Manipulation action server for {robot_id} unavailable")
            return False

        goal = ManipulationTask.Goal()
        goal.block = block
        goal.stockpile = stockpile

        send_future = client.send_goal_async(goal)
        send_future.add_done_callback(lambda f: self._on_manipulation_goal_response(f, robot_id))
        self.robot_status[robot_id] = RobotStatus.TASKED
        return True

    def _on_manipulation_goal_response(self, future, robot_id: str) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"Manipulation goal rejected by {robot_id}")

            # RW: Return the block to its ready queue and restore stock since it wasn't picked
            block_id = self.manipulation_in_flight[robot_id]
            self._release_placeable_block(block_id)

            self.robot_status[robot_id] = RobotStatus.IDLE

            self.allocate_manipulator_tasks()

            return

        goal_handle.get_result_async().add_done_callback(lambda f: self._on_manipulation_task_result(f, robot_id))

    def _on_manipulation_task_result(self, future, robot_id: str) -> None:
        result = future.result().result

        # RW: Update dependency graph and stockpile on manipulation result; then trigger next manipulator assignment pass
        block_id = self.manipulation_in_flight[robot_id]
        block_node = self.dependency_graph[block_id]
        if result.success:
            block_node["placed"] = True
            block_node["in_progress"] = False

            for child_id in block_node["child_ids"]:
                child_node = self.dependency_graph[child_id]
                if not child_node["placed"] and self._parents_placed(child_node):
                    child_type = child_node["block_type"]
                    self.ready_blocks_by_type[child_type].append(child_id)
                    self.get_logger().info(f"Block {child_id} is now dependency-ready")

            self.get_logger().info(f"{robot_id} placed block {block_id}")
        else:
            self._release_placeable_block(block_id)
            self.get_logger().warn(f"{robot_id} manipulation failed for block {block_id}")
        self.robot_status[robot_id] = RobotStatus.IDLE
        self.allocate_manipulator_tasks()


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
