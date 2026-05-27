"""
Scripted integration test harness for the cc_planner planner node.

Stands up fake scouts, retrievers, and manipulators, then drives them through an
ordered list of events read from a scenario YAML file, asserting that the
planner allocates the correct tasks to the correct robots. The planner runs
unmodified and reacts autonomously; this harness only feeds inputs and answers
the planner's action goals.
"""

import contextlib
import sys
import threading
import time
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from cc_interfaces.action import ManipulationTask, RetrievalTask
from cc_interfaces.msg import Block, Stockpiles, StructurePlan
from geometry_msgs.msg import Point32, Polygon, PoseStamped, TransformStamped
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

_BLOCK_TYPES = {
    "A": Block.TYPE_A,
    "B": Block.TYPE_B,
    "C": Block.TYPE_C,
}


class HarnessError(Exception):
    """Raised when a scenario is mis-constructed or an assertion fails."""


def _block_type(name: object) -> int:
    """Map a scenario block-type name ('A'/'B'/'C') to a Block.TYPE_* constant."""
    try:
        return _BLOCK_TYPES[str(name).upper()]
    except KeyError as exc:
        raise HarnessError(f"Unknown block type {name!r}; expected A, B, or C") from exc


def _square_polygon(x: float, y: float, half: float = 0.1) -> Polygon:
    """Build a small axis-aligned square polygon centered on (x, y)."""
    poly = Polygon()
    for dx, dy in ((-half, -half), (half, -half), (half, half), (-half, half)):
        poly.points.append(Point32(x=float(x + dx), y=float(y + dy), z=0.0))
    return poly


class ControllableActionServer:
    """
    A fake action server that accepts goals at the ROS level but defers execution.

    An incoming goal handle is stashed in `_pending`. The harness script advances
    it explicitly: `accept()` moves it to executing, `complete()` finishes it with
    success. This makes 'accept' and 'complete' independently scriptable steps.
    """

    def __init__(self, node: Node, action_type, action_name: str, robot_id: str) -> None:
        """Initialize the fake action server and register it with the given ROS node."""
        self._node = node
        self._robot_id = robot_id
        self._result_cls = action_type.Result
        self._pending = None  # goal handle awaiting an accept event
        self._executing = None  # goal handle currently executing
        self._complete_event = threading.Event()
        self._server = ActionServer(
            node,
            action_type,
            action_name,
            execute_callback=self._execute,
            goal_callback=lambda goal_request: GoalResponse.ACCEPT,
            handle_accepted_callback=self._on_accepted,
        )
        node.get_logger().info(f"fake action server ready: {action_name}")

    def _on_accepted(self, goal_handle) -> None:
        # Override the default (which would execute immediately): stash instead.
        if self._pending is not None or self._executing is not None:
            self._node.async_fault = (
                f"{self._robot_id} received a second goal while still busy; the planner double-tasked a robot"
            )
            return
        self._pending = goal_handle

    def _execute(self, goal_handle):
        # Runs in an executor thread once accept() calls goal_handle.execute().
        self._complete_event.wait()
        self._complete_event.clear()
        goal_handle.succeed()
        result = self._result_cls()
        result.success = True
        return result

    def pending_block(self):
        """Return the Block of the pending goal, or None if nothing is pending."""
        if self._pending is None:
            return None
        return self._pending.request.block

    def accept(self) -> None:
        """Move the pending goal to executing state and start the execute callback."""
        if self._pending is None:
            raise HarnessError(f"{self._robot_id}: accept() called with no pending goal")
        self._executing = self._pending
        self._pending = None
        self._executing.execute()

    def complete(self) -> None:
        """Signal the executing goal to finish with success."""
        if self._executing is None:
            raise HarnessError(f"{self._robot_id}: complete event with no task being executed")
        self._complete_event.set()
        self._executing = None


class TestHarnessNode(Node):
    """ROS node that owns all fake robots and runs a scenario script."""

    def __init__(self) -> None:
        """Initialize publishers, fake action servers, and load the scenario file."""
        super().__init__("test_harness")

        self.declare_parameter("scenario_file", "")
        self.declare_parameter("goal_wait_timeout", 5.0)
        self.declare_parameter("planner_wait_timeout", 10.0)
        self.declare_parameter("retrieval_action_name", "retrieval_task")
        self.declare_parameter("manipulation_action_name", "manipulation_task")
        self.declare_parameter("scout_report_topic", "scout_report")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("robot_frame_prefix", "aruco_")

        scenario_file = self.get_parameter("scenario_file").get_parameter_value().string_value
        self._goal_wait_timeout = self.get_parameter("goal_wait_timeout").get_parameter_value().double_value
        self._planner_wait_timeout = self.get_parameter("planner_wait_timeout").get_parameter_value().double_value
        retrieval_action = self.get_parameter("retrieval_action_name").get_parameter_value().string_value
        manipulation_action = self.get_parameter("manipulation_action_name").get_parameter_value().string_value
        scout_report_topic = self.get_parameter("scout_report_topic").get_parameter_value().string_value
        self._world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        self._frame_prefix = self.get_parameter("robot_frame_prefix").get_parameter_value().string_value

        if not scenario_file:
            raise HarnessError("scenario_file parameter is required")

        config_path = Path(get_package_share_directory("cc_planner")) / "config" / "robots.yaml"
        with config_path.open() as f:
            robots = yaml.safe_load(f)["robots"]
        self._aruco_ids = {r["identifier"]: int(r["aruco_tag_id"]) for r in robots}

        # Set by ControllableActionServer callbacks running on executor threads;
        # checked by the scenario worker thread after every event.
        self.async_fault: str | None = None

        self._tf_broadcaster = StaticTransformBroadcaster(self)

        self._stockpiles_pub = self.create_publisher(Stockpiles, "stockpile_polygons", 10)
        self._structure_pub = self.create_publisher(StructurePlan, "structure_plan", 10)

        self._scout_pubs: dict = {}
        self._servers: dict = {}
        for robot in robots:
            rid = robot["identifier"]
            capability = robot["capability"]
            if capability == "scout":
                topic = f"{rid}/{scout_report_topic}"
                self._scout_pubs[rid] = self.create_publisher(Block, topic, 10)
            elif capability == "retriever":
                self._servers[rid] = ControllableActionServer(self, RetrievalTask, f"{rid}/{retrieval_action}", rid)
            elif capability == "manipulator":
                self._servers[rid] = ControllableActionServer(
                    self, ManipulationTask, f"{rid}/{manipulation_action}", rid
                )

        with Path(scenario_file).open() as f:
            scenario = yaml.safe_load(f)
        if not isinstance(scenario, dict) or "events" not in scenario:
            raise HarnessError(f"scenario file {scenario_file} must have an 'events' list")
        self._events = scenario["events"]

        self._handlers = {
            "set_robot_pose": self._ev_set_robot_pose,
            "publish_stockpiles": self._ev_publish_stockpiles,
            "publish_structure_plan": self._ev_publish_structure_plan,
            "scout_report": self._ev_scout_report,
            "retriever_accept": self._ev_accept,
            "manipulator_accept": self._ev_accept,
            "retriever_complete": self._ev_complete,
            "manipulator_complete": self._ev_complete,
            "wait": self._ev_wait,
        }

    # --- helpers -----------------------------------------------------------

    def _check_async_fault(self) -> None:
        if self.async_fault is not None:
            raise HarnessError(self.async_fault)

    def _await_pending(self, robot: str) -> ControllableActionServer:
        """Poll until `robot`'s fake server has a pending goal, or abort on timeout."""
        server = self._servers.get(robot)
        if server is None:
            raise HarnessError(f"unknown robot {robot!r} (no fake action server)")
        deadline = time.monotonic() + self._goal_wait_timeout
        while server.pending_block() is None:
            self._check_async_fault()
            if time.monotonic() > deadline:
                raise HarnessError(
                    f"{robot}: no goal arrived within {self._goal_wait_timeout}s; "
                    f"the planner never dispatched a task to accept"
                )
            time.sleep(0.05)
        return server

    def _assert_block(self, robot: str, block, expect: dict) -> None:
        expected_type = _block_type(expect["type"])
        if block.type != expected_type:
            raise HarnessError(
                f"{robot}: expected block type {expect['type']} ({expected_type}) "
                f"but planner assigned type {block.type}"
            )
        ex, ey = float(expect["x"]), float(expect["y"])
        ax = block.pose.pose.position.x
        ay = block.pose.pose.position.y
        if ax != ex or ay != ey:
            raise HarnessError(f"{robot}: expected block at ({ex}, {ey}) but planner assigned block at ({ax}, {ay})")

    # --- event handlers ----------------------------------------------------

    def _ev_set_robot_pose(self, params: dict) -> None:
        robot = params["robot"]
        if robot not in self._aruco_ids:
            raise HarnessError(f"set_robot_pose: unknown robot {robot!r}")
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self._world_frame
        tf.child_frame_id = f"{self._frame_prefix}{self._aruco_ids[robot]}"
        tf.transform.translation.x = float(params["x"])
        tf.transform.translation.y = float(params["y"])
        tf.transform.translation.z = 0.0
        tf.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(tf)
        self.get_logger().info(f"set_robot_pose {robot} -> ({params['x']}, {params['y']})")

    def _ev_publish_stockpiles(self, params: dict) -> None:
        msg = Stockpiles()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._world_frame
        for stockpile in params["stockpiles"]:
            msg.ids.append(int(stockpile["id"]))
            msg.polygons.append(_square_polygon(stockpile["x"], stockpile["y"]))
        self._stockpiles_pub.publish(msg)
        self.get_logger().info(f"published {len(msg.ids)} stockpile(s)")

    def _ev_publish_structure_plan(self, params: dict) -> None:
        msg = StructurePlan()
        for entry in params["blocks"]:
            block = Block()
            block.type = _block_type(entry["type"])
            pose = PoseStamped()
            pose.header.frame_id = self._world_frame
            pose.pose.position.x = float(entry["x"])
            pose.pose.position.y = float(entry["y"])
            pose.pose.position.z = float(entry.get("z", 0.0))
            pose.pose.orientation.w = 1.0
            block.pose = pose
            msg.blocks.append(block)
        self._structure_pub.publish(msg)
        self.get_logger().info(f"published structure plan with {len(msg.blocks)} block(s)")

    def _ev_scout_report(self, params: dict) -> None:
        scout = params["scout"]
        if scout not in self._scout_pubs:
            raise HarnessError(f"scout_report: unknown scout {scout!r}")
        block = Block()
        block.type = _block_type(params["type"])
        block.pose.header.frame_id = self._world_frame
        block.pose.pose.position.x = float(params["x"])
        block.pose.pose.position.y = float(params["y"])
        block.pose.pose.orientation.w = 1.0
        self._scout_pubs[scout].publish(block)
        self.get_logger().info(f"scout_report {scout}: type={params['type']} at ({params['x']}, {params['y']})")

    def _ev_accept(self, params: dict) -> None:
        robot = params["robot"]
        expect = params.get("expect")
        if expect is None:
            raise HarnessError(f"{robot}: accept event requires an 'expect' block")
        server = self._await_pending(robot)
        self._assert_block(robot, server.pending_block(), expect)
        server.accept()
        self.get_logger().info(f"{robot}: accepted task (assertions passed)")

    def _ev_complete(self, params: dict) -> None:
        robot = params["robot"]
        server = self._servers.get(robot)
        if server is None:
            raise HarnessError(f"unknown robot {robot!r} (no fake action server)")
        server.complete()
        self.get_logger().info(f"{robot}: completed task")

    def _ev_wait(self, params: dict) -> None:
        seconds = float(params["seconds"])
        self.get_logger().info(f"wait {seconds}s")
        time.sleep(seconds)

    # --- scenario driver ---------------------------------------------------

    def wait_for_planner(self) -> None:
        """Block until the planner has subscribed to every harness publisher."""
        deadline = time.monotonic() + self._planner_wait_timeout
        pubs = [self._stockpiles_pub, self._structure_pub, *self._scout_pubs.values()]
        while True:
            if all(p.get_subscription_count() > 0 for p in pubs):
                self.get_logger().info("planner connected to all harness topics")
                return
            if time.monotonic() > deadline:
                raise HarnessError(f"planner did not subscribe within {self._planner_wait_timeout}s")
            time.sleep(0.1)

    def run_scenario(self) -> None:
        """Execute every event in the scenario in order, raising HarnessError on failure."""
        self.wait_for_planner()
        for index, event in enumerate(self._events):
            if not isinstance(event, dict) or len(event) != 1:
                raise HarnessError(f"event {index} must be a single-key mapping, got {event!r}")
            ((name, params),) = event.items()
            handler = self._handlers.get(name)
            if handler is None:
                raise HarnessError(f"event {index}: unknown event type {name!r}")
            self.get_logger().info(f"--- event {index}: {name} ---")
            handler(params or {})
            self._check_async_fault()
        self.get_logger().info("SCENARIO PASSED")


def main() -> None:
    """Entry point: spin the harness node and run the scenario, exiting with 0 or 1."""
    rclpy.init()
    node = TestHarnessNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    exit_code = [0]

    def worker() -> None:
        try:
            node.run_scenario()
        except HarnessError as exc:
            node.get_logger().error(f"SCENARIO FAILED: {exc}")
            exit_code[0] = 1
        except Exception as exc:
            node.get_logger().error(f"SCENARIO FAILED (unexpected): {exc}")
            exit_code[0] = 1
        finally:
            executor.shutdown()

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    with contextlib.suppress(KeyboardInterrupt):
        executor.spin()
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(exit_code[0])


if __name__ == "__main__":
    main()
