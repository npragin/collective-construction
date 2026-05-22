#!/usr/bin/env python3

"""Publishes a static collision box to MoveIt's planning scene.

The box is anchored to the Jackal's base_link so it travels with the robot, and
acts as a workspace / support surface that MoveIt plans around. Box dimensions
and pose are configurable via ROS parameters.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


class AddCollisionBoxNode(Node):
    def __init__(self):
        super().__init__('add_collision_box')

        self.declare_parameter('namespace', 'j100_0897')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('object_id', 'planning_box')
        self.declare_parameter('size', [0.15, 0.40, 0.04])
        self.declare_parameter('position', [0.15, 0.0, 0.25])
        self.declare_parameter('orientation', [0.0, 0.0, 0.0, 1.0])

        self.namespace = self.get_parameter('namespace').value
        self.frame_id = self.get_parameter('frame_id').value
        self.object_id = self.get_parameter('object_id').value
        self.size = list(self.get_parameter('size').value)
        self.position = list(self.get_parameter('position').value)
        self.orientation = list(self.get_parameter('orientation').value)

        if len(self.size) != 3:
            raise ValueError(f"'size' must have 3 elements, got {self.size}")
        if len(self.position) != 3:
            raise ValueError(f"'position' must have 3 elements, got {self.position}")
        if len(self.orientation) != 4:
            raise ValueError(f"'orientation' must be a quaternion [x,y,z,w], got {self.orientation}")

        srv_name = f'/{self.namespace}/apply_planning_scene'
        self.apply_client = self.create_client(ApplyPlanningScene, srv_name)
        self.get_logger().info(f"Waiting for '{srv_name}'...")
        while not self.apply_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Still waiting for '{srv_name}'...")

        self.publish_box()

    def build_collision_object(self) -> CollisionObject:
        obj = CollisionObject()
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.header.frame_id = self.frame_id
        obj.id = self.object_id

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [float(d) for d in self.size]

        pose = Pose()
        pose.position.x = float(self.position[0])
        pose.position.y = float(self.position[1])
        pose.position.z = float(self.position[2])
        pose.orientation.x = float(self.orientation[0])
        pose.orientation.y = float(self.orientation[1])
        pose.orientation.z = float(self.orientation[2])
        pose.orientation.w = float(self.orientation[3])

        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD
        return obj

    def publish_box(self):
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(self.build_collision_object())

        req = ApplyPlanningScene.Request()
        req.scene = scene

        future = self.apply_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result is not None and result.success:
            self.get_logger().info(
                f"Added collision box '{self.object_id}' in '{self.frame_id}' "
                f"at pos={self.position}, size={self.size}"
            )
        else:
            self.get_logger().error(
                f"Failed to apply planning scene (result={result})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = AddCollisionBoxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
