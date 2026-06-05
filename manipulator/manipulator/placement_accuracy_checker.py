#!/usr/bin/env python3

import math
from typing import Dict, Optional, Tuple

import cv2
import numpy as np

import rclpy
import rclpy.duration
import rclpy.time

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray

from cc_interfaces.srv import CheckPlacement
from cc_interfaces.action import CorrectionTask

import tf2_ros
from tf2_ros import TransformException, TransformBroadcaster
from tf2_geometry_msgs import do_transform_pose_stamped


class PlacementAccuracyChecker(Node):
    """
    Placement checker with built-in AprilTag perception.

    Pipeline:
        RGB-D camera
            -> detect AprilTag 16h5
            -> estimate tag pose in camera frame
            -> transform pose to target/world frame
            -> compare with planner desired pose
            -> return good / misplaced / unseen
    """

    def __init__(self):
        super().__init__('placement_accuracy_checker')

        # ------------------------------------------------------------
        # Perception parameters
        # ------------------------------------------------------------
        self.declare_parameter(
            'color_topic',
            '/camera/camera/color/image_raw'
        )
        self.declare_parameter(
            'depth_topic',
            '/camera/camera/aligned_depth_to_color/image_raw'
        )
        self.declare_parameter(
            'camera_info_topic',
            '/camera/camera/color/camera_info'
        )

        self.declare_parameter('tag_family', '16h5')
        self.declare_parameter('target_id', -1)
        self.declare_parameter('marker_size', 0.055)

        self.declare_parameter('depth_window_size', 5)

        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_image_topic', '/placement_checker/debug_image')
        self.declare_parameter('show_window', False)

        # ------------------------------------------------------------
        # Frame / TF parameters
        # ------------------------------------------------------------
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('fallback_source_frame', 'camera_color_optical_frame')
        self.declare_parameter('tf_timeout_sec', 0.2)
        self.declare_parameter('publish_tag_tf', True)

        # ------------------------------------------------------------
        # Placement tolerance parameters
        # ------------------------------------------------------------
        self.declare_parameter('x_tolerance', 0.10)
        self.declare_parameter('y_tolerance', 0.10)
        self.declare_parameter('theta_tolerance_deg', 10.0)

        # ------------------------------------------------------------
        # Correction parameters
        # ------------------------------------------------------------
        self.declare_parameter('enable_correction', True)
        self.declare_parameter(
            'correction_action_name',
            '/manipulator/correction_task'
        )

        # ------------------------------------------------------------
        # Read parameters
        # ------------------------------------------------------------
        self.color_topic = self.get_parameter('color_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value

        self.tag_family = self.get_parameter('tag_family').value
        self.target_id = int(self.get_parameter('target_id').value)
        self.marker_size = float(self.get_parameter('marker_size').value)

        self.depth_window_size = int(self.get_parameter('depth_window_size').value)

        self.publish_debug_image = bool(
            self.get_parameter('publish_debug_image').value
        )
        self.debug_image_topic = self.get_parameter('debug_image_topic').value
        self.show_window = bool(self.get_parameter('show_window').value)

        self.target_frame = self.get_parameter('target_frame').value
        self.fallback_source_frame = self.get_parameter(
            'fallback_source_frame'
        ).value
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)
        self.publish_tag_tf = bool(self.get_parameter('publish_tag_tf').value)

        self.x_tolerance = float(self.get_parameter('x_tolerance').value)
        self.y_tolerance = float(self.get_parameter('y_tolerance').value)
        self.theta_tolerance = math.radians(
            float(self.get_parameter('theta_tolerance_deg').value)
        )

        self.enable_correction = bool(self.get_parameter('enable_correction').value)
        self.correction_action_name = self.get_parameter(
            'correction_action_name'
        ).value

        # ------------------------------------------------------------
        # Internal perception state
        # ------------------------------------------------------------
        self.bridge = CvBridge()

        self.camera_matrix = None
        self.dist_coeffs = None

        self.latest_depth_image = None
        self.latest_depth_encoding = None

        # tag_id -> PoseStamped in target_frame/world
        self.perceived_blocks: Dict[int, PoseStamped] = {}

        # ------------------------------------------------------------
        # TF
        # ------------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ------------------------------------------------------------
        # OpenCV AprilTag detector
        # ------------------------------------------------------------
        self.aruco_dict = self.create_dictionary(self.tag_family)
        self.aruco_params = self.create_detector_parameters()

        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(
                self.aruco_dict,
                self.aruco_params
            )
            self.detector_mode = 'ArucoDetector'
        else:
            self.detector = None
            self.detector_mode = 'legacy detectMarkers'

        # ------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            qos_profile_sensor_data
        )

        self.color_sub = self.create_subscription(
            Image,
            self.color_topic,
            self.color_callback,
            qos_profile_sensor_data
        )

        # ------------------------------------------------------------
        # Service server
        # ------------------------------------------------------------
        self.check_service = self.create_service(
            CheckPlacement,
            '/placement_checker/check_placement',
            self.check_placement_callback
        )

        # ------------------------------------------------------------
        # Correction action client
        # ------------------------------------------------------------
        self.correction_client = ActionClient(
            self,
            CorrectionTask,
            self.correction_action_name
        )

        # ------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------
        self.status_marker_pub = self.create_publisher(
            MarkerArray,
            '/placement_checker/marker_array',
            10
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            self.debug_image_topic,
            10
        )

        self.get_logger().info('Placement accuracy checker with perception started.')
        self.get_logger().info(f'Color topic: {self.color_topic}')
        self.get_logger().info(f'Depth topic: {self.depth_topic}')
        self.get_logger().info(f'Camera info topic: {self.camera_info_topic}')
        self.get_logger().info(f'Tag family: {self.tag_family}')
        self.get_logger().info(f'Target ID: {self.target_id} (-1 means all tags)')
        self.get_logger().info(f'Marker size: {self.marker_size:.3f} m')
        self.get_logger().info(f'OpenCV detector mode: {self.detector_mode}')
        self.get_logger().info(f'Target/world frame: {self.target_frame}')
        self.get_logger().info(f'Fallback source frame: {self.fallback_source_frame}')
        self.get_logger().info(f'X tolerance: {self.x_tolerance:.3f} m')
        self.get_logger().info(f'Y tolerance: {self.y_tolerance:.3f} m')
        self.get_logger().info(
            f'Theta tolerance: {math.degrees(self.theta_tolerance):.2f} deg'
        )
        self.get_logger().info(f'Correction enabled: {self.enable_correction}')

    # ------------------------------------------------------------
    # OpenCV detector helpers
    # ------------------------------------------------------------
    def create_dictionary(self, tag_family: str):
        if not hasattr(cv2, 'aruco'):
            raise RuntimeError(
                'OpenCV does not include cv2.aruco. Install opencv-contrib.'
            )

        dictionary_map = {
            '16h5': cv2.aruco.DICT_APRILTAG_16H5,
            '25h9': cv2.aruco.DICT_APRILTAG_25H9,
            '36h10': cv2.aruco.DICT_APRILTAG_36H10,
            '36h11': cv2.aruco.DICT_APRILTAG_36H11,
            'original': cv2.aruco.DICT_ARUCO_ORIGINAL,
            '4x4_50': cv2.aruco.DICT_4X4_50,
            '5x5_50': cv2.aruco.DICT_5X5_50,
            '6x6_50': cv2.aruco.DICT_6X6_50,
        }

        if tag_family not in dictionary_map:
            raise RuntimeError(
                f'Unsupported tag family: {tag_family}. '
                f'Valid options: {list(dictionary_map.keys())}'
            )

        return cv2.aruco.getPredefinedDictionary(dictionary_map[tag_family])

    def create_detector_parameters(self):
        if hasattr(cv2.aruco, 'DetectorParameters'):
            params = cv2.aruco.DetectorParameters()
        else:
            params = cv2.aruco.DetectorParameters_create()

        if hasattr(params, 'cornerRefinementMethod'):
            if hasattr(cv2.aruco, 'CORNER_REFINE_SUBPIX'):
                params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        return params

    def detect_tags(self, gray_image):
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray_image)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray_image,
                self.aruco_dict,
                parameters=self.aruco_params
            )

        return corners, ids

    # ------------------------------------------------------------
    # Camera callbacks
    # ------------------------------------------------------------
    def camera_info_callback(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)

        if len(msg.d) > 0:
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        else:
            self.dist_coeffs = np.zeros((5,), dtype=np.float64)

    def depth_callback(self, msg: Image):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='passthrough'
            )
            self.latest_depth_encoding = msg.encoding

        except Exception as exc:
            self.get_logger().error(f'Depth image conversion failed: {exc}')

    def color_callback(self, msg: Image):
        if self.camera_matrix is None:
            self.get_logger().warn('Waiting for camera_info...')
            return

        if self.latest_depth_image is None:
            self.get_logger().warn('Waiting for depth image...')
            return

        try:
            color_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as exc:
            self.get_logger().error(f'Color image conversion failed: {exc}')
            return

        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        corners, ids = self.detect_tags(gray_image)

        # Clear perception each frame. If the tag disappears, service returns unseen.
        self.perceived_blocks.clear()

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(color_image, corners, ids)

            for i, marker_id_np in enumerate(ids.flatten()):
                marker_id = int(marker_id_np)

                if self.target_id != -1 and marker_id != self.target_id:
                    continue

                marker_corners = corners[i]

                pose_camera = self.estimate_tag_pose(marker_corners, msg.header)

                if pose_camera is None:
                    continue

                pose_world = self.transform_pose_to_target_frame(pose_camera)

                if pose_world is None:
                    continue

                self.perceived_blocks[marker_id] = pose_world

                if self.publish_tag_tf:
                    self.publish_tag_tf_from_pose(marker_id, pose_camera)

                center_x, center_y = self.get_marker_center(marker_corners)
                depth_m = pose_camera.pose.position.z

                self.draw_debug_text(
                    color_image,
                    marker_id,
                    center_x,
                    center_y,
                    depth_m,
                    pose_world
                )

                self.get_logger().info(
                    f'Tag ID {marker_id} perceived. '
                    f'Camera pose: '
                    f'x={pose_camera.pose.position.x:.3f}, '
                    f'y={pose_camera.pose.position.y:.3f}, '
                    f'z={pose_camera.pose.position.z:.3f}. '
                    f'World pose: '
                    f'x={pose_world.pose.position.x:.3f}, '
                    f'y={pose_world.pose.position.y:.3f}, '
                    f'z={pose_world.pose.position.z:.3f}.'
                )

        self.publish_debug(color_image, msg.header)

        if self.show_window:
            cv2.imshow('placement_checker_perception', color_image)
            cv2.waitKey(1)

    # ------------------------------------------------------------
    # Pose estimation
    # ------------------------------------------------------------
    def estimate_tag_pose(self, marker_corners, image_header) -> Optional[PoseStamped]:
        success, rvec, tvec = self.solve_pnp(marker_corners)

        if not success:
            self.get_logger().warn('solvePnP failed for detected tag.')
            return None

        center_x, center_y = self.get_marker_center(marker_corners)

        depth_m = self.get_depth_at_pixel(
            self.latest_depth_image,
            self.latest_depth_encoding,
            center_x,
            center_y,
            window_size=self.depth_window_size
        )

        if depth_m > 0.0:
            x, y, z = self.backproject(center_x, center_y, depth_m)
        else:
            x, y, z = tvec.flatten()

        quat = self.rvec_to_quaternion(rvec)

        source_frame = image_header.frame_id
        if source_frame == '':
            source_frame = self.fallback_source_frame

        pose_msg = PoseStamped()
        pose_msg.header.stamp = image_header.stamp
        pose_msg.header.frame_id = source_frame

        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        pose_msg.pose.position.z = float(z)

        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])

        return pose_msg

    def solve_pnp(self, marker_corners):
        half_size = self.marker_size / 2.0

        object_points = np.array([
            [-half_size,  half_size, 0.0],
            [ half_size,  half_size, 0.0],
            [ half_size, -half_size, 0.0],
            [-half_size, -half_size, 0.0],
        ], dtype=np.float64)

        image_points = marker_corners.reshape((4, 2)).astype(np.float64)

        solvepnp_flag = (
            cv2.SOLVEPNP_IPPE_SQUARE
            if hasattr(cv2, 'SOLVEPNP_IPPE_SQUARE')
            else cv2.SOLVEPNP_ITERATIVE
        )

        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=solvepnp_flag
        )

        return success, rvec, tvec

    def get_marker_center(self, marker_corners) -> Tuple[int, int]:
        c = marker_corners.reshape((4, 2))

        center_x = int(np.mean(c[:, 0]))
        center_y = int(np.mean(c[:, 1]))

        return center_x, center_y

    def get_depth_at_pixel(
        self,
        depth_image,
        encoding,
        center_x,
        center_y,
        window_size=5
    ) -> float:
        if depth_image is None:
            return 0.0

        h, w = depth_image.shape[:2]

        half_window = window_size // 2
        depth_values = []

        for y in range(center_y - half_window, center_y + half_window + 1):
            for x in range(center_x - half_window, center_x + half_window + 1):
                if x < 0 or y < 0 or x >= w or y >= h:
                    continue

                d = depth_image[y, x]

                if not np.isfinite(float(d)) or float(d) <= 0.0:
                    continue

                depth_values.append(float(d))

        if len(depth_values) == 0:
            return 0.0

        median_depth = float(np.median(depth_values))

        if encoding == '16UC1' or depth_image.dtype == np.uint16:
            return median_depth * 0.001

        return median_depth

    def backproject(self, u, v, depth_m):
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        x = (u - cx) * depth_m / fx
        y = (v - cy) * depth_m / fy
        z = depth_m

        return x, y, z

    def rvec_to_quaternion(self, rvec):
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        r = rotation_matrix
        trace = np.trace(r)

        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (r[2, 1] - r[1, 2]) / s
            qy = (r[0, 2] - r[2, 0]) / s
            qz = (r[1, 0] - r[0, 1]) / s
        elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
            s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
            qw = (r[2, 1] - r[1, 2]) / s
            qx = 0.25 * s
            qy = (r[0, 1] + r[1, 0]) / s
            qz = (r[0, 2] + r[2, 0]) / s
        elif r[1, 1] > r[2, 2]:
            s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
            qw = (r[0, 2] - r[2, 0]) / s
            qx = (r[0, 1] + r[1, 0]) / s
            qy = 0.25 * s
            qz = (r[1, 2] + r[2, 1]) / s
        else:
            s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
            qw = (r[1, 0] - r[0, 1]) / s
            qx = (r[0, 2] + r[2, 0]) / s
            qy = (r[1, 2] + r[2, 1]) / s
            qz = 0.25 * s

        quat = np.array([qx, qy, qz, qw], dtype=np.float64)
        norm = np.linalg.norm(quat)

        if norm > 0.0:
            quat = quat / norm

        return quat

    # ------------------------------------------------------------
    # TF helpers
    # ------------------------------------------------------------
    def transform_pose_to_target_frame(
        self,
        pose_msg: PoseStamped
    ) -> Optional[PoseStamped]:
        source_frame = pose_msg.header.frame_id

        if source_frame == '':
            source_frame = self.fallback_source_frame
            pose_msg.header.frame_id = source_frame

        if source_frame == self.target_frame:
            return pose_msg

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout_sec)
            )

            transformed = do_transform_pose_stamped(
                pose_msg,
                transform
            )

            transformed.header.stamp = pose_msg.header.stamp
            transformed.header.frame_id = self.target_frame

            return transformed

        except TransformException as exc:
            self.get_logger().warn(
                f'Could not transform from {source_frame} '
                f'to {self.target_frame}: {exc}'
            )
            return None

    def transform_desired_pose_if_needed(
        self,
        desired_pose_stamped: PoseStamped
    ) -> Optional[PoseStamped]:
        desired_frame = desired_pose_stamped.header.frame_id

        if desired_frame == '':
            desired_pose_stamped.header.frame_id = self.target_frame
            return desired_pose_stamped

        if desired_frame == self.target_frame:
            return desired_pose_stamped

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                desired_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout_sec)
            )

            transformed = do_transform_pose_stamped(
                desired_pose_stamped,
                transform
            )

            transformed.header.frame_id = self.target_frame
            return transformed

        except TransformException as exc:
            self.get_logger().warn(
                f'Could not transform desired pose from {desired_frame} '
                f'to {self.target_frame}: {exc}'
            )
            return None

    def publish_tag_tf_from_pose(self, marker_id: int, pose_camera: PoseStamped):
        transform = TransformStamped()

        transform.header.stamp = pose_camera.header.stamp
        transform.header.frame_id = pose_camera.header.frame_id
        transform.child_frame_id = f'tag16h5_{marker_id}'

        transform.transform.translation.x = pose_camera.pose.position.x
        transform.transform.translation.y = pose_camera.pose.position.y
        transform.transform.translation.z = pose_camera.pose.position.z

        transform.transform.rotation = pose_camera.pose.orientation

        self.tf_broadcaster.sendTransform(transform)

    # ------------------------------------------------------------
    # Service callback
    # ------------------------------------------------------------
    def check_placement_callback(self, request, response):
        response.success = True
        response.message = ''

        response.properly_placed_ids = []
        response.misplaced_ids = []
        response.unseen_ids = []

        misplaced_block_ids = []
        misplaced_aruco_ids = []
        misplaced_current_poses = []
        misplaced_desired_poses = []

        if not (
            len(request.block_ids)
            == len(request.aruco_ids)
            == len(request.desired_poses)
        ):
            response.success = False
            response.message = (
                'Invalid request: block_ids, aruco_ids, and desired_poses '
                'must have the same length.'
            )
            return response

        marker_array = MarkerArray()

        for i in range(len(request.block_ids)):
            block_id = request.block_ids[i]
            tag_id = int(request.aruco_ids[i])

            desired_pose_stamped = self.transform_desired_pose_if_needed(
                request.desired_poses[i]
            )

            if desired_pose_stamped is None:
                response.success = False
                response.message = (
                    f'Could not transform desired pose for {block_id} '
                    f'to {self.target_frame}.'
                )
                return response

            if tag_id not in self.perceived_blocks:
                response.unseen_ids.append(block_id)
                self.get_logger().warn(
                    f'{block_id} with AprilTag ID {tag_id} is unseen.'
                )
                continue

            current_pose_stamped = self.perceived_blocks[tag_id]

            desired_pose = desired_pose_stamped.pose
            current_pose = current_pose_stamped.pose

            x_error = abs(desired_pose.position.x - current_pose.position.x)
            y_error = abs(desired_pose.position.y - current_pose.position.y)

            desired_theta = self.yaw_from_quaternion(desired_pose.orientation)
            current_theta = self.yaw_from_quaternion(current_pose.orientation)
            theta_error = abs(self.wrap_angle(desired_theta - current_theta))

            is_good = (
                x_error <= self.x_tolerance
                and y_error <= self.y_tolerance
                and theta_error <= self.theta_tolerance
            )

            self.get_logger().info(
                f'Checking {block_id} / AprilTag {tag_id}: '
                f'x_err={x_error:.3f} m, '
                f'y_err={y_error:.3f} m, '
                f'theta_err={math.degrees(theta_error):.2f} deg'
            )

            if is_good:
                response.properly_placed_ids.append(block_id)

                marker_array.markers.append(
                    self.create_status_marker(
                        marker_id=i,
                        pose=current_pose,
                        good=True
                    )
                )
            else:
                response.misplaced_ids.append(block_id)

                misplaced_block_ids.append(block_id)
                misplaced_aruco_ids.append(tag_id)
                misplaced_current_poses.append(current_pose_stamped)
                misplaced_desired_poses.append(desired_pose_stamped)

                marker_array.markers.append(
                    self.create_status_marker(
                        marker_id=i,
                        pose=current_pose,
                        good=False
                    )
                )

        self.status_marker_pub.publish(marker_array)

        if misplaced_block_ids and self.enable_correction:
            self.send_correction_task(
                misplaced_block_ids,
                misplaced_aruco_ids,
                misplaced_current_poses,
                misplaced_desired_poses
            )

        response.message = (
            f'Placement check complete. '
            f'good={list(response.properly_placed_ids)}, '
            f'misplaced={list(response.misplaced_ids)}, '
            f'unseen={list(response.unseen_ids)}'
        )

        self.get_logger().info(response.message)

        return response

    # ------------------------------------------------------------
    # Correction action
    # ------------------------------------------------------------
    def send_correction_task(
        self,
        block_ids,
        aruco_ids,
        current_poses,
        desired_poses
    ):
        if not self.correction_client.server_is_ready():
            self.get_logger().warn(
                f'Correction action server {self.correction_action_name} '
                f'is not ready.'
            )
            return

        goal_msg = CorrectionTask.Goal()
        goal_msg.block_ids = block_ids
        goal_msg.aruco_ids = aruco_ids
        goal_msg.current_poses = current_poses
        goal_msg.desired_poses = desired_poses

        self.get_logger().info(
            f'Sending correction task for misplaced blocks: {block_ids}'
        )

        send_future = self.correction_client.send_goal_async(
            goal_msg,
            feedback_callback=self.correction_feedback_callback
        )

        send_future.add_done_callback(self.correction_goal_response_callback)

    def correction_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Correction task was rejected.')
            return

        self.get_logger().info('Correction task accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.correction_result_callback)

    def correction_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Correction feedback: {feedback.status}')

    def correction_result_callback(self, future):
        result = future.result().result

        if result.success:
            self.get_logger().info(
                f'Correction succeeded: corrected={list(result.corrected_ids)}'
            )
        else:
            self.get_logger().warn(
                f'Correction failed: failed={list(result.failed_ids)}, '
                f'message={result.message}'
            )

    # ------------------------------------------------------------
    # Math helpers
    # ------------------------------------------------------------
    def yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        return math.atan2(siny_cosp, cosy_cosp)

    def wrap_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    # ------------------------------------------------------------
    # Visualization helpers
    # ------------------------------------------------------------
    def create_status_marker(self, marker_id: int, pose: Pose, good: bool):
        marker = Marker()

        marker.header.frame_id = self.target_frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'placement_checker'
        marker.id = int(marker_id)

        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose

        marker.scale.x = 0.12
        marker.scale.y = 0.12
        marker.scale.z = 0.03

        if good:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
        else:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8

        marker.lifetime.sec = 1
        marker.lifetime.nanosec = 0

        return marker

    def draw_debug_text(
        self,
        image,
        marker_id,
        center_x,
        center_y,
        depth_m,
        pose_world: PoseStamped
    ):
        cv2.circle(
            image,
            (center_x, center_y),
            5,
            (0, 255, 0),
            -1
        )

        cv2.putText(
            image,
            f'ID: {marker_id}',
            (center_x + 10, center_y - 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

        cv2.putText(
            image,
            f'Depth: {depth_m:.3f} m',
            (center_x + 10, center_y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

        cv2.putText(
            image,
            (
                f'World: x={pose_world.pose.position.x:.2f}, '
                f'y={pose_world.pose.position.y:.2f}, '
                f'z={pose_world.pose.position.z:.2f}'
            ),
            (center_x + 10, center_y + 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2
        )

    def publish_debug(self, image, header):
        if not self.publish_debug_image:
            return

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(
                image,
                encoding='bgr8'
            )
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)

        except Exception as exc:
            self.get_logger().error(f'Debug image publish failed: {exc}')


def main(args=None):
    rclpy.init(args=args)

    node = PlacementAccuracyChecker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        show_window = bool(getattr(node, 'show_window', False))

        node.destroy_node()

        if show_window:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()