#!/usr/bin/env python3

"""
aruco_depth_node.py

Single-file ROS 2 ArUco + depth node with OpenCV ArUco compatibility.

Designed to work with:
- OpenCV 4.6.x legacy ArUco API:
    cv2.aruco.DetectorParameters_create()
    cv2.aruco.detectMarkers()

- Newer OpenCV 4.x API, including 4.11.x:
    cv2.aruco.DetectorParameters()
    cv2.aruco.ArucoDetector()

This keeps OpenCV-version-specific logic inside OpenCVArucoCompat.
"""

import sys
import faulthandler

faulthandler.enable(file=sys.stderr, all_threads=True)

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray, TransformStamped, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Int32MultiArray

from cv_bridge import CvBridge
from tf2_geometry_msgs import do_transform_pose_stamped, do_transform_point
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

class OpenCVArucoCompat:
    """
    Compatibility adapter for OpenCV ArUco, now supporting multiple
    dictionaries at once. Detection runs across every active dictionary
    and results are merged, tagged with their source dictionary.

    OpenCV 4.6 often uses:
        DetectorParameters_create()
        detectMarkers()

    Newer OpenCV versions may use:
        DetectorParameters()
        ArucoDetector().detectMarkers()
    """

    DICTIONARIES = {
        "original": "DICT_ARUCO_ORIGINAL",
        "4x4_50": "DICT_4X4_50",
        "4x4_100": "DICT_4X4_100",
        "4x4_250": "DICT_4X4_250",
        "4x4_1000": "DICT_4X4_1000",
        "5x5_50": "DICT_5X5_50",
        "5x5_100": "DICT_5X5_100",
        "5x5_250": "DICT_5X5_250",
        "5x5_1000": "DICT_5X5_1000",
        "6x6_50": "DICT_6X6_50",
        "6x6_100": "DICT_6X6_100",
        "6x6_250": "DICT_6X6_250",
        "6x6_1000": "DICT_6X6_1000",
        "7x7_50": "DICT_7X7_50",
        "7x7_100": "DICT_7X7_100",
        "7x7_250": "DICT_7X7_250",
        "7x7_1000": "DICT_7X7_1000",
        "25h9": "DICT_APRILTAG_25H9",
        "16h5": "DICT_APRILTAG_16H5",
    }

    # "all" expands to one superset dict per family: e.g. 4x4_1000 already
    # contains every 4x4_50/100/250 marker, so scanning the smaller ones too
    # would be wasted work.
    ALL_FAMILIES = [
        "4x4_1000",
        "5x5_1000",
        "6x6_1000",
        "7x7_1000",
        "original",
        "25h9",
        "16h5",
    ]

    def __init__(self, dictionary_name="4x4_50", logger=None):
        self.logger = logger

        if not hasattr(cv2, "aruco"):
            raise RuntimeError(
                "This OpenCV build does not include cv2.aruco. "
                "Install an OpenCV build with ArUco support, usually opencv-contrib."
            )

        self.aruco = cv2.aruco
        self.parameters = self._create_detector_parameters()

        # Sub-pixel corner refinement markedly improves pose accuracy.
        if hasattr(self.parameters, "cornerRefinementMethod"):
            if hasattr(self.aruco, "CORNER_REFINE_SUBPIX"):
                self.parameters.cornerRefinementMethod = self.aruco.CORNER_REFINE_SUBPIX

        names = self._resolve_names(dictionary_name)

        # name -> (detector_or_None, dictionary)
        self.detectors = {}
        for name in names:
            dictionary = self._create_dictionary(name)
            detector = self._create_new_detector_if_available(dictionary)
            self.detectors[name] = (detector, dictionary)

        if self.logger is not None:
            any_new = any(d is not None for d, _ in self.detectors.values())
            mode = "ArucoDetector API" if any_new else "legacy detectMarkers API"
            self.logger.info(f"OpenCV version: {cv2.__version__}")
            self.logger.info(f"OpenCV file: {cv2.__file__}")
            self.logger.info(f"Using ArUco mode: {mode}")
            self.logger.info(f"Active dictionaries: {list(self.detectors.keys())}")

    def _resolve_names(self, dictionary_name):
        if isinstance(dictionary_name, (list, tuple)):
            requested = list(dictionary_name)
        elif isinstance(dictionary_name, str) and \
                dictionary_name.strip().lower() in ("all", "all_families"):
            requested = list(self.ALL_FAMILIES)
        else:
            requested = [n.strip() for n in str(dictionary_name).split(",") if n.strip()]

        names = []
        for n in requested:
            if n in self.DICTIONARIES:
                names.append(n)
            elif self.logger is not None:
                self.logger.warn(f"Unknown ArUco dictionary '{n}', ignoring")

        if not names:
            if self.logger is not None:
                self.logger.warn("No valid dictionaries given, falling back to '4x4_50'")
            names = ["4x4_50"]

        seen = set()
        unique = []
        for n in names:
            if n not in seen:
                seen.add(n)
                unique.append(n)
        return unique

    def _create_dictionary(self, name):
        dict_attr_name = self.DICTIONARIES[name]

        if not hasattr(self.aruco, dict_attr_name):
            raise RuntimeError(
                f"cv2.aruco is missing dictionary constant '{dict_attr_name}'. "
                "Your OpenCV ArUco module may be incomplete."
            )

        dict_id = getattr(self.aruco, dict_attr_name)

        if hasattr(self.aruco, "getPredefinedDictionary"):
            return self.aruco.getPredefinedDictionary(dict_id)

        if hasattr(self.aruco, "Dictionary_get"):
            return self.aruco.Dictionary_get(dict_id)

        raise RuntimeError(
            "Could not create ArUco dictionary. "
            "Neither getPredefinedDictionary() nor Dictionary_get() exists."
        )

    def _create_detector_parameters(self):
        if not hasattr(self.aruco, "ArucoDetector") and hasattr(self.aruco, "DetectorParameters_create"):
            return self.aruco.DetectorParameters_create()
        if hasattr(self.aruco, "DetectorParameters"):
            return self.aruco.DetectorParameters()

        if hasattr(self.aruco, "DetectorParameters_create"):
            return self.aruco.DetectorParameters_create()

        raise RuntimeError(
            "Could not create ArUco detector parameters. "
            "Neither DetectorParameters() nor DetectorParameters_create() exists."
        )

    def _create_new_detector_if_available(self, dictionary):
        if not hasattr(self.aruco, "ArucoDetector"):
            return None

        try:
            return self.aruco.ArucoDetector(dictionary, self.parameters)
        except Exception as exc:
            if self.logger is not None:
                self.logger.warn(
                    f"ArucoDetector exists but failed to initialize. "
                    f"Falling back to legacy detectMarkers(). Error: {exc}"
                )
            return None

    def detect_markers(self, gray_image):
        """
        Returns:
            corners:    list of corner arrays (one per detected marker)
            ids:        np.ndarray (N, 1) or None if nothing found
            dict_names: list of length N, source dictionary per marker
        """
        all_corners = []
        all_ids = []
        all_dicts = []

        for name, (detector, dictionary) in self.detectors.items():
            if detector is not None:
                corners, ids, _ = detector.detectMarkers(gray_image)
            elif hasattr(self.aruco, "detectMarkers"):
                corners, ids, _ = self.aruco.detectMarkers(
                    gray_image, dictionary, parameters=self.parameters
                )
            else:
                raise RuntimeError(
                    "cv2.aruco.detectMarkers() is missing and ArucoDetector is unavailable."
                )

            if ids is None or len(ids) == 0:
                continue

            for c, mid in zip(corners, ids.flatten()):
                all_corners.append(c)
                all_ids.append(int(mid))
                all_dicts.append(name)

        if not all_ids:
            return [], None, []

        ids_array = np.array(all_ids, dtype=np.int32).reshape(-1, 1)
        return all_corners, ids_array, all_dicts

    def draw_detected_markers(self, image, corners, ids):
        if ids is None or corners is None or len(corners) == 0:
            return
        if hasattr(self.aruco, "drawDetectedMarkers"):
            self.aruco.drawDetectedMarkers(image, corners, ids)

    def draw_axes(self, image, camera_matrix, dist_coeffs, rvec, tvec, axis_length):
        if hasattr(cv2, "drawFrameAxes"):
            cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvec, tvec, axis_length)
            return
        if hasattr(self.aruco, "drawAxis"):
            self.aruco.drawAxis(image, camera_matrix, dist_coeffs, rvec, tvec, axis_length)

class ArucoDepthNode(Node):
    def __init__(self):
        super().__init__("aruco_depth_node")

        self.declare_parameter("color_topic", "/j100_0897/sensors/camera_0/color/image")
        self.declare_parameter("depth_topic", "/j100_0897/sensors/camera_0/depth/image")
        self.declare_parameter("camera_info_topic", "/j100_0897/sensors/camera_0/color/camera_info")
        # Depth is NOT aligned to color on this camera (no aligned_depth_to_color
        # stream). We register it ourselves, which needs the depth intrinsics.
        self.declare_parameter("depth_camera_info_topic", "/j100_0897/sensors/camera_0/depth/camera_info")

        self.declare_parameter("marker_size", 0.0495)
        self.declare_parameter("aruco_dictionary", "25h9")
        self.declare_parameter("target_id", -1)

        self.declare_parameter("show_window", False)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_rviz_markers", False)

        # Use the arm base as the default target frame.
        # Override at runtime if needed:
        #   -p arm_base_frame:=base_link
        self.declare_parameter("arm_base_frame", "arm_0_base_link")
        self.declare_parameter("world_frame", "base_link")

        self.color_topic = self.get_parameter("color_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.depth_camera_info_topic = self.get_parameter("depth_camera_info_topic").value

        self.marker_size = float(self.get_parameter("marker_size").value)
        self.aruco_dictionary_name = self.get_parameter("aruco_dictionary").value
        self.target_id = int(self.get_parameter("target_id").value)

        self.show_window = bool(self.get_parameter("show_window").value)
        self.publish_tf_enabled = bool(self.get_parameter("publish_tf").value)
        self.publish_rviz_markers_enabled = bool(
            self.get_parameter("publish_rviz_markers").value
        )

        self.arm_base_frame = self.get_parameter("arm_base_frame").value
        self.world_frame = self.get_parameter("world_frame").value

        self.bridge = CvBridge()

        self.latest_depth_image = None
        self.latest_depth_encoding = None

        self.camera_matrix = None
        self.dist_coeffs = None

        # Depth camera intrinsics + optical frame, used to register the
        # (unaligned) depth image against the color detection.
        self.depth_camera_matrix = None
        self.depth_frame_id = None

        self.aruco_backend = OpenCVArucoCompat(
            dictionary_name=self.aruco_dictionary_name,
            logger=self.get_logger()
        )

        self.color_sub = self.create_subscription(
            Image,
            self.color_topic,
            self.color_callback,
            qos_profile_sensor_data
        )

        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            qos_profile_sensor_data
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        self.depth_camera_info_sub = self.create_subscription(
            CameraInfo,
            self.depth_camera_info_topic,
            self.depth_camera_info_callback,
            qos_profile_sensor_data
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            "/aruco/debug_image",
            10
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            "/aruco/pose",
            10
        )

        self.pose_array_pub = self.create_publisher(
            PoseArray,
            "/aruco/poses/camera_frame",
            10
        )

        self.pose_array_arm_pub = self.create_publisher(
            PoseArray,
            "/aruco/poses/arm_base_frame",
            10
        )

        self.pose_array_world_pub = self.create_publisher(
            PoseArray,
            "/aruco/poses/world_frame",
            10
        )

        self.marker_ids_pub = self.create_publisher(
            Int32MultiArray,
            "/aruco/marker_ids",
            10
        )

        self.marker_array_pub = self.create_publisher(
            MarkerArray,
            "/aruco/marker_array",
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("ArUco depth ROS 2 node started.")
        self.get_logger().info(f"RGB topic: {self.color_topic}")
        self.get_logger().info(f"Depth topic: {self.depth_topic}")
        self.get_logger().info(f"Camera info topic: {self.camera_info_topic}")
        self.get_logger().info(f"Depth camera info topic: {self.depth_camera_info_topic}")
        self.get_logger().info(f"Marker size: {self.marker_size} m")
        self.get_logger().info(f"Dictionary: {self.aruco_dictionary_name}")
        self.get_logger().info(f"Target ID: {self.target_id} (-1 means all markers)")
        self.get_logger().info(f"Arm base frame: {self.arm_base_frame}")
        self.get_logger().info(f"World frame: {self.world_frame}")
        self.get_logger().info(f"Publish TF: {self.publish_tf_enabled}")
        self.get_logger().info(f"Publish RViz markers: {self.publish_rviz_markers_enabled}")
        self.get_logger().info(f"Show OpenCV window: {self.show_window}")

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)

        if len(msg.d) > 0:
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        else:
            self.dist_coeffs = np.zeros((5,), dtype=np.float64)

    def depth_camera_info_callback(self, msg):
        self.depth_camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        # The depth image lives in its own optical frame; we need its id to
        # look up the color -> depth transform when registering depth.
        self.depth_frame_id = msg.header.frame_id

    def depth_callback(self, msg):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="passthrough"
            )
            self.latest_depth_encoding = msg.encoding

        except Exception as exc:
            self.get_logger().error(f"Depth image conversion failed: {exc}")

    def backproject(self, u, v, depth_m):
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
        return (u - cx) * depth_m / fx, (v - cy) * depth_m / fy, depth_m

    def sample_marker_depth(self, tvec_color, color_frame_id):
        """
        Register the solvePnP marker position into the unaligned depth image,
        sample depth there, and return the refined position in the color frame.

        The depth stream is not aligned to color on this camera, so the depth
        and color sensors have different intrinsics and a physical baseline.
        Reusing the color marker pixel to index the depth image therefore reads
        the wrong location. Instead we transform the PnP position into the depth
        optical frame, project it with the depth intrinsics to find the matching
        depth pixel, sample there, then transform the result back to color.

        Returns:
            (depth_m, (x, y, z)) on success, where depth_m is the measured range
            and (x, y, z) is in `color_frame_id`. (0.0, None) if depth could not
            be sampled, in which case the caller keeps the PnP estimate.
        """
        if (self.latest_depth_image is None
                or self.depth_camera_matrix is None
                or self.depth_frame_id is None):
            return 0.0, None

        p = tvec_color.flatten()

        point_color = PointStamped()
        point_color.header.frame_id = color_frame_id
        point_color.point.x = float(p[0])
        point_color.point.y = float(p[1])
        point_color.point.z = float(p[2])

        # color optical frame -> depth optical frame
        try:
            tf_c2d = self.tf_buffer.lookup_transform(
                self.depth_frame_id,
                color_frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2)
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"No transform '{color_frame_id}' -> '{self.depth_frame_id}' "
                f"for depth registration: {exc}"
            )
            return 0.0, None

        point_depth = do_transform_point(point_color, tf_c2d)
        if point_depth.point.z <= 0.0:
            return 0.0, None

        # Project into the depth image using the depth intrinsics.
        fx = self.depth_camera_matrix[0, 0]
        fy = self.depth_camera_matrix[1, 1]
        cx = self.depth_camera_matrix[0, 2]
        cy = self.depth_camera_matrix[1, 2]

        u = int(round(fx * point_depth.point.x / point_depth.point.z + cx))
        v = int(round(fy * point_depth.point.y / point_depth.point.z + cy))

        depth_m = self.get_depth_at_pixel(
            self.latest_depth_image,
            self.latest_depth_encoding,
            u,
            v,
            window_size=5
        )
        if depth_m <= 0.0:
            return 0.0, None

        # Backproject the measured depth in the depth frame ...
        xd = (u - cx) * depth_m / fx
        yd = (v - cy) * depth_m / fy

        refined_depth = PointStamped()
        refined_depth.header.frame_id = self.depth_frame_id
        refined_depth.point.x = float(xd)
        refined_depth.point.y = float(yd)
        refined_depth.point.z = float(depth_m)

        # ... and bring it back into the color frame for publishing.
        try:
            tf_d2c = self.tf_buffer.lookup_transform(
                color_frame_id,
                self.depth_frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2)
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"No transform '{self.depth_frame_id}' -> '{color_frame_id}' "
                f"for depth registration: {exc}"
            )
            return depth_m, None

        refined_color = do_transform_point(refined_depth, tf_d2c)
        return depth_m, (
            refined_color.point.x,
            refined_color.point.y,
            refined_color.point.z,
        )

    def color_callback(self, msg):
        self.get_logger().debug("Received color image, processing for ArUco detection...")
        if self.camera_matrix is None:
            self.get_logger().warn("Waiting for camera_info...")
            return

        if self.latest_depth_image is None:
            self.get_logger().warn("Waiting for depth image...")
            return

        try:
            color_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )
        except Exception as exc:
            self.get_logger().error(f"RGB image conversion failed: {exc}")
            return

        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        try:
            corners, ids, marker_dicts = self.aruco_backend.detect_markers(gray_image)
        except Exception as exc:
            self.get_logger().error(f"ArUco detection failed: {exc}")
            return

        pose_array_msg = PoseArray()
        pose_array_msg.header = msg.header

        pose_array_arm_msg = PoseArray()
        pose_array_arm_msg.header.stamp = msg.header.stamp
        pose_array_arm_msg.header.frame_id = self.arm_base_frame

        pose_array_world_msg = PoseArray()
        pose_array_world_msg.header.stamp = msg.header.stamp
        pose_array_world_msg.header.frame_id = self.world_frame

        marker_ids_msg = Int32MultiArray()
        marker_array_msg = MarkerArray()

        if ids is not None:
            self.aruco_backend.draw_detected_markers(color_image, corners, ids)

            for i, marker_id_np in enumerate(ids.flatten()):
                marker_id = int(marker_id_np)
                dict_name = marker_dicts[i]

                if self.target_id != -1 and marker_id != self.target_id:
                    continue

                marker_corners = corners[i]

                success, rvec, tvec = self.estimate_marker_pose(marker_corners)
                if not success:
                    self.get_logger().warn(f"solvePnP failed for marker ID {marker_id}")
                    continue

                x, y, z = tvec.flatten()

                # Color-image marker center, used only for the debug overlay.
                center_x, center_y = self.get_marker_center(marker_corners)

                # Depth is not aligned to color on this camera, so the color
                # marker pixel does not map to the same depth pixel. Register
                # the PnP position into the depth image instead of reusing the
                # color pixel, then refine in the color frame.
                depth_m, depth_point = self.sample_marker_depth(
                    tvec, msg.header.frame_id
                )

                # Prefer depth-measured position; keep PnP only as fallback.
                if depth_m > 0.0 and depth_point is not None:
                    x, y, z = depth_point

                self.aruco_backend.draw_axes(
                    color_image,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    self.marker_size / 2.0
                )

                quat = self.rvec_to_quaternion(rvec)

                pose_msg = PoseStamped()
                pose_msg.header = msg.header

                pose_msg.pose.position.x = float(x)
                pose_msg.pose.position.y = float(y)
                pose_msg.pose.position.z = float(z)

                pose_msg.pose.orientation.x = float(quat[0])
                pose_msg.pose.orientation.y = float(quat[1])
                pose_msg.pose.orientation.z = float(quat[2])
                pose_msg.pose.orientation.w = float(quat[3])

                self.pose_pub.publish(pose_msg)
                pose_array_msg.poses.append(pose_msg.pose)
                marker_ids_msg.data.append(marker_id)

                arm_pose = self.transform_pose(pose_msg, self.arm_base_frame)
                if arm_pose is not None:
                    pose_array_arm_msg.poses.append(arm_pose.pose)

                world_pose = self.transform_pose(pose_msg, self.world_frame)
                if world_pose is not None:
                    pose_array_world_msg.poses.append(world_pose.pose)

                if self.publish_tf_enabled:
                    self.publish_aruco_tf(
                        msg.header, marker_id, dict_name, x, y, z, quat
                    )

                if self.publish_rviz_markers_enabled:
                    rviz_marker = self.create_rviz_marker(
                        msg.header, marker_id, dict_name, pose_msg.pose
                    )
                    marker_array_msg.markers.append(rviz_marker)

                self.draw_text(
                    color_image,
                    marker_id,
                    center_x,
                    center_y,
                    depth_m,
                    x,
                    y,
                    z
                )

                self.get_logger().info(
                    f"Marker ID: {marker_id} | "
                    f"Depth: {depth_m:.3f} m | "
                    f"Camera-frame pose: x={x:.3f}, y={y:.3f}, z={z:.3f} m | "
                    f"Frame: {msg.header.frame_id}"
                )

                if arm_pose is not None:
                    self.get_logger().info(
                        f"Marker ID: {marker_id} | "
                        f"Arm-frame pose: "
                        f"x={arm_pose.pose.position.x:.3f}, "
                        f"y={arm_pose.pose.position.y:.3f}, "
                        f"z={arm_pose.pose.position.z:.3f} m | "
                        f"Frame: {arm_pose.header.frame_id}"
                    )

        self.pose_array_pub.publish(pose_array_msg)
        self.pose_array_arm_pub.publish(pose_array_arm_msg)
        self.pose_array_world_pub.publish(pose_array_world_msg)
        self.marker_ids_pub.publish(marker_ids_msg)

        if self.publish_rviz_markers_enabled:
            self.marker_array_pub.publish(marker_array_msg)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(
                color_image,
                encoding="bgr8"
            )
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)

        except Exception as exc:
            self.get_logger().error(f"Debug image publish failed: {exc}")

        if self.show_window:
            # Keep this false on Linux if your OpenCV GUI backend is unstable.
            cv2.imshow("ROS 2 ArUco Depth Detection", color_image)
            cv2.waitKey(1)

    def estimate_marker_pose(self, marker_corners):
        half_size = self.marker_size / 2.0

        object_points = np.array([
            [-half_size,  half_size, 0.0],
            [ half_size,  half_size, 0.0],
            [ half_size, -half_size, 0.0],
            [-half_size, -half_size, 0.0]
        ], dtype=np.float64)

        image_points = marker_corners.reshape((4, 2)).astype(np.float64)

        solvepnp_flag = (
            cv2.SOLVEPNP_IPPE_SQUARE
            if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE")
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

    def get_marker_center(self, marker_corners):
        c = marker_corners.reshape((4, 2))

        center_x = int(np.mean(c[:, 0]))
        center_y = int(np.mean(c[:, 1]))

        return center_x, center_y

    def get_depth_at_pixel(self, depth_image, encoding, center_x, center_y, window_size=5):
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

        # Most ROS depth images use:
        # 16UC1: millimetres
        # 32FC1: metres
        if encoding == "16UC1" or depth_image.dtype == np.uint16:
            return median_depth * 0.001

        return median_depth

    def rvec_to_quaternion(self, rvec):
        rotation_matrix, _ = cv2.Rodrigues(rvec)

        R = rotation_matrix
        trace = np.trace(R)

        if trace > 0.0:
            s = np.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s

        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s

        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s

        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s

        quat = np.array([qx, qy, qz, qw], dtype=np.float64)

        norm = np.linalg.norm(quat)
        if norm > 0.0:
            quat = quat / norm

        return quat

    def transform_pose(self, pose_msg, target_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                pose_msg.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2)
            )

            transformed = do_transform_pose_stamped(pose_msg, transform)
            transformed.header.stamp = pose_msg.header.stamp
            transformed.header.frame_id = target_frame
            return transformed

        except TransformException as exc:
            self.get_logger().warn(
                f"Could not transform from '{pose_msg.header.frame_id}' "
                f"to '{target_frame}': {exc}"
            )
            return None

    def publish_aruco_tf(self, header, marker_id, dict_name, x, y, z, quat):
        transform = TransformStamped()
        transform.header.stamp = header.stamp
        transform.header.frame_id = header.frame_id
        transform.child_frame_id = f"aruco_{dict_name}_{marker_id}"
        transform.transform.translation.x = float(x)
        transform.transform.translation.y = float(y)
        transform.transform.translation.z = float(z)
        transform.transform.rotation.x = float(quat[0])
        transform.transform.rotation.y = float(quat[1])
        transform.transform.rotation.z = float(quat[2])
        transform.transform.rotation.w = float(quat[3])
        self.tf_broadcaster.sendTransform(transform)

    def draw_text(self, image, marker_id, center_x, center_y, depth_m, x, y, z):
        cv2.circle(
            image,
            (center_x, center_y),
            5,
            (0, 255, 0),
            -1
        )

        cv2.putText(
            image,
            f"ID: {marker_id}",
            (center_x + 10, center_y - 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

        cv2.putText(
            image,
            f"Depth: {depth_m:.3f} m",
            (center_x + 10, center_y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

        cv2.putText(
            image,
            f"Pose: x={x:.2f}, y={y:.2f}, z={z:.2f}",
            (center_x + 10, center_y + 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2
        )

    def create_rviz_marker(self, header, marker_id, dict_name, pose):
        marker = Marker()

        marker.header = header
        marker.ns = f"aruco_{dict_name}"      # ns per family avoids id overwrite
        marker.id = int(marker_id)

        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose

        marker.scale.x = float(self.marker_size)
        marker.scale.y = float(self.marker_size)
        marker.scale.z = 0.005

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 300_000_000

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = ArucoDepthNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        show_window = False

        if node is not None:
            show_window = bool(getattr(node, "show_window", False))
            node.destroy_node()

        # Safer on Linux: only call this if you actually created OpenCV windows.
        if show_window:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
