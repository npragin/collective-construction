#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Int32MultiArray

from cv_bridge import CvBridge
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


class ArucoDepthNode(Node):
    def __init__(self):
        super().__init__("aruco_depth_node")

      
        self.declare_parameter("color_topic", "/j100_0897/sensors/camera_0/color/image")
        self.declare_parameter("depth_topic", "/j100_0897/sensors/camera_0/depth/image")
        self.declare_parameter("camera_info_topic", "/j100_0897/sensors/camera_0/color/camera_info")

        self.declare_parameter("marker_size", 0.05)          # 50 mm = 0.05 m(the sixe of the aruco)
        self.declare_parameter("aruco_dictionary", "original")
        self.declare_parameter("target_id", -1)              # -1 means detect all ids

        self.declare_parameter("show_window", False)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_rviz_markers", True)
        self.declare_parameter("arm_base_frame", "arm_0_base_link")
        self.declare_parameter("world_frame", "base_link")

        self.color_topic = self.get_parameter("color_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value

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

        
        self.aruco_dict = self.get_aruco_dictionary(self.aruco_dictionary_name)

        if hasattr(cv2.aruco, "DetectorParameters"):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(
                self.aruco_dict,
                self.aruco_params
            )
        else:
            self.detector = None

        
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
        self.marker_ids_pub = self.create_publisher(Int32MultiArray, "/aruco/marker_ids", 10)

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
        self.get_logger().info(f"Marker size: {self.marker_size} m")
        self.get_logger().info(f"Dictionary: {self.aruco_dictionary_name}")
        self.get_logger().info(f"Target ID: {self.target_id}  (-1 means all markers)")
        self.get_logger().info(f"Arm base frame: {self.arm_base_frame}")
        self.get_logger().info(f"World frame: {self.world_frame}")
        self.get_logger().info(f"Publish TF: {self.publish_tf_enabled}")
        self.get_logger().info(f"Publish RViz markers: {self.publish_rviz_markers_enabled}")

    

    def get_aruco_dictionary(self, name):
        dictionaries = {
            "original": cv2.aruco.DICT_ARUCO_ORIGINAL,
            "4x4_50": cv2.aruco.DICT_4X4_50,
            "4x4_100": cv2.aruco.DICT_4X4_100,
            "4x4_250": cv2.aruco.DICT_4X4_250,
            "4x4_1000": cv2.aruco.DICT_4X4_1000,
            "5x5_50": cv2.aruco.DICT_5X5_50,
            "5x5_100": cv2.aruco.DICT_5X5_100,
            "5x5_250": cv2.aruco.DICT_5X5_250,
            "5x5_1000": cv2.aruco.DICT_5X5_1000,
            "6x6_50": cv2.aruco.DICT_6X6_50,
            "6x6_100": cv2.aruco.DICT_6X6_100,
            "6x6_250": cv2.aruco.DICT_6X6_250,
            "6x6_1000": cv2.aruco.DICT_6X6_1000,
            "7x7_50": cv2.aruco.DICT_7X7_50,
            "7x7_100": cv2.aruco.DICT_7X7_100,
            "7x7_250": cv2.aruco.DICT_7X7_250,
            "7x7_1000": cv2.aruco.DICT_7X7_1000,
        }

        if name not in dictionaries:
            self.get_logger().warn(
                f"Unknown dictionary '{name}', using original ArUco."
            )
            name = "original"

        return cv2.aruco.getPredefinedDictionary(dictionaries[name])

    
    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k, dtype=np.float32).reshape(3, 3)

        if len(msg.d) > 0:
            self.dist_coeffs = np.array(msg.d, dtype=np.float32)
        else:
            self.dist_coeffs = np.zeros((5,), dtype=np.float32)

    def depth_callback(self, msg):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="passthrough"
            )
            self.latest_depth_encoding = msg.encoding

        except Exception as e:
            self.get_logger().error(f"Depth image conversion failed: {e}")

    def color_callback(self, msg):
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
        except Exception as e:
            self.get_logger().error(f"RGB image conversion failed: {e}")
            return

        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = self.detect_markers(gray_image)

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
            cv2.aruco.drawDetectedMarkers(color_image, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):

                # If target_id = -1, process all markers.
                # Otherwise, process only the selected marker ID.
                if self.target_id != -1 and int(marker_id) != self.target_id:
                    continue

                marker_corners = corners[i]

                
                success, rvec, tvec = self.estimate_marker_pose(marker_corners)

                if not success:
                    continue

                x, y, z = tvec.flatten()

                
                center_x, center_y = self.get_marker_center(marker_corners)

                
                depth_m = self.get_depth_at_pixel(
                    self.latest_depth_image,
                    self.latest_depth_encoding,
                    center_x,
                    center_y,
                    window_size=5
                )

                
                cv2.drawFrameAxes(
                    color_image,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    self.marker_size / 2.0
                )

               
                quat = self.rvec_to_quaternion(rvec)

               
                if self.publish_tf_enabled:
                    self.publish_aruco_tf(
                        msg.header,
                        int(marker_id),
                        x,
                        y,
                        z,
                        quat
                    )

                
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
                marker_ids_msg.data.append(int(marker_id))
                arm_pose = self.transform_pose(pose_msg, self.arm_base_frame)
                if arm_pose is not None:
                    pose_array_arm_msg.poses.append(arm_pose.pose)
                world_pose = self.transform_pose(pose_msg, self.world_frame)
                if world_pose is not None:
                    pose_array_world_msg.poses.append(world_pose.pose)

                
                if self.publish_rviz_markers_enabled:
                    rviz_marker = self.create_rviz_marker(
                        msg.header,
                        int(marker_id),
                        pose_msg.pose
                    )
                    marker_array_msg.markers.append(rviz_marker)

                
                self.draw_text(
                    color_image,
                    int(marker_id),
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
                    f"Pose: x={x:.3f}, y={y:.3f}, z={z:.3f} m | "
                    f"Frame: {msg.header.frame_id}"
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

        except Exception as e:
            self.get_logger().error(f"Debug image publish failed: {e}")

        
        if self.show_window:
            cv2.imshow("ROS 2 ArUco Depth Detection", color_image)
            cv2.waitKey(1)

    

    def detect_markers(self, gray_image):
        if self.detector is not None:
            corners, ids, rejected = self.detector.detectMarkers(gray_image)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray_image,
                self.aruco_dict,
                parameters=self.aruco_params
            )

        return corners, ids, rejected

    def estimate_marker_pose(self, marker_corners):
        half_size = self.marker_size / 2.0

        
        object_points = np.array([
            [-half_size,  half_size, 0.0],
            [ half_size,  half_size, 0.0],
            [ half_size, -half_size, 0.0],
            [-half_size, -half_size, 0.0]
        ], dtype=np.float32)

        image_points = marker_corners.reshape((4, 2)).astype(np.float32)

        if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE"):
            solvepnp_flag = cv2.SOLVEPNP_IPPE_SQUARE
        else:
            solvepnp_flag = cv2.SOLVEPNP_ITERATIVE

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
        h, w = depth_image.shape[:2]

        half_window = window_size // 2
        depth_values = []

        for y in range(center_y - half_window, center_y + half_window + 1):
            for x in range(center_x - half_window, center_x + half_window + 1):

                if x < 0 or y < 0 or x >= w or y >= h:
                    continue

                d = depth_image[y, x]

                if np.isnan(d) or d <= 0:
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

        return np.array([qx, qy, qz, qw], dtype=np.float64)


    # TF helper
    def transform_pose(self, pose_msg, target_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                pose_msg.header.frame_id,
                rclpy.time.Time(),
            )
            transformed = do_transform_pose(pose_msg, transform)
            transformed.header.stamp = pose_msg.header.stamp
            transformed.header.frame_id = target_frame
            return transformed
        except TransformException as ex:
            self.get_logger().debug(
                f"Could not transform {pose_msg.header.frame_id} to {target_frame}: {ex}"
            )
            return None


    def publish_aruco_tf(self, header, marker_id, x, y, z, quat):
        transform = TransformStamped()

        transform.header.stamp = header.stamp
        transform.header.frame_id = header.frame_id

        transform.child_frame_id = f"aruco_marker_{marker_id}"

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


    def create_rviz_marker(self, header, marker_id, pose):
        marker = Marker()

        marker.header = header
        marker.ns = "aruco_markers"
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

    node = ArucoDepthNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
