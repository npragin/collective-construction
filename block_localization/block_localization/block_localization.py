"""Block localization via RealSense ArUco/AprilTag detection.

Recovers each detected marker's pose in the `map` (global) frame by
combining onboard depth + PnP with the externally-supplied robot pose,
and publishes the cumulative set on `{ns}/found_blocks`.

Position comes from depth back-projection of the marker centroid;
orientation from PnP on the four corners.
"""

import math

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from geometry_msgs.msg import Pose, Quaternion

from cc_interfaces.msg import FoundBlock, FoundBlockArray


ARUCO_DICTIONARIES = {
    "DICT_4X4_50":         cv2.aruco.DICT_4X4_50,
    "DICT_5X5_50":         cv2.aruco.DICT_5X5_50,
    "DICT_6X6_50":         cv2.aruco.DICT_6X6_50,
    "DICT_APRILTAG_16h5":  cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9":  cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


# Jazzy on the Jetson currently ships OpenCV 4.6 (legacy aruco API);
# desktop installs are 4.7+ with ArucoDetector. Support both.
if hasattr(cv2.aruco, "ArucoDetector"):
    def _make_detector(dict_id):
        d = cv2.aruco.getPredefinedDictionary(dict_id)
        return cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())

    def _detect(detector, gray):
        return detector.detectMarkers(gray)
else:
    def _make_detector(dict_id):
        return (cv2.aruco.Dictionary_get(dict_id),
                cv2.aruco.DetectorParameters_create())

    def _detect(detector, gray):
        d, params = detector
        return cv2.aruco.detectMarkers(gray, d, parameters=params)


# REP-103 optical (X right, Y down, Z forward) -> body (X fwd, Y left, Z up).
R_OPT_BODY = np.array([
    [ 0,  0,  1],
    [-1,  0,  0],
    [ 0, -1,  0],
], dtype=np.float64)


def yaw_from_q(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def q_from_yaw(yaw):
    half = yaw * 0.5
    q = Quaternion()
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def euler_to_R(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [    -sp,                cp * sr,                cp * cr],
    ], dtype=np.float64)


class BlockLocalization(Node):
    def __init__(self):
        super().__init__("block_localization")
        self.declare_parameter("aruco_dict",        "DICT_APRILTAG_16h5")
        self.declare_parameter("marker_size_m",     0.0413)
        self.declare_parameter("camera_xyz",        [0.10, 0.0, 0.30])
        self.declare_parameter("camera_rpy",        [0.0, 0.0, 0.0])
        self.declare_parameter("image_topic",       "camera/color/image_raw")
        self.declare_parameter("depth_topic",       "camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "camera/color/camera_info")
        self.declare_parameter("output_topic",      "found_blocks")
        self.declare_parameter("pose_timeout_sec",  1.0)
        self.declare_parameter("block_ttl_sec",     30.0)
        self.declare_parameter("global_frame",      "map")
        p = self.get_parameter
        self.aruco_dict        = p("aruco_dict").value
        self.marker_size_m     = float(p("marker_size_m").value)
        self.camera_xyz        = np.asarray(p("camera_xyz").value, dtype=np.float64).reshape(3, 1)
        self.R_mount           = euler_to_R(p("camera_rpy").value)
        self.image_topic       = p("image_topic").value
        self.depth_topic       = p("depth_topic").value
        self.camera_info_topic = p("camera_info_topic").value
        self.output_topic      = p("output_topic").value
        self.pose_timeout      = Duration(seconds=float(p("pose_timeout_sec").value))
        self.block_ttl         = Duration(seconds=float(p("block_ttl_sec").value))
        self.global_frame      = p("global_frame").value

        self.bridge = CvBridge()
        self.detector = _make_detector(ARUCO_DICTIONARIES[self.aruco_dict])
        self.R_cam_body = self.R_mount @ R_OPT_BODY
        self.camera_matrix = None
        self.distortion = None
        self.pose = None
        self.pose_stamp = None
        self.blocks_by_id = {}     # tag_id -> FoundBlock
        self.last_seen = {}        # tag_id -> rclpy.time.Time
        self.logger = self.get_logger()

        s = self.marker_size_m / 2.0
        self.obj_pts = np.array([
            [-s,  s, 0.0],
            [ s,  s, 0.0],
            [ s, -s, 0.0],
            [-s, -s, 0.0],
        ], dtype=np.float64)
        # Reusable scratch buffers for the hot path.
        self._T_marker_opt = np.zeros((3, 1), dtype=np.float64)
        self._R_body_world = np.eye(3, dtype=np.float64)
        self._T_body_world = np.zeros((3, 1), dtype=np.float64)

        ns = self.get_namespace().rstrip("/")
        img_sub   = message_filters.Subscriber(self, Image,
                       f"{ns}/{self.image_topic}",       qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(self, Image,
                       f"{ns}/{self.depth_topic}",       qos_profile=qos_profile_sensor_data)
        info_sub  = message_filters.Subscriber(self, CameraInfo,
                       f"{ns}/{self.camera_info_topic}", qos_profile=qos_profile_sensor_data)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [img_sub, depth_sub, info_sub], queue_size=5, slop=0.05)
        self.ts.registerCallback(self.on_image)

        self.create_subscription(Pose, f"{ns}/pose", self.on_pose, 10)

        latched = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(FoundBlockArray, self.output_topic, latched)
        self.pub_debug = self.create_publisher(
            CompressedImage, f"{ns}/block_localization/debug_image/compressed",
            qos_profile_sensor_data)

        self.logger.info(
            f"block_localization up, ns={ns or '/'}, dict={self.aruco_dict}, "
            f"size={self.marker_size_m:.3f}m, frame={self.global_frame}"
        )

    def on_pose(self, msg):
        self.pose = msg
        self.pose_stamp = self.get_clock().now()

    def _publish_debug(self, img, corners, ids, stamp):
        if self.pub_debug.get_subscription_count() == 0:
            return
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(img, corners, ids, (0, 255, 0))
        ok, jpg = cv2.imencode(".jpg", img,
                               [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_color_optical_frame"
        msg.format = "jpeg"
        msg.data = jpg.tobytes()
        self.pub_debug.publish(msg)

    def on_image(self, color_msg, depth_msg, info_msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(info_msg.k, dtype=np.float64).reshape(3, 3)
            self.distortion   = np.array(info_msg.d, dtype=np.float64)
        now = self.get_clock().now()
        if self.pose is None or now - self.pose_stamp > self.pose_timeout:
            return

        try:
            img = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.logger.warn(f"cv_bridge: {e}")
            return
        depth_scale = 0.001 if depth_msg.encoding == "16UC1" else 1.0
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = _detect(self.detector, gray)
        self._publish_debug(img, corners, ids, color_msg.header.stamp)

        changed = False
        if ids is not None:
            for c, tag_id in zip(corners, ids.flatten()):
                tag_id = int(tag_id)
                block = self._estimate(c, tag_id, depth, depth_scale, now)
                if block is None:
                    continue
                self.blocks_by_id[tag_id] = block
                self.last_seen[tag_id] = now
                changed = True

        # Evict blocks not seen within block_ttl.
        for stale_id in [tid for tid, t in self.last_seen.items()
                         if now - t > self.block_ttl]:
            self.last_seen.pop(stale_id, None)
            self.blocks_by_id.pop(stale_id, None)
            changed = True

        if changed:
            out = FoundBlockArray()
            out.header.frame_id = self.global_frame
            out.header.stamp = now.to_msg()
            out.blocks = list(self.blocks_by_id.values())
            self.pub.publish(out)

    def _estimate(self, corners, tag_id, depth, depth_scale, now):
        pts = corners.reshape(4, 2)
        u, v = float(pts[:, 0].mean()), float(pts[:, 1].mean())
        iu, iv = int(round(u)), int(round(v))
        h, w = depth.shape[:2]
        if not (0 <= iu < w and 0 <= iv < h):
            return None

        z = float(depth[iv, iu]) * depth_scale
        if z <= 0.0 or not math.isfinite(z):
            return None

        fx, fy   = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        ppx, ppy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
        self._T_marker_opt[0, 0] = (u - ppx) * z / fx
        self._T_marker_opt[1, 0] = (v - ppy) * z / fy
        self._T_marker_opt[2, 0] = z

        pts_f = pts.astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(
            self.obj_pts, pts_f, self.camera_matrix, self.distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        R_marker_opt, _ = cv2.Rodrigues(rvec)

        # Confidence: RMS pixel error between corners and reprojected obj_pts.
        proj, _ = cv2.projectPoints(self.obj_pts, rvec, tvec,
                                    self.camera_matrix, self.distortion)
        rms = float(np.sqrt(np.mean(np.sum(
            (proj.reshape(4, 2) - pts_f) ** 2, axis=1))))
        confidence = max(0.0, 1.0 - rms / 10.0)

        T_marker_body = self.R_cam_body @ self._T_marker_opt + self.camera_xyz

        ryaw = yaw_from_q(self.pose.orientation)
        cy, sy = math.cos(ryaw), math.sin(ryaw)
        self._R_body_world[0, 0] = cy
        self._R_body_world[0, 1] = -sy
        self._R_body_world[1, 0] = sy
        self._R_body_world[1, 1] = cy
        self._T_body_world[0, 0] = self.pose.position.x
        self._T_body_world[1, 0] = self.pose.position.y
        R_marker_world = self._R_body_world @ self.R_cam_body @ R_marker_opt
        T_marker_world = self._R_body_world @ T_marker_body + self._T_body_world
        yaw_marker = math.atan2(R_marker_world[1, 0], R_marker_world[0, 0])

        block = FoundBlock()
        block.tag_id = tag_id
        block.pose.position.x = float(T_marker_world[0, 0])
        block.pose.position.y = float(T_marker_world[1, 0])
        block.pose.position.z = float(T_marker_world[2, 0])
        block.pose.orientation = q_from_yaw(yaw_marker)
        block.confidence = confidence
        block.last_seen = now.to_msg()
        return block


def main(args=None):
    rclpy.init(args=args)
    node = BlockLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
