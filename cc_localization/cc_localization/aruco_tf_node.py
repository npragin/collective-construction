# ruff: noqa: N803, N806
"""
Publish outer and inner ArUco workspace frames on tf.

Each frame's four corner tags are laid out counter-clockwise from the origin:
``origin -> +X -> +X+Y -> +Y``.
"""

from pathlib import Path

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory

import rclpy
from geometry_msgs.msg import Point, TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

# Hard-coded tag ids per frame. Convention: counter-clockwise starting at origin
# (origin -> +X -> +X+Y -> +Y). Edit here if the printed tags change.
OUTER_TAG_IDS = (0, 1, 2, 3)
INNER_TAG_IDS = (4, 5, 6, 7)
FRAME_TAG_IDS = set(OUTER_TAG_IDS) | set(INNER_TAG_IDS)


def corners_for(ids: tuple[int, int, int, int], length: float, width: float) -> dict[int, tuple[float, float]]:
    """Map the four corner tag ids to their (x, y) positions in their own frame."""
    origin, plus_x, plus_xy, plus_y = ids
    return {
        origin: (0.0, 0.0),
        plus_x: (length, 0.0),
        plus_xy: (length, width),
        plus_y: (0.0, width),
    }


def tag_local(marker_size: float) -> np.ndarray:
    """Return the four marker corners in the marker's local frame (z=0)."""
    half = marker_size / 2.0
    return np.array(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
        dtype=np.float32,
    )


def rotation_matrix_to_quaternion(R: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to a (x, y, z, w) quaternion."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


class ArucoTfNode(Node):
    """Detect ArUco frames and broadcast world/inner transforms on tf."""

    def __init__(self) -> None:
        """Declare parameters, open the camera, and start the detection timer."""
        super().__init__("cc_localization")

        self.declare_parameter("device_id", 42)
        self.declare_parameter("marker_size", 0.15)
        self.declare_parameter("world_length", 1.35)
        self.declare_parameter("world_width", 0.885)
        self.declare_parameter("inner_length", 0.41)
        self.declare_parameter("inner_width", 0.54)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("inner_frame", "inner")
        self.declare_parameter("camera_frame", "camera")
        self.declare_parameter("tick_rate_hz", 30.0)

        device_id = self.get_parameter("device_id").get_parameter_value().integer_value
        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        world_length = self.get_parameter("world_length").get_parameter_value().double_value
        world_width = self.get_parameter("world_width").get_parameter_value().double_value
        inner_length = self.get_parameter("inner_length").get_parameter_value().double_value
        inner_width = self.get_parameter("inner_width").get_parameter_value().double_value
        self.world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        self.inner_frame = self.get_parameter("inner_frame").get_parameter_value().string_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        tick_rate_hz = self.get_parameter("tick_rate_hz").get_parameter_value().double_value

        self.outer_corners = corners_for(OUTER_TAG_IDS, world_length, world_width)
        self.inner_corners = corners_for(INNER_TAG_IDS, inner_length, inner_width)
        self.outer_dims = (world_length, world_width)
        self.inner_dims = (inner_length, inner_width)
        self.tag_local = tag_local(self.marker_size)

        calib_path = Path(get_package_share_directory("cc_localization")) / "config" / "gopro_calib.npz"
        calib = np.load(calib_path)
        self.mtx = calib["camera_matrix"]
        self.dist = calib["dist_coeffs"]

        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
        )
        self.cap = cv2.VideoCapture(device_id)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open video device {device_id}")

        self.broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(MarkerArray, "workspace_markers", 1)
        self.timer = self.create_timer(1.0 / tick_rate_hz, self.tick)

    def destroy_node(self) -> None:
        """Release the camera before standard node teardown."""
        self.cap.release()
        super().destroy_node()

    def solve_frame(
        self,
        frame_corners: dict[int, tuple[float, float]],
        ids_flat: np.ndarray,
        corners: tuple,
    ) -> tuple[bool, np.ndarray | None, np.ndarray | None]:
        """Solve PnP for a frame given its corner-tag layout. Returns (ok, rvec, tvec)."""
        obj_list, img_list = [], []
        for tag_id, (cx, cy) in frame_corners.items():
            if tag_id not in ids_flat:
                continue
            idx = int(np.where(ids_flat == tag_id)[0][0])
            obj_list.append(self.tag_local + np.array([cx, cy, 0.0], dtype=np.float32))
            img_list.append(corners[idx].reshape(-1, 2))
        if not obj_list:
            return False, None, None
        obj_pts = np.concatenate(obj_list, axis=0)
        img_pts = np.concatenate(img_list, axis=0).astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self.mtx, self.dist)
        return ok, rvec, tvec

    def make_transform(self, parent: str, child: str, R: np.ndarray, t: np.ndarray) -> TransformStamped:
        """Build a stamped transform message from a rotation matrix and translation vector."""
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x = float(t[0])
        msg.transform.translation.y = float(t[1])
        msg.transform.translation.z = float(t[2])
        qx, qy, qz, qw = rotation_matrix_to_quaternion(R)
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        return msg

    def make_rect_marker(
        self,
        frame: str,
        marker_id: int,
        length: float,
        width: float,
        color: tuple[float, float, float],
    ) -> Marker:
        """Build a closed-rectangle LINE_STRIP marker spanning (0,0) -> (L,0) -> (L,W) -> (0,W)."""
        m = Marker()
        # Leave stamp at zero so RViz uses the latest available tf for this frame,
        # avoiding races between tf and marker publish times.
        m.header.frame_id = frame
        m.ns = "workspace_bounds"
        m.id = marker_id
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.01  # line width
        m.color.r, m.color.g, m.color.b = color
        m.color.a = 1.0
        m.pose.orientation.w = 1.0
        corners = [(0.0, 0.0), (length, 0.0), (length, width), (0.0, width), (0.0, 0.0)]
        m.points = [Point(x=x, y=y, z=0.0) for x, y in corners]
        return m

    def tick(self) -> None:
        """Read a frame, solve PnP for both frames, and broadcast available transforms."""
        ret, frame = self.cap.read()
        if not ret:
            return

        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is None:
            return
        ids_flat = ids.flatten()

        outer_ok, rvec_w, tvec_w = self.solve_frame(self.outer_corners, ids_flat, corners)
        inner_ok, rvec_i, tvec_i = self.solve_frame(self.inner_corners, ids_flat, corners)

        transforms = []

        # world -> camera: invert the outer PnP (which gives world points in camera coords)
        if outer_ok:
            R_wc, _ = cv2.Rodrigues(rvec_w)
            R_cw = R_wc.T
            t_cw = -R_cw @ tvec_w.flatten()
            transforms.append(self.make_transform(self.world_frame, self.camera_frame, R_cw, t_cw))

        # world -> inner: compose outer^-1 with inner (both expressed in camera coords)
        if outer_ok and inner_ok:
            R_wc, _ = cv2.Rodrigues(rvec_w)
            R_ic, _ = cv2.Rodrigues(rvec_i)
            R_wi = R_wc.T @ R_ic
            t_wi = R_wc.T @ (tvec_i.flatten() - tvec_w.flatten())
            transforms.append(self.make_transform(self.world_frame, self.inner_frame, R_wi, t_wi))

        # world -> aruco_{id} for any detected tag that doesn't define a frame
        if outer_ok:
            R_wc, _ = cv2.Rodrigues(rvec_w)
            for i, tag_id in enumerate(ids_flat):
                tag_id_int = int(tag_id)
                if tag_id_int in FRAME_TAG_IDS:
                    continue
                ok, rvec_t, tvec_t = cv2.solvePnP(
                    self.tag_local, corners[i], self.mtx, self.dist, False, cv2.SOLVEPNP_IPPE_SQUARE,
                )
                if not ok:
                    continue
                R_tc, _ = cv2.Rodrigues(rvec_t)
                R_wt = R_wc.T @ R_tc
                t_wt = R_wc.T @ (tvec_t.flatten() - tvec_w.flatten())
                transforms.append(self.make_transform(self.world_frame, f"aruco_{tag_id_int}", R_wt, t_wt))

        if transforms:
            self.broadcaster.sendTransform(transforms)

        markers = MarkerArray()
        if outer_ok:
            wl, ww = self.outer_dims
            markers.markers.append(
                self.make_rect_marker(self.world_frame, 0, wl, ww, (0.1, 0.9, 0.1)),
            )
        if outer_ok and inner_ok:
            il, iw = self.inner_dims
            markers.markers.append(
                self.make_rect_marker(self.inner_frame, 1, il, iw, (0.1, 0.5, 1.0)),
            )
        if markers.markers:
            self.marker_pub.publish(markers)


def main() -> None:
    """Spin the ArucoTfNode until interrupted."""
    rclpy.init()
    node = ArucoTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
