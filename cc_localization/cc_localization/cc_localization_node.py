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
from cc_interfaces.msg import Stockpiles

import rclpy
from geometry_msgs.msg import Point, Point32, Polygon, PolygonStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

# Hard-coded tag ids per frame. Convention: counter-clockwise starting at origin
# (origin -> +X -> +X+Y -> +Y). Edit here if the printed tags change.
OUTER_TAG_IDS = (0, 1, 2, 3)
INNER_TAG_IDS = (4, 5, 6, 7)
# Each stockpile tag sits at the center of its own rectangle (single-tag frame).
STOCKPILE_TAG_IDS = (8, 9, 10)
NON_ROBOT_TAG_IDS = set(OUTER_TAG_IDS) | set(INNER_TAG_IDS) | set(STOCKPILE_TAG_IDS)


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


def yaw_only(R: np.ndarray) -> np.ndarray:
    """Project a 3x3 rotation onto its yaw-about-z component."""
    yaw = float(np.arctan2(R[1, 0], R[0, 0]))
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def ema_step(
    prev: tuple[np.ndarray, float] | None,
    pos_xy: np.ndarray,
    yaw: float,
    alpha: float,
) -> tuple[np.ndarray, float]:
    """One EMA step on (xy position, yaw). Yaw is averaged via unit-vector mean."""
    if prev is None:
        return pos_xy, yaw
    prev_pos, prev_yaw = prev
    smoothed_pos = alpha * pos_xy + (1.0 - alpha) * prev_pos
    cs = alpha * np.cos(yaw) + (1.0 - alpha) * np.cos(prev_yaw)
    sn = alpha * np.sin(yaw) + (1.0 - alpha) * np.sin(prev_yaw)
    smoothed_yaw = float(np.arctan2(sn, cs))
    return smoothed_pos, smoothed_yaw


def rect_in_world(R2: np.ndarray, t2: np.ndarray, length: float, width: float) -> np.ndarray:
    """Transform a (0,0)->(L,W) rectangle by a 2x2 rotation and 2D translation."""
    local = np.array([[0.0, 0.0], [length, 0.0], [length, width], [0.0, width]])
    return (R2 @ local.T).T + t2


def image_to_grid_homography(
    R_cw: np.ndarray,
    t_w: np.ndarray,
    mtx: np.ndarray,
    resolution: float,
) -> np.ndarray:
    """
    Build the 3x3 homography mapping image pixels to KOZ-grid cell coordinates.

    Uses the world->camera pose (R_wc = R_cw.T, t_wc = t_w) and intrinsics K to
    construct H_wi = K @ [r1 | r2 | t_wc] (ground-plane projection). Inverts to
    get image->world (meters), then scales by 1/resolution to land in grid cells.
    """
    R_wc = R_cw.T
    H_wi = mtx @ np.column_stack((R_wc[:, 0], R_wc[:, 1], t_w))
    H_iw_meters = np.linalg.inv(H_wi)
    S = np.array(
        [[1.0 / resolution, 0.0, 0.0], [0.0, 1.0 / resolution, 0.0], [0.0, 0.0, 1.0]]
    )
    return S @ H_iw_meters


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


class CcLocalizationNode(Node):
    """Detect ArUco frames and broadcast world/inner transforms on tf."""

    def __init__(self) -> None:
        """Declare parameters, open the camera, and start the detection timer."""
        super().__init__("cc_localization")

        self.declare_parameter("device_id", 42)
        self.declare_parameter("marker_size", 0.15)
        self.declare_parameter("world_length", 1.27)
        self.declare_parameter("world_width", 1.45)
        self.declare_parameter("inner_length", 0.395)
        self.declare_parameter("inner_width", 0.36)
        self.declare_parameter("stockpile_length", 0.15)
        self.declare_parameter("stockpile_width", 0.15)
        self.declare_parameter("koz_mask_resolution", 0.01)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("inner_frame", "build")
        self.declare_parameter("camera_frame", "camera")
        self.declare_parameter("tick_rate_hz", 30.0)
        self.declare_parameter("stockpile_ema_alpha", 0.1)
        self.declare_parameter("orange_hsv_low", [0.0, 128.0, 200.0])
        self.declare_parameter("orange_hsv_high", [15.0, 255.0, 255.0])
        self.declare_parameter("orange_morph_kernel", 3)

        device_id = self.get_parameter("device_id").get_parameter_value().integer_value
        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        world_length = self.get_parameter("world_length").get_parameter_value().double_value
        world_width = self.get_parameter("world_width").get_parameter_value().double_value
        inner_length = self.get_parameter("inner_length").get_parameter_value().double_value
        inner_width = self.get_parameter("inner_width").get_parameter_value().double_value
        self.stockpile_dims = (
            self.get_parameter("stockpile_length").get_parameter_value().double_value,
            self.get_parameter("stockpile_width").get_parameter_value().double_value,
        )
        self.koz_resolution = self.get_parameter("koz_mask_resolution").get_parameter_value().double_value
        self.world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        self.inner_frame = self.get_parameter("inner_frame").get_parameter_value().string_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        tick_rate_hz = self.get_parameter("tick_rate_hz").get_parameter_value().double_value
        self.stockpile_ema_alpha = (
            self.get_parameter("stockpile_ema_alpha").get_parameter_value().double_value
        )
        self.orange_hsv_low = np.array(
            self.get_parameter("orange_hsv_low").get_parameter_value().double_array_value,
            dtype=np.uint8,
        )
        self.orange_hsv_high = np.array(
            self.get_parameter("orange_hsv_high").get_parameter_value().double_array_value,
            dtype=np.uint8,
        )
        self.orange_morph_kernel = (
            self.get_parameter("orange_morph_kernel").get_parameter_value().integer_value
        )

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

        self.sorted_stockpile_ids = tuple(sorted(STOCKPILE_TAG_IDS))

        self.broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(MarkerArray, "workspace_markers", 1)
        self.koz_pub = self.create_publisher(OccupancyGrid, "koz_mask", 1)
        self.block_pub = self.create_publisher(OccupancyGrid, "block_mask", 1)
        self.free_map_pub = self.create_publisher(OccupancyGrid, "free_map", 1)
        self.stockpile_pub = self.create_publisher(Stockpiles, "stockpile_polygons", 1)
        self.build_site_pub = self.create_publisher(PolygonStamped, "build_site_polygon", 1)
        self.timer = self.create_timer(1.0 / tick_rate_hz, self.tick)

        self.last_R_cw: np.ndarray | None = None
        self.last_t_w: np.ndarray | None = None
        self.stockpile_ema: dict[int, tuple[np.ndarray, float]] = {}

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

    def _new_grid_skeleton(self) -> tuple[OccupancyGrid, int, int]:
        """Build an OccupancyGrid with metadata set; caller fills `.data`."""
        wl, ww = self.outer_dims
        res = self.koz_resolution
        width = max(1, int(np.ceil(wl / res)))
        height = max(1, int(np.ceil(ww / res)))

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.world_frame
        grid.info.resolution = float(res)
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.orientation.w = 1.0
        return grid, width, height

    def detect_orange_mask(self, frame_bgr: np.ndarray) -> np.ndarray:
        """Return a uint8 mask (0/255) of orange pixels after HSV threshold + morph open/close."""
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.orange_hsv_low, self.orange_hsv_high)
        k = self.orange_morph_kernel
        if k > 0:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def build_koz_grid(self, rects_world: list[np.ndarray], extra_mask: np.ndarray | None = None) -> OccupancyGrid:
        """Rasterize rotated rectangles (each Nx2 in world coords) and OR an optional
        grid-resolution mask into an OccupancyGrid."""
        grid, width, height = self._new_grid_skeleton()
        res = self.koz_resolution
        mask = np.zeros((height, width), dtype=np.uint8)
        for rect in rects_world:
            pix = np.round(rect / res).astype(np.int32)
            cv2.fillConvexPoly(mask, pix, 100)
        if extra_mask is not None:
            mask = np.where((mask > 0) | (extra_mask > 0), 100, 0).astype(np.uint8)
        grid.data = mask.flatten().astype(np.int8).tolist()
        return grid

    def build_free_grid(self) -> OccupancyGrid:
        """Same metadata as the KOZ grid, but no cells occupied."""
        grid, width, height = self._new_grid_skeleton()
        grid.data = np.zeros(width * height, dtype=np.int8).tolist()
        return grid

    def build_block_grid(self, orange_grid_mask: np.ndarray) -> OccupancyGrid:
        """Wrap a grid-resolution orange mask (0/255) into an OccupancyGrid (0/100)."""
        grid, _, _ = self._new_grid_skeleton()
        occ = np.where(orange_grid_mask > 0, 100, 0).astype(np.int8)
        grid.data = occ.flatten().tolist()
        return grid

    def publish_stockpiles(self, stockpile_rects: dict[int, np.ndarray]) -> None:
        """
        Publish stockpiles with parallel ids and polygons arrays.

        Emit polygons for stockpiles detected this tick, in sorted tag-id order.
        ids[i] is the aruco tag id whose footprint is polygons[i].
        """
        if not stockpile_rects:
            return
        msg = Stockpiles()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame
        for tid in self.sorted_stockpile_ids:
            rect = stockpile_rects.get(tid)
            if rect is None:
                continue
            poly = Polygon()
            poly.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in rect]
            msg.ids.append(tid)
            msg.polygons.append(poly)
        self.stockpile_pub.publish(msg)

    def tick(self) -> None:
        """Read a frame, solve PnP for both frames, and broadcast available transforms."""
        ret, frame = self.cap.read()
        if not ret:
            return

        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is None:
            return
        ids_flat = ids.flatten()

        if sum(1 for tid in OUTER_TAG_IDS if tid in ids_flat) < 3:
            return

        outer_ok, rvec_w, tvec_w = self.solve_frame(self.outer_corners, ids_flat, corners)
        inner_ok, rvec_i, tvec_i = self.solve_frame(self.inner_corners, ids_flat, corners)

        transforms = []
        rects: list[np.ndarray] = []
        stockpile_ids: list[int] = []
        stockpile_rects: dict[int, np.ndarray] = {}

        if outer_ok:
            R_wc = cv2.Rodrigues(rvec_w)[0]
            R_cw = R_wc.T
            t_w = tvec_w.flatten()
            self.last_R_cw = R_cw
            self.last_t_w = t_w
        elif self.last_R_cw is not None and self.last_t_w is not None:
            R_cw = self.last_R_cw
            t_w = self.last_t_w
        else:
            return

        # Undistort once for both detection and projection so the pinhole model is exact.
        undistorted = cv2.undistort(frame, self.mtx, self.dist)
        orange_mask = self.detect_orange_mask(undistorted)
        _, grid_w, grid_h = self._new_grid_skeleton()
        H = image_to_grid_homography(R_cw, t_w, self.mtx, self.koz_resolution)
        orange_grid_mask = cv2.warpPerspective(orange_mask, H, (grid_w, grid_h))

        # world -> camera: invert the outer PnP.
        transforms.append(self.make_transform(self.world_frame, self.camera_frame, R_cw, -R_cw @ t_w))

        # world -> inner: compose outer^-1 with inner, then flatten to (x, y, yaw).
        if inner_ok:
            R_wi = yaw_only(R_cw @ cv2.Rodrigues(rvec_i)[0])
            t_wi = R_cw @ (tvec_i.flatten() - t_w)
            t_wi[2] = 0.0
            transforms.append(self.make_transform(self.world_frame, self.inner_frame, R_wi, t_wi))
            inner_rect = rect_in_world(R_wi[:2, :2], t_wi[:2], *self.inner_dims)
            rects.append(inner_rect)

            poly_msg = PolygonStamped()
            poly_msg.header.stamp = self.get_clock().now().to_msg()
            poly_msg.header.frame_id = self.world_frame
            poly_msg.polygon.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in inner_rect]
            self.build_site_pub.publish(poly_msg)

        # Per-tag PnP for everything that isn't a frame-defining outer/inner corner.
        sl, sw = self.stockpile_dims
        for i, tag_id in enumerate(ids_flat):
            tid = int(tag_id)
            if tid in OUTER_TAG_IDS or tid in INNER_TAG_IDS:
                continue
            ok, rvec_t, tvec_t = cv2.solvePnP(
                self.tag_local,
                corners[i],
                self.mtx,
                self.dist,
                False,
                cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not ok:
                continue
            R_wt = yaw_only(R_cw @ cv2.Rodrigues(rvec_t)[0])
            t_wt = R_cw @ (tvec_t.flatten() - t_w)
            t_wt[2] = 0.0
            if tid in STOCKPILE_TAG_IDS:
                # Shift origin from tag center to rectangle bottom-left, then smooth.
                t_wt = t_wt - R_wt @ np.array([sl / 2.0, sw / 2.0, 0.0])
                yaw_measured = float(np.arctan2(R_wt[1, 0], R_wt[0, 0]))
                pos_measured = t_wt[:2].copy()
                smoothed_pos, smoothed_yaw = ema_step(
                    self.stockpile_ema.get(tid),
                    pos_measured,
                    yaw_measured,
                    self.stockpile_ema_alpha,
                )
                self.stockpile_ema[tid] = (smoothed_pos, smoothed_yaw)
                c, s = np.cos(smoothed_yaw), np.sin(smoothed_yaw)
                R_wt = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
                t_wt = np.array([smoothed_pos[0], smoothed_pos[1], 0.0])

                transforms.append(self.make_transform(self.world_frame, f"stockpile_{tid}", R_wt, t_wt))
                stockpile_rect = rect_in_world(R_wt[:2, :2], t_wt[:2], sl, sw)
                rects.append(stockpile_rect)
                stockpile_ids.append(tid)
                stockpile_rects[tid] = stockpile_rect
            else:
                transforms.append(self.make_transform(self.world_frame, f"aruco_{tid}", R_wt, t_wt))

        self.broadcaster.sendTransform(transforms)
        self.koz_pub.publish(self.build_koz_grid(rects, extra_mask=orange_grid_mask))
        self.block_pub.publish(self.build_block_grid(orange_grid_mask))
        self.free_map_pub.publish(self.build_free_grid())
        self.publish_stockpiles(stockpile_rects)

        markers = MarkerArray()
        markers.markers.append(self.make_rect_marker(self.world_frame, 0, *self.outer_dims, (0.1, 0.9, 0.1)))
        if inner_ok:
            markers.markers.append(self.make_rect_marker(self.inner_frame, 1, *self.inner_dims, (0.1, 0.5, 1.0)))
        for idx, tid in enumerate(stockpile_ids):
            markers.markers.append(self.make_rect_marker(f"stockpile_{tid}", 100 + idx, sl, sw, (1.0, 0.3, 0.1)))
        self.marker_pub.publish(markers)


def main() -> None:
    """Spin the CcLocalizationNode until interrupted."""
    rclpy.init()
    node = CcLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
