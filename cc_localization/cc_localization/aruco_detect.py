# ruff: noqa: N806
from pathlib import Path

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory

DEVICE_ID = 42
MARKER_SIZE = 0.15  # meters — printed marker side length

# Outer frame (tags 0-3)
WORLD_LENGTH = 1.35  # meters — distance between tags along world +X (tag 0 -> tag 1)
WORLD_WIDTH = 0.885  # meters — distance between tags along world +Y (tag 0 -> tag 3)

# Inner frame (tags 4-7), same convention: 4=origin, 5=+X, 6=+X+Y, 7=+Y
INNER_LENGTH = 0.41  # meters — 4 -> 5
INNER_WIDTH = 0.54  # meters — 4 -> 7

OUTER_TAG_CORNERS = {
    0: (0.0, 0.0),
    1: (WORLD_LENGTH, 0.0),
    2: (WORLD_LENGTH, WORLD_WIDTH),
    3: (0.0, WORLD_WIDTH),
}

INNER_TAG_CORNERS = {
    4: (0.0, 0.0),
    5: (INNER_LENGTH, 0.0),
    6: (INNER_LENGTH, INNER_WIDTH),
    7: (0.0, INNER_WIDTH),
}

FRAME_TAG_IDS = set(OUTER_TAG_CORNERS) | set(INNER_TAG_CORNERS)

CALIB_PATH = Path(get_package_share_directory("cc_localization")) / "config" / "gopro_calib.npz"
calib = np.load(CALIB_PATH)
mtx = calib["camera_matrix"]
dist = calib["dist_coeffs"]


TAG_LOCAL = np.array(
    [
        [-MARKER_SIZE / 2, MARKER_SIZE / 2, 0],
        [MARKER_SIZE / 2, MARKER_SIZE / 2, 0],
        [MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
        [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
    ],
    dtype=np.float32,
)


def solve_frame(tag_corners, ids_flat, corners, mtx, dist):
    """
    Solve PnP for a frame defined by a dict of {tag_id: (x, y)} corner positions.

    Returns (ok, rvec, tvec) where rvec/tvec map points from frame coords to camera coords.
    """
    obj_list, img_list = [], []
    for tag_id, (cx, cy) in tag_corners.items():
        if tag_id not in ids_flat:
            continue
        idx = int(np.where(ids_flat == tag_id)[0][0])
        obj_list.append(TAG_LOCAL + np.array([cx, cy, 0.0], dtype=np.float32))
        img_list.append(corners[idx].reshape(-1, 2))
    if not obj_list:
        return False, None, None, 0
    obj_pts = np.concatenate(obj_list, axis=0)
    img_pts = np.concatenate(img_list, axis=0).astype(np.float32)
    ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, mtx, dist)
    return ok, rvec, tvec, len(obj_list)


def main():
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))

    cap = cv2.VideoCapture(DEVICE_ID)
    cv2.namedWindow("ArUco")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        corners, ids, _ = detector.detectMarkers(frame)
        ids_flat = ids.flatten() if ids is not None else np.array([], dtype=int)
        outer_axis_len = max(WORLD_LENGTH, WORLD_WIDTH) * 0.25
        inner_axis_len = max(INNER_LENGTH, INNER_WIDTH) * 0.25

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        outer_ok, rvec_w, tvec_w, n_outer = solve_frame(OUTER_TAG_CORNERS, ids_flat, corners, mtx, dist)
        inner_ok, rvec_i, tvec_i, n_inner = solve_frame(INNER_TAG_CORNERS, ids_flat, corners, mtx, dist)

        if outer_ok:
            cv2.drawFrameAxes(frame, mtx, dist, rvec_w, tvec_w, outer_axis_len, 3)
        if inner_ok:
            cv2.drawFrameAxes(frame, mtx, dist, rvec_i, tvec_i, inner_axis_len, 3)

        R_wc = cv2.Rodrigues(rvec_w)[0] if outer_ok else np.eye(3)
        R_ic = cv2.Rodrigues(rvec_i)[0] if inner_ok else np.eye(3)

        # Inner frame pose in outer (world) coords — useful for tf publishing later
        if outer_ok and inner_ok:
            inner_origin_world = R_wc.T @ (tvec_i.flatten() - tvec_w.flatten())
            cv2.putText(
                frame,
                f"inner@world: ({inner_origin_world[0]:.2f}, {inner_origin_world[1]:.2f}, {inner_origin_world[2]:.2f})",
                (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 200, 0),
                2,
            )

        # Localize unknown tags in whichever frames are available
        if outer_ok or inner_ok:
            for i, tag_id in enumerate(ids_flat):
                if int(tag_id) in FRAME_TAG_IDS:
                    continue
                ok, _, tvec_t = cv2.solvePnP(TAG_LOCAL, corners[i], mtx, dist, False, cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok:
                    continue
                c = corners[i].reshape(-1, 2).mean(axis=0).astype(int)
                line = 0
                if outer_ok:
                    p_w = R_wc.T @ (tvec_t.flatten() - tvec_w.flatten())
                    cv2.putText(
                        frame,
                        f"id {int(tag_id)} W: ({p_w[0]:.2f}, {p_w[1]:.2f})",
                        (c[0] + 10, c[1] + line),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 0),
                        2,
                    )
                    line += 20
                if inner_ok:
                    p_i = R_ic.T @ (tvec_t.flatten() - tvec_i.flatten())
                    cv2.putText(
                        frame,
                        f"id {int(tag_id)} I: ({p_i[0]:.2f}, {p_i[1]:.2f})",
                        (c[0] + 10, c[1] + line),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 200, 255),
                        2,
                    )

        cv2.putText(
            frame,
            f"outer: {n_outer}/{len(OUTER_TAG_CORNERS)}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0) if outer_ok else (0, 0, 255),
            2,
        )
        cv2.putText(
            frame,
            f"inner: {n_inner}/{len(INNER_TAG_CORNERS)}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0) if inner_ok else (0, 0, 255),
            2,
        )

        cv2.imshow("ArUco", frame)
        if cv2.waitKey(30) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
