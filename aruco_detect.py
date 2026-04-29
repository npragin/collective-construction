import cv2
import numpy as np

DEVICE_ID = 42
MARKER_SIZE = 0.15  # meters — printed marker side length
WORLD_LENGTH = 1.23  # meters — distance between tags along world +X (tag 0 -> tag 1)
WORLD_WIDTH = 1.52  # meters — distance between tags along world +Y (tag 0 -> tag 3)

# Tag IDs at the four world corners (centers), all on z=0 plane
TAG_CORNERS = {
    0: (0.0, 0.0),  # origin
    1: (WORLD_LENGTH, 0.0),  # +X
    2: (WORLD_LENGTH, WORLD_WIDTH),  # +X +Y
    3: (0.0, WORLD_WIDTH),  # +Y
}

calib = np.load("gopro_calib.npz")
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


detector = cv2.aruco.ArucoDetector(
    cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
)

cap = cv2.VideoCapture(DEVICE_ID)
cv2.namedWindow("ArUco")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    corners, ids, _ = detector.detectMarkers(frame)
    ids_flat = ids.flatten() if ids is not None else np.array([], dtype=int)
    axis_len = max(WORLD_LENGTH, WORLD_WIDTH) * 0.25

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    obj_list, img_list = [], []
    for tag_id, (cx, cy) in TAG_CORNERS.items():
        if tag_id not in ids_flat:
            continue
        idx = int(np.where(ids_flat == tag_id)[0][0])
        obj_list.append(TAG_LOCAL + np.array([cx, cy, 0.0], dtype=np.float32))
        img_list.append(corners[idx].reshape(-1, 2))

    world_ok = False
    if obj_list:
        obj_pts = np.concatenate(obj_list, axis=0)
        img_pts = np.concatenate(img_list, axis=0).astype(np.float32)
        world_ok, rvec_w, tvec_w = cv2.solvePnP(obj_pts, img_pts, mtx, dist)
        if world_ok:
            cv2.drawFrameAxes(frame, mtx, dist, rvec_w, tvec_w, axis_len, 3)

    if world_ok:
        R_wc, _ = cv2.Rodrigues(rvec_w)
        for i, tag_id in enumerate(ids_flat):
            if int(tag_id) in TAG_CORNERS:
                continue
            ok, rvec_t, tvec_t = cv2.solvePnP(
                TAG_LOCAL, corners[i], mtx, dist, False, cv2.SOLVEPNP_IPPE_SQUARE
            )
            if not ok:
                continue
            # tag center is the origin of tag-local frame; in camera frame that's tvec_t
            p_world = R_wc.T @ (tvec_t.flatten() - tvec_w.flatten())
            c = corners[i].reshape(-1, 2).mean(axis=0).astype(int)
            cv2.putText(
                frame,
                f"id {int(tag_id)}: ({p_world[0]:.2f}, {p_world[1]:.2f})",
                (c[0] + 10, c[1]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 0),
                2,
            )

    cv2.putText(
        frame,
        f"tags: {len(obj_list)}/{len(TAG_CORNERS)}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0) if obj_list else (0, 0, 255),
        2,
    )

    cv2.imshow("ArUco", frame)
    if cv2.waitKey(30) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
