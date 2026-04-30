import cv2
import numpy as np

CHECKERBOARD = (10, 7)  # internal corners for 11x8 board
CAPTURE_EVERY_N = 5  # only keep every Nth detected frame
DEVICE_ID = 42

objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0 : CHECKERBOARD[0], 0 : CHECKERBOARD[1]].T.reshape(-1, 2)

obj_points = []
img_points = []
frame_count = 0

cap = cv2.VideoCapture(DEVICE_ID)
cv2.namedWindow("Calibration")

print("Move the board around. Q to quit and calibrate.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    display = frame.copy()
    if found:
        frame_count += 1
        cv2.drawChessboardCorners(display, CHECKERBOARD, corners, found)
        if frame_count % CAPTURE_EVERY_N == 0:
            corners2 = cv2.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
            )
            obj_points.append(objp)
            img_points.append(corners2)
            print(f"Captured frame {len(obj_points)}")

    cv2.putText(
        display,
        f"Captured: {len(obj_points)}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0) if found else (0, 0, 255),
        2,
    )
    cv2.imshow("Calibration", display)

    if cv2.waitKey(30) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()

if len(obj_points) < 5:
    print("Not enough frames to calibrate")
else:
    print("Calibrating...")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)
    np.savez("gopro_calib.npz", camera_matrix=mtx, dist_coeffs=dist)
    print(f"RMS reprojection error: {ret:.4f}")
    print("Saved to gopro_calib.npz")
    print(f"Camera matrix:\n{mtx}")
    print(f"Distortion coefficients:\n{dist}")
