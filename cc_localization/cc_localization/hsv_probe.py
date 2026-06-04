"""Standalone OpenCV viewer that shows the HSV value at the mouse cursor.

Run with: ros2 run cc_localization hsv_probe [--device-id N]

Use this to tune the orange HSV thresholds consumed by cc_localization_node.
Do not run it at the same time as cc_localization_node — they share the
camera device.
"""

import argparse
import sys

import cv2
import numpy as np

WINDOW = "hsv_probe"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device-id", type=int, default=42, help="cv2.VideoCapture device id")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.device_id)
    if not cap.isOpened():
        print(f"Failed to open video device {args.device_id}", file=sys.stderr)
        sys.exit(1)

    cursor: dict[str, int | None] = {"x": None, "y": None}

    def on_mouse(event: int, x: int, y: int, flags: int, param: object) -> None:
        if event == cv2.EVENT_MOUSEMOVE:
            cursor["x"] = x
            cursor["y"] = y

    cv2.namedWindow(WINDOW)
    cv2.setMouseCallback(WINDOW, on_mouse)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            h_img, w_img = hsv.shape[:2]

            if cursor["x"] is None or cursor["y"] is None:
                label = "(no cursor)"
            else:
                x = int(np.clip(cursor["x"], 0, w_img - 1))
                y = int(np.clip(cursor["y"], 0, h_img - 1))
                h, s, v = (int(c) for c in hsv[y, x])
                label = f"H:{h} S:{s} V:{v}  (x,y)=({x},{y})"

            cv2.rectangle(frame, (5, 5), (5 + 360, 5 + 30), (0, 0, 0), thickness=-1)
            cv2.putText(
                frame,
                label,
                (10, 27),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

            cv2.imshow(WINDOW, frame)
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
