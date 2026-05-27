"""Temporary map-frame visualization for block_localization.

Subscribes to {ns}/found_blocks (latched), {ns}/pose, and the localizer's
debug image. Renders a 600x600 top-down canvas at 100 px/m with a green
triangle for the robot and yellow circles labelled with tag_id for each
block. Serves two MJPEG streams (/world.mjpg, /camera.mjpg) on :8556 via
the Python stdlib http.server.

Remove this file, its setup.py entry-point, and block_viz_launch.py to
unship.
"""

import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CompressedImage

from cc_interfaces.msg import FoundBlockArray


CANVAS = 600
PX_PER_M = 100

INDEX_HTML = (
    b"<!doctype html><html><body style='margin:0;background:#000;color:#ccc;"
    b"font-family:sans-serif'>"
    b"<div style='display:flex;gap:8px;padding:8px;align-items:flex-start'>"
    b"<div><div style='padding:4px'>map</div>"
    b"<img src='/world.mjpg' style='display:block'></div>"
    b"<div><div style='padding:4px'>camera</div>"
    b"<img src='/camera.mjpg' style='display:block;max-width:960px'></div>"
    b"</div></body></html>"
)


def yaw_from_q(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class BlockViz(Node):
    def __init__(self):
        super().__init__("block_viz")
        self.declare_parameter("port", 8556)
        self.port = int(self.get_parameter("port").value)

        ns = self.get_namespace().rstrip("/")
        self.ns = ns or "/"
        self.pose = None
        self.blocks = []
        self.camera_jpeg = None

        latched = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Pose, f"{ns}/pose", self._on_pose, 10)
        self.create_subscription(FoundBlockArray, f"{ns}/found_blocks",
                                 self._on_blocks, latched)
        self.create_subscription(
            CompressedImage,
            f"{ns}/block_localization/debug_image/compressed",
            self._on_camera, qos_profile_sensor_data)
        self.get_logger().info(f"block_viz up, ns={self.ns}, port={self.port}")

    def _on_pose(self, msg):   self.pose = msg
    def _on_blocks(self, msg): self.blocks = list(msg.blocks)
    def _on_camera(self, msg): self.camera_jpeg = bytes(msg.data)

    def _w2p(self, x, y):
        return (int(CANVAS / 2 + x * PX_PER_M),
                int(CANVAS / 2 - y * PX_PER_M))

    def render(self):
        img = np.full((CANVAS, CANVAS, 3), 30, np.uint8)

        step = int(0.5 * PX_PER_M)
        for g in range(0, CANVAS, step):
            cv2.line(img, (g, 0), (g, CANVAS), (50, 50, 50), 1)
            cv2.line(img, (0, g), (CANVAS, g), (50, 50, 50), 1)

        if self.pose is not None:
            yaw = yaw_from_q(self.pose.orientation)
            cx, cy = self._w2p(self.pose.position.x, self.pose.position.y)
            tip = (cx + int(20 * math.cos(yaw)),
                   cy - int(20 * math.sin(yaw)))
            l = (cx + int(12 * math.cos(yaw + 2.5)),
                 cy - int(12 * math.sin(yaw + 2.5)))
            r = (cx + int(12 * math.cos(yaw - 2.5)),
                 cy - int(12 * math.sin(yaw - 2.5)))
            cv2.fillPoly(img, [np.array([tip, l, r])], (0, 220, 0))

        for b in self.blocks:
            x, y = self._w2p(b.pose.position.x, b.pose.position.y)
            cv2.circle(img, (x, y), 6, (0, 255, 255), -1)
            cv2.putText(img, str(b.tag_id), (x + 8, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1,
                        cv2.LINE_AA)

        header = f"{self.ns}  blocks={len(self.blocks)}"
        cv2.putText(img, header, (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1,
                    cv2.LINE_AA)

        ok, jpg = cv2.imencode(".jpg", img,
                               [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        return jpg.tobytes() if ok else None


_node = None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # silence per-request logging

    def _stream(self, source):
        self.send_response(200)
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                f = source()
                if f is None:
                    time.sleep(0.05)
                    continue
                self.wfile.write(
                    b"--frame\r\nContent-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(f)).encode() +
                    b"\r\n\r\n" + f + b"\r\n"
                )
                time.sleep(0.1)
        except (BrokenPipeError, ConnectionResetError):
            return

    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(INDEX_HTML)))
            self.end_headers()
            self.wfile.write(INDEX_HTML)
            return
        if self.path == "/world.mjpg":
            return self._stream(_node.render)
        if self.path == "/camera.mjpg":
            return self._stream(lambda: _node.camera_jpeg)
        self.send_error(404)


def main(args=None):
    global _node
    rclpy.init(args=args)
    _node = BlockViz()
    threading.Thread(target=lambda: rclpy.spin(_node), daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", _node.port), Handler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        _node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
