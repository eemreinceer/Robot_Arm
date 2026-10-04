#!/usr/bin/env python3
"""Live browser view of what the board detector actually sees.

Aiming the board from a raw camera stream is guesswork: you cannot tell whether
the 48 corners were found, and you certainly cannot see the thing that matters
for the flip sweep -- WHICH corner the detector calls number zero. When the
native ordering flips, corner[0] jumps to the far end of the board. This draws
it, so the operator can see the flip happen instead of discovering it later in
a log.

Runs on the PC (detection at 640x480 was measured at 717 ms on the Nano versus
~15 Hz on the PC). Serves MJPEG over HTTP so it can be watched from a phone at
the bench, not only from the machine running it.

    python3 scripts/board_overlay.py            # then open http://localhost:8090/

Read-only: subscribes to the camera, commands nothing, and does not touch the
arm or the servo rail.
"""
import argparse
import math
import sys
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from arm_perception import board_pnp
from arm_perception.board_pnp import (
    DEFAULT_BOARD_COLS,
    DEFAULT_BOARD_ROWS,
    DEFAULT_SQUARE_SIZE_MM,
)

FLIP_MARGIN_PX = 0.05  # same threshold board_pnp.py decides with

_lock = threading.Lock()
_frame_ready = threading.Condition(_lock)
_jpeg = None


def index_html(width_px):
    return ("<!doctype html><meta charset='utf-8'>"
            "<title>board overlay</title>"
            "<style>body{background:#111;color:#ddd;font-family:sans-serif;"
            "margin:0;padding:12px}img{max-width:100%%;width:%dpx;"
            "image-rendering:pixelated}</style>"
            "<h3>board detector overlay</h3>"
            "<img src='/stream'>"
            "<p>Yellow ring = corner[0]. If it jumps to the other end of the "
            "board, the ordering flipped.</p>" % width_px).encode()


class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


class BoardOverlay(Node):
    def __init__(self, args):
        super().__init__("board_overlay")
        self.args = args
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        # Ordering stability is the real question, and it is a property of
        # consecutive frames, not of a single one.
        self.last_corner0 = None
        self.flip_count = 0
        self.frames_since_flip = 0

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        self.create_subscription(Image, "/camera/image_raw", self._on_image, qos)
        self.create_subscription(CameraInfo, "/camera/camera_info",
                                 self._on_info, qos)

    def _on_info(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
            self.get_logger().info("intrinsics received")

    def _on_image(self, msg):
        global _jpeg
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners = board_pnp.detect_corners(gray, self.args.cols, self.args.rows)
        if corners is None:
            self._banner(frame, "BOARD NOT DETECTED", (40, 40, 200))
            cv2.putText(frame, "whole board must be in frame, not too oblique",
                        (10, frame.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (200, 200, 200), 1, cv2.LINE_AA)
        else:
            self._draw_detection(frame, gray, corners)

        ok, buffer = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.args.quality])
        if not ok:
            return
        with _frame_ready:
            _jpeg = buffer.tobytes()
            _frame_ready.notify_all()

    def _banner(self, frame, text, colour):
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 26), colour, -1)
        cv2.putText(frame, text, (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 2, cv2.LINE_AA)

    def _draw_detection(self, frame, gray, corners):
        cv2.drawChessboardCorners(
            frame, (self.args.cols, self.args.rows), corners, True)

        corner0 = corners[0].reshape(2)
        corner_last = corners[-1].reshape(2)
        # corner[0] is the whole point of this view: it is the observable that
        # moves discontinuously when the ordering flips.
        cv2.circle(frame, (int(corner0[0]), int(corner0[1])), 14, (0, 255, 255), 3)
        cv2.putText(frame, "0", (int(corner0[0]) + 16, int(corner0[1]) + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.circle(frame, (int(corner_last[0]), int(corner_last[1])), 8,
                   (255, 160, 0), 2)

        vector = corner_last - corner0
        in_plane = math.degrees(math.atan2(float(vector[1]), float(vector[0])))

        if self.camera_matrix is None:
            self._banner(frame, "waiting for camera_info (no intrinsics)",
                         (0, 120, 200))
            return

        objp = board_pnp.board_object_points(
            self.args.cols, self.args.rows, self.args.square_mm)
        reproj = {}
        pose = None
        for name, pts in (("native", corners), ("reversed", corners[::-1].copy())):
            ok, rvec, tvec = board_pnp.solve_pose(
                objp, pts, self.camera_matrix, self.dist_coeffs)
            if not ok:
                self._banner(frame, "PnP failed", (0, 0, 200))
                return
            reproj[name] = board_pnp.reprojection_error_px(
                objp, pts, rvec, tvec, self.camera_matrix, self.dist_coeffs)
            if name == "native":
                pose = (rvec, tvec)

        margin = reproj["reversed"] - reproj["native"]
        ambiguous = abs(margin) < FLIP_MARGIN_PX
        rotation, _ = cv2.Rodrigues(pose[0])
        normal = rotation @ np.array([0.0, 0.0, 1.0])
        tilt = math.degrees(math.acos(min(1.0, abs(float(normal[2])))))
        distance = float(np.linalg.norm(pose[1]))

        # MEASURED 2026-07-29: the reprojection margin is ~0 for EVERY
        # rectangular chessboard, whatever its parity, because a rectangular
        # grid of object points is 180-degree symmetric as a geometric object --
        # both orderings fit equally well. Banner-ing on `ambiguous` therefore
        # showed ORANGE even on a correct 6x9 board and misled the operator.
        #
        # What parity actually buys is DETECTOR stability: on a 6x9 board a
        # 180-degree turn inverts the square colours, so findChessboardCorners
        # can tell the two apart and keeps corner[0] put. So the honest live
        # indicator is whether corner[0] stays put across frames.
        jumped_now = False
        if self.last_corner0 is not None:
            if float(np.linalg.norm(corner0 - self.last_corner0)) > 60.0:
                self.flip_count += 1
                self.frames_since_flip = 0
                jumped_now = True
        self.last_corner0 = corner0
        if not jumped_now:
            self.frames_since_flip += 1

        if jumped_now:
            self._banner(frame, "ORDERING JUST FLIPPED", (0, 0, 220))
        elif self.flip_count == 0:
            self._banner(frame, "DETECTED  -  ordering stable (0 flips)", (40, 140, 40))
        else:
            self._banner(frame, "DETECTED  -  %d flip(s) so far" % self.flip_count,
                         (0, 140, 220))

        lines = [
            "corners %d   dist %.2f m   tilt %.1f deg" % (
                corners.shape[0], distance, tilt),
            "in-plane %+.1f deg   stable for %d frames" % (
                in_plane, self.frames_since_flip),
            "reproj native %.3f  reversed %.3f" % (
                reproj["native"], reproj["reversed"]),
            "margin %+.4f px  (~0 is EXPECTED; it cannot resolve orientation)"
            % margin,
        ]
        for index, text in enumerate(lines):
            y = 48 + index * 20
            cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (230, 230, 230), 1, cv2.LINE_AA)


def make_handler(fps, width_px):
    interval = 1.0 / max(1.0, fps)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path != "/stream":
                html = index_html(width_px)
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(html)))
                self.end_headers()
                self.wfile.write(html)
                return
            self.send_response(200)
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with _frame_ready:
                        # Wait for a real frame instead of spinning on None,
                        # which would burn a core before the first image.
                        while _jpeg is None:
                            _frame_ready.wait(timeout=1.0)
                        payload = _jpeg
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: " +
                                     str(len(payload)).encode() + b"\r\n\r\n")
                    self.wfile.write(payload)
                    self.wfile.write(b"\r\n")
                    threading.Event().wait(interval)
            except (BrokenPipeError, ConnectionResetError):
                return

    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--cols", type=int, default=DEFAULT_BOARD_COLS)
    parser.add_argument("--rows", type=int, default=DEFAULT_BOARD_ROWS)
    parser.add_argument("--square-mm", type=float, default=DEFAULT_SQUARE_SIZE_MM)
    parser.add_argument("--quality", type=int, default=80)
    parser.add_argument("--fps", type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = BoardOverlay(args)
    server = ThreadingHTTPServer(("0.0.0.0", args.port),
                                 make_handler(args.fps, 640))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print("overlay on http://localhost:%d/   (and http://<this-host>:%d/)"
          % (args.port, args.port))
    print("yellow ring = corner[0]; watch it jump if the ordering flips")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print()
    except rclpy.executors.ExternalShutdownException:
        print("\n(interrupted externally)")
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
