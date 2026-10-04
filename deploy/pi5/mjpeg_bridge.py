#!/usr/bin/env python3
"""Subscribe to a ROS 2 Image topic and serve it as an MJPEG HTTP stream.

Live view for the robot arm camera without web_video_server. Runs in the
robot_arm_camera container (host net); open http://<nano-ip>:8080/ in a browser.
"""
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn


class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

PORT = 8080
TOPIC = "/camera/pose_overlay"

_lock = threading.Lock()
_latest_jpeg = None


class CamSub(Node):
    def __init__(self):
        super().__init__("mjpeg_bridge")
        self.bridge = CvBridge()
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        self.create_subscription(Image, TOPIC, self.cb, qos)
        self.get_logger().info(f"mjpeg_bridge subscribing {TOPIC}, serving :{PORT}")

    def cb(self, msg):
        global _latest_jpeg
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with _lock:
                _latest_jpeg = buf.tobytes()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            html = (b"<html><body style='margin:0;background:#111'>"
                    b"<img src='/stream' style='width:640px;max-width:100%;height:auto;display:block;margin:8px auto;image-rendering:auto'/>"
                    b"</body></html>")
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(html)))
            self.end_headers()
            self.wfile.write(html)
            return
        if self.path != "/stream":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header(
            "Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with _lock:
                    jpg = _latest_jpeg
                if jpg is None:
                    continue
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(
                    ("Content-Length: %d\r\n\r\n" % len(jpg)).encode())
                self.wfile.write(jpg)
                self.wfile.write(b"\r\n")
                import time
                time.sleep(1 / 30.0)
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    rclpy.init()
    node = CamSub()
    srv = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        srv.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
