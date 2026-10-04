#!/usr/bin/env python3
"""ROS 2 Image topic'ini MJPEG HTTP akışı olarak sunar.

Robot Arm kol kamerasını `web_video_server` olmadan canlı izlemek için. `robot_arm_camera`
konteynerinde (host network) koşar; tarayıcıdan `http://<nano-ip>:8080/`.

Neden var: kamera Jetson'da, operatör PC'de. RViz için tam DDS kurulumu
gerekirken bu köprü herhangi bir tarayıcıdan çalışır — tahtayı kadraja
yerleştirirken veya kolun ne gördüğünü kontrol ederken en hızlı yol budur.

KÖKEN: 2026-07-28'e kadar Jetson runtime workspace'inde yerel bir script
olarak gevşek dosya halinde duruyordu, repoda yoktu — Jetson'a bir şey olsa
kaybolacaktı. Pakete alınırken iki düzeltme yapıldı:
  1. `<img style='width:100%'>` görüntüyü pencere genişliğine geriyordu;
     640x480 akış 1080p ekranda 3x büyütülüp bulanıklaşıyordu. Artık doğal
     boyutta (küçük ekranda `max-width` ile küçülür).
  2. İlk kare gelmeden önceki `while True: ... continue` döngüsü boşta
     %100 CPU yakıyordu. Nano'da kontrol döngüsü de aynı çekirdekleri
     paylaştığı için bu gerçek bir maliyetti; artık olaya bekliyor.

UYUMLULUK: bu dosya Jetson Nano'nun Python 3.6 ortamında doğrudan çalışır.
`dataclasses`, postponed annotations veya Python 3.7+ sözdizimi eklenmemelidir.
"""
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

DEFAULT_PORT = 8080
DEFAULT_TOPIC = "/camera/image_raw"
DEFAULT_QUALITY = 80
DEFAULT_COMPRESSED_TOPIC = "/camera/image_raw/compressed"

_lock = threading.Lock()
_frame_ready = threading.Condition(_lock)
_latest_jpeg = None


def index_html(width_px=640):
    """Tarayıcı sayfası. Görüntü DOĞAL boyutta, gerdirilmez.

    `width:100%` kullanılırsa 640x480 akış pencere genişliğine büyütülür ve
    bulanıklaşır — bu, akışın kendisi büyükmüş gibi görünmesine yol açar.
    """
    return (
        "<html><body style='margin:0;background:#111'>"
        f"<img src='/stream' style='width:{width_px}px;max-width:100%;"
        "height:auto;display:block;margin:8px auto'/>"
        "</body></html>"
    ).encode()


class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


class MjpegBridge(Node):
    def __init__(self):
        super().__init__("mjpeg_bridge")
        self.declare_parameter("port", DEFAULT_PORT)
        self.declare_parameter("image_topic", DEFAULT_TOPIC)
        self.declare_parameter("jpeg_quality", DEFAULT_QUALITY)
        self.declare_parameter("stream_fps", 30.0)
        self.declare_parameter("compressed_topic", DEFAULT_COMPRESSED_TOPIC)

        self.bridge = CvBridge()
        self.quality = int(self.get_parameter("jpeg_quality").value)
        topic = str(self.get_parameter("image_topic").value)
        compressed_topic = str(self.get_parameter("compressed_topic").value)

        # Kamera BEST_EFFORT yayınlıyor; RELIABLE abone eşleşmez ve sessizce
        # hiç kare almaz.
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        self.compressed_publisher = self.create_publisher(
            CompressedImage, compressed_topic, qos)
        self.create_subscription(Image, topic, self._on_image, qos)
        self.get_logger().info(
            f"mjpeg_bridge {topic} dinliyor, {compressed_topic} yayınlıyor, "
            f":{self.get_parameter('port').value} portunda sunuyor")

    def _on_image(self, msg):
        global _latest_jpeg
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        ok, buf = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            return
        jpeg = buf.tobytes()
        # HTTP akışı JPEG'i her durumda kullanır. ROS compressed mesajının
        # kurulumu/yayını ise yalnız gerçek bir abone varken yapılır; aksi
        # halde Nano'da 30 Hz gereksiz DDS işi oluşur.
        if self.compressed_publisher.get_subscription_count() > 0:
            compressed = CompressedImage()
            compressed.header = msg.header
            compressed.format = "jpeg"
            compressed.data = jpeg
            self.compressed_publisher.publish(compressed)
        with _frame_ready:
            _latest_jpeg = jpeg
            _frame_ready.notify_all()


def make_handler(stream_fps):
    period = 1.0 / max(1.0, float(stream_fps))

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                html = index_html()
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
                    # Kare yoksa BEKLE. Eski sürüm burada `continue` ile
                    # boş döngüye giriyordu ve bir çekirdeği doldururdu.
                    with _frame_ready:
                        if _latest_jpeg is None:
                            _frame_ready.wait(timeout=1.0)
                        jpg = _latest_jpeg
                    if jpg is None:
                        continue
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(
                        ("Content-Length: %d\r\n\r\n" % len(jpg)).encode())
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                    time.sleep(period)
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


def main():
    rclpy.init()
    node = MjpegBridge()
    port = int(node.get_parameter("port").value)
    fps = float(node.get_parameter("stream_fps").value)
    srv = ThreadingHTTPServer(("0.0.0.0", port), make_handler(fps))
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
