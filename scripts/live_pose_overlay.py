#!/usr/bin/env python3
"""Tahtanin mesafe/egimini goruntunun UZERINE yazar — cetvel yerine.

NEDEN VAR
  `board_tilt_live.py` ayni olcumu terminale basiyor, ama operator tahtayi
  tutarken terminali goremiyor -- ekrani/telefonu izliyor. Bu arac ayni PnP
  hesabini yapip sonucu goruntunun uzerine yazar ve yayinlar, boylece
  mjpeg_bridge uzerinden tarayicidan canli okunabilir.

  Mesafe/egim hedefi olcmek icin cetvel kullanmaya gerek yok: kalibrasyon
  capture'i mesafe (0.3-1.2 m) ve egim (45-60 derece) CESITLILIGI ister,
  hassas bir sayiya kilitlenmek degil -- bu ekran o araligi gozle takip
  etmek icindir.

EMNIYET
  Salt dinleyici + goruntu yayinlayici. Hicbir hareket komutu, hicbir
  kalibrasyon/config dosyasi yazmaz. `board_tilt_live.py` ile ayni PnP
  yolunu (board_pnp.estimate_board_pose) kullanir, sayilar birbirini
  tutar.

KULLANIM
  python3 scripts/live_pose_overlay.py
  # ayri bir terminalde: mjpeg_bridge.py'nin TOPIC'ini /camera/pose_overlay
  # yapip tarayicidan izle.
"""

import argparse
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'src', 'arm_perception'))

import cv2                                                    # noqa: E402
import rclpy                                                  # noqa: E402
from cv_bridge import CvBridge                                # noqa: E402
from rclpy.node import Node                                   # noqa: E402
from rclpy.qos import qos_profile_sensor_data                 # noqa: E402
from sensor_msgs.msg import CameraInfo, Image                 # noqa: E402

from arm_perception import board_pnp as bp                    # noqa: E402


class PoseOverlay(Node):
    def __init__(self, image_topic, info_topic, overlay_topic):
        super().__init__('live_pose_overlay')
        self.bridge = CvBridge()
        self.image = None
        self.K = None
        self.overlay_pub = self.create_publisher(Image, overlay_topic, 1)
        self.create_subscription(Image, image_topic, self._on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_topic, self._on_info,
                                 qos_profile_sensor_data)

    def _on_image(self, message):
        self.image = message

    def _on_info(self, message):
        self.K = (np.array(message.k, np.float64).reshape(3, 3),
                  np.array(message.d, np.float64).reshape(1, -1))


def draw_banner(vis, lines, ok):
    h, w = vis.shape[:2]
    color = (60, 220, 60) if ok else (60, 60, 220)
    cv2.rectangle(vis, (0, 0), (w, 22 * len(lines) + 8), (0, 0, 0), -1)
    for index, line in enumerate(lines):
        cv2.putText(vis, line, (6, 18 + 22 * index),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--image-topic', default='/camera/image_raw')
    ap.add_argument('--info-topic', default='/camera/camera_info')
    ap.add_argument('--overlay-topic', default='/camera/pose_overlay')
    ap.add_argument('--min-distance-m', type=float, default=0.3)
    ap.add_argument('--max-distance-m', type=float, default=1.2)
    ap.add_argument('--min-tilt-deg', type=float, default=45.0)
    ap.add_argument('--max-tilt-deg', type=float, default=60.0)
    args = ap.parse_args()

    rclpy.init()
    node = PoseOverlay(args.image_topic, args.info_topic, args.overlay_topic)

    print(__doc__)
    print(f'Yayinliyor: {args.overlay_topic}  (Ctrl-C ile cik)')

    centre_offset = np.array([
        (bp.DEFAULT_BOARD_COLS - 1) * bp.DEFAULT_SQUARE_SIZE_MM / 2000.0,
        (bp.DEFAULT_BOARD_ROWS - 1) * bp.DEFAULT_SQUARE_SIZE_MM / 2000.0,
        0.0])

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.image is None or node.K is None:
                continue
            frame = node.bridge.imgmsg_to_cv2(node.image, 'rgb8')
            vis = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            K, dist = node.K
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            result = bp.estimate_board_pose(gray, bp.DEFAULT_BOARD_COLS,
                                            bp.DEFAULT_BOARD_ROWS,
                                            bp.DEFAULT_SQUARE_SIZE_MM, K, dist)
            if result is None:
                draw_banner(vis, ['tahta bulunamadi'], ok=False)
            else:
                R, _ = cv2.Rodrigues(result['rvec'])
                t = np.asarray(result['tvec'], float).ravel()
                centre = R @ centre_offset + t
                cos = abs(float(R[:, 2] @ centre)) / np.linalg.norm(centre)
                tilt = float(np.degrees(np.arccos(np.clip(cos, 0.0, 1.0))))
                distance = float(np.linalg.norm(centre))
                dist_ok = args.min_distance_m <= distance <= args.max_distance_m
                tilt_ok = args.min_tilt_deg <= tilt <= args.max_tilt_deg
                draw_banner(vis, [
                    f'mesafe {distance:.2f} m  (hedef {args.min_distance_m:.1f}-'
                    f'{args.max_distance_m:.1f})  {"OK" if dist_ok else "..."}',
                    f'egim   {tilt:5.1f} deg  (hedef {args.min_tilt_deg:.0f}-'
                    f'{args.max_tilt_deg:.0f} ya da ~0)  '
                    f'{"OK" if tilt_ok else "..."}',
                    f'reproj {result["reproj_px"]:.2f} px',
                ], ok=(dist_ok and tilt_ok))

            out = node.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
            out.header = node.image.header
            node.overlay_pub.publish(out)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
