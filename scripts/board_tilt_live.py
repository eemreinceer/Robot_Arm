#!/usr/bin/env python3
"""Tahtanin egimini CANLI yazar — yerlestirirken bakmak icin.

NE OLCUYOR
  Tahta normali ile tahtanin merkezine giden bakis hatti arasindaki aci.
  0 derece = tam cepheden (duzlemsel PnP'nin DEJENERE oldugu bolge),
  90 derece = tahtaya kenardan bakis.

NEDEN GEREKLI
  Egimi gozle 0-2 dereceye ya da 10 derecenin ustune getirmek mumkun degil;
  trapez gorunumu birkac derecede fark edilmiyor. 2026-08-15'te olculdu:
  0 derecede poz hatasi 1.77 derece, 5 derecede 0.04 -- yani karar birkac
  derecede veriliyor.

EMNIYET
  Salt dinleyici. Hicbir sey yayinlamaz, hicbir hareket komutlamaz.

KULLANIM
  python3 scripts/board_tilt_live.py            # ctrl-c ile cik
"""

import argparse
import os
import sys
import time

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


class TiltProbe(Node):
    def __init__(self, image_topic, info_topic):
        super().__init__('board_tilt_live')
        self.bridge = CvBridge()
        self.image = None
        self.K = None
        self.create_subscription(Image, image_topic, self._on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_topic, self._on_info,
                                 qos_profile_sensor_data)

    def _on_image(self, message):
        self.image = self.bridge.imgmsg_to_cv2(message, 'rgb8')

    def _on_info(self, message):
        self.K = (np.array(message.k, np.float64).reshape(3, 3),
                  np.array(message.d, np.float64).reshape(1, -1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--image-topic', default='/camera/image_raw')
    ap.add_argument('--info-topic', default='/camera/camera_info')
    ap.add_argument('--target-deg', type=float, default=0.0,
                    help='hedef egim; ekranda farki gosterir')
    args = ap.parse_args()

    rclpy.init()
    node = TiltProbe(args.image_topic, args.info_topic)
    deadline = time.monotonic() + 5.0
    while node.K is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.K is None:
        raise SystemExit(f'camera_info yok ({args.info_topic})')
    K, dist = node.K

    centre_offset = np.array([
        (bp.DEFAULT_BOARD_COLS - 1) * bp.DEFAULT_SQUARE_SIZE_MM / 2000.0,
        (bp.DEFAULT_BOARD_ROWS - 1) * bp.DEFAULT_SQUARE_SIZE_MM / 2000.0,
        0.0])
    try:
        while rclpy.ok():
            node.image = None
            end = time.monotonic() + 1.0
            while node.image is None and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
            if node.image is None:
                print('kare yok', flush=True)
                continue
            gray = cv2.cvtColor(node.image, cv2.COLOR_RGB2GRAY)
            result = bp.estimate_board_pose(gray, bp.DEFAULT_BOARD_COLS,
                                            bp.DEFAULT_BOARD_ROWS,
                                            bp.DEFAULT_SQUARE_SIZE_MM, K, dist)
            if result is None:
                print('tahta bulunamadi', flush=True)
                continue
            R, _ = cv2.Rodrigues(result['rvec'])
            t = np.asarray(result['tvec'], float).ravel()
            centre = R @ centre_offset + t
            cos = abs(float(R[:, 2] @ centre)) / np.linalg.norm(centre)
            tilt = np.degrees(np.arccos(np.clip(cos, 0.0, 1.0)))
            print('egim %5.1f deg (hedef %.0f, fark %+5.1f) | mesafe %.2f m | '
                  'reproj %.2f px'
                  % (tilt, args.target_deg, tilt - args.target_deg,
                     np.linalg.norm(centre), result['reproj_px']), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
