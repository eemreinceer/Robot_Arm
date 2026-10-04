#!/usr/bin/env python3
"""Reprojeksiyon artiginin deseni TAHTADA mi LENSTE mi — iki gorunumle ayirir.

NEDEN
  2026-08-18'de olculdu: reprojeksiyon buyutmeyle orantili artiyor (76 px ayak
  izinde 0.241, 149 px'te 0.496 -- oran 0.0032 / 0.0033) ve artigin %90'i tahta
  koordinatinda bir kuadratikle aciklaniyor, kareler arasi dalgalanma ise 0.081
  px. Yani artik gurultu degil, SABIT ve DUZGUN bir desen. Iki aday var:

    (a) tahta duz degil (kagit kartona tam yapismamis, kambur),
    (b) lens distorsiyon modeli yetersiz -- intrinsics'in ESKI 25 mm tahtayla
        alindigi `board_pnp` kaynaginda yaziyor.

  Cozumleri farkli: (a) icin duz bir tahta basilir, (b) icin intrinsics yeniden
  kalibre edilir. Tek gorunum ayirt EDEMEZ, cunku tek gorunumde tahta ve
  goruntu koordinatlari arasindaki donusum neredeyse afindir.

  Ayirt eden sey su: desen TAHTA koordinatinda sabitse kusur tahtadadir; tahta
  goruntude yer/aci degistirdiginde desen de degisiyorsa kusur lenstedir.

KULLANIM
  python3 scripts/board_residual_probe.py --save runs/hand_eye/residual_a.json
  # tahtayi EGIP ve/veya kaydirip:
  python3 scripts/board_residual_probe.py --save runs/hand_eye/residual_b.json \
      --compare runs/hand_eye/residual_a.json

EMNIYET
  Salt dinleyici. Hicbir sey yayinlamaz, hicbir hareket komutlamaz.
"""

import argparse
import json
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


class Probe(Node):
    def __init__(self):
        super().__init__('board_residual_probe')
        self.bridge = CvBridge()
        self.images = []
        self.K = None
        self.create_subscription(Image, '/camera/image_raw', self._on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera_info',
                                 self._on_info, qos_profile_sensor_data)

    def _on_image(self, message):
        self.images.append(self.bridge.imgmsg_to_cv2(message, 'rgb8'))

    def _on_info(self, message):
        self.K = (np.array(message.k, np.float64).reshape(3, 3),
                  np.array(message.d, np.float64).reshape(1, -1))


def capture(frames):
    rclpy.init()
    node = Probe()
    end = time.monotonic() + 15.0
    while time.monotonic() < end and (node.K is None
                                      or len(node.images) < frames):
        rclpy.spin_once(node, timeout_sec=0.05)
    if node.K is None:
        raise SystemExit('camera_info yok')
    K, dist = node.K
    objp = bp.board_object_points(bp.DEFAULT_BOARD_COLS,
                                  bp.DEFAULT_BOARD_ROWS,
                                  bp.DEFAULT_SQUARE_SIZE_MM)
    residuals, pixels, poses = [], [], []
    for image in node.images[-frames:]:
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        corners = bp.detect_corners(gray, bp.DEFAULT_BOARD_COLS,
                                    bp.DEFAULT_BOARD_ROWS)
        if corners is None:
            continue
        ok, rvec, tvec = cv2.solvePnP(objp, corners, K, dist)
        projected, _ = cv2.projectPoints(objp, rvec, tvec, K, dist)
        residuals.append(projected.reshape(-1, 2) - corners.reshape(-1, 2))
        pixels.append(corners.reshape(-1, 2))
        R, _ = cv2.Rodrigues(rvec)
        centre = R @ (objp.mean(axis=0)) + np.asarray(tvec, float).ravel()
        cos = abs(float(R[:, 2] @ centre)) / np.linalg.norm(centre)
        poses.append({'tilt_deg': float(np.degrees(np.arccos(np.clip(cos, 0, 1)))),
                      'distance_m': float(np.linalg.norm(centre))})
    node.destroy_node()
    rclpy.shutdown()
    if len(residuals) < 5:
        raise SystemExit(f'yalniz {len(residuals)} kare — tahta kadrajda mi?')
    mean = np.mean(residuals, axis=0)
    return {'frames': len(residuals),
            'residual_px': mean.tolist(),
            'frame_to_frame_std_px': float(np.std(residuals, axis=0).mean()),
            'mean_magnitude_px': float(np.linalg.norm(mean, axis=1).mean()),
            'pixels': np.mean(pixels, axis=0).tolist(),
            'tilt_deg': float(np.median([p['tilt_deg'] for p in poses])),
            'distance_m': float(np.median([p['distance_m'] for p in poses]))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frames', type=int, default=10)
    ap.add_argument('--save', required=True)
    ap.add_argument('--compare', default=None)
    args = ap.parse_args()

    result = capture(args.frames)
    print('kare %d | mesafe %.2f m | egim %.1f deg'
          % (result['frames'], result['distance_m'], result['tilt_deg']))
    print('ortalama artik %.3f px | kareler arasi std %.3f px'
          % (result['mean_magnitude_px'], result['frame_to_frame_std_px']))

    out = os.path.join(REPO, args.save)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    json.dump(result, open(out, 'w'), indent=1)
    print('yazildi:', args.save)

    if args.compare:
        other = json.load(open(os.path.join(REPO, args.compare)))
        a = np.array(other['residual_px'])
        b = np.array(result['residual_px'])
        # Desen tahta koordinatinda AYNI mi? Buyutme farkini normalize et.
        scale = (other['distance_m'] / result['distance_m'])
        corr = float(np.corrcoef(a.ravel(), b.ravel())[0, 1])
        print()
        print('KARSILASTIRMA (%s)' % args.compare)
        print('  a: mesafe %.2f m, egim %.1f deg, artik %.3f px'
              % (other['distance_m'], other['tilt_deg'],
                 other['mean_magnitude_px']))
        print('  b: mesafe %.2f m, egim %.1f deg, artik %.3f px'
              % (result['distance_m'], result['tilt_deg'],
                 result['mean_magnitude_px']))
        print('  tahta koordinatinda desen korelasyonu: %+.2f' % corr)
        print('  (buyutme orani %.2f)' % scale)
        if corr > 0.7:
            print('  => desen TAHTA ile birlikte tasiniyor: kusur TAHTADA '
                  '(duz degil). Duz bir tahta basip tekrar olc.')
        elif corr < 0.3:
            print('  => desen tahtayla tasinmiyor: kusur LENS/INTRINSICS '
                  'tarafinda. Intrinsics yeniden kalibre edilmeli.')
        else:
            print('  => ayrim net degil; iki gorunum arasindaki fark yetersiz '
                  'olabilir. Tahtayi daha cok egip/kaydirip tekrarla.')


if __name__ == '__main__':
    main()
