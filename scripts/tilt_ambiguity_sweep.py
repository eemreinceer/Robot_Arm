#!/usr/bin/env python3
"""Duzlemsel PnP cift-cozum belirsizligini EGIME KARSI olcer — tahta ELDE.

NEDEN BU, STATIK OLCUM DEGIL
  Fronto-paralel dejenerasyonu bugune dek yalniz SENTETIK taramada olculdu
  (`docs/hardware_session_plan.md`: 0 derecede 1.77 derece poz hatasi, 5
  derecede 0.04). Gercek kamerada dogrulamak icin tahtayi 0 derecede sabit
  tutmak gerekirdi -- sabitleme imkani yoksa bu olcum yapilamaz.

  Ama gerekmiyor. Belirsizlik KARE BASINA bir ozelliktir: duzlemsel hedefte
  PnP'nin iki cozumu vardir ve dejenerasyona yaklastikca bu ikisi birbirinden
  ayirt edilemez hale gelir. Yani tahta HAREKET EDEBILIR; her kare bagimsiz
  bir olcumdur. Elde yavasca cevirmek yeterli, hatta gereklidir: egim araligini
  taramak isin ta kendisi.

NE VERIR
  Egim bantlari halinde: iki cozumun reprojeksiyon orani (1'e yaklasmasi =
  ayirt edilemez), aralarindaki aci farki (yanlis cozum secilirse ne kadar
  yaniliriz), ve belirsiz kare orani. `hand_eye_pose_search.py`'deki
  MIN_TILT_DEG su an sentetik taramadan geliyor; bu arac onu OLCUMLE
  degistirmek icin var.

EMNIYET
  Salt dinleyici. Hicbir sey yayinlamaz, hicbir hareket komutlamaz.

KULLANIM
  # tahtayi elde tut, ~30 saniye boyunca YAVASCA cepheden kenara dogru cevir
  python3 scripts/tilt_ambiguity_sweep.py --seconds 45
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

# Iki cozum bu orandan yakinsa, reprojeksiyon aralarinda SECIM YAPAMAZ.
AMBIGUOUS_RATIO = 1.5


class SweepProbe(Node):
    def __init__(self, image_topic, info_topic):
        super().__init__('tilt_ambiguity_sweep')
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


def rotation_gap_deg(rvec_a, rvec_b):
    Ra, _ = cv2.Rodrigues(rvec_a)
    Rb, _ = cv2.Rodrigues(rvec_b)
    value = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(value, -1.0, 1.0))))


def measure_frame(gray, K, dist, objp, centre_offset):
    corners = bp.detect_corners(gray, bp.DEFAULT_BOARD_COLS,
                               bp.DEFAULT_BOARD_ROWS)
    if corners is None:
        return None
    count, rvecs, tvecs, errors = cv2.solvePnPGeneric(
        objp, corners, K, dist, flags=cv2.SOLVEPNP_IPPE)
    if count < 1:
        return None
    R, _ = cv2.Rodrigues(rvecs[0])
    t = np.asarray(tvecs[0], float).ravel()
    centre = R @ centre_offset + t
    cos = abs(float(R[:, 2] @ centre)) / np.linalg.norm(centre)
    tilt = float(np.degrees(np.arccos(np.clip(cos, 0.0, 1.0))))
    best = float(np.asarray(errors[0]).ravel()[0])
    # Dejenere karede solvePnPGeneric NaN dondurebiliyor; dry-run'da ozetin
    # egim araligi "nan-nan" cikti. Olcum olmayan kare kayda GIRMEZ.
    if not (np.isfinite(tilt) and np.isfinite(best)):
        return None
    record = {'tilt_deg': round(tilt, 3),
              'distance_m': round(float(np.linalg.norm(centre)), 4),
              'reproj_best_px': round(best, 4),
              'solutions': int(count)}
    if count >= 2:
        second = float(np.asarray(errors[1]).ravel()[0])
        record['reproj_second_px'] = round(second, 4)
        if not np.isfinite(second):
            return record
        record['ratio'] = round(second / max(best, 1e-9), 4)
        record['rotation_gap_deg'] = round(
            rotation_gap_deg(rvecs[0], rvecs[1]), 3)
        record['translation_gap_mm'] = round(float(np.linalg.norm(
            np.asarray(tvecs[0]).ravel() - np.asarray(tvecs[1]).ravel())
            * 1000.0), 2)
    return record


def summarise(records, edges):
    rows = []
    for low, high in zip(edges[:-1], edges[1:]):
        band = [r for r in records
                if low <= r['tilt_deg'] < high and 'ratio' in r]
        if not band:
            continue
        ratios = np.array([r['ratio'] for r in band])
        gaps = np.array([r['rotation_gap_deg'] for r in band])
        rows.append({'tilt_low': low, 'tilt_high': high, 'frames': len(band),
                     'ratio_median': round(float(np.median(ratios)), 3),
                     'rotation_gap_median_deg': round(float(np.median(gaps)), 2),
                     'ambiguous_fraction': round(
                         float((ratios < AMBIGUOUS_RATIO).mean()), 3)})
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=45.0)
    ap.add_argument('--image-topic', default='/camera/image_raw')
    ap.add_argument('--info-topic', default='/camera/camera_info')
    ap.add_argument('--out', default='runs/hand_eye/tilt_ambiguity.json')
    args = ap.parse_args()

    rclpy.init()
    node = SweepProbe(args.image_topic, args.info_topic)
    deadline = time.monotonic() + 5.0
    while node.K is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.K is None:
        raise SystemExit(f'camera_info yok ({args.info_topic})')
    K, dist = node.K

    objp = bp.board_object_points(bp.DEFAULT_BOARD_COLS, bp.DEFAULT_BOARD_ROWS,
                                  bp.DEFAULT_SQUARE_SIZE_MM)
    centre_offset = np.array([
        (bp.DEFAULT_BOARD_COLS - 1) * bp.DEFAULT_SQUARE_SIZE_MM / 2000.0,
        (bp.DEFAULT_BOARD_ROWS - 1) * bp.DEFAULT_SQUARE_SIZE_MM / 2000.0,
        0.0])

    print(f'{args.seconds:.0f} saniye: tahtayi YAVASCA cepheden kenara cevir')
    records, missed = [], 0
    end = time.monotonic() + args.seconds
    last_print = 0.0
    while time.monotonic() < end and rclpy.ok():
        node.image = None
        wait = time.monotonic() + 0.5
        while node.image is None and time.monotonic() < wait:
            rclpy.spin_once(node, timeout_sec=0.02)
        if node.image is None:
            missed += 1
            continue
        record = measure_frame(cv2.cvtColor(node.image, cv2.COLOR_RGB2GRAY),
                               K, dist, objp, centre_offset)
        if record is None:
            missed += 1
            continue
        records.append(record)
        if time.monotonic() - last_print > 1.0:
            last_print = time.monotonic()
            print('  egim %5.1f deg | oran %s | cozum farki %s deg'
                  % (record['tilt_deg'],
                     record.get('ratio', '-'),
                     record.get('rotation_gap_deg', '-')), flush=True)

    node.destroy_node()
    rclpy.shutdown()

    if len(records) < 20:
        raise SystemExit(f'yalniz {len(records)} kare olculdu '
                         f'({missed} kacti) — tahta kadrajda kaldi mi?')

    tilts = np.array([r['tilt_deg'] for r in records])
    edges = [0, 2, 5, 10, 15, 20, 30, 45, 90]
    rows = summarise(records, edges)

    print(f'\n=== CIFT-COZUM BELIRSIZLIGI ({len(records)} kare, '
          f'egim {tilts.min():.1f}-{tilts.max():.1f} deg) ===')
    print('  egim bandi   kare   oran(med)  cozum farki  belirsiz')
    for row in rows:
        print('  %3d-%3d deg %6d %10.2f %11.1f %9.0f%%'
              % (row['tilt_low'], row['tilt_high'], row['frames'],
                 row['ratio_median'], row['rotation_gap_median_deg'],
                 100.0 * row['ambiguous_fraction']))
    print(f'\n  oran = ikinci cozumun reprojeksiyonu / birincininki.')
    print(f'  {AMBIGUOUS_RATIO} altinda reprojeksiyon ikisi arasinda SECEMEZ.')
    print('  Kapsanmayan egim bandi varsa tahtayi o acilarda da gezdir.')

    out = os.path.join(REPO, args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    json.dump({'frames': len(records), 'missed': missed,
               'ambiguous_ratio_threshold': AMBIGUOUS_RATIO,
               'tilt_range_deg': [float(tilts.min()), float(tilts.max())],
               'bands': rows, 'records': records}, open(out, 'w'), indent=1)
    print(f'\nyazildi: {args.out}')


if __name__ == '__main__':
    main()
