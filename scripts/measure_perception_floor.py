#!/usr/bin/env python3
"""Açısal algı tabanını ölç — KOL HAREKET ETMEZ (D3-a).

NE ÖLÇÜYOR
  Kol neredeyse orada dururken arka arkaya N kare alır ve tahtanın kamera
  frame'indeki pozunun saçılımını çıkarır. Kol kımıldamadığı için buradaki
  saçılım **saf algı gürültüsüdür**: PnP, köşe tespiti, sensör gürültüsü,
  aydınlatma. Servo tekrarlanabilirliği ve FK hatası bu sayıya KARIŞMAZ.

NEDEN AÇISAL
  2026-08-14'te ölçüldü: hand-eye çözümünün hatasını belirleyen şey tahtanın
  ÖTELEME hatası değil, ORYANTASYON hatasıdır. 0.41 mm öteleme gürültüsü tek
  başına X hatasını 0.45 mm veriyor; üzerine yalnız 0.5 derece dönme eklemek
  2.94 mm yapıyor -- gerçek veride ölçülenin ta kendisi. Ayrıntı:
  `docs/hand_eye_pose_set_requirement.md` son bölüm.

  Sim'de bu sayı p90 0.46 derece. **Gerçek kolda ne?** Bilmiyoruz; bu araç onu
  ölçmek için var. Cevap, sıradaki işin algı tarafında mı mekanik tarafta mı
  olduğunu belirleyecek.

EMNİYET
  Bu araç HİÇBİR ŞEY YAYINLAMAZ ve hiçbir hareket komutlamaz. Yalnız kamera
  konusunu dinler. Ray kesikken de, kol enerjiliyken de koşulabilir; kolun
  duruşunu değiştirmez. Tek şartı tahtanın kadrajda olması.

KULLANIM
  # kol nerede duruyorsa orada, 60 kare:
  python3 scripts/measure_perception_floor.py --frames 60

  # birden çok duruşta ölçmek için: kolu ELLE (ya da ayrı bir araçla) taşı,
  # her duruşta bunu tekrar koştur ve --label ile ayır.
  python3 scripts/measure_perception_floor.py --frames 60 --label pose_a
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


def rotation_angle_deg(R):
    value = (np.trace(R) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(value, -1.0, 1.0))))


class FloorProbe(Node):
    """Salt DİNLEYİCİ. Yayıncısı yok; hiçbir aktüatöre erişmez."""

    def __init__(self, image_topic, info_topic):
        super().__init__('perception_floor_probe')
        self.bridge = CvBridge()
        self.image = None
        self.K = None
        # QoS gercek kameraya gore secilir, varsayilana gore degil: hem
        # csi_camera_node hem gz kamerasi BEST_EFFORT yayinlar, varsayilan
        # abone ise RELIABLE ister ve TEK KARE almaz. Bu arac 2026-08-18'de
        # Nano'da tam bunu yasadi -- "camera_info yok, kamera ayakta mi?"
        # diyerek cikti, oysa kamera 29 Hz yayindaydi.
        self.create_subscription(Image, image_topic, self._on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_topic, self._on_info,
                                 qos_profile_sensor_data)

    def _on_image(self, message):
        self.image = self.bridge.imgmsg_to_cv2(message, 'rgb8')

    def _on_info(self, message):
        self.K = (np.array(message.k, np.float64).reshape(3, 3),
                  np.array(message.d, np.float64).reshape(1, -1))

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--frames', type=int, default=60)
    parser.add_argument('--image-topic', default='/camera/image_raw')
    parser.add_argument('--info-topic', default='/camera/camera_info')
    parser.add_argument('--label', default='')
    parser.add_argument('--out', default='runs/hand_eye/perception_floor.json')
    parser.add_argument('--max-reproj-px', type=float, default=1.0,
                        help='bu esigin ustundeki kareler REDDEDILIR ve ayrica '
                             'raporlanir; sessizce ortalamaya karismaz')
    args = parser.parse_args()

    rclpy.init()
    node = FloorProbe(args.image_topic, args.info_topic)
    node.spin(4.0)
    if node.K is None:
        raise SystemExit(f'camera_info yok ({args.info_topic}) — kamera ayakta mi?')
    K, dist = node.K

    translations, rotations, reprojections = [], [], []
    # Red sebepleri AYRI sayilir. Tek sayacla toplanınca "kare gelmedi",
    # "tahta bulunamadi" ve "reprojeksiyon kapisi" ayni gorunuyordu ve arac
    # ucunun de "tahta kadrajda mi?" diye sormasina yol aciyordu -- 2026-08-18'de
    # tam bu oldu: tahta kadrajdaydi, bulunuyordu, kapida takiliyordu.
    no_frame = no_board = high_reproj = 0
    worst_reproj = 0.0
    for index in range(args.frames):
        node.image = None
        node.spin(1.0)
        if node.image is None:
            no_frame += 1
            continue
        gray = cv2.cvtColor(node.image, cv2.COLOR_RGB2GRAY)
        result = bp.estimate_board_pose(gray, bp.DEFAULT_BOARD_COLS,
                                        bp.DEFAULT_BOARD_ROWS,
                                        bp.DEFAULT_SQUARE_SIZE_MM, K, dist)
        if result is None:
            no_board += 1
            continue
        if result['reproj_px'] > args.max_reproj_px:
            high_reproj += 1
            worst_reproj = max(worst_reproj, float(result['reproj_px']))
            continue
        R, _ = cv2.Rodrigues(result['rvec'])
        translations.append(np.asarray(result['tvec'], float).ravel())
        rotations.append(R)
        reprojections.append(float(result['reproj_px']))
        print(f'  kare {index + 1:3d}/{args.frames}: reproj {result["reproj_px"]:.3f} px')

    node.destroy_node()
    rclpy.shutdown()

    rejected = no_frame + no_board + high_reproj
    if len(translations) < 5:
        detail = (f'kare gelmedi {no_frame} | tahta bulunamadi {no_board} | '
                  f'reprojeksiyon kapisi {high_reproj}')
        hint = 'tahta kadrajda mi?'
        if high_reproj > no_frame + no_board:
            hint = (f'tahta BULUNUYOR ama reprojeksiyon kapiyi asiyor '
                    f'(en kotu {worst_reproj:.2f} px > {args.max_reproj_px} px). '
                    'Tahta kadrajda cok kucuk ya da goruntu bulanik olabilir; '
                    'yaklastir ve isigi artir.')
        raise SystemExit(f'yalniz {len(translations)} gecerli kare — {detail}\n{hint}')

    translations = np.array(translations)
    centre = np.median(translations, axis=0)
    deltas = translations - centre
    offsets = np.linalg.norm(deltas, axis=1) * 1000.0

    # DERINLIK / YANAL AYRIMI. Toplam sacilim tek sayi olarak "1.3 mm" der ve
    # ne yapilacagini soylemez; derinlikse tahtayi yaklastirmak dogrudan
    # dusurur, yanalsa dusurmez. 2026-08-18'de olculen oteleme tabani modelin
    # varsaydiginin 3 kati cikinca bu ayrim gerekli oldu.
    axis = centre / np.linalg.norm(centre)          # kameradan tahtaya bakis
    depth = np.abs(deltas @ axis) * 1000.0
    lateral = np.linalg.norm(deltas - np.outer(deltas @ axis, axis),
                             axis=1) * 1000.0

    mean_rotation = np.mean(rotations, axis=0)
    U, _, Vt = np.linalg.svd(mean_rotation)
    reference = U @ Vt
    angles = np.array([rotation_angle_deg(reference.T @ R) for R in rotations])

    print(f'\n=== ALGI TABANI ({len(translations)} gecerli kare, {rejected} reddedildi) ===')
    print(f'  oteleme sacilimi : medyan {np.median(offsets):.3f} mm | '
          f'p90 {np.quantile(offsets, 0.90):.3f} | maks {offsets.max():.3f}')
    print(f'    derinlik       : medyan {np.median(depth):.3f} mm | '
          f'p90 {np.quantile(depth, 0.90):.3f}')
    print(f'    yanal          : medyan {np.median(lateral):.3f} mm | '
          f'p90 {np.quantile(lateral, 0.90):.3f}')
    print(f'  ACISAL sacilim   : medyan {np.median(angles):.3f} deg | '
          f'p90 {np.quantile(angles, 0.90):.3f} | maks {angles.max():.3f}')
    print(f'  reprojeksiyon    : medyan {np.median(reprojections):.3f} px')
    print('\n  KARSILASTIRMA (sim, 2026-08-14): acisal p90 0.46 deg')
    print('  Bu sayi hand-eye hatasini BELIRLEYEN taban. 0.5 deg ~ 2.9 mm X hatasi.')
    print('  KOL HAREKET ETMEDI: burada olculen sey yalnizca ALGI gurultusudur;')
    print('  servo/FK payi icin ayrica tekrarlanabilirlik olcumu gerekir (D3-b).')

    payload = {
        'label': args.label,
        'frames_requested': args.frames,
        'frames_used': len(translations),
        'frames_rejected': rejected,
        'rejected_detail': {'no_frame': no_frame, 'no_board': no_board,
                            'high_reproj': high_reproj},
        'translation_mm': {'median': float(np.median(offsets)),
                           'p90': float(np.quantile(offsets, 0.90)),
                           'max': float(offsets.max())},
        'angular_deg': {'median': float(np.median(angles)),
                        'p90': float(np.quantile(angles, 0.90)),
                        'max': float(angles.max())},
        'translation_depth_mm': {'median': float(np.median(depth)),
                                 'p90': float(np.quantile(depth, 0.90))},
        'translation_lateral_mm': {'median': float(np.median(lateral)),
                                   'p90': float(np.quantile(lateral, 0.90))},
        'reproj_px_median': float(np.median(reprojections)),
        # HAM KARELER. Ozet istatistikler (medyan/p90) modelin tukettigi
        # eksen-basi sigma'ya dogrudan cevrilemez: cevrim dagilim varsayimi
        # gerektirir ve 2026-08-18'de o varsayimin dogrulugu HEDEFIN GECILIP
        # GECILMEDIGINI belirler hale geldi. Ham degerler saklanirsa sigma
        # varsayimsiz kestirilir ve dagilim ayrica SINANIR.
        'samples': {
            'translation_m': [[float(v) for v in row] for row in translations],
            'centre_m': [float(v) for v in centre],
            'angle_deg': [float(v) for v in angles],
            'reproj_px': [float(v) for v in reprojections],
            'note': 'kamera frame; centre orneklerin medyani',
        },
        # D3-b ("ayni pozu farkli yaklasma yonlerinden ziyaret et, 3a'nin
        # ustune cikan fark mekaniktir") iki kosuyu KARSILASTIRMAK zorunda, ama
        # sacilim tek basina iki kosuyu iliskilendirmez -- pozun kendisi de
        # gerekir. Ceyrek gunluk cevrimdisi provada fark edildi (2026-08-15);
        # kolun basinda fark edilseydi ziyaretler tekrar edilmek zorunda kalirdi.
        'board_pose_camera_frame': {
            'translation_m': [float(v) for v in centre],
            'rotation_matrix': [[float(v) for v in row] for row in reference],
            'note': 'median translation and chordal-mean rotation over the '
                    'accepted frames; this is the reference the scatter above '
                    'is measured against',
        },
        'note': 'arm stationary; perception noise only, no servo/FK contribution',
    }
    out = os.path.join(REPO, args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w') as handle:
        json.dump(payload, handle, indent=1)
    print(f'\nyazildi: {args.out}')


if __name__ == '__main__':
    main()
