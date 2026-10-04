#!/usr/bin/env python3
"""Sentetik kamera: bilinen pozdaki checkerboard'u yayınlar (ÇEVRİMDIŞI PROVA).

NEDEN VAR
  `measure_perception_floor.py` adım 3a'nın aracı ve commit'inde açıkça yazıyor:
  "has only been syntax- and failure-path-checked; it has never seen a live
  camera." Yani ilk gerçek koşusu kolun başında olacaktı. 2026-08-15'te bunun
  bedelini ölçtük: watchdog ölçümü dört kez geçersiz alındı ve zamanın çoğu
  aracı/rig'i ayıklamaya gitti. Kolun başındaki saatleri ÖLÇÜME harcamak
  istiyorsak, araç oraya gitmeden önce uçtan uca koşmuş olmalı.

  Bu düğüm `/camera/image_raw` + `/camera/camera_info` yayınlar, tıpkı gerçek
  kamera gibi. Fark: board'un pozu BİZİM SEÇTİĞİMİZ değerdir. Dolayısıyla
  yalnız "araç çalışıyor mu" değil, "doğru cevabı veriyor mu" da sorulabilir --
  gerçek kamerada yer gerçeği yoktur, burada vardır.

NE KANITLAR, NE KANITLAMAZ
  Kanıtlar: abone olma, kare çözme, köşe tespiti, PnP, saçılım hesabı ve
  artifact yazma zincirinin tamamı; ve sıfır gürültüde çözülen pozun yer
  gerçeğine oturduğu.
  KANITLAMAZ: gerçek kameranın gürültü tabanını. Sentetik render'da lens
  bulanıklığı, aydınlatma, motion blur, rolling shutter, JPEG artefaktı yoktur.
  Buradan çıkan taban sayısı GERÇEK TABAN DEĞİLDİR; yalnız aracın matematiğinin
  doğru olduğunu gösterir.

KULLANIM
  # bir terminalde:
  python3 scripts/fake_board_camera.py --noise-sigma 0.0
  # başka terminalde:
  python3 scripts/measure_perception_floor.py --frames 30 --label prova
"""

import argparse
import math
import os
import sys
import time

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'src', 'arm_perception'))

from arm_perception.board_pnp import (  # noqa: E402
    DEFAULT_BOARD_COLS, DEFAULT_BOARD_ROWS, DEFAULT_SQUARE_SIZE_MM)

# fillConvexPoly'nin sabit nokta kesir biti sayısı (1/64 px).
FILL_SHIFT = 6


def render_board(K, D, rvec, tvec, width, height, cols, rows, square_m,
                 noise_sigma=0.0, rng=None, supersample=1):
    """Board'u beyaz zemine siyah kareler olarak çizer.

    findChessboardCorners iç köşeleri arar, yani cols x rows iç köşe için
    (cols+1) x (rows+1) kare gerekir. Kareler tek tek projekte edilip
    doldurulur; böylece perspektif her karede doğru olur (tek homografi ile
    warp etmek düzlemsel board için de çalışırdı ama kenar payını yönetmek
    daha kırılgan).

    Board'un ETRAFINA bir kare genişliğinde beyaz pay bırakılır: OpenCV kenara
    dayanmış bir board'da köşeleri güvenilir bulamaz.

    ALT-PİKSEL DOLDURMA ŞART, SUPERSAMPLE DEĞİL. İlk sürüm köşeleri tam sayıya
    yuvarlayarak dolduruyordu ve reprojeksiyon 1.20 px çıkıyordu -- aracın
    1.0 px kapısının üstünde, yani HER kare reddediliyordu (gerçek yakalamalarda
    bu sayı 0.19-0.23 px). Kapıyı gevşetmek yanlış cevap olurdu: prova, aracı
    sahada kullanılacağından farklı bir ayarla sınamış olurdu.

    İlk açıklamam "rasterizasyon gürültüsü, supersample çözer" idi; ÖLÇÜM
    ÇÜRÜTTÜ -- supersample'ı 1'den 8'e çıkarmak reprojeksiyonu düşürmedi
    (0.20-1.44 arası düzensiz gezindi). Gerçek sebep sistematik bir kırpmaydı:
    köşeler her seferinde (-0.5, -0.5) px kayıyordu. `shift` ile sabit noktalı
    doldurmaya geçince sapma (0.013, 0.048) px'e indi.

    Supersample artık ZARARLI: INTER_AREA'nın kendi ızgara kayması yeni bir
    önyargı ekliyor (s=2'de -0.26 px, s=4'te -0.39 px). Varsayılan 1.
    """
    scale = max(1, int(supersample))
    big_w, big_h = width * scale, height * scale
    big_K = K.copy()
    big_K[0, 0] *= scale
    big_K[1, 1] *= scale
    big_K[0, 2] *= scale
    big_K[1, 2] *= scale
    image = np.full((big_h, big_w, 3), 255, np.uint8)

    def project(points_board):
        pts, _ = cv2.projectPoints(
            np.asarray(points_board, np.float32), rvec, tvec, big_K, D)
        return pts.reshape(-1, 2)

    # board_object_points ile AYNI orijin: iç köşe (0,0) noktası board
    # koordinatlarında (0,0,0). Kareler bir kare geriden başlar.
    for row in range(rows + 1):
        for col in range(cols + 1):
            if (row + col) % 2 == 1:
                continue
            x0 = (col - 1) * square_m
            y0 = (row - 1) * square_m
            quad = [(x0, y0, 0.0),
                    (x0 + square_m, y0, 0.0),
                    (x0 + square_m, y0 + square_m, 0.0),
                    (x0, y0 + square_m, 0.0)]
            pts = project(quad)
            if not np.all(np.isfinite(pts)):
                continue
            # Sabit noktalı koordinat (1/64 px). Tam sayıya YUVARLAMAK köşeleri
            # sistematik olarak (-0.5, -0.5) px kaydırıyordu -- ölçüldü
            # 2026-08-15, ve supersample arttırmak DÜZELTMİYORDU, çünkü hata
            # örnekleme değil kırpma kaynaklıydı. Yanlış teşhisten dönüş:
            # önce bunu rasterizasyon gürültüsü sanmıştım; sabit ofset olması
            # ve supersample ile küçülmemesi o açıklamayı çürüttü.
            fixed = np.round(pts * (1 << FILL_SHIFT)).astype(np.int32)
            cv2.fillConvexPoly(image, fixed, (0, 0, 0), cv2.LINE_AA, FILL_SHIFT)

    if scale > 1:
        image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)

    if noise_sigma > 0.0:
        rng = rng or np.random.default_rng()
        noise = rng.normal(0.0, noise_sigma, image.shape)
        image = np.clip(image.astype(np.float64) + noise, 0, 255).astype(np.uint8)
    return image


class FakeBoardCamera(Node):
    def __init__(self, args):
        super().__init__('fake_board_camera')
        self.args = args
        self.bridge = CvBridge()
        self.rng = np.random.default_rng(args.seed)

        fx = fy = args.focal_px
        self.K = np.array([[fx, 0.0, args.width / 2.0],
                           [0.0, fy, args.height / 2.0],
                           [0.0, 0.0, 1.0]], np.float64)
        self.D = np.zeros((1, 5), np.float64)

        # Board'u kadrajın ortasına getir: iç köşe ızgarasının merkezini
        # optik eksene taşı, sonra istenen dönmeyi uygula.
        span_x = (args.cols - 1) * args.square_m
        span_y = (args.rows - 1) * args.square_m
        self.rvec = np.array(
            [math.radians(args.roll_deg),
             math.radians(args.pitch_deg),
             math.radians(args.yaw_deg)], np.float64).reshape(3, 1)
        R, _ = cv2.Rodrigues(self.rvec)
        centre_board = np.array([span_x / 2.0, span_y / 2.0, 0.0], np.float64)
        self.tvec = (np.array([0.0, 0.0, args.distance_m], np.float64)
                     - R.dot(centre_board)).reshape(3, 1)

        # Gercek kamera (csi_camera_node, gz) BEST_EFFORT yayinlar. Bu sahte
        # kamera RELIABLE yayinlarsa provada gecen bir arac kolun basinda
        # sessizce hicbir kare almaz -- 2026-08-18'de tam bu oldu.
        self.image_publisher = self.create_publisher(
            Image, args.image_topic, qos_profile_sensor_data)
        self.info_publisher = self.create_publisher(
            CameraInfo, args.info_topic, qos_profile_sensor_data)
        self.create_timer(1.0 / args.rate_hz, self._tick)
        self.frames = 0
        self.started_at = time.time()

        self.get_logger().info(
            'yer gercegi: rvec=%s tvec=%s (mesafe %.3f m)'
            % (np.round(self.rvec.ravel(), 6).tolist(),
               np.round(self.tvec.ravel(), 6).tolist(), args.distance_m))

    def ground_truth(self):
        return self.rvec.copy(), self.tvec.copy()

    def _sweep_pose(self):
        """Tahtayı zamanla döndür (flip_sweep.py'nin istediği girdi).

        flip_sweep yalnız kamera konularını dinler ve tahtanın ÇEVRİLMESİNİ
        bekler -- "kendi düzleminde döndür VE eğ" diyor. Elle çevrilecek bir
        tahta yokken bu tarama otomatik sürülebilir: düzlem-içi dönme köşe
        sıralamasının kırılma noktasını, eğim ise PnP koşullanmasını yoklar.
        """
        elapsed = time.time() - self.started_at
        spin = math.radians(self.args.spin_dps * elapsed)
        tilt = math.radians(
            self.args.tilt_deg * math.sin(2.0 * math.pi * elapsed
                                          / max(self.args.tilt_period_s, 1e-6)))
        # Önce montaj oryantasyonu, sonra düzlem-içi dönme (board'un kendi z'si).
        base, _ = cv2.Rodrigues(self.rvec)
        tilt_R, _ = cv2.Rodrigues(np.array([tilt, 0.0, 0.0], np.float64))
        spin_R, _ = cv2.Rodrigues(np.array([0.0, 0.0, spin], np.float64))
        rotation = tilt_R @ base @ spin_R
        rvec, _ = cv2.Rodrigues(rotation)
        span = np.array([(self.args.cols - 1) * self.args.square_m / 2.0,
                         (self.args.rows - 1) * self.args.square_m / 2.0, 0.0])
        tvec = (np.array([0.0, 0.0, self.args.distance_m])
                - rotation.dot(span)).reshape(3, 1)
        return rvec, tvec

    def _tick(self):
        if self.args.spin_dps or self.args.tilt_deg:
            self.rvec, self.tvec = self._sweep_pose()
        image = render_board(
            self.K, self.D, self.rvec, self.tvec,
            self.args.width, self.args.height,
            self.args.cols, self.args.rows, self.args.square_m,
            noise_sigma=self.args.noise_sigma, rng=self.rng,
            supersample=self.args.supersample)

        stamp = self.get_clock().now().to_msg()
        message = self.bridge.cv2_to_imgmsg(image, 'rgb8')
        message.header.stamp = stamp
        message.header.frame_id = 'camera_optical_frame'
        self.image_publisher.publish(message)

        info = CameraInfo()
        info.header = message.header
        info.width = self.args.width
        info.height = self.args.height
        info.k = self.K.reshape(-1).tolist()
        info.d = self.D.reshape(-1).tolist()
        info.distortion_model = 'plumb_bob'
        self.info_publisher.publish(info)
        self.frames += 1


def build_parser():
    parser = argparse.ArgumentParser(
        description='bilinen pozdaki sentetik checkerboard yayinlar')
    parser.add_argument('--image-topic', default='/camera/image_raw')
    parser.add_argument('--info-topic', default='/camera/camera_info')
    parser.add_argument('--rate-hz', type=float, default=10.0)
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--focal-px', type=float, default=900.0)
    parser.add_argument('--distance-m', type=float, default=0.40)
    parser.add_argument('--roll-deg', type=float, default=0.0)
    parser.add_argument('--pitch-deg', type=float, default=15.0,
                        help='VARSAYILAN SIFIR DEGIL. Tahtaya tam karsidan '
                             'bakmak duzlemsel PnP icin dejenere durumdur: '
                             '2026-08-15 olcumu, 0 derece egimde poz hatasi '
                             '1.77 derece, 5 derecede 0.04. Sifir vermek '
                             'isteyen bilerek versin.')
    parser.add_argument('--yaw-deg', type=float, default=0.0)
    parser.add_argument('--noise-sigma', type=float, default=0.0,
                        help='piksel yogunluguna eklenen gauss gurultusu; '
                             '0 = gurultusuz (taban ~0 cikmali)')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--spin-dps', type=float, default=0.0,
                        help='tahtayi kendi duzleminde saniyede kac derece '
                             'dondur (flip_sweep icin; 0 = sabit)')
    parser.add_argument('--tilt-deg', type=float, default=0.0,
                        help='egim salinimi genligi (derece)')
    parser.add_argument('--tilt-period-s', type=float, default=20.0)
    parser.add_argument('--supersample', type=int, default=1,
                        help='1 birak. Alt-piksel doldurma (FILL_SHIFT) '
                             'kaymayi zaten kaldiriyor; buyutmek INTER_AREA '
                             'izgara kaymasi yuzunden yeni bir onyargi EKLIYOR '
                             '(s=2 icin -0.26 px, s=4 icin -0.39 px).')
    parser.add_argument('--cols', type=int, default=DEFAULT_BOARD_COLS)
    parser.add_argument('--rows', type=int, default=DEFAULT_BOARD_ROWS)
    parser.add_argument('--square-m', type=float,
                        default=DEFAULT_SQUARE_SIZE_MM / 1000.0)
    return parser


def main():
    args = build_parser().parse_args()
    rclpy.init()
    node = FakeBoardCamera(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Ctrl-C rclpy'nin sinyal isleyicisiyle context'i zaten kapatmis
        # olabilir; kosulsuz shutdown o durumda RCLError firlatiyor ve cikisi
        # bir hata gibi gosteriyor.
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
