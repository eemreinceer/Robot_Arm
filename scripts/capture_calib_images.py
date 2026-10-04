#!/usr/bin/env python3
"""Kamera intrinsics kalibrasyonu icin satranc tahtasi kareleri topla.

`/camera/image_raw` dinler, her karede satranc tahtasini arar ve KABUL EDILEN
kareleri diske yazar. Kabul kriteri sadece "tahta goruldu" degildir:

  * Ayni yerde duran tahtadan onlarca kare toplamak kalibrasyonu IYILESTIRMEZ,
    sadece ayni bilgiyi tekrarlar. Araç yeni karenin oncekilerden yeterince
    FARKLI oldugunu arar (kose merkezi ve tahtanin egimi).
  * Goruntunun kenarlarindan ve koselerinden veri olmadan lens distorsiyonu
    kestirilemez. Arac goruntuyu 3x3 boleye ayirir ve hangi bolgelerin hala
    bos oldugunu SOYLER.

NEREDE CALISTIRILIR: **PC'de**, Nano'da degil. Nano yalnizca goruntuyu
yayinlar. Sebep olculdu (2026-07-22): tespit maliyeti tamamen cozunurluge
bagli ve Jetson Nano'da 320x240'ta bile ~170 ms/kare. Arac Nano'da kosunca
geri bildirim goruntusu 1.1 Hz'e dusuyor ve bir cekirdek doluyor; ayni arac
PC'de ayni akisi isleyip **15.1 Hz** veriyor. Kaydedilen kareler de PC'de,
repoda birikir.

Kullanim (PC'deki humble konteyneri, kamera node'u Nano'da ayaktayken):
    python3 capture_calib_images.py --cols 6 --rows 8 \
        --square-mm 25 --flip-method 2 \
        --out runs/calib_$(date +%Y%m%d_%H%M) --detect-hz 10 --overlay-hz 15

Canli geri bildirim: `/camera/calib_overlay` — taninan koseler, kapsama
haritasi ve sayaclar goruntunun uzerine cizilir. Izlemek icin:
    ros2 run rqt_image_view rqt_image_view /camera/calib_overlay

KRITIK: kalibrasyon, KULLANACAGIN goruntuyle ayni olmali. csi_camera_node
1640x1232 yakalayip 640x480'e olcekliyor ve flip_method uyguluyor; kalibrasyon
da o cikistan yapilmali. Farkli cozunurluk/flip ile alinan intrinsics sessizce
yanlis olur. Arac gordugu goruntu boyutunu kaydeder ve cozucu bunu kontrol eder.
"""

import argparse
import json
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def image_to_bgr(msg):
    """cv_bridge'siz donusum -- Nano konteynerinde bridge her zaman yok."""
    data = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc in ("bgr8", "rgb8"):
        img = data.reshape(msg.height, msg.width, 3)
        return img if enc == "bgr8" else cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if enc == "mono8":
        return cv2.cvtColor(data.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
    raise ValueError(f"desteklenmeyen encoding: {msg.encoding}")


def candidate_patterns(cols, rows):
    """Verilen desenin makul komsulari — ic kose / kare karisikligi icin.

    En sik hata: tahtadaki KARE sayisini vermek. n kareli kenarin ic kosesi
    n-1'dir, yani 7x9 kare -> 6x8 ic kose. Devrik de denenir cunku tahtanin
    hangi kenarinin 'cols' oldugu bakis acisina gore degisir.
    """
    seen, out = set(), []
    for c, r in ((cols, rows), (rows, cols),
                 (cols - 1, rows - 1), (rows - 1, cols - 1),
                 (cols + 1, rows + 1), (rows + 1, cols + 1)):
        if c >= 3 and r >= 3 and (c, r) not in seen:
            seen.add((c, r))
            out.append((c, r))
    return out



# 3x3 kapsama TEK BASINA yeterli bir olcut degil (2026-08-18 olcumu). Ayni
# veriden uretilen kalibrasyonlar kabul kapisini gectikleri halde pozda
# 0.54 derece ayrildi, algi tabaninin 9.9 kati; fx %6.6 oynadi. Sebep
# geometrik: odak uzakligi ile mesafe birbirinin yerine gecebilir ve bunu
# ayiran sey farkli MESAFELERDEN ve GUCLU EGIMLERDEN gelen goruntulerdir.
# Kapsama haritasi bunlarin ikisini de olcmuyordu.
#
# Asagidaki iki olcu de INTRINSICS GEREKTIRMEZ, cunku yakalama sirasinda
# henuz intrinsics yoktur:
#   * olcek  -> komsu ic koseler arasi medyan piksel araligi. Mesafeyle ters
#               orantili: yakinlasinca buyur.
#   * egim   -> on-paralel bir tahtanin kaplayacagi alana gore kisalma orani.
#               foreshortening = alan / (aralik^2 * (cols-1) * (rows-1)),
#               on-paralelde 1.0'a yakin, egildikce kucur. Aci karsiligi
#               arccos(oran)'dir; kalibre bir aci DEGIL, vekil bir olcudur.
DISTANCE_BIN_EDGES_PX = (18.0, 30.0)
TILT_BIN_EDGES_DEG = (15.0, 35.0)


def board_geometry(points, pattern):
    """Return intrinsics-free scale and tilt proxies for one board view."""
    cols, rows = pattern
    if cols < 2 or rows < 2:
        raise ValueError('desen en az 2x2 ic kose olmali')
    grid = np.asarray(points, dtype=np.float64).reshape(rows, cols, 2)
    horizontal = np.linalg.norm(grid[:, 1:] - grid[:, :-1], axis=2)
    vertical = np.linalg.norm(grid[1:] - grid[:-1], axis=2)
    spacing = float(np.median(np.concatenate([
        horizontal.ravel(), vertical.ravel()])))
    hull = cv2.contourArea(
        cv2.convexHull(grid.reshape(-1, 2).astype(np.float32)))
    fronto_area = spacing * spacing * (cols - 1) * (rows - 1)
    ratio = float(hull / fronto_area) if fronto_area > 0.0 else 0.0
    tilt = float(np.degrees(np.arccos(np.clip(ratio, 0.0, 1.0))))
    return {
        'spacing_px': spacing,
        'hull_area_px': float(hull),
        'foreshortening': ratio,
        'tilt_deg': tilt,
    }


def bin_index(value, edges):
    """Return the bin index of value for ascending edges."""
    index = 0
    for edge in edges:
        if value >= edge:
            index += 1
    return index


def diversity_complete(distance_bins, tilt_bins):
    """Diversity is met with every distance bin and a strongly tilted view.

    Distance separates focal length from range, so all three bins are needed.
    Tilt is what breaks the near-affine degeneracy of a fronto-parallel view;
    one strongly tilted bin is the part that actually carries information, so
    the highest bin must be non-empty rather than merely two of three.
    """
    return (len(distance_bins) == len(DISTANCE_BIN_EDGES_PX) + 1
            and len(TILT_BIN_EDGES_DEG) in tilt_bins)


def collection_complete(
        count, target, covered_cells, distance_bins=None,
        tilt_bins=None) -> bool:
    """A fit run needs count, image coverage AND geometric diversity.

    Coverage alone was the gate until 2026-08-18 and it let through a set that
    left focal length unconstrained. Callers that pass the bin sets get the
    stronger gate; passing None keeps the old behaviour for tests and tools
    that do not track geometry.
    """
    if count < target or len(covered_cells) != 9:
        return False
    if distance_bins is None or tilt_bins is None:
        return True
    return diversity_complete(distance_bins, tilt_bins)


def cells_covered_by_points(points, width, height):
    """Return every 3x3 image cell containing an observed board corner."""
    if width <= 0 or height <= 0:
        raise ValueError('image dimensions must be positive')
    cells = set()
    for x, y in np.asarray(points).reshape(-1, 2):
        column = min(max(int(x / width * 3), 0), 2)
        row = min(max(int(y / height * 3), 0), 2)
        cells.add((column, row))
    return cells


class Collector(Node):
    def __init__(self, args):
        super().__init__("robot_arm_calib_collector")
        self.args = args
        self.pattern = (args.cols, args.rows)
        self.accepted = []          # (kose merkezi, alan) listesi
        self.count = 0
        self.seen = 0
        self.shape = None
        self.cells = set()
        self.pattern_confirmed = False
        self.probe_done = False
        os.makedirs(args.out, exist_ok=True)
        self.create_subscription(
            Image, args.topic, self._on_image, qos_profile_sensor_data)
        # Uzerine isaretlenmis geri bildirim yayini: ham goruntuye bakmak
        # tahtanin TANINIP taninmadigini soylemez. Operator PC'den bunu izler.
        self.overlay_pub = self.create_publisher(Image, args.overlay_topic, 1)
        self.flash = 0
        self.last_detect = 0.0
        self.last_overlay = 0.0
        self.last_found = False
        self.last_corners = None
        # Kapsama disindaki iki cesitlilik ekseni; bkz. board_geometry.
        self.distance_bins = set()
        self.tilt_bins = set()
        self.geometry = []

    def _probe_pattern(self, gray):
        """Verilen desen tutmuyorsa komsulari dene ve BULDUGUNU soyle."""
        for cand in candidate_patterns(*self.pattern):
            if cand == self.pattern:
                continue
            found, _ = cv2.findChessboardCorners(
                gray, cand,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
            if found:
                print(f"\n⚠ {self.pattern[0]}x{self.pattern[1]} ic kose"
                      f" bulunamadi, ama {cand[0]}x{cand[1]} BULUNDU.")
                print("   Buyuk ihtimalle KARE sayisi verildi; OpenCV IC KOSE")
                print(f"   bekler. Desen {cand[0]}x{cand[1]} olarak devam ediyor.")
                print("   Yanlissa Ctrl+C ile cik ve --cols/--rows duzelt.\n")
                self.pattern = cand
                return True
        print(f"\n⚠ {self.seen} karede hicbir desen bulunamadi"
              f" ({', '.join(f'{c}x{r}' for c, r in candidate_patterns(*self.pattern))}"
              " denendi).")
        print("   Tahtanin tamami goruntude mi, isik yeterli mi, odak tamam mi?\n")
        return False

    def _publish_overlay(self, bgr, corners, found, stamp):
        """Tanima durumunu ve kapsama haritasini goruntunun uzerine ciz."""
        vis = bgr.copy()
        h, w = vis.shape[:2]

        # 3x3 kapsama izgarasi. Once yarisaydam dolgu kullaniliyordu; iki tam
        # goruntu kopyasi + addWeighted Nano'da pahaliya geliyordu. Ayni bilgi
        # kalin yesil cerceve ile veriliyor, maliyet neredeyse sifir.
        for cy in range(3):
            for cx in range(3):
                x0, y0 = int(cx * w / 3), int(cy * h / 3)
                x1, y1 = int((cx + 1) * w / 3), int((cy + 1) * h / 3)
                if (cx, cy) in self.cells:
                    cv2.rectangle(vis, (x0 + 2, y0 + 2), (x1 - 2, y1 - 2),
                                  (0, 180, 0), 3)
                else:
                    cv2.rectangle(vis, (x0, y0), (x1, y1), (70, 70, 70), 1)

        if found and corners is not None:
            cv2.drawChessboardCorners(vis, self.pattern, corners, True)

        state = "TAHTA BULUNDU" if found else "tahta yok"
        color = (0, 220, 0) if found else (0, 0, 220)
        cv2.rectangle(vis, (0, 0), (w, 26), (0, 0, 0), -1)
        cv2.putText(vis, f"{self.count}/{self.args.target}  {state}"
                    f"  desen {self.pattern[0]}x{self.pattern[1]}"
                    f"  bolge {len(self.cells)}/9",
                    (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1,
                    cv2.LINE_AA)
        if self.flash > 0:
            cv2.rectangle(vis, (1, 1), (w - 2, h - 2), (0, 255, 255), 4)
            self.flash -= 1

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera"
        msg.height, msg.width = h, w
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = w * 3
        msg.data = vis.tobytes()
        self.overlay_pub.publish(msg)

    def _on_image(self, msg):
        if collection_complete(self.count, self.args.target, self.cells,
                               self.distance_bins, self.tilt_bins):
            return
        try:
            bgr = image_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        self.seen += 1
        now = time.time()

        # GORUNTU AKISI TESPITTEN AYRI. Tespit Nano'da pahali (yari
        # cozunurlukte bile ~0.4 s/kare, %100 CPU); her karede kosarsa
        # operatorun gordugu goruntu 0.8 Hz'lik bir slayt gosterisine
        # donuyor. Tespit --detect-hz ile sinirlanir, overlay ise her zaman
        # EN SON kareyi gosterir, boylece video akici kalir.
        overlay_due = (now - self.last_overlay) >= 1.0 / self.args.overlay_hz
        detect_due = (now - self.last_detect) >= 1.0 / self.args.detect_hz
        if not detect_due:
            if overlay_due:
                self.last_overlay = now
                fresh = (now - self.last_detect) < 0.6
                self._publish_overlay(
                    bgr, self.last_corners if fresh else None,
                    self.last_found and fresh, msg.header.stamp)
            return
        self.last_detect = now

        if self.shape is None:
            self.shape = (msg.width, msg.height)
            print(f"goruntu boyutu: {msg.width}x{msg.height} ({msg.encoding})")
        elif self.shape != (msg.width, msg.height):
            self.get_logger().error(
                "goruntu boyutu KOSU ORTASINDA degisti — kalibrasyon gecersiz")
            return

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        # ARAMA YARI COZUNURLUKTE. Tam cozunurlukte aramak Jetson Nano'da
        # olculdu: kare basina ~0.93 s, %111 CPU, overlay 0.35 Hz —
        # kullanilamaz. FAST_CHECK tek basina yetmiyor cunku kalabalik bir
        # sahne elemeyi geciyor ve pahali yol yine her karede kosuyor.
        # Yari cozunurlukte dortte bir piksel var; bulunan koseler 2x
        # olceklenip cornerSubPix ile TAM cozunurlukte hassaslastiriliyor,
        # yani alt-piksel dogrulugu kaybedilmiyor.
        sc = self.args.detect_scale
        small = cv2.resize(gray, None, fx=sc, fy=sc,
                           interpolation=cv2.INTER_AREA)
        found, corners_small = cv2.findChessboardCorners(
            small, self.pattern,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE +
            cv2.CALIB_CB_FAST_CHECK)
        # cornerSubPix float32 ister; numpy sürümüne göre skaler carpim
        # float64'e yukseltebilir, o yuzden acikca donusturuluyor.
        corners = ((corners_small / sc).astype(np.float32)
                   if found else None)
        self.last_found = found
        self.last_corners = None
        if not found:
            # Bir suredir hic bulamadiysak once DESENI sorgula: yanlis sayiyla
            # saatlerce tahta sallamak bu araclarin klasik zaman kaybi.
            if not self.pattern_confirmed and not self.probe_done and \
                    self.seen >= self.args.probe_frames:
                self.probe_done = True
                # Ana aramayla AYNI olcekte probe et: tam cozunurlukte bulunup
                # yari cozunurlukte bulunamayan bir desen yaniltici olurdu.
                if self._probe_pattern(small):
                    self.pattern_confirmed = True
            self.last_overlay = now
            self._publish_overlay(bgr, None, False, msg.header.stamp)
            return
        self.pattern_confirmed = True

        corners = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        pts = corners.reshape(-1, 2)
        center = pts.mean(axis=0)
        area = cv2.contourArea(cv2.convexHull(pts.astype(np.float32)))

        # Yeterince farkli mi? Ayni pozdan tekrar toplamak bilgi eklemez.
        for prev_c, prev_a in self.accepted:
            moved = np.linalg.norm(center - prev_c)
            scaled = abs(area - prev_a) / max(prev_a, 1.0)
            if moved < self.args.min_shift_px and scaled < self.args.min_area_change:
                # Taniniyor ama zaten elimizde benzeri var: operator gorsun ki
                # tahtayi tasimasi gerektigini anlasin.
                self.last_corners = corners
                self.last_overlay = now
                self._publish_overlay(bgr, corners, True, msg.header.stamp)
                return

        name = os.path.join(self.args.out, f"calib_{self.count:03d}.png")
        cv2.imwrite(name, bgr)
        self.accepted.append((center, area))
        self.cells.update(cells_covered_by_points(pts, msg.width, msg.height))
        geometry = board_geometry(pts, self.pattern)
        self.distance_bins.add(
            bin_index(geometry['spacing_px'], DISTANCE_BIN_EDGES_PX))
        self.tilt_bins.add(bin_index(geometry['tilt_deg'], TILT_BIN_EDGES_DEG))
        self.geometry.append(geometry)
        self.count += 1
        self.flash = 5
        self.last_corners = corners
        self.last_overlay = now
        self._publish_overlay(bgr, corners, True, msg.header.stamp)

        missing = [f"({r},{c})" for r in range(3) for c in range(3)
                   if (c, r) not in self.cells]
        print(f"[{self.count}/{self.args.target}] kaydedildi {os.path.basename(name)}"
              f"  merkez=({center[0]:.0f},{center[1]:.0f})")
        if missing:
            print(f"    hala bos bolge: {', '.join(missing)}  (satir,sutun)")
        else:
            print("    9 bolgenin hepsinden veri var ✓")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cols", type=int, default=9,
                    help="yatay IC KOSE (kare degil: 9 kare -> 8 ic kose)")
    ap.add_argument("--rows", type=int, default=6,
                    help="dikey IC KOSE (kare degil: 7 kare -> 6 ic kose)")
    ap.add_argument("--square-mm", type=float, required=True,
                    help="basili tahtada cetvelle olculen kare kenari")
    ap.add_argument("--flip-method", type=int, choices=range(4), required=True,
                    help="kamera node'unda kilitlenen nvvidconv flip-method")
    # Jetson Nano'da OLCULDU (2026-07-22, tek kare, 3 tekrar): maliyet
    # bayraklardan bagimsiz, tamamen cozunurluge bagli --
    #   640x480 717ms | 320x240 170ms | 160x120 53ms
    # (FAST_CHECK bu sahnede hicbir sey kazandirmadi.) 0.5 olcek + 2 Hz
    # tespit ~%34 CPU demek; geri kalan cekirdek overlay'e kaliyor.
    ap.add_argument("--detect-hz", type=float, default=2.0,
                    help="saniyede kac kez tahta aransin")
    ap.add_argument("--detect-scale", type=float, default=0.5,
                    help="tespit olcegi; dusurmek hizlandirir ama kucuk/uzak "
                         "tahtayi kacirabilir (0.5 -> 170ms, 0.25 -> 53ms)")
    ap.add_argument("--overlay-hz", type=float, default=8.0,
                    help="geri bildirim goruntusunun yayin hizi")
    ap.add_argument("--probe-frames", type=int, default=40,
                    help="bu kadar karede desen bulunamazsa alternatifleri dene")
    ap.add_argument("--topic", default="/camera/image_raw")
    ap.add_argument("--overlay-topic", default="/camera/calib_overlay",
                    help="tanima geri bildirimi yayini (PC'den izlenir)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--target", type=int, default=25)
    ap.add_argument("--min-shift-px", type=float, default=40.0)
    ap.add_argument("--min-area-change", type=float, default=0.15)
    args = ap.parse_args()

    if args.square_mm <= 0.0:
        ap.error("--square-mm sifirdan buyuk olmali")
    if args.detect_hz <= 0.0 or args.overlay_hz <= 0.0:
        ap.error("--detect-hz ve --overlay-hz sifirdan buyuk olmali")
    if not 0.0 < args.detect_scale <= 1.0:
        ap.error("--detect-scale 0 < scale <= 1 araliginda olmali")

    print(__doc__)
    print("Tahtayi yavas gezdir: merkez, dort kose, dort kenar; her konumda")
    print("hafif egerek. Egim OLMADAN toplanan veri odak uzakligi ile")
    print("distorsiyonu birbirinden ayiramaz.\n")

    rclpy.init()
    node = Collector(args)
    try:
        while rclpy.ok() and not collection_complete(
                node.count, args.target, node.cells):
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        print("\nkesildi")
    finally:
        meta = {
            # Desen otomatik duzeltilmis olabilir; cozucunun GERCEKTEN
            # kullanilani gormesi sart, argumani degil.
            "pattern_cols": node.pattern[0], "pattern_rows": node.pattern[1],
            "pattern_requested": [args.cols, args.rows],
            "square_size_mm": args.square_mm,
            "flip_method": args.flip_method,
            "image_topic": args.topic, "accepted": node.count,
            "frames_seen": node.seen,
            "image_width": node.shape[0] if node.shape else None,
            "image_height": node.shape[1] if node.shape else None,
            "covered_cells": sorted(f"{r},{c}" for (c, r) in node.cells),
            # Kapsama TEK BASINA yeterli degil: mesafe ve egim cesitliligi
            # olmadan odak uzakligi kisitlanmiyor (2026-08-18).
            "distance_bin_edges_px": list(DISTANCE_BIN_EDGES_PX),
            "tilt_bin_edges_deg": list(TILT_BIN_EDGES_DEG),
            "distance_bins_covered": sorted(node.distance_bins),
            "tilt_bins_covered": sorted(node.tilt_bins),
            "diversity_complete": diversity_complete(
                node.distance_bins, node.tilt_bins),
            "per_frame_geometry": node.geometry,
        }
        with open(os.path.join(args.out, "capture_meta.json"), "w") as f:
            json.dump(meta, f, indent=2)
        print(f"\n{node.count} kare kaydedildi -> {args.out}")
        if len(node.cells) < 9:
            print(f"UYARI: goruntunun {9 - len(node.cells)} bolgesinden hic veri")
            print("yok. Distorsiyon kestirimi o bolgelerde EKSTRAPOLASYON olur.")
        if node.count < 10:
            print("UYARI: 10'dan az kare — kalibrasyon guvenilir olmaz.")
        node.destroy_node()
        # Kesilme sirasinda context zaten kapanmis olabilir; ikinci kapatma
        # RCLError firlatir ve gercek sonucu gizler.
        try:
            rclpy.shutdown()
        except Exception:
            pass
        sys.exit(0 if collection_complete(node.count, args.target, node.cells) else 1)


if __name__ == "__main__":
    main()
