#!/usr/bin/env python3
"""Vizyon-encoder PnP çekirdeğini GERÇEK kalibrasyon kareleri üzerinde offline koş.

Kamera Nano'ya bağlı değilken bile çekirdeği doğrular: intrinsics kalibrasyonunda
çekilmiş gerçek checkerboard fotoğrafları (bu kamera, bu board) üzerinde
tespit→solvePnP→reprojection koşar ve doğruluğu raporlar. Reprojection hatası
intrinsics RMS'ine (~0.2px) yakınsa boru hattı sağlıklı demektir.

DİKKAT — geometri varsayılanları GÖRÜNTÜ KÜMESİNE bağlıdır, deployment'a değil.
Varsayılan `--images` 2026-07-22 intrinsics kalibrasyon kareleridir ve o gün
kullanılan tahta **6x8 iç köşe / 25 mm**'dir; varsayılanlar bu yüzden öyledir ve
o küme üzerinde 25/25 tespit verir. Kanonik *deployment* tahtası ise
**6x9 / 27.5 mm** (`src/arm_perception/config/vision_encoder.yaml`) — bu betiğin
varsayılanlarını ona çekmek varsayılan koşuyu 0/25'e düşürür (ölçüldü
2026-08-12). Yeni tahtayla çekilmiş karelere bakarken geometriyi elle ver.

Kullanım:
    # tarihsel küme (varsayılan geometri doğrudur)
    python3 scripts/run_board_pnp_offline.py

    # güncel tahtayla çekilmiş kareler
    python3 scripts/run_board_pnp_offline.py \
        --images <dizin> --cols 6 --rows 9 --square-mm 27.5
"""
import argparse
import glob
import os

import cv2
import numpy as np
import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Bu üçlü, DEFAULT_IMAGES'ın çekildiği günün tahtasıdır — deployment tahtası değil.
HISTORICAL_IMAGES = os.path.join(REPO_ROOT, "runs", "calib_fit_20260722_174459")
HISTORICAL_BOARD = (6, 8, 25.0)
import sys
sys.path.insert(0, os.path.join(REPO_ROOT, "src", "arm_perception"))
from arm_perception.board_pnp import estimate_board_pose  # noqa: E402


def load_intrinsics(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    K = np.array(data["camera_matrix"]["data"], np.float64).reshape(3, 3)
    dist = np.array(data["distortion_coefficients"]["data"], np.float64).reshape(1, -1)
    return K, dist, int(data["image_width"]), int(data["image_height"])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--images", default=HISTORICAL_IMAGES)
    ap.add_argument("--intrinsics", default=os.path.join(
        REPO_ROOT, "src", "arm_perception", "config", "imx219_640x480.yaml"))
    ap.add_argument("--cols", type=int, default=HISTORICAL_BOARD[0])
    ap.add_argument("--rows", type=int, default=HISTORICAL_BOARD[1])
    ap.add_argument("--square-mm", type=float, default=HISTORICAL_BOARD[2])
    args = ap.parse_args()

    # Başka bir kümeye tarihsel geometriyle bakmak sessizce 0% tespit verir ve
    # "çekirdek bozuk" gibi okunur. Tespit denemeden önce söyle.
    board = (args.cols, args.rows, args.square_mm)
    if os.path.abspath(args.images) != HISTORICAL_IMAGES and board == HISTORICAL_BOARD:
        print(f"UYARI: --images tarihsel kümenin dışında ama geometri hâlâ "
              f"{board[0]}x{board[1]} @ {board[2]}mm (2026-07-22 tahtası). "
              f"Güncel tahta 6x9 @ 27.5mm ise --cols/--rows/--square-mm ver, "
              f"yoksa tespit 0% çıkar ve bu çekirdek arızası DEĞİLDİR.\n")

    K, dist, w, h = load_intrinsics(args.intrinsics)
    print(f"intrinsics: {w}x{h}  fx={K[0,0]:.1f} fy={K[1,1]:.1f} "
          f"cx={K[0,2]:.1f} cy={K[1,2]:.1f}")
    print(f"board: {args.cols}x{args.rows} iç köşe, {args.square_mm}mm kare\n")

    paths = sorted(glob.glob(os.path.join(args.images, "*.png")))
    if not paths:
        raise SystemExit(f"görüntü yok: {args.images}")

    reprojs, dists, detected = [], [], 0
    for p in paths:
        img = cv2.imread(p)
        if img is None:
            continue
        if (img.shape[1], img.shape[0]) != (w, h):
            print(f"  ATLA {os.path.basename(p)}: {img.shape[1]}x{img.shape[0]} "
                  f"≠ intrinsics {w}x{h}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        res = estimate_board_pose(gray, args.cols, args.rows, args.square_mm, K, dist)
        name = os.path.basename(p)
        if res is None:
            print(f"  {name}: board YOK (tespit başarısız)")
            continue
        detected += 1
        reprojs.append(res["reproj_px"])
        dists.append(res["distance_m"])
        print(f"  {name}: reproj={res['reproj_px']:.3f}px  "
              f"mesafe={res['distance_m']*100:.1f}cm  köşe={res['n_corners']}")

    n = len(paths)
    print(f"\n=== ÖZET ===")
    print(f"kare: {n}  tespit: {detected} ({100*detected/max(n,1):.0f}%)")
    if reprojs:
        r = np.array(reprojs)
        d = np.array(dists)
        print(f"reprojection px: ort {r.mean():.3f}  medyan {np.median(r):.3f}  "
              f"maks {r.max():.3f}")
        print(f"board mesafesi : {d.min()*100:.1f}–{d.max()*100:.1f} cm")
        print(f"YORUM: reproj ort intrinsics RMS'ine (~0.19px) yakınsa PnP "
              f"çekirdeği SAĞLIKLI.")


if __name__ == "__main__":
    main()
