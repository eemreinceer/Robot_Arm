#!/usr/bin/env python3
"""Robot Arm vizyon-encoder — yazdırılabilir ChArUco board + tek-kaynak geometri üret.

NEDEN ChArUco: eye-in-hand kamera SABİT bir referans board'a bakıp solvePnP ile
kendi (=TCP) pozunu çıkaracak — "sanal encoder"ın çekirdeği bu. ChArUco,
checkerboard alt-piksel köşe doğruluğunu ArUco ID kimliğiyle birleştirir; düzlemsel
bir hedeften elde edilebilecek EN doğru/tekrarlanabilir PnP pozunu verir ve kısmi
occlusion'a dayanır (yeterli köşe görünürse çözer).

ÇIKTI iki dosya:
  * PNG — 300 DPI, %100 ölçekte basılacak yazdırılabilir board.
  * YAML — board geometrisi (dict, kare/marker mm). Board üreteci VE
    vision_encoder_node AYNI bu dosyayı okur; iki yer ayrışırsa PnP ölçeği
    sessizce bozulur (pozlar mm cinsinden yanlış çıkar, residual normal görünür).

⚠️ BASKI UYARISI: metrik poz doğrudan basılı kare boyutuna bağlı. Yazıcıda
"gerçek boyut / %100 / ölçekleme YOK" seç; bastıktan sonra bir kareyi cetvelle
ölç, square_len_mm ile birebir tutmuyorsa YAML'daki değeri ölçülen değere çek —
tahmin etme, ölç (bu projenin kuralı).

Kullanım:
    python3 scripts/make_charuco_board.py [--squares-x 5] [--squares-y 7] \
        [--square-mm 30] [--marker-mm 23] [--dict DICT_5X5_100] [--dpi 300] \
        [--out-dir runs]
"""
import argparse
import os

import cv2
import numpy as np
import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def build_dictionary(name):
    """DICT_* adını yeni-API sözlüğüne çevir; yoksa anlaşılır hata ver."""
    attr = getattr(cv2.aruco, name, None)
    if attr is None:
        raise SystemExit(f"bilinmeyen aruco dict: {name}")
    return cv2.aruco.getPredefinedDictionary(attr)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--squares-x", type=int, default=5)
    ap.add_argument("--squares-y", type=int, default=7)
    ap.add_argument("--square-mm", type=float, default=30.0)
    ap.add_argument("--marker-mm", type=float, default=23.0)
    ap.add_argument("--dict", default="DICT_5X5_100")
    ap.add_argument("--dpi", type=int, default=300)
    ap.add_argument("--margin-mm", type=float, default=10.0)
    ap.add_argument("--out-dir", default=os.path.join(REPO_ROOT, "runs"))
    args = ap.parse_args()

    if args.marker_mm >= args.square_mm:
        raise SystemExit("marker_mm < square_mm olmalı (marker kareye sığar)")

    dictionary = build_dictionary(args.dict)
    # Yeni API: CharucoBoard((cols,rows), squareLen, markerLen, dict).
    # Uzunluk birimi metre (poz da metre çıksın diye); üretim/piksel ölçeği
    # ayrıca DPI'dan hesaplanır, bu uzunluk sadece poz için saklanır.
    board = cv2.aruco.CharucoBoard(
        (args.squares_x, args.squares_y),
        args.square_mm / 1000.0, args.marker_mm / 1000.0, dictionary)

    px_per_mm = args.dpi / 25.4
    board_w_mm = args.squares_x * args.square_mm
    board_h_mm = args.squares_y * args.square_mm
    margin_px = int(round(args.margin_mm * px_per_mm))
    img_w = int(round(board_w_mm * px_per_mm)) + 2 * margin_px
    img_h = int(round(board_h_mm * px_per_mm)) + 2 * margin_px

    image = board.generateImage((img_w, img_h), marginSize=margin_px, borderBits=1)

    os.makedirs(args.out_dir, exist_ok=True)
    tag = f"charuco_{args.squares_x}x{args.squares_y}_{int(args.square_mm)}mm"
    png_path = os.path.join(args.out_dir, tag + ".png")
    yaml_path = os.path.join(args.out_dir, tag + ".yaml")
    cv2.imwrite(png_path, image)

    params = {
        "board_type": "charuco",
        "dictionary": args.dict,
        "squares_x": args.squares_x,
        "squares_y": args.squares_y,
        "square_len_mm": args.square_mm,
        "marker_len_mm": args.marker_mm,
        "board_width_mm": round(board_w_mm, 2),
        "board_height_mm": round(board_h_mm, 2),
        "n_charuco_corners": (args.squares_x - 1) * (args.squares_y - 1),
        "dpi": args.dpi,
        "note": "vision_encoder_node bu dosyayı okur. Baskı %100; kareyi cetvelle "
                "doğrula, sapıyorsa square_len_mm'i ÖLÇÜLEN değere çek.",
    }
    with open(yaml_path, "w") as f:
        yaml.safe_dump(params, f, sort_keys=False, allow_unicode=True)

    print(f"board PNG : {png_path}  ({img_w}x{img_h}px @ {args.dpi}dpi)")
    print(f"params    : {yaml_path}")
    print(f"fiziksel  : {board_w_mm:.0f} x {board_h_mm:.0f} mm "
          f"({args.squares_x}x{args.squares_y} kare, {args.square_mm}mm)")
    print(f"iç köşe   : {params['n_charuco_corners']} adet (PnP için)")
    print("BASKI: %100 ölçek, sonra kareyi cetvelle ölç → square_len_mm doğrula.")


if __name__ == "__main__":
    main()
