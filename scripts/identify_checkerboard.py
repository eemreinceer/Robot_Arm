#!/usr/bin/env python3
"""Kameradaki satranc tahtasinin ic kose sayisini OLCEREK bulur.

Neden gerekli: "kac kare var" sorusu goz karariyla ya da fotograftan sayarak
cevaplaniyor ve yanlis cevap butun kalibrasyon oturumunu sessizce yakiyor --
yanlis desende findChessboardCorners hicbir zaman eslesmez, arac da sessiz
kalir. Bu arac tahtayi kameraya gosterip TUM makul desenleri dener.

Onemli ayrinti: kucuk bir desen buyuk tahtanin ICINDE de bulunabilir (6x8'lik
bir tahtada 5x7 de vardir). Bu yuzden tum eslesmeler raporlanir ve EN BUYUGU
onerilir.

Kullanim (PC'deki humble konteynerinde, tahtayi kameraya gosterirken):
    python3 identify_checkerboard.py
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def image_to_gray(msg):
    data = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc in ("bgr8", "rgb8"):
        img = data.reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if enc == "mono8":
        return data.reshape(msg.height, msg.width)
    raise ValueError(f"desteklenmeyen encoding: {msg.encoding}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--topic", default="/camera/image_raw")
    ap.add_argument("--min", type=int, default=3)
    ap.add_argument("--max", type=int, default=11)
    ap.add_argument("--frames", type=int, default=3,
                    help="kac karede denensin (gurultuye karsi)")
    # Varsayilan REPO ICINDE: arac konteynerde kosuyor, /tmp oraya hapsolur ve
    # operator dosyayi host'tan acamaz. Repo konteynere KENDI host yoluna
    # mount edildigi icin buraya yazilan dosya iki taraftan da gorunur.
    # (`*.png` gitignore'da, repoyu kirletmez.)
    ap.add_argument("--save-frame",
                    default=os.path.join(REPO_ROOT, "runs",
                                         "identify_checkerboard.png"),
                    help="gorulen ilk kare buraya yazilir")
    args = ap.parse_args()

    print(__doc__)
    print("Tahtayi kameraya, TAMAMI kadraja girecek sekilde tut.\n")

    rclpy.init()
    node = Node("robot_arm_board_identifier")
    frames = []
    node.create_subscription(
        Image, args.topic,
        lambda m: frames.append(m) if len(frames) < args.frames else None,
        qos_profile_sensor_data)

    deadline = time.time() + 20
    while rclpy.ok() and len(frames) < args.frames:
        if time.time() > deadline:
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(f"{args.topic} uzerinden goruntu gelmedi")
        rclpy.spin_once(node, timeout_sec=0.2)

    grays = [image_to_gray(m) for m in frames]
    print(f"{len(grays)} kare alindi ({grays[0].shape[1]}x{grays[0].shape[0]})")

    # Arac pencere ACMAZ, yani operator tahtayi kadraja alip almadigini
    # goremez. Gorulen kareyi diske yaz: "bulunamadi" ciktisinda sebebin
    # tahta mi, cerceveleme mi, isik mi oldugu buradan anlasilir.
    saved = False
    if args.save_frame:
        try:
            os.makedirs(os.path.dirname(args.save_frame) or ".", exist_ok=True)
            cv2.imwrite(args.save_frame, grays[0])
            saved = True
        except cv2.error as error:
            print(f"kare yazilamadi ({args.save_frame}): {error}")

    print("desenler taraniyor...\n")

    hits = {}
    for cols in range(args.min, args.max + 1):
        for rows in range(args.min, args.max + 1):
            if rows < cols:
                continue          # (c,r) ve (r,c) ayni tahtadir
            n = 0
            for gray in grays:
                found, _ = cv2.findChessboardCorners(
                    gray, (cols, rows),
                    cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
                n += int(found)
            if n:
                hits[(cols, rows)] = n

    if not hits:
        print("HICBIR desen bulunamadi.")
        if saved:
            print(f"  → KAMERANIN GORDUGU KARE: {args.save_frame}")
            print("    Once ona bak: tahta kadrajda miydi?")
        print("  - Tahtanin TAMAMI kadrajda mi?")
        print("  - Isik yeterli, parlama yok, odak tamam mi?")
        print("  - Tahta duz mu (bukuk karton eslesmeyi bozar)?")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    print(f"{'desen':<12}{'kac karede':>12}")
    for (c, r), n in sorted(hits.items(), key=lambda kv: kv[0][0] * kv[0][1]):
        print(f"{c}x{r:<10}{n:>8}/{len(grays)}")

    if saved:
        print(f"\ngorulen kare: {args.save_frame}")
    best = max(hits, key=lambda k: (k[0] * k[1], hits[k]))
    print(f"\nONERILEN: --cols {best[0]} --rows {best[1]}")
    print(f"  ic kose {best[0]}x{best[1]}  ->  tahtada {best[0] + 1}x{best[1] + 1} kare")
    if len(hits) > 1:
        print("  (kucuk desenler buyugun icinde de bulunur; en buyuk dogru olanidir)")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
