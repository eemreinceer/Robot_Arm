#!/usr/bin/env python3
"""Kalibrasyon icin BASILABILIR satranc tahtasi deseni uretir (SVG).

Neden SVG, PNG degil: SVG milimetre biriminde cizilir, yazici olceklemesi
karismaz. PNG'de DPI yorumu yaziciya gore degisir ve kare boyutu sessizce
kayar; kare boyutu kalibrasyonun tek fiziksel referansidir.

Kullanim:
    python3 make_checkerboard.py --cols 9 --rows 6 --square-mm 25 \
        --out reports/checkerboard_9x6_25mm.svg

Basim ve kullanim notlari (kalibrasyonun dogrulugu bunlara bagli):
  * "Olcekle/sayfaya sigdir" KAPALI, %100 olcekte bas.
  * Bastiktan sonra bir kareyi cetvelle OLC ve --square-mm ile ayni oldugunu
    dogrula; degilse gercek olcuyu kullan.
  * Sert ve DUZ bir yuzeye yapistir. Bukuk tahta kalibrasyonu bozar ve bunu
    reprojection hatasindan anlamak zordur.
  * --cols/--rows IC KOSE sayisidir (kare sayisi degil). 9x6 ic kose =
    10x7 kare. OpenCV findChessboardCorners ic koseyi bekler.
"""

import argparse
import os

TEMPLATE_HEAD = (
    '<?xml version="1.0" encoding="UTF-8"?>\n'
    '<svg xmlns="http://www.w3.org/2000/svg" version="1.1"\n'
    '     width="{w}mm" height="{h}mm" viewBox="0 0 {w} {h}">\n'
    '  <rect x="0" y="0" width="{w}" height="{h}" fill="white"/>\n')

TEMPLATE_TAIL = (
    '  <text x="{tx}" y="{ty}" font-family="sans-serif" font-size="4"'
    ' fill="black">{label}</text>\n'
    '</svg>\n')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cols", type=int, default=9,
                    help="yatay IC KOSE sayisi (vars. 9)")
    ap.add_argument("--rows", type=int, default=6,
                    help="dikey IC KOSE sayisi (vars. 6)")
    ap.add_argument("--square-mm", type=float, default=25.0)
    ap.add_argument("--margin-mm", type=float, default=10.0)
    ap.add_argument("--out", default="reports/checkerboard.svg")
    args = ap.parse_args()

    # Ic kose sayisi n ise kare sayisi n+1'dir.
    nx, ny = args.cols + 1, args.rows + 1
    s, m = args.square_mm, args.margin_mm
    w = nx * s + 2 * m
    h = ny * s + 2 * m

    parts = [TEMPLATE_HEAD.format(w=round(w, 3), h=round(h, 3))]
    for iy in range(ny):
        for ix in range(nx):
            if (ix + iy) % 2:
                continue
            parts.append(
                f'  <rect x="{round(m + ix * s, 3)}" y="{round(m + iy * s, 3)}"'
                f' width="{s}" height="{s}" fill="black"/>\n')

    label = (f"{args.cols}x{args.rows} ic kose, kare {args.square_mm:g} mm "
             "- %100 olcekte bas, sonra bir kareyi cetvelle dogrula")
    parts.append(TEMPLATE_TAIL.format(
        tx=round(m, 3), ty=round(h - m / 2, 3), label=label))

    out_dir = os.path.dirname(os.path.abspath(args.out))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.out, "w") as f:
        f.write("".join(parts))

    print(f"{args.out} yazildi")
    print(f"  ic kose : {args.cols} x {args.rows}")
    print(f"  kare    : {args.square_mm:g} mm")
    if w <= 210 and h <= 297:
        fit = "A4 dikey sigar"
    elif w <= 297 and h <= 210:
        fit = "A4 YATAY sigar (yaziciyi yatay yap)"
    else:
        fit = "A4'e SIGMAZ - kare boyutunu veya kose sayisini kucult"
    print(f"  sayfa   : {w:.1f} x {h:.1f} mm ({fit})")
    print("\nBasarken 'sayfaya sigdir' KAPALI olsun; bastiktan sonra bir kareyi")
    print("cetvelle olc. Olcu farkliysa kalibrasyonda GERCEK olcuyu gir.")


if __name__ == "__main__":
    main()
