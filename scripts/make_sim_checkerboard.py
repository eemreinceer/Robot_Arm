#!/usr/bin/env python3
"""Sim için satranç tahtası modeli üretir (PNG doku + SDF).

`make_checkerboard.py`'den AYRI bir iştir: o basım için SVG üretir, bu ise
Gazebo'ya konacak bir model üretir. Ortak olan tek şey geometri; ikisi de
`board_pnp`'nin kanonik değerlerini kullanır.

NEDEN GEREKLİ: sim'de `camera_mount_joint` tam olarak bilinir (launch'tan
verilir), yani hand-eye zincirinin geri bulması gereken yer gerçeği vardır.
Ama zincirin girdisi tahta tespitidir ve sim dünyasında tahta yoktu.

ÖLÇEK KRİTİKTİR: PnP'nin ürettiği mesafe, karenin fiziksel boyutuna doğrudan
bağlıdır. SDF plane'i tam olarak `(kare sayısı × square_mm)` boyutunda çizilir;
doku da aynı ızgaraya oturur, böylece sim'deki fiziksel kare boyu
`--square-mm` ile birebir aynı olur.

SESSİZ KENAR KURALI: findChessboardCorners tahtanın etrafında açık bir kenar
ister; desen kenara dayanırsa dış köşeler bulunamaz. Bu yüzden doku bir kare
genişliğinde beyaz çerçeveyle üretilir ve plane o çerçeveyi de kapsar.

    python3 scripts/make_sim_checkerboard.py
"""

import argparse
import os

import cv2
import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Kanonik değerler board_pnp'den gelir; burada yeniden yazılmaz.
import sys  # noqa: E402
sys.path.insert(0, os.path.join(REPO_ROOT, "src", "arm_perception"))
from arm_perception.board_pnp import (  # noqa: E402
    DEFAULT_BOARD_COLS,
    DEFAULT_BOARD_ROWS,
    DEFAULT_SQUARE_SIZE_MM,
)

MODEL_SDF = """<?xml version="1.0"?>
<!-- ÜRETİLMİŞ DOSYA — scripts/make_sim_checkerboard.py. Elle düzenleme. -->
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <!-- ÇÖZÜLMÜŞ (2026-08-12): yüzey bir süre DÜZ SİYAH render edildi ve
           detektör hiç köşe bulamadı. Sebep aşağıdaki metalness'tı.
           Eleme sırası kayda değer, çünkü üçü de yanlış adaydı: <plane> yerine
           <box>, albedo_map'te model:// yerine göreli yol, sahneye ambient 0.7
           (diğer yüzeyler değişti, tahta siyah kaldı). gz bu süre boyunca HİÇ
           hata vermedi — model yüklendi, doku çözüldü, `-v 4` log'u temizdi.
           Hata mesajının yokluğu materyalin sağlam olduğunun kanıtı değildi.
           box, plane yerine bilinçli kaldı: plane'in UV'si yok, box'ta var.

           ⚠ AÇIK OLAN BAŞKA: tahta artık 54/54 köşe veriyor, AMA
           reprojection 14.53 px (gerçek
           tahtada ~0.2 px) ve PnP mesafesi 0.2964 m iken dünyada 0.50 m'ye
           konuldu. Yani doku ile fiziksel boyut arasındaki sözleşme henüz
           doğrulanmadı; bu rig ŞU HÂLİYLE yer gerçeği kıyaslaması yapamaz.

           ⚠ ÖLÇÜM TUZAĞI, sonraki kişi düşmesin: kayıtlı karede komşu köşe
           aralıkları yatay 40.78 px / dikey 22.94 px (oran 0.563) ölçüldü ve
           bu ilk bakışta "kareler gerilmiş" gibi duruyor. DEĞİL — tahta
           kameraya göre eğik durduğu için perspektif kısalması tek başına bu
           oranı üretebilir. Bu ölçüm doku gerilmesini perspektiften AYIRMIYOR.
           Ayırmanın yolu tahtayı kameraya tam dik (face-on) getirip ölçmek. -->
      <visual name="visual">
        <geometry>
          <box>
            <!-- x/y BİLEREK TAKAS: gz doku u eksenini kutunun Y'sine, v'yi
                 X'ine eşliyor. Doğal sırada (x=genişlik) yazılınca 9 karelik
                 doku ekseni 0.330 m'ye, 12 karelik eksen 0.2475 m'ye yayılıyor
                 → kareler 36.67 x 20.63 mm, oran 0.5625. Kayıtlı karede ölçülen
                 köşe aralığı oranı 0.563 idi; üç hanede eşleşiyor. Takas ile
                 her iki eksende de kare tam {square_note} mm olur. -->
            <size>{height_m:.6f} {width_m:.6f} {thickness_m:.6f}</size>
          </box>
        </geometry>
        <material>
          <!-- Klasik ışıksız-PBR fallback'i: ortam/IBL haritası olmayan bir
               sahnede metalness yüksek kalırsa yüzey albedo'dan BAĞIMSIZ siyah
               render edilir. Tahta mat kâğıttır; metalness 0, roughness 1. -->
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 1</specular>
          <pbr>
            <metal>
              <metalness>0.0</metalness>
              <roughness>1.0</roughness>
              <!-- Göreli yol model:// ile DENENDİ, ikisi de siyah verdi
                   (2026-08-12). Göreli hâli bırakıldı çünkü model dizinine
                   bağlı ve GZ_SIM_RESOURCE_PATH'e bağımlı değil — ama bunun
                   sorunu çözdüğü İDDİA EDİLMİYOR. Bkz. yukarıdaki açık sorun. -->
              <albedo_map>materials/textures/{texture}</albedo_map>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>
    {cols}x{rows} iç köşe, {square_mm} mm kare kalibrasyon tahtası (sim).
    Üretici: scripts/make_sim_checkerboard.py
  </description>
</model>
"""


def render_texture(cols, rows, px_per_square):
    """İç köşe sayısından deseni çizer, etrafına bir kare beyaz kenar koyar."""
    sq_x, sq_y = cols + 1, rows + 1          # iç köşe -> kare sayısı
    total_x, total_y = sq_x + 2, sq_y + 2    # + sessiz kenar (her yanda 1 kare)
    img = np.full((total_y * px_per_square, total_x * px_per_square),
                  255, np.uint8)
    for j in range(sq_y):
        for i in range(sq_x):
            if (i + j) % 2:
                continue
            y0 = (j + 1) * px_per_square
            x0 = (i + 1) * px_per_square
            img[y0:y0 + px_per_square, x0:x0 + px_per_square] = 0
    return img, total_x, total_y


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cols", type=int, default=DEFAULT_BOARD_COLS)
    ap.add_argument("--rows", type=int, default=DEFAULT_BOARD_ROWS)
    ap.add_argument("--square-mm", type=float, default=DEFAULT_SQUARE_SIZE_MM)
    ap.add_argument("--px-per-square", type=int, default=64)
    ap.add_argument("--thickness-mm", type=float, default=3.0,
                    help="tahta kalınlığı; doku UV'si için kutu gerekiyor")
    ap.add_argument("--models-dir", default=os.path.join(
        REPO_ROOT, "src", "robot_arm_description", "models"))
    args = ap.parse_args()

    name = (f"checkerboard_{args.cols}x{args.rows}_"
            f"{str(args.square_mm).replace('.', 'p')}mm")
    img, total_x, total_y = render_texture(
        args.cols, args.rows, args.px_per_square)

    # Plane, sessiz kenar dahil bütün dokuyu kapsar → doku ile fiziksel
    # ızgara birebir örtüşür ve kare boyu tam olarak --square-mm olur.
    width_m = total_x * args.square_mm / 1000.0
    height_m = total_y * args.square_mm / 1000.0

    model_dir = os.path.join(args.models_dir, name)
    tex_dir = os.path.join(model_dir, "materials", "textures")
    os.makedirs(tex_dir, exist_ok=True)
    texture = f"{name}.png"
    cv2.imwrite(os.path.join(tex_dir, texture), img)
    with open(os.path.join(model_dir, "model.sdf"), "w") as f:
        f.write(MODEL_SDF.format(name=name, width_m=width_m,
                                 height_m=height_m, texture=texture,
                                 thickness_m=args.thickness_mm / 1000.0,
                                 square_note=args.square_mm))
    with open(os.path.join(model_dir, "model.config"), "w") as f:
        f.write(MODEL_CONFIG.format(name=name, cols=args.cols,
                                    rows=args.rows, square_mm=args.square_mm))

    print(f"model    : {model_dir}")
    print(f"desen    : {args.cols}x{args.rows} iç köşe "
          f"({args.cols + 1}x{args.rows + 1} kare) @ {args.square_mm} mm")
    print(f"doku     : {img.shape[1]}x{img.shape[0]} px "
          f"({total_x}x{total_y} kare, sessiz kenar dahil)")
    print(f"plane    : {width_m * 1000:.1f} x {height_m * 1000:.1f} mm")

    # Üretileni hemen tespit ettir: desen yanlışsa burada patlasın, sim'de değil.
    found, corners = cv2.findChessboardCorners(img, (args.cols, args.rows))
    expected = args.cols * args.rows
    if not found or len(corners) != expected:
        raise SystemExit(
            f"DOĞRULAMA BAŞARISIZ: dokuda {args.cols}x{args.rows} iç köşe "
            f"bulunamadı (found={found}). Model yazıldı ama KULLANILMAMALI.")
    print(f"doğrulama: findChessboardCorners {len(corners)}/{expected} köşe OK")


if __name__ == "__main__":
    main()
