#!/usr/bin/env python3
"""Issue #8: ~9 mm sabit sapma nereden geliyor? (ÇEVRİMDIŞI, sim gerekmez)

SORU
  2026-08-12 sim provasında üç poz kümesi çözüldü. Koşullanmaları 60.20 / 3.54 /
  25.64 mm -- yani 17 kat fark -- ama yer gerçeği X'e göre hataları 8.92 / 9.34 /
  9.07 mm çıktı. Koşullanma bu kadar değişirken hatanın kıpırdamaması,
  GÜRÜLTÜ olmadığını söyler: gürültü koşullanmayla ölçeklenir. İmza, sabit bir
  DÖNÜŞÜM hatasınındır.

  Açık soru: sapmanın kaynağı örnek kurma konvansiyonu mu, çözücü mü?
  Sonraki adım sentetik veriyle ayrıştırmadır; sim gerekmez.

ÇÖZÜCÜ ZATEN AKLANDI
  `solve_hand_eye.py --self-test` kusursuz veride beş yöntemin beşiyle de X'i
  0.0000 mm hatayla geri kazanıyor (2026-08-15 koşuldu). Dolayısıyla kalan tek
  yer örneklerin KURULMASIDIR.

BU ARAÇ NE YAPAR
  Kusursuz sentetik veri üretir (X ve board pozu bilinir), sonra TEK TEK aday
  konvansiyon hatası enjekte edip X hatasını ölçer. Doğru aday iki testi birden
  geçmeli:
    1. Büyüklük ~9 mm mertebesinde olmalı.
    2. Hata poz kümesinin koşullanmasına DUYARSIZ olmalı -- gerçek gözlemin
       ayırt edici yanı buydu.

  İkinci şart önemli: yalnız büyüklüğe bakmak birçok adayı yanlışlıkla
  doğrular. Bu yüzden her aday iki farklı poz kümesinde koşulur.

NE KANITLAMAZ
  Sim provasının GERÇEKTEN hangi hatayı yaptığını kanıtlamaz; hangi hata
  sınıfının gözlenen imzayı ürettiğini daraltır. Kesin cevap, prova kodundaki
  frame kullanımına bakarak doğrulanır.
"""

import argparse
import math
import sys

import numpy as np
import cv2


def rt_to_matrix(rotation, translation):
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = np.asarray(translation).ravel()
    return matrix


def random_transform(rng, spread_deg=60.0, radius=0.35):
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    angle = math.radians(rng.uniform(-spread_deg, spread_deg))
    rotation, _ = cv2.Rodrigues(axis * angle)
    translation = rng.normal(scale=radius / 3.0, size=3)
    return rt_to_matrix(rotation, translation)


def build_exact(rng, count, spread_deg):
    """Kusursuz AX = XB verisi. Konvansiyon solve_hand_eye.py ile AYNI:
       A = base -> wrist, B = camera -> board."""
    true_x = rt_to_matrix(
        cv2.Rodrigues(np.array([0.05, -0.9, 0.12]))[0],
        np.array([0.031, -0.014, 0.052]))
    t_base_board = rt_to_matrix(
        cv2.Rodrigues(np.array([2.9, 0.1, 0.2]))[0],
        np.array([0.28, 0.02, 0.04]))
    a_matrices = [random_transform(rng, spread_deg) for _ in range(count)]
    b_matrices = [np.linalg.inv(true_x) @ np.linalg.inv(a) @ t_base_board
                  for a in a_matrices]
    return true_x, t_base_board, a_matrices, b_matrices


def solve_x(a_matrices, b_matrices, method=cv2.CALIB_HAND_EYE_PARK):
    rotation, translation = cv2.calibrateHandEye(
        [a[:3, :3] for a in a_matrices], [a[:3, 3] for a in a_matrices],
        [b[:3, :3] for b in b_matrices], [b[:3, 3] for b in b_matrices],
        method=method)
    return rt_to_matrix(rotation, translation)


def conditioning_mm(a_matrices, b_matrices):
    """Poz kümesinin koşullanması: X'e dik yönde ne kadar bilgi var.

    Provada kullanılan ölçüyle aynı ruhta -- dönme eksenlerinin saçılımı ne
    kadar darsa çözüm o kadar kötü koşullanır. Burada sayı yalnız KÜMELERİ
    BİRBİRİYLE kıyaslamak için kullanılır.
    """
    axes = []
    for index in range(len(a_matrices) - 1):
        relative = np.linalg.inv(a_matrices[index]) @ a_matrices[index + 1]
        vector, _ = cv2.Rodrigues(relative[:3, :3])
        norm = np.linalg.norm(vector)
        if norm > 1e-9:
            axes.append(vector.ravel() / norm)
    if len(axes) < 2:
        return float('inf')
    singular = np.linalg.svd(np.array(axes), compute_uv=False)
    return float(singular[0] / max(singular[-1], 1e-12))


# --- aday konvansiyon hataları -------------------------------------------

def candidate_b_inverted(a_matrices, b_matrices, _):
    """B ters yönde kaydedilmiş: board -> camera."""
    return a_matrices, [np.linalg.inv(b) for b in b_matrices]


def candidate_a_inverted(a_matrices, b_matrices, _):
    """A ters yönde: wrist -> base."""
    return [np.linalg.inv(a) for a in a_matrices], b_matrices


def make_fixed_offset(offset_m):
    def candidate(a_matrices, b_matrices, _):
        """A, X'in tanımlandığı frame'den SABİT bir ötelemeyle kayık frame'de
        kaydedilmiş (ör. link_5 yerine başka bir uç frame)."""
        shift = np.eye(4)
        shift[:3, 3] = offset_m
        return [a @ shift for a in a_matrices], b_matrices
    return candidate


def make_board_scale(scale):
    def candidate(a_matrices, b_matrices, _):
        """Board model ölçeği yanlış: PnP mesafeyi orantılı kaydırır, yani
        B'nin ötelemesi ölçeklenir."""
        scaled = []
        for b in b_matrices:
            copy = b.copy()
            copy[:3, 3] = copy[:3, 3] * scale
            scaled.append(copy)
        return a_matrices, scaled
    return candidate


def candidate_optical_rotation(a_matrices, b_matrices, _):
    """B, camera_link yerine camera_optical_frame'de (REP-103 dönüşü) verilmiş.

    Saf dönme: öteleme ofseti YOK. Yine de X'in ötelemesini bozar mi?
    """
    rotation, _ = cv2.Rodrigues(np.array([-math.pi / 2, 0.0, -math.pi / 2]))
    optical = rt_to_matrix(rotation, np.zeros(3))
    return a_matrices, [optical @ b for b in b_matrices]


def make_camera_side_offset(offset_m):
    def candidate(a_matrices, b_matrices, _):
        """Board, camera_optical_frame'den SABİT bir ötelemeyle kayık bir
        orijine göre ölçülmüş (sim kamerasının optik merkezi vs camera_link).

        A tarafındaki kaymanın aynadaki görüntüsü: X'in iki ucundan hangisi
        kayarsa kaysın imza aynı. Bu yüzden ölçüm tek başına HANGİ UÇ olduğunu
        söyleyemez; onu ancak frame zincirine bakmak söyler.
        """
        shift = np.eye(4)
        shift[:3, 3] = offset_m
        return a_matrices, [shift @ b for b in b_matrices]
    return candidate


CANDIDATES = [
    ('B ters (board->camera)', candidate_b_inverted),
    ('A ters (wrist->base)', candidate_a_inverted),
    ('A frame 9 mm kayik', make_fixed_offset(np.array([0.009, 0.0, 0.0]))),
    ('A frame 118 mm kayik (tool0 vs link_5)',
     make_fixed_offset(np.array([0.09475, -0.07099, -0.00352]))),
    ('board olcegi %2 buyuk', make_board_scale(1.02)),
    ('board olcegi %0.5 buyuk', make_board_scale(1.005)),
    ('B optik frame donusunde', candidate_optical_rotation),
    ('kamera tarafi 9 mm kayik', make_camera_side_offset(np.array([0.0, 0.0, 0.009]))),
]


def main():
    parser = argparse.ArgumentParser(
        description='hand-eye sabit sapmasinin kaynagini sentetik veriyle daraltir')
    parser.add_argument('--samples', type=int, default=14)
    parser.add_argument('--seed', type=int, default=7)
    args = parser.parse_args()

    # İki poz kümesi: biri geniş (iyi koşullanmış), biri dar (kötü).
    sets = []
    for label, spread in (('genis', 60.0), ('dar', 12.0)):
        rng = np.random.default_rng(args.seed)
        truth, _, a_matrices, b_matrices = build_exact(rng, args.samples, spread)
        sets.append((label, truth, a_matrices, b_matrices,
                     conditioning_mm(a_matrices, b_matrices)))

    print('poz kumeleri:')
    for label, _, _, _, cond in sets:
        print('  %-6s kosullanma orani %.2f' % (label, cond))

    print('\nsaglik kontrolu (hata enjekte edilmemis):')
    for label, truth, a_matrices, b_matrices, _ in sets:
        x = solve_x(a_matrices, b_matrices)
        error = np.linalg.norm(x[:3, 3] - truth[:3, 3]) * 1000.0
        print('  %-6s X hatasi %.4f mm' % (label, error))

    print('\n%-40s %12s %12s %10s' % ('aday konvansiyon hatasi', 'genis (mm)',
                                      'dar (mm)', 'oran'))
    print('-' * 78)
    for name, candidate in CANDIDATES:
        errors = []
        for _, truth, a_matrices, b_matrices, _ in sets:
            broken_a, broken_b = candidate(a_matrices, b_matrices, truth)
            x = solve_x(broken_a, broken_b)
            errors.append(np.linalg.norm(x[:3, 3] - truth[:3, 3]) * 1000.0)
        ratio = errors[1] / errors[0] if errors[0] > 1e-9 else float('inf')
        print('%-40s %12.3f %12.3f %10.2f' % (name, errors[0], errors[1], ratio))

    print('\nOKUMA: gozlenen imza ~9 mm VE kosullanmaya duyarsiz (oran ~1.0).')
    print('Buyuklugu tutturup oranı tutturmayan aday, aciklama DEGILDIR.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
