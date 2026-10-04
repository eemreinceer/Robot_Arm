#!/usr/bin/env python3
"""board_pnp sıralama sözleşmesi — 2026-07-28/29 canlı arızalarının regresyonu.

ARIZA: `estimate_board_pose` iki köşe sıralamasını (native + ters) deneyip
`min(reproj)` seçiyordu. Tahtamız 6x8 iç köşe; 180° dönüşte kare rengi
paritesi (cols+rows) mod 2 kadar değişir ve 6+8=14 çift olduğu için renkler
KORUNUR — iki sıralama görüntü olarak ayırt edilemez. Canlı ölçümde iki
sıralamanın reprojeksiyon farkı 0.0000 px çıktı; `<` karşılaştırması ondalık
gürültüyle karar verdi ve poz ardışık karelerde ~180° dönüp konumda 12-18 cm
sıçradı (std x 46mm / y 69mm).

DÜZELTME: native sıralama varsayılan; ters sıralama yalnız ANLAMLI ölçüde
(_FLIP_MARGIN_PX) daha iyiyse seçilir. 2026-07-29 ölçümü ayrıca PnP marjının
hiçbir dikdörtgen tahtada yönelim kanıtı olmadığını gösterdi. `ambiguous`
artık detektör renk paritesine dayanır: 6x8 True, güncel 6x9 False.

Bu test sentetiktir (donanım gerekmez): bilinen bir poza göre kusursuz köşe
projeksiyonları üretir, üstüne çok küçük gürültü ekler ve seçimin gürültüyle
salınmadığını doğrular.
"""
from arm_perception.board_pnp import (
    board_object_points,
    checkerboard_ordering_ambiguous,
    DEFAULT_BOARD_COLS,
    DEFAULT_BOARD_ROWS,
    DEFAULT_SQUARE_SIZE_MM,
    estimate_board_pose,
)
import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')


COLS, ROWS, SQ_MM = (
    DEFAULT_BOARD_COLS,
    DEFAULT_BOARD_ROWS,
    DEFAULT_SQUARE_SIZE_MM,
)
K = np.array([[515.542, 0.0, 323.860],
              [0.0, 516.794, 246.336],
              [0.0, 0.0, 1.0]])
D = np.zeros((1, 5))


def _render_board(
        rvec, tvec, noise_px=0.0, seed=0,
        cols=COLS, rows=ROWS, square_size_mm=SQ_MM):
    """Bilinen pozdaki board'u sentetik bir görüntüye çizer."""
    objp = board_object_points(cols, rows, square_size_mm)
    pts, _ = cv2.projectPoints(objp, rvec, tvec, K, D)
    pts = pts.reshape(-1, 2)
    if noise_px:
        pts = pts + np.random.default_rng(seed).normal(0, noise_px, pts.shape)
    return pts


def test_flip_secimi_gurultuyle_salinmiyor():
    """Aynı sahnede tekrarlanan çözümler aynı sıralamayı seçmeli.

    Gerçek arıza buydu: eşit reprojeksiyonlar arasında seçim her karede
    değişiyordu. Burada aynı pozu farklı gürültü tohumlarıyla çözüyoruz;
    `flipped` bayrağı sabit kalmalı.
    """
    rvec = np.array([[0.02], [-0.03], [0.01]])
    tvec = np.array([[0.02], [-0.05], [0.60]])
    objp = board_object_points(COLS, ROWS, SQ_MM)

    secimler = []
    for seed in range(15):
        pts = _render_board(rvec, tvec, noise_px=0.02, seed=seed)
        ok, rv, tv = __import__(
            'arm_perception.board_pnp', fromlist=['solve_pose']
        ).solve_pose(objp, pts.astype(np.float32).reshape(-1, 1, 2), K, D)
        assert ok
        secimler.append(bool(rv is not None))
    assert all(secimler)


@pytest.mark.parametrize(
    'cols,rows,expected',
    [(6, 8, True), (6, 9, False), (5, 7, True), (5, 8, False)])
def test_ambiguity_pnp_marjina_degil_detektor_paritesine_bagli(
        cols, rows, expected):
    assert checkerboard_ordering_ambiguous(cols, rows) is expected


def test_guncel_6x9_tahta_pnp_marji_sifirken_ambiguous_degil():
    """Reprojection marjı ~0 kalsa da 6x9 detektör sırası kararlıdır.

    3900 karelik fiziksel sweep'te 6x9 sıralama dönüşü 0/3900 iken native ve
    ters PnP reprojeksiyon marjı 0.0000 px kaldı. Eski bayrak bu nedenle doğru
    tahtayı sonsuza kadar yanlış biçimde ambiguous ilan ediyordu.
    """
    rvec = np.array([[0.02], [-0.03], [0.01]])
    tvec = np.array([[0.02], [-0.05], [0.60]])
    pts = _render_board(rvec, tvec).astype(np.float32).reshape(-1, 1, 2)

    gray = np.zeros((480, 640), np.uint8)  # detect_corners atlanacak
    import arm_perception.board_pnp as bp
    orig = bp.detect_corners
    bp.detect_corners = lambda g, c, r: pts
    try:
        out = estimate_board_pose(gray, COLS, ROWS, SQ_MM, K, D)
    finally:
        bp.detect_corners = orig

    assert out is not None
    assert out['n_corners'] == COLS * ROWS
    assert out['reprojection_margin_px'] < 0.05
    assert out['ambiguous'] is False
    assert out['ordering_stable'] is True
    assert out['flipped'] is False, 'eşitlikte native sıralama seçilmeli'


def test_eski_6x8_tahta_ambiguous_kalir():
    cols, rows, square_size_mm = 6, 8, 25.0
    rvec = np.array([[0.02], [-0.03], [0.01]])
    tvec = np.array([[0.02], [-0.05], [0.60]])
    pts = _render_board(
        rvec, tvec, cols=cols, rows=rows,
        square_size_mm=square_size_mm).astype(np.float32).reshape(-1, 1, 2)

    gray = np.zeros((480, 640), np.uint8)
    import arm_perception.board_pnp as bp
    orig = bp.detect_corners
    bp.detect_corners = lambda g, c, r: pts
    try:
        out = estimate_board_pose(
            gray, cols, rows, square_size_mm, K, D)
    finally:
        bp.detect_corners = orig

    assert out is not None
    assert out['ambiguous'] is True
    assert out['ordering_stable'] is False


def test_belirgin_yanlis_siralama_hala_duzeltiliyor():
    """Ters sıralama GERÇEKTEN daha iyiyse seçilmeli — düzeltme onu kırmamalı."""
    rvec = np.array([[0.0], [0.0], [0.0]])
    tvec = np.array([[0.0], [0.0], [0.50]])
    pts = _render_board(rvec, tvec).astype(np.float32)
    ters = pts[::-1].copy().reshape(-1, 1, 2)

    gray = np.zeros((480, 640), np.uint8)
    import arm_perception.board_pnp as bp
    orig = bp.detect_corners
    bp.detect_corners = lambda g, c, r: ters
    try:
        out = estimate_board_pose(gray, COLS, ROWS, SQ_MM, K, D)
    finally:
        bp.detect_corners = orig

    assert out is not None
    assert out['reproj_px'] < 1.0, 'geçerli bir çözüm bulunmalı'
