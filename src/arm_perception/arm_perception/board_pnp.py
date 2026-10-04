#!/usr/bin/env python3
"""
Robot Arm vizyon-encoder — düz checkerboard'dan kamera↔board pozu.

Eye-in-hand kamera SABİT bir checkerboard'a bakar; bu modül board'un kamera
frame'indeki pozunu (rvec/tvec) çıkarır. "Sanal encoder"ın alt katmanı budur:
kamera pozu → (hand-eye ile) TCP pozu → (IK ile) eklem kestirimi.

Neden düz checkerboard (ChArUco değil): saf OpenCV çekirdeği, opencv-contrib/aruco
BAĞIMLILIĞI YOK — hedef Nano konteynerinin OpenCV sürümü ne olursa olsun çalışır.
SINIRLARI (encoder olarak akılda tut): (1) köşe sıralaması yalnız renk paritesi
180° dönüşü ayırt eden bir geometride kararlıdır; (2) tüm board kadrajda olmalı,
kısmi görüntüde findChessboardCorners düşer. Bunlar ısırırsa ChArUco'ya geçilir.

Birimler: obj noktaları METRE (pose metre çıksın diye). square_size_mm/1000.
"""
import numpy as np

try:
    import cv2
except ImportError:  # test ortamında cv2 yoksa import zamanı patlamasın
    cv2 = None

# SB dedektor bayraklari; `solve_camera_intrinsics.py` ile AYNI olmali ki
# kalibrasyon ile olcum ayni kose tanimini kullansin.
_SB_FLAGS = 0 if cv2 is None else (
    cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE
    | cv2.CALIB_CB_ACCURACY)

# cornerSubPix durdurma kriteri: 30 iterasyon veya 0.001 px hareket.
_SUBPIX_CRITERIA = (3, 30, 0.001) if cv2 is None else (
    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Güncel fiziksel tahta: 7x10 kare, 6x9 iç köşe. Baskı %110 olduğu için
# gerçek kare 27.5 mm; intrinsics'in alındığı eski 25 mm tahtayla karıştırma.
DEFAULT_BOARD_COLS = 6
DEFAULT_BOARD_ROWS = 9
DEFAULT_SQUARE_SIZE_MM = 27.5


def checkerboard_ordering_ambiguous(cols, rows):
    """
    180° dönüşün detektör köşe sırasını geometriden ayırt edip etmediği.

    İç-köşe ızgarasında 180° dönüş kare rengini ``(cols + rows) mod 2`` kadar
    değiştirir. Toplam çiftse renkler korunur ve OpenCV detektörü iki uç
    sıralamasını ayırt edemez; toplam tekse renkler terslenir ve sıralama
    kararlıdır. Bu, PnP reprojection marjından farklıdır: dikdörtgen nesne-nokta
    ızgarası 180° simetrik olduğundan düz/ters PnP adayları her iki geometride
    de eşit iyi oturabilir.
    """
    cols = int(cols)
    rows = int(rows)
    if cols <= 0 or rows <= 0:
        raise ValueError('checkerboard cols/rows pozitif olmali')
    return (cols + rows) % 2 == 0


def board_object_points(cols, rows, square_size_mm):
    """
    Checkerboard iç köşelerinin 3B koordinatları.

    Sıra findChessboardCorners'ın döndürdüğü sırayla AYNI olmalı: satır-major,
    her satır soldan sağa. cols = satır başına iç köşe, rows = satır sayısı.
    """
    objp = np.zeros((rows * cols, 3), np.float32)
    grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp[:, :2] = grid * (square_size_mm / 1000.0)
    return objp


def detect_corners(gray, cols, rows):
    """
    Checkerboard iç köşelerini bul (alt-piksel dahil).

    Tüm board görünür olmalı; kısmi/eğik-aşırı görüntüde None döner (sessiz
    kaybı önlemek için çağıran None kontrolü yapmalı).

    DEDEKTÖR SEÇİMİ — 2026-08-18'de ölçüldü. `solve_camera_intrinsics.py`
    2026-07-22'de sektör tabanlı `findChessboardCornersSB`'ye geçmişti; notu
    "legacy dedektör + cornerSubPix, IMX219'un ISP ile yeniden ölçeklenmiş
    640x480 akışında çok-piksellik sistematik sapma gösterdi" diyor. Bu dosya
    o geçişi hiç almamıştı, ve BÜTÜN ölçüm hattı buradan besleniyor.

    25 kalibrasyon karesinde, aynı intrinsics ile ölçüldü:

        dedektör   reproj ort   medyan   maks     Nano maliyeti
        legacy       0.450 px    0.291   2.561      981 ms/kare
        SB           0.279 px    0.234   0.590      209 ms/kare

    Yani SB hem daha doğru (özellikle kuyrukta: 2.56 -> 0.59) hem 4.7 kat
    hızlı. Taviz yok. Legacy yol yalnız OpenCV < 4.0 için yedek kalıyor.
    """
    if cv2 is None:
        raise RuntimeError('cv2 yok')
    if hasattr(cv2, 'findChessboardCornersSB'):
        found, corners = cv2.findChessboardCornersSB(gray, (cols, rows),
                                                     _SB_FLAGS)
        if found:
            return corners
        return None
    return _detect_corners_legacy(gray, cols, rows)


def _detect_corners_legacy(gray, cols, rows):
    """OpenCV < 4.0 icin yedek yol. SB varsa KULLANILMAZ.

    Bu yol IMX219'un ISP ile 640x480'e olceklenmis akisinda sistematik sapma
    tasiyor; olculen degerler ust taraftaki `detect_corners` notunda.
    """
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
             + cv2.CALIB_CB_FAST_CHECK)
    found, corners = cv2.findChessboardCorners(gray, (cols, rows), flags)
    if not found:
        return None
    corners = cv2.cornerSubPix(
        gray, corners, subpix_window(corners, cols, rows), (-1, -1),
        _SUBPIX_CRITERIA)
    return corners


def subpix_window(corners, cols, rows):
    """Alt-piksel arama penceresini ÖLÇÜLEN köşe aralığına göre seç.

    Sabit (11, 11) yarı-pencere 23x23 px tarar. Tahta kadrajda küçük kaldığında
    -- 2026-08-18'de gerçek kamerada köşe aralığı 11.5 px ölçüldü -- bu pencere
    komşu köşenin ÖTESİNE uzanır ve iyileştirme köşeyi yanlış özelliğe çeker.
    Aynı karede ölçüldü: pencere (11,11) ile reprojeksiyon 5.51 px, (9,9) ile
    2.09, (7,7) ve altında 0.25. Yani araç sessizce kabul edilebilir bir
    tespiti kullanılamaz hale getiriyordu; `measure_perception_floor.py` de
    bunu 1.0 px kapısında reddedip "tahta kadrajda mı?" diye soruyordu.

    Pencere aralığın yarısının altında kalmalı. Tahta büyükse eski davranış
    korunur (üst sınır 11).
    """
    grid = corners.reshape(rows, cols, 2)
    spacing = min(float(np.linalg.norm(np.diff(grid, axis=1), axis=2).min()),
                  float(np.linalg.norm(np.diff(grid, axis=0), axis=2).min()))
    return (max(2, min(11, int(spacing / 2.0) - 1)),) * 2


def solve_pose(object_points, image_points, camera_matrix, dist_coeffs):
    """
    Düzlemsel checkerboard pozu → (ok, rvec, tvec).

    NEDEN IPPE + en-iyi-seçim: düz bir checkerboard'ın kamera pozu DÜZLEMSEL poz
    belirsizliği taşır — matematiksel olarak iki farklı poz aynı köşe izdüşümünü
    (yaklaşık) üretir. SOLVEPNP_ITERATIVE bazı görüş açılarında yanlış dala
    yakınsıyordu (offline testte 6/25 karede reproj 4-6px). IPPE düzlemsel hedefe
    özeldir ve HER İKİ çözümü döndürür; reprojection'ı düşük olanı seçeriz.
    (Not: bu, flip belirsizliğini görüş-geometrisiyle giderir ama sıfırlamaz —
    kritik uygulamada ChArUco ID'leri ya da önsel poz gerekir.)
    """
    if cv2 is None:
        raise RuntimeError('cv2 yok')
    try:
        n, rvecs, tvecs, reproj = cv2.solvePnPGeneric(
            object_points, image_points, camera_matrix, dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE)
    except cv2.error:
        # IPPE ≥4 eş-düzlem nokta ister; beklenmedik durumda iteratif'e düş.
        ok, rvec, tvec = cv2.solvePnP(
            object_points, image_points, camera_matrix, dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE)
        return ok, rvec, tvec
    if n < 1:
        return False, None, None
    # reproj (solvePnPGeneric'ten) en düşük olan çözümü seç.
    # np.ravel()[0]: solvePnPGeneric reproj'u (n,1) dizi olarak döndürüyor;
    # float() ile doğrudan skalara çevirmek NumPy 1.25'te DeprecationWarning,
    # ileride hata olacak.
    best = int(np.argmin([float(np.ravel(r)[0]) for r in reproj])) if n > 1 else 0
    return True, rvecs[best], tvecs[best]


def reprojection_error_px(object_points, image_points, rvec, tvec,
                          camera_matrix, dist_coeffs):
    """
    Ortalama reprojection hatası — poz kalitesinin tek sayılık ölçüsü.

    Intrinsics RMS'ine yakınsa (~0.2px) PnP sağlıklı; büyükse yanlış eşleşme,
    yanlış kare boyutu veya bozuk board düzlemselliği işaretidir.
    """
    if cv2 is None:
        raise RuntimeError('cv2 yok')
    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, dist_coeffs)
    projected = projected.reshape(-1, 2)
    observed = image_points.reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum((projected - observed) ** 2, axis=1))))


def estimate_board_pose(gray, cols, rows, square_size_mm,
                        camera_matrix, dist_coeffs):
    """
    Tam boru hattı: tespit → PnP → residual.

    Döndürür: dict{rvec, tvec, reproj_px, n_corners, distance_m,
    ambiguous, ordering_stable, reprojection_margin_px} ya da None.
    distance_m = board merkezinin kameraya uzaklığı (tvec norm'u ~), hızlı sanity.

    ``ambiguous`` detektör sıralama sözleşmesidir; PnP aday marjı değildir.
    Eski 6x8 tahta için True, güncel 6x9 tahta için False döner.
    """
    corners = detect_corners(gray, cols, rows)
    if corners is None:
        return None
    objp = board_object_points(cols, rows, square_size_mm)

    # Düz checkerboard'ın kimliği yok: findChessboardCorners köşeleri board'un
    # görünüşüne göre ters uçtan döndürebilir (180° detection-order flip). Sabit
    # objp'ye karşı yanlış eşleşme yüksek reproj / yanlış poz verir. İki sıralamayı
    # da (düz + ters) dene, reprojection'ı düşük olanı seç. Bu, DETECTION flip'ini
    # giderir; gerçek fiziksel 180° poz belirsizliği için (kamera hangi taraftan
    # bakıyor bilinmiyorsa) yine önsel/ChArUco gerekir — bkz. flipped bayrağı.
    # 🔴 2026-07-28 CANLI ÖLÇÜM — saf `min(reproj)` tie-break'i POZU ZIPLATIYOR.
    # Tahtamız 6x8 iç köşe. 180° dönüşte iç köşe (i,j) -> (cols-1-i, rows-1-j)
    # gider ve kare rengi (i+j) pariteyle belirlendiğinden parite değişimi
    # (cols+rows) mod 2'ye eşittir. 6+8=14 ÇİFT -> renkler korunur -> iki
    # sıralama GÖRÜNTÜ OLARAK AYIRT EDİLEMEZ. Ölçüldü: 25 karede iki
    # sıralamanın reprojeksiyon farkı 0.0000 px (maks 0.0000). Eşit iki değer
    # arasında `<` karşılaştırması ondalık gürültüyle karar veriyordu; sonuç
    # ardışık karelerde ~180° dönen poz, konumda 12-18 cm sıçrama
    # (std x 46mm / y 69mm, açı sapması 180°'ye kadar).
    #
    # `findChessboardCorners`'ın NATIVE sıralaması ise sabit sahnede 25/25
    # kararlı ölçüldü. Bu yüzden: native sıralama VARSAYILAN, ters sıralama
    # yalnız ANLAMLI ölçüde daha iyiyse seçilir. Anlamlı fark, gerçekten yanlış
    # eşleşmiş bir sıralamayı gösterir (o durumda reproj kat kat büyür).
    #
    # SINIR — bu bir belirsizlik ÇÖZÜMÜ DEĞİL, kararlılık düzeltmesidir:
    # native sıralama görüntü uzayına bağlıdır (soldan-üstten). Kamera tahtaya
    # belirgin biçimde farklı bir yönelimle yaklaşırsa native sıralama da
    # dönebilir. Kol gerçekten hareket ettiğinde bu YENİDEN ÖLÇÜLMELİ.
    # Kalıcı çözüm: cols+rows TEK olan bir tahta (ör. 6x9) veya ChArUco.
    _FLIP_MARGIN_PX = 0.05

    candidates = []
    for flipped, pts in ((False, corners), (True, corners[::-1].copy())):
        ok, rvec, tvec = solve_pose(objp, pts, camera_matrix, dist_coeffs)
        if not ok:
            continue
        reproj = reprojection_error_px(
            objp, pts, rvec, tvec, camera_matrix, dist_coeffs)
        candidates.append({'rvec': rvec, 'tvec': tvec, 'reproj_px': reproj,
                           'flipped': flipped})
    if not candidates:
        return None

    native = next((c for c in candidates if not c['flipped']), None)
    flipped = next((c for c in candidates if c['flipped']), None)
    if native is None:
        best = flipped
    elif flipped is None:
        best = native
    else:
        # Ters sıralamayı ancak belirgin biçimde daha iyiyse seç.
        best = flipped if flipped['reproj_px'] < native['reproj_px'] - _FLIP_MARGIN_PX \
            else native
    # PnP marjı bir yönelim kanıtı değildir: dikdörtgen nesne-nokta ızgarası
    # 180° simetrik olduğu için doğru 6x9 tahtada bile yaklaşık sıfırdır.
    # Operatöre gösterilen `ambiguous` bunun yerine DETEKTÖR sıralamasının renk
    # paritesinden ayırt edilip edilmediğini bildirir. Marj ayrıca telemetri
    # olarak tutulur, fakat ambiguity kararı vermez.
    if native is not None and flipped is not None:
        best['reprojection_margin_px'] = abs(
            flipped['reproj_px'] - native['reproj_px'])
    else:
        best['reprojection_margin_px'] = None
    best['ambiguous'] = checkerboard_ordering_ambiguous(cols, rows)
    best['ordering_stable'] = not best['ambiguous']
    best['n_corners'] = int(corners.shape[0])
    best['distance_m'] = float(np.linalg.norm(best['tvec']))
    return best
