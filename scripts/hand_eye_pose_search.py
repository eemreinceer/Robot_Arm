#!/usr/bin/env python3
"""Duyarlılık kapısını GEÇEN, kolun ULAŞABİLDİĞİ bir poz kümesi arar (issue #8).

Problem: provanın poz kümesi kadraj kısıtı yüzünden dar bir dönüş çeşitliliğine
sıkışıyor ve AX=XB'yi kötü koşullandırıyor — 0.4 mm gürültü 4.56 mm hataya
büyüyor (k ≈ 11). `docs/hand_eye_pose_set_requirement.md` bunu kapı yapıyor:

    k × (ölçülen gürültü tabanı) ≤ (koşudan ÖNCE ilan edilen hedef)

İKİ AŞAMA:

  pool   — geniş bir aday havuzu üretir; `sim_hand_eye_capture.py --poses` ile
           yakalanır. Hangi pozun tahtayı kadrajda tuttuğu ölçümle belirlenir,
           tahminle değil.
  select — yakalanan havuzdan, k'yı en küçük yapan ALT KÜMEYİ arar. Arama
           yalnız A (base->wrist) pozlarını kullanır: B tarafı bilinen X ve
           sabit tahtadan sentetik kurulur, üzerine gürültü enjekte edilir.
           Yani seçim, ölçülen kamera verisine bakmadan yapılır ve seçilen küme
           GERÇEK veriyle ayrıca doğrulanır.

k, çözücünün kendi hatası değil poz geometrisinin büyütme katsayısıdır; aynı
gürültü çeşitli (ama ulaşılamaz) bilek pozlarında k ≈ 1.4 veriyor.

Kullanım:

    python3 scripts/hand_eye_pose_search.py pool --count 150
    ./start_simulation.sh --hand-eye --headless            # ayrı terminal
    python3 scripts/sim_hand_eye_capture.py \
        --poses runs/hand_eye/pose_pool.json \
        --out   runs/hand_eye/pool_samples.json
    python3 scripts/hand_eye_pose_search.py select \
        --samples runs/hand_eye/pool_samples.json --size 10
"""

import argparse
import json
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "scripts"))

import cv2                                                    # noqa: E402
from solve_hand_eye import solve, rt_to_matrix, quat_to_rotation  # noqa: E402

_conv = open(os.path.join(REPO, "scripts",
                          "hand_eye_convention_check.py")).read()
exec(_conv.split("def main()")[0])          # rpy / T / R2q / X_TRUE / BOARD

# URDF limitleri (robot_arm_body.xacro): j1..j4 ±1.52, j5 [-1.52, 0.86].
# Havuz bunların içinde ve kenarlardan uzak kalır — sim'de bile limite dayanan
# poz, gerçek kolda taşınamayacak bir kümeyi meşru gösterir.
LIMITS = [(-1.0, 1.0), (-0.7, 0.7), (-0.7, 0.7), (-1.0, 1.0), (-1.0, 0.86)]

# --- Kinematik zincir, KADRAJI ÖNCEDEN KESTİRMEK İÇİN ------------------------
# İlk deneme kör rastgele örneklemeydi: 150 adayın yalnız 6'sı (%4) tahtayı
# kadrajda tuttu, çünkü j1'i serbest bırakmak kamerayı tahtadan başka yöne
# çeviriyor. Sim'i 14 dakika koşturup %96'sını çöpe atmak yerine kadraj burada
# kestiriliyor; sim yalnız DOĞRULAMA için koşuyor.
#
# Değerler `robot_arm.urdf.xacro` çıktısından (base_link -> link_5, hepsi
# revolute, eksen 0 0 -1). Doğruluğu her koşuda ÖLÇÜLEN A pozlarına karşı
# sınanıyor (`validate_fk`), yani URDF değişirse sessizce yanlışlamaz.
CHAIN = [
    ([0, 0, 0.057], [0, 0, 0]),
    ([-0.012979, -0.0064242, 0.036276], [-1.5708, 0, 0.17977]),
    ([0.035175, -0.11496, 0.0022], [0, 0, 0]),
    ([-0.082205, -0.036372, -0.0107], [1.5708, 0.37973, -1.098]),
    ([-0.0027195, 0.020778, 0.02985], [1.5708, -1.098, -2.7705]),
]

# Kamera (sim): 640x480, fx=fy=554.383, cx=320, cy=240.
K_CAM = np.array([[554.3827128226441, 0, 320.0],
                  [0, 554.3827128226441, 240.0],
                  [0, 0, 1.0]])
IMG_W, IMG_H = 640, 480
MARGIN_PX = 25          # köşeler kenara bu kadar yaklaşmasın
DIST_RANGE_M = (0.25, 0.90)

# Tahta köşe ızgarası, board_pnp.board_object_points ile birebir aynı
# (cols=6, rows=9, kare 27.5 mm; z=0).
_g = np.mgrid[0:6, 0:9].T.reshape(-1, 2) * 0.0275
BOARD_PTS = np.hstack([_g, np.zeros((_g.shape[0], 1))])
BOARD_CENTRE = BOARD_PTS.mean(axis=0)

# --- KISIT 1: TAM CEPHEDEN BAKMA (2026-08-15'te olculdu) ---------------------
# Duzlemsel PnP fronto-paralel goruste dejenere. Gurultusuz sentetik taramada
# egim 0 derecede donme hatasi 1.77 derece, 1 derecede 0.30, 5 derecede 0.04
# (`docs/hardware_session_plan.md`). 0.5 derece ~ 2.9 mm X hatasi oldugu icin
# tek basina hedefi iskalatir; ustelik 0 derecede reprojeksiyon 0.671 px, yani
# aracin 1.0 px kapisinin ALTINDA -- kapi bu kareyi sessizce kabul eder.
# 399 pozluk havuzda orneklerin ~%14'i bu belirsizlige dusuyordu.
# Birkac derece yetiyor; pay birakmak icin 10.
MIN_TILT_DEG = 10.0

# --- KISIT 2: POZLAR ARASI DUZLEM-ICI DONME (2026-08-15'te olculdu) ----------
# 6x9 ic-kose izgarasi 180 derece donme altinda KENDINE eslenir, yani ters
# siralama nativ siralamayla BIREBIR ayni reprojeksiyonu verir: flip_sweep
# 120 karenin 120'sinde marj 0.0000 olctu. board_pnp'nin "ters siralamayi
# yalniz gercek bir marjla iyiyse sec" kurali bu yuzden HIC ATESLENEMEZ --
# beraberlik rastlanti degil, simetri. Dolayisiyla iki poz arasinda buyuk bir
# duzlem-ici donme farki, aracin yapisal olarak fark edemeyecegi bir kose
# eslesmesi uretebilir.
# 90 derece keyfi degil: bunun otesinde bir poz, digerinin NATIV yonelimindense
# 180 donmus haline daha yakindir. Asil cozum ChArUco (kose kimlikleri benzersiz).
MAX_INPLANE_DIFF_DEG = 90.0


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def fk(q):
    """base_link -> link_5. Eksen 0 0 -1 oldugu icin donme Rz(-q)."""
    M = np.eye(4)
    for (xyz, rpyv), qi in zip(CHAIN, q):
        M = M @ T(rpy(*rpyv), xyz) @ T(Rz(-qi), [0, 0, 0])
    return M


def validate_fk(samples, tol_mm=1.0):
    """Offline FK'yi OLCULEN A pozlarina karsi sinar. Gecmezse hicbir sey yapma."""
    worst = 0.0
    for s in samples:
        if not s.get("joints"):
            continue
        A = rt_to_matrix(quat_to_rotation(*s["base_to_wrist"]["quat_xyzw"]),
                         s["base_to_wrist"]["xyz"])
        d = np.linalg.norm(fk(s["joints"])[:3, 3] - A[:3, 3]) * 1000.0
        worst = max(worst, d)
    if worst > tol_mm:
        raise SystemExit(f"FK olculen pozlarla uyusmuyor ({worst:.2f} mm) — "
                         "URDF degismis olabilir, havuz uretilmedi.")
    return worst


def board_in_base(samples):
    """Sabit tahtanin base frame'deki pozu: T = A * X * B, orneklerin ortalamasi."""
    poses = []
    for s in samples:
        A = rt_to_matrix(quat_to_rotation(*s["base_to_wrist"]["quat_xyzw"]),
                         s["base_to_wrist"]["xyz"])
        B = rt_to_matrix(quat_to_rotation(*s["cam_to_board"]["quat_xyzw"]),
                         s["cam_to_board"]["xyz"])
        poses.append(A @ X_TRUE @ B)
    M = np.mean(poses, axis=0)
    U, _, Vt = np.linalg.svd(M[:3, :3])       # ortalamayi tekrar ortonormalle
    M[:3, :3] = U @ Vt
    return M


def frames_board(q, board):
    """Bu pozda tahtanin 54 kosesi de kadrajda mi? (kestirim, olcum degil)"""
    cam = fk(q) @ X_TRUE                       # base -> camera_optical_frame
    inv = np.linalg.inv(cam)
    pts = (inv[:3, :3] @ (board[:3, :3] @ BOARD_PTS.T + board[:3, 3:4])
           + inv[:3, 3:4])
    z = pts[2]
    if z.min() < DIST_RANGE_M[0] or z.max() > DIST_RANGE_M[1]:
        return False
    uv = K_CAM @ (pts / z)
    return bool(uv[0].min() > MARGIN_PX and uv[0].max() < IMG_W - MARGIN_PX
                and uv[1].min() > MARGIN_PX and uv[1].max() < IMG_H - MARGIN_PX)

def board_in_camera(q, board):
    """Tahtanin kamera frame'indeki pozu (B). base->camera = fk(q) @ X_TRUE."""
    return np.linalg.inv(fk(q) @ X_TRUE) @ board


def board_tilt_deg(B):
    """Tahta normali ile tahtanin MERKEZINE giden bakis hatti arasindaki aci.

    0 derece = TAM CEPHEDEN bakis (dejenere), 90 = tahtaya kenardan bakis.
    Isaret onemsiz oldugu icin mutlak kosinus alinir: tahtanin normali
    kameraya donuk de olabilir, ters de.
    """
    normal = B[:3, 2]
    ray = B[:3, :3] @ BOARD_CENTRE + B[:3, 3]
    cos = abs(float(normal @ ray)) / (np.linalg.norm(normal) * np.linalg.norm(ray))
    return float(np.degrees(np.arccos(np.clip(cos, 0.0, 1.0))))


def board_inplane_deg(B):
    """Tahtanin ilk satirinin GORUNTU duzlemindeki yonelimi (derece).

    Kose siralamasini belirleyen sey tahtanin 3B ekseni degil, o eksenin
    GORUNTUDEKI izdusumudur -- cv2 goruntuye bakar. Bu yuzden aci iki kose
    noktasinin izdusumu arasindan olculur, rotasyon matrisinin sutunundan
    degil; egik tahtalarda ikisi 30 dereceye varan fark verebiliyor.
    """
    def project(point):
        c = B[:3, :3] @ np.asarray(point, np.float64) + B[:3, 3]
        return (K_CAM @ (c / c[2]))[:2]

    first, last = project(BOARD_PTS[0]), project(BOARD_PTS[5])
    delta = last - first
    return float(np.degrees(np.arctan2(delta[1], delta[0])))


def inplane_diff_deg(a, b):
    """Iki yonelim arasindaki fark, [0, 180] araligina sarilmis."""
    d = abs(a - b) % 360.0
    return min(d, 360.0 - d)


def inplane_spread_deg(angles):
    """Kumedeki EN BUYUK ikili fark. Kisit bunun uzerine kurulur."""
    worst = 0.0
    for i in range(len(angles)):
        for j in range(i + 1, len(angles)):
            worst = max(worst, inplane_diff_deg(angles[i], angles[j]))
    return worst


POOL_OUT = os.path.join(REPO, "runs", "hand_eye", "pose_pool.json")
SET_OUT = os.path.join(REPO, "data", "hand_eye", "gated_pose_set.json")

# --- KAPI, KOŞUDAN ÖNCE İLAN EDİLİR -----------------------------------------
# Hedef: çözülen X, yer gerçeğinden en fazla 1.0 mm sapsın (issue #8 kriteri).
# Ölçülen gürültü tabanı: kanonik provanın zincir saçılımı 0.41 mm rms.
# Dolayısıyla kabul için k ≤ 1.0 / 0.41 = 2.44.
TARGET_MM = 1.0
# ⚠ GÜRÜLTÜ TABANI SABİT DEĞİL, KÜMEYE GÖRE DEĞİŞİR — 2026-08-14'te ÖLÇÜLDÜ.
# Bu değer 5 pozluk provadan geliyordu (0.41 mm) ve 20 pozluk aranmış kümeye
# taşındığında YANLIŞ çıktı: aynı kümenin gerçek yakalamasında tahta saçılımı
# 1.38 mm rms, yani 3.4 KAT. Sebep fiziksel — geniş bilek dönüşleri FK/zero-offset/
# servo hatasını büyütüyor, ve poz çeşitliliği tam da aramanın ARTIRDIĞI şey.
# Sonuç: sentetik kapı GEÇTİ (k p90 1.81), gerçek veri 2.81 mm verdi; 1.81 × 1.38
# = 2.50 mm ile tutarlı. Yani model doğru, GİRDİ varsayımı yanlıştı.
# Bu yüzden taban artık ELLE VERİLİR: aday kümenin KENDİ yakalamasından ölçülüp
# geçilmelidir (solve_hand_eye.py çıktısındaki board scatter rms).
DEFAULT_NOISE_FLOOR_MM = 0.41
# Backward-compatible names used by the tracked regression fixture.  They
# describe the historical five-pose baseline; live selection computes its
# threshold from the explicitly supplied noise floor below.
NOISE_FLOOR_MM = DEFAULT_NOISE_FLOOR_MM
K_GATE = TARGET_MM / NOISE_FLOOR_MM

# ⚠ k TEK GÜRÜLTÜ ÇEKİLİŞİNDEN OKUNMAZ. Kötü koşullu bir kümede hata, gürültünün
# hangi yöne düştüğüne kuvvetle bağlı: provanın kümesi 300 çekilişte ortalama
# 3.85, medyan 3.50, p90 6.41, maks 13.73 veriyor. Tek çekiliş 11.4 de
# gösterebilir, 1.2 de — ikisi de kümeyi tarif etmez. Kapı bu yüzden p90
# üzerinden kurulur: kötü ama makul bir çekilişte bile hedef tutmalı.
SEARCH_TRIALS = 24    # arama sırasında ucuz vekil (ortalama)
GATE_TRIALS = 300     # nihai kapı ölçümü (p90)
GATE_QUANTILE = 0.90
PROBE_NOISE_MM = 0.4  # k doğrusal olduğu için tek seviyede ölçmek yeterli


def pool(count, seed, reference, min_tilt_deg=MIN_TILT_DEG):
    """Kadrajda kalacagi KESTIRILEN adaylardan havuz kurar.

    Referans, tahtanin nerede oldugunu ve FK'nin dogru oldugunu belirlemek icin
    daha onceki bir yakalamadan gelir — kestirim olcume dayanir, varsayima degil.
    """
    ref = json.load(open(reference))["samples"]
    worst = validate_fk(ref)
    board = board_in_base(ref)
    print(f"FK dogrulamasi : olculen A ile en buyuk fark {worst:.2f} mm "
          f"({len(ref)} ornek)")
    print(f"tahta (base)   : {np.round(board[:3, 3], 4).tolist()} m")

    rng = np.random.default_rng(seed)
    poses, tilts = [], []
    out_of_frame = too_frontal = tried = 0

    def consider(q):
        """Kadraj VE egim kapisi. Ikisi ayri sayilir; hangisinin eledigi bilinsin."""
        nonlocal out_of_frame, too_frontal
        if not frames_board(q, board):
            out_of_frame += 1
            return False
        tilt = board_tilt_deg(board_in_camera(q, board))
        if tilt < min_tilt_deg:
            too_frontal += 1
            return False
        poses.append([round(v, 4) for v in q])
        tilts.append(round(tilt, 2))
        return True

    # Ev pozu (hepsi sifir) once denenir ama MUAF DEGIL: onceki surum onu
    # kosulsuz listenin basina koyuyordu, yani kadraj ve egim kapilarinin
    # ikisini de atlayan tek bir poz her havuza giriyordu.
    if not consider([0.0] * 5):
        print("not            : ev pozu (0,0,0,0,0) kapilardan gecmedi, alinmadi")
    while len(poses) < count and tried < count * 4000:
        tried += 1
        consider([float(rng.uniform(lo, hi)) for lo, hi in LIMITS])
    print(f"kadraj kestirimi: {len(poses)} aday / {tried} deneme "
          f"({100.0 * len(poses) / max(tried, 1):.1f}%)")
    print(f"  elenen        : kadraj disi {out_of_frame} | "
          f"cepheye cok yakin {too_frontal} "
          f"(egim < {min_tilt_deg:.1f} derece)")
    if tilts:
        t = np.array(tilts)
        print(f"  egim (derece) : min {t.min():.1f} | medyan {np.median(t):.1f} "
              f"| maks {t.max():.1f}")
    if len(poses) < count:
        print(f"  ⚠ {count} istendi, {len(poses)} bulundu — kisitlar havuzu "
              "daraltiyor; --count dusur ya da tahtayi tasi.")
    os.makedirs(os.path.dirname(POOL_OUT), exist_ok=True)
    json.dump({"note": "hand-eye poz kumesi aramasi icin aday havuz; "
                       "kadrajda kalacagi FK ile kestirildi, cepheden bakan "
                       "pozlar elendi",
               "seed": seed, "limits": LIMITS,
               "min_tilt_deg": min_tilt_deg,
               "rejected": {"out_of_frame": out_of_frame,
                            "too_frontal": too_frontal},
               "tilt_deg": tilts,
               "reference": os.path.relpath(reference, REPO),
               "poses": poses}, open(POOL_OUT, "w"), indent=1)
    print(f"{len(poses)} aday yazildi: {os.path.relpath(POOL_OUT, REPO)}")
    print("Yakalama: python3 scripts/sim_hand_eye_capture.py "
          f"--poses {os.path.relpath(POOL_OUT, REPO)} "
          "--out runs/hand_eye/pool_samples.json")


def k_samples(A_list, trials, sigma_mm=PROBE_NOISE_MM, seed=11, sigma_deg=0.0):
    """Her gurultu cekilisi icin k = |X_cozulen - X_gercek| / gurultu.

    B tarafi bilinen X ve sabit tahtadan kurulur, yani veri tanim geregi
    kusursuz tutarlidir; olculen tek sey poz geometrisinin gurultuyu ne kadar
    buyuttugu. Dagilim doner — tek sayi degil, cunku kotu kosullu kumede
    cekilisler arasi fark 10 katı bulabiliyor.
    """
    rng = np.random.default_rng(seed)
    Xinv = np.linalg.inv(X_TRUE)
    truth = X_TRUE[:3, 3] * 1000.0
    errs = []
    for _ in range(trials):
        B_list = []
        for A in A_list:
            B = (Xinv @ np.linalg.inv(A) @ BOARD).copy()
            B[:3, 3] += rng.normal(0, sigma_mm / 1000.0, 3)
            # DONME GURULTUSU 2026-08-14'te EKLENDI ve baskin cikti. Kapi bir
            # sure yalniz otelemeyi enjekte etti; olculdu: 0.41 mm oteleme tek
            # basina X hatasini 0.45 mm veriyor, uzerine YALNIZ 0.5 derece donme
            # eklemek 2.94 mm yapiyor -- gercek veride olculen 2.81 mm'nin
            # kendisi. Yani tahtanin ORYANTASYON hatasi baskin kanaldir ve
            # oteleme yaninda neredeyse onemsizdir.
            if sigma_deg > 0.0:
                axis = rng.normal(size=3)
                axis /= np.linalg.norm(axis)
                delta, _ = cv2.Rodrigues(axis * np.radians(rng.normal(0, sigma_deg)))
                B[:3, :3] = delta @ B[:3, :3]
            B_list.append(B)
        try:
            X = solve(A_list, B_list, cv2.CALIB_HAND_EYE_PARK)
        except cv2.error:
            return np.array([np.inf])
        errs.append(np.linalg.norm(X[:3, 3] * 1000.0 - truth))
    return np.array(errs) / sigma_mm


def amplification(A_list, trials=SEARCH_TRIALS, seed=11):
    """Arama vekili: ortalama k. Ucuz, siralamayi dogru veriyor."""
    return float(k_samples(A_list, trials, seed=seed).mean())


def expected_error_mm(A_list, noise_mm, noise_deg, trials=GATE_TRIALS, seed=101):
    """Beklenen X hatasi (mm), OLCULEN iki gurultuyle birlikte.

    k ile karistirilmamali: k bir buyutme katsayisidir ve yalniz oteleme
    gurultusune gore tanimliydi. Donme gurultusu baskin cikinca 'k x taban'
    carpimi anlamini yitirdi -- bu fonksiyon dogrudan hatayi olcer.
    """
    rng = np.random.default_rng(seed)
    Xinv = np.linalg.inv(X_TRUE)
    truth = X_TRUE[:3, 3] * 1000.0
    errors = []
    for _ in range(trials):
        B_list = []
        for A in A_list:
            B = (Xinv @ np.linalg.inv(A) @ BOARD).copy()
            B[:3, 3] += rng.normal(0, noise_mm / 1000.0, 3)
            if noise_deg > 0.0:
                axis = rng.normal(size=3)
                axis /= np.linalg.norm(axis)
                delta, _ = cv2.Rodrigues(axis * np.radians(rng.normal(0, noise_deg)))
                B[:3, :3] = delta @ B[:3, :3]
            B_list.append(B)
        try:
            X = solve(A_list, B_list, cv2.CALIB_HAND_EYE_PARK)
        except cv2.error:
            return {"mean": float("inf"), "p90": float("inf"), "max": float("inf")}
        errors.append(np.linalg.norm(X[:3, 3] * 1000.0 - truth))
    e = np.array(errors)
    return {"mean": float(e.mean()), "median": float(np.median(e)),
            "p90": float(np.quantile(e, 0.90)), "max": float(e.max())}


def gate_k(A_list):
    """Kapi olcumu: GATE_TRIALS cekilisin p90'i, artı tanimlayici istatistik."""
    e = k_samples(A_list, GATE_TRIALS, seed=101)
    return {"mean": float(e.mean()), "median": float(np.median(e)),
            "p90": float(np.quantile(e, GATE_QUANTILE)), "max": float(e.max())}


def gate_verdict(stats, k_gate=K_GATE):
    """Return the declared gate verdict; only the p90 statistic is normative."""
    return "GECTI" if stats["p90"] <= k_gate else "KALDI"


def select(sample_path, size, seed, restarts, noise_floor_mm,
           max_inplane_deg=MAX_INPLANE_DIFF_DEG, set_out=None,
           min_tilt_deg=MIN_TILT_DEG):
    """Aday havuzdan k'yi en kucuk yapan alt kumeyi arar.

    Girdi iki bicimde olabilir:
      * yakalanmis ornekler (`samples`) — A olculmus pozlardan gelir,
      * kadraj kestirimli havuz (`poses`) — A, dogrulanmis FK'den gelir.
    Ikisinde de secim yalniz A'ya bakar; kamera verisi secime girmez.
    """
    payload = json.load(open(sample_path))
    if "samples" in payload:
        samples = payload["samples"]
        A = [rt_to_matrix(quat_to_rotation(*s["base_to_wrist"]["quat_xyzw"]),
                          s["base_to_wrist"]["xyz"]) for s in samples]
    else:
        samples = [{"pose_index": i, "joints": q,
                    "base_to_wrist": {"xyz": [float(v) for v in fk(q)[:3, 3]],
                                      "quat_xyzw": R2q(fk(q)[:3, :3])}}
                   for i, q in enumerate(payload["poses"])]
        A = [fk(q) for q in payload["poses"]]
    n = len(A)
    print(f"havuz : {n} poz ({os.path.relpath(sample_path, REPO)})")
    if n < size:
        raise SystemExit(f"{size} poz istendi, havuzda {n} var.")

    # Tahtanin her pozdaki goruntu-duzlemi yonelimi. OLCULMUS cam_to_board
    # varsa o kullanilir; yoksa yer gercegi BOARD ile kurulur (k_samples de
    # ayni kaynagi kullaniyor, yani ikisi ayni dunyayi tarif eder).
    Xinv = np.linalg.inv(X_TRUE)
    B_list = []
    for s_i, A_i in zip(samples, A):
        meas = s_i.get("cam_to_board")
        if meas:
            B_list.append(rt_to_matrix(quat_to_rotation(*meas["quat_xyzw"]),
                                       meas["xyz"]))
        else:
            B_list.append(Xinv @ np.linalg.inv(A_i) @ BOARD)
    spin = [board_inplane_deg(B) for B in B_list]
    tilt = [board_tilt_deg(B) for B in B_list]
    print(f"egim (derece)          : min {min(tilt):.1f} | "
          f"medyan {float(np.median(tilt)):.1f} | maks {max(tilt):.1f}")

    # Egim kapisi burada da UYGULANIR, yalnizca raporlanmaz. Kapi eklenmeden
    # once uretilmis havuzlar mevcut ve onlarin buyuk cogunlugu cepheden
    # bakiyor; raporlayip secmek, kapiyi hic koymamakla ayni sonucu verir.
    if min_tilt_deg > 0.0:
        allowed = [i for i in range(n) if tilt[i] >= min_tilt_deg]
        print(f"cephe kapisi           : egim >= {min_tilt_deg:.0f} derece — "
              f"{len(allowed)}/{n} poz uygun, {n - len(allowed)} elendi")
        if len(allowed) < size:
            raise SystemExit(
                f"{size} poz istendi ama egim kapisini ({min_tilt_deg:.0f} "
                f"derece) yalniz {len(allowed)} poz geciyor.\n"
                "Bu havuz cephe kapisindan ONCE uretilmis olabilir; "
                "pool komutunu yeniden kosturmak gerekir.\n"
                "Kapiyi --min-tilt-deg 0 ile kapatmak, duzlemsel PnP'nin "
                "dejenere bolgesini geri acar (0 derecede 1.77 derece poz "
                "hatasi, reprojeksiyon bunu GOSTERMEZ).")
    else:
        allowed = list(range(n))
        print("cephe kapisi           : KAPALI (--min-tilt-deg 0)")

    def compatible(indices, candidate=None):
        """Duzlem-ici donme kisiti: kumedeki EN BUYUK ikili fark <= tavan."""
        if max_inplane_deg <= 0.0:
            return True
        pool_i = list(indices) + ([candidate] if candidate is not None else [])
        for a in range(len(pool_i)):
            for b in range(a + 1, len(pool_i)):
                if inplane_diff_deg(spin[pool_i[a]], spin[pool_i[b]]) > max_inplane_deg:
                    return False
        return True

    def greedy_feasible(order):
        chosen = []
        for i in order:
            if compatible(chosen, i):
                chosen.append(i)
                if len(chosen) == size:
                    break
        return chosen

    if max_inplane_deg <= 0.0:
        print("duzlem-ici donme kisiti: KAPALI (--max-inplane-deg 0)")
        print("  ⚠ 6x9 izgarada ters siralama nativ ile BIREBIR ayni "
              "reprojeksiyonu verir; marj kurali bunu yakalayamaz.")
    else:
        print(f"duzlem-ici donme kisiti: ikili fark <= {max_inplane_deg:.0f} derece")
    print(f"tum havuz              : k(ort) = {amplification(A):.2f}")
    k_gate = TARGET_MM / noise_floor_mm
    print(f"ilan edilen kapi       : k(p90) <= {k_gate:.2f} "
          f"({TARGET_MM} mm hedef / {noise_floor_mm} mm gurultu tabani)")
    if abs(noise_floor_mm - DEFAULT_NOISE_FLOOR_MM) < 1e-9:
        print("  ⚠ taban PROVANIN 5 pozundan devralindi. Aranmis kume daha genis")
        print("    donus cesitliligi tasiyor ve tabani BUYUTUYOR (2026-08-14: 0.41 ->")
        print("    1.38 mm). Kumenin kendi yakalamasindan olcup --noise-floor ile ver.")
    print()

    rng = np.random.default_rng(seed)
    best, best_k = None, float("inf")
    for r in range(restarts):
        # Baslangic kumesi de KISITLI olmak zorunda: rastgele bir alt kume
        # neredeyse her zaman kisiti ihlal eder ve yerel arama oradan cikamaz.
        order = [int(i) for i in rng.permutation(len(allowed))]
        idx = greedy_feasible([allowed[i] for i in order])
        if len(idx) < size:
            reachable = max(
                len(greedy_feasible([allowed[int(i)]
                                     for i in rng.permutation(len(allowed))]))
                for _ in range(200))
            raise SystemExit(
                f"{size} poz istendi ama duzlem-ici donme kisiti "
                f"(<= {max_inplane_deg:.0f} derece) altinda havuzdan en fazla "
                f"~{reachable} poz bir arada secilebiliyor.\n"
                "Secenekler: --size dusur, havuzu buyut (pool --count), ya da "
                "tahtayi/kolu goruntude daha az doner bir yerlesime tasi.\n"
                "Kisiti --max-inplane-deg ile gevsetmek, yakalanamayan bir "
                "kose eslesmesi riskini geri getirir.")
        k = amplification([A[i] for i in idx])
        improved = True
        while improved:                      # yerel arama: birer birer degistir
            improved = False
            for slot in range(size):
                for cand in allowed:
                    if cand in idx:
                        continue
                    trial = list(idx)
                    trial[slot] = cand
                    if not compatible(trial):
                        continue
                    kt = amplification([A[i] for i in trial])
                    if kt < k - 1e-4:
                        idx, k, improved = trial, kt, True
        if k < best_k:
            best, best_k = list(idx), k
            print(f"  restart {r + 1}/{restarts}: k = {k:.2f}  (en iyi)")
        else:
            print(f"  restart {r + 1}/{restarts}: k = {k:.2f}")

    print()
    stats = gate_k([A[i] for i in best])
    verdict = gate_verdict(stats, k_gate)
    print(f"secilen {size} poz, {GATE_TRIALS} cekilis:")
    print(f"  k ortalama {stats['mean']:.2f} | medyan {stats['median']:.2f} | "
          f"p90 {stats['p90']:.2f} | maks {stats['max']:.2f}")
    print(f"  p90 x gurultu tabani = {stats['p90'] * noise_floor_mm:.2f} mm "
          f"(hedef {TARGET_MM} mm)")
    print(f"kapi (k p90 <= {k_gate:.2f}) : {verdict}")
    chosen_spin = [spin[i] for i in best]
    chosen_tilt = [tilt[i] for i in best]
    spread = inplane_spread_deg(chosen_spin)
    print(f"secilen kumede egim   : min {min(chosen_tilt):.1f} derece "
          f"(cephe kapisi {MIN_TILT_DEG:.0f})")
    limit = (f"tavan {max_inplane_deg:.0f}" if max_inplane_deg > 0
             else "KISIT KAPALI")
    print(f"secilen kumede donme  : en buyuk ikili fark {spread:.1f} derece "
          f"({limit})")
    print("  NOT: bu SENTETIK gurultuyle olculen duyarliliktir. Kume ayrica")
    print("  GERCEK veriyle sinanmalidir: solve_hand_eye.py --samples <gated_samples>")

    chosen = [samples[i] for i in best]
    out = {
        "note": "Duyarlilik kapisini gecmek uzere ARANMIS poz kumesi (#8). "
                "Secim yalniz A pozlarina bakar; olculen kamera verisi "
                "dogrulama icin kullanilir, secim icin degil.",
        "gate": {
            "target_mm": TARGET_MM,
            "noise_floor_mm": noise_floor_mm,
            "noise_floor_source": ("provadan devralindi (SUPHELI)"
                                   if abs(noise_floor_mm - DEFAULT_NOISE_FLOOR_MM) < 1e-9
                                   else "bu kumeden olculdu"),
            "k_gate_p90": round(k_gate, 2),
            "k_mean": round(stats["mean"], 2),
            "k_median": round(stats["median"], 2),
            "k_p90": round(stats["p90"], 2),
            "k_max": round(stats["max"], 2),
            "expected_error_p90_mm": round(stats["p90"] * noise_floor_mm, 2),
            "verdict": verdict,
            "probe_noise_mm": PROBE_NOISE_MM,
            "gate_trials": GATE_TRIALS,
            "search_trials": SEARCH_TRIALS,
        },
        "geometry_constraints": {
            "min_tilt_deg": MIN_TILT_DEG,
            "tilt_deg": [round(v, 2) for v in chosen_tilt],
            "tilt_min_deg": round(min(chosen_tilt), 2),
            "max_inplane_diff_deg": max_inplane_deg,
            "inplane_deg": [round(v, 2) for v in chosen_spin],
            "inplane_spread_deg": round(spread, 2),
            "note": "egim 0 = tam cepheden (dejenere); duzlem-ici donme "
                    "goruntu duzleminde olculur ve 180 derece civari "
                    "farklar kose siralamasini sessizce ters cevirebilir",
        },
        "source_pool": os.path.relpath(sample_path, REPO),
        "poses": [{"pose_index": s.get("pose_index"),
                   "joints_rad": s.get("joints"),
                   "base_to_wrist": s["base_to_wrist"]} for s in chosen],
        "joints_rad": [s.get("joints") for s in chosen],
    }
    # Varsayilan hedef KANONIK kumedir; deneme kosulari onu ezmesin diye
    # --out ile baska bir yere yazilabilir.
    set_path = set_out or SET_OUT
    os.makedirs(os.path.dirname(set_path), exist_ok=True)
    json.dump(out, open(set_path, "w"), indent=1)
    print(f"yazildi         : {os.path.relpath(set_path, REPO)}")

    val = (os.path.splitext(set_path)[0] + "_samples.json" if set_out
           else os.path.join(REPO, "runs", "hand_eye", "gated_samples.json"))
    json.dump({"source": "gated subset of " + os.path.basename(sample_path),
               "samples": chosen}, open(val, "w"), indent=1)
    print(f"dogrulama girdisi: {os.path.relpath(val, REPO)}  "
          "(solve_hand_eye.py --samples ile gercek veriyle sina)")


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("pool")
    p.add_argument("--count", type=int, default=150)
    p.add_argument("--seed", type=int, default=5)
    p.add_argument("--min-tilt-deg", type=float, default=MIN_TILT_DEG,
                   help="tahtaya bu aciden daha CEPHEDEN bakan pozlar elenir; "
                        "0 = kapali (dejenere PnP riski geri gelir)")
    p.add_argument("--reference",
                   default=os.path.join(REPO, "runs", "hand_eye",
                                        "sim_samples.json"),
                   help="tahta pozu ve FK dogrulamasi icin onceki yakalama")
    s = sub.add_parser("select")
    s.add_argument("--samples", required=True)
    s.add_argument("--size", type=int, default=10)
    s.add_argument("--seed", type=int, default=5)
    s.add_argument("--restarts", type=int, default=4)
    s.add_argument("--out", default=None,
                   help="secilen kumenin yazilacagi yol; verilmezse KANONIK "
                        f"{os.path.relpath(SET_OUT, REPO)} EZILIR")
    s.add_argument("--min-tilt-deg", type=float, default=MIN_TILT_DEG,
                   help="tahtaya bu aciden daha CEPHEDEN bakan pozlar "
                        "SECILMEZ; 0 = kapali")
    s.add_argument("--max-inplane-deg", type=float,
                   default=MAX_INPLANE_DIFF_DEG,
                   help="pozlar arasi izin verilen en buyuk duzlem-ici donme "
                        "farki (derece); 0 = kapali")
    s.add_argument("--noise-floor", type=float, default=DEFAULT_NOISE_FLOOR_MM,
                   help="mm; ADAY KUMENIN KENDI yakalamasindan olculen tahta "
                        "sacilimi rms. Varsayilan provadan devralinmistir ve "
                        "genis kumelerde iyimserdir (bkz. dosya basi).")
    a = ap.parse_args()
    if a.cmd == "pool":
        pool(a.count, a.seed, a.reference, a.min_tilt_deg)
    else:
        select(a.samples, a.size, a.seed, a.restarts, a.noise_floor,
               a.max_inplane_deg, a.out, a.min_tilt_deg)


if __name__ == "__main__":
    main()
