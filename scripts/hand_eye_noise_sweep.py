#!/usr/bin/env python3
"""Hand-eye çözümünün gürültüye duyarlılığını poz kümesine göre ölçer.

Issue #8: sim provası ~9 mm sapma ölçtü. Konvansiyon ve zincir elendi
(`hand_eye_convention_check.py` X'i tam geri veriyor; yer gerçeği X ile
tahta 0.38--0.78 mm'ye yeniden kuruluyor). Kalan soru: sub-milimetrelik
tutarsızlık nasıl 9 mm oluyor?

Cevap POZ GEOMETRİSİ. Aynı gürültü, çeşitli bilek pozlarında ~1:1 kalıyor;
sim'in gerçekte ulaşabildiği pozlarda kat kat büyüyor.

⚠ DÜZELTME (2026-08-12): bu script bir sürüm boyunca **tek gürültü çekilişi**
basıyordu ve sim kümesi için k = 11.4 okunuyordu. O bir çekilişin değeriydi,
kümenin değil: 300 çekilişte aynı küme ortalama 3.85, medyan 3.50, p90 6.41,
maks 13.73 veriyor. Kötü koşullu kümede sonuç gürültünün yönüne kuvvetle bağlı,
o yüzden artık her hücre çok çekilişin medyanı ve p90'ı olarak basılıyor ve
kapı p90 üzerinden kuruluyor (`hand_eye_pose_search.py`).

İki sütun da BU SCRIPT tarafından, aynı koşuda üretilir:

  * ÇEŞİTLİ  — `hand_eye_convention_check.WRIST`, eksen çeşitliliği için
    kasten geniş kurulmuş sentetik pozlar.
  * SİM      — `data/hand_eye/sim_wrist_poses.json`, kolun tahtayı kadrajda
    tutarken gerçekten ulaşabildiği pozlar. Bu dosya prova koşusundan
    `hand_eye_pose_set_export.py` ile çıkarılır ve TAKİPTEDİR — daha önce
    yalnız gitignore'lu `runs/` altında olduğu için bu sütun depoda yeniden
    üretilemiyordu.

⚠ Yayılım (cross-method spread) BU DUYARLILIĞIN ÖLÇÜSÜ DEĞİLDİR. Daha önce
onu koşullanma vekili sayıp "koşullanma elendi" sonucuna vardım; yanlıştı.
Yöntemlerin birbirine yakın olması, tahminin gürültüye dayanıklı olduğunu
göstermez.

Gerçek kol için sonuç: bir poz kümesi kabul edilmeden önce DUYARLILIĞI
ölçülmeli — bilinen gürültü enjekte edilip hatanın ne kadar büyüdüğüne
bakılmalı. Tahta saçılımı da yöntem uyumu da bunu yakalamıyor.

Not: az örnekle (5--8) tek tek satırlar monoton olmayabilir; eğilime bak.

    python3 scripts/hand_eye_noise_sweep.py
"""

import json, os, re, subprocess, sys, tempfile
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
src = open(os.path.join(REPO, "scripts",
                        "hand_eye_convention_check.py")).read()
exec(src.split("def main()")[0])

POSE_SET_FILE = os.path.join(REPO, "data", "hand_eye", "sim_wrist_poses.json")
NOISE_MM = (0.0, 0.1, 0.2, 0.4, 0.8, 1.5)
TRUTH = np.array([20., 0., 60.])
DRAWS = 25          # her hücre bu kadar gürültü çekilişinin özeti


def q2R(q):
    x, y, z, w = np.array(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def sim_wrist():
    """Provanın gerçek pozları. Yoksa sütun ATLANIR, uydurulmaz."""
    if not os.path.exists(POSE_SET_FILE):
        return None
    data = json.load(open(POSE_SET_FILE))
    return [(q2R(p["base_to_wrist"]["quat_xyzw"]),
             np.array(p["base_to_wrist"]["xyz"], float))
            for p in data["poses"]], data


def one_draw(wrist, sig, rng):
    """Tek gürültü çekilişi: hata (mm) ve çözücünün tahta saçılımı (mm)."""
    Xinv = np.linalg.inv(X_TRUE)
    samples = []
    for R, t in wrist:
        A = T(R, t)
        B = (Xinv @ np.linalg.inv(A) @ BOARD).copy()
        B[:3, 3] += rng.normal(0, sig / 1000.0, 3)
        samples.append({
            "base_to_wrist": {"xyz": [float(v) for v in A[:3, 3]],
                              "quat_xyzw": R2q(A[:3, :3])},
            "cam_to_board": {"xyz": [float(v) for v in B[:3, 3]],
                             "quat_xyzw": R2q(B[:3, :3])}})
    f = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
    json.dump({"samples": samples}, f)
    f.close()
    try:
        out = subprocess.run(
            [sys.executable, os.path.join(REPO, "scripts", "solve_hand_eye.py"),
             "--samples", f.name], capture_output=True, text=True).stdout
    finally:
        os.unlink(f.name)
    for line in out.splitlines():
        if "PARK" in line:
            m = re.search(r"t=\[([-\d., ]+)\].*rms ([\d.]+) mm", line)
            v = np.array([float(x) for x in m.group(1).split(",")])
            return float(np.linalg.norm(v - TRUTH)), float(m.group(2))
    raise SystemExit("solve_hand_eye PARK satirini vermedi:\n" + out[-500:])


def solve(wrist, sig, seed=3, draws=DRAWS):
    """DRAWS çekilişin medyanı ve p90'ı. Tek çekiliş kümeyi TARİF ETMEZ."""
    rng = np.random.default_rng(seed)
    errs, scat = [], []
    for _ in range(draws):
        e, s = one_draw(wrist, sig, rng)
        errs.append(e)
        scat.append(s)
    return (float(np.median(errs)), float(np.quantile(errs, 0.90)),
            float(np.median(scat)))


def main():
    loaded = sim_wrist()
    print(f"cesitli pozlar : {len(WRIST)} (sentetik, eksen cesitliligi icin)")
    if loaded is None:
        print(f"sim pozlari    : YOK — {os.path.relpath(POSE_SET_FILE, REPO)} "
              "bulunamadi, o sutun atlaniyor")
        print("                 (uretmek icin: ./start_simulation.sh --hand-eye"
              " --headless, sonra")
        print("                  sim_hand_eye_capture.py + "
              "hand_eye_pose_set_export.py)")
        sim, meta = None, None
    else:
        sim, meta = loaded
        cap = meta["capture"]
        print(f"sim pozlari    : {len(sim)}/{cap['candidate_poses']} "
              f"(kadrajda kalan), zincir "
              f"{meta['chain_check']['board_scatter_rms_mm']} mm rms")
    print(f"her hucre {DRAWS} gurultu cekilisi: medyan (p90)")
    print()
    print("gurultu(mm) |     cesitli: hata |    tahta |         sim: hata |"
          "    tahta   [mm]")
    for sig in NOISE_MM:
        e_v, p_v, s_v = solve(WRIST, sig)
        row = f"{sig:11.2f} | {e_v:8.2f} ({p_v:6.2f}) | {s_v:8.2f}"
        if sim is not None:
            e_s, p_s, s_s = solve(sim, sig)
            row += f" | {e_s:8.2f} ({p_s:6.2f}) | {s_s:8.2f}"
        print(row)
    print()
    print("OKUNUSU: ayni gurultu, iki poz kumesi. Sim sutunu daha hizli")
    print("  buyuyorsa fark cozucuden veya veriden degil, POZ GEOMETRISINDEN")
    print("  gelir. 'tahta' sutunu (board scatter) bu buyumeyi TAKIP ETMEZ —")
    print("  gercek kolda yer gercegi olmadigi icin kapi olarak kullanilamaz.")
    print()
    print("⚠ p90 sutunu bosuna degil: kotu kosullu kumede hata gurultunun HANGI")
    print("  YONE dustugune kuvvetle bagli. Bu scriptin onceki surumu TEK")
    print("  cekilis basiyordu ve sim kumesi icin k=11.4 gibi okunuyordu;")
    print("  300 cekiliste ayni kume ortalama 3.85, medyan 3.50, p90 6.41,")
    print("  maks 13.73 veriyor. Tek cekilise dayanan bir kapi yaniltir.")


if __name__ == "__main__":
    main()
