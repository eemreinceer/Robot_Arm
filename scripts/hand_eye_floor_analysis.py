#!/usr/bin/env python3
"""Gürültü tabanı neyin fonksiyonu? (D3-a, ÇEVRİMDIŞI — donanım gerekmez)

SORU
  2026-08-14'te ölçüldü: aynı zincir kontrolü provanın 5 pozunda 0.41 mm rms,
  aranmış 20 pozluk kümede 1.38 mm. Kapı bu yüzden yanıldı (bkz.
  `docs/hand_eye_pose_set_requirement.md`). Peki taban NEDEN büyüyor?

  İlk yazdığım gerekçe "geniş bilek dönüşleri FK/zero-offset/servo hatasını
  büyütüyor" idi. O gerekçe BU VERİ İÇİN YANLIŞ OLMALI: iki ölçüm de SİM'den
  geliyor, sim'de FK kusursuz ve servo pozisyon hatası yok. Dolayısıyla sim'de
  kalan tek aday GÖRÜŞ GEOMETRİSİ — tahtaya eğik/uzak bakan bir kamerada PnP
  kötü koşullanır.

  Bu script o hipotezi 399 pozluk havuzda sınar. Ayrım önemli: eğer taban
  görüş geometrisinden geliyorsa çözüm poz SEÇİMİNDEDİR (daha az eğik bak) ve
  bugün yapılabilir; donanımdan geliyorsa çözüm mekaniktedir ve kol başında
  çalışmak gerekir.

NASIL ÖLÇÜYOR
  Sim'de X (kamera montajı) YER GERÇEĞİ olarak bilindiği için her örnekten
  tahtanın base frame'indeki pozu hesaplanabilir:  T_i = A_i · X · B_i
  Tahta FİZİKSEL OLARAK SABİT, yani tüm T_i aynı olmalı. Aradaki fark ölçüm
  hatasıdır -- "zincir saçılımı" dediğimiz taban tam olarak budur.

  Her örneğin sapması, uydurulan ortalama tahta pozuna göre hesaplanır ve
  şunlarla ilişkilendirilir:
    * bilek dönüş açısı   (A_i'nin q=0'a göre dönme büyüklüğü)
    * bakış eğikliği      (kamera ekseni ile tahta normali arasındaki açı)
    * mesafe              (kamera-tahta)
    * reprojeksiyon       (PnP'nin kendi kalıntısı)

  ÇÜRÜTÜLEBİLİR: eğiklik ile sapma arasında ilişki YOKSA hipotez yanlıştır ve
  taban artışının sebebi başka yerdedir.

KULLANIM
  python3 scripts/hand_eye_floor_analysis.py
  python3 scripts/hand_eye_floor_analysis.py --samples runs/hand_eye/pool_samples.json
"""

import argparse
import json
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "scripts"))

from solve_hand_eye import rt_to_matrix, quat_to_rotation  # noqa: E402

_conv = open(os.path.join(REPO, "scripts",
                          "hand_eye_convention_check.py")).read()
exec(_conv.split("def main()")[0])          # rpy / T / R2q / X_TRUE / BOARD


def rotation_angle(R):
    """Dönme matrisinin açı büyüklüğü (rad)."""
    value = (np.trace(R) - 1.0) / 2.0
    return float(np.arccos(np.clip(value, -1.0, 1.0)))


def board_in_base(samples):
    """Sabit tahtanın base frame'deki pozu: T = A·X·B.

    Merkez ORTALAMA DEĞİL MEDYAN alınır. İlk sürüm ortalama kullanıyordu ve
    sayılar okunamaz çıktı: düzlemsel PnP'nin çift-çözüm belirsizliği yüzünden
    örneklerin bir kısmı 100-200 mm sapan "flip" pozlar veriyor, ortalama onlar
    tarafından çekiliyor ve SAĞLAM örnekler de merkeze uzak görünüyor. Her
    kutunun medyanının ~4.85 mm'de takılması bu kirlenmenin izidir.

    Medyan, örneklerin yarısından fazlası sağlam olduğu sürece flip'lerden
    etkilenmez -- ve burada öyle (399'un ~100'ü flip).
    """
    poses = []
    for sample in samples:
        A = rt_to_matrix(
            quat_to_rotation(*sample["base_to_wrist"]["quat_xyzw"]),
            sample["base_to_wrist"]["xyz"])
        B = rt_to_matrix(
            quat_to_rotation(*sample["cam_to_board"]["quat_xyzw"]),
            sample["cam_to_board"]["xyz"])
        poses.append(A @ X_TRUE @ B)

    translations = np.array([pose[:3, 3] for pose in poses])
    center = np.median(translations, axis=0)

    # Dönme kısmı yalnız medyana yakın (flip olmayan) örneklerden kurulur.
    residuals = np.linalg.norm(translations - center, axis=1)
    inliers = residuals <= max(np.median(residuals) * 3.0, 0.005)
    rotation_mean = np.mean([poses[i][:3, :3] for i in np.flatnonzero(inliers)], axis=0)
    U, _, Vt = np.linalg.svd(rotation_mean)

    board = np.eye(4)
    board[:3, :3] = U @ Vt
    board[:3, 3] = center
    return board, poses, int(inliers.sum())


def describe(values):
    array = np.asarray(values)
    return (f"{array.mean():6.2f} {np.median(array):6.2f} "
            f"{np.quantile(array, 0.90):6.2f} {array.max():6.2f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples",
                        default=os.path.join(REPO, "runs", "hand_eye",
                                             "pool_samples.json"))
    parser.add_argument("--bins", type=int, default=5)
    args = parser.parse_args()

    samples = json.load(open(args.samples))["samples"]
    board, poses, inliers = board_in_base(samples)
    print(f"ornek       : {len(samples)}  ({os.path.relpath(args.samples, REPO)})")
    print(f"tahta (base): {np.round(board[:3, 3], 4).tolist()} m "
          f"(medyan merkez; {inliers}/{len(samples)} ornek flip DEGIL)")

    rows = []
    for sample, pose in zip(samples, poses):
        A = rt_to_matrix(
            quat_to_rotation(*sample["base_to_wrist"]["quat_xyzw"]),
            sample["base_to_wrist"]["xyz"])
        # Sapma: bu örneğin gördüğü tahta pozu ile ortak tahta pozu arasındaki
        # mesafe. Tahta sabit olduğu için bu tamamen ölçüm hatasıdır.
        deviation_mm = float(np.linalg.norm(pose[:3, 3] - board[:3, 3]) * 1000.0)

        camera = A @ X_TRUE                    # base -> camera_optical_frame
        # Bakış eğikliği: kameranın optik ekseni (+z) ile tahta normali
        # arasındaki açı. 0 = tam cepheden, buyudukce egik.
        obliquity_deg = float(np.degrees(
            np.arccos(np.clip(abs(camera[:3, 2] @ board[:3, 2]), -1.0, 1.0))))
        distance_m = float(np.linalg.norm(
            np.array(sample["cam_to_board"]["xyz"])))
        wrist_deg = float(np.degrees(rotation_angle(A[:3, :3])))

        rows.append({
            "deviation_mm": deviation_mm,
            "obliquity_deg": obliquity_deg,
            "distance_m": distance_m,
            "wrist_deg": wrist_deg,
            "reproj_px": float(sample["reproj_px"]),
        })

    deviations = np.array([r["deviation_mm"] for r in rows])
    print(f"\nzincir sacilimi (tumu): rms {np.sqrt((deviations ** 2).mean()):.2f} mm, "
          f"medyan {np.median(deviations):.2f}, maks {deviations.max():.2f}")

    print("\n--- SAPMA, BAKIS EGIKLIGINE GORE ---")
    print("egiklik(deg)  n |   ort medyan    p90   maks   [mm] | reproj(px)")
    obliquity = np.array([r["obliquity_deg"] for r in rows])
    edges = np.quantile(obliquity, np.linspace(0, 1, args.bins + 1))
    for low, high in zip(edges[:-1], edges[1:]):
        mask = (obliquity >= low) & (obliquity <= high)
        if mask.sum() == 0:
            continue
        reproj = np.array([r["reproj_px"] for r in rows])[mask]
        print(f"{low:5.1f}-{high:5.1f} {mask.sum():4d} | "
              f"{describe(deviations[mask])} | {reproj.mean():.3f}")

    print("\n--- SAPMA, BILEK DONUS BUYUKLUGUNE GORE ---")
    print("donus(deg)    n |   ort medyan    p90   maks   [mm]")
    wrist = np.array([r["wrist_deg"] for r in rows])
    edges = np.quantile(wrist, np.linspace(0, 1, args.bins + 1))
    for low, high in zip(edges[:-1], edges[1:]):
        mask = (wrist >= low) & (wrist <= high)
        if mask.sum() == 0:
            continue
        print(f"{low:5.1f}-{high:5.1f} {mask.sum():4d} | {describe(deviations[mask])}")

    print("\n--- SAPMA, MESAFEYE GORE ---")
    print("mesafe(m)     n |   ort medyan    p90   maks   [mm]")
    distance = np.array([r["distance_m"] for r in rows])
    edges = np.quantile(distance, np.linspace(0, 1, args.bins + 1))
    for low, high in zip(edges[:-1], edges[1:]):
        mask = (distance >= low) & (distance <= high)
        if mask.sum() == 0:
            continue
        print(f"{low:5.2f}-{high:5.2f} {mask.sum():4d} | {describe(deviations[mask])}")

    print("\n--- KORELASYONLAR (Pearson, sapma ile) ---")
    for key in ("obliquity_deg", "wrist_deg", "distance_m", "reproj_px"):
        values = np.array([r[key] for r in rows])
        correlation = float(np.corrcoef(values, deviations)[0, 1])
        print(f"  {key:14s} r = {correlation:+.3f}")

    print("\nOKUNUSU: sim'de FK kusursuz ve servo hatasi YOK. Dolayisiyla buradaki")
    print("  sapma tamamen GORUS GEOMETRISI + PnP'dir. Egiklik/mesafe ile guclu")
    print("  iliski varsa taban artisi bir ALGI etkisidir ve poz secimiyle")
    print("  kucultulebilir. Iliski yoksa hipotez CURUR ve baska bir kaynak aranir.")
    print("  GERCEK KOLDA bunlara FK/zero_offset/servo hatasi EKLENIR; bu tablo")
    print("  o katkinin ALT SINIRIDIR, tamami degil.")


if __name__ == "__main__":
    main()
