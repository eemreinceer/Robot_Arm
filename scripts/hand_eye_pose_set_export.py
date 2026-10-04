#!/usr/bin/env python3
"""Sim provasının POZ KÜMESİNİ takipteki bir dosyaya çıkarır.

NEDEN VAR: `hand_eye_noise_sweep.py` iki poz kümesini karşılaştırıyor — kasten
çeşitli bilek pozları ile sim'in gerçekte ulaşabildiği pozlar. İkincisi
`runs/hand_eye/sim_samples.json`'dan geliyordu, ama `runs/` **gitignore'lu**:
yani tablonun ikinci sütunu depoda hiç kimse tarafından yeniden üretilemiyordu.
Bu script o kümeyi `data/hand_eye/sim_wrist_poses.json` olarak takibe alır.

Yalnız A tarafı (base -> wrist) saklanır. B tarafı zaten bilinen bir X ve sabit
tahtadan sentetik üretiliyor, dolayısıyla ölçülen kamera verisi gerekmiyor —
duyarlılığı belirleyen şey pozların GEOMETRİSİ.

Çıkarmadan önce zinciri yer gerçeği montajla doğrular: tahta sabit olduğuna
göre her örnek için A·X·B aynı yeri vermeli. Bu kapı geçmezse dosya yazılmaz,
çünkü geometrisi bozuk bir kümeyi referans olarak saklamak yanıltıcıdır.

Kullanım (prova koşusundan hemen sonra):

    ./start_simulation.sh --hand-eye --headless      # ayrı terminal
    python3 scripts/sim_hand_eye_capture.py
    python3 scripts/hand_eye_pose_set_export.py
"""

import json
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "scripts"))

_conv = open(os.path.join(REPO, "scripts",
                          "hand_eye_convention_check.py")).read()
exec(_conv.split("def main()")[0])                      # noqa: S102  (rpy/T/R2q)

IN = os.path.join(REPO, "runs", "hand_eye", "sim_samples.json")
OUT = os.path.join(REPO, "data", "hand_eye", "sim_wrist_poses.json")

# `start_simulation.sh --hand-eye` varsayılanı ile aynı montaj. Sim'de bu
# transform LAUNCH'TAN VERİLDİĞİ için yer gerçeğidir; gerçek kolda ölçülmedi.
MOUNT_XYZ = [0.02, 0.0, 0.06]
MOUNT_RPY = [0.0, 0.2, 0.0]
X_GT = T(rpy(*MOUNT_RPY), MOUNT_XYZ) @ T(rpy(-1.5708, 0, -1.5708), [0, 0, 0])

# Zincir kapısı. Prova koşuları 0.38--0.78 mm ölçtü; 2 mm'yi aşan bir saçılım
# poz kümesinin değil, yakalamanın bozuk olduğu anlamına gelir.
CHAIN_GATE_MM = 2.0


def q2R(q):
    x, y, z, w = np.array(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    if not os.path.exists(IN):
        raise SystemExit(f"{IN} yok — once sim_hand_eye_capture.py kosun.")
    data = json.load(open(IN))
    samples = data.get("samples") or []
    if len(samples) < 3:
        raise SystemExit(f"yalniz {len(samples)} ornek — poz kumesi cikarilmaz.")

    board = []
    for s in samples:
        A = T(q2R(s["base_to_wrist"]["quat_xyzw"]), s["base_to_wrist"]["xyz"])
        B = T(q2R(s["cam_to_board"]["quat_xyzw"]), s["cam_to_board"]["xyz"])
        board.append((A @ X_GT @ B)[:3, 3])
    board = np.array(board)
    resid = np.linalg.norm(board - board.mean(0), axis=1) * 1000.0
    rms, mx = float(np.sqrt((resid ** 2).mean())), float(resid.max())
    print(f"zincir kontrolu : rms {rms:.2f} mm, maks {mx:.2f} mm "
          f"({len(samples)} ornek)")
    if rms > CHAIN_GATE_MM:
        raise SystemExit(f"zincir kapisi GECMEDI (> {CHAIN_GATE_MM} mm) — "
                         "poz kumesi yazilmadi.")

    out = {
        "note": "Sim provasinin ULASILABILIR poz kumesi. Duyarlilik "
                "karsilastirmasinin ikinci sutunu bu kumeden uretilir.",
        "capture": {
            "launcher": "./start_simulation.sh --hand-eye --headless",
            "capture_script": "scripts/sim_hand_eye_capture.py",
            "camera_mount_xyz_m": MOUNT_XYZ,
            "camera_mount_rpy_rad": MOUNT_RPY,
            "candidate_poses": 18,
            "kept_poses": len(samples),
            "reprojection_px": [round(float(s.get("reproj_px", 0.0)), 3)
                                for s in samples],
        },
        "chain_check": {
            "method": "T_base_board = A * X_gt * B, sabit tahta",
            "board_scatter_rms_mm": round(rms, 2),
            "board_scatter_max_mm": round(mx, 2),
            "gate_mm": CHAIN_GATE_MM,
        },
        "poses": [{
            "pose_index": s.get("pose_index"),
            "joints_rad": s.get("joints"),
            "base_to_wrist": s["base_to_wrist"],
        } for s in samples],
    }
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(out, open(OUT, "w"), indent=1)
    print(f"yazildi         : {os.path.relpath(OUT, REPO)}  "
          f"({len(samples)} poz)")


if __name__ == "__main__":
    main()
