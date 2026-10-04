#!/usr/bin/env python3
"""solve_hand_eye.py'ın AX=XB konvansiyonunu sentetik veriyle sınar.

NEDEN: sim provası ~9 mm sapma ölçtü (issue #8) ve zincir elendi — yer gerçeği
X ile sabit tahta 0.41 mm rms'e yeniden kuruluyor, yani FK/TF/PnP sağlam.
Sıradaki aday: örneklerin kurulma şekli ile çözücünün beklediği konvansiyonun
uyuşmaması. Bu test onu eler; kalan sebep poz geometrisidir
(`hand_eye_noise_sweep.py`).

⚠ Bu dosyanın önceki sürümü gerekçe olarak "koşullanma elendi, çünkü yöntem
yayılımı 17 kat oynarken hata kıpırdamadı" diyordu. Yanlış: yayılım
koşullanmanın vekili değildir. Koşullanma elenmedi, tam tersine sapmanın
sebebi çıktı.

YÖNTEM: bilinen bir X ve sabit bir tahta pozundan, GÜRÜLTÜSÜZ örnekler üretilir:

    T_base_board = A · X · B    =>    B = X⁻¹ · A⁻¹ · T_base_board

Bu bağıntı gerçek sim verisinde ölçülerek doğrulandı, yani üretilen veri
fiziksel olarak tutarlıdır. Çözücü bu veriden X'i geri veremiyorsa sorun veride
değil, konvansiyondadır — ve hangi tarafın yanlış olduğu ayrılmış olur.

    python3 scripts/hand_eye_convention_check.py
"""

import json
import math
import os
import subprocess
import sys
import tempfile

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def T(R, t):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


def R2q(R):
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = (0.25 * s, (R[2, 1] - R[1, 2]) / s,
                      (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s)
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w, x, y, z = ((R[2, 1] - R[1, 2]) / s, 0.25 * s,
                          (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s)
        elif i == 1:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w, x, y, z = ((R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
                          0.25 * s, (R[1, 2] + R[2, 1]) / s)
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w, x, y, z = ((R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
                          (R[1, 2] + R[2, 1]) / s, 0.25 * s)
    return [float(x), float(y), float(z), float(w)]


# Sim provasının yer gerçeğiyle aynı X: link_5 -> camera_optical_frame.
X_TRUE = T(rpy(0, 0.2, 0), [0.02, 0.0, 0.06]) @ T(rpy(-1.5708, 0, -1.5708),
                                                  [0, 0, 0])
# Sabit tahta, base frame'de. Değeri önemsiz; sabit olması önemli.
BOARD = T(rpy(0.1, -0.2, 0.4), [-0.58, -0.14, 0.26])

# Bilek pozları: eksen çeşitliliği için kasten geniş ve karışık.
WRIST = [
    (rpy(0.0, 0.0, 0.0), [0.10, 0.00, 0.30]),
    (rpy(0.3, 0.1, -0.2), [0.12, 0.04, 0.28]),
    (rpy(-0.25, 0.35, 0.15), [0.08, -0.05, 0.32]),
    (rpy(0.15, -0.4, 0.5), [0.14, 0.02, 0.27]),
    (rpy(-0.5, 0.2, -0.35), [0.09, -0.03, 0.31]),
    (rpy(0.45, -0.15, 0.25), [0.11, 0.06, 0.29]),
    (rpy(-0.2, -0.3, -0.5), [0.13, -0.02, 0.33]),
    (rpy(0.35, 0.45, 0.1), [0.07, 0.03, 0.30]),
]


def main():
    Xinv = np.linalg.inv(X_TRUE)
    samples = []
    for R, t in WRIST:
        A = T(R, t)
        B = Xinv @ np.linalg.inv(A) @ BOARD      # tanim geregi tutarli
        # Yeniden kurulum kontrolu: uretilen veri gercekten tutarli mi?
        assert np.allclose(A @ X_TRUE @ B, BOARD, atol=1e-9)
        samples.append({
            "base_to_wrist": {"xyz": [float(v) for v in A[:3, 3]],
                              "quat_xyzw": R2q(A[:3, :3])},
            "cam_to_board": {"xyz": [float(v) for v in B[:3, 3]],
                             "quat_xyzw": R2q(B[:3, :3])},
        })

    print(f"sentetik ornek : {len(samples)}  (gurultu YOK, tam tutarli)")
    print(f"yer gercegi X  : t = {np.round(X_TRUE[:3, 3] * 1000, 2)} mm")
    print()

    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
        json.dump({"source": "synthetic convention check",
                   "samples": samples}, f)
        path = f.name
    try:
        out = subprocess.run(
            [sys.executable, os.path.join(REPO, "scripts", "solve_hand_eye.py"),
             "--samples", path],
            capture_output=True, text=True)
        print(out.stdout.strip() or out.stderr.strip()[-800:])
    finally:
        os.unlink(path)

    print()
    print("OKUNUŞU:")
    print("  Çözücü X'i (20.00, 0.00, 60.00) mm olarak geri veriyorsa örnek")
    print("  kurma konvansiyonu ile solve_hand_eye UYUMLUDUR; o zaman sim'deki")
    print("  9 mm başka bir yerden gelir ve bu test onu ELER.")
    print("  Geri veremiyorsa — veri kusursuz tutarlı olduğu hâlde — sapma")
    print("  KONVANSİYON uyuşmazlığıdır ve kaynağı burada izole edilmiştir.")


if __name__ == "__main__":
    main()
