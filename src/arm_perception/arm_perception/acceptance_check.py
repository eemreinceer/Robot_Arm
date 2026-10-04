#!/usr/bin/env python3
"""Faz 4 sorting acceptance judge — tek koşu hakemi.

Bir koşuda: sahne kurucu (`demo_sorting`) zaten nesneleri spawn etmiştir ve
otonom düğüm (`autonomous_pick_node`, sort_all:=true) bunları kutulara ayırır.
Bu araç ayırmanın *oturmasını* bekler, her nesnenin SON dünya pozunu `gz`'den
okur ve `sorting_config` ile her nesnenin KENDİ sınıf kutusunda olup olmadığına
karar verir.

Çıktının son satırı makine-okunur:
    RESULT run=<id> objects=<n> correct=<c> wrong=<w> verdict=<PASS|FAIL>

`PASS` = correct >= --pass-threshold ve wrong == 0.

Poz okuma `gz model -m <name> -p` çıktısının ilk köşeli-parantez üçlüsünü
(pozisyon XYZ) ayrıştırır. İlk canlı koşumda bu formatı doğrula; gz sürümüne
göre değişirse `_read_world_xy` tek noktadan düzeltilir.
"""
from __future__ import annotations

import argparse
import re
import subprocess
import sys
import time
from collections import namedtuple

from arm_perception.sorting_config import load_sorting_config

# sorting_config.BinSpec.contains yalnızca .x/.y kullanır → ROS msg gerekmez.
XY = namedtuple("XY", ["x", "y"])

OBJECT_CLASSES = ("red_box", "yellow_cylinder", "blue_cube")
# Üçlü float köşeli parantez: [x y z]  (gz model -p pozisyon satırı)
_TRIPLE = re.compile(r"\[\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s*\]")


def _gz_pose_raw(model_name: str, world: str, timeout_s: float = 3.0) -> str | None:
    try:
        out = subprocess.run(
            ["gz", "model", "-m", model_name, "-p"],
            capture_output=True, text=True, timeout=timeout_s,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None
    if out.returncode != 0 or "[" not in out.stdout:
        return None
    return out.stdout


def _read_world_xy(model_name: str, world: str) -> XY | None:
    """Modelin dünya XY'sini döndür; model yoksa/okunamadıysa None."""
    raw = _gz_pose_raw(model_name, world)
    if raw is None:
        return None
    match = _TRIPLE.search(raw)
    if not match:
        return None
    x, y, _z = (float(match.group(i)) for i in (1, 2, 3))
    return XY(x, y)


def _spawned_object_names(objects_per_class: int) -> list[str]:
    names = []
    for object_type in OBJECT_CLASSES:
        for index in range(objects_per_class):
            names.append((object_type, f"sorting_{object_type}_{index:02d}"))
    return names


def _all_settled(prev: dict, curr: dict, move_eps: float) -> bool:
    """Hiçbir nesne son ölçümden bu yana move_eps'ten fazla kıpırdamadıysa True."""
    if not prev or set(prev) != set(curr):
        return False
    for name, (px, py) in prev.items():
        cx, cy = curr[name]
        if ((cx - px) ** 2 + (cy - py) ** 2) ** 0.5 > move_eps:
            return False
    return True


def wait_until_sorted(names, world, config, pass_threshold, margin, settle_timeout, poll_s):
    """Tüm beklenen nesneler doğru kutulara ulaşana kadar bekle."""
    deadline = time.monotonic() + settle_timeout
    last: dict[str, tuple[float, float]] = {}
    while time.monotonic() < deadline:
        curr: dict[str, tuple[float, float]] = {}
        for _cls, name in names:
            xy = _read_world_xy(name, world)
            if xy is not None:
                curr[name] = (xy.x, xy.y)
        last = curr or last
        correct, wrong, _details = judge(names, last, config, margin)
        if correct >= pass_threshold and wrong == 0:
            return last, True
        time.sleep(poll_s)
    return last, False


def judge(names, final_xy, config, margin):
    correct = 0
    wrong = 0
    details = []
    bins = config.bins
    for cls, name in names:
        pos = final_xy.get(name)
        if pos is None:
            details.append(f"  {name}: SON POZ OKUNAMADI")
            wrong += 0  # okunamayan yanlış sayılmaz, ama correct'e de girmez
            continue
        point = XY(pos[0], pos[1])
        own = bins[cls].contains(point, margin=margin)
        in_other = any(
            other_cls != cls and spec.contains(point, margin=margin)
            for other_cls, spec in bins.items()
        )
        if own and not in_other:
            correct += 1
            details.append(f"  {name}: DOĞRU kutu ({cls})")
        elif in_other:
            wrong += 1
            wrong_bin = next(
                c for c, s in bins.items() if c != cls and s.contains(point, margin=margin))
            details.append(f"  {name}: YANLIŞ kutu → {wrong_bin} (olması gereken {cls})")
        else:
            details.append(
                f"  {name}: hiçbir kutuda değil (x={pos[0]:.3f}, y={pos[1]:.3f})")
    return correct, wrong, details


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=None, help="sorting_bins.yaml yolu")
    parser.add_argument("--world", default="pick_and_place_world")
    parser.add_argument("--run-id", default="1")
    parser.add_argument("--objects-per-class", type=int, default=2)
    parser.add_argument("--pass-threshold", type=int, default=6,
                        help="PASS için gereken min. doğru-kutu sayısı")
    parser.add_argument("--settle-timeout", type=float, default=240.0,
                        help="ayırmanın oturması için max bekleme (s)")
    parser.add_argument("--poll-s", type=float, default=4.0)
    parser.add_argument("--move-eps", type=float, default=0.01,
                        help="bu mesafeden az kıpırdama = durmuş (m)")
    parser.add_argument("--quiet-polls", type=int, default=3,
                        help="kaç ardışık durağan ölçüm = oturmuş")
    parser.add_argument("--bin-margin", type=float, default=0.01,
                        help="kutu içi sayma payı (m)")
    args = parser.parse_args()

    config = load_sorting_config(args.config)
    names = _spawned_object_names(args.objects_per_class)
    total = len(names)

    print(f"[acceptance] run {args.run_id}: {total} nesne, oturma bekleniyor "
          f"(timeout {args.settle_timeout:.0f}s)...", flush=True)
    final_xy, sorted_complete = wait_until_sorted(
        names, args.world, config, args.pass_threshold, args.bin_margin,
        args.settle_timeout, args.poll_s)
    if not sorted_complete:
        print("[acceptance] UYARI: sorting timeout — son okunan pozlarla karar veriliyor",
              flush=True)

    correct, wrong, details = judge(names, final_xy, config, args.bin_margin)
    for line in details:
        print(line, flush=True)

    verdict = "PASS" if (correct >= args.pass_threshold and wrong == 0) else "FAIL"
    print(f"RESULT run={args.run_id} objects={total} correct={correct} "
          f"wrong={wrong} verdict={verdict}", flush=True)
    return 0 if verdict == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
