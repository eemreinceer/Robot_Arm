#!/usr/bin/env python3
"""Faz 4 real grasp sorting acceptance judge.

Bir koşuda: sahne kurucu (`demo_sorting`) nesneleri spawn eder,
otonom düğüm (`autonomous_pick_node`, sort_all:=true, fast_sort:=false)
nesneleri fiziksel olarak kavrayıp taşır.
Bu araç ayırmanın oturmasını bekler ve nesnelerin pozlarını okuyarak
fiziksel tutuş hedeflerini değerlendirir.

Kabul Kapıları:
1. Doğru Kutu: Nesne kendi sınıf kutusunda.
2. Fırlatma Yok: Nesne çalışma alanından uçmamış.
3. Çökme Yok: Log dosyasında segfault/ODE/assert yok.
"""

import argparse
import re
import subprocess
import sys
import time
from collections import namedtuple

from arm_perception.sorting_config import load_sorting_config

XY = namedtuple("XY", ["x", "y"])
XYZ = namedtuple("XYZ", ["x", "y", "z"])

OBJECT_CLASSES = ("red_box", "yellow_cylinder", "blue_cube")
_TRIPLE = re.compile(r"\[\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s*\]")

def _gz_pose_raw(model_name: str, timeout_s: float = 3.0) -> str | None:
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

def _read_world_xyz(model_name: str) -> XYZ | None:
    raw = _gz_pose_raw(model_name)
    if raw is None:
        return None
    match = _TRIPLE.search(raw)
    if not match:
        return None
    x, y, z = (float(match.group(i)) for i in (1, 2, 3))
    return XYZ(x, y, z)

def _spawned_object_names(objects_per_class: int) -> list[tuple[str, str]]:
    names = []
    for object_type in OBJECT_CLASSES:
        for index in range(objects_per_class):
            names.append((object_type, f"sorting_{object_type}_{index:02d}"))
    return names

def _distance(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5


def _placement_candidate(cls, xyz, config, margin, placed_z_min, placed_z_max):
    point = XY(xyz.x, xyz.y)
    z = xyz.z

    # Fırlatma kontrolü sadece nesne bırakma yüksekliğinin altına indikten sonra
    # anlamlıdır; transit sırasında kol nesneyi çalışma alanı dışına taşıyabilir.
    if z < placed_z_max and (abs(point.x) >= 0.9 or abs(point.y) >= 0.9 or z < -0.2):
        return "fling", None

    if not (placed_z_min <= z <= placed_z_max):
        return None, None

    own = config.bins[cls].contains(point, margin=margin)
    other_bin = None
    for other_cls, spec in config.bins.items():
        if other_cls != cls and spec.contains(point, margin=margin):
            other_bin = other_cls
            break

    if own and not other_bin:
        return "correct", cls
    if other_bin:
        return "wrong", other_bin
    return None, None


def wait_until_sorted(
    names, config, pass_threshold, margin, per_object_timeout, poll_s,
    stable_samples=3, stable_epsilon=0.02, placed_z_min=0.55, placed_z_max=0.75,
):
    total = len(names)
    deadline = time.monotonic() + per_object_timeout * total

    status = {name: "pending" for _, name in names}
    final_xyz = {}
    stable_state = {name: {"candidate": None, "pos": None, "count": 0, "bin": None}
                    for _, name in names}

    while time.monotonic() < deadline:
        all_done = True

        for cls, name in names:
            if status[name] != "pending":
                continue

            all_done = False
            xyz = _read_world_xyz(name)
            if xyz is None:
                continue

            pos = (xyz.x, xyz.y, xyz.z)
            final_xyz[name] = pos
            candidate, bin_name = _placement_candidate(
                cls, xyz, config, margin, placed_z_min, placed_z_max)
            state = stable_state[name]

            if candidate is None:
                state.update({"candidate": None, "pos": pos, "count": 0, "bin": None})
                continue

            same_candidate = candidate == state["candidate"] and bin_name == state["bin"]
            settled = state["pos"] is not None and _distance(pos, state["pos"]) <= stable_epsilon
            if same_candidate and settled:
                state["count"] += 1
            else:
                state.update({"candidate": candidate, "count": 1, "bin": bin_name})
            state["pos"] = pos

            if state["count"] < stable_samples:
                continue

            status[name] = candidate
            if candidate == "correct":
                print(f"  [+] {name} yerleşti: DOĞRU KUTU ({cls}) "
                      f"stable={state['count']} pos=({xyz.x:.3f}, {xyz.y:.3f}, {xyz.z:.3f})",
                      flush=True)
            elif candidate == "wrong":
                print(f"  [+] {name} yerleşti: YANLIŞ KUTU → {bin_name} "
                      f"stable={state['count']} pos=({xyz.x:.3f}, {xyz.y:.3f}, {xyz.z:.3f})",
                      flush=True)
            elif candidate == "fling":
                print(f"  [+] {name} yerleşti: FIRLATILDI/DIŞARIDA "
                      f"stable={state['count']} pos=({xyz.x:.3f}, {xyz.y:.3f}, {xyz.z:.3f})",
                      flush=True)

        if all_done:
            break

        time.sleep(poll_s)

    return final_xyz, status

def judge(names, final_xyz, status, config):
    correct = sum(1 for s in status.values() if s == "correct")
    wrong = sum(1 for s in status.values() if s == "wrong")
    fling = sum(1 for s in status.values() if s == "fling")
    
    details = []
    for cls, name in names:
        s = status.get(name, "pending")
        pos = final_xyz.get(name)
        if pos is None:
            details.append(f"  {name}: SON POZ OKUNAMADI")
        elif s == "pending":
            details.append(f"  {name}: Hiçbir kutuda değil (zaman aşımı) (x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f})")
            
    return correct, wrong, fling, details

def check_crash(log_file):
    if not log_file:
        return 0, []
    
    crash_count = 0
    details = []
    try:
        with open(log_file, "r") as f:
            content = f.read().lower()
            if "segmentation fault" in content or "segfault" in content:
                crash_count += 1
                details.append("  CRASH: Segmentation fault detected in log")
            if "assertion failed" in content or "assertion `" in content or "core dumped" in content or "stack smashing" in content:
                crash_count += 1
                details.append("  CRASH: Assertion failure or memory corruption detected in log")
    except FileNotFoundError:
        details.append(f"  CRASH: Log file not found ({log_file})")
        crash_count += 1
        
    return crash_count, details


def check_attach_detach(log_file, expected_count):
    if not log_file:
        return 0, []
    try:
        with open(log_file, "r") as f:
            content = f.read()
    except FileNotFoundError:
        return 1, [f"  ATTACH: Log file not found ({log_file})"]

    attach_count = content.count("ATTACH:")
    detach_count = content.count("DETACH:")
    if attach_count >= expected_count and detach_count >= expected_count:
        return 0, []
    return 1, [
        f"  ATTACH/DETACH: expected >= {expected_count}, "
        f"attach={attach_count}, detach={detach_count}"
    ]

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=None, help="sorting_bins.yaml yolu")
    parser.add_argument("--run-id", default="1")
    parser.add_argument("--objects-per-class", type=int, default=2)
    parser.add_argument("--pass-threshold", type=int, default=6)
    parser.add_argument("--settle-timeout", type=float, default=240.0)
    parser.add_argument("--poll-s", type=float, default=4.0)
    parser.add_argument("--bin-margin", type=float, default=0.01)
    parser.add_argument("--stable-samples", type=int, default=3)
    parser.add_argument("--stable-epsilon", type=float, default=0.02)
    parser.add_argument("--placed-z-min", type=float, default=0.55)
    parser.add_argument("--placed-z-max", type=float, default=0.75)
    parser.add_argument("--log-file", default=None, help="Gazebo log file for crash detection")
    args = parser.parse_args()

    config = load_sorting_config(args.config)
    names = _spawned_object_names(args.objects_per_class)
    total = len(names)

    print(f"[acceptance] run {args.run_id}: {total} nesne, fiziksel tutuş oturması bekleniyor "
          f"(timeout {args.settle_timeout:.0f}s)...", flush=True)
          
    final_xyz, status = wait_until_sorted(
        names, config, args.pass_threshold, args.bin_margin,
        args.settle_timeout, args.poll_s, args.stable_samples,
        args.stable_epsilon, args.placed_z_min, args.placed_z_max)
        
    pending_count = sum(1 for s in status.values() if s == "pending")
    if pending_count > 0:
        print("[acceptance] UYARI: sorting timeout — bazı nesneler yerleştirilemedi", flush=True)

    correct, wrong, fling, details = judge(names, final_xyz, status, config)
    crash_count, crash_details = check_crash(args.log_file)
    attach_count, attach_details = check_attach_detach(args.log_file, args.pass_threshold)
    
    for line in details:
        print(line, flush=True)
    for line in crash_details:
        print(line, flush=True)
    for line in attach_details:
        print(line, flush=True)

    verdict = "PASS" if (correct >= args.pass_threshold and wrong == 0 and fling == 0 and crash_count == 0 and attach_count == 0) else "FAIL"
    print(f"RESULT run={args.run_id} objects={total} correct={correct} "
          f"wrong={wrong} fling={fling} crash={crash_count} verdict={verdict}", flush=True)
          
    return 0 if verdict == "PASS" else 1

if __name__ == "__main__":
    sys.exit(main())
