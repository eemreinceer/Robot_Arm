#!/usr/bin/env python3
"""Eye-in-hand calibration solver with a self-test and a noise study.

Solves AX = XB for X = (wrist -> camera), from samples of
  A: base -> wrist   (forward kinematics, i.e. TF base_link -> link_5)
  B: camera -> board (PnP, i.e. /vision_encoder/board_pose)

THE HEADLINE METRIC IS NOT THE SOLVER RESIDUAL. The board is physically bolted
down, so for every sample

    T_base_board = T_base_wrist @ X @ T_cam_board

must give the SAME pose. The scatter of that reconstruction, in millimetres and
degrees, is the end-to-end error of the whole chain -- URDF geometry, zero
offsets, servo positioning and camera mount together. That is the number that
decides whether vision can act as an encoder, and no amount of solver
cross-validation substitutes for it.

WHY THE SELF-TEST MATTERS: on this arm the joint angles are OPEN LOOP (no
encoders; read() echoes the commanded value) and zero_offset_rad was set by eye
against an RViz ghost. So FK feeds noise straight into A. `--self-test` recovers
a known X from exact data, then repeats with injected joint noise so the
expected error is a measured curve rather than a hope.

Usage:
    python3 scripts/solve_hand_eye.py --self-test
    python3 scripts/solve_hand_eye.py --samples runs/hand_eye_YYYYmmdd/samples.json
"""
import argparse
import json
import math
import sys

import cv2
import numpy as np

# All five are run on purpose. They fail differently on degenerate or noisy
# data, so their disagreement is itself a diagnostic: if they spread far apart,
# the samples are the problem, not the solver.
METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def rt_to_matrix(rotation, translation):
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = np.asarray(translation).reshape(3)
    return matrix


def quat_to_rotation(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise ValueError("zero quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rotation_angle_deg(rotation):
    value = (np.trace(rotation) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, value))))


def solve(a_matrices, b_matrices, method):
    r_gripper2base = [m[:3, :3] for m in a_matrices]
    t_gripper2base = [m[:3, 3] for m in a_matrices]
    r_target2cam = [m[:3, :3] for m in b_matrices]
    t_target2cam = [m[:3, 3] for m in b_matrices]
    rotation, translation = cv2.calibrateHandEye(
        r_gripper2base, t_gripper2base, r_target2cam, t_target2cam, method=method)
    return rt_to_matrix(rotation, translation)


def board_consistency(a_matrices, b_matrices, x_matrix):
    """Scatter of the reconstructed base->board pose. The board does not move."""
    poses = [a @ x_matrix @ b for a, b in zip(a_matrices, b_matrices)]
    positions = np.array([p[:3, 3] for p in poses])
    centre = positions.mean(axis=0)
    offsets = np.linalg.norm(positions - centre, axis=1)
    reference = poses[0][:3, :3]
    angles = [rotation_angle_deg(reference.T @ p[:3, :3]) for p in poses]
    return {
        "position_std_mm": [round(v, 3) for v in (positions.std(axis=0) * 1000.0)],
        "position_max_dev_mm": round(float(offsets.max()) * 1000.0, 3),
        "position_rms_mm": round(float(np.sqrt((offsets ** 2).mean())) * 1000.0, 3),
        "angle_max_dev_deg": round(float(max(angles)), 3),
    }


def report(a_matrices, b_matrices, label=""):
    results = {}
    for name, flag in sorted(METHODS.items()):
        try:
            x_matrix = solve(a_matrices, b_matrices, flag)
        except cv2.error as error:
            results[name] = {"error": str(error).strip().splitlines()[-1]}
            continue
        results[name] = {
            "translation_mm": [round(v * 1000.0, 2) for v in x_matrix[:3, 3]],
            "consistency": board_consistency(a_matrices, b_matrices, x_matrix),
            "_matrix": x_matrix,
        }

    good = {k: v for k, v in results.items() if "_matrix" in v}
    if len(good) > 1:
        translations = np.array([v["_matrix"][:3, 3] for v in good.values()])
        spread_mm = float(np.linalg.norm(translations - translations.mean(axis=0),
                                         axis=1).max()) * 1000.0
    else:
        spread_mm = float("nan")

    print("=== hand-eye %s(%d samples) ===" % (label, len(a_matrices)))
    for name, value in sorted(results.items()):
        if "error" in value:
            print("  %-11s FAILED: %s" % (name, value["error"]))
            continue
        cons = value["consistency"]
        print("  %-11s t=%s mm | board scatter rms %.2f mm, max %.2f mm, %.2f deg"
              % (name, value["translation_mm"], cons["position_rms_mm"],
                 cons["position_max_dev_mm"], cons["angle_max_dev_deg"]))
    print("  cross-method translation spread: %.2f mm" % spread_mm)
    return results, spread_mm


def random_transform(rng, angle_scale=0.6, translation_scale=0.25):
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    angle = rng.uniform(-angle_scale, angle_scale)
    rotation, _ = cv2.Rodrigues(axis * angle)
    return rt_to_matrix(rotation, rng.uniform(-translation_scale, translation_scale, 3))


def self_test(sample_count=14, seed=7):
    """Recover a known X from exact data, then measure degradation under noise."""
    rng = np.random.default_rng(seed)

    true_x = rt_to_matrix(cv2.Rodrigues(np.array([0.05, -0.9, 0.12]))[0],
                          np.array([0.031, -0.014, 0.052]))
    t_base_board = rt_to_matrix(cv2.Rodrigues(np.array([2.9, 0.1, 0.2]))[0],
                                np.array([0.28, 0.02, 0.04]))

    a_matrices = [random_transform(rng) for _ in range(sample_count)]
    b_matrices = [np.linalg.inv(x) @ np.linalg.inv(a) @ t_base_board
                  for a, x in ((a, true_x) for a in a_matrices)]

    print("true X translation: %s mm"
          % [round(v * 1000.0, 2) for v in true_x[:3, 3]])
    results, _ = report(a_matrices, b_matrices, label="exact ")
    worst = 0.0
    for name, value in results.items():
        if "_matrix" not in value:
            continue
        error_mm = np.linalg.norm(value["_matrix"][:3, 3] - true_x[:3, 3]) * 1000.0
        worst = max(worst, error_mm)
        print("    %-11s translation error vs truth: %.4f mm" % (name, error_mm))
    if worst > 0.5:
        print("SELF-TEST FAILED: solver cannot recover a known X on exact data.")
        return 1
    print("SELF-TEST PASS: known X recovered within %.4f mm on exact data.\n" % worst)

    # Noise study: perturb the wrist pose the way an imperfect zero offset or a
    # servo that lands short of its command would.
    print("=== noise study: how joint error shows up in the result ===")
    print("(perturbs A only; B stays exact, so this isolates FK error)")
    for degrees in (0.25, 0.5, 1.0, 2.0, 3.0):
        noisy = []
        for a in a_matrices:
            axis = rng.normal(size=3)
            axis /= np.linalg.norm(axis)
            delta, _ = cv2.Rodrigues(axis * math.radians(degrees))
            perturbed = a.copy()
            perturbed[:3, :3] = delta @ a[:3, :3]
            noisy.append(perturbed)
        x_matrix = solve(noisy, b_matrices, cv2.CALIB_HAND_EYE_PARK)
        error_mm = np.linalg.norm(x_matrix[:3, 3] - true_x[:3, 3]) * 1000.0
        cons = board_consistency(noisy, b_matrices, x_matrix)
        print("  %.2f deg joint noise -> X error %6.2f mm | board scatter rms %6.2f mm"
              % (degrees, error_mm, cons["position_rms_mm"]))
    print("\nRead this table before trusting a field result: it is the accuracy")
    print("ceiling set by open-loop joint angles, independent of the camera.")
    return 0


def load_samples(path):
    with open(path) as handle:
        payload = json.load(handle)
    a_matrices, b_matrices = [], []
    for sample in payload["samples"]:
        wrist = sample["base_to_wrist"]
        board = sample["cam_to_board"]
        a_matrices.append(rt_to_matrix(
            quat_to_rotation(*wrist["quat_xyzw"]), wrist["xyz"]))
        b_matrices.append(rt_to_matrix(
            quat_to_rotation(*board["quat_xyzw"]), board["xyz"]))
    return a_matrices, b_matrices, payload


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        return self_test()
    if not args.samples:
        parser.error("give --samples or --self-test")

    a_matrices, b_matrices, payload = load_samples(args.samples)
    if len(a_matrices) < 3:
        print("need at least 3 samples, got %d" % len(a_matrices))
        return 1
    print("run: %s" % payload.get("meta", {}))
    results, spread = report(a_matrices, b_matrices)
    print()
    print("Interpretation:")
    print("  * cross-method spread large  -> the data, not the solver, is the problem")
    print("  * board scatter large        -> FK/zero-offset/servo error dominates;")
    print("                                  compare against the --self-test table")
    return 0


if __name__ == "__main__":
    sys.exit(main())
