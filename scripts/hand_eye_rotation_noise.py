#!/usr/bin/env python3
"""Measure hand-eye chain scatter as a function of wrist rotation magnitude.

This script is analysis-only: it imports no ROS packages and cannot command a
robot.  One global PARK hand-eye transform is solved from all accepted samples;
that same transform is then used in every rotation bin so a noisy bin cannot
hide its error by fitting a different transform.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys

import cv2
import numpy as np


REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
import hand_eye_pose_search as pose_search  # noqa: E402
import solve_hand_eye as solver  # noqa: E402


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    with open(temporary, "w") as handle:
        json.dump(payload, handle, indent=2)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(str(temporary), str(path))


def parse_edges(text):
    try:
        edges = [float(value) for value in text.split(",")]
    except ValueError:
        raise ValueError("--bins must be comma-separated degrees")
    if len(edges) < 3 or edges[0] != 0.0:
        raise ValueError("--bins must start at 0 and contain at least 3 edges")
    if any(not math.isfinite(value) for value in edges):
        raise ValueError("--bins values must be finite")
    if any(right <= left for left, right in zip(edges, edges[1:])):
        raise ValueError("--bins must be strictly increasing")
    if edges[-1] < 180.0:
        raise ValueError("--bins final edge must cover 180 degrees")
    return edges


def matrix_from_pose(pose):
    return solver.rt_to_matrix(
        solver.quat_to_rotation(*pose["quat_xyzw"]), pose["xyz"])


def validate_payload(samples_payload, pose_payload):
    meta = samples_payload.get("meta", {})
    if meta.get("aborted"):
        raise ValueError("sample run was aborted: %s" %
                         meta.get("abort_reason"))
    if samples_payload.get("suspect_samples"):
        raise ValueError("sample file contains unverified suspect_samples")
    samples = samples_payload.get("samples")
    expected = pose_payload.get("joints_rad")
    if not isinstance(samples, list) or len(samples) < 8:
        raise ValueError("at least 8 verified samples are required")
    if not isinstance(expected, list) or not expected:
        raise ValueError("pose set must contain joints_rad")

    for number, sample in enumerate(samples):
        index = sample.get("pose_index")
        if not isinstance(index, int) or isinstance(index, bool):
            raise ValueError("sample %d has no integer pose_index" % number)
        if index < 0 or index >= len(expected):
            raise ValueError("sample %d pose_index %d is outside pose set" %
                             (number, index))
        commanded = sample.get("commanded")
        if commanded is not None:
            if not isinstance(commanded, list) or len(commanded) != 5:
                raise ValueError("sample %d command is not a five-joint list" %
                                 number)
            difference = max(
                abs(float(a) - float(b))
                for a, b in zip(commanded, expected[index]))
            if difference > 1e-4:
                raise ValueError(
                    "sample %d command disagrees with tracked pose %d" %
                    (number, index))
        for key in ("base_to_wrist", "cam_to_board"):
            pose = sample.get(key)
            if not isinstance(pose, dict):
                raise ValueError("sample %d has invalid %s" % (number, key))
            xyz_bad = len(pose.get("xyz", [])) != 3
            quaternion_bad = len(pose.get("quat_xyzw", [])) != 4
            if xyz_bad or quaternion_bad:
                raise ValueError("sample %d has invalid %s" % (number, key))
    return samples


def rotation_magnitude_deg(a_matrix, zero_rotation):
    return solver.rotation_angle_deg(zero_rotation.T @ a_matrix[:3, :3])


def build_curve(a_matrices, b_matrices, edges):
    x_matrix = solver.solve(a_matrices, b_matrices, cv2.CALIB_HAND_EYE_PARK)
    boards = [a @ x_matrix @ b for a, b in zip(a_matrices, b_matrices)]
    positions = np.array([board[:3, 3] for board in boards])
    global_centre = positions.mean(axis=0)
    global_residual_mm = (
        np.linalg.norm(positions - global_centre, axis=1) * 1000.0)
    zero_rotation = pose_search.fk([0.0] * 5)[:3, :3]
    rotations = np.array([
        rotation_magnitude_deg(a_matrix, zero_rotation)
        for a_matrix in a_matrices
    ])

    curve = []
    for index, (lower, upper) in enumerate(zip(edges, edges[1:])):
        if index == len(edges) - 2:
            members = np.where((rotations >= lower) & (rotations <= upper))[0]
        else:
            members = np.where((rotations >= lower) & (rotations < upper))[0]
        entry = {
            "rotation_min_deg": lower,
            "rotation_max_deg": upper,
            "sample_count": int(len(members)),
            "status": "PASS" if len(members) >= 3 else "INCONCLUSIVE",
        }
        if len(members):
            local_positions = positions[members]
            local_centre = local_positions.mean(axis=0)
            local_residual = np.linalg.norm(
                local_positions - local_centre, axis=1) * 1000.0
            reference = boards[int(members[0])][:3, :3]
            local_angles = [
                solver.rotation_angle_deg(
                    reference.T @ boards[int(item)][:3, :3])
                for item in members
            ]
            entry.update({
                "rotation_median_deg": float(np.median(rotations[members])),
                "within_bin_scatter_rms_mm": float(
                    np.sqrt(np.mean(local_residual ** 2))),
                "within_bin_scatter_max_mm": float(local_residual.max()),
                "within_bin_rotation_max_deg": float(max(local_angles)),
                "global_centre_residual_rms_mm": float(
                    np.sqrt(np.mean(global_residual_mm[members] ** 2))),
            })
        curve.append(entry)

    if len(rotations) >= 3 and float(np.std(rotations)) > 1e-12:
        slope, intercept = np.polyfit(rotations, global_residual_mm, 1)
        correlation = float(np.corrcoef(rotations, global_residual_mm)[0, 1])
    else:
        slope = float("nan")
        intercept = float("nan")
        correlation = float("nan")
    return {
        "x_translation_mm": [
            float(value * 1000.0) for value in x_matrix[:3, 3]],
        "sample_count": len(a_matrices),
        "rotation_range_deg": [
            float(rotations.min()), float(rotations.max())],
        "overall_board_scatter_rms_mm": float(
            np.sqrt(np.mean(global_residual_mm ** 2))),
        "trend": {
            "global_residual_slope_mm_per_deg": float(slope),
            "global_residual_intercept_mm": float(intercept),
            "pearson_r": correlation,
            "classification": "DESCRIPTIVE_NOT_CAUSAL",
        },
        "curve": curve,
    }


def analyse(samples_path, pose_set_path, edges, out_path):
    with open(samples_path) as handle:
        samples_payload = json.load(handle)
    with open(pose_set_path) as handle:
        pose_payload = json.load(handle)
    samples = validate_payload(samples_payload, pose_payload)
    a_matrices = [
        matrix_from_pose(sample["base_to_wrist"]) for sample in samples]
    b_matrices = [
        matrix_from_pose(sample["cam_to_board"]) for sample in samples]
    result = build_curve(a_matrices, b_matrices, edges)
    report = {
        "kind": "hand_eye_rotation_noise_curve",
        "version": 1,
        "source": {
            "samples": str(samples_path.resolve()),
            "samples_sha256": sha256_file(samples_path),
            "pose_set": str(pose_set_path.resolve()),
            "pose_set_sha256": sha256_file(pose_set_path),
        },
        "units": {"translation": "mm", "rotation": "deg"},
        "rotation_definition":
            "geodesic angle of measured base_to_wrist relative to canonical "
            "FK q=0",
        "fit_policy":
            "one PARK X from all samples; the same X is evaluated in every "
            "bin",
        "result": result,
        "hypothesis": {
            "supports_rotation_dependent_floor_if":
                "higher-rotation bins repeatedly show larger residual/scatter",
            "refutes_simple_rotation_hypothesis_if":
                "bins are flat within repeatability and vision noise controls",
            "cannot_alone_identify":
                "which joint, zero offset, compliance, backlash, or vision "
                "term dominates",
        },
        "acceptance": "MEASUREMENT_ONLY_NO_AUTOMATIC_PHYSICAL_GO",
    }
    write_json(out_path, report)
    print("rotation bin       n  status        local rms mm  global rms mm")
    for item in result["curve"]:
        local = item.get("within_bin_scatter_rms_mm")
        global_rms = item.get("global_centre_residual_rms_mm")
        print("%5.1f..%5.1f  %3d  %-12s  %s  %s" %
              (item["rotation_min_deg"], item["rotation_max_deg"],
               item["sample_count"], item["status"],
               "%-12.3f" % local if local is not None else "-           ",
               "%.3f" % global_rms if global_rms is not None else "-"))
    print("trend: %.5f mm/deg, Pearson r=%.3f" %
          (result["trend"]["global_residual_slope_mm_per_deg"],
           result["trend"]["pearson_r"]))
    print("report: %s" % out_path)
    return report


def self_test(edges):
    rng = np.random.default_rng(17)
    true_x = solver.rt_to_matrix(
        cv2.Rodrigues(np.array([0.08, -0.45, 0.11]))[0],
        np.array([0.03, -0.012, 0.05]))
    board = solver.rt_to_matrix(np.eye(3), np.array([0.35, 0.02, 0.12]))
    zero_rotation = pose_search.fk([0.0] * 5)[:3, :3]
    a_matrices, b_matrices = [], []
    for angle_deg in np.linspace(2.0, 88.0, 72):
        angle = math.radians(float(angle_deg))
        axis = np.array([
            math.sin(angle_deg * 0.13),
            math.cos(angle_deg * 0.09),
            0.35 + 0.1 * math.sin(angle_deg * 0.07),
        ])
        axis /= np.linalg.norm(axis)
        delta, _ = cv2.Rodrigues(axis * angle)
        rotation = zero_rotation @ delta
        a_matrix = solver.rt_to_matrix(
            rotation, np.array([0.20 + 0.03 * math.cos(angle),
                                0.02 * math.sin(2.0 * angle),
                                0.25 + 0.02 * math.sin(angle)]))
        b_matrix = np.linalg.inv(true_x) @ np.linalg.inv(a_matrix) @ board
        sigma_m = (0.05 + 0.012 * angle_deg) / 1000.0
        b_matrix = b_matrix.copy()
        b_matrix[:3, 3] += rng.normal(0.0, sigma_m, 3)
        a_matrices.append(a_matrix)
        b_matrices.append(b_matrix)
    result = build_curve(a_matrices, b_matrices, edges)
    slope = result["trend"]["global_residual_slope_mm_per_deg"]
    correlation = result["trend"]["pearson_r"]
    if not slope > 0.005 or not correlation > 0.35:
        print("SELF-TEST FAIL: injected rotation-dependent noise was not "
              "detected")
        return 1
    print("SELF-TEST PASS: injected slope %.5f mm/deg, r=%.3f" %
          (slope, correlation))
    return 0


def main():
    parser = argparse.ArgumentParser(
        description="Analysis-only wrist-rotation vs hand-eye noise curve")
    parser.add_argument("--samples", type=Path,
                        default=REPO.joinpath(
                            "runs", "hand_eye", "gated_samples.json"))
    parser.add_argument("--pose-set", type=Path,
                        default=REPO.joinpath(
                            "data", "hand_eye", "gated_pose_set.json"))
    parser.add_argument("--bins", default="0,15,30,45,60,90,180")
    parser.add_argument("--out", type=Path,
                        default=REPO.joinpath(
                            "runs", "hand_eye", "rotation_noise_curve.json"))
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    try:
        edges = parse_edges(args.bins)
        if args.self_test:
            return self_test(edges)
        if not args.samples.is_file():
            raise ValueError(
                "sample file is absent: %s (collect it at the reviewed live "
                "session)"
                % args.samples)
        if not args.pose_set.is_file():
            raise ValueError("pose set is absent: %s" % args.pose_set)
        analyse(args.samples, args.pose_set, edges, args.out)
        return 0
    except (IOError, OSError, ValueError, KeyError, cv2.error) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    sys.exit(main())
