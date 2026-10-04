#!/usr/bin/env python3
"""Plan, execute, and analyse approach-direction repeatability at one pose.

The default mode is non-motion: it writes and prints the exact approach/target
sequence.  ``--execute`` delegates every trajectory, observation gate,
interrupt handler, board-moved check, and verified q=0 return to the reviewed
``hand_eye_session.py`` implementation.  This wrapper never publishes a ROS
trajectory itself.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys


REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
import hand_eye_session as session  # noqa: E402


CONFIRMATION = "KESICI-HAZIR-RAY-KESIK"
MAX_APPROACH_DELTA_RAD = 0.05
MAX_SPEED_CAP_RAD_S = 0.10
BOARD_CHECK_EVERY_COMMANDS = 4


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_target(path, pose_index):
    with open(path) as handle:
        payload = json.load(handle)
    poses = payload.get("joints_rad")
    if not isinstance(poses, list) or not poses:
        raise ValueError("pose set must contain a non-empty joints_rad list")
    if pose_index < 0 or pose_index >= len(poses):
        raise ValueError("pose index %d outside 0..%d" %
                         (pose_index, len(poses) - 1))
    target = poses[pose_index]
    if not isinstance(target, list):
        raise ValueError("pose %d is not a list" % pose_index)
    if len(target) != len(session.ARM_JOINTS):
        raise ValueError("pose %d does not contain five joints" % pose_index)
    if not all(session.is_finite_number(value) for value in target):
        raise ValueError("pose %d is not a finite five-joint vector" %
                         pose_index)
    return [float(value) for value in target], payload


def build_plan(target, repetitions, joint_index, delta, envelope):
    route = []
    visits = []
    for repetition in range(repetitions):
        for direction in (-1, 1):
            approach = list(target)
            approach[joint_index] += direction * delta
            if abs(approach[joint_index]) > envelope:
                raise ValueError(
                    "approach on %s exceeds +/-%s rad; reduce --approach-delta"
                    % (session.ARM_JOINTS[joint_index], envelope))
            route.append(approach)
            route.append(list(target))
            visits.append({
                "repetition": repetition + 1,
                "direction": "negative" if direction < 0 else "positive",
                "approach_pose_index": len(route) - 2,
                "target_pose_index": len(route) - 1,
            })
    return route, visits


def maximum_command_rate(route, move_seconds,
                         reset_every=BOARD_CHECK_EVERY_COMMANDS,
                         zero_approach=session.DEFAULT_RETURN_PROBE):
    zero = [0.0] * len(session.ARM_JOINTS)
    previous = zero
    worst = 0.0

    def rate(first, second):
        delta = max(abs(a - b) for a, b in zip(first, second))
        return delta / move_seconds

    # set_zero_reference() performs this approach before the first route pose.
    worst = max(worst, rate(previous, zero_approach))
    worst = max(worst, rate(zero_approach, zero))
    previous = zero
    for index, pose in enumerate(route):
        worst = max(worst, rate(previous, pose))
        previous = pose
        if reset_every and (index + 1) % reset_every == 0:
            # The periodic board check approaches, then ends at q=0.
            worst = max(worst, rate(previous, zero_approach))
            worst = max(worst, rate(zero_approach, zero))
            previous = zero
    return worst


def write_json(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    with open(temporary, "w") as handle:
        json.dump(payload, handle, indent=2)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(str(temporary), str(path))


def analyse(raw_path, plan_meta, report_path):
    with open(raw_path) as handle:
        raw = json.load(handle)
    meta = raw.get("meta", {})
    if meta.get("aborted"):
        raise ValueError("raw session aborted: %s" % meta.get("abort_reason"))
    if raw.get("suspect_samples"):
        raise ValueError("raw session contains unverified suspect_samples")

    by_pose = {}
    for sample in raw.get("samples", []):
        by_pose.setdefault(sample.get("pose_index"), []).append(sample)

    visit_summaries = []
    for visit in plan_meta["visits"]:
        samples = by_pose.get(visit["target_pose_index"], [])
        if len(samples) < plan_meta["samples_per_visit"]:
            raise ValueError(
                "target visit %d has %d/%d accepted samples"
                % (visit["target_pose_index"], len(samples),
                   plan_meta["samples_per_visit"]))
        observations = [sample["cam_to_board"] for sample in samples]
        summary = session.summarize_observations(observations)
        visit_summaries.append({
            "repetition": visit["repetition"],
            "approach_direction": visit["direction"],
            "target_pose_index": visit["target_pose_index"],
            "sample_count": len(samples),
            "cam_to_board": {
                "xyz": summary["xyz"],
                "quat_xyzw": summary["quat_xyzw"],
            },
            "within_visit_spread_mm": summary["internal_spread_mm"],
            "within_visit_spread_deg": summary["internal_spread_deg"],
        })

    collapsed = [entry["cam_to_board"] for entry in visit_summaries]
    overall = session.summarize_observations(collapsed)
    directions = {}
    for direction in ("negative", "positive"):
        values = [entry["cam_to_board"] for entry in visit_summaries
                  if entry["approach_direction"] == direction]
        directions[direction] = session.summarize_observations(values)

    direction_offset_mm = session.vector_distance(
        directions["negative"]["xyz"],
        directions["positive"]["xyz"]) * 1000.0
    direction_offset_deg = math.degrees(session.quat_angle_rad(
        directions["negative"]["quat_xyzw"],
        directions["positive"]["quat_xyzw"]))
    split_repetition = plan_meta["repetitions"] // 2
    phase_summaries = {}
    for name, predicate in (
            ("characterization",
             lambda repetition: repetition <= split_repetition),
            ("validation", lambda repetition: repetition > split_repetition)):
        values = [entry["cam_to_board"] for entry in visit_summaries
                  if predicate(entry["repetition"])]
        summary = session.summarize_observations(values)
        phase_summaries[name] = {
            "visit_count": len(values),
            "visit_scatter_rms_mm": summary["internal_spread_mm"],
            "visit_scatter_max_deg": summary["internal_spread_deg"],
        }
    report = {
        "kind": "hand_eye_approach_repeatability",
        "version": 1,
        "created_utc": session.utc_now_iso(),
        "git_commit": session.git_commit(),
        "units": {"translation": "mm", "rotation": "deg", "joints": "rad"},
        "source": {
            "raw_session": str(Path(raw_path).resolve()),
            "raw_sha256": sha256_file(raw_path),
            "pose_set": plan_meta["pose_set"],
            "pose_set_sha256": plan_meta["pose_set_sha256"],
        },
        "frames": meta.get("base_frame"),
        "target_pose_index": plan_meta["target_pose_index"],
        "target_joints_rad": plan_meta["target_joints_rad"],
        "approach_joint": plan_meta["approach_joint"],
        "approach_delta_rad": plan_meta["approach_delta_rad"],
        "visit_count": len(visit_summaries),
        "visit_scatter_rms_mm": overall["internal_spread_mm"],
        "visit_scatter_max_deg": overall["internal_spread_deg"],
        "approach_direction_offset_mm": direction_offset_mm,
        "approach_direction_offset_deg": direction_offset_deg,
        "dataset_split": {
            "policy": ("first half characterizes; second half validates "
                       "without retuning"),
            "split_after_repetition": split_repetition,
            "phases": phase_summaries,
        },
        "visits": visit_summaries,
        "hypothesis": {
            "supports_servo_repeatability_limit_if":
                "same commanded target has non-trivial between-visit scatter",
            "supports_backlash_or_gravity_if":
                "positive and negative approach centres differ systematically",
            "does_not_separate":
                "camera/PnP noise without the within-visit spread control",
        },
        "acceptance": "MEASUREMENT_ONLY_NO_AUTOMATIC_PHYSICAL_GO",
    }
    write_json(report_path, report)
    print("visit scatter : %.3f mm / %.3f deg" %
          (report["visit_scatter_rms_mm"], report["visit_scatter_max_deg"]))
    print("direction bias: %.3f mm / %.3f deg" %
          (direction_offset_mm, direction_offset_deg))
    print("report: %s" % report_path)
    return report


def build_parser():
    parser = argparse.ArgumentParser(
        description="One-pose, two-direction repeatability plan; default is "
                    "no motion")
    parser.add_argument("--pose-set", type=Path,
                        default=REPO.joinpath(
                            "data", "hand_eye", "gated_pose_set.json"))
    parser.add_argument("--pose-index", type=int, required=True)
    parser.add_argument("--repetitions", type=int, default=6)
    parser.add_argument("--approach-joint",
                        choices=["auto"] + session.ARM_JOINTS,
                        default="auto",
                        help="default chooses joint_4 or joint_5 with more "
                             "envelope margin")
    parser.add_argument("--approach-delta", type=float, default=0.03)
    parser.add_argument("--max-abs-rad", type=float, default=1.0)
    parser.add_argument("--move-seconds", type=float, default=24.0)
    parser.add_argument("--speed-cap-rad-s", type=float, default=0.05)
    parser.add_argument("--settle-seconds", type=float, default=3.0)
    parser.add_argument("--samples-per-visit", type=int, default=3)
    parser.add_argument("--plan-out", type=Path,
                        default=REPO.joinpath(
                            "runs", "hand_eye", "repeatability_plan.json"))
    parser.add_argument("--raw-out", type=Path,
                        default=REPO.joinpath(
                            "runs", "hand_eye", "repeatability_raw.json"))
    parser.add_argument("--report-out", type=Path,
                        default=REPO.joinpath(
                            "runs", "hand_eye", "repeatability_report.json"))
    parser.add_argument("--analyse-only", type=Path,
                        help="analyse an existing raw session; commands "
                             "nothing")
    parser.add_argument("--execute", action="store_true",
                        help="execute through hand_eye_session.py; absent by "
                             "default")
    parser.add_argument("--gate-from", type=Path,
                        help="completed calibration-purpose q=0 gate artifact")
    parser.add_argument("--architect-reviewed", action="store_true")
    parser.add_argument("--calibration-gate-pass", action="store_true",
                        help="operator confirms verify_calibration_gate.py "
                             "PASS")
    parser.add_argument("--known-physical-q0", action="store_true",
                        help="operator confirms supported physical q=0 start")
    parser.add_argument("--operator-confirm", default="",
                        help="must equal %s at the arm" % CONFIRMATION)
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if args.repetitions < 4 or args.repetitions % 2:
            raise ValueError("--repetitions must be an even number >= 4")
        if not (0.0 < args.approach_delta <= MAX_APPROACH_DELTA_RAD):
            raise ValueError("--approach-delta must be >0 and <= %.2f rad" %
                             MAX_APPROACH_DELTA_RAD)
        if not (0.0 < args.max_abs_rad <= session.ENVELOPE_CEILING_RAD):
            raise ValueError("--max-abs-rad must be >0 and <= %.2f" %
                             session.ENVELOPE_CEILING_RAD)
        if args.move_seconds < 1.0 or args.settle_seconds < 0.0:
            raise ValueError(
                "move must be >=1s and settle must be non-negative")
        if not (0.0 < args.speed_cap_rad_s <= MAX_SPEED_CAP_RAD_S):
            raise ValueError("speed cap must be >0 and <= %.2f rad/s" %
                             MAX_SPEED_CAP_RAD_S)
        if args.samples_per_visit < 3:
            raise ValueError(
                "at least 3 samples per target visit are required")

        target, _ = load_target(args.pose_set, args.pose_index)
        if max(abs(value) for value in target) > args.max_abs_rad:
            raise ValueError("target exceeds the configured motion envelope")
        if args.approach_joint == "auto":
            joint_index = max(
                (3, 4),
                key=lambda item: args.max_abs_rad - abs(target[item]))
            approach_joint = session.ARM_JOINTS[joint_index]
        else:
            joint_index = session.ARM_JOINTS.index(args.approach_joint)
            approach_joint = args.approach_joint
        route, visits = build_plan(target, args.repetitions, joint_index,
                                   args.approach_delta, args.max_abs_rad)
        worst_rate = maximum_command_rate(route, args.move_seconds)
        if worst_rate > args.speed_cap_rad_s + 1e-12:
            raise ValueError(
                "plan needs %.3f rad/s, above %.3f cap; increase "
                "--move-seconds"
                % (worst_rate, args.speed_cap_rad_s))

        plan_meta = {
            "kind": "hand_eye_approach_repeatability_plan",
            "version": 1,
            "created_utc": session.utc_now_iso(),
            "git_commit": session.git_commit(),
            "pose_set": str(args.pose_set.resolve()),
            "pose_set_sha256": sha256_file(args.pose_set),
            "target_pose_index": args.pose_index,
            "target_joints_rad": target,
            "approach_joint": approach_joint,
            "approach_delta_rad": args.approach_delta,
            "repetitions": args.repetitions,
            "dataset_split": {
                "characterization_repetitions": [1, args.repetitions // 2],
                "validation_repetitions": [
                    args.repetitions // 2 + 1, args.repetitions],
            },
            "samples_per_visit": args.samples_per_visit,
            "move_seconds": args.move_seconds,
            "speed_cap_rad_s": args.speed_cap_rad_s,
            "maximum_planned_rate_rad_s": worst_rate,
            "settle_seconds": args.settle_seconds,
            "board_check_every_commands": BOARD_CHECK_EVERY_COMMANDS,
            "visits": visits,
            "route": route,
            "safety": {
                "default": "NO_MOTION",
                "execution_requires": [
                    "architect review",
                    "calibration gate PASS",
                    "physical cutoff ready",
                    "ray starts OFF",
                    "supported and visually confirmed physical q=0 start",
                    "clear and gravity-supported workspace",
                ],
                "interrupt": "reviewed hand_eye_session verified q=0 return",
            },
        }
        write_json(args.plan_out, route)
        write_json(str(args.plan_out) + ".meta.json", plan_meta)
        print("plan: %s (%d commands, %d target visits)" %
              (args.plan_out, len(route), len(visits)))
        print("max planned rate: %.3f rad/s (cap %.3f)" %
              (worst_rate, args.speed_cap_rad_s))

        if args.analyse_only:
            analyse(args.analyse_only, plan_meta, args.report_out)
            return 0
        if not args.execute:
            print("DRY PLAN ONLY: no ROS import, UART, rail, or motion was "
                  "used.")
            return 0
        if not args.architect_reviewed:
            parser.error("--execute requires --architect-reviewed")
        if not args.calibration_gate_pass:
            parser.error("--execute requires --calibration-gate-pass")
        if not args.known_physical_q0:
            parser.error("--execute requires --known-physical-q0")
        if args.operator_confirm != CONFIRMATION:
            parser.error("--execute requires --operator-confirm %s" %
                         CONFIRMATION)
        if not args.gate_from or not args.gate_from.is_file():
            parser.error(
                "--execute requires an existing --gate-from artifact")

        session_args = [
            "--out", str(args.raw_out),
            "--poses-json", str(args.plan_out),
            "--move-seconds", str(args.move_seconds),
            "--settle-seconds", str(args.settle_seconds),
            "--samples-per-pose", str(args.samples_per_visit),
            "--max-abs-rad", str(args.max_abs_rad),
            "--check-every", str(BOARD_CHECK_EVERY_COMMANDS),
            "--gate-from", str(args.gate_from),
            "--purpose", session.PURPOSE_DIAGNOSTIC,
        ]
        code = session.main(session_args)
        if code != 0:
            print("repeatability session failed closed with rc=%d" % code)
            return code
        analyse(args.raw_out, plan_meta, args.report_out)
        return 0
    except (IOError, OSError, ValueError, KeyError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    sys.exit(main())
