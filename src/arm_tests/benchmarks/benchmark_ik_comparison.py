#!/usr/bin/env python3
"""Phase 3 — Live 3-solver IK comparison (Analytic DLS vs Deep Learning vs MoveIt/pick_ik).

Methodology (corrected 2026-05-30):
  * Targets are GUARANTEED reachable: we sample random joints within the canonical
    self-collision JOINT_LIMITS and run them through the analytic FK service
    (`/fk_solve`) to obtain a real Link_6 pose (position AND orientation — not a
    fixed identity box). A single zero-seed local solver is no longer penalised by
    unreachable / identity-orientation targets.
  * Accuracy is measured, not just a per-solver success flag. For every solver we
    record the Link_6 FK-reconstruction position error of its returned joints:
      - `/ik_solve` (DLS) and `/dl_ik_solve` (DL) already return `position_error`.
      - `/compute_ik` (MoveIt) solution joints are run back through `/fk_solve`.
  * "Success" is accuracy-gated: error <= ACCURACY_THRESHOLD_MM (and the solver's
    own success/error flag). A feed-forward net that always returns a vector is NOT
    counted as 100% success unless it is actually accurate.

All frames are Link_6 (canonical, consistent with arm_kinematics FK/IK and the DL
dataset). Run the three services first (see launch_phase3_test.sh).
"""
import math
import os
from pathlib import Path
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node

from moveit_msgs.srv import GetPositionIK

try:
    from arm_interfaces.srv import SolveFk, SolveIk
except ImportError:
    SolveFk = None
    SolveIk = None

# Canonical self-collision joint limits (matches dataset joint_limits & C++ DH table).
JOINT_LIMITS = np.array(
    [
        [-3.14, 3.14],
        [-2.86, 1.25],
        [-3.30, 0.94],
        [-3.14, 3.14],
        [-2.44, 2.33],
        [-3.14, 3.14],
    ]
)
ARM_JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
ACCURACY_THRESHOLD_MM = 5.0  # FK-reconstruction position error gate for "success"
SOLVERS = ["moveit_ik", "custom_ik", "dl_ik"]


class IKComparisonBenchmarker(Node):
    def __init__(self):
        super().__init__("ik_comparison_benchmarker")
        self.moveit_ik_client = self.create_client(GetPositionIK, "/compute_ik")
        if SolveIk is not None:
            self.fk_client = self.create_client(SolveFk, "/fk_solve")
            self.custom_ik_client = self.create_client(SolveIk, "/ik_solve")
            self.dl_ik_client = self.create_client(SolveIk, "/dl_ik_solve")
        else:
            self.fk_client = None
            self.custom_ik_client = None
            self.dl_ik_client = None

    def wait_for_services(self):
        self.get_logger().info("Waiting for services (/fk_solve, /ik_solve, /dl_ik_solve, /compute_ik)...")
        self.fk_client.wait_for_service()
        self.moveit_ik_client.wait_for_service()
        self.custom_ik_client.wait_for_service()
        self.dl_ik_client.wait_for_service()
        self.get_logger().info("All services available!")

    def call_service(self, client, req):
        start_time = time.perf_counter()
        future = client.call_async(req)
        while not future.done():
            time.sleep(0.0001)
        latency = (time.perf_counter() - start_time) * 1000.0  # ms
        return future.result(), latency

    def fk(self, joints):
        """Analytic FK -> Link_6 Pose. Returns Pose or None."""
        req = SolveFk.Request()
        req.joint_angles = [float(j) for j in joints]
        res, _ = self.call_service(self.fk_client, req)
        if res is not None and res.success:
            return res.tcp_pose.pose
        return None

    @staticmethod
    def position_error_mm(pose_a, pose_b):
        dx = pose_a.position.x - pose_b.position.x
        dy = pose_a.position.y - pose_b.position.y
        dz = pose_a.position.z - pose_b.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz) * 1000.0

    def benchmark(self, num_samples=500):
        if self.custom_ik_client is None or self.dl_ik_client is None:
            self.get_logger().error("arm_interfaces services unavailable. Build arm_interfaces + arm_ml.")
            return

        self.get_logger().info(f"Starting Phase 3 IK comparison with {num_samples} reachable samples...")
        rng = np.random.default_rng(42)

        latencies = {k: [] for k in SOLVERS}
        pos_errors = {k: [] for k in SOLVERS}  # mm, only for solvers that returned joints
        returned = {k: 0 for k in SOLVERS}     # solver reported a solution
        accurate = {k: 0 for k in SOLVERS}     # solution within ACCURACY_THRESHOLD_MM
        valid_targets = 0

        for i in range(num_samples):
            # --- Generate a guaranteed-reachable target from FK of random valid joints ---
            q = rng.uniform(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
            target_pose = self.fk(q)
            if target_pose is None:
                continue
            valid_targets += 1
            seed = np.zeros(6).tolist()

            # --- 1. MoveIt2 (pick_ik) ---
            req_mik = GetPositionIK.Request()
            req_mik.ik_request.group_name = "arm"
            req_mik.ik_request.pose_stamped.header.frame_id = "base_link"
            req_mik.ik_request.pose_stamped.pose = target_pose
            req_mik.ik_request.ik_link_name = "Link_6"
            req_mik.ik_request.timeout.sec = 1
            res_mik, lat_mik = self.call_service(self.moveit_ik_client, req_mik)
            latencies["moveit_ik"].append(lat_mik)
            if res_mik is not None and res_mik.error_code.val == 1:
                returned["moveit_ik"] += 1
                err = self._moveit_error(res_mik, target_pose)
                if err is not None:
                    pos_errors["moveit_ik"].append(err)
                    if err <= ACCURACY_THRESHOLD_MM:
                        accurate["moveit_ik"] += 1

            # --- 2. Custom analytic DLS (/ik_solve) ---
            req_cik = SolveIk.Request()
            req_cik.target_pose = target_pose
            req_cik.seed_angles = seed
            req_cik.solver = "dls"
            res_cik, lat_cik = self.call_service(self.custom_ik_client, req_cik)
            latencies["custom_ik"].append(lat_cik)
            if res_cik is not None and res_cik.success:
                returned["custom_ik"] += 1
                err = res_cik.position_error * 1000.0
                pos_errors["custom_ik"].append(err)
                if err <= ACCURACY_THRESHOLD_MM:
                    accurate["custom_ik"] += 1

            # --- 3. Deep Learning IK (/dl_ik_solve) ---
            req_dl = SolveIk.Request()
            req_dl.target_pose = target_pose
            req_dl.seed_angles = seed
            req_dl.solver = "dl"
            res_dl, lat_dl = self.call_service(self.dl_ik_client, req_dl)
            latencies["dl_ik"].append(lat_dl)
            if res_dl is not None:
                # DL always returns a vector; only count it once we measure accuracy.
                err = res_dl.position_error * 1000.0
                pos_errors["dl_ik"].append(err)
                returned["dl_ik"] += 1
                if err <= ACCURACY_THRESHOLD_MM:
                    accurate["dl_ik"] += 1

            if (i + 1) % 50 == 0:
                self.get_logger().info(f"Processed {i + 1}/{num_samples} poses...")

        stats = {}
        for key in SOLVERS:
            lat = latencies[key]
            errs = pos_errors[key]
            stats[key] = {
                "returned_rate": (returned[key] / valid_targets * 100.0) if valid_targets else 0.0,
                "accurate_rate": (accurate[key] / valid_targets * 100.0) if valid_targets else 0.0,
                "lat_avg": float(np.mean(lat)) if lat else float("nan"),
                "lat_median": float(np.median(lat)) if lat else float("nan"),
                "lat_p95": float(np.percentile(lat, 95)) if lat else float("nan"),
                "err_mean": float(np.mean(errs)) if errs else float("nan"),
                "err_median": float(np.median(errs)) if errs else float("nan"),
                "err_p95": float(np.percentile(errs, 95)) if errs else float("nan"),
                "err_max": float(np.max(errs)) if errs else float("nan"),
            }
        self.generate_report(stats, valid_targets)

    def _moveit_error(self, res_mik, target_pose):
        """Run MoveIt's solution joints back through FK to get position error (mm)."""
        js = res_mik.solution.joint_state
        name_to_pos = dict(zip(js.name, js.position))
        try:
            joints = [name_to_pos[n] for n in ARM_JOINT_NAMES]
        except KeyError:
            return None
        fk_pose = self.fk(joints)
        if fk_pose is None:
            return None
        return self.position_error_mm(fk_pose, target_pose)

    def generate_report(self, stats, num_samples):
        def row(name, label):
            s = stats[name]
            return (
                f"| **{label}** | {s['accurate_rate']:.2f}% | {s['returned_rate']:.2f}% | "
                f"{s['err_mean']:.3f} | {s['err_median']:.3f} | {s['err_p95']:.3f} | {s['err_max']:.3f} | "
                f"{s['lat_avg']:.4f} | {s['lat_median']:.4f} | {s['lat_p95']:.4f} |"
            )

        report = f"""# Phase 3: IK Solvers Comparison Report (Analytic DLS vs Deep Learning vs MoveIt)

Generated on: {time.strftime('%Y-%m-%d %H:%M:%S')}
Reachable samples (FK of random valid joints): {num_samples}
Accuracy gate: position error <= {ACCURACY_THRESHOLD_MM:.1f} mm (Link_6 FK reconstruction)

## Methodology
Targets are produced by sampling random joints within the canonical self-collision
JOINT_LIMITS and running `/fk_solve` — every target is therefore reachable, with a
real orientation (not a fixed identity box). Accuracy is the Link_6 FK-reconstruction
position error of each solver's returned joints: `/ik_solve` and `/dl_ik_solve` report
`position_error` directly; MoveIt's solution is run back through `/fk_solve`.

## Results

| Solver | Accurate (≤{ACCURACY_THRESHOLD_MM:.0f}mm) | Returned | Err Mean (mm) | Err Median (mm) | Err p95 (mm) | Err Max (mm) | Lat Avg (ms) | Lat Median (ms) | Lat p95 (ms) |
|---|---|---|---|---|---|---|---|---|---|
{row('moveit_ik', 'MoveIt2 (pick_ik)')}
{row('custom_ik', 'Custom DLS (`arm_kinematics`)')}
{row('dl_ik', 'Deep Learning IK (`arm_ml`)')}

## Notes
- **Accurate** = fraction whose FK-reconstruction error is within the {ACCURACY_THRESHOLD_MM:.0f} mm gate.
  **Returned** = fraction where the solver reported a solution at all. For the DL net
  these differ sharply because it always returns a vector regardless of accuracy.
- DL IK is intended as a fast seed generator; pair it with analytic DLS / MoveIt
  refinement for precise picks (mean error well above the {ACCURACY_THRESHOLD_MM:.0f} mm gate).
- Frames are Link_6 throughout; grasp offset is applied in the Phase 4 pick layer.
"""
        report_dir = Path(__file__).resolve().parents[1] / "benchmark_results"
        report_dir.mkdir(parents=True, exist_ok=True)
        report_path = report_dir / "ik_comparison_report.md"
        with report_path.open("w", encoding="utf-8") as f:
            f.write(report)
        print(f"\n--- IK Comparison Report generated at: {report_path} ---")
        print(report)


def main(args=None):
    rclpy.init(args=args)
    node = IKComparisonBenchmarker()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    num_samples = int(sys.argv[1]) if len(sys.argv) > 1 else 500
    try:
        node.wait_for_services()
        node.benchmark(num_samples=num_samples)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
