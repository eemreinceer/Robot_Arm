#!/usr/bin/env python3
"""Measures whether the native corner ordering stays stable as the board turns.

WHY THIS EXISTS. The 2026-07-28 flip fix made the board pose repeatable by
trusting `findChessboardCorners`' native ordering and only preferring the
reversed one when it is better by a real margin. That fix carries an explicit
limit, recorded in board_pnp.py:

    native ordering is IMAGE-SPACE dependent (left-to-right, top-to-bottom).
    If the camera approaches the board with a noticeably different orientation
    the native ordering can rotate too. This MUST be re-measured.

The 231-sample, 0.027 mm stability was taken at ONE orientation. This tool
sweeps orientations and answers: where does the ordering actually break?

That matters BEFORE hand-eye, not after: samples collected at an orientation
where the ordering is unstable are already corrupt, and no amount of solving
recovers them.

WHAT IT MEASURES per frame, computing both orderings itself rather than
trusting the node's internal choice:
  * native and reversed reprojection, and the MARGIN between them
    (margin ~ 0 means the two are indistinguishable -- a coin flip)
  * `ambiguous` under the same rule board_pnp.py uses
  * corner[0] in image space. If the ordering flips, corner[0] jumps from one
    end of the board to the other. That is a direct, discontinuous observable
    that needs no pose reasoning to detect.
  * in-plane angle of the corner[0] -> corner[-1] vector, which is the variable
    the native ordering actually depends on
  * board tilt away from the camera axis, and distance

Run on the PC while turning the board by hand in front of a STATIONARY camera.
The arm does not need to move and the servo rail can stay off -- this measures
the detector, not the mechanism.
"""
import argparse
import json
import math
import os
import sys

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from arm_perception import board_pnp
from arm_perception.board_pnp import (
    DEFAULT_BOARD_COLS,
    DEFAULT_BOARD_ROWS,
    DEFAULT_SQUARE_SIZE_MM,
)

# Same threshold board_pnp.py decides with, so the verdict here matches what
# the live node would have done.
FLIP_MARGIN_PX = 0.05


class FlipSweep(Node):
    def __init__(self, args):
        super().__init__("flip_sweep")
        self.args = args
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.records = []
        self.last_corner0 = None
        self.last_in_plane = 0.0
        self.flip_events = []

        self.create_subscription(CameraInfo, "/camera/camera_info",
                                 self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/image_raw",
                                 self._on_image, qos_profile_sensor_data)

    def _on_info(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
            self.get_logger().info("intrinsics received")

    def _on_image(self, msg):
        if self.camera_matrix is None:
            return
        import cv2

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners = board_pnp.detect_corners(gray, self.args.cols, self.args.rows)
        if corners is None:
            return
        objp = board_pnp.board_object_points(
            self.args.cols, self.args.rows, self.args.square_mm)

        results = {}
        for name, pts in (("native", corners), ("reversed", corners[::-1].copy())):
            ok, rvec, tvec = board_pnp.solve_pose(
                objp, pts, self.camera_matrix, self.dist_coeffs)
            if not ok:
                return
            results[name] = {
                "reproj": board_pnp.reprojection_error_px(
                    objp, pts, rvec, tvec, self.camera_matrix, self.dist_coeffs),
                "rvec": rvec.reshape(3).tolist(),
                "tvec": tvec.reshape(3).tolist(),
            }

        margin = results["reversed"]["reproj"] - results["native"]["reproj"]
        chosen = "reversed" if margin < -FLIP_MARGIN_PX else "native"
        ambiguous = abs(margin) < FLIP_MARGIN_PX

        corner0 = corners[0].reshape(2)
        corner_last = corners[-1].reshape(2)
        vector = corner_last - corner0
        in_plane_deg = math.degrees(math.atan2(float(vector[1]), float(vector[0])))

        rotation, _ = cv2.Rodrigues(np.array(results[chosen]["rvec"]))
        normal = rotation @ np.array([0.0, 0.0, 1.0])
        tilt_deg = math.degrees(math.acos(min(1.0, abs(float(normal[2])))))
        distance = float(np.linalg.norm(results[chosen]["tvec"]))

        # An ordering flip moves corner[0] to the far end of the board. Between
        # consecutive frames of a hand-moved board that is a discontinuity no
        # physical motion produces.
        # A raw pixel threshold alone gives FALSE POSITIVES: a quick hand move
        # displaces corner[0] too. Measured 2026-07-29 on the 6x9 board -- three
        # "jumps" were all hand motion. A genuine ordering flip leaves two marks
        # together, and fast motion leaves neither:
        #   1. corner[0] moves about the board's DIAGONAL, since it lands at the
        #      opposite end of the grid;
        #   2. the in-plane angle turns by ~180 degrees.
        # Both are required before calling it a flip.
        jumped = None
        if self.last_corner0 is not None:
            jump = float(np.linalg.norm(corner0 - self.last_corner0))
            focal = float(self.camera_matrix[0, 0])
            span_x = focal * (self.args.cols - 1) * self.args.square_mm / 1000.0 / distance
            span_y = focal * (self.args.rows - 1) * self.args.square_mm / 1000.0 / distance
            diagonal_px = math.hypot(span_x, span_y)
            angle_delta = (in_plane_deg - self.last_in_plane + 180.0) % 360.0 - 180.0
            looks_like_flip = (jump > 0.7 * diagonal_px and
                               abs(abs(angle_delta) - 180.0) < 45.0)
            if jump > self.args.jump_px:
                jumped = jump
                self.flip_events.append({
                    "index": len(self.records),
                    "corner0_jump_px": round(jump, 1),
                    "expected_flip_px": round(diagonal_px, 1),
                    "angle_delta_deg": round(angle_delta, 1),
                    "classified": "ORDERING_FLIP" if looks_like_flip else "hand_motion",
                    "in_plane_deg": round(in_plane_deg, 1),
                    "tilt_deg": round(tilt_deg, 1),
                    "margin_px": round(margin, 4),
                    "ambiguous": ambiguous,
                })
        self.last_corner0 = corner0
        self.last_in_plane = in_plane_deg

        self.records.append({
            "native_reproj_px": round(results["native"]["reproj"], 4),
            "reversed_reproj_px": round(results["reversed"]["reproj"], 4),
            "margin_px": round(margin, 4),
            "chosen": chosen,
            "ambiguous": ambiguous,
            "in_plane_deg": round(in_plane_deg, 1),
            "tilt_deg": round(tilt_deg, 1),
            "distance_m": round(distance, 4),
            "corner0_px": [round(float(corner0[0]), 1), round(float(corner0[1]), 1)],
        })

        # Save as we go. The first real bench sweep was killed with SIGKILL and
        # lost 6500 frames because the only write happened at exit. A recording
        # tool that can only deliver its data by exiting cleanly is a tool that
        # loses bench time.
        if len(self.records) % self.args.save_every == 0:
            self.save(self.args.out, quiet=True)

        if len(self.records) % self.args.log_every == 0 or jumped:
            flag = ""
            if jumped:
                flag = "  <== CORNER0 JUMPED %.0f px" % jumped
            print("  n=%4d  in-plane %+7.1f deg  tilt %5.1f deg  %.2f m  "
                  "native %.3f  rev %.3f  margin %+.4f  %s%s"
                  % (len(self.records), in_plane_deg, tilt_deg, distance,
                     results["native"]["reproj"], results["reversed"]["reproj"],
                     margin, "AMBIGUOUS" if ambiguous else chosen, flag))

    def summarise(self):
        if not self.records:
            print("\nno frames with a detected board; nothing to report")
            return
        margins = [abs(r["margin_px"]) for r in self.records]
        ambiguous = [r for r in self.records if r["ambiguous"]]
        reversed_chosen = [r for r in self.records if r["chosen"] == "reversed"]
        angles = [r["in_plane_deg"] for r in self.records]
        tilts = [r["tilt_deg"] for r in self.records]

        print("\n=== flip sweep: %d frames with a detected board ===" % len(self.records))
        print("  in-plane angle swept : %.1f .. %.1f deg" % (min(angles), max(angles)))
        print("  tilt swept           : %.1f .. %.1f deg" % (min(tilts), max(tilts)))
        print("  |margin| min/median/max : %.4f / %.4f / %.4f px"
              % (min(margins), sorted(margins)[len(margins) // 2], max(margins)))
        print("  ambiguous frames     : %d / %d (%.1f%%)"
              % (len(ambiguous), len(self.records),
                 100.0 * len(ambiguous) / len(self.records)))
        print("  reversed ordering won: %d frames" % len(reversed_chosen))
        real = [e for e in self.flip_events if e.get("classified") == "ORDERING_FLIP"]
        motion = [e for e in self.flip_events if e.get("classified") != "ORDERING_FLIP"]
        print("  corner0 jumps        : %d  (%d ordering flips, %d hand motion)"
              % (len(self.flip_events), len(real), len(motion)))
        for event in self.flip_events[:12]:
            print("     idx %-5d jump %6.1f px (flip would be ~%.0f) "
                  "angle %+7.1f  -> %s"
                  % (event["index"], event["corner0_jump_px"],
                     event.get("expected_flip_px", 0.0),
                     event.get("angle_delta_deg", 0.0),
                     event.get("classified", "?")))
        if len(self.flip_events) > 12:
            print("     ... %d more" % (len(self.flip_events) - 12))

        print()
        if not real:
            print("  VERDICT: native ordering held across the whole swept range")
            print("  (%d raw jump(s) were hand motion, not ordering flips)."
                  % len(motion))
            print("  NOTE: 100% ambiguous is EXPECTED and not a failure -- the")
            print("  reprojection margin is ~0 for ANY rectangular board, because a")
            print("  rectangular object-point grid is 180-degree symmetric. Parity")
            print("  buys DETECTOR stability, which is what the jump count measures.")
        elif real:
            print("  VERDICT: the ordering DID flip. Hand-eye samples must avoid")
            print("  these orientations, or the board must be replaced with one")
            print("  whose cols+rows is ODD (e.g. 6x9) / a ChArUco target.")
        else:
            print("  VERDICT: no discontinuity seen, but %d frames were ambiguous"
                  " -- the two orderings were indistinguishable there, so the"
                  " choice was luck, not evidence." % len(ambiguous))

    def save(self, path, quiet=False):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w") as handle:
            json.dump({
                "meta": {
                    "cols": self.args.cols, "rows": self.args.rows,
                    "square_mm": self.args.square_mm,
                    "flip_margin_px": FLIP_MARGIN_PX,
                    "jump_px": self.args.jump_px,
                    "frames": len(self.records),
                    "note": "board moved by hand in front of a stationary camera; "
                            "measures the detector, not the mechanism",
                },
                "flip_events": self.flip_events,
                "records": self.records,
            }, handle, indent=2)
        if not quiet:
            print("wrote %d frames to %s" % (len(self.records), path))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cols", type=int, default=DEFAULT_BOARD_COLS)
    parser.add_argument("--rows", type=int, default=DEFAULT_BOARD_ROWS)
    parser.add_argument("--square-mm", type=float, default=DEFAULT_SQUARE_SIZE_MM)
    parser.add_argument("--jump-px", type=float, default=60.0,
                        help="corner[0] displacement between frames that counts "
                             "as an ordering flip rather than board motion")
    parser.add_argument("--log-every", type=int, default=10)
    parser.add_argument("--save-every", type=int, default=200,
                        help="flush to disk every N frames so a killed run "
                             "still leaves its data behind")
    parser.add_argument("--out", default="runs/flip_sweep/sweep.json")
    args = parser.parse_args()

    rclpy.init()
    node = FlipSweep(args)
    print("Turn the board slowly in front of the camera.")
    print("Cover: in-plane rotation (spin it in its own plane) AND tilt.")
    print("Go slowly -- a fast move looks like an ordering flip to the jump test.")
    print("Ctrl-C when done.\n")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print()
    except rclpy.executors.ExternalShutdownException:
        # Killed rather than Ctrl-C'd. Still report: a sweep that was cut short
        # measured something, and silently discarding it wastes the operator's
        # time at the bench.
        print("\n(interrupted externally)")
    finally:
        node.summarise()
        if node.records:
            node.save(args.out)
        node.destroy_node()
        # SIGTERM can tear the context down before we get here; shutting down
        # twice raises and would bury the summary printed above.
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
