#!/usr/bin/env python3
"""Records (base->wrist, camera->board) pairs for eye-in-hand calibration.

The operator moves the arm; this tool only observes. It never commands motion.

Run it on the PC, not the Jetson: chessboard detection at 640x480 was measured
at 717 ms on the Nano versus ~15 Hz on the PC, and the Nano also has to run the
control loop. The Nano publishes, the PC processes.

Quality gating is deliberate and refuses rather than warns:
  * the detector must report detected=True
  * reprojection must be under --max-reproj-px
  * the TF and the board pose must be within --max-age-s of each other, so a
    stale wrist pose is never paired with a fresh board pose (that pairing error
    is indistinguishable from a calibration error later)

Two modes:
  capture   -- one sample per ENTER, at whatever pose the operator has set
  repeat    -- N samples WITHOUT moving, to measure the noise floor of the
               observation itself before blaming the mechanism

Sample poses should differ in ORIENTATION, not just position: hand-eye is
degenerate under pure translation, and near-degenerate if every rotation shares
an axis. Vary all of them.
"""
import argparse
import json
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32
from tf2_ros import Buffer, TransformListener


class HandEyeCapture(Node):
    def __init__(self, args):
        super().__init__("hand_eye_capture")
        self.args = args
        self.board_pose = None
        self.board_stamp = 0.0
        self.reproj = None
        self.detected = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            PoseStamped, "/vision_encoder/board_pose", self._on_pose, 10)
        self.create_subscription(
            Float32, "/vision_encoder/reprojection_px", self._on_reproj, 10)
        self.create_subscription(
            Bool, "/vision_encoder/detected", self._on_detected,
            qos_profile_sensor_data)

        self.samples = []

    def _on_pose(self, msg):
        self.board_pose = msg
        self.board_stamp = time.monotonic()

    def _on_reproj(self, msg):
        self.reproj = float(msg.data)

    def _on_detected(self, msg):
        self.detected = bool(msg.data)

    def spin_briefly(self, seconds=0.3):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def status_line(self):
        age = time.monotonic() - self.board_stamp if self.board_pose else float("inf")
        return ("detected=%s reproj=%s board_age=%.2fs samples=%d"
                % (self.detected,
                   "%.3f" % self.reproj if self.reproj is not None else "?",
                   age, len(self.samples)))

    def grab(self):
        """Returns (sample, reason). sample is None when a gate refuses."""
        self.spin_briefly(0.4)
        if not self.detected:
            return None, "board not detected"
        if self.board_pose is None:
            return None, "no board pose received yet"
        age = time.monotonic() - self.board_stamp
        if age > self.args.max_age_s:
            return None, "board pose is %.2fs stale" % age
        if self.reproj is None:
            return None, "no reprojection value received"
        if self.reproj > self.args.max_reproj_px:
            return None, "reprojection %.3f px over gate %.3f" % (
                self.reproj, self.args.max_reproj_px)

        wrist_entry = None
        if not self.args.no_tf:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.args.base_frame, self.args.wrist_frame,
                    rclpy.time.Time())
            except Exception as error:  # tf2 raises several unrelated types
                return None, "TF %s -> %s unavailable: %s" % (
                    self.args.base_frame, self.args.wrist_frame, error)
            wrist_entry = {
                "xyz": [tf.transform.translation.x,
                        tf.transform.translation.y,
                        tf.transform.translation.z],
                "quat_xyzw": [tf.transform.rotation.x, tf.transform.rotation.y,
                              tf.transform.rotation.z, tf.transform.rotation.w],
            }

        pose = self.board_pose.pose
        sample = {
            "base_to_wrist": wrist_entry,
            "cam_to_board": {
                "xyz": [pose.position.x, pose.position.y, pose.position.z],
                "quat_xyzw": [pose.orientation.x, pose.orientation.y,
                              pose.orientation.z, pose.orientation.w],
            },
            "reprojection_px": self.reproj,
            "board_frame_id": self.board_pose.header.frame_id,
        }
        return sample, "ok"

    def report_noise_floor(self):
        """Scatter of the board pose with nothing moving.

        This is the floor that later hand-eye scatter must be compared against.
        Without it, a 6 mm field result cannot be attributed: camera or arm?
        """
        if len(self.samples) < 3:
            print("\nnot enough samples for a noise floor (%d)" % len(self.samples))
            return
        import math

        positions = [s["cam_to_board"]["xyz"] for s in self.samples]
        centre = [sum(axis) / len(positions) for axis in zip(*positions)]
        deviations = [
            math.sqrt(sum((p[i] - centre[i]) ** 2 for i in range(3)))
            for p in positions
        ]
        stds = [
            math.sqrt(sum((p[i] - centre[i]) ** 2 for p in positions) / len(positions))
            for i in range(3)
        ]
        reprojections = [s["reprojection_px"] for s in self.samples]
        print("\n=== observation noise floor (%d samples, nothing moving) ==="
              % len(self.samples))
        print("  position std   : %.4f / %.4f / %.4f mm (x/y/z)"
              % tuple(v * 1000.0 for v in stds))
        print("  max deviation  : %.4f mm" % (max(deviations) * 1000.0))
        print("  distance       : %.1f cm" % (
            math.sqrt(sum(v * v for v in centre)) * 100.0))
        print("  reprojection   : min %.3f  mean %.3f  max %.3f px"
              % (min(reprojections),
                 sum(reprojections) / len(reprojections),
                 max(reprojections)))
        print("  -> compare the hand-eye board scatter against THIS, not zero.")

    def save(self, path):
        payload = {
            "meta": {
                "base_frame": self.args.base_frame,
                "wrist_frame": self.args.wrist_frame,
                "max_reproj_px": self.args.max_reproj_px,
                "sample_count": len(self.samples),
                "note": "base_to_wrist from TF (OPEN LOOP: commanded angles, no "
                        "encoders); cam_to_board from PnP",
            },
            "samples": self.samples,
        }
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w") as handle:
            json.dump(payload, handle, indent=2)
        print("wrote %d samples to %s" % (len(self.samples), path))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="runs/hand_eye/samples.json")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--wrist-frame", default="link_5",
                        help="parent link of camera_mount_joint")
    parser.add_argument("--max-reproj-px", type=float, default=1.0)
    parser.add_argument("--max-age-s", type=float, default=0.5)
    parser.add_argument("--repeat", type=int, default=0,
                        help="grab N samples without moving (noise floor)")
    parser.add_argument("--repeat-interval-s", type=float, default=0.4)
    parser.add_argument("--no-tf", action="store_true",
                        help="skip the wrist TF. Only valid for --repeat: the "
                             "observation noise floor is a property of the "
                             "camera and PnP alone, so it can be measured with "
                             "the arm stack down. Samples taken this way CANNOT "
                             "be used to solve hand-eye.")
    args = parser.parse_args()

    if args.no_tf and not args.repeat:
        parser.error("--no-tf is only meaningful with --repeat; a hand-eye "
                     "capture without the wrist pose is unusable")

    rclpy.init()
    node = HandEyeCapture(args)
    print("waiting for the vision encoder...")
    node.spin_briefly(2.0)

    try:
        if args.repeat:
            print("REPEAT MODE: %d samples, DO NOT MOVE THE ARM." % args.repeat)
            print("This measures the observation noise floor, so that later")
            print("scatter can be attributed to the mechanism and not the camera.")
            for index in range(args.repeat):
                sample, reason = node.grab()
                if sample is None:
                    print("  [%2d] refused: %s" % (index + 1, reason))
                else:
                    node.samples.append(sample)
                    print("  [%2d] ok  reproj=%.3f" % (index + 1,
                                                       sample["reprojection_px"]))
                node.spin_briefly(args.repeat_interval_s)
            node.report_noise_floor()
        else:
            print("CAPTURE MODE. Move the arm, then press ENTER to grab.")
            print("Vary ORIENTATION between poses, not just position -- hand-eye")
            print("is degenerate under pure translation.")
            print("Type 'q' + ENTER to finish.")
            while True:
                node.spin_briefly(0.2)
                answer = input("[%s] ENTER=grab, q=quit > " % node.status_line())
                if answer.strip().lower().startswith("q"):
                    break
                sample, reason = node.grab()
                if sample is None:
                    print("  REFUSED: %s" % reason)
                    continue
                node.samples.append(sample)
                print("  captured #%d (reproj %.3f px)"
                      % (len(node.samples), sample["reprojection_px"]))
    except KeyboardInterrupt:
        print()
    finally:
        if node.samples:
            node.save(args.out)
        else:
            print("no samples captured; nothing written")
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
