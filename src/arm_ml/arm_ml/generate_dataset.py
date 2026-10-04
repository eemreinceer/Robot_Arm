#!/usr/bin/env python3
"""Generate synthetic IK training pairs using the URDF-derived FK model."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from arm_ml.kinematics import JOINT_LIMITS, normalize_joints, normalize_pose, pose7_from_joints


def generate_dataset(samples: int, output: Path, seed: int = 7) -> None:
    rng = np.random.default_rng(seed)
    joints = rng.uniform(
        low=JOINT_LIMITS[:, 0],
        high=JOINT_LIMITS[:, 1],
        size=(samples, 6),
    ).astype(np.float32)
    poses = np.stack([pose7_from_joints(q) for q in joints], axis=0).astype(np.float32)
    normalized_joints = normalize_joints(joints)
    normalized_poses = normalize_pose(poses)

    train_end = int(samples * 0.70)
    val_end = train_end + int(samples * 0.15)
    split = np.empty(samples, dtype=np.uint8)
    split[:train_end] = 0
    split[train_end:val_end] = 1
    split[val_end:] = 2

    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output,
        joints=joints,
        poses=poses,
        normalized_joints=normalized_joints,
        normalized_poses=normalized_poses,
        split=split,
        joint_limits=JOINT_LIMITS.astype(np.float32),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", "--num_samples", dest="samples", type=int, default=100_000)
    parser.add_argument("--output", type=Path, default=Path("src/arm_ml/data/ik_dataset_v1.npz"))
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()
    generate_dataset(args.samples, args.output, args.seed)
    print(f"wrote {args.samples} samples to {args.output}")


if __name__ == "__main__":
    main()
