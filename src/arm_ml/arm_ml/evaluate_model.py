#!/usr/bin/env python3
"""Evaluate IKNet FK reconstruction error on a generated dataset split."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import torch

from arm_ml.benchmark_latency import load_any
from arm_ml.train import torch_fk_positions


SPLITS = {"train": 0, "val": 1, "test": 2}


def evaluate(dataset: Path, model_path: Path, split_name: str, batch_size: int) -> None:
    loaded = np.load(dataset)
    mask = loaded["split"] == SPLITS[split_name]
    poses = torch.from_numpy(loaded["normalized_poses"][mask].astype(np.float32))
    model = load_any(model_path)
    errors = []
    with torch.no_grad():
        for start in range(0, poses.shape[0], batch_size):
            batch = poses[start:start + batch_size]
            predicted_positions = torch_fk_positions(model(batch))
            errors.append(torch.linalg.vector_norm(predicted_positions - batch[:, :3], dim=1))
    error_mm = torch.cat(errors).numpy() * 1000.0
    print(f"split={split_name} samples={error_mm.size}")
    print(f"mean_mm={np.mean(error_mm):.6f}")
    print(f"median_mm={np.median(error_mm):.6f}")
    print(f"p95_mm={np.percentile(error_mm, 95):.6f}")
    print(f"max_mm={np.max(error_mm):.6f}")
    print(f"rmse_mm={np.sqrt(np.mean(np.square(error_mm))):.6f}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, default=Path("src/arm_ml/data/ik_dataset_v1.npz"))
    parser.add_argument("--model", type=Path, default=Path("src/arm_ml/models/ik_net_scripted.pt"))
    parser.add_argument("--split", choices=sorted(SPLITS), default="test")
    parser.add_argument("--batch-size", type=int, default=2048)
    args = parser.parse_args()
    evaluate(args.dataset, args.model, args.split, args.batch_size)


if __name__ == "__main__":
    main()
