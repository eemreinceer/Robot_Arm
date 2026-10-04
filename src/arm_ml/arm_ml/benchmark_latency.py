#!/usr/bin/env python3
"""Benchmark IKNet inference latency using a local checkpoint."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

import numpy as np
import torch

from arm_ml.model import IKNet


def load_any(path: Path):
    if path.name.endswith("_scripted.pt"):
        model = torch.jit.load(str(path), map_location="cpu")
    else:
        model = IKNet()
        checkpoint = torch.load(path, map_location="cpu")
        model.load_state_dict(checkpoint.get("model_state_dict", checkpoint))
    model.eval()
    return model


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=Path("src/arm_ml/models/ik_net_scripted.pt"))
    parser.add_argument("--requests", type=int, default=1000)
    args = parser.parse_args()
    model = load_any(args.model)
    samples = torch.from_numpy(np.random.uniform(-1.0, 1.0, size=(args.requests, 7)).astype(np.float32))
    with torch.no_grad():
        for _ in range(10):
            model(samples[:1])
        started = time.perf_counter()
        for i in range(args.requests):
            model(samples[i:i + 1])
        elapsed_ms = (time.perf_counter() - started) * 1000.0
    print(f"requests={args.requests} total_ms={elapsed_ms:.3f} mean_ms={elapsed_ms / args.requests:.3f}")


if __name__ == "__main__":
    main()
