#!/usr/bin/env python3
"""Create one deterministic domain-randomization preview image."""
import argparse
from pathlib import Path

import cv2
import numpy as np

from arm_perception.domain_randomization import augment_image_and_bbox, sample_augmentation

parser = argparse.ArgumentParser()
parser.add_argument("input", type=Path)
parser.add_argument("output", type=Path)
parser.add_argument("--seed", type=int, default=42)
args = parser.parse_args()
image = cv2.imread(str(args.input))
if image is None:
    raise SystemExit(f"Unable to read {args.input}")
height, width = image.shape[:2]
augmentation = sample_augmentation(np.random.default_rng(args.seed))
preview, _ = augment_image_and_bbox(image, (0, 0, width - 1, height - 1), augmentation)
args.output.parent.mkdir(parents=True, exist_ok=True)
cv2.imwrite(str(args.output), preview)
print(augmentation)
