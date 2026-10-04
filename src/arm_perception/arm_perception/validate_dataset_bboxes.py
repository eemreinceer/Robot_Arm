#!/usr/bin/env python3
"""Validate synthetic YOLO bbox annotations against rendered object pixels."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import random

import cv2
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--samples', type=int, default=20)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--min-iou', type=float, default=0.5)
    parser.add_argument('--output', type=Path, required=True)
    return parser.parse_args()


def bbox_iou(first, second) -> float:
    ax1, ay1, ax2, ay2 = first
    bx1, by1, bx2, by2 = second
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    intersection = max(0, ix2 - ix1 + 1) * max(0, iy2 - iy1 + 1)
    first_area = max(0, ax2 - ax1 + 1) * max(0, ay2 - ay1 + 1)
    second_area = max(0, bx2 - bx1 + 1) * max(0, by2 - by1 + 1)
    union = first_area + second_area - intersection
    return intersection / union if union else 0.0


def object_mask(image: np.ndarray, object_type: str, hue_shift: int):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue, saturation = hsv[:, :, 0].astype(int), hsv[:, :, 1]
    center = {'red_box': 0, 'yellow_cylinder': 30, 'blue_cube': 120}[object_type] + hue_shift
    hue_distance = np.abs(((hue - center + 90) % 180) - 90)
    return ((hue_distance <= 13) & (saturation > 70)).astype(np.uint8)


def pixel_bbox(image: np.ndarray, record):
    mask = object_mask(image, record['class'], int(record['augmentation']['hue_shift']))
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask)
    candidates = []
    ground_truth = tuple(int(round(value)) for value in record['bbox_xyxy'])
    for index in range(1, count):
        x, y, width, height, area = map(int, stats[index])
        if area >= 8:
            bbox = (x, y, x + width - 1, y + height - 1)
            candidates.append((bbox_iou(ground_truth, bbox), bbox))
    return max(candidates, default=(0.0, (0, 0, 0, 0)))


def main() -> None:
    args = parse_args()
    records = [json.loads(line) for line in (args.dataset / 'gt' / 'annotations.jsonl').read_text().splitlines() if line.strip()]
    if len(records) < args.samples:
        raise RuntimeError(f'Dataset has {len(records)} records, expected at least {args.samples}')
    sampled = random.Random(args.seed).sample(records, args.samples)
    tiles, scores = [], []
    for record in sampled:
        image = cv2.imread(str(args.dataset / record['image']))
        if image is None:
            raise RuntimeError(f"Unable to read {record['image']}")
        ground_truth = tuple(int(round(value)) for value in record['bbox_xyxy'])
        score, pixels = pixel_bbox(image, record)
        scores.append(score)
        cv2.rectangle(image, ground_truth[:2], ground_truth[2:], (0, 255, 0), 2)
        if score:
            cv2.rectangle(image, pixels[:2], pixels[2:], (255, 0, 255), 1)
        label = f"{Path(record['image']).stem} IoU={score:.2f}"
        cv2.putText(image, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(image, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (0, 0, 0), 1, cv2.LINE_AA)
        tiles.append(cv2.resize(image, (320, 240)))
    columns = 5
    rows = []
    for start in range(0, len(tiles), columns):
        row = tiles[start:start + columns]
        row.extend([np.zeros_like(tiles[0])] * (columns - len(row)))
        rows.append(np.hstack(row))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), np.vstack(rows))
    passed = sum(score > args.min_iou for score in scores)
    print(f'samples={len(scores)} min_iou={min(scores):.6f} mean_iou={np.mean(scores):.6f} pass_gt_{args.min_iou}={passed}/{len(scores)}')
    print('scores=' + ','.join(f'{score:.3f}' for score in scores))
    print(f'overlay={args.output}')
    if passed != len(scores):
        raise SystemExit('BBox acceptance gate failed')


if __name__ == '__main__':
    main()
