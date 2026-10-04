#!/usr/bin/env python3
"""Train and validate the Faz 4 YOLO detector."""
from __future__ import annotations

import argparse
from pathlib import Path
import shutil


def parse_args():
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', type=Path, default=package / 'datasets' / 'pick_objects' / 'data.yaml')
    parser.add_argument('--base-model', default='yolov8n.pt')
    parser.add_argument('--epochs', type=int, default=80)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--batch', type=int, default=16)
    parser.add_argument('--patience', type=int, default=15)
    parser.add_argument('--workers', type=int, default=8)
    parser.add_argument('--device', default=None)
    parser.add_argument('--output', type=Path, default=package / 'models' / 'yolo_arm.pt')
    parser.add_argument('--project', type=Path, default=package / 'runs')
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    from ultralytics import YOLO
    model = YOLO(args.base_model)
    result = model.train(
        data=str(args.data), epochs=args.epochs, imgsz=args.imgsz,
        batch=args.batch, patience=args.patience, workers=args.workers, device=args.device,
        project=str(args.project.resolve()), name='faz4_yolo', exist_ok=True)
    save_dir = Path(result.save_dir)
    best = save_dir / 'weights' / 'best.pt'
    if not best.is_file():
        raise RuntimeError(f'Training finished without best checkpoint: {best}')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(best, args.output)
    metrics = YOLO(str(args.output)).val(data=str(args.data), split='test', workers=args.workers)
    print(f'model={args.output}')
    print(f'mAP50={metrics.box.map50:.6f}')
    print(f'mAP50-95={metrics.box.map:.6f}')


if __name__ == '__main__':
    main()
