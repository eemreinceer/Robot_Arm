"""Image-space domain randomization helpers for synthetic Gazebo captures."""

from __future__ import annotations

from dataclasses import dataclass
import cv2
import numpy as np


@dataclass(frozen=True)
class Augmentation:
    brightness: float
    hue_shift: int
    camera_roll_deg: float


def sample_augmentation(rng: np.random.Generator) -> Augmentation:
    return Augmentation(float(rng.uniform(0.8, 1.2)), int(rng.integers(-10, 11)), float(rng.uniform(-5.0, 5.0)))


def augment_image_and_bbox(image: np.ndarray, bbox, augmentation: Augmentation):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV).astype(np.float32)
    hsv[:, :, 0] = (hsv[:, :, 0] + augmentation.hue_shift) % 180
    hsv[:, :, 2] = np.clip(hsv[:, :, 2] * augmentation.brightness, 0, 255)
    adjusted = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)
    height, width = adjusted.shape[:2]
    matrix = cv2.getRotationMatrix2D((width / 2.0, height / 2.0), augmentation.camera_roll_deg, 1.0)
    rotated = cv2.warpAffine(adjusted, matrix, (width, height), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
    x1, y1, x2, y2 = bbox
    corners = np.array([[x1, y1, 1], [x2, y1, 1], [x2, y2, 1], [x1, y2, 1]], dtype=np.float32)
    transformed = corners @ matrix.T
    out = [float(np.clip(transformed[:, 0].min(), 0, width - 1)), float(np.clip(transformed[:, 1].min(), 0, height - 1)), float(np.clip(transformed[:, 0].max(), 0, width - 1)), float(np.clip(transformed[:, 1].max(), 0, height - 1))]
    return rotated, out


def yolo_label(class_id: int, bbox, width: int, height: int) -> str:
    x1, y1, x2, y2 = bbox
    return f"{class_id} {((x1 + x2) / 2.0) / width:.6f} {((y1 + y2) / 2.0) / height:.6f} {(x2 - x1) / width:.6f} {(y2 - y1) / height:.6f}"
