"""RGB-only image-to-table projection helpers.

The homography maps image pixels directly into the robot target frame's XY
table plane.  Object height is then used to lift the reported centre above the
plane; no depth image or point cloud is involved.

VALIDITY (2026-07-26): a single constant homography encodes ONE camera pose
relative to the table.  The Robot Arm camera is EYE-IN-HAND (mounted on the arm,
gripper fingers visible in frame), so the image-to-table mapping changes with
every joint motion.  A stored homography is therefore only valid while the arm
sits at the exact joint pose it was captured from — see the observation-pose
gate in rgb_planar_detector_node.py, which refuses frames taken elsewhere.
A pose-independent projection needs the measured hand-eye transform plus a
per-frame TF lookup, not a fixed matrix; that work is still open.
"""
from typing import Mapping, Sequence, Tuple

import numpy as np


def bbox_contact_pixel(bbox_xyxy: Sequence[float]) -> Tuple[float, float]:
    """Return the bottom-centre pixel, a practical table contact estimate."""
    if len(bbox_xyxy) != 4:
        raise ValueError('bbox_xyxy must contain x_min, y_min, x_max, y_max')
    x_min, y_min, x_max, y_max = (float(value) for value in bbox_xyxy)
    if x_max <= x_min or y_max <= y_min:
        raise ValueError('bbox_xyxy must have positive width and height')
    return ((x_min + x_max) * 0.5, y_max)


def image_pixel_to_plane_xy(
        pixel_uv: Sequence[float], homography: Sequence[float]) -> Tuple[float, float]:
    """Project an image pixel through a row-major 3x3 pixel-to-plane matrix."""
    matrix = np.asarray(homography, dtype=np.float64)
    if matrix.size != 9:
        raise ValueError('homography must contain exactly 9 values')
    matrix = matrix.reshape((3, 3))
    pixel = np.asarray([float(pixel_uv[0]), float(pixel_uv[1]), 1.0])
    projected = matrix @ pixel
    if abs(projected[2]) < 1e-12:
        raise ValueError('homography maps pixel to a point at infinity')
    return (float(projected[0] / projected[2]), float(projected[1] / projected[2]))


def bbox_to_object_position(
        bbox_xyxy: Sequence[float], homography: Sequence[float], table_z_m: float,
        object_height_m: float) -> Tuple[float, float, float]:
    """Estimate an object's target-frame centre from bbox and known height."""
    if object_height_m <= 0.0:
        raise ValueError('object_height_m must be positive')
    x_m, y_m = image_pixel_to_plane_xy(bbox_contact_pixel(bbox_xyxy), homography)
    return (x_m, y_m, float(table_z_m) + 0.5 * float(object_height_m))


def object_dimensions(
        class_name: str, dimensions_by_class: Mapping[str, Sequence[float]]) -> Tuple[float, float, float]:
    """Return configured XYZ dimensions and validate the calibration contract."""
    if class_name not in dimensions_by_class:
        raise KeyError('missing dimensions for class: ' + class_name)
    values = tuple(float(value) for value in dimensions_by_class[class_name])
    if len(values) != 3 or any(value <= 0.0 for value in values):
        raise ValueError('class dimensions must contain three positive values')
    return values
