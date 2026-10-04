"""Point-cloud based 6DOF object pose estimation."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from arm_perception.math3d import quaternion_from_matrix


@dataclass
class PoseEstimate:
    position: np.ndarray
    quaternion: np.ndarray
    dimensions: np.ndarray
    point_count: int


def camera_link_to_optical_points(points: np.ndarray) -> np.ndarray:
    """Convert Gazebo RGB-D +X-forward points to REP-103 optical coordinates."""
    cloud = np.asarray(points, dtype=np.float64)
    optical = np.empty_like(cloud)
    optical[..., 0] = -cloud[..., 1]
    optical[..., 1] = -cloud[..., 2]
    optical[..., 2] = cloud[..., 0]
    return optical


def top_pick_link6_transform(target_from_object: np.ndarray, offset_xyz) -> np.ndarray:
    """Place Link_6 above an object while leaving wrist orientation unconstrained."""
    target_from_link6 = np.asarray(target_from_object, dtype=np.float64).copy()
    target_from_link6[:3, 3] += np.asarray(offset_xyz, dtype=np.float64)
    return target_from_link6


def points_from_bbox(points: np.ndarray, width: int, height: int, bbox, depth_band_m: float = 0.08) -> np.ndarray:
    cloud = np.asarray(points, dtype=np.float64).reshape(height, width, 3)
    x1, y1, x2, y2 = [int(round(value)) for value in bbox]
    x1, x2 = sorted((max(0, x1), min(width, x2)))
    y1, y2 = sorted((max(0, y1), min(height, y2)))
    crop = cloud[y1:y2, x1:x2].reshape(-1, 3)
    valid = crop[np.isfinite(crop).all(axis=1) & (crop[:, 2] > 0.0)]
    if valid.size == 0:
        return valid
    nearest = float(np.percentile(valid[:, 2], 10))
    foreground = valid[valid[:, 2] <= nearest + depth_band_m]
    return foreground if foreground.shape[0] >= 12 else valid


def estimate_pose(points: np.ndarray, minimum_points: int = 20) -> PoseEstimate:
    cloud = np.asarray(points, dtype=np.float64)
    if cloud.shape[0] < minimum_points:
        raise ValueError(f"point cloud cluster has only {cloud.shape[0]} points")
    median_center = np.median(cloud, axis=0)
    centered = cloud - median_center
    covariance = np.cov(centered, rowvar=False)
    values, vectors = np.linalg.eigh(covariance)
    order = np.argsort(values)[::-1]
    axes = vectors[:, order]
    if np.linalg.det(axes) < 0.0:
        axes[:, 2] *= -1.0
    # Keep the object normal facing the camera for deterministic PCA orientation.
    if axes[2, 2] > 0.0:
        axes[:, 2] *= -1.0
        axes[:, 1] *= -1.0
    local = centered @ axes
    low, high = np.percentile(local, [2, 98], axis=0)
    dimensions = np.maximum(high - low, 1e-4)
    raw_low, raw_high = np.percentile(cloud, [2, 98], axis=0)
    object_center = (raw_low + raw_high) * 0.5
    return PoseEstimate(object_center.astype(np.float32), quaternion_from_matrix(axes).astype(np.float32), dimensions.astype(np.float32), cloud.shape[0])
