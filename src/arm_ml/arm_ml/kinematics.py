"""URDF-derived Link_6 FK helpers used by the ML dataset generator."""

from __future__ import annotations

import math
from typing import Iterable, Tuple

import numpy as np

JOINT_LIMITS = np.array(
    [[-3.14, 3.14], [-2.86, 1.25], [-3.30, 0.94], [-3.14, 3.14], [-2.44, 2.33], [-3.14, 3.14]],
    dtype=np.float32,
)
POSITION_SCALE = np.array([1.0, 1.0, 1.0], dtype=np.float32)
URDF_JOINTS: Tuple[Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]], ...] = (
    ((0.0, 0.0, 0.084), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    ((0.11106, 0.11837, 0.16), (-1.5708, 0.0, 1.2093), (0.0, 0.0, -1.0)),
    ((-0.019287, -0.29938, 0.0020019), (0.0, 0.0, 0.0), (0.0, 0.0, -1.0)),
    ((-0.22999, 0.014126, 0.088023), (1.5708, 1.5199, -1.4356), (0.0, 0.0, -1.0)),
    ((-0.011421, 0.0051115, 0.133), (1.5707, 0.74125, 1.1495), (0.0, 0.0, 1.0)),
    ((-0.073341, 0.073736, 0.0125), (-1.5708, -1.1641, 0.78271), (0.0, 0.0, 1.0)),
)


def _rpy_to_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    return rz @ ry @ rx


def _origin_transform(xyz: Iterable[float], rpy: Iterable[float]) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = _rpy_to_rotation(*rpy)
    transform[:3, 3] = np.asarray(tuple(xyz), dtype=np.float64)
    return transform


def _axis_rotation(axis_values: Iterable[float], angle: float) -> np.ndarray:
    axis = np.asarray(tuple(axis_values), dtype=np.float64)
    axis = axis / np.linalg.norm(axis) if np.linalg.norm(axis) > 1e-12 else np.array([0.0, 0.0, 1.0])
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    cc = 1.0 - c
    rot = np.array(
        [[c + x * x * cc, x * y * cc - z * s, x * z * cc + y * s],
         [y * x * cc + z * s, c + y * y * cc, y * z * cc - x * s],
         [z * x * cc - y * s, z * y * cc + x * s, c + z * z * cc]],
        dtype=np.float64,
    )
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rot
    return transform


def _joint_transform(joint, angle: float) -> np.ndarray:
    xyz, rpy, axis = joint
    return _origin_transform(xyz, rpy) @ _axis_rotation(axis, angle)


def quaternion_from_matrix(rotation: np.ndarray) -> np.ndarray:
    trace = np.trace(rotation)
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = np.array([(rotation[2, 1] - rotation[1, 2]) / s, (rotation[0, 2] - rotation[2, 0]) / s, (rotation[1, 0] - rotation[0, 1]) / s, 0.25 * s])
    else:
        idx = int(np.argmax(np.diag(rotation)))
        if idx == 0:
            s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            q = np.array([0.25 * s, (rotation[0, 1] + rotation[1, 0]) / s, (rotation[0, 2] + rotation[2, 0]) / s, (rotation[2, 1] - rotation[1, 2]) / s])
        elif idx == 1:
            s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            q = np.array([(rotation[0, 1] + rotation[1, 0]) / s, 0.25 * s, (rotation[1, 2] + rotation[2, 1]) / s, (rotation[0, 2] - rotation[2, 0]) / s])
        else:
            s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            q = np.array([(rotation[0, 2] + rotation[2, 0]) / s, (rotation[1, 2] + rotation[2, 1]) / s, 0.25 * s, (rotation[1, 0] - rotation[0, 1]) / s])
    q = q / np.linalg.norm(q)
    if q[3] < 0.0:
        q = -q
    return q.astype(np.float32)


def forward_kinematics(joint_angles: Iterable[float]) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    for joint, angle in zip(URDF_JOINTS, joint_angles):
        transform = transform @ _joint_transform(joint, float(angle))
    return transform


def pose7_from_joints(joint_angles: Iterable[float]) -> np.ndarray:
    transform = forward_kinematics(joint_angles)
    return np.concatenate([transform[:3, 3], quaternion_from_matrix(transform[:3, :3])]).astype(np.float32)


def normalize_joints(joints: np.ndarray) -> np.ndarray:
    center = (JOINT_LIMITS[:, 0] + JOINT_LIMITS[:, 1]) / 2.0
    half_range = (JOINT_LIMITS[:, 1] - JOINT_LIMITS[:, 0]) / 2.0
    return ((joints - center) / half_range).astype(np.float32)


def denormalize_joints(normalized: np.ndarray) -> np.ndarray:
    center = (JOINT_LIMITS[:, 0] + JOINT_LIMITS[:, 1]) / 2.0
    half_range = (JOINT_LIMITS[:, 1] - JOINT_LIMITS[:, 0]) / 2.0
    return (normalized * half_range + center).astype(np.float32)


def normalize_pose(pose7: np.ndarray) -> np.ndarray:
    normalized = pose7.astype(np.float32).copy()
    normalized[..., :3] = normalized[..., :3] / POSITION_SCALE
    return normalized


def denormalize_pose(normalized_pose7: np.ndarray) -> np.ndarray:
    pose = normalized_pose7.astype(np.float32).copy()
    pose[..., :3] = pose[..., :3] * POSITION_SCALE
    return pose
