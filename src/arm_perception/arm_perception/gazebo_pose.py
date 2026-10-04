"""Gazebo ground-truth model pose helpers for simulation sorting.

The camera/YOLO pipeline is still used to decide that a fresh object is visible,
but the sorting simulation can read exact model poses from Gazebo before
building pick/place targets. This keeps logs honest: action success is not used
as proof that the object or bin coordinates were correct.
"""
from __future__ import annotations

from dataclasses import dataclass
import re
import subprocess
import time
from typing import Dict, Iterable


@dataclass(frozen=True)
class GazeboPose:
    name: str
    xyz: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]


def _field(text: str, name: str, default: float) -> float:
    match = re.search(rf'\b{name}:\s*([-\d.eE]+)', text)
    return float(match.group(1)) if match else default


def _normalize_quaternion(q: Iterable[float]) -> tuple[float, float, float, float]:
    values = tuple(float(value) for value in q)
    norm = sum(value * value for value in values) ** 0.5
    if norm <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in values)  # type: ignore[return-value]


def parse_pose_info(text: str) -> Dict[str, GazeboPose]:
    """Parse `gz topic -e .../pose/info` text output into model poses.

    proto3 text çıktısı SIFIR değerli alanları YAZMAZ: zeminde duran statik bir
    kutu için `position { x: -0.12 y: -0.19 }` gelir (z yok), origin'deki bir
    model için position bloğu TAMAMEN yok olabilir. Bu yüzden x/y/z tek tek
    opsiyoneldir (default 0.0) — eski "üçü de zorunlu" regex statik bin'leri
    kaçırıyordu (2026-07-05 kök neden).
    """
    poses: Dict[str, GazeboPose] = {}
    for block in re.split(r'\bpose\s*{', text):
        name_m = re.search(r'name:\s*"([^"]+)"', block)
        if not name_m:
            continue
        pos_block = re.search(r'position\s*{([^}]*)}', block, re.DOTALL)
        position = pos_block.group(1) if pos_block else ''
        orient_block = re.search(r'orientation\s*{([^}]*)}', block, re.DOTALL)
        orientation = orient_block.group(1) if orient_block else ''
        poses[name_m.group(1)] = GazeboPose(
            name=name_m.group(1),
            xyz=(
                _field(position, 'x', 0.0),
                _field(position, 'y', 0.0),
                _field(position, 'z', 0.0),
            ),
            quaternion=_normalize_quaternion((
                _field(orientation, 'x', 0.0),
                _field(orientation, 'y', 0.0),
                _field(orientation, 'z', 0.0),
                _field(orientation, 'w', 1.0),
            )),
        )
    return poses


def read_world_poses(
    world: str = 'pick_and_place_world',
    timeout_s: float = 10.0,
    dynamic: bool = False,
) -> Dict[str, GazeboPose]:
    suffix = 'dynamic_pose/info' if dynamic else 'pose/info'
    topic = f'/world/{world}/{suffix}'
    result = subprocess.run(
        ['gz', 'topic', '-e', '-t', topic, '-n', '1'],
        check=False,
        capture_output=True,
        text=True,
        timeout=timeout_s,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f'Unable to read Gazebo pose topic {topic}: {result.stderr.strip()}')
    poses = parse_pose_info(result.stdout)
    if not poses:
        raise RuntimeError(f'Gazebo pose topic {topic} returned no model poses')
    return poses


def read_dynamic_world_poses(
    world: str = 'pick_and_place_world',
    timeout_s: float = 10.0,
) -> Dict[str, GazeboPose]:
    return read_world_poses(world=world, timeout_s=timeout_s, dynamic=True)


def wait_for_model_pose(
    model_name: str,
    world: str = 'pick_and_place_world',
    attempts: int = 8,
    delay_s: float = 0.25,
    timeout_s: float = 5.0,
    dynamic: bool = False,
) -> GazeboPose:
    last_error: Exception | None = None
    for _ in range(max(attempts, 1)):
        try:
            pose = read_world_poses(
                world=world, timeout_s=timeout_s, dynamic=dynamic).get(model_name)
        except Exception as exc:  # Gazebo may still be starting.
            last_error = exc
            pose = None
        if pose is not None:
            return pose
        time.sleep(max(delay_s, 0.0))
    detail = f': {last_error}' if last_error else ''
    raise RuntimeError(f'Gazebo model pose not found: {model_name}{detail}')


def pose_inside_bin_xy(
    pose: GazeboPose,
    bin_center_xyz: tuple[float, float, float],
    inner_size_xyz: tuple[float, float, float],
    margin_m: float = 0.0,
) -> bool:
    half_x = inner_size_xyz[0] * 0.5 + margin_m
    half_y = inner_size_xyz[1] * 0.5 + margin_m
    epsilon = 1e-9
    return (
        abs(pose.xyz[0] - bin_center_xyz[0]) <= half_x + epsilon
        and abs(pose.xyz[1] - bin_center_xyz[1]) <= half_y + epsilon
    )
