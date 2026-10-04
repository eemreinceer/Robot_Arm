"""Validated class-to-bin configuration for the sorting demo."""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Dict, Sequence

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose
import yaml


@dataclass(frozen=True)
class BinSpec:
    object_type: str
    model_name: str
    spawn_world_xyz: tuple[float, float, float]
    drop_link6_xyz: tuple[float, float, float]
    drop_link6_quaternion: tuple[float, float, float, float]
    ik_seed: tuple[float, float, float, float, float, float]
    inner_size_xyz: tuple[float, float, float]

    def place_pose(self) -> Pose:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = self.drop_link6_xyz
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = self.drop_link6_quaternion
        return pose

    def contains(self, point: Point, margin: float = 0.0) -> bool:
        half_x = self.inner_size_xyz[0] / 2.0 + margin
        half_y = self.inner_size_xyz[1] / 2.0 + margin
        return (
            abs(float(point.x) - self.spawn_world_xyz[0]) <= half_x
            and abs(float(point.y) - self.spawn_world_xyz[1]) <= half_y
        )


@dataclass(frozen=True)
class ModelPaths:
    """Nesne/kutu SDF köklerini config'e taşır — Robot Arm gibi farklı ölçekli
    sahneler eski robotun modellerini bozmadan kendi modellerini seçebilir."""
    objects_package: str = 'arm_gazebo'
    objects_dir: str = 'models'
    bins_package: str = 'arm_perception'
    bins_dir: str = 'models/sorting_bins'


DEFAULT_OBJECT_HALF_HEIGHTS: Dict[str, float] = {
    'red_box': 0.025,
    'yellow_cylinder': 0.025,
    'blue_cube': 0.020,
}


@dataclass(frozen=True)
class SortingConfig:
    bins: Dict[str, BinSpec]
    table_world_z: float
    base_world_xyz: tuple[float, float, float]
    max_horizontal_reach_m: float
    object_zone_world: tuple[tuple[float, float], tuple[float, float]]
    minimum_object_spacing_m: float
    models: ModelPaths = ModelPaths()
    object_half_heights: Dict[str, float] = None  # None → DEFAULT_OBJECT_HALF_HEIGHTS

    def half_height(self, object_type: str) -> float:
        table = self.object_half_heights or DEFAULT_OBJECT_HALF_HEIGHTS
        return float(table[object_type])

    @property
    def object_zone_base_link(self) -> tuple[tuple[float, float], tuple[float, float]]:
        # Backwards-compatible alias for older tests/callers; Gazebo spawning uses world frame.
        return self.object_zone_world


def default_sorting_config_path() -> Path:
    return Path(get_package_share_directory('arm_perception')) / 'config' / 'sorting_bins.yaml'


def _tuple(values: Sequence[float], size: int, field: str) -> tuple[float, ...]:
    if len(values) != size:
        raise ValueError(f'{field} must contain {size} values')
    return tuple(float(value) for value in values)


def load_sorting_config(path: str | Path | None = None) -> SortingConfig:
    config_path = Path(path).expanduser() if path else default_sorting_config_path()
    document = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    workspace = document['workspace']
    base_world_xyz = _tuple(workspace['base_world_xyz'], 3, 'workspace.base_world_xyz')
    max_reach = float(workspace['max_horizontal_reach_m'])
    table_world_z = float(workspace['table_world_z'])
    object_zone_key = 'object_zone_world' if 'object_zone_world' in workspace else 'object_zone_base_link'
    object_zone = workspace[object_zone_key]
    object_zone_world = (
        _tuple(object_zone['x'], 2, f'workspace.{object_zone_key}.x'),
        _tuple(object_zone['y'], 2, f'workspace.{object_zone_key}.y'),
    )
    bins: Dict[str, BinSpec] = {}
    for object_type, values in document['bins'].items():
        bin_spec = BinSpec(
            object_type=str(object_type),
            model_name=str(values['model_name']),
            spawn_world_xyz=_tuple(values['spawn_world_xyz'], 3, f'{object_type}.spawn_world_xyz'),
            drop_link6_xyz=_tuple(values['drop_link6_xyz'], 3, f'{object_type}.drop_link6_xyz'),
            drop_link6_quaternion=_tuple(
                values['drop_link6_quaternion'], 4, f'{object_type}.drop_link6_quaternion'),
            ik_seed=_tuple(values['ik_seed'], 6, f'{object_type}.ik_seed'),
            inner_size_xyz=_tuple(values['inner_size_xyz'], 3, f'{object_type}.inner_size_xyz'),
        )
        radial_distance = math.hypot(
            bin_spec.spawn_world_xyz[0] - base_world_xyz[0],
            bin_spec.spawn_world_xyz[1] - base_world_xyz[1],
        )
        if radial_distance > max_reach:
            raise ValueError(
                f'{object_type} bin radial distance {radial_distance:.3f}m exceeds '
                f'{max_reach:.3f}m workspace limit')
        bins[object_type] = bin_spec
    expected = {'red_box', 'yellow_cylinder', 'blue_cube'}
    if set(bins) != expected:
        raise ValueError(f'bins must define exactly {sorted(expected)}')
    models_doc = document.get('models', {})
    models = ModelPaths(
        objects_package=str(models_doc.get('objects_package', 'arm_gazebo')),
        objects_dir=str(models_doc.get('objects_dir', 'models')),
        bins_package=str(models_doc.get('bins_package', 'arm_perception')),
        bins_dir=str(models_doc.get('bins_dir', 'models/sorting_bins')),
    )
    half_heights = None
    if 'objects' in document:
        half_heights = {
            name: float(values['half_height'])
            for name, values in document['objects'].items()
        }
        if set(half_heights) != expected:
            raise ValueError(f'objects must define exactly {sorted(expected)}')
    return SortingConfig(
        bins=bins,
        table_world_z=table_world_z,
        base_world_xyz=base_world_xyz,
        max_horizontal_reach_m=max_reach,
        object_zone_world=object_zone_world,
        minimum_object_spacing_m=float(workspace['minimum_object_spacing_m']),
        models=models,
        object_half_heights=half_heights,
    )
