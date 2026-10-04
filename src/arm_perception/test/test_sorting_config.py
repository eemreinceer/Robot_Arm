import math
from pathlib import Path

from geometry_msgs.msg import Point
import numpy as np
import yaml

from arm_perception.math3d import matrix_from_quaternion
from arm_perception.sorting_config import load_sorting_config


CONFIG = Path(__file__).parents[1] / 'config' / 'sorting_bins.yaml'
SRC_ROOT = Path(__file__).parents[2]


def test_sorting_config_has_three_reachable_bins():
    config = load_sorting_config(CONFIG)
    assert set(config.bins) == {'red_box', 'yellow_cylinder', 'blue_cube'}
    centers = []
    for bin_spec in config.bins.values():
        x, y, _z = bin_spec.spawn_world_xyz
        radius = math.hypot(
            x - config.base_world_xyz[0],
            y - config.base_world_xyz[1],
        )
        assert radius <= config.max_horizontal_reach_m
        assert 0.0 <= x <= 0.90
        assert -0.55 <= y <= 0.55
        assert len(bin_spec.ik_seed) == 6
        assert len(bin_spec.drop_link6_quaternion) == 4
        centers.append((x, y, bin_spec.inner_size_xyz))
    for index, (x1, y1, size1) in enumerate(centers):
        for x2, y2, size2 in centers[index + 1:]:
            half_diag_1 = math.hypot(size1[0], size1[1]) * 0.5
            half_diag_2 = math.hypot(size2[0], size2[1]) * 0.5
            assert math.hypot(x1 - x2, y1 - y2) >= half_diag_1 + half_diag_2
    x_range, y_range = config.object_zone_world
    assert 0.40 <= x_range[0] < x_range[1] <= 0.95
    assert -0.55 <= y_range[0] < y_range[1] <= 0.55
    for x in x_range:
        for y in y_range:
            assert math.hypot(
                x - config.base_world_xyz[0],
                y - config.base_world_xyz[1],
            ) <= config.max_horizontal_reach_m


def test_bin_contains_filters_sorted_objects_in_xy():
    config = load_sorting_config(CONFIG)
    red = config.bins['red_box']
    inside = Point(x=red.spawn_world_xyz[0], y=red.spawn_world_xyz[1], z=0.04)
    outside = Point(x=0.62, y=0.00, z=0.04)
    assert red.contains(inside)
    assert not red.contains(outside)


def test_drop_link6_places_grasp_link_over_bin_center_low():
    config = load_sorting_config(CONFIG)
    link6_to_grasp = np.array([0.0075, -0.0032, 0.1023])
    for object_type, bin_spec in config.bins.items():
        link6_base = np.array(bin_spec.drop_link6_xyz)
        grasp_base = (
            link6_base
            + matrix_from_quaternion(bin_spec.drop_link6_quaternion) @ link6_to_grasp
        )
        # world_joint yaw=+90°: world_x = base_world_x - base_y,
        #                        world_y = base_world_y + base_x
        grasp_world = np.array([
            config.base_world_xyz[0] - grasp_base[1],
            config.base_world_xyz[1] + grasp_base[0],
            grasp_base[2] + config.base_world_xyz[2],
        ])
        half_height = 0.020 if object_type == 'blue_cube' else 0.025
        expected_z = bin_spec.spawn_world_xyz[2] + 0.015 + half_height + 0.005
        assert abs(grasp_world[0] - bin_spec.spawn_world_xyz[0]) < 0.010
        assert abs(grasp_world[1] - bin_spec.spawn_world_xyz[1]) < 0.010
        assert abs(grasp_world[2] - expected_z) < 0.006


def test_motion_safety_config_does_not_mask_large_state_drift():
    ompl_path = SRC_ROOT / 'arm_moveit_config' / 'config' / 'ompl_planning.yaml'
    launch_path = SRC_ROOT / 'arm_moveit_config' / 'launch' / 'move_group.launch.py'
    ompl_config = yaml.safe_load(ompl_path.read_text(encoding='utf-8'))
    launch_text = launch_path.read_text(encoding='utf-8')

    assert ompl_config['ompl']['start_state_max_bounds_error'] <= 0.25
    assert "get_package_prefix('arm_nodes')" in launch_text
    assert "'trajectory_execution.allowed_start_tolerance': 0.3" in launch_text
