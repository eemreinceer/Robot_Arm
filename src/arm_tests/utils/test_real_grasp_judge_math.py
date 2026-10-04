#!/usr/bin/env python3
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

# Adı "math" ama zincir saf değil: test_real_grasp_sorting -> arm_perception
# .sorting_config -> ament_index_python. ROS source edilmemişse bu bir collection
# error olup TÜM koşuyu iptal ediyordu; sebebi görünür bir skip'e çeviriyoruz.
pytest.importorskip(
    "ament_index_python",
    reason="arm_perception.sorting_config ROS ister; once ROS 2 source edin",
)

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "src" / "arm_tests" / "integration"))
sys.path.insert(0, str(REPO / "src" / "arm_perception"))

from test_real_grasp_sorting import _placement_candidate, XY, XYZ
from arm_perception.sorting_config import BinSpec, SortingConfig


def _config():
    return SortingConfig(
        base_world_xyz=(0.15, 0.0, 0.6),
        table_world_z=0.6,
        max_horizontal_reach_m=0.72,
        object_zone_world=((0.45, 0.47), (0.15, 0.18)),
        minimum_object_spacing_m=0.075,
        bins={
            "red_box": BinSpec(
                object_type="red_box", model_name="bin_red",
                spawn_world_xyz=(0.59, -0.31, 0.6),
                drop_link6_xyz=(0.0, 0.0, 0.0),
                drop_link6_quaternion=(0.0, 0.0, 0.0, 1.0),
                ik_seed=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                inner_size_xyz=(0.16, 0.16, 0.08),
            ),
            "blue_cube": BinSpec(
                object_type="blue_cube", model_name="bin_blue",
                spawn_world_xyz=(0.515, 0.45, 0.6),
                drop_link6_xyz=(0.0, 0.0, 0.0),
                drop_link6_quaternion=(0.0, 0.0, 0.0, 1.0),
                ik_seed=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                inner_size_xyz=(0.16, 0.16, 0.08),
            ),
        },
    )


def test_placement_candidate_requires_settled_height_band():
    cfg = _config()
    assert _placement_candidate("red_box", XYZ(0.59, -0.31, 0.82), cfg, 0.01, 0.55, 0.75) == (None, None)
    assert _placement_candidate("red_box", XYZ(0.59, -0.31, 0.63), cfg, 0.01, 0.55, 0.75) == ("correct", "red_box")


def test_placement_candidate_reports_wrong_bin_only_after_drop_band():
    cfg = _config()
    assert _placement_candidate("red_box", XYZ(0.515, 0.45, 0.90), cfg, 0.01, 0.55, 0.75) == (None, None)
    assert _placement_candidate("red_box", XYZ(0.515, 0.45, 0.62), cfg, 0.01, 0.55, 0.75) == ("wrong", "blue_cube")


def test_placement_candidate_marks_fling_after_object_is_low():
    cfg = _config()
    assert _placement_candidate("red_box", XYZ(1.1, 0.0, 0.90), cfg, 0.01, 0.55, 0.75) == (None, None)
    assert _placement_candidate("red_box", XYZ(1.1, 0.0, 0.62), cfg, 0.01, 0.55, 0.75) == ("fling", None)
