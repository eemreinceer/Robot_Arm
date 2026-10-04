import importlib.util
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location(
    'capture_calib_images', ROOT / 'scripts' / 'capture_calib_images.py')
CAPTURE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CAPTURE)


def test_collection_requires_target_count_and_all_nine_cells():
    all_cells = {(column, row) for column in range(3) for row in range(3)}
    assert not CAPTURE.collection_complete(24, 25, all_cells)
    assert not CAPTURE.collection_complete(25, 25, set(list(all_cells)[:-1]))
    assert CAPTURE.collection_complete(25, 25, all_cells)


def test_collection_may_exceed_target_while_filling_coverage():
    all_cells = {(column, row) for column in range(3) for row in range(3)}
    assert CAPTURE.collection_complete(31, 25, all_cells)


def test_coverage_uses_all_observed_corners_not_only_board_center():
    points = np.array([
        [10.0, 10.0], [320.0, 20.0], [639.0, 30.0],
        [15.0, 240.0], [320.0, 240.0], [630.0, 240.0],
        [20.0, 470.0], [320.0, 470.0], [620.0, 470.0],
    ])
    expected = {(column, row) for column in range(3) for row in range(3)}
    assert CAPTURE.cells_covered_by_points(points, 640, 480) == expected


def _synthetic_board(cols, rows, spacing, tilt_deg=0.0, origin=(100.0, 80.0)):
    """Build a projected board with a known spacing and foreshortening.

    Tilt is applied as a pure horizontal compression, which is what a rotation
    about the vertical axis does to a planar target under weak perspective.
    That keeps the fixture free of camera intrinsics, exactly like the code it
    exercises.
    """
    scale = np.cos(np.radians(tilt_deg))
    points = []
    for row in range(rows):
        for col in range(cols):
            points.append([
                origin[0] + col * spacing * scale,
                origin[1] + row * spacing,
            ])
    return np.array(points, dtype=np.float64)


def test_board_geometry_recovers_spacing_and_flags_fronto_parallel():
    points = _synthetic_board(6, 9, spacing=20.0)

    geometry = CAPTURE.board_geometry(points, (6, 9))

    assert geometry['spacing_px'] == 20.0
    assert geometry['foreshortening'] > 0.99
    assert geometry['tilt_deg'] < 5.0


def test_board_geometry_reports_tilt_when_the_board_is_foreshortened():
    """A compressed board must read as tilted, not as a smaller board."""
    points = _synthetic_board(6, 9, spacing=20.0, tilt_deg=60.0)

    geometry = CAPTURE.board_geometry(points, (6, 9))

    # Median spacing survives because only one axis is compressed.
    assert geometry['spacing_px'] == 20.0
    assert geometry['tilt_deg'] > 35.0


def test_closer_board_lands_in_a_different_distance_bin():
    """Scale is the only distance cue available before intrinsics exist."""
    far = CAPTURE.board_geometry(_synthetic_board(6, 9, 12.0), (6, 9))
    near = CAPTURE.board_geometry(_synthetic_board(6, 9, 40.0), (6, 9))

    far_bin = CAPTURE.bin_index(far['spacing_px'], CAPTURE.DISTANCE_BIN_EDGES_PX)
    near_bin = CAPTURE.bin_index(
        near['spacing_px'], CAPTURE.DISTANCE_BIN_EDGES_PX)

    assert far_bin == 0
    assert near_bin == len(CAPTURE.DISTANCE_BIN_EDGES_PX)


def test_full_coverage_without_diversity_does_not_complete_the_run():
    """The 2026-08-18 defect: nine cells filled, focal length still free.

    Every image cell is covered and the count is met, but every frame was shot
    from the same distance and fronto-parallel. That is precisely the set that
    left fx unconstrained by 6.6 percent, so the gate must refuse it.
    """
    all_cells = {(column, row) for column in range(3) for row in range(3)}

    assert not CAPTURE.collection_complete(
        25, 25, all_cells, distance_bins={1}, tilt_bins={0})


def test_run_completes_once_distance_and_strong_tilt_are_present():
    all_cells = {(column, row) for column in range(3) for row in range(3)}
    every_distance = set(range(len(CAPTURE.DISTANCE_BIN_EDGES_PX) + 1))

    assert CAPTURE.collection_complete(
        25, 25, all_cells, distance_bins=every_distance,
        tilt_bins={0, len(CAPTURE.TILT_BIN_EDGES_DEG)})


def test_strong_tilt_alone_is_not_enough_without_distance_spread():
    all_cells = {(column, row) for column in range(3) for row in range(3)}

    assert not CAPTURE.collection_complete(
        25, 25, all_cells, distance_bins={2},
        tilt_bins={0, len(CAPTURE.TILT_BIN_EDGES_DEG)})
