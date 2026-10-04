import importlib.util
import json
from pathlib import Path

import cv2

import numpy as np

import pytest


ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location(
    'solve_camera_intrinsics', ROOT / 'scripts' / 'solve_camera_intrinsics.py')
SOLVER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SOLVER)


def metadata(**overrides):
    data = {
        'pattern_cols': 6,
        'pattern_rows': 8,
        'square_size_mm': 25.0,
        'flip_method': 2,
        'image_width': 640,
        'image_height': 480,
    }
    data.update(overrides)
    return data


def test_point_rms_is_true_euclidean_rms_not_norm_divided_by_count():
    observed = np.array([[0.0, 0.0], [3.0, 4.0]])
    projected = np.zeros((2, 2))
    assert SOLVER.point_rms(observed, projected) == pytest.approx(
        np.sqrt(12.5))


@pytest.mark.parametrize(
    'policy, expected_centre',
    [
        ('image_center', (320.0, 240.0)),
        ('pixel_grid_center', (319.5, 239.5)),
    ],
)
def test_fixed_principal_point_policies_are_explicit(policy, expected_centre):
    flags, camera_matrix, distortion = SOLVER.calibration_inputs(
        (640, 480), policy)

    assert flags & cv2.CALIB_USE_INTRINSIC_GUESS
    assert flags & cv2.CALIB_FIX_PRINCIPAL_POINT
    assert tuple(camera_matrix[:2, 2]) == expected_centre
    assert distortion.shape == (1, 5)


def test_free_principal_point_has_no_hidden_initial_guess():
    flags, camera_matrix, distortion = SOLVER.calibration_inputs(
        (640, 480), 'free')

    assert flags == 0
    assert camera_matrix is None
    assert distortion is None


def test_unknown_principal_point_policy_fails_closed():
    with pytest.raises(ValueError, match='policy gecersiz'):
        SOLVER.calibration_inputs((640, 480), 'automatic')


def test_solver_yaml_records_principal_point_policy():
    source = (ROOT / 'scripts' / 'solve_camera_intrinsics.py').read_text()
    assert 'principal_point_policy: {args.principal_point_policy}' in source


def test_capture_metadata_requires_flip_and_square_size(tmp_path):
    (tmp_path / 'capture_meta.json').write_text(json.dumps({
        'pattern_cols': 6,
        'pattern_rows': 8,
        'image_width': 640,
        'image_height': 480,
    }))
    with pytest.raises(ValueError, match='square_size_mm, flip_method'):
        SOLVER.load_capture_metadata(str(tmp_path))


@pytest.mark.parametrize('bad_flip', [True, 2.5, '2'])
def test_capture_metadata_rejects_non_integer_flip(tmp_path, bad_flip):
    (tmp_path / 'capture_meta.json').write_text(json.dumps(
        metadata(flip_method=bad_flip)))
    with pytest.raises(ValueError, match='flip_method tam sayi'):
        SOLVER.load_capture_metadata(str(tmp_path))


@pytest.mark.parametrize(
    'overrides, field',
    [
        ({'flip_method': 0}, 'flip_method'),
        ({'image_width': 480, 'image_height': 640}, 'image_width'),
        ({'pattern_cols': 9}, 'pattern_cols'),
        ({'square_size_mm': 24.9}, 'square_size_mm'),
    ],
)
def test_fit_and_validation_contract_must_match(overrides, field):
    with pytest.raises(ValueError, match=field):
        SOLVER.validate_dataset_contract(metadata(), metadata(**overrides))


def test_independent_validation_reprojects_fixed_intrinsics():
    object_points = SOLVER.checkerboard_object_points(6, 8, 25.0)
    camera_matrix = np.array([
        [520.0, 0.0, 319.5],
        [0.0, 520.0, 239.5],
        [0.0, 0.0, 1.0],
    ])
    distortion = np.zeros((5, 1))
    rvec = np.array([[0.12], [-0.18], [0.04]])
    tvec = np.array([[0.01], [-0.02], [0.75]])
    observed, _ = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, distortion)

    errors = SOLVER.independent_validation_errors(
        object_points, [observed], camera_matrix, distortion)
    assert errors == pytest.approx([0.0], abs=1e-4)


def test_calibration_corner_detection_uses_sector_based_detector(monkeypatch):
    expected = np.zeros((48, 1, 2), dtype=np.float32)
    observed = {}

    def fake_sb(gray, pattern, flags):
        observed['pattern'] = pattern
        observed['flags'] = flags
        return True, expected

    monkeypatch.setattr(cv2, 'findChessboardCornersSB', fake_sb)
    monkeypatch.setattr(
        cv2, 'findChessboardCorners',
        lambda *args, **kwargs: pytest.fail(
            'legacy detector must not be used'))

    found, corners = SOLVER.detect_checkerboard_corners(
        np.zeros((480, 640), dtype=np.uint8), 6, 8)

    assert found
    assert corners is expected
    assert observed['pattern'] == (6, 8)
    assert observed['flags'] & cv2.CALIB_CB_ACCURACY
    assert observed['flags'] & cv2.CALIB_CB_EXHAUSTIVE
