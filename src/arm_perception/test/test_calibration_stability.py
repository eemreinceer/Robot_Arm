"""Regression tests for the offline intrinsics stability analyzer."""

import importlib.util
import json
from pathlib import Path

import numpy as np

import pytest


ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location(
    'calibration_stability', ROOT / 'scripts' / 'calibration_stability.py')
STABILITY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(STABILITY)


def test_bootstrap_draws_with_replacement_and_is_deterministic():
    first = STABILITY.draw_indices(10, 5, 10, 'bootstrap', 7)
    second = STABILITY.draw_indices(10, 5, 10, 'bootstrap', 7)

    assert all(np.array_equal(a, b) for a, b in zip(first, second))
    assert any(len(np.unique(draw)) < len(draw) for draw in first)


def test_subsample_draws_without_replacement():
    draws = STABILITY.draw_indices(10, 4, 8, 'subsample', 7)

    assert all(len(np.unique(draw)) == len(draw) for draw in draws)


def test_subsample_rejects_more_frames_than_available():
    with pytest.raises(ValueError, match='asamaz'):
        STABILITY.draw_indices(10, 2, 11, 'subsample', 7)


def test_summary_keeps_matching_quantiles_separate():
    observed = STABILITY.summary([1.0, 2.0, 3.0, 10.0])

    assert observed['median'] == pytest.approx(2.5)
    assert observed['p90'] == pytest.approx(7.9)
    assert observed['max'] == pytest.approx(10.0)


def test_rotation_gap_is_zero_for_same_rotation():
    assert STABILITY.rotation_gap_deg(
        np.eye(3), np.eye(3)) == pytest.approx(0.0)


def test_atomic_write_json_leaves_complete_checkpoint_and_no_temporary(
        tmp_path):
    target = tmp_path / 'checkpoint.json'

    STABILITY.atomic_write_json(target, {'status': 'running', 'draws': 3})

    assert json.loads(target.read_text()) == {
        'status': 'running', 'draws': 3}
    assert not (tmp_path / 'checkpoint.json.tmp').exists()


def test_resume_rejects_changed_inputs():
    checkpoint = {
        'schema_version': 3,
        'run_fingerprint': 'old',
        'policy_draws': {},
    }

    with pytest.raises(ValueError, match='girdileri'):
        STABILITY.validate_resume(
            checkpoint, 'new', ['free'], draw_count=10)


def test_resume_rejects_more_completed_draws_than_requested():
    checkpoint = {
        'schema_version': 3,
        'run_fingerprint': 'same',
        'policy_draws': {'free': [{}, {}, {}]},
    }

    with pytest.raises(ValueError, match='draw sayisi'):
        STABILITY.validate_resume(
            checkpoint, 'same', ['free'], draw_count=2)


def test_run_fingerprint_is_deterministic_and_input_sensitive():
    first = STABILITY.run_fingerprint({'seed': 3, 'policies': ['free']})
    reordered = STABILITY.run_fingerprint(
        {'policies': ['free'], 'seed': 3})
    changed = STABILITY.run_fingerprint({'seed': 4, 'policies': ['free']})

    assert first == reordered
    assert first != changed


def test_analyze_policy_resumes_after_last_completed_draw(monkeypatch):
    calibrations = []

    def fake_calibrate(*args):
        calibrations.append(args)
        return 0.1, np.eye(3), np.zeros((1, 5)), [], []

    monkeypatch.setattr(
        STABILITY.solver, 'calibrate_intrinsics', fake_calibrate)
    monkeypatch.setattr(
        STABILITY.solver, 'independent_validation_errors',
        lambda *args: [0.2])
    monkeypatch.setattr(
        STABILITY, 'solve_rotations', lambda *args: [np.eye(3)])
    checkpoints = []
    completed = [{
        'fit_rms_px': 0.1,
        'intrinsics': [1.0, 1.0, 0.0, 0.0],
        'validation_rotation_gaps_deg': [0.0],
    }]

    result = STABILITY.analyze_policy(
        'free', np.zeros((5, 3)), [np.zeros((5, 2))] * 5,
        [np.zeros((5, 2))], (640, 480),
        [np.arange(5), np.arange(5)], [np.eye(3)], None,
        completed_draws=completed,
        checkpoint_callback=lambda draws: checkpoints.append(list(draws)))

    assert len(calibrations) == 2  # one full fit and one missing draw
    assert len(result['draw_results']) == 2
    assert [len(draws) for draws in checkpoints] == [2]


def _rotation_about_z(degrees):
    """Return a rotation matrix about Z, used to fake a policy-induced shift."""
    angle = np.radians(degrees)
    return np.array([
        [np.cos(angle), -np.sin(angle), 0.0],
        [np.sin(angle), np.cos(angle), 0.0],
        [0.0, 0.0, 1.0],
    ])


def _fake_policy_run(monkeypatch, policy, reference_rotations, shifted_by_deg):
    """Drive analyze_policy with injected rotations; no images involved."""
    monkeypatch.setattr(
        STABILITY.solver, 'calibrate_intrinsics',
        lambda *args: (0.1, np.eye(3), np.zeros((1, 5)), [], []))
    monkeypatch.setattr(
        STABILITY.solver, 'independent_validation_errors',
        lambda *args: [0.2])
    monkeypatch.setattr(
        STABILITY, 'solve_rotations',
        lambda *args: [_rotation_about_z(shifted_by_deg)])
    return STABILITY.analyze_policy(
        policy, np.zeros((5, 3)), [np.zeros((5, 2))] * 5,
        [np.zeros((5, 2))], (640, 480),
        [np.arange(5)], reference_rotations, None)


def test_free_reference_reports_zero_policy_shift(monkeypatch):
    """`free` is the reference, so its shift from itself must be exactly zero.

    Guards the bias half of the report: if the reference ever picks up a
    non-zero shift, every fixed policy's bias is measured against a moving
    baseline and the variance/bias split becomes meaningless.
    """
    result = _fake_policy_run(
        monkeypatch, 'free', [_rotation_about_z(0.0)], shifted_by_deg=0.0)

    assert result['pose_shift_from_free_deg']['median'] == 0.0
    assert result['pose_shift_from_free_deg']['max'] == 0.0


def test_fixed_policy_shift_is_measured_against_the_free_reference(monkeypatch):
    """A fixed policy that moves the pose must report that move as bias.

    This is the defect #16 was opened for: fixing the principal point lowers
    the resampling spread, and a report that only showed spread would call
    that an improvement while hiding the systematic shift it paid for.
    """
    result = _fake_policy_run(
        monkeypatch, 'image_center', [_rotation_about_z(0.0)],
        shifted_by_deg=0.75)

    assert result['pose_shift_from_free_deg']['median'] == pytest.approx(
        0.75, abs=1e-6)
    # Uncertainty is the spread across draws, not the shift: with one draw
    # that reproduces the policy's own reference it must stay zero.
    assert result['uncertainty_deg']['max'] == pytest.approx(0.0, abs=1e-9)


def test_uncertainty_and_shift_are_separate_keys(monkeypatch):
    """Variance and bias must never be collapsed into one number."""
    result = _fake_policy_run(
        monkeypatch, 'image_center', [_rotation_about_z(0.0)],
        shifted_by_deg=0.5)

    assert 'uncertainty_deg' in result
    assert 'pose_shift_from_free_deg' in result
    assert set(result['uncertainty_deg']) == set(
        result['pose_shift_from_free_deg'])


def _fake_results(median, p90):
    return {'a_policy': {'uncertainty_deg': {'median': median, 'p90': p90}}}


def test_sigma_gate_fails_when_spread_exceeds_declared_sigma():
    """The gate must fail on the shape of the real measurement.

    Measured on the Pi 5 set: uncertainty median 0.2749 deg against a declared
    perception sigma of 0.0546 deg. Reprojection cannot see this — the whole
    point of the gate is that it can.
    """
    gate = STABILITY.sigma_gate(_fake_results(0.2749, 0.6364), 0.0546)

    assert gate['per_policy']['a_policy']['passes'] is False
    assert gate['passes'] is False
    assert gate['per_policy']['a_policy']['median_ratio'] == pytest.approx(
        0.2749 / 0.0546)


def test_sigma_gate_requires_both_quantiles_inside_sigma():
    """A good median must not rescue a bad tail.

    A scalar sigma has no quantiles of its own, so median and p90 are both
    compared against it; passing on the median alone would let a calibration
    that is usually fine but occasionally 3x off be declared acceptable.
    """
    gate = STABILITY.sigma_gate(_fake_results(0.04, 0.30), 0.0546)

    assert gate['per_policy']['a_policy']['passes'] is False


def test_sigma_gate_passes_only_when_both_quantiles_fit():
    gate = STABILITY.sigma_gate(_fake_results(0.02, 0.05), 0.0546)

    assert gate['per_policy']['a_policy']['passes'] is True
    assert gate['passes'] is True


def test_sigma_gate_rejects_non_positive_sigma():
    with pytest.raises(ValueError):
        STABILITY.sigma_gate(_fake_results(0.1, 0.2), 0.0)
