#!/usr/bin/env python3
"""Unit tests for the hand-eye session tool's safety gates.

These are the checks that stand between a mistyped flag and a real arm, so
they are pinned here rather than left as ad-hoc terminal runs.

The module under test imports ROS lazily, so these tests run without a ROS
environment. Nothing here initialises rclpy or publishes anything.
"""
import importlib.util
import json
import math
import os
import signal

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCRIPT = os.path.normpath(
    os.path.join(_HERE, '..', '..', '..', 'scripts', 'hand_eye_session.py'))


def _load_module():
    spec = importlib.util.spec_from_file_location('hand_eye_session', _SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


hes = _load_module()


# -- pure math -------------------------------------------------------------

def _quat_about_x(degrees):
    half = math.radians(degrees) / 2.0
    return [math.sin(half), 0.0, 0.0, math.cos(half)]


def test_quat_angle_identity_and_known_angles():
    identity = [0.0, 0.0, 0.0, 1.0]
    assert hes.quat_angle_rad(identity, identity) == pytest.approx(0.0)
    for degrees in (10.0, 45.0, 90.0, 179.0):
        measured = math.degrees(
            hes.quat_angle_rad(identity, _quat_about_x(degrees)))
        assert measured == pytest.approx(degrees, abs=1e-6)


def test_quat_angle_handles_double_cover():
    """q and -q are the same rotation; without abs() this reads as 180 deg."""
    identity = [0.0, 0.0, 0.0, 1.0]
    negated = [0.0, 0.0, 0.0, -1.0]
    assert math.degrees(hes.quat_angle_rad(identity, negated)) == pytest.approx(0.0)


def test_vector_distance():
    assert hes.vector_distance([0, 0, 0], [0.003, 0.004, 0.0]) == pytest.approx(0.005)


def test_summarize_observations_medoid_and_spread():
    identity = [0.0, 0.0, 0.0, 1.0]
    observations = [
        {'xyz': [0.5000, 0.0, 0.0], 'quat_xyzw': identity},
        {'xyz': [0.5002, 0.0, 0.0], 'quat_xyzw': [0.0, 0.0, 0.0, -1.0]},
        {'xyz': [0.4998, 0.0, 0.0], 'quat_xyzw': identity},
    ]
    summary = hes.summarize_observations(observations)
    assert summary['xyz'][0] == pytest.approx(0.5)
    assert summary['sample_count'] == 3
    assert summary['internal_spread_mm'] == pytest.approx(0.2, abs=1e-6)
    # The sign-flipped quaternion is the same rotation, so spread stays zero.
    assert summary['internal_spread_deg'] == pytest.approx(0.0, abs=1e-9)


# -- is_finite_number ------------------------------------------------------

@pytest.mark.parametrize('value', [0.0, -1.5, 3, 1e-9])
def test_is_finite_number_accepts_real_numbers(value):
    assert hes.is_finite_number(value)


@pytest.mark.parametrize('value', [
    float('nan'), float('inf'), float('-inf'), None, '0.1', [1], True, False,
])
def test_is_finite_number_rejects_the_rest(value):
    """bool must be rejected: it subclasses int, so True would be 1.0 rad."""
    assert not hes.is_finite_number(value)


# -- clamp_pose: the last line before the servos ---------------------------

def test_clamp_pose_clamps_within_envelope():
    assert hes.clamp_pose([0.0, 0.5, -0.5, 0.1, -0.1], 0.30) == [
        0.0, 0.30, -0.30, 0.1, -0.1]


def test_clamp_pose_rejects_negative_envelope():
    """The 2026-07-31 defect: a negative envelope made clamp return +envelope
    for every input, so a commanded zero pose became a full-scale move on all
    five joints."""
    with pytest.raises(hes.UnsafeCommand):
        hes.clamp_pose([0.0] * 5, -0.30)


def test_clamp_pose_rejects_zero_and_nonfinite_envelope():
    for bad in (0.0, float('nan'), float('inf'), None):
        with pytest.raises(hes.UnsafeCommand):
            hes.clamp_pose([0.0] * 5, bad)


def test_clamp_pose_rejects_envelope_over_ceiling():
    with pytest.raises(hes.UnsafeCommand):
        hes.clamp_pose([0.0] * 5, hes.ENVELOPE_CEILING_RAD + 0.01)


def test_clamp_pose_rejects_nonfinite_joint():
    for bad in (float('nan'), float('inf'), None):
        with pytest.raises(hes.UnsafeCommand):
            hes.clamp_pose([0.0, 0.0, 0.0, 0.0, bad], 0.30)


# -- span split: unverified data must never reach the solver ---------------

def _samples(spans):
    return [{'span': span, 'id': index} for index, span in enumerate(spans)]


def test_split_clean_run_verifies_everything():
    verified, suspect = hes.split_samples(_samples([0, 0, 1, 1, 2]), 3)
    assert [s['id'] for s in verified] == [0, 1, 2, 3, 4]
    assert suspect == []


def test_split_first_check_failed_verifies_nothing():
    verified, suspect = hes.split_samples(_samples([0, 0]), 0)
    assert verified == []
    assert [s['id'] for s in suspect] == [0, 1]


def test_split_second_check_failed_keeps_only_closed_span():
    verified, suspect = hes.split_samples(_samples([0, 0, 1, 1]), 1)
    assert [s['id'] for s in verified] == [0, 1]
    assert [s['id'] for s in suspect] == [2, 3]


def test_split_interrupt_leaves_open_span_suspect():
    verified, suspect = hes.split_samples(_samples([0, 0, 1]), 1)
    assert [s['id'] for s in verified] == [0, 1]
    assert [s['id'] for s in suspect] == [2]


# -- argument validation ---------------------------------------------------

def _parse(argv):
    """Run the real parse + validate path, returning the parsed args."""
    parser = hes.build_parser()
    args = parser.parse_args(argv)
    poses = hes.DEFAULT_POSES
    if args.poses_json:
        with open(args.poses_json) as handle:
            poses = json.load(handle)
    hes.validate_args(parser, args, poses)
    return parser, args, poses


GOOD_GATE = ['--board-drift-mm', '4', '--board-drift-deg', '1']


def test_valid_arguments_pass():
    _parse(GOOD_GATE + ['--dry-run'])


@pytest.mark.parametrize('argv', [
    ['--max-abs-rad', '-0.30'],           # made zero commands full-scale moves
    ['--max-abs-rad', '0'],
    ['--max-abs-rad', 'nan'],
    ['--max-abs-rad', '5.0'],             # over the ceiling
    ['--check-every', '0'],               # modulo-by-zero mid-session
    ['--check-every', '-1'],
    ['--zero-samples', '0'],
    ['--samples-per-pose', '0'],
    ['--measure-return', '0'],
    ['--move-seconds', '0.01'],           # a slam, not a calibration move
    ['--move-seconds', 'inf'],
    ['--settle-seconds', '-1'],
    ['--max-age-s', '0'],
    ['--square-size-mm', '0'],
    ['--square-size-mm', '-1'],
    ['--board-cols', '1'],
    ['--board-rows', '0'],
    ['--max-reproj-px', '-1'],
    ['--board-drift-mm', '0', '--board-drift-deg', '1'],
    ['--zero-approach', '[0,0,0,0,NaN]'],
    ['--zero-approach', '[0,0,0,0,Infinity]'],
    ['--zero-approach', '[0,0,0]'],
    ['--zero-approach', '[0,0,0,0,true]'],  # bool is not a joint angle
])
def test_bad_scalars_and_poses_are_rejected(argv):
    with pytest.raises(SystemExit) as excinfo:
        _parse(GOOD_GATE + argv + ['--dry-run'])
    assert excinfo.value.code != 0


def test_malformed_pose_file_is_rejected(tmp_path):
    path = tmp_path / 'poses.json'
    path.write_text(json.dumps([[0.1, 0.2, 0.3, 0.4, 0.5], [0.1, 0.2]]))
    with pytest.raises(SystemExit):
        _parse(GOOD_GATE + ['--poses-json', str(path), '--dry-run'])


def test_nonfinite_pose_file_is_rejected(tmp_path):
    path = tmp_path / 'poses.json'
    path.write_text('[[0.1, 0.2, 0.3, 0.4, NaN]]')
    with pytest.raises(SystemExit):
        _parse(GOOD_GATE + ['--poses-json', str(path), '--dry-run'])


# -- gate provenance: no unmeasured threshold may start a session ----------

def _resolve(argv):
    parser = hes.build_parser()
    args = parser.parse_args(argv)
    hes.validate_args(parser, args, hes.DEFAULT_POSES)
    return hes.resolve_gate(parser, args)


def test_session_refuses_to_run_without_a_gate():
    """The whole point: an unmeasured placeholder must not certify a dataset."""
    with pytest.raises(SystemExit) as excinfo:
        _resolve([])
    assert excinfo.value.code != 0


def test_half_a_gate_is_rejected():
    for argv in (['--board-drift-mm', '4'], ['--board-drift-deg', '1']):
        with pytest.raises(SystemExit):
            _resolve(argv)


def test_explicit_gate_records_its_provenance():
    gate_mm, gate_deg, provenance = _resolve(GOOD_GATE)
    assert (gate_mm, gate_deg) == (4.0, 1.0)
    assert provenance['source'] == 'operator-supplied'


def _artifact(**overrides):
    """A realistic artifact, built by the real writer.

    Hand-rolling the dict here would let the test drift from the producer;
    building it through build_return_artifact keeps writer and reader paired.
    """
    args = _calibration_args(4)
    payload = hes.build_return_artifact(
        args, _cycles([(2.0, 0.20), (3.0, 0.30), (1.0, 0.10), (2.5, 0.25)]),
        list(args.zero_approach), _REFERENCE, complete=True)
    payload.update(overrides)
    return payload


def _write(tmp_path, payload):
    path = tmp_path / 'artifact.json'
    path.write_text(json.dumps(payload))
    return str(path)


def test_gate_from_artifact_is_used(tmp_path):
    path = _write(tmp_path, _artifact())
    gate_mm, gate_deg, provenance = _resolve(['--gate-from', path])
    # 1.5 x the worst measured cycle: 3.0 mm and 0.30 deg.
    assert (gate_mm, gate_deg) == pytest.approx((4.5, 0.45))
    assert provenance['source'] == 'artifact'
    assert provenance['artifact_cycles_measured'] == 4


@pytest.mark.parametrize('overrides', [
    {'kind': 'something_else'},
    {'version': 99},
    {'cycles_measured': hes.MIN_RETURN_CYCLES - 1},   # anecdote, not a floor
    {'suggested_gate_deg': 0},
    {'suggested_gate_mm': float('nan')},
    # Context that makes the number meaningful. Absent is not "default".
    {'units': None},
    {'units': {'translation': 'm', 'rotation': 'rad'}},
    {'units': {'translation': 'mm', 'rotation': 'deg'}},
    {'units': {'translation': 'mm', 'rotation': 'deg', 'joints': 'deg'}},
    {'frames': None},
    {'arm_joints': ['joint_1', 'joint_2']},
    {'probe_pose_rad': [0.1, 0.2]},
    {'probe_pose_rad': None},
    {'cycles': None},
    {'cycles': []},
    # v4 raw poses are the only thing the residuals can be re-derived from.
    {'zero_reference': None},
    {'zero_reference': {'xyz': [0.5, 0.0, 0.4]}},
    {'zero_reference': {'xyz': [0.5, 0.0], 'quat_xyzw': [0, 0, 0, 1]}},
    {'zero_reference': {'xyz': [0.5, 0.0, 0.4],
                        'quat_xyzw': [0.0, 0.0, 0.0, 0.0]}},
])
def test_unusable_artifact_is_refused_not_defaulted(tmp_path, overrides):
    path = _write(tmp_path, _artifact(**overrides))
    with pytest.raises(SystemExit):
        _resolve(['--gate-from', path])


def test_minimal_forged_artifact_cannot_set_a_huge_gate(tmp_path):
    """A hand-written file with only a headline number must not be believed.

    Before hardening this returned a 9999 mm gate, which would have disabled
    the board-moved check entirely while looking configured.
    """
    forged = {'kind': hes.RETURN_ARTIFACT_KIND, 'version': 1,
              'cycles_measured': 99,
              'suggested_gate_mm': 9999.0, 'suggested_gate_deg': 9999.0}
    path = _write(tmp_path, forged)
    with pytest.raises(ValueError):
        hes.load_gate_artifact(path)


def test_tampered_summary_that_contradicts_raw_cycles_is_rejected(tmp_path):
    """Widening suggested_gate_mm by hand must not widen the gate."""
    path = _write(tmp_path, _artifact(suggested_gate_mm=500.0))
    with pytest.raises(ValueError) as excinfo:
        hes.load_gate_artifact(path)
    assert 'inconsistent' in str(excinfo.value)


def test_gate_is_recomputed_from_raw_cycles_not_read_off(tmp_path):
    payload = _artifact()
    del payload['suggested_gate_mm']
    del payload['suggested_gate_deg']
    path = _write(tmp_path, payload)
    gate_mm, gate_deg, _summary = hes.load_gate_artifact(path)
    assert gate_mm == pytest.approx(4.5)   # 1.5 x worst measured 3.0 mm
    assert gate_deg == pytest.approx(0.45)


def test_cycle_count_that_disagrees_with_the_data_is_rejected(tmp_path):
    path = _write(tmp_path, _artifact(cycles_measured=40))
    with pytest.raises(ValueError):
        hes.load_gate_artifact(path)


def test_v1_artifact_is_refused(tmp_path):
    """v1 could not express whether the probe was actually executed."""
    path = _write(tmp_path, _artifact(version=1))
    with pytest.raises(ValueError) as excinfo:
        hes.load_gate_artifact(path)
    assert 'version' in str(excinfo.value)


def test_artifact_without_executed_probe_is_refused(tmp_path):
    """Regression 1: a v1-shaped file whose probe was never executed.

    probe_pose_rad=0.9 under max_abs_rad=0.3 describes a motion that the
    envelope would have clamped to 0.3 -- a calibration that did not happen.
    """
    payload = _artifact()
    del payload['executed_probe_pose_rad']
    payload['probe_pose_rad'] = [0.9, 0, 0, 0, 0]
    payload['max_abs_rad'] = 0.3
    path = _write(tmp_path, payload)
    with pytest.raises(ValueError) as excinfo:
        hes.load_gate_artifact(path)
    assert 'executed_probe_pose_rad' in str(excinfo.value)


def test_executed_probe_outside_its_own_envelope_is_refused(tmp_path):
    payload = _artifact()
    payload['probe_pose_rad'] = [0.9, 0, 0, 0, 0]
    payload['executed_probe_pose_rad'] = [0.9, 0, 0, 0, 0]
    payload['max_abs_rad'] = 0.3
    path = _write(tmp_path, payload)
    with pytest.raises(ValueError) as excinfo:
        hes.load_gate_artifact(path)
    assert 'envelope' in str(excinfo.value)


def test_requested_and_executed_probe_must_agree(tmp_path):
    payload = _artifact()
    payload['probe_pose_rad'] = [0.9, 0, 0, 0, 0]
    payload['executed_probe_pose_rad'] = [0.3, 0, 0, 0, 0]
    payload['max_abs_rad'] = 0.3
    path = _write(tmp_path, payload)
    with pytest.raises(ValueError) as excinfo:
        hes.load_gate_artifact(path)
    assert 'did not perform the motion it records' in str(excinfo.value)


@pytest.mark.parametrize('envelope', [None, 0, -0.3, float('nan'),
                                      hes.ENVELOPE_CEILING_RAD + 0.1])
def test_artifact_envelope_is_validated(tmp_path, envelope):
    path = _write(tmp_path, _artifact(max_abs_rad=envelope))
    with pytest.raises(ValueError):
        hes.load_gate_artifact(path)


def test_forged_max_drift_is_refused(tmp_path):
    """Regression 2: a forged residual must not be believed even though
    it cannot move the gate -- it would become the session's justification."""
    path = _write(tmp_path, _artifact(max_drift_mm=9999.0))
    with pytest.raises(ValueError) as excinfo:
        hes.load_gate_artifact(path)
    assert 'inconsistent' in str(excinfo.value)


@pytest.mark.parametrize('field', ['max_drift_mm', 'max_drift_deg',
                                   'median_drift_mm', 'median_drift_deg'])
def test_every_stored_summary_is_recomputed(tmp_path, field):
    path = _write(tmp_path, _artifact(**{field: 1234.5}))
    with pytest.raises(ValueError):
        hes.load_gate_artifact(path)


@pytest.mark.parametrize('field', ['max_drift_mm', 'median_drift_deg',
                                   'suggested_gate_mm'])
def test_forged_nan_summary_is_refused(tmp_path, field):
    """abs(nan - x) > tol is False, so NaN must be caught before comparing."""
    path = _write(tmp_path, _artifact(**{field: float('nan')}))
    with pytest.raises(ValueError):
        hes.load_gate_artifact(path)


def test_provenance_carries_recomputed_values_not_the_files_claims(tmp_path):
    """Even a consistent file's provenance is built from the recomputation."""
    path = _write(tmp_path, _artifact())
    _gate_mm, _gate_deg, provenance = _resolve(['--gate-from', path])
    assert provenance['values_recomputed_from_raw_cycles'] is True
    assert provenance['artifact_max_drift_mm'] == pytest.approx(3.0)
    assert provenance['artifact_median_drift_mm'] == pytest.approx(2.5)
    assert provenance['artifact_max_abs_rad'] == pytest.approx(0.30)


def test_artifact_from_another_geometry_is_refused(tmp_path):
    """A gate measured on different frames is not this arm's return error."""
    payload = _artifact()
    payload['frames'] = dict(payload['frames'], wrist='some_other_link')
    path = _write(tmp_path, payload)
    with pytest.raises(ValueError) as excinfo:
        hes.load_gate_artifact(path, 'base_link', 'link_5')
    assert 'geometry' in str(excinfo.value)


def test_artifact_and_explicit_gate_together_are_refused(tmp_path):
    path = _write(tmp_path, _artifact())
    with pytest.raises(SystemExit):
        _resolve(['--gate-from', path] + GOOD_GATE)


def test_suggested_gate_has_a_floor():
    """An unusually clean run must not produce a gate tighter than the
    measured 0.59 mm observation noise, or every session would abort."""
    gate_mm, gate_deg = hes.suggested_gate(0.01, 0.001)
    assert gate_mm >= 2.0
    assert gate_deg >= 0.2
    assert hes.suggested_gate(10.0, 1.0) == (15.0, 1.5)


# -- writer/reader contract ------------------------------------------------
# The artifact producer and consumer must not drift apart: a gate file whose
# schema quietly changed hands over a number meaning something other than what
# it says. These tests run the real writer into the real reader.

_REFERENCE = {'xyz': [0.5, 0.0, 0.4], 'quat_xyzw': [0.0, 0.0, 0.0, 1.0],
              'sample_count': 3, 'internal_spread_mm': 0.1,
              'internal_spread_deg': 0.01}


def _measured_pose(drift_mm, drift_deg):
    """A raw zero pose the requested distance and angle from the reference."""
    return {'xyz': [_REFERENCE['xyz'][0] + drift_mm / 1000.0,
                    _REFERENCE['xyz'][1], _REFERENCE['xyz'][2]],
            'quat_xyzw': _quat_about_x(drift_deg)}


def _cycles(drifts):
    """Cycles whose stored residuals agree with their own raw poses.

    The loader recomputes every residual from `zero_pose` against
    `zero_reference`, so a fixture carrying only scalars would exercise a shape
    the writer no longer produces -- and would keep hiding that the raw poses
    were never checked at all.
    """
    cycles = []
    for index, (mm, deg) in enumerate(drifts):
        pose = _measured_pose(mm, deg)
        cycles.append({
            'cycle': index + 1, 'measured': True,
            'drift_mm': hes.vector_distance(pose['xyz'],
                                            _REFERENCE['xyz']) * 1000.0,
            'drift_deg': math.degrees(hes.quat_angle_rad(
                pose['quat_xyzw'], _REFERENCE['quat_xyzw'])),
            'zero_pose': pose,
            'delta_xyz_m': [p - r for p, r in zip(pose['xyz'],
                                                  _REFERENCE['xyz'])],
            'internal_spread_mm': 0.1, 'internal_spread_deg': 0.01})
    return cycles


def _calibration_args(cycle_count, purpose=hes.PURPOSE_CALIBRATION):
    parser = hes.build_parser()
    args = parser.parse_args(['--measure-return', str(cycle_count),
                              '--purpose', purpose])
    hes.validate_args(parser, args, hes.DEFAULT_POSES)
    return args


def test_written_artifact_is_readable_by_the_gate_loader(tmp_path):
    args = _calibration_args(4)
    artifact = hes.build_return_artifact(
        args, _cycles([(2.0, 0.20), (3.0, 0.30), (1.0, 0.10), (2.5, 0.25)]),
        list(args.zero_approach), _REFERENCE, complete=True)
    assert artifact['cycles_measured'] == 4
    assert artifact['max_drift_mm'] == pytest.approx(3.0)
    assert artifact['median_drift_deg'] == pytest.approx(0.25)

    path = str(tmp_path / 'return.json')
    hes.write_json(path, artifact)
    gate_mm, gate_deg, summary = hes.load_gate_artifact(path)
    # 1.5x the worst measured return error.
    assert gate_mm == pytest.approx(4.5)
    assert gate_deg == pytest.approx(0.45)
    assert summary['probe_pose_rad'] == list(args.zero_approach)
    assert summary['git_commit'] is not None
    assert summary['max_drift_mm'] == pytest.approx(3.0)


def test_artifact_from_too_few_cycles_cannot_set_a_gate(tmp_path):
    args = _calibration_args(4)
    artifact = hes.build_return_artifact(
        args, _cycles([(2.0, 0.2)]) + [{'cycle': 2, 'measured': False,
                                        'reason': 'board not detected'}],
        list(args.zero_approach), _REFERENCE, complete=True)
    assert artifact['cycles_measured'] == 1
    assert artifact['suggested_gate_mm'] is None

    path = str(tmp_path / 'return.json')
    hes.write_json(path, artifact)
    with pytest.raises(ValueError):
        hes.load_gate_artifact(path)


def test_gate_derived_from_written_artifact_end_to_end(tmp_path):
    args = _calibration_args(3)
    artifact = hes.build_return_artifact(
        args, _cycles([(4.0, 0.5), (6.0, 0.8), (5.0, 0.6)]),
        list(args.zero_approach), _REFERENCE, complete=True)
    path = str(tmp_path / 'return.json')
    hes.write_json(path, artifact)
    gate_mm, gate_deg, provenance = _resolve(['--gate-from', path])
    assert gate_mm == pytest.approx(9.0)
    assert gate_deg == pytest.approx(1.2)
    assert provenance['source'] == 'artifact'
    assert provenance['artifact_max_drift_mm'] == pytest.approx(6.0)


# -- a rejected frame must never be re-accepted ----------------------------
# grab() burns the stamp when it CONSIDERS a frame, not when it accepts one.
# If consumption waited for acceptance, a frame rejected by the diagnostic
# window would still count as unseen, and the next call could pair that same
# stale pose with diagnostics that arrived afterwards -- exactly the
# cross-topic mismatch the stamp check exists to prevent.

class _Stub(object):
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class _FakeTfBuffer(object):
    def lookup_transform(self, *_args, **_kwargs):
        return _Stub(transform=_Stub(
            translation=_Stub(x=0.1, y=0.2, z=0.3),
            rotation=_Stub(x=0.0, y=0.0, z=0.0, w=1.0)))


def _observation(stamp, detected=True, reproj=0.3, corners=54,
                 expected=54, cols=hes.BOARD_COLS, rows=hes.BOARD_ROWS,
                 square=hes.BOARD_SQUARE_MM):
    """One atomic observation: pose, verdict and quality under one stamp."""
    return _Stub(
        header=_Stub(stamp=_Stub(sec=stamp[0], nanosec=stamp[1]),
                     frame_id='camera_optical_frame'),
        detected=detected,
        pose=_Stub(position=_Stub(x=0.5, y=0.0, z=0.4),
                   orientation=_Stub(x=0.0, y=0.0, z=0.0, w=1.0)),
        reprojection_px=reproj,
        corners_detected=corners,
        corners_expected=expected,
        board_cols=cols, board_rows=rows, square_size_mm=square)


def _fake_session(monkeypatch, observation, now=1000.0):
    """A HandEyeSession with the ROS parts replaced, so grab() can be driven."""
    parser = hes.build_parser()
    args = parser.parse_args(GOOD_GATE)
    hes.validate_args(parser, args, hes.DEFAULT_POSES)

    node = object.__new__(hes.HandEyeSession)
    node.args = args
    node.interrupted = {'flag': False, 'signal': None}
    node.tf_buffer = _FakeTfBuffer()
    node.observation = observation
    node.board_stamp_key = (observation.header.stamp.sec,
                            observation.header.stamp.nanosec)
    node.board_arrival = now
    node.consumed_stamp_key = None

    monkeypatch.setattr(hes, 'rclpy', _Stub(
        spin_once=lambda *a, **k: None,
        time=_Stub(Time=lambda *a, **k: None)))

    # The clock must ADVANCE. grab()'s wait loop is bounded by wall time, so a
    # frozen clock makes it spin forever waiting for a deadline that never
    # arrives. Small steps keep the "no new frame" path terminating quickly.
    clock = {'t': now}

    def fake_monotonic():
        clock['t'] += 0.01
        return clock['t']

    monkeypatch.setattr(hes.time, 'monotonic', fake_monotonic)
    return node


def _set_observation(node, observation):
    node.observation = observation
    node.board_stamp_key = (observation.header.stamp.sec,
                            observation.header.stamp.nanosec)


def test_good_frame_is_accepted_once(monkeypatch):
    node = _fake_session(monkeypatch, _observation((10, 0), reproj=0.3))
    sample, reason = node.grab()
    assert reason == 'ok'
    assert sample['board_stamp'] == [10, 0]
    # This frame's own value, from the same message as the pose.
    assert sample['reprojection_px'] == pytest.approx(0.3)
    assert sample['corners'] == [54, 54]


def test_same_frame_cannot_be_sampled_twice(monkeypatch):
    node = _fake_session(monkeypatch, _observation((10, 0)))
    first, reason = node.grab()
    assert reason == 'ok' and first is not None
    second, reason = node.grab()
    assert second is None
    assert 'repeated' in reason


def test_frame_rejected_by_a_gate_is_not_retried(monkeypatch):
    """A frame is spent when CONSIDERED. Even if the next observation would
    pass, the rejected stamp must never be revisited."""
    node = _fake_session(monkeypatch, _observation((10, 0), reproj=9.9))
    sample, reason = node.grab()
    assert sample is None
    assert 'reprojection' in reason

    # Same stamp, now with a passing value: still refused.
    _set_observation(node, _observation((10, 0), reproj=0.2))
    sample, reason = node.grab()
    assert sample is None, 'a rejected frame was re-accepted'
    assert 'repeated' in reason


def test_only_a_new_stamp_is_accepted_after_a_rejection(monkeypatch):
    node = _fake_session(monkeypatch, _observation((10, 0), reproj=9.9))
    assert node.grab()[0] is None

    _set_observation(node, _observation((11, 0), reproj=0.2))
    sample, reason = node.grab()
    assert reason == 'ok'
    assert sample['board_stamp'] == [11, 0]


def test_undetected_frame_is_rejected(monkeypatch):
    node = _fake_session(monkeypatch,
                         _observation((10, 0), detected=False, corners=12))
    sample, reason = node.grab()
    assert sample is None
    assert 'not detected' in reason and '12/54' in reason


def test_partial_board_is_rejected(monkeypatch):
    node = _fake_session(monkeypatch, _observation((10, 0), corners=53))
    sample, reason = node.grab()
    assert sample is None
    assert 'partial board' in reason


@pytest.mark.parametrize('override', [
    {'cols': 6, 'rows': 8},          # the stale default that failed silently
    {'square': 25.0},                # the intrinsics board, not the pose board
    {'cols': 7},
])
def test_board_geometry_mismatch_stops_the_session(monkeypatch, override):
    """The producer's geometry travels with the observation so it can be
    checked. Mismatched defaults previously failed silently."""
    node = _fake_session(monkeypatch, _observation((10, 0), **override))
    sample, reason = node.grab()
    assert sample is None
    assert 'board mismatch' in reason


def test_stale_observation_is_rejected(monkeypatch):
    node = _fake_session(monkeypatch, _observation((10, 0)))
    node.board_arrival = 1000.0 - (node.args.max_age_s + 1.0)
    sample, reason = node.grab()
    assert sample is None
    assert 'stale' in reason


def test_reprojection_at_the_gate_is_accepted(monkeypatch):
    node = _fake_session(monkeypatch, _observation((10, 0), reproj=1.0))
    assert node.grab()[1] == 'ok'


# -- clamped motion must never be recorded as the requested motion ---------

def test_pose_outside_the_envelope_is_rejected_not_clamped():
    """The artifact would otherwise claim a movement that never happened:
    --zero-approach [0.9,...] with a 0.30 rad envelope commands 0.30."""
    with pytest.raises(SystemExit):
        _parse(GOOD_GATE + ['--measure-return', '3',
                            '--zero-approach', '[0.9,0,0,0,0]', '--dry-run'])


def test_pose_file_outside_the_envelope_is_rejected(tmp_path):
    path = tmp_path / 'poses.json'
    path.write_text(json.dumps([[0.9, 0.0, 0.0, 0.0, 0.0]]))
    with pytest.raises(SystemExit):
        _parse(GOOD_GATE + ['--poses-json', str(path), '--dry-run'])


def test_pose_at_the_envelope_edge_is_allowed():
    parser, args, poses = _parse(
        GOOD_GATE + ['--max-abs-rad', '0.30',
                     '--zero-approach', '[0.30,-0.30,0,0,0]', '--dry-run'])
    assert args.zero_approach == [0.30, -0.30, 0, 0, 0]


def test_artifact_records_the_executed_probe(tmp_path):
    args = _calibration_args(4)
    executed = hes.clamp_pose(args.zero_approach, args.max_abs_rad)
    artifact = hes.build_return_artifact(
        args, _cycles([(2.0, 0.2), (3.0, 0.3), (1.0, 0.1)]), executed,
        _REFERENCE, complete=True)
    assert artifact['executed_probe_pose_rad'] == executed
    # Validation refuses out-of-envelope poses, so these must agree.
    assert artifact['executed_probe_pose_rad'] == artifact['probe_pose_rad']


# -- dry-run drives the real entry point -----------------------------------

def test_dry_run_returns_zero_with_a_gate():
    assert hes.main(GOOD_GATE + ['--dry-run']) == 0


def test_dry_run_without_gate_exits_nonzero():
    with pytest.raises(SystemExit) as excinfo:
        hes.main(['--dry-run'])
    assert excinfo.value.code != 0


def test_measure_return_dry_run_needs_no_gate():
    assert hes.main(['--measure-return', '8', '--dry-run']) == 0


# -- the approach direction must be identical for every zero measurement ---
# The 2026-07-31 run took its reference from the arm's startup pose and every
# cycle from the probe pose. With no encoders, backlash makes those two
# approaches settle differently, which showed up as ~62 mm of "drift" that was
# really an artefact of how the arm arrived.

def test_v3_artifact_is_refused_because_its_residuals_are_not_comparable(tmp_path):
    path = _write(tmp_path, _artifact(version=3))
    with pytest.raises(ValueError) as excinfo:
        hes.load_gate_artifact(path)
    assert 'version' in str(excinfo.value)


def test_measure_zero_passes_through_the_approach_before_zero(monkeypatch):
    """Reference and cycles must run the identical path, approach included."""
    node = _fake_session(monkeypatch, _observation((10, 0)))
    commanded = []
    monkeypatch.setattr(type(node), 'move_to',
                        lambda self, pose, secs: commanded.append(list(pose)))
    monkeypatch.setattr(type(node), 'spin_for', lambda self, secs: None)

    real_grab = hes.HandEyeSession.grab
    stamps = iter(range(20, 60))

    def fresh_grab(self):
        # Feed a genuinely new frame each call; the real grab still runs.
        _set_observation(self, _observation((next(stamps), 0)))
        return real_grab(self)

    monkeypatch.setattr(type(node), 'grab', fresh_grab)
    reference, reason = node.measure_zero()
    assert reason == 'ok' and reference is not None
    # Approach pose first, zero second -- in that order, every time.
    assert commanded[0] == list(node.args.zero_approach)
    assert commanded[1] == [0.0] * 5


# -- a diagnostic or interrupted run must never set the gate ---------------
# MIN_RETURN_CYCLES is 3, so a three-cycle hypothesis test would otherwise
# produce a gate-valid artifact. And the 2026-07-31 run lost every completed
# cycle because the file was only written at the end.

def test_diagnostic_artifact_cannot_set_a_gate(tmp_path):
    args = _calibration_args(3, purpose=hes.PURPOSE_DIAGNOSTIC)
    artifact = hes.build_return_artifact(
        args, _cycles([(2.0, 0.2), (3.0, 0.3), (1.0, 0.1)]),
        list(args.zero_approach), _REFERENCE, complete=True)
    assert artifact['usable_for_gate'] is False
    path = _write(tmp_path, artifact)
    with pytest.raises(ValueError) as excinfo:
        hes.load_gate_artifact(path)
    assert 'diagnostic' in str(excinfo.value)


def test_incomplete_artifact_cannot_set_a_gate(tmp_path):
    args = _calibration_args(8)
    artifact = hes.build_return_artifact(
        args, _cycles([(2.0, 0.2), (3.0, 0.3), (1.0, 0.1)]),
        list(args.zero_approach), _REFERENCE, complete=False)
    assert artifact['complete'] is False
    assert artifact['usable_for_gate'] is False
    path = _write(tmp_path, artifact)
    with pytest.raises(ValueError) as excinfo:
        hes.load_gate_artifact(path)
    assert 'complete' in str(excinfo.value)


def test_purpose_defaults_to_diagnostic():
    """A run must declare itself a calibration; it cannot drift into one."""
    parser = hes.build_parser()
    args = parser.parse_args(['--measure-return', '3'])
    assert args.purpose == hes.PURPOSE_DIAGNOSTIC


def test_artifact_keeps_raw_poses_for_axis_recovery(tmp_path):
    """Scalar magnitudes could not say whether 5.4 deg matched a joint axis."""
    args = _calibration_args(3)
    artifact = hes.build_return_artifact(
        args, _cycles([(2.0, 0.2), (3.0, 0.3), (1.0, 0.1)]),
        list(args.zero_approach), _REFERENCE, complete=True)
    assert artifact['zero_reference']['quat_xyzw'] == [0.0, 0.0, 0.0, 1.0]
    for entry in artifact['cycles']:
        assert 'zero_pose' in entry and 'delta_xyz_m' in entry


# -- the loader must re-derive the residuals, not read them -----------------
# Regression round 5, finding 4: v4 stores raw poses but nothing required them or
# checked them, so a gate-valid file could still carry residuals that its own
# poses contradict -- the exact shape of file that made the 2026-07-31
# rotation impossible to attribute to an axis.

@pytest.mark.parametrize('mutate', [
    lambda cycle: cycle.pop('zero_pose'),
    lambda cycle: cycle.pop('delta_xyz_m'),
    lambda cycle: cycle.update(zero_pose={'xyz': [0.5, 0.0, 0.4]}),
    lambda cycle: cycle.update(delta_xyz_m=[0.0, 0.0]),
    lambda cycle: cycle.update(delta_xyz_m=[9.0, 0.0, 0.0]),
    lambda cycle: cycle['zero_pose'].update(quat_xyzw=[0.0, 0.0, 0.0, 4.0]),
    lambda cycle: cycle.update(drift_mm=1.0),      # contradicts its own pose
    lambda cycle: cycle.update(drift_deg=9.0),
    lambda cycle: cycle.update(drift_mm=float('nan')),
])
def test_cycle_whose_raw_poses_are_missing_or_contradictory_is_refused(
        tmp_path, mutate):
    payload = _artifact()
    mutate(payload['cycles'][1])
    path = _write(tmp_path, payload)
    with pytest.raises(ValueError):
        hes.load_gate_artifact(path)


def test_gate_comes_from_the_raw_poses_not_the_stored_residuals(tmp_path):
    """Deleting the stored scalars changes nothing: the poses are the data."""
    payload = _artifact()
    for cycle in payload['cycles']:
        cycle['drift_mm'] = hes.vector_distance(
            cycle['zero_pose']['xyz'], payload['zero_reference']['xyz']) * 1000.0
    path = _write(tmp_path, payload)
    gate_mm, gate_deg, summary = hes.load_gate_artifact(path)
    assert gate_mm == pytest.approx(4.5)      # 1.5 x the widest raw pose, 3 mm
    assert gate_deg == pytest.approx(0.45)
    assert summary['max_drift_mm'] == pytest.approx(3.0)


def test_write_json_is_atomic_and_leaves_no_temp(tmp_path):
    path = str(tmp_path / 'sub' / 'artifact.json')
    hes.write_json(path, {'a': 1})
    assert json.loads(open(path).read()) == {'a': 1}
    assert not os.path.exists(path + '.tmp')
    hes.write_json(path, {'a': 2})
    assert json.loads(open(path).read()) == {'a': 2}


# -- after a signal, the ONLY motion is the verified zero return ------------
# Regression round 5, findings 1-3. The previous version set a flag that only the
# repeatability loop read: a SIGINT during a 15-pose capture drove all fifteen
# poses and exited 0. A failed zero return printed CUT THE SERVO RAIL and also
# exited 0, because the early `return` in the measure-return branch fixed the
# code before the finally block could raise it. And the return was confirmed
# from whatever /joint_states value was already in hand, so an arm parked at a
# stale zero reading "confirmed" a return that never happened.

class _FakeTrajectory(object):
    def __init__(self):
        self.joint_names = []
        self.points = []


class _FakeTrajectoryPoint(object):
    def __init__(self):
        self.positions = []
        self.time_from_start = _Stub(sec=0, nanosec=0)


_STAMP_CLOCK = [100]


def _joint_state(positions, stamp=None):
    """A JointState the way the controller publishes one: stamped.

    Regression round 6: the earlier fixture built an unstamped stub, so the
    freshness gate could be read from callback order alone and nobody noticed
    that the publisher's own stamp was never read. `stamp` is source seconds --
    default advances, so construction order models publication order; pass it
    explicitly to build a message that was produced out of order, before a
    barrier, or (0) without a usable stamp at all.
    """
    if stamp is None:
        _STAMP_CLOCK[0] += 1
        stamp = _STAMP_CLOCK[0]
    return _Stub(name=list(hes.ARM_JOINTS), position=list(positions),
                 header=_Stub(stamp=_Stub(sec=int(stamp), nanosec=0)))


@pytest.fixture
def restore_signals():
    """These tests install real handlers; pytest must get its own back."""
    saved = [(number, signal.getsignal(number))
             for number in (signal.SIGINT, signal.SIGTERM)]
    yield
    for number, handler in saved:
        signal.signal(number, handler)


def _motion_node(monkeypatch, argv):
    """A session with the REAL move_to and interrupt logic, and a fake bus.

    Faking move_to would test the mock: the refusal being checked lives inside
    it, which is the point -- it is the one place every commanded pose passes.
    """
    parser = hes.build_parser()
    args = parser.parse_args(argv)
    hes.validate_args(parser, args, hes.DEFAULT_POSES)
    monkeypatch.setattr(hes, 'JointTrajectory', _FakeTrajectory, raising=False)
    monkeypatch.setattr(hes, 'JointTrajectoryPoint', _FakeTrajectoryPoint,
                        raising=False)

    node = object.__new__(hes.HandEyeSession)
    node.args = args
    node.interrupted = {'flag': False, 'signal': None}
    node.gate_mm, node.gate_deg = 4.0, 1.0
    node.gate_provenance = {'source': 'operator-supplied', 'path': None}
    node.commanded = []
    node.pub = _Stub(
        publish=lambda msg: node.commanded.append(list(msg.points[0].positions)))
    node.get_logger = lambda: _Stub(warn=lambda *a, **k: None)
    node.joint_positions = None
    node.joint_state_seq = 0
    node.joint_state_msgs = 0
    node.joint_state_stamp = None
    node.zero_return_reason = None
    # The real session builds a listener node that nothing spins; `executor is
    # None` is the invariant the take path checks before every read.
    node.state_node = _Stub(destroy_node=lambda: None, executor=None)
    node.state_sub = None       # _drive installs the transport fake
    node.samples, node.skipped, node.checks = [], [], []
    node.span, node.zero_ref = 0, None
    node.destroy_node = lambda: None
    return args, node


class _FakeSubHandle(object):
    """The transport boundary: take_message, exactly as rclpy exposes it.

    The fake sits here rather than at rclpy.spin_once because that is where the
    real code now reads joint states. `None` means the queue is empty, which is
    the statement the whole freshness gate rests on.
    """

    def __init__(self, pop):
        self._pop = pop

    def __enter__(self):
        # rclpy's handle returns None from __enter__ -- it is a lock, not a
        # value. The first version of this fake returned self, which hid a real
        # `with ... as handle` bug that only the live API caught.
        return None

    def __exit__(self, *_exc):
        return False

    def take_message(self, _msg_type, _raw):
        msg = self._pop()
        return None if msg is None else (msg, None)


def _drive(monkeypatch, node, feed, step=0.5, queued=None, others=0):
    """Stand in for rclpy, modelling the two things the fence depends on.

    Two joint-state queues, because the bug Review found in round 6 lives in the
    difference between them. `queued` messages already sit in the transport
    when
    the call starts, so a take before the zero command finds them, while `feed`
    messages do not exist until the command has been published. One queue
    cannot
    express "produced before the command, delivered after it", which is exactly
    the case that must not confirm a return.

    `others` is work pending on the SESSION node -- camera and TF callbacks.
    Round 7 and round 8 both turned on it: while joint states were read through
    an executor, that work could be mistaken for an empty joint-state queue.
    The
    session node's spin still runs it here, and it must now be structurally
    incapable of touching the joint-state queue.
    """
    waiting = list(queued or [])
    later = list(feed)
    unrelated = [others]
    commands_before = len(node.commanded)

    def pop():
        if waiting:
            return waiting.pop(0)
        if later and len(node.commanded) > commands_before:
            return later.pop(0)
        return None

    node.state_sub = _Stub(handle=_FakeSubHandle(pop), msg_type=object,
                           raw=False)

    def spin_once(*_args, **_kwargs):
        # The session node's own callbacks. They cannot reach joint states: the
        # only reader is the subscription handle above.
        if unrelated[0]:
            unrelated[0] -= 1

    monkeypatch.setattr(hes, 'rclpy', _Stub(
        init=lambda *a, **k: None, ok=lambda: False, shutdown=lambda: None,
        spin_once=spin_once, time=_Stub(Time=lambda *a, **k: None)),
        raising=False)
    monkeypatch.setattr(hes, 'ROS_IMPORT_ERROR', None)
    # Nothing spins the listener, so the echo wait polls; the sleep would
    # otherwise add real seconds to the suite.
    monkeypatch.setattr(hes.time, 'sleep', lambda _seconds: None)

    # The wait loops are bounded by wall time, so the clock must advance or a
    # 20 s timeout never arrives.
    clock = {'t': 1000.0}

    def fake_monotonic():
        clock['t'] += step
        return clock['t']

    monkeypatch.setattr(hes.time, 'monotonic', fake_monotonic)


def test_stop_handlers_flag_both_signals(restore_signals):
    """SIGTERM counts too: a killed container must not skip the return."""
    for number in (signal.SIGINT, signal.SIGTERM):
        state = hes.install_stop_handlers({'flag': False, 'signal': None})
        signal.getsignal(number)(number, None)
        assert state['flag'] is True
        assert state['signal'] == number


def test_move_to_refuses_every_pose_but_the_zero_return(monkeypatch):
    _args, node = _motion_node(monkeypatch, GOOD_GATE)
    node.move_to([0.1] * 5, 5.0)
    node.interrupted.update(flag=True, signal=signal.SIGINT)

    with pytest.raises(hes.SessionInterrupted):
        node.move_to([0.1] * 5, 5.0)
    # Zero, but not declared as the recovery: the exemption cannot be borrowed.
    with pytest.raises(hes.SessionInterrupted):
        node.move_to([0.0] * 5, 5.0)
    # ... and the recovery flag cannot smuggle a non-zero pose through either.
    with pytest.raises(hes.SessionInterrupted):
        node.move_to([0.1] * 5, 5.0, zero_return=True)

    node.move_to([0.0] * 5, 5.0, zero_return=True)
    assert node.commanded == [[0.1] * 5, [0.0] * 5]


def test_measure_zero_refuses_to_start_its_approach_after_a_signal(monkeypatch):
    """The approach excursion is real motion, reached from every caller."""
    _args, node = _motion_node(monkeypatch, GOOD_GATE)
    node.interrupted.update(flag=True, signal=signal.SIGTERM)
    with pytest.raises(hes.SessionInterrupted):
        node.measure_zero()
    assert node.commanded == []


def test_zero_return_refuses_a_stale_echo(monkeypatch):
    """A zero reading taken BEFORE the command proves nothing about after it."""
    _args, node = _motion_node(monkeypatch, GOOD_GATE)
    node._on_joint_states(_joint_state([0.0] * 5))
    _drive(monkeypatch, node, feed=[])

    delivered, positions = node.return_to_zero_verified()
    assert delivered is False
    assert positions is None, 'a pre-command reading was treated as delivery'
    assert node.commanded == [[0.0] * 5]


def test_zero_return_confirms_on_fresh_echoes(monkeypatch):
    _args, node = _motion_node(monkeypatch, GOOD_GATE)
    node._on_joint_states(_joint_state([0.20] * 5, stamp=10))  # probe pose
    _drive(monkeypatch, node,
           feed=[_joint_state([0.0] * 5, stamp=11 + n)
                 for n in range(hes.ZERO_RETURN_CONFIRM_SAMPLES)])

    delivered, positions = node.return_to_zero_verified()
    assert delivered is True
    assert positions == [0.0] * 5


def test_zero_return_needs_consecutive_fresh_zeros(monkeypatch):
    """One reading can straddle the command; a run of them cannot."""
    _args, node = _motion_node(monkeypatch, GOOD_GATE)
    node._on_joint_states(_joint_state([0.20] * 5, stamp=10))
    _drive(monkeypatch, node, feed=[_joint_state([0.0] * 5, stamp=11),
                                    _joint_state([0.20] * 5, stamp=12),
                                    _joint_state([0.0] * 5, stamp=13),
                                    _joint_state([0.0] * 5, stamp=14)])
    delivered, _positions = node.return_to_zero_verified()
    assert delivered is False


# -- Regression round 6: a callback is not a publication -------------------------
# Counting callbacks made "fresh" mean "delivered after the command". A message
# that entered the DDS queue BEFORE the command and was delivered after it
# satisfied that, so three queued pre-command zero echoes could confirm a
# return while the arm sat at the probe pose. Freshness now comes from the
# publisher's
# own header.stamp, compared against a barrier taken in that same clock.


def test_zero_return_refuses_pre_command_stamps_delivered_late(monkeypatch):
    """Regression reproduction: command at 20 s, sources stamped 11/12/13 s."""
    _args, node = _motion_node(monkeypatch, GOOD_GATE)
    node._on_joint_states(_joint_state([0.20] * 5, stamp=20))
    _drive(monkeypatch, node,
           feed=[_joint_state([0.0] * 5, stamp=11),
                 _joint_state([0.0] * 5, stamp=12),
                 _joint_state([0.0] * 5, stamp=13)])

    delivered, positions = node.return_to_zero_verified()
    assert delivered is False, 'a pre-command echo was accepted as fresh'
    # Round 10: a usable arm reading with a stamp older than the barrier is
    # unorderable, so it refuses fail-closed and reports the offending reading
    # rather than skipping it.
    assert 'cannot be ordered' in node.zero_return_reason
    assert node.commanded == [[0.0] * 5], 'the zero command must still go out'


def test_zero_return_drains_the_queue_before_taking_the_barrier(monkeypatch):
    """The barrier is what the PUBLISHER had produced, not what was processed.

    Stale zero echoes from before the probe sit in the transport, stamped newer
    than the last message this process handled. Draining moves the barrier past
    them; without that they clear a barrier they predate and confirm a return
    while the post-command echoes say the arm is still at the probe pose.
    """
    _args, node = _motion_node(monkeypatch, GOOD_GATE)
    node._on_joint_states(_joint_state([0.20] * 5, stamp=10))
    _drive(monkeypatch, node,
           queued=[_joint_state([0.0] * 5, stamp=11),
                   _joint_state([0.0] * 5, stamp=12),
                   _joint_state([0.0] * 5, stamp=13)],
           feed=[_joint_state([0.20] * 5, stamp=14),
                 _joint_state([0.20] * 5, stamp=15),
                 _joint_state([0.20] * 5, stamp=16)])

    delivered, positions = node.return_to_zero_verified()
    assert delivered is False, 'queued pre-command zeros confirmed a return'
    assert positions == [0.20] * 5


# -- Regression round 7: an unrelated callback is not a quiet queue ---------------
# The drain used to spin the SESSION node and call a spin that ran no
# joint-state callback "quiet". That node also carries the camera and TF
# callbacks, so three of those satisfied the test while newer pre-command joint
# states sat in the queue: the barrier stayed behind them and they confirmed a
# return the arm had not made. The listener node now carries nothing else, so a
# spin of it runs the joint-state callback or runs nothing.


def test_zero_return_fence_survives_unrelated_callbacks(monkeypatch):
    """Regression reproduction: unrelated callbacks, then queued stale zeros.

    The queued zeros are stamped newer than the last processed message but were
    produced BEFORE the command; the arm is at the probe pose and stays there.
    A drain fooled by the unrelated callbacks confirms a return here.
    """
    _args, node = _motion_node(monkeypatch, GOOD_GATE)
    node._on_joint_states(_joint_state([0.20] * 5, stamp=10))
    _drive(monkeypatch, node, others=3,
           queued=[_joint_state([0.0] * 5, stamp=11),
                   _joint_state([0.0] * 5, stamp=12),
                   _joint_state([0.0] * 5, stamp=13)],
           feed=[_joint_state([0.20] * 5, stamp=14),
                 _joint_state([0.20] * 5, stamp=15),
                 _joint_state([0.20] * 5, stamp=16)])

    delivered, positions = node.return_to_zero_verified()
    assert delivered is False, 'unrelated callbacks passed as a drained queue'
    assert positions == [0.20] * 5, 'the arm was still at the probe pose'


def test_zero_return_refuses_when_the_queue_never_goes_quiet(monkeypatch):
    """A publisher outrunning the drain leaves the barrier unprovable."""
    _args, node = _motion_node(monkeypatch, GOOD_GATE)
    node._on_joint_states(_joint_state([0.0] * 5, stamp=10))
    flood = [_joint_state([0.0] * 5, stamp=11 + n)
             for n in range(hes.ZERO_RETURN_DRAIN_MESSAGES + 5)]
    _drive(monkeypatch, node, queued=flood, feed=[])

    delivered, positions = node.return_to_zero_verified()
    assert delivered is False, 'confirmed on a barrier that was never fenced'
    assert positions is None
    assert 'never went quiet' in node.zero_return_reason
    assert node.commanded == [[0.0] * 5], 'the zero command must still go out'


def test_drain_counts_messages_not_usable_states(monkeypatch):
    """A message naming other joints is still a message in the queue.

    The quiet test asks whether the queue is empty. Keying it on USABLE states
    would drain a gripper-only publisher's message and call the queue empty
    with arm states still behind it.
    """
    _args, node = _motion_node(monkeypatch, GOOD_GATE)
    node._on_joint_states(_joint_state([0.20] * 5, stamp=10))
    gripper = _Stub(name=['gripper_joint'], position=[0.0],
                    header=_Stub(stamp=_Stub(sec=11, nanosec=0)))
    _drive(monkeypatch, node,
           queued=[gripper] + [_joint_state([0.0] * 5, stamp=12 + n)
                               for n in range(3)],
           feed=[_joint_state([0.20] * 5, stamp=20 + n) for n in range(3)])

    delivered, _positions = node.return_to_zero_verified()
    assert delivered is False, 'the drain stopped at an unusable message'


# -- Regression round 8: the executor was the wrong place to read from -----------
# Round 7 put /joint_states on its own node and called a spin that ran no
# callback "quiet". Review then measured this machine's Jazzy: no
# `enable_type_description_service` keyword, so the constructor fell back and
# left a live get_type_description service plus two QoS EventHandler waitables
# that cannot be switched off. A spin could service that request instead of the
# subscription, and Regression reproduced a false confirmation through it. Joint
# states are now taken straight off the subscription, so "empty" is the
# transport's answer about that subscription and nothing else can stand in for
# it.


def test_zero_return_counts_every_message_of_one_take(monkeypatch):
    """A take can return a batch; each message must be judged on its own.

    Folding a batch into its newest message turns "three consecutive fresh
    zeros" into "three polls" -- weaker than the gate claims to be, and it
    stalls a confirmation that the data supports.
    """
    _args, node = _motion_node(monkeypatch, GOOD_GATE)
    node._on_joint_states(_joint_state([0.20] * 5, stamp=10))
    # All three arrive in a single take, as a 50 Hz publisher and a 10 ms poll
    # routinely produce.
    _drive(monkeypatch, node,
           feed=[_joint_state([0.0] * 5, stamp=11 + n) for n in range(3)])

    delivered, positions = node.return_to_zero_verified()
    assert delivered is True, 'a batched take lost samples'
    assert positions == [0.0] * 5


def test_zero_return_refuses_if_an_executor_holds_the_listener(monkeypatch):
    """An executor would consume messages behind the take's back.

    Then neither the count nor the order describes the queue, so the fence
    cannot be established -- but the arm is still commanded home.
    """
    _args, node = _motion_node(monkeypatch, GOOD_GATE)
    node._on_joint_states(_joint_state([0.0] * 5, stamp=10))
    _drive(monkeypatch, node,
           feed=[_joint_state([0.0] * 5, stamp=11 + n) for n in range(3)])
    node.state_node = _Stub(destroy_node=lambda: None, executor=object())

    delivered, positions = node.return_to_zero_verified()
    assert delivered is False
    assert positions is None
    assert 'executor is attached' in node.zero_return_reason
    assert node.commanded == [[0.0] * 5], 'the zero command must still go out'


def test_session_spin_cannot_consume_joint_states(monkeypatch):
    """The session node's callbacks and the joint-state queue are disjoint.

    This is the round 7 and round 8 failure made structural rather than
    argued: spinning the session node any number of times must not move the
    joint-state queue, because the only reader is the subscription handle.
    """
    _args, node = _motion_node(monkeypatch, GOOD_GATE)
    _drive(monkeypatch, node, others=5,
           queued=[_joint_state([0.0] * 5, stamp=11)], feed=[])
    for _ in range(5):
        hes.rclpy.spin_once(node, timeout_sec=0.0)
    assert node.joint_state_msgs == 0, 'a session spin took a joint state'
    samples, drained = node._take_joint_states()
    assert len(samples) == 1 and drained is True


# -- Regression round 9: a batch is judged whole, or not at all -------------------
# The batch loop returned on the third consecutive zero from inside itself, so
# a newer out-of-tolerance sample sitting behind those zeros in the SAME read
# was never looked at. The arm was back at the probe pose and the tool said
# CONFIRMED.


def test_zero_return_refuses_when_a_later_sample_in_the_batch_refutes_it(
        monkeypatch):
    """Regression case: [zero@11, zero@12, zero@13, nonzero@14] in one take."""
    _args, node = _motion_node(monkeypatch, GOOD_GATE)
    node._on_joint_states(_joint_state([0.20] * 5, stamp=10))
    _drive(monkeypatch, node,
           feed=[_joint_state([0.0] * 5, stamp=11),
                 _joint_state([0.0] * 5, stamp=12),
                 _joint_state([0.0] * 5, stamp=13),
                 _joint_state([0.20] * 5, stamp=14)])

    delivered, positions = node.return_to_zero_verified()
    assert delivered is False, 'a refuted zero run confirmed the return'
    assert positions == [0.20] * 5, 'the newest sample was not reported'


def test_zero_return_confirms_on_a_trailing_zero_run(monkeypatch):
    """The mirror case: [nonzero, zero, zero, zero] in one take must pass.

    The run of fresh zeros that is unbroken at the END of the batch is exactly
    what the gate is about, so the fix must not refuse this one.
    """
    _args, node = _motion_node(monkeypatch, GOOD_GATE)
    node._on_joint_states(_joint_state([0.20] * 5, stamp=10))
    _drive(monkeypatch, node,
           feed=[_joint_state([0.20] * 5, stamp=11),
                 _joint_state([0.0] * 5, stamp=12),
                 _joint_state([0.0] * 5, stamp=13),
                 _joint_state([0.0] * 5, stamp=14)])

    delivered, positions = node.return_to_zero_verified()
    assert delivered is True
    assert positions == [0.0] * 5


def test_zero_return_refuses_unstamped_echoes(monkeypatch):
    """A usable arm reading with an all-zero stamp cannot be ordered.

    Round 10 made this fail-closed rather than skipped: an unstamped arm
    reading in a run of zeros used to be stepped over, so an unstamped nonzero
    would have hidden inside a confirmed run. Now the first one refuses.
    """
    _args, node = _motion_node(monkeypatch, GOOD_GATE)
    node._on_joint_states(_joint_state([0.20] * 5, stamp=10))
    _drive(monkeypatch, node,
           feed=[_joint_state([0.0] * 5, stamp=0)] * 4)

    delivered, positions = node.return_to_zero_verified()
    assert delivered is False
    assert 'cannot be ordered' in node.zero_return_reason


def test_zero_return_refuses_a_bad_stamp_hidden_in_a_zero_run(monkeypatch):
    """Regression round 10: a nonzero reading with an unusable stamp, mid-run.

    [q0@11, q0@12, q0.20@zero-stamp, q0@13]: the nonzero is real motion away
    from zero, but its stamp is unorderable. Skipping it would let the zeros
    around it confirm a return the arm did not make.
    """
    _args, node = _motion_node(monkeypatch, GOOD_GATE)
    node._on_joint_states(_joint_state([0.20] * 5, stamp=10))
    _drive(monkeypatch, node,
           feed=[_joint_state([0.0] * 5, stamp=11),
                 _joint_state([0.0] * 5, stamp=12),
                 _joint_state([0.20] * 5, stamp=0),
                 _joint_state([0.0] * 5, stamp=13)])

    delivered, positions = node.return_to_zero_verified()
    assert delivered is False, 'a bad-stamp nonzero was skipped past'
    assert 'cannot be ordered' in node.zero_return_reason


def test_zero_return_refuses_a_duplicate_stamp_nonzero_in_a_run(monkeypatch):
    """The duplicate-stamp variant: a nonzero re-using an earlier stamp."""
    _args, node = _motion_node(monkeypatch, GOOD_GATE)
    node._on_joint_states(_joint_state([0.20] * 5, stamp=10))
    _drive(monkeypatch, node,
           feed=[_joint_state([0.0] * 5, stamp=11),
                 _joint_state([0.0] * 5, stamp=12),
                 _joint_state([0.20] * 5, stamp=12),   # duplicate of 12
                 _joint_state([0.0] * 5, stamp=13)])

    delivered, _positions = node.return_to_zero_verified()
    assert delivered is False
    assert 'cannot be ordered' in node.zero_return_reason


def test_zero_return_refuses_a_cap_limited_confirmation_batch(monkeypatch):
    """Regression round 10: the take cap is not a proof the queue is empty.

    A nonzero echo queued behind a full cap of zeros is never seen if the cap
    is read as silence, so the arm is confirmed home while it sits at the
    probe pose.
    """
    _args, node = _motion_node(monkeypatch, GOOD_GATE)
    node._on_joint_states(_joint_state([0.20] * 5, stamp=10))
    cap = hes.ZERO_RETURN_DRAIN_MESSAGES
    feed = [_joint_state([0.0] * 5, stamp=11 + n) for n in range(cap)]
    feed.append(_joint_state([0.20] * 5, stamp=11 + cap))
    _drive(monkeypatch, node, feed=feed)

    delivered, _positions = node.return_to_zero_verified()
    assert delivered is False, 'a cap batch read as the whole queue'
    assert 'cap' in node.zero_return_reason


def test_zero_return_refuses_partial_arm_state_hidden_in_zero_run(monkeypatch):
    """A partial arm reading is not harmless gripper-only traffic.

    Two full zeros before and one after a partial nonzero are not three
    consecutive complete zero states. Skipping the partial reading falsely
    confirms the return.
    """
    _args, node = _motion_node(monkeypatch, GOOD_GATE)
    node._on_joint_states(_joint_state([0.20] * 5, stamp=10))
    partial = _Stub(
        name=['joint_1'], position=[0.20],
        header=_Stub(stamp=_Stub(sec=13, nanosec=0)))
    _drive(monkeypatch, node,
           feed=[_joint_state([0.0] * 5, stamp=11),
                 _joint_state([0.0] * 5, stamp=12),
                 partial,
                 _joint_state([0.0] * 5, stamp=14)])

    delivered, positions = node.return_to_zero_verified()
    assert delivered is False
    assert positions is None
    assert 'incomplete or malformed' in node.zero_return_reason


def test_zero_return_ignores_gripper_only_state_inside_zero_run(monkeypatch):
    """Unrelated gripper traffic must not poison complete arm evidence."""
    _args, node = _motion_node(monkeypatch, GOOD_GATE)
    node._on_joint_states(_joint_state([0.20] * 5, stamp=10))
    gripper = _Stub(
        name=['joint_6'], position=[0.0],
        header=_Stub(stamp=_Stub(sec=13, nanosec=0)))
    _drive(monkeypatch, node,
           feed=[_joint_state([0.0] * 5, stamp=11),
                 _joint_state([0.0] * 5, stamp=12),
                 gripper,
                 _joint_state([0.0] * 5, stamp=14)])

    delivered, positions = node.return_to_zero_verified()
    assert delivered is True
    assert positions == [0.0] * 5


def test_zero_return_still_commands_zero_if_fence_take_raises(monkeypatch):
    """A broken evidence path must not prevent the best-effort zero command."""
    class RaisingHandle(object):
        def __enter__(self):
            return None

        def __exit__(self, *_exc):
            return False

        def take_message(self, *_args):
            raise RuntimeError('synthetic transport failure')

    _args, node = _motion_node(monkeypatch, GOOD_GATE)
    node._on_joint_states(_joint_state([0.20] * 5, stamp=10))
    node.state_sub = _Stub(handle=RaisingHandle(), msg_type=object, raw=False)

    delivered, positions = node.return_to_zero_verified()
    assert delivered is False
    assert positions is None
    assert node.commanded == [[0.0] * 5]
    assert 'fence failed before the zero command' in node.zero_return_reason


def test_zero_return_refuses_without_a_barrier(monkeypatch):
    """No stamped state before the command means nothing can postdate it."""
    _args, node = _motion_node(monkeypatch, GOOD_GATE)
    _drive(monkeypatch, node,
           feed=[_joint_state([0.0] * 5, stamp=11 + n) for n in range(3)])

    delivered, positions = node.return_to_zero_verified()
    assert delivered is False
    assert positions is None
    assert 'before the zero command' in node.zero_return_reason
    assert node.commanded == [[0.0] * 5]


def test_zero_return_ignores_duplicate_and_backwards_stamps(monkeypatch):
    """Three copies of one message are one sample, not three."""
    _args, node = _motion_node(monkeypatch, GOOD_GATE)
    node._on_joint_states(_joint_state([0.20] * 5, stamp=10))
    _drive(monkeypatch, node, feed=[_joint_state([0.0] * 5, stamp=11),
                                    _joint_state([0.0] * 5, stamp=11),
                                    _joint_state([0.0] * 5, stamp=11),
                                    _joint_state([0.0] * 5, stamp=9),
                                    _joint_state([0.0] * 5, stamp=11)])

    delivered, _positions = node.return_to_zero_verified()
    assert delivered is False


def test_stamp_key_rejects_what_cannot_be_compared():
    assert hes.stamp_key(_joint_state([0.0] * 5, stamp=7)) == (7, 0)
    assert hes.stamp_key(_joint_state([0.0] * 5, stamp=0)) is None
    assert hes.stamp_key(_Stub(name=[], position=[])) is None
    assert hes.stamp_key(
        _Stub(header=_Stub(stamp=_Stub(sec=True, nanosec=0)))) is None
    assert hes.stamp_key(
        _Stub(header=_Stub(stamp=_Stub(sec=1.5, nanosec=0)))) is None
    # A stamp of 0 s but nonzero nanoseconds is a real, usable stamp.
    assert hes.stamp_key(
        _Stub(header=_Stub(stamp=_Stub(sec=0, nanosec=1)))) == (0, 1)


def _run_main(monkeypatch, node, argv, feed):
    monkeypatch.setattr(hes, 'HandEyeSession', lambda *a, **k: node)
    _drive(monkeypatch, node, feed)
    return hes.main(argv)


def test_signal_during_capture_commands_nothing_but_the_zero_return(
        monkeypatch, restore_signals, tmp_path):
    """The reproduction: a REAL SIGINT while waiting for vision, before pose 1.

    Board measurement is canned so nothing but the capture loop and the
    trajectory publisher is under test -- when the guard is removed, this
    probe drives all fifteen poses, which is what the bench run did.
    """
    argv = GOOD_GATE + ['--out', str(tmp_path / 'session.json')]
    _args, node = _motion_node(monkeypatch, argv)
    node.measure_zero = lambda: (dict(_REFERENCE), 'ok')
    node.grab = lambda: (None, 'board not sampled in this probe')
    fired = {'done': False}

    def wait(_seconds):
        if not fired['done']:
            fired['done'] = True
            os.kill(os.getpid(), signal.SIGINT)

    node.spin_for = wait
    node._on_joint_states(_joint_state([0.0] * 5, stamp=10))    # live barrier
    code = _run_main(monkeypatch, node, argv,
                     [_joint_state([0.0] * 5, stamp=11 + n)
                      for n in range(hes.ZERO_RETURN_CONFIRM_SAMPLES)])

    assert node.commanded == [[0.0] * 5], (
        '%d motion(s) were commanded after the signal: %r'
        % (len(node.commanded), node.commanded))
    assert code != 0


def test_signal_before_a_repeatability_run_moves_nothing(tmp_path):
    args = _return_args(3, hes.PURPOSE_DIAGNOSTIC, str(tmp_path / 'r.json'))
    node = _ReturnNode([_measurement(2.0, 0.2)])
    node.interrupted.update(flag=True, signal=signal.SIGINT)

    with pytest.raises(hes.SessionInterrupted):
        hes.run_return_repeatability(node, args)
    assert node.reference_calls == 0 and node.measure_calls == 0
    assert not os.path.exists(str(tmp_path / 'r.json'))


def test_signal_mid_repeatability_stops_after_the_current_cycle(tmp_path):
    path = str(tmp_path / 'r.json')
    args = _return_args(8, hes.PURPOSE_CALIBRATION, path)
    node = _ReturnNode([_measurement(2.0, 0.2), _measurement(3.0, 0.3)],
                       interrupt_after=1)

    code = hes.run_return_repeatability(node, args)
    assert code == 1
    assert node.measure_calls == 1, 'a cycle ran after the signal'
    written = json.loads(open(path).read())
    assert written['cycles_measured'] == 1        # journalled, not lost
    assert written['complete'] is False
    assert written['usable_for_gate'] is False


@pytest.mark.parametrize('inner_code,delivered,expected', [
    (0, True, 0),
    (1, True, 1),
    (0, False, 3),      # the run said fine; the arm never came home
    (1, False, 3),
])
def test_measure_return_exit_code_reflects_the_zero_return(
        monkeypatch, restore_signals, tmp_path, inner_code, delivered,
        expected):
    """The early `return` froze this at whatever the run reported."""
    argv = ['--measure-return', '3', '--return-out', str(tmp_path / 'r.json')]
    _args, node = _motion_node(monkeypatch, argv)
    node.spin_for = lambda _seconds: None
    monkeypatch.setattr(hes, 'run_return_repeatability',
                        lambda *a, **k: inner_code)
    echo = [0.0] * 5 if delivered else [0.20] * 5
    node._on_joint_states(_joint_state(echo, stamp=10))         # live barrier
    code = _run_main(monkeypatch, node, argv,
                     [_joint_state(echo, stamp=11 + n)
                      for n in range(hes.ZERO_RETURN_CONFIRM_SAMPLES)])
    assert code == expected


# -- a diagnostic run must not offer a command the loader will refuse -------

class _ReturnNode(object):
    """A session whose board measurements are canned, motion path removed."""

    stop_requested = hes.HandEyeSession.stop_requested
    raise_if_interrupted = hes.HandEyeSession.raise_if_interrupted

    def __init__(self, measurements, interrupt_after=None):
        self.interrupted = {'flag': False, 'signal': None}
        self.measurements = list(measurements)
        self.interrupt_after = interrupt_after
        self.reference_calls = 0
        self.measure_calls = 0
        self.zero_ref = None

    def set_zero_reference(self):
        self.reference_calls += 1
        self.zero_ref = dict(_REFERENCE)

    def measure_zero(self):
        self.measure_calls += 1
        if (self.interrupt_after is not None
                and self.measure_calls >= self.interrupt_after):
            self.interrupted.update(flag=True, signal=signal.SIGINT)
        return dict(self.measurements.pop(0)), 'ok'


def _measurement(drift_mm, drift_deg):
    pose = _measured_pose(drift_mm, drift_deg)
    return dict(pose, internal_spread_mm=0.1, internal_spread_deg=0.01)


def _return_args(cycles, purpose, out):
    parser = hes.build_parser()
    args = parser.parse_args(['--measure-return', str(cycles),
                              '--purpose', purpose, '--return-out', out])
    hes.validate_args(parser, args, hes.DEFAULT_POSES)
    return args


_THREE_CYCLES = [(2.0, 0.20), (3.0, 0.30), (1.0, 0.10)]


def test_calibration_run_offers_the_gate_command_and_the_file_loads(
        capsys, tmp_path):
    """Writer to disk to reader, through the real cycle loop."""
    path = str(tmp_path / 'r.json')
    args = _return_args(3, hes.PURPOSE_CALIBRATION, path)
    node = _ReturnNode([_measurement(mm, deg) for mm, deg in _THREE_CYCLES])

    assert hes.run_return_repeatability(node, args) == 0
    assert '--gate-from %s' % path in capsys.readouterr().out

    gate_mm, gate_deg, summary = hes.load_gate_artifact(path)
    assert gate_mm == pytest.approx(4.5)
    assert gate_deg == pytest.approx(0.45)
    assert summary['cycles_measured'] == 3


def test_diagnostic_run_does_not_offer_a_command_that_will_be_refused(
        capsys, tmp_path):
    path = str(tmp_path / 'r.json')
    args = _return_args(3, hes.PURPOSE_DIAGNOSTIC, path)
    node = _ReturnNode([_measurement(mm, deg) for mm, deg in _THREE_CYCLES])

    assert hes.run_return_repeatability(node, args) == 0
    output = capsys.readouterr().out
    # It may NAME the flag to explain the refusal; it must not hand over a
    # runnable command, which is what reads as a tool bug when it is rejected.
    assert '--gate-from %s' % path not in output
    assert 'CANNOT set a gate' in output
    # And the refusal it warns about is real.
    with pytest.raises(ValueError):
        hes.load_gate_artifact(path)
