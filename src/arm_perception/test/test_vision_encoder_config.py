import importlib.util
from pathlib import Path
from types import SimpleNamespace

from arm_perception.board_pnp import (
    DEFAULT_BOARD_COLS,
    DEFAULT_BOARD_ROWS,
    DEFAULT_SQUARE_SIZE_MM,
)
from launch.actions import EmitEvent, TimerAction
import numpy as np
import pytest
from std_msgs.msg import Header
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _load_camera_launch():
    path = PACKAGE_ROOT / 'launch' / 'imx219_camera.launch.py'
    spec = importlib.util.spec_from_file_location('imx219_camera_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_canonical_vision_encoder_config_matches_safe_fallback_defaults():
    with (PACKAGE_ROOT / 'config' / 'vision_encoder.yaml').open() as handle:
        params = yaml.safe_load(handle)['vision_encoder_node']['ros__parameters']

    assert params['board_cols'] == DEFAULT_BOARD_COLS == 6
    assert params['board_rows'] == DEFAULT_BOARD_ROWS == 9
    assert params['square_size_mm'] == pytest.approx(
        DEFAULT_SQUARE_SIZE_MM)
    assert params['square_size_mm'] == pytest.approx(27.5)
    assert params['observation_topic'] == '/vision_encoder/observation'


def _observation_builder():
    from arm_perception.vision_encoder_node import build_board_observation
    return build_board_observation


def _image_header():
    header = Header()
    header.stamp.sec = 123
    header.stamp.nanosec = 456
    header.frame_id = 'camera_message_frame'
    return header


def test_atomic_observation_preserves_image_stamp_geometry_and_pose():
    result = {
        'rvec': np.zeros((3, 1), dtype=np.float64),
        'tvec': np.array([[0.1], [-0.2], [0.3]], dtype=np.float64),
        'reproj_px': 0.42,
        'n_corners': 54,
    }

    observation = _observation_builder()(
        _image_header(), 'camera_optical_frame', result, True, 6, 9, 27.5)

    assert observation.header.stamp.sec == 123
    assert observation.header.stamp.nanosec == 456
    assert observation.header.frame_id == 'camera_optical_frame'
    assert observation.detected is True
    assert observation.reprojection_px == pytest.approx(0.42)
    assert observation.pose.position.x == pytest.approx(0.1)
    assert observation.pose.position.y == pytest.approx(-0.2)
    assert observation.pose.position.z == pytest.approx(0.3)
    assert observation.pose.orientation.w == pytest.approx(1.0)
    assert observation.corners_detected == 54
    assert observation.corners_expected == 54
    assert observation.board_cols == 6
    assert observation.board_rows == 9
    assert observation.square_size_mm == pytest.approx(27.5)


def test_rejected_observation_does_not_expose_pose_or_reprojection():
    rejected_result = {
        'rvec': np.array([[0.2], [0.1], [-0.1]], dtype=np.float64),
        'tvec': np.array([[1.0], [2.0], [3.0]], dtype=np.float64),
        'reproj_px': 9.5,
        'n_corners': 54,
    }

    observation = _observation_builder()(
        _image_header(), 'camera_optical_frame', rejected_result, False,
        6, 9, 27.5)

    assert observation.detected is False
    assert observation.reprojection_px == 0.0
    assert observation.pose.position.x == 0.0
    # geometry_msgs/Pose has a valid identity quaternion as its ROS default;
    # the rejected source rotation must not replace it.
    assert observation.pose.orientation.x == 0.0
    assert observation.pose.orientation.y == 0.0
    assert observation.pose.orientation.z == 0.0
    assert observation.pose.orientation.w == 1.0
    assert observation.corners_detected == 54
    assert observation.corners_expected == 54


def test_camera_restart_budget_is_bounded():
    launch_module = _load_camera_launch()
    state = launch_module._RetryState(max_restarts=2, delay_s=5.0)

    assert state.begin_attempt() == 1
    assert state.can_restart()
    assert state.begin_attempt() == 2
    assert state.can_restart()
    assert state.begin_attempt() == 3
    assert not state.can_restart()


def test_camera_exit_schedules_delay_then_shuts_down_when_budget_exhausted():
    launch_module = _load_camera_launch()
    context = SimpleNamespace(is_shutdown=False)
    event = SimpleNamespace(returncode=1)
    state = launch_module._RetryState(max_restarts=1, delay_s=3.0)

    state.begin_attempt()
    retry_actions = launch_module._camera_exited(event, context, state)
    assert len(retry_actions) == 1
    assert isinstance(retry_actions[0], TimerAction)

    state.begin_attempt()
    exhausted_actions = launch_module._camera_exited(event, context, state)
    assert len(exhausted_actions) == 1
    assert isinstance(exhausted_actions[0], EmitEvent)


def test_camera_exit_never_restarts_during_launch_shutdown():
    launch_module = _load_camera_launch()
    context = SimpleNamespace(is_shutdown=True)
    event = SimpleNamespace(returncode=-2)
    state = launch_module._RetryState(max_restarts=2, delay_s=5.0, starts=1)

    assert launch_module._camera_exited(event, context, state) == []


@pytest.mark.parametrize(
    'parser,value',
    [('_non_negative_int', '-1'), ('_non_negative_int', 'x'),
     ('_non_negative_float', '-0.1'), ('_non_negative_float', 'x')])
def test_camera_retry_policy_rejects_invalid_values(parser, value):
    launch_module = _load_camera_launch()
    with pytest.raises(RuntimeError):
        getattr(launch_module, parser)(value, 'test_value')
