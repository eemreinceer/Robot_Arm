"""Regression tests for the IMX219 first-frame watchdog integration."""

from unittest.mock import patch

import pytest

from arm_perception.csi_camera_node import CsiCameraNode


class _Logger:
    def __init__(self):
        self.errors = []

    def error(self, message):
        self.errors.append(message)


def test_watchdog_exits_with_code_one_after_deadline():
    node = object.__new__(CsiCameraNode)
    node.first_frame_timeout_s = 15.0
    node.started_at = 100.0
    node.frames_published = 0
    node.backend = 'libcamera'
    logger = _Logger()
    node.get_logger = lambda: logger

    with patch(
            'arm_perception.csi_camera_node.time.monotonic',
            return_value=115.1):
        with pytest.raises(SystemExit) as error:
            node._check_frame_watchdog()

    assert error.value.code == 1
    assert len(logger.errors) == 1
    assert 'TEK KARE gelmedi' in logger.errors[0]


@pytest.mark.parametrize('timeout,elapsed,frames', [
    (15.0, 14.9, 0),
    (0.0, 120.0, 0),
    (15.0, 120.0, 1),
])
def test_watchdog_stays_disabled_or_armed_until_failure(
        timeout, elapsed, frames):
    node = object.__new__(CsiCameraNode)
    node.first_frame_timeout_s = timeout
    node.started_at = 100.0
    node.frames_published = frames
    node.backend = 'libcamera'
    logger = _Logger()
    node.get_logger = lambda: logger

    with patch(
            'arm_perception.csi_camera_node.time.monotonic',
            return_value=100.0 + elapsed):
        node._check_frame_watchdog()

    assert logger.errors == []
