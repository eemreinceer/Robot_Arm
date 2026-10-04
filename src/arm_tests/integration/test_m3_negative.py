"""Prove the production M3 predicates catch lifecycle failures and enforce allowlists."""

import os
import re
import subprocess
import sys
from pathlib import Path
from unittest.mock import patch

import pytest

sys.path.insert(0, os.path.dirname(__file__))
from m3_process_assertions import (  # noqa: E402
    assert_processes_shutdown_cleanly,
    verify_active_controllers,
)

FIXTURE = Path(__file__).with_name("m3_bad_launch_fixture.py")


class DummyProcessInfo:

    def __init__(self, process_name, returncode, pid=12345):
        self.process_name = process_name
        self.returncode = returncode
        self.pid = pid


class DummyProcInfoHandler:

    def __init__(self, processes):
        self._processes = processes

    def processes(self):
        return self._processes

    def __getitem__(self, item):
        for p in self._processes:
            if getattr(p, "process_name", None) == item:
                return p
        # For mock tests, we only look up by the dummy object itself most of the time
        return item


@patch("m3_process_assertions.get_moveit_teardown_exception_status")
def test_shutdown_assertion_passes_on_all_zero(mock_status):
    mock_status.return_value = (False, "Mismatched version")
    handler = DummyProcInfoHandler([
        DummyProcessInfo("ros2_control_node-1", 0),
        DummyProcessInfo("robot_state_publisher-2", 0),
        DummyProcessInfo("move_group-3", 0),
        DummyProcessInfo("robot_arm_pick_place_node-4", 0),
        DummyProcessInfo("spawner-5", 0),
    ])
    assert_processes_shutdown_cleanly(handler)


@patch("m3_process_assertions.get_moveit_teardown_exception_status")
def test_shutdown_assertion_allows_moveit_sigsegv_when_version_matches(mock_status):
    mock_status.return_value = (True, "MOVEIT_JAZZY_2_12_4_TEARDOWN_SIGSEGV")
    handler = DummyProcInfoHandler([
        DummyProcessInfo("ros2_control_node-1", 0),
        DummyProcessInfo("robot_state_publisher-2", 0),
        DummyProcessInfo("robot_arm_pick_place_node-4", 0),
        DummyProcessInfo("move_group-2", -11),
    ])
    # When current version exactly matches, this should pass
    assert_processes_shutdown_cleanly(handler)


@patch("m3_process_assertions.get_moveit_teardown_exception_status")
def test_shutdown_assertion_rejects_moveit_sigsegv_when_version_mismatched(mock_status):
    mock_status.return_value = (False, "Debian version mismatch")
    handler = DummyProcInfoHandler([
        DummyProcessInfo("ros2_control_node-1", 0),
        DummyProcessInfo("robot_state_publisher-2", 0),
        DummyProcessInfo("robot_arm_pick_place_node-4", 0),
        DummyProcessInfo("move_group-2", -11),
    ])
    with pytest.raises(AssertionError, match="unauthorized code -11"):
        assert_processes_shutdown_cleanly(handler)


@patch("m3_process_assertions.get_moveit_teardown_exception_status")
def test_shutdown_assertion_rejects_moveit_sigterm_always(mock_status):
    # Even if version matches, -15 is rejected because only -11 is allowlisted now
    mock_status.return_value = (True, "MOVEIT_JAZZY_2_12_4_TEARDOWN_SIGSEGV")
    handler = DummyProcInfoHandler([
        DummyProcessInfo("ros2_control_node-1", 0),
        DummyProcessInfo("robot_state_publisher-2", 0),
        DummyProcessInfo("robot_arm_pick_place_node-4", 0),
        DummyProcessInfo("move_group-2", -15),
    ])
    with pytest.raises(AssertionError, match="non-zero code -15"):
        assert_processes_shutdown_cleanly(handler)


def test_shutdown_assertion_rejects_unauthorized_sigterm_on_other_processes():
    handler = DummyProcInfoHandler([
        DummyProcessInfo("ros2_control_node-1", -15),
        DummyProcessInfo("robot_state_publisher-2", 0),
        DummyProcessInfo("robot_arm_pick_place_node-4", 0),
        DummyProcessInfo("move_group-2", 0),
    ])
    with pytest.raises(AssertionError, match="Process ros2_control_node-1.*non-zero code -15"):
        assert_processes_shutdown_cleanly(handler)


def test_shutdown_assertion_rejects_process_crash():
    handler = DummyProcInfoHandler([
        DummyProcessInfo("ros2_control_node-1", 0),
        DummyProcessInfo("robot_state_publisher-2", 0),
        DummyProcessInfo("move_group-2", 0),
        DummyProcessInfo("robot_arm_pick_place_node-1", 1),
    ])
    with pytest.raises(AssertionError, match="Process robot_arm_pick_place_node-1.*non-zero code 1"):
        assert_processes_shutdown_cleanly(handler)


def test_shutdown_assertion_rejects_missing_required_processes():
    handler = DummyProcInfoHandler([
        DummyProcessInfo("ros2_control_node-1", 0),
        DummyProcessInfo("robot_state_publisher-2", 0),
        DummyProcessInfo("move_group-2", 0),
        # missing robot_arm_pick_place_node
    ])
    expected = "Missing required processes in teardown verification: "
    with pytest.raises(AssertionError, match=re.escape(expected + "['robot_arm_pick_place_node']")):
        assert_processes_shutdown_cleanly(handler)


def test_verify_active_controllers_rejects_missing_service():
    with pytest.raises(AssertionError, match="Service /controller_manager/list_controllers not available"):
        verify_active_controllers(timeout_seconds=0.3)


@pytest.mark.parametrize(
    ("case", "expected"),
    [
        (
            "early_exit_zero",
            "mandatory process exited inside alive window: ros2_control_node",
        ),
        ("missing", "mandatory process missing: ros2_control_node"),
        ("unauthorized_sigterm", "Process ros2_control_node"),
        ("moveit_sigterm", "non-zero code -15"),
    ],
)
def test_real_launch_testing_handler_rejects_bad_fixture(case, expected, tmp_path):
    env = os.environ.copy()
    env["M3_NEGATIVE_CASE"] = case
    result = subprocess.run(
        [
            "launch_test",
            str(FIXTURE),
            "--junit-xml",
            str(tmp_path / f"{case}.xml"),
        ],
        capture_output=True,
        check=False,
        env=env,
        text=True,
        timeout=25,
    )
    combined = result.stdout + result.stderr
    assert result.returncode != 0, combined
    expected_test = (
        "test_all_processes_exit_cleanly"
        if case in ("unauthorized_sigterm", "moveit_sigterm")
        else "test_shared_liveness_contract_rejects_bad_launch"
    )
    assert expected_test in combined
    assert expected in combined
