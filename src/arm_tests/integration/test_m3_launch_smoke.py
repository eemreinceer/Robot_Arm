"""M3: the canonical real-hardware bringup stays alive in mock-serial mode."""

import os
import sys
import unittest

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch_testing
import launch_testing.actions
import launch_testing.asserts

sys.path.insert(0, os.path.dirname(__file__))
from m3_process_assertions import (  # noqa: E402
    REQUIRED_LONG_RUNNING_PROCESSES,
    assert_processes_shutdown_cleanly,
    assert_required_processes_live,
    verify_active_controllers,
)


# Precondition, not a preference. move_group segfaults during teardown; with
# DEBUGINFOD_URLS populated, backward_ros' crash handler blocks in libdw
# talking to debuginfod.ubuntu.com, the crash never surfaces as -11, and the
# launch is SIGTERM-escalated to -15 instead. CMake pins this for `colcon
# test`; this guard catches direct `launch_test` invocations.
if os.environ.get("DEBUGINFOD_URLS", "") != "":
    raise AssertionError(
        "DEBUGINFOD_URLS must be empty: it masks the move_group teardown "
        "segfault as an unbounded hang (Issue #14). "
        'Run: export DEBUGINFOD_URLS=""'
    )

os.environ["DEBUGINFOD_URLS"] = ""


def generate_test_description():
    launch_path = os.path.join(
        get_package_share_directory("arm_bringup"),
        "launch",
        "real_hardware.launch.py",
    )
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments={"mock_serial": "true", "use_rviz": "false"}.items(),
    )
    return LaunchDescription([
        bringup,
        launch_testing.actions.ReadyToTest(),
    ])


class TestM3MockBringup(unittest.TestCase):

    def test_mandatory_processes_and_controllers_active(self, proc_info):
        # 1. Verify active controllers through ROS 2 service call
        active_controllers = verify_active_controllers(timeout_seconds=30.0)
        self.assertIsNotNone(active_controllers)

        # 2. Verify all 4 long-running processes survive bounded liveness window
        assert_required_processes_live(
            proc_info,
            REQUIRED_LONG_RUNNING_PROCESSES,
            window_seconds=3.0,
            startup_timeout=15.0,
        )


@launch_testing.post_shutdown_test()
class TestM3MockBringupShutdown(unittest.TestCase):

    def test_mandatory_processes_exit_cleanly(self, proc_info):
        assert_processes_shutdown_cleanly(
            proc_info,
            required_processes=REQUIRED_LONG_RUNNING_PROCESSES,
        )
