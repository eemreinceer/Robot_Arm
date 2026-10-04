"""Deliberately failing launch fixture, executed only by the M3 meta-test."""

import os
import sys
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess

import launch_testing.actions
import launch_testing.asserts

sys.path.insert(0, os.path.dirname(__file__))
from m3_process_assertions import (  # noqa: E402
    assert_processes_shutdown_cleanly,
    assert_required_processes_live,
)

CASE = os.environ["M3_NEGATIVE_CASE"]

# Ignore SIGINT so launch must escalate to SIGTERM: that is the negative signal.
IGNORE_SIGINT = (
    "import signal,time; "
    "signal.signal(signal.SIGINT, signal.SIG_IGN); "
    "time.sleep(10)"
)


def generate_test_description():
    actions = []

    # We must spawn all 4 required processes to pass the missing-process check
    # during teardown, so we can test the exit code assertions.
    for p in ("ros2_control_node", "robot_state_publisher", "move_group", "robot_arm_pick_place_node"):
        if CASE == "missing" and p == "ros2_control_node":
            continue

        if CASE == "early_exit_zero" and p == "ros2_control_node":
            cmd = [sys.executable, "-c", "pass"]
        elif CASE in ("forced_sigterm", "unauthorized_sigterm") and p == "ros2_control_node":
            cmd = [sys.executable, "-c", IGNORE_SIGINT]
        elif CASE == "moveit_sigterm" and p == "move_group":
            cmd = [sys.executable, "-c", IGNORE_SIGINT]
        else:
            cmd = [sys.executable, "-c", "import time; time.sleep(10)"]

        actions.append(ExecuteProcess(
            cmd=cmd,
            name=p,
            sigterm_timeout="0.2",
        ))

    actions.append(launch_testing.actions.ReadyToTest())
    return LaunchDescription(actions)


class TestDeliberatelyBadM3Launch(unittest.TestCase):

    def test_shared_liveness_contract_rejects_bad_launch(self, proc_info):
        process = "move_group" if CASE == "moveit_sigterm" else "ros2_control_node"
        assert_required_processes_live(
            proc_info,
            (process,),
            window_seconds=0.2,
            startup_timeout=2.0,
        )


@launch_testing.post_shutdown_test()
class TestDeliberatelyBadM3Shutdown(unittest.TestCase):

    def test_all_processes_exit_cleanly(self, proc_info):
        # We always call with default arguments now, and the mocked status or
        # actual system state handles the policy. Since this fixture simulates -15,
        # the new assertion should inherently reject it.
        assert_processes_shutdown_cleanly(proc_info)
