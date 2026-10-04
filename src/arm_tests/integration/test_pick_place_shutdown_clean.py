"""Regression: robot_arm_pick_place_node must not leak plugin objects at shutdown.

WHAT THIS PROVES
  Before the object-lifetime fix, `main` called `rclcpp::shutdown()` while the
  node and its `MoveGroupInterface` were still alive. The MoveIt plugins those
  objects held were therefore still on the heap when class_loader unloaded the
  library, and class_loader printed:

      SEVERE WARNING!!! Attempting to unload library while objects created by
      this loader exist in the heap!

  The process still exited 0, so an exit-code-only assertion cannot see this.
  This test watches the *output* instead, which is the only place the defect
  is visible.

WHY A SEPARATE LAUNCH TEST
  The M3 smoke test owns the process-exit contract (Issue #14) and is under
  active edit. Keeping this narrow regression in its own file avoids collisions
  and lets it state one thing only. It can be folded into M3 later.

DEBUGINFOD
  Pinned empty: with it populated, a crashing `move_group` blocks on the
  network during teardown and this launch never reaches its post-shutdown
  phase. See docs/m3_shutdown_policy.md.

SAFE: mock_serial only. No robot, UART, servo rail, PWM, flash or simulation.
"""

import os
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

CLASS_LOADER_LEAK = "SEVERE WARNING"
NODE = "robot_arm_pick_place_node"


@pytest.mark.launch_test
def generate_test_description():
    os.environ["DEBUGINFOD_URLS"] = ""
    bringup = os.path.join(
        get_package_share_directory("arm_bringup"), "launch", "real_hardware.launch.py"
    )
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments={"mock_serial": "true"}.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestPickPlaceComesUp(unittest.TestCase):

    def test_node_becomes_ready(self, proc_info, proc_output):
        proc_info.assertWaitForStartup(process=NODE, timeout=30.0)
        # Shut down only after the node has actually built its
        # MoveGroupInterface. Signalling earlier tears down a half-built node
        # and produces SIGINT-death noise instead of the defect under test.
        proc_output.assertWaitFor(
            "Robot Arm pick/place action ready on /pick_and_place", timeout=60.0
        )
        time.sleep(3.0)


@launch_testing.post_shutdown_test()
class TestPickPlaceShutdownIsClean(unittest.TestCase):

    def test_no_class_loader_leak_warning(self, proc_output):
        """The node must delete its plugin-owning objects before unload."""
        leaked = []
        for line in proc_output:
            text = line.text.decode(errors="replace")
            name = str(getattr(line, "process_name", ""))
            if CLASS_LOADER_LEAK in text and NODE in name:
                leaked.append(f"[{name}] {text}")
        self.assertEqual(
            leaked,
            [],
            "robot_arm_pick_place_node still holds plugin objects when class_loader "
            "unloads the library. Expected the node, its MoveGroupInterface and "
            "its action clients to be destroyed before the ROS context closes.\n"
            + "\n".join(leaked),
        )

    def test_node_still_exits_zero(self, proc_info):
        """The lifetime fix must not trade a warning for a nonzero exit."""
        launch_testing.asserts.assertExitCodes(proc_info, process=NODE)
