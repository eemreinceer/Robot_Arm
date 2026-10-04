"""Minimal ros2_control stack pointed at the fake ESP32 pty.

Only the controller manager -- no MoveIt, no pick/place. The question under test
is purely what ros2_control does when write() returns ERROR.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    device = os.environ["FAKE_TTY"]
    share = get_package_share_directory("robot_arm_description")
    xacro_path = os.path.join(share, "urdf", "robot_arm_real.urdf.xacro")
    controllers = os.path.join(share, "config", "robot_arm_controllers_real.yaml")

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"), " ", xacro_path,
                " serial_device:=", device,
                " mock_serial:=false",
            ]),
            value_type=str,
        )
    }

    return LaunchDescription([
        # Jazzy's controller_manager takes robot_description from the topic, so
        # the state publisher has to be here even though this test never looks
        # at TF.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="log",
            parameters=[robot_description, {"use_sim_time": False}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[robot_description, controllers, {"use_sim_time": False}],
        ),
    ])
