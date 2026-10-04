import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    hardware_share = get_package_share_directory('arm_hardware')
    xacro_file = os.path.join(hardware_share, 'test', 'mock_system.urdf.xacro')
    calibration_file = os.path.join(hardware_share, 'test', 'mock_calibration.yaml')
    controller_file = os.path.join(hardware_share, 'test', 'mock_controllers.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' calibration_file:=', calibration_file]),
        value_type=str,
    )

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[{'robot_description': robot_description}, controller_file],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
    ])
