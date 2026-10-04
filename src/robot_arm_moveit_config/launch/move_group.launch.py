import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def load_file(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    with open(os.path.join(package_path, relative_path), "r", encoding="utf-8") as f:
        return f.read()


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    with open(os.path.join(package_path, relative_path), "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description_content = Command([
        FindExecutable(name="xacro"),
        " ",
        PathJoinSubstitution([
            FindPackageShare("robot_arm_description"),
            "urdf",
            "robot_arm.urdf.xacro",
        ]),
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            {"robot_description_semantic": load_file("robot_arm_moveit_config", "config/robot_arm.srdf")},
            {"robot_description_kinematics": load_yaml("robot_arm_moveit_config", "config/kinematics.yaml")},
            {"robot_description_planning": load_yaml("robot_arm_moveit_config", "config/joint_limits.yaml")},
            load_yaml("robot_arm_moveit_config", "config/ompl_planning.yaml"),
            load_yaml("robot_arm_moveit_config", "config/moveit_controllers.yaml"),
            {"use_sim_time": use_sim_time},
            # Ana koldan taşınan yavaş-RTF sim toleransları
            {
                "trajectory_execution.allowed_start_tolerance": 0.3,
                "trajectory_execution.execution_duration_monitoring": False,
                "trajectory_execution.allowed_execution_duration_scaling": 5.0,
                "trajectory_execution.allowed_goal_duration_margin": 30.0,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        move_group_node,
    ])
