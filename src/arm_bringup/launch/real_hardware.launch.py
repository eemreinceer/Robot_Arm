"""Robot Arm gerçek donanım bringup — Faz 6 Aşama C.

Jetson Nano (Humble) hedefi; Gazebo YOK. Zincir:
  robot_state_publisher + ros2_control_node (arm_hardware/STM32SystemInterface)
  + spawner'lar + move_group + robot_arm_pick_place_node.

M3 kabulü: `mock_serial:=true` ile STM32'siz çökmeden kalkar (hardware interface
plugin'i gerektirir). Gerçek koşu: serial_device/baud argümanlarıyla.
RViz default KAPALI (use_rviz:=true ile laptop'tan).
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_file(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    with open(os.path.join(package_path, relative_path), "r", encoding="utf-8") as f:
        return f.read()


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    with open(os.path.join(package_path, relative_path), "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    description_share = get_package_share_directory("robot_arm_description")

    serial_device = LaunchConfiguration("serial_device")
    baud_rate = LaunchConfiguration("baud_rate")
    mock_serial = LaunchConfiguration("mock_serial")
    use_rviz = LaunchConfiguration("use_rviz")

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"), " ",
                PathJoinSubstitution([
                    FindPackageShare("robot_arm_description"), "urdf",
                    "robot_arm_real.urdf.xacro",
                ]),
                " serial_device:=", serial_device,
                " baud_rate:=", baud_rate,
                " mock_serial:=", mock_serial,
            ]),
            value_type=str,
        )
    }

    controllers_yaml = os.path.join(
        description_share, "config", "robot_arm_controllers_real.yaml")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": False}],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controllers_yaml, {"use_sim_time": False}],
    )

    def spawner(controller):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller,
                       "--controller-manager", "/controller_manager",
                       "--controller-manager-timeout", "60"],
            output="screen",
        )

    # move_group — sim launch'ı DEĞİL: use_sim_time false, gerçek (muhafazakâr)
    # joint limitleri, sim'in yavaş-RTF toleransları YOK (open-loop echo state
    # komutu anında izler; default toleranslar yeterli).
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            {"robot_description_semantic": load_file(
                "robot_arm_moveit_config", "config/robot_arm.srdf")},
            {"robot_description_kinematics": load_yaml(
                "robot_arm_moveit_config", "config/kinematics.yaml")},
            {"robot_description_planning": load_yaml(
                "robot_arm_moveit_config", "config/joint_limits_real.yaml")},
            load_yaml("robot_arm_moveit_config", "config/ompl_planning.yaml"),
            load_yaml("robot_arm_moveit_config", "config/moveit_controllers.yaml"),
            {"use_sim_time": False},
        ],
    )

    pick_place_node = Node(
        package="arm_nodes",
        executable="robot_arm_pick_place_node",
        output="screen",
        parameters=[
            robot_description,
            {"robot_description_semantic": load_file(
                "robot_arm_moveit_config", "config/robot_arm.srdf")},
            {"robot_description_kinematics": load_yaml(
                "robot_arm_moveit_config", "config/kinematics.yaml")},
            {"robot_description_planning": load_yaml(
                "robot_arm_moveit_config", "config/joint_limits_real.yaml")},
            {"use_sim_time": False},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        parameters=[robot_description, {"use_sim_time": False}],
    )

    return LaunchDescription([
        # ESP32 firmware'i 2026-08-19'dan itibaren UART0 (USB/CP2102) uzerinden
        # konusuyor, GPIO16/17 uzerinden degil; host tarafinda GPIO/header
        # kablolamasi gerekmiyor, USB kablosu takili port bu. Bkz.
        # firmware/esp32_servo_ctrl/README.md "Hardware and transport".
        DeclareLaunchArgument("serial_device", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baud_rate", default_value="115200"),
        DeclareLaunchArgument("mock_serial", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        robot_state_publisher,
        ros2_control_node,
        spawner("joint_state_broadcaster"),
        spawner("robot_arm_controller"),
        spawner("robot_arm_gripper_controller"),
        move_group_node,
        pick_place_node,
        rviz,
    ])
