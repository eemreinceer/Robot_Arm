"""Complete Robot Arm simulation: Gazebo + MoveIt + YOLO + autonomous sorting."""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def load_file(package_share, relative_path):
    with open(os.path.join(package_share, relative_path), "r", encoding="utf-8") as stream:
        return stream.read()


def load_yaml(package_share, relative_path):
    with open(os.path.join(package_share, relative_path), "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    description_share = get_package_share_directory("robot_arm_description")
    moveit_share = get_package_share_directory("robot_arm_moveit_config")
    perception_share = get_package_share_directory("arm_perception")
    robot_description = ParameterValue(
        Command([
            "xacro ", os.path.join(description_share, "urdf", "robot_arm.urdf.xacro")
        ]),
        value_type=str,
    )
    moveit_params = {
        "robot_description": robot_description,
        "robot_description_semantic": load_file(
            moveit_share, "config/robot_arm.srdf"),
        "robot_description_kinematics": load_yaml(
            moveit_share, "config/kinematics.yaml"),
        "robot_description_planning": load_yaml(
            moveit_share, "config/joint_limits.yaml"),
        "use_sim_time": use_sim_time,
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "headless": headless,
            "world": "robot_arm_pick.world",
        }.items(),
    )

    move_group = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_share, "launch", "move_group.launch.py")
            ),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        )],
    )

    pick_place = TimerAction(
        period=12.0,
        actions=[Node(
            package="arm_nodes",
            executable="robot_arm_pick_place_node",
            output="screen",
            parameters=[moveit_params, {
                # SİM: gz-sim mimic desteklemez → ikinci parmak ayrı sürülür.
                "gripper_mirror_joint": "joint_6_mirror",
                # Gerçek CAD parmak (dişli-gripper split) stroke — render doğrulandı.
                "gripper_open": 0.2,
                "gripper_closed": -0.12,
            }],
        )],
    )

    perception = TimerAction(
        period=15.0,
        actions=[Node(
            package="arm_perception",
            executable="perception_node",
            name="perception_node",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "model_path": os.path.join(perception_share, "models", "yolo_arm.pt"),
                "image_topic": "/camera/image",
                "camera_info_topic": "/camera/camera_info",
                "points_topic": "/camera/points",
                "detections_topic": "/detected_objects",
                "target_frame": "base_link",
                "pre_pick_offset_m": 0.07,
                "link6_to_grasp_xyz": [0.0, 0.0, 0.0],
                "link6_to_grasp_rpy": [0.0, 0.0, 0.0],
                "grasp_orientations_file": "/dev/null",
                "top_grasp_link6_quaternion": [1.0, 0.0, 0.0, 0.0],
            }],
        )],
    )

    autonomous = TimerAction(
        period=19.0,
        actions=[Node(
            package="arm_perception",
            executable="autonomous_pick_node",
            name="autonomous_pick_node",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "sort_all": True,
                "sorting_bins_file": os.path.join(
                    perception_share, "config", "robot_arm_sorting_bins.yaml"),
                "simulation_world": "robot_arm_pick_world",
                "simulation_sync_enabled": True,
                "simulation_post_place_sync_enabled": True,
                "simulation_place_hover_offset_m": 0.04,
                "simulation_objects_per_class": 2,
                "simulation_exact_pose_enabled": True,
                "base_world_yaw_rad": 0.0,
                "pick_descent_droop_compensation_m": 0.0,
                "bin_floor_thickness_m": 0.008,
                "pre_pick_offset_m": 0.07,
                "pre_place_offset_m": 0.06,
                "link6_to_grasp_xyz": [0.0, 0.0, 0.0],
                "grasp_orientations_file": "/dev/null",
                "top_grasp_link6_quaternion": [1.0, 0.0, 0.0, 0.0],
            }],
        )],
    )

    scene = TimerAction(
        period=23.0,
        actions=[ExecuteProcess(
            cmd=[
                "ros2", "run", "arm_perception", "demo_sorting",
                "--config", os.path.join(perception_share, "config", "robot_arm_sorting_bins.yaml"),
                "--world", "robot_arm_pick_world",
                "--objects-per-class", "2",
                "--seed", "7",
                "--skip-ik-validation",
                "--object-timeout", "480",
            ],
            output="screen",
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false"),
        gazebo,
        move_group,
        pick_place,
        perception,
        autonomous,
        scene,
    ])
