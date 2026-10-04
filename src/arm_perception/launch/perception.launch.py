#!/usr/bin/env python3
"""Launch the YOLO RGB-D perception node and optional autonomous pick loop."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model_path = LaunchConfiguration("model_path")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autonomous = LaunchConfiguration("autonomous")
    sort_all = LaunchConfiguration("sort_all")
    sorting_bins_file = LaunchConfiguration("sorting_bins_file")
    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value=PathJoinSubstitution([FindPackageShare("arm_perception"), "models", "yolo_arm.pt"])),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("autonomous", default_value="false"),
        DeclareLaunchArgument("sort_all", default_value="false"),
        DeclareLaunchArgument(
            "sorting_bins_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("arm_perception"), "config", "sorting_bins.yaml"
            ]),
        ),
        Node(
            package="arm_perception", executable="perception_node", name="perception_node",
            output="screen", parameters=[{
                "model_path": model_path, "use_sim_time": use_sim_time,
                "image_topic": "/camera/image", "camera_info_topic": "/camera/camera_info",
                "points_topic": "/camera/points", "detections_topic": "/detected_objects",
                "target_frame": "base_link",
            }],
        ),
        Node(
            package="arm_perception", executable="autonomous_pick_node", name="autonomous_pick_node",
            output="screen", condition=IfCondition(autonomous),
            parameters=[{
                "use_sim_time": use_sim_time,
                "sort_all": sort_all,
                "sorting_bins_file": sorting_bins_file,
            }],
        ),
    ])
