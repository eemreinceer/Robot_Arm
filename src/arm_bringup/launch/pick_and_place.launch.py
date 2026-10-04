"""
Full system launch: Gazebo + MoveIt2 + pick_place_node + RViz.
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ── arguments ────────────────────────────────────────────────────────────
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz for visualization",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock",
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="pick_and_place.world",
        description="arm_gazebo/worlds world dosya adı (sorting: sorting.world)",
    )

    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")

    # ── package share paths ───────────────────────────────────────────────────
    arm_moveit_config_share = FindPackageShare("arm_moveit_config")
    arm_bringup_share = FindPackageShare("arm_bringup")

    # ── robot description + semantic (SRDF) — shared by move_group, RViz, pick_place_node ──
    xacro_file = os.path.join(
        get_package_share_directory("arm_description"), "urdf", "arm.urdf.xacro"
    )
    robot_description_content = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file]),
        value_type=str,
    )

    srdf_file = os.path.join(
        get_package_share_directory("arm_moveit_config"), "config", "arm.srdf"
    )
    with open(srdf_file, "r", encoding="utf-8") as f:
        robot_description_semantic_content = f.read()

    kinematics_file = os.path.join(
        get_package_share_directory("arm_moveit_config"), "config", "kinematics.yaml"
    )
    with open(kinematics_file, "r", encoding="utf-8") as f:
        robot_description_kinematics_content = yaml.safe_load(f)

    # joint_limits.yaml MUST also reach pick_place_node, not just move_group:
    # the node runs TimeOptimalTrajectoryGeneration on its OWN RobotModel for
    # Cartesian lift/descend segments, and URDF carries no acceleration limits.
    # Without robot_description_planning here TOTG fails ("No acceleration
    # limit was defined for joint joint_1") right after a successful grasp.
    joint_limits_file = os.path.join(
        get_package_share_directory("arm_moveit_config"), "config", "joint_limits.yaml"
    )
    with open(joint_limits_file, "r", encoding="utf-8") as f:
        robot_description_planning_content = yaml.safe_load(f)

    moveit_params = {
        "robot_description": robot_description_content,
        "robot_description_semantic": robot_description_semantic_content,
        "robot_description_kinematics": robot_description_kinematics_content,
        "robot_description_planning": robot_description_planning_content,
        "use_sim_time": use_sim_time,
    }

    # ── 1. Gazebo (includes RSP, ros2_control spawners internally) ────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("arm_gazebo"), "launch", "gazebo.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time, "world": world}.items(),
    )

    # ── 2. MoveIt2 move_group ─────────────────────────────────────────────────
    moveit_launch = TimerAction(
        period=7.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([arm_moveit_config_share, "launch", "move_group.launch.py"])
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            )
        ],
    )

    # ── 3. pick_place_node ────────────────────────────────────────────────────
    pick_place_node = TimerAction(
        period=20.0,
        actions=[
            Node(
                package="arm_nodes",
                executable="pick_place_node",
                output="screen",
                parameters=[moveit_params],
            )
        ],
    )

    # ── 4. RViz (optional) ────────────────────────────────────────────────────
    rviz_config = PathJoinSubstitution([arm_bringup_share, "config", "arm.rviz"])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[moveit_params],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            use_rviz_arg,
            use_sim_time_arg,
            world_arg,
            gazebo_launch,
            moveit_launch,
            pick_place_node,
            rviz_node,
        ]
    )
