"""
Simulation-only launch: Gazebo + robot_state_publisher + joint_state_broadcaster.
Used for quick visual testing without MoveIt2 or application nodes.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── arguments ────────────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ── robot description ─────────────────────────────────────────────────────
    arm_description_share = FindPackageShare("arm_description")

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([arm_description_share, "urdf", "arm.urdf.xacro"]),
            " use_sim:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # ── 1. Gazebo ─────────────────────────────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("arm_gazebo"), "launch", "gazebo.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # ── 2. robot_state_publisher ──────────────────────────────────────────────
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # ── 3. joint_state_broadcaster ────────────────────────────────────────────
    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            gazebo_launch,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
        ]
    )
