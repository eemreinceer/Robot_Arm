import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def load_file(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, 'r', encoding='utf-8') as file:
        return file.read()


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, 'r', encoding='utf-8') as file:
        return yaml.safe_load(file)


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('arm_description'),
            'urdf',
            'arm.urdf.xacro',
        ]),
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    robot_description_semantic = {
        'robot_description_semantic': load_file('arm_moveit_config', 'config/arm.srdf')
    }

    robot_description_kinematics = {
        'robot_description_kinematics': load_yaml('arm_moveit_config', 'config/kinematics.yaml')
    }

    joint_limits = {
        'robot_description_planning': load_yaml('arm_moveit_config', 'config/joint_limits.yaml')
    }

    ompl_planning = load_yaml('arm_moveit_config', 'config/ompl_planning.yaml')
    moveit_controllers = load_yaml('arm_moveit_config', 'config/moveit_controllers.yaml')
    ompl_seed_preload = os.path.join(
        get_package_prefix('arm_nodes'), 'lib', 'libompl_seed_preload.so')

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        # Deterministic planning: LD_PRELOAD the ompl_seed_preload library so its
        # constructor pins the OMPL RNG seed BEFORE move_group creates any
        # generator. This is the only place the seed can be fixed effectively —
        # a planning-request adapter runs too late (OMPL rejects a late reseed).
        # Use the absolute installed path instead of relying on LD_LIBRARY_PATH;
        # if the preload cannot be found, move_group would silently fall back to
        # clock-seeded OMPL sampling.
        # Override the seed with the OMPL_SEED env var.
        additional_env={'LD_PRELOAD': ompl_seed_preload},
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            ompl_planning,
            moveit_controllers,
            {'use_sim_time': use_sim_time},
            # Slow-RTF sim robustness: the arm settles slowly, so the default
            # 0.01 rad start-state check aborts back-to-back motions ("start
            # point deviates from current robot state") and cascades into wild
            # retries. Relax the start tolerance and disable wall-clock-based
            # execution-duration monitoring (a normal sim trajectory plays out
            # in ~10x wall time at low real-time factor).
            #
            # 0.5 -> 0.3 (2026-06-19): keep this below the old broad mask, but above
            # the small residual lag seen in slow Gazebo after the controller clamps
            # per-cycle commands. Larger mismatches are still caught by pick/home
            # physical guards.
            {
                'trajectory_execution.allowed_start_tolerance': 0.3,
                'trajectory_execution.execution_duration_monitoring': False,
                'trajectory_execution.allowed_execution_duration_scaling': 5.0,
                'trajectory_execution.allowed_goal_duration_margin': 30.0,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        move_group_node,
    ])
