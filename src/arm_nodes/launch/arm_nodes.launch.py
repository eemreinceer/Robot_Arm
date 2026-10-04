from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    planning_group = LaunchConfiguration('planning_group')
    gripper_action_name = LaunchConfiguration('gripper_action_name')
    motion_timeout_seconds = LaunchConfiguration('motion_timeout_seconds')

    return LaunchDescription([
        DeclareLaunchArgument('planning_group', default_value='arm'),
        DeclareLaunchArgument('gripper_action_name', default_value='/gripper_controller/gripper_action'),
        DeclareLaunchArgument('motion_timeout_seconds', default_value='30'),
        Node(
            package='arm_nodes',
            executable='pick_place_node',
            output='screen',
            parameters=[{
                'planning_group': planning_group,
                'gripper_action_name': gripper_action_name,
                'motion_timeout_seconds': motion_timeout_seconds,
            }],
        ),
    ])
