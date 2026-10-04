"""Launch the vision encoder with one canonical board-geometry config."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare('arm_perception'), 'config', 'vision_encoder.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description=(
                'Vision encoder parameters. Board geometry is canonical here; '
                'do not duplicate board_cols/rows/square_size in launch args.')),
        Node(
            package='arm_perception',
            executable='vision_encoder_node',
            name='vision_encoder_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')]),
    ])
